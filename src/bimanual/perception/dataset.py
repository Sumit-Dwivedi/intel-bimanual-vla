"""PoseNetDataset -- a torch Dataset over M09a's rendered frames + labels.

Reads the dataset produced by `scripts/generate_posenet_data.py` (M10 Phase 1
/ Phase 1.5, ADR-039/ADR-040/ADR-041): 5,000 (image, label) pairs under
`data/posenet/images/sample_NNNNN.png` and `data/posenet/labels/sample_NNNNN.json`.
`data/posenet/dataset_meta.json` (committed, unlike the bulk images/labels
which are gitignored -- see `.gitignore`'s "M10 Phase 1" block) records the
generation-time schema this file assumes; read it once before trusting this
docstring if the two ever disagree.

Label schema (verified against `scripts/generate_posenet_data.py`'s actual
`json.dumps(label, ...)` call, not guessed):
    {
      "sample_id": "sample_00000",
      "sample_index": 0,
      "target_props": ["fork", "water_bottle", "mug"],   # == PROP_ORDER
      "output_shape": [3, 3],
      "positions_xyz_m": [[x,y,z], [x,y,z], [x,y,z]],     # same order as target_props
      "objects": {
        "fork":         {"xyz_m": [x,y,z], "visibility_ratio": 0.0-1.0, ...},
        "water_bottle": {"xyz_m": [x,y,z], "visibility_ratio": 0.0-1.0, ...},
        "mug":          {"xyz_m": [x,y,z], "visibility_ratio": 0.0-1.0, ...}
      },
      "decoration_props_in_scene_unlabelled": ["plate", "spoon"],  # IGNORED --
          these two props are visible in the image for realism/occlusion but
          were never given ground-truth positions and must not be predicted.
      ... (other bookkeeping fields: env_reset_seed, rng_seed, camera,
           resolution, image_path, placement_draws, placement_order_used,
           visibility_resamples -- all ignored here, not part of the
           supervised-learning input/target.)
    }

Why PIL, here specifically
---------------------------
`scripts/generate_posenet_data.py` hand-rolls its own `write_png` because
Pillow is absent from BOTH the bm-ptl `ov_env` (which runs the simulator) and
the laptop dev env (see that script's own module docstring). This module is
different: it lives in a NEW, SEPARATE training venv
(`C:\\Users\\devcloud\\project\\train_env` on bm-ptl,
`scripts/requirements-train.txt`) created specifically for M10 Phase 2, which
DOES install Pillow (it needs an image loader and re-deriving a hand-rolled
PNG *reader* -- inverse of `write_png` -- would be strictly more code for no
benefit once Pillow is available). PoseNetDataset therefore uses
`PIL.Image.open` rather than a hand-rolled decoder. The PNGs themselves are
plain, standard 8-bit RGB (no PNG extensions Pillow would choke on) -- see
`generate_posenet_data.py::write_png`'s IHDR color-type=2 (truecolor).

Deterministic split
--------------------
`sample_index % 10 == 0` -> validation (500 samples: indices 0, 10, 20, ...,
4990). Everything else -> train (4500 samples). `sample_index` is parsed
directly from the filename ("sample_00000" -> 0), which is cheap (no need to
open every label file just to decide a split) and is cross-checked against
the label JSON's own "sample_index" field the first time each sample is
actually loaded, so a mismatch (e.g. a corrupted/renamed file) fails loudly
rather than silently mis-splitting one sample.
"""

import json
from pathlib import Path

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

from bimanual.perception.posenet import PROP_ORDER, INPUT_HW

DEFAULT_ROOT = Path(__file__).resolve().parents[3] / "data" / "posenet"


def _parse_sample_index(stem: str) -> int:
    """"sample_00013" -> 13. Raises ValueError loudly on anything that does
    not match the generator's fixed `sample_{i:05d}` naming, rather than
    silently skipping or mis-sorting a malformed filename.
    """
    prefix = "sample_"
    if not stem.startswith(prefix):
        raise ValueError(f"unexpected label filename stem {stem!r}, expected 'sample_NNNNN'")
    return int(stem[len(prefix):])


class PoseNetDataset(Dataset):
    """One item = (image, labels, visibility).

    image:      (3, 224, 224) float32 tensor, RGB, values in [0, 1].
    labels:     (9,) float32 tensor -- PROP_ORDER's 3 props x (x, y, z)
                metres, flattened in PROP_ORDER order (fork, water_bottle,
                mug), matching `PoseNet`'s (B, 9) output layout exactly.
    visibility: (3,) float32 tensor -- one occlusion-ratio-based visibility
                score per prop (same order), in [0, 1]. Consumed by
                `scripts/train_posenet.py`'s visibility-weighted loss so a
                heavily-occluded prop's noisy/ambiguous label contributes
                less to the gradient than a clearly-visible one.
    """

    def __init__(self, root: Path = DEFAULT_ROOT, split: str = "train"):
        if split not in ("train", "val"):
            raise ValueError(f"split must be 'train' or 'val', got {split!r}")
        self.root = Path(root)
        self.images_dir = self.root / "images"
        self.labels_dir = self.root / "labels"
        self.split = split

        if not self.labels_dir.exists():
            raise FileNotFoundError(
                f"{self.labels_dir} does not exist. The bulk images/labels are "
                f"gitignored (.gitignore's 'M10 Phase 1' block) and live only "
                f"on bm-ptl, where scripts/generate_posenet_data.py produced "
                f"them -- see data/posenet/dataset_meta.json for the "
                f"reproducibility record of that run."
            )

        all_label_paths = sorted(self.labels_dir.glob("sample_*.json"))
        if not all_label_paths:
            raise FileNotFoundError(f"no sample_*.json label files found under {self.labels_dir}")

        kept = []
        for p in all_label_paths:
            idx = _parse_sample_index(p.stem)
            is_val = (idx % 10 == 0)
            if (split == "val") == is_val:
                kept.append((idx, p))
        kept.sort(key=lambda t: t[0])
        self.items = kept  # list of (sample_index, label_path)

    def __len__(self) -> int:
        return len(self.items)

    def __getitem__(self, i: int):
        sample_index, label_path = self.items[i]
        label = json.loads(label_path.read_text())

        if label.get("sample_index") != sample_index:
            raise ValueError(
                f"{label_path}: filename implies sample_index={sample_index} "
                f"but the label JSON says sample_index={label.get('sample_index')!r} "
                f"-- refusing to silently mis-split this sample."
            )
        if label.get("target_props") != PROP_ORDER:
            raise ValueError(
                f"{label_path}: target_props={label.get('target_props')!r} does "
                f"not match PoseNet.PROP_ORDER={PROP_ORDER!r} -- the label "
                f"schema has drifted from what this dataset loader assumes."
            )

        image_path = self.root / label["image_path"]  # e.g. "images/sample_00000.png"
        img = Image.open(image_path).convert("RGB")
        if img.size != (INPUT_HW, INPUT_HW):
            raise ValueError(f"{image_path}: expected {INPUT_HW}x{INPUT_HW}, got {img.size}")
        img_arr = np.asarray(img, dtype=np.float32) / 255.0  # (H, W, 3) in [0, 1]
        img_chw = np.transpose(img_arr, (2, 0, 1))            # (3, H, W)
        image = torch.from_numpy(img_chw.copy())

        objects = label["objects"]
        xyz_flat = []
        visibility = []
        for prop_name in PROP_ORDER:
            obj = objects[prop_name]
            xyz_flat.extend(obj["xyz_m"])
            visibility.append(obj["visibility_ratio"])

        labels_t = torch.tensor(xyz_flat, dtype=torch.float32)       # (9,)
        visibility_t = torch.tensor(visibility, dtype=torch.float32)  # (3,)

        return image, labels_t, visibility_t


if __name__ == "__main__":
    # Smoke test: load one train sample and one val sample, print shapes and
    # plausible-value checks. Run this on bm-ptl (train_env) where the real
    # data/posenet/images + labels actually exist.
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()

    train_ds = PoseNetDataset(args.root, split="train")
    val_ds = PoseNetDataset(args.root, split="val")
    print(f"train split: {len(train_ds)} samples")
    print(f"val split:   {len(val_ds)} samples")
    assert len(train_ds) + len(val_ds) == len(list((args.root / 'labels').glob('sample_*.json')))

    image, labels, visibility = train_ds[0]
    print(f"train[0] image shape={tuple(image.shape)} dtype={image.dtype} "
          f"min={image.min().item():.3f} max={image.max().item():.3f}")
    print(f"train[0] labels (xyz x 3 props, metres) = {labels.tolist()}")
    print(f"train[0] visibility (3 props)           = {visibility.tolist()}")

    image_v, labels_v, visibility_v = val_ds[0]
    print(f"val[0]   image shape={tuple(image_v.shape)} dtype={image_v.dtype}")
    print(f"val[0]   labels    = {labels_v.tolist()}")
    print(f"val[0]   visibility= {visibility_v.tolist()}")
    print("Dataset smoke test OK.")
