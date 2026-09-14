"""M10 Phase 2 -- PoseNet training script (PLAN.md M10, ARCHITECTURE.md ADR-009).

Trains `src/bimanual/perception/posenet.py::PoseNet` on
`src/bimanual/perception/dataset.py::PoseNetDataset` (M09a's 5,000 rendered
frames + privileged-simulator-state labels, ADR-041).

WHERE THIS RUNS (read before running it)
------------------------------------------
This script needs torch (+ideally an XPU-capable build) and Pillow. Per this
module's task brief, `ov_env` on bm-ptl has NEITHER -- it holds the
load-bearing `mujoco==3.2.7` + `openvino==2026.3.1` pairing four verified
skills depend on, and is NOT to be touched. This script instead runs in a
NEW, SEPARATE venv, `C:\\Users\\devcloud\\project\\train_env` on bm-ptl,
built from `scripts/requirements-train.txt`. It does not need `mujoco` and
does not import it.

DEVICE SELECTION
------------------
`--device` defaults to `xpu` (Intel Arc iGPU, native `torch.xpu` backend --
tried before the older IPEX path; see `docs/hardware/m10-phase2-smoke.md`
for which path actually worked on this hardware and why). If the requested
device is unavailable, this script falls back to CPU **loudly** (a printed
warning identifying exactly what was requested and what is being used
instead) rather than silently training somewhere the operator did not ask
for.

LOSS
-----
Visibility-weighted MSE: for each sample and each of the 3 props, the
squared Euclidean position error (||pred_xyz - target_xyz||^2, in the
prop's own (x, y, z) triple, i.e. NOT summed across props first) is
multiplied by that prop's `visibility_ratio` label (an occlusion-based
score in [0, 1], ADR-041) before averaging. A prop that is mostly occluded
in a given frame has a less trustworthy label (privileged simulator state
is still exact, but the *visual evidence* the network has to work with is
weak/absent), so this down-weights its contribution to the gradient rather
than either dropping it entirely or trusting it fully.

WINDOWS DATALOADER CAVEAT
---------------------------
`num_workers > 0` on Windows uses the `spawn` multiprocessing start method,
which re-imports this module in each worker process -- this is exactly why
the training entry point below is guarded by `if __name__ == "__main__":`.
Without that guard, spawn would recursively re-launch the whole script in
every worker. Default is `--num-workers 0`: a corrected, guarded throughput
probe on this Windows host measured 104.7 samples/sec at 0 workers versus
53.3 samples/sec at 4 workers over a 10-batch/320-sample window (ADR-042) --
spawn's one-time per-process startup cost outweighs the parallel-loading
gain at this dataset size, so more workers is NOT faster here. Raise
`--num-workers` explicitly (e.g. `--num-workers 4`) only if you re-measure
and confirm it helps on the machine you are running on.
"""

import argparse
import time
from pathlib import Path

import torch
import torch.nn as nn
from torch.optim import Adam
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader

from bimanual.perception.posenet import PoseNet, PROP_ORDER, count_parameters
from bimanual.perception.dataset import PoseNetDataset, DEFAULT_ROOT

# Keys match this module's task brief EXACTLY: "fork_mae", "bottle_mae",
# "mug_mae" -- note "bottle_mae", not "water_bottle_mae", even though
# PROP_ORDER's actual prop name is "water_bottle". This mapping is the one
# place that naming difference is bridged.
MAE_REPORT_KEYS = {"fork": "fork_mae", "water_bottle": "bottle_mae", "mug": "mug_mae"}

DEFAULT_CHECKPOINT_DIR = Path(__file__).resolve().parent.parent / "checkpoints"


def resolve_device(requested: str) -> torch.device:
    """Resolve `--device` to an actual, usable torch.device, with a loud
    (printed, not silent) fallback to CPU when the request cannot be
    honoured. Never raises for an unavailable accelerator -- that would
    make this script unusable on a machine mid-way through the device
    investigation this module exists to produce.
    """
    if requested == "cpu":
        return torch.device("cpu")

    if requested == "cuda":
        if torch.cuda.is_available():
            return torch.device("cuda")
        print("WARNING: --device cuda requested but torch.cuda.is_available() "
              "is False. Falling back to cpu.")
        return torch.device("cpu")

    if requested == "xpu":
        has_xpu_attr = hasattr(torch, "xpu")
        xpu_available = False
        if has_xpu_attr:
            try:
                xpu_available = bool(torch.xpu.is_available())
            except Exception as e:  # noqa: BLE001 -- report verbatim, do not hide
                print(f"WARNING: torch.xpu.is_available() raised "
                      f"{type(e).__name__}: {e}. Falling back to cpu.")
                return torch.device("cpu")
        if xpu_available:
            return torch.device("xpu")
        print(f"WARNING: --device xpu requested but unavailable "
              f"(hasattr(torch, 'xpu')={has_xpu_attr}, "
              f"torch.xpu.is_available()={xpu_available if has_xpu_attr else 'n/a'}). "
              f"Falling back to cpu.")
        return torch.device("cpu")

    raise ValueError(f"unknown device {requested!r}")


def visibility_weighted_mse(pred: torch.Tensor, target: torch.Tensor,
                              visibility: torch.Tensor) -> torch.Tensor:
    """mean(visibility * ||pred - target||^2), computed PER PROP (not per
    scalar coordinate): for each of the 3 props, the squared Euclidean
    distance between its predicted and true (x, y, z) is one term, weighted
    by that prop's visibility, then averaged over (batch x props).

    pred, target: (B, 9). visibility: (B, 3).
    """
    b = pred.shape[0]
    n_props = len(PROP_ORDER)
    pred_r = pred.view(b, n_props, 3)
    target_r = target.view(b, n_props, 3)
    sq_err = ((pred_r - target_r) ** 2).sum(dim=-1)  # (B, n_props) squared L2 per prop
    weighted = visibility * sq_err                    # (B, n_props)
    return weighted.mean()


def train_one_epoch(model: nn.Module, loader: DataLoader, optimizer, device) -> float:
    model.train()
    total_loss = 0.0
    n_samples = 0
    for images, labels, visibility in loader:
        images = images.to(device)
        labels = labels.to(device)
        visibility = visibility.to(device)

        optimizer.zero_grad()
        preds = model(images)
        loss = visibility_weighted_mse(preds, labels, visibility)
        loss.backward()
        optimizer.step()

        batch_n = images.shape[0]
        total_loss += loss.item() * batch_n
        n_samples += batch_n
    return total_loss / n_samples


@torch.no_grad()
def evaluate(model: nn.Module, loader: DataLoader, device) -> tuple[float, dict]:
    """Returns (val_loss, per_prop_mae_dict). `per_prop_mae` is the mean
    Euclidean position error in metres per prop -- i.e. mean(||pred_xyz -
    target_xyz||), the full 3D distance, not a component-wise |pred-target|
    average -- because "how far off was the predicted position, in metres"
    is the physically meaningful number for a position-regression model,
    and it is what a reader comparing this to the visibility-weighted
    SQUARED loss above should expect ("mae" here = mean absolute
    *positional* error, not mean absolute *coordinate* error).
    """
    model.eval()
    n_props = len(PROP_ORDER)
    total_loss = 0.0
    n_samples = 0
    per_prop_err_sum = torch.zeros(n_props)

    for images, labels, visibility in loader:
        images = images.to(device)
        labels = labels.to(device)
        visibility = visibility.to(device)

        preds = model(images)
        loss = visibility_weighted_mse(preds, labels, visibility)
        batch_n = images.shape[0]
        total_loss += loss.item() * batch_n

        pred_r = preds.view(batch_n, n_props, 3)
        target_r = labels.view(batch_n, n_props, 3)
        dist = torch.linalg.norm(pred_r - target_r, dim=-1)  # (B, n_props) metres
        per_prop_err_sum += dist.sum(dim=0).cpu()
        n_samples += batch_n

    val_loss = total_loss / n_samples
    per_prop_mae = (per_prop_err_sum / n_samples).tolist()
    mae_dict = {MAE_REPORT_KEYS[name]: mae for name, mae in zip(PROP_ORDER, per_prop_mae)}
    return val_loss, mae_dict


def run_smoke_test(model: nn.Module, loader: DataLoader, optimizer, device) -> None:
    """Run exactly ONE training batch, report a loss, exit. This is the
    "SMOKE TEST ONLY -- NO FULL TRAINING" mode this module's task brief
    requires; it proves the model, dataset and optimizer wire together and
    actually execute on `device`, without spending any real training wall
    clock.
    """
    model.train()
    t0 = time.time()
    images, labels, visibility = next(iter(loader))
    images = images.to(device)
    labels = labels.to(device)
    visibility = visibility.to(device)

    optimizer.zero_grad()
    preds = model(images)
    loss = visibility_weighted_mse(preds, labels, visibility)
    loss.backward()
    optimizer.step()
    dt = time.time() - t0

    print(f"[SMOKE TEST] device={device} batch_size={images.shape[0]} "
          f"output_shape={tuple(preds.shape)} loss={loss.item():.6f} "
          f"wall_clock={dt:.2f}s")
    print("[SMOKE TEST] one batch trained and one optimizer step taken. Exiting cleanly.")


def parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data-root", type=Path, default=DEFAULT_ROOT,
                    help="Directory containing images/ and labels/ (default: data/posenet).")
    p.add_argument("--checkpoint-dir", type=Path, default=DEFAULT_CHECKPOINT_DIR,
                    help="Where to write posenet_best.pth / posenet_final.pth.")
    p.add_argument("--epochs", type=int, default=30)
    p.add_argument("--batch-size", type=int, default=32)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--weight-decay", type=float, default=1e-5)
    p.add_argument("--num-workers", type=int, default=0,
                    help="DataLoader worker processes. Default 0: measured "
                         "104.7 samples/sec at 0 workers vs 53.3 at 4 on this "
                         "Windows host (spawn overhead) -- see this file's "
                         "module docstring / ADR-042. Override explicitly "
                         "if you have re-measured on your own machine.")
    p.add_argument("--device", choices=["cpu", "xpu", "cuda"], default="xpu",
                    help="Preferred device; falls back to cpu automatically "
                         "and loudly if unavailable.")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--smoke-test", action="store_true",
                    help="Run exactly one training batch and exit. Does NOT "
                         "run full training. This is the mode this module "
                         "was smoke-tested with.")
    return p.parse_args()


def main():
    args = parse_args()
    torch.manual_seed(args.seed)

    device = resolve_device(args.device)
    print(f"Using device: {device}")

    train_ds = PoseNetDataset(args.data_root, split="train")
    val_ds = PoseNetDataset(args.data_root, split="val")
    print(f"train samples: {len(train_ds)}, val samples: {len(val_ds)}")

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,
                                num_workers=args.num_workers, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False,
                              num_workers=args.num_workers)

    model = PoseNet().to(device)
    print(f"PoseNet parameter count: {count_parameters(model):,}")

    optimizer = Adam(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    if args.smoke_test:
        run_smoke_test(model, train_loader, optimizer, device)
        return

    scheduler = CosineAnnealingLR(optimizer, T_max=args.epochs)

    args.checkpoint_dir.mkdir(parents=True, exist_ok=True)
    best_val_loss = float("inf")
    best_mae = None
    total_samples_seen = 0
    t_start = time.time()

    for epoch in range(1, args.epochs + 1):
        t_epoch0 = time.time()
        train_loss = train_one_epoch(model, train_loader, optimizer, device)
        val_loss, mae_dict = evaluate(model, val_loader, device)
        scheduler.step()
        epoch_dt = time.time() - t_epoch0
        total_samples_seen += len(train_ds)

        print(f"epoch {epoch}/{args.epochs} train_loss={train_loss:.6f} "
              f"val_loss={val_loss:.6f} "
              f"fork_mae={mae_dict['fork_mae']:.4f} "
              f"bottle_mae={mae_dict['bottle_mae']:.4f} "
              f"mug_mae={mae_dict['mug_mae']:.4f} "
              f"({epoch_dt:.1f}s)")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_mae = mae_dict
            torch.save({
                "model_state_dict": model.state_dict(),
                "epoch": epoch,
                "val_loss": val_loss,
                "mae": mae_dict,
                "prop_order": PROP_ORDER,
            }, args.checkpoint_dir / "posenet_best.pth")

    torch.save({
        "model_state_dict": model.state_dict(),
        "epoch": args.epochs,
        "val_loss": val_loss,
        "mae": mae_dict,
        "prop_order": PROP_ORDER,
    }, args.checkpoint_dir / "posenet_final.pth")

    wall_clock = time.time() - t_start
    samples_per_sec = total_samples_seen / wall_clock if wall_clock > 0 else float("nan")

    print("\n==== TRAINING COMPLETE ====")
    print(f"Best val loss: {best_val_loss:.6f}")
    print(f"Best-epoch per-prop MAE (metres): {best_mae}")
    print(f"Wall clock: {wall_clock:.1f}s ({wall_clock / 60:.1f} min)")
    print(f"Throughput: {samples_per_sec:.2f} samples/sec")
    print(f"Checkpoints: {args.checkpoint_dir / 'posenet_best.pth'}, "
          f"{args.checkpoint_dir / 'posenet_final.pth'}")


if __name__ == "__main__":
    # Guard required for Windows DataLoader num_workers > 0 (spawn
    # re-imports this module in each worker process) -- see module docstring.
    main()
