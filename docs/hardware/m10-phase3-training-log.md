# M10 Phase 3 — PoseNet training log

**Run:** `python scripts/train_posenet.py --device xpu --epochs 30 --batch-size 32 --num-workers 0`
**Machine:** bm-ptl, `train_env` (torch 2.14.0+xpu), Intel Arc B390 iGPU (ADR-020, ADR-042).
**Dataset:** `d1157e1`'s 5000 samples, 4500 train / 500 val, deterministic `sample_index % 10` split.
**Model:** PoseNet, 11,310,153 params, ResNet18-scale backbone re-derived from `scripts/ov_smoke.py`.
**Loss:** visibility-weighted MSE. **Optimiser:** Adam lr 1e-3, wd 1e-5, cosine schedule.

## Headline

| metric | value |
|---|---|
| wall clock | **956.1 s (15.9 min)** |
| throughput | **141.20 samples/sec** |
| best val loss | **0.000011** (epoch 30) |
| fork MAE | **3.2 mm** |
| water_bottle MAE | **2.6 mm** |
| mug MAE | **2.8 mm** |

All three props land far inside the "< 0.02 m excellent" band. For scale: props
randomise over x,y in [-0.18, +0.18], so a model that learned nothing and
predicted the dataset mean would score roughly **90 mm**. These are ~30x better.

## Per-epoch curve

| epoch | train_loss | val_loss | fork_mae | bottle_mae | mug_mae |
|---:|---:|---:|---:|---:|---:|
| 1 | 0.019601 | 0.004372 | 0.0597 | 0.0881 | 0.0351 |
| 2 | 0.002538 | 0.001810 | 0.0576 | 0.0221 | 0.0366 |
| 3 | 0.001568 | 0.002075 | 0.0275 | 0.0470 | 0.0536 |
| 4 | 0.001445 | 0.001589 | 0.0434 | 0.0325 | 0.0407 |
| 5 | 0.001062 | 0.001095 | 0.0327 | 0.0317 | 0.0313 |
| 6 | 0.000960 | 0.000589 | 0.0223 | 0.0263 | 0.0194 |
| 7 | 0.000995 | 0.000610 | 0.0284 | 0.0215 | 0.0178 |
| 8 | 0.000880 | 0.001329 | 0.0326 | 0.0288 | 0.0394 |
| 9 | 0.000705 | 0.001060 | 0.0310 | 0.0288 | 0.0320 |
| 10 | 0.000627 | 0.001960 | 0.0513 | 0.0345 | 0.0432 |
| 11 | 0.000653 | 0.000941 | 0.0206 | 0.0291 | 0.0364 |
| 12 | 0.000593 | 0.000662 | 0.0205 | 0.0227 | 0.0299 |
| 13 | 0.000515 | 0.000635 | 0.0179 | 0.0328 | 0.0156 |
| 14 | 0.000409 | 0.000271 | 0.0142 | 0.0188 | 0.0128 |
| 15 | 0.000354 | 0.000670 | 0.0289 | 0.0211 | 0.0244 |
| 16 | 0.000346 | 0.000148 | 0.0118 | 0.0121 | 0.0097 |
| 17 | 0.000249 | 0.000580 | 0.0165 | 0.0329 | 0.0161 |
| 18 | 0.000264 | 0.000233 | 0.0153 | 0.0143 | 0.0140 |
| 19 | 0.000221 | 0.000292 | 0.0197 | 0.0170 | 0.0114 |
| 20 | 0.000182 | 0.000271 | 0.0227 | 0.0130 | 0.0086 |
| 21 | 0.000154 | 0.000265 | 0.0209 | 0.0162 | 0.0077 |
| 22 | 0.000118 | 0.000147 | 0.0104 | 0.0126 | 0.0104 |
| 23 | 0.000088 | 0.000085 | 0.0093 | 0.0082 | 0.0087 |
| 24 | 0.000060 | 0.000050 | 0.0077 | 0.0053 | 0.0062 |
| 25 | 0.000049 | 0.000055 | 0.0088 | 0.0068 | 0.0045 |
| 26 | 0.000038 | 0.000039 | 0.0065 | 0.0044 | 0.0059 |
| 27 | 0.000030 | 0.000030 | 0.0065 | 0.0038 | 0.0037 |
| 28 | 0.000024 | 0.000022 | 0.0056 | 0.0039 | 0.0030 |
| 29 | 0.000020 | 0.000015 | 0.0039 | 0.0034 | 0.0030 |
| 30 | 0.000017 | 0.000011 | 0.0032 | 0.0026 | 0.0028 |

## Curve shape

Three regimes. **Epochs 1-2** drop steeply (train 0.0196 -> 0.0025) as the model
leaves initialisation. **Epochs 3-15 oscillate**: val_loss rises and falls
repeatedly (0.00159, 0.00110, 0.00059, 0.00133, 0.00106, 0.00196, ...) while
train_loss falls steadily. That is the cosine schedule still at a high learning
rate, not overfitting -- train and val move together on the downswings.
**Epochs 16-30 descend monotonically** to 0.000011 as the cosine schedule anneals.

**The best epoch is the last one (30), and val_loss was still falling when
training stopped.** The run was not allowed to converge to a plateau, so more
epochs would likely improve these numbers further. Train and val loss end
essentially equal (0.000017 vs 0.000011), with val *below* train -- no overfitting
signal anywhere in the run, which is unsurprising given 4500 synthetic samples
from a single fixed camera with no augmentation.

## Mean-collapse check

A low loss alone does not prove a regressor learned anything -- a model that
outputs the dataset mean for every input can look plausible. Tested directly on
5 validation samples with the best checkpoint:

```
std of PREDICTIONS across 5 samples: [0.1533 0.0672 0.0017 0.0472 0.0981 0.0021 0.0549 0.1108 0.0008]
std of GROUND TRUTH  across 5 samples: [0.1537 0.0673 0.      0.0469 0.0978 0.      0.0558 0.1098 0.     ]
pred_spread / gt_spread = 1.009
```

The predictions vary as much as the ground truth does (ratio 1.009; near 0 would
mean collapse). **Genuinely input-dependent.**

Note the z columns (indices 2, 5, 8): ground-truth std is exactly **0**, because
each prop's z is pinned to its own resting height by design (ADR-039 correction --
plate 0.355, fork/spoon 0.356, mug 0.39, bottle 0.44). The model predicts those
near-constants to within 1-3 mm. Only x and y carry real signal, so the effective
task is 2-D localisation per prop, and the MAE figures should be read that way.

## Per-sample errors, best checkpoint

| sample | prop | predicted (x, y, z) | ground truth (x, y, z) | abs error (mm) |
|---:|---|---|---|---|
| 0 | fork | (+0.0969, +0.1526, 0.3563) | (+0.0946, +0.1515, 0.3560) | (2.4, 1.2, 0.3) |
| 0 | water_bottle | (+0.0468, +0.0362, 0.4399) | (+0.0484, +0.0347, 0.4400) | (1.5, 1.5, 0.1) |
| 0 | mug | (+0.1141, -0.0800, 0.3904) | (+0.1139, -0.0802, 0.3900) | (0.2, 0.2, 0.4) |
| 1 | fork | (+0.1642, -0.0025, 0.3571) | (+0.1652, -0.0023, 0.3560) | (1.1, 0.1, 1.1) |
| 1 | water_bottle | (-0.0413, +0.0497, 0.4382) | (-0.0384, +0.0502, 0.4400) | (3.0, 0.5, 1.8) |
| 1 | mug | (+0.0621, +0.1713, 0.3886) | (+0.0627, +0.1680, 0.3900) | (0.6, 3.3, 1.4) |
| 2 | fork | (-0.1802, -0.0463, 0.3549) | (-0.1782, -0.0496, 0.3560) | (1.9, 3.2, 1.1) |
| 2 | water_bottle | (-0.0654, -0.1195, 0.4435) | (-0.0650, -0.1202, 0.4400) | (0.4, 0.7, 3.5) |
| 2 | mug | (-0.0241, -0.0011, 0.3900) | (-0.0273, -0.0008, 0.3900) | (3.2, 0.3, 0.0) |
| 3 | fork | (+0.1389, +0.0056, 0.3532) | (+0.1409, +0.0105, 0.3560) | (2.0, 5.0, 2.8) |
| 3 | water_bottle | (-0.0783, -0.1745, 0.4393) | (-0.0761, -0.1741, 0.4400) | (2.3, 0.4, 0.7) |
| 3 | mug | (+0.1064, +0.1577, 0.3906) | (+0.1036, +0.1558, 0.3900) | (2.8, 1.9, 0.6) |
| 4 | fork | (-0.1727, +0.0390, 0.3579) | (-0.1754, +0.0419, 0.3560) | (2.7, 2.9, 1.9) |
| 4 | water_bottle | (+0.0102, -0.1663, 0.4430) | (+0.0103, -0.1657, 0.4400) | (0.1, 0.6, 3.0) |
| 4 | mug | (+0.1261, -0.0800, 0.3909) | (+0.1261, -0.0813, 0.3900) | (0.0, 1.4, 0.9) |

Worst single-axis error across all 15 prop-samples is **5.0 mm**; most are under
3 mm. All five samples had `visibility_ratio == 1.00` for every target prop, so
this slice does not exercise the occluded tail -- accuracy on heavily-occluded
samples (the 3.94% the filter rejected sit below 0.30, but accepted samples run
down to ~0.44) is not characterised here.

## Caveats worth carrying into Phase 4

1. **Synthetic-only, single camera, no augmentation.** Every sample comes from
   `posenet_cam` at one fixed pose in one lighting condition. These MAE figures
   describe in-distribution accuracy and should not be read as robustness.
2. **z is not really being predicted.** It is constant per prop by construction;
   only x and y are learned.
3. **Best epoch was the final epoch** -- the model had not plateaued, so the run
   is under-trained rather than over-trained.
4. **Arm pose is fixed.** Every image has both arms at the home keyframe. A scene
   with arms mid-motion is out of distribution.
