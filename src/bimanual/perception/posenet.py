"""PoseNet -- a small CNN that maps a camera image to 3D object positions.

This is M10 (PLAN.md "M10 -- Vision perception model (PoseNet) and its
OpenVINO path"). ADR-009 (ARCHITECTURE.md) explains *why* this model exists:
with the ACT learned-policy branch cut from the critical path (ADR-023), this
is the only trained model in the submission, so it is what makes the
20-point OpenVINO/Core-Ultra-optimization rubric line real rather than a
benchmark of a model nothing uses.

Backbone topology, deliberately NOT reinvented
------------------------------------------------
The convolutional backbone below is the exact same hand-rolled ResNet18-scale
encoder proved to convert cleanly through OpenVINO on CPU/GPU/NPU in M03
(`scripts/ov_smoke.py::build_model`'s `ResNet18Scale`, see that file's
docstring for the deviation-from-torchvision rationale: torchvision is not a
pinned dependency anywhere in this repo, so the topology is written directly
in plain `torch.nn` instead of imported). PLAN.md M03's "Dividend to protect"
note is explicit that M10 should reuse this exact backbone shape so M13's
OpenVINO export recipe transfers directly -- so this file re-derives the
identical block layout rather than inventing a new one. The only new part is
the *head* on top (a global-average-pool followed by a small MLP that
regresses 3D positions instead of a generic 512-d embedding).

For a reader new to robotics/ML
--------------------------------
- A convolutional neural network (CNN) turns an image into a small vector of
  numbers ("features") by repeatedly sliding small filters over it. Each
  filter learns to react to a visual pattern (an edge, a corner, eventually a
  whole object).
- "ResNet" (Residual Network) is a specific, very common CNN design. Its key
  trick is the "skip connection": each block's output is `f(x) + x` rather
  than just `f(x)`. This means a block only has to learn a *correction* to
  its input, which turns out to make much deeper networks trainable.
- "BasicBlock" is ResNet's smallest repeating unit: two 3x3 convolutions plus
  the skip connection described above.
- "Global average pool" collapses a (channels, height, width) feature map
  down to (channels,) by averaging every pixel in each channel. This throws
  away exactly *where* in the image a feature fired and keeps only *whether*
  it fired -- appropriate here because we are about to regress a fixed-size
  vector of object coordinates, not a per-pixel map.
- "Regression" (as opposed to "classification") means the network outputs
  continuous numbers (here: metres of x/y/z position) rather than a
  probability over a fixed set of labels.
"""

import torch
import torch.nn as nn

# ADR-041 / dataset_meta.json's "target_props": the three props this model is
# trained and evaluated to localize. Order matters -- it is the SAME order
# used to build the (9,) label vector in PoseNetDataset and to flatten this
# model's (9,) output back into per-prop (x, y, z) triples. Two other props
# (plate, spoon) are present in every rendered scene as visual/occlusion
# clutter but are deliberately NOT labelled or predicted (ADR-041).
PROP_ORDER = ["fork", "water_bottle", "mug"]

# Each prop contributes 3 numbers (x, y, z in metres, world frame).
COORDS_PER_PROP = 3
OUTPUT_DIM = len(PROP_ORDER) * COORDS_PER_PROP  # 9

# The dataset images are 224x224 (dataset_meta.json "resolution": [224, 224]),
# the standard input size this ResNet18-scale stem/pooling stack is built for.
INPUT_HW = 224


class BasicBlock(nn.Module):
    """One ResNet "BasicBlock": conv -> BN -> ReLU -> conv -> BN, plus a skip
    connection back to the block's input, then a final ReLU.

    `downsample`, when given, is a small conv+BN applied to the block's
    *input* so its shape (channel count and spatial size) matches the main
    branch's output before the two are added -- needed whenever this block
    changes the number of channels or uses stride > 1 to halve the spatial
    resolution.
    """

    def __init__(self, in_planes: int, planes: int, stride: int = 1, downsample=None):
        super().__init__()
        self.conv1 = nn.Conv2d(in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.BatchNorm2d(planes)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3, stride=1, padding=1, bias=False)
        self.bn2 = nn.BatchNorm2d(planes)
        self.downsample = downsample

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = x
        out = self.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        if self.downsample is not None:
            identity = self.downsample(x)
        out = out + identity  # the skip connection
        return self.relu(out)


class ResNet18ScaleBackbone(nn.Module):
    """The convolutional trunk only -- stem + 4 stages of 2 BasicBlocks each
    (channels 64/128/256/512), stride-2 at the start of stages 2-4. Ends at
    the (B, 512, 7, 7) feature map for a 224x224 input; does NOT include the
    global-average-pool or any fully-connected head -- those belong to the
    task-specific head below (PoseNet), following the same "backbone vs head"
    split `scripts/ov_smoke.py` uses (its `ResNet18Scale.fc` is a generic
    512-d embedding head; here the head instead regresses 9 coordinates).

    Verified same topology as `scripts/ov_smoke.py::build_model`'s
    `ResNet18Scale` up to (and not including) its `avgpool`/`fc`.
    """

    def __init__(self):
        super().__init__()
        self.in_planes = 64
        self.stem = nn.Sequential(
            nn.Conv2d(3, 64, kernel_size=7, stride=2, padding=3, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
        )
        self.layer1 = self._make_layer(64, blocks=2, stride=1)
        self.layer2 = self._make_layer(128, blocks=2, stride=2)
        self.layer3 = self._make_layer(256, blocks=2, stride=2)
        self.layer4 = self._make_layer(512, blocks=2, stride=2)

    def _make_layer(self, planes: int, blocks: int, stride: int) -> nn.Sequential:
        downsample = None
        if stride != 1 or self.in_planes != planes:
            downsample = nn.Sequential(
                nn.Conv2d(self.in_planes, planes, kernel_size=1, stride=stride, bias=False),
                nn.BatchNorm2d(planes),
            )
        layers = [BasicBlock(self.in_planes, planes, stride, downsample)]
        self.in_planes = planes
        for _ in range(1, blocks):
            layers.append(BasicBlock(self.in_planes, planes))
        return nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.stem(x)
        x = self.layer1(x)
        x = self.layer2(x)
        x = self.layer3(x)
        x = self.layer4(x)  # (B, 512, 7, 7) for a 224x224 input
        return x


class PoseNet(nn.Module):
    """Image -> 3 object positions.

    Input:  (B, 3, 224, 224) float tensor, RGB, normalized to [0, 1]
            (NOT ImageNet-mean/std normalized -- this is trained from
            scratch on simulator renders, not fine-tuned from an ImageNet
            checkpoint, so there is no pretrained-normalization convention to
            match).
    Output: (B, 9) float tensor. Reshape to (B, 3, 3) to get, for each of
            PROP_ORDER = ["fork", "water_bottle", "mug"], its predicted
            (x, y, z) position in metres, world frame.

    Head: global-average-pool (B,512,7,7) -> (B,512), then
          Linear(512, 256) -> ReLU -> Linear(256, 9).
    """

    def __init__(self):
        super().__init__()
        self.backbone = ResNet18ScaleBackbone()
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.head = nn.Sequential(
            nn.Linear(512, 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, OUTPUT_DIM),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        features = self.backbone(x)          # (B, 512, 7, 7)
        pooled = self.avgpool(features)       # (B, 512, 1, 1)
        pooled = torch.flatten(pooled, 1)     # (B, 512)
        return self.head(pooled)              # (B, 9)


def count_parameters(model: nn.Module) -> int:
    """Total trainable-parameter count, used to report the "~11-12M expected"
    figure from PLAN.md M10 / this module's task brief.
    """
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Smoke test: instantiate, print parameter count, forward-pass a dummy
    # batch, print the output shape. This is exactly the check the module's
    # "SMOKE TEST ONLY" requirement asks for -- no training happens here.
    torch.manual_seed(0)
    model = PoseNet()
    model.eval()

    n_params = count_parameters(model)
    print(f"PoseNet parameter count: {n_params:,}")

    dummy = torch.rand(4, 3, INPUT_HW, INPUT_HW)  # (B=4, C=3, H=224, W=224), values in [0, 1]
    with torch.no_grad():
        out = model(dummy)
    print(f"Input shape:  {tuple(dummy.shape)}")
    print(f"Output shape: {tuple(out.shape)}")
    assert out.shape == (4, OUTPUT_DIM), f"expected (4, {OUTPUT_DIM}), got {tuple(out.shape)}"
    print("Forward pass OK.")
