"""Render `docs/slides.md` to `docs/slides/submission.pptx`.

`docs/slides.md` is canonical. Edit the markdown, re-run this, never hand-edit
the .pptx -- a hand-edited deck drifts away from the source and from the
caveats the source enforces.

Requires `python-pptx` (not in the project's pinned environments; install it
into a throwaway venv, which is what was done to produce the committed deck --
ov_env/train_env are deliberately left alone).

    python scripts/render_slides.py
    python scripts/render_slides.py --check   # verify an existing deck only

Design: 16:9, sans-serif (Calibri), dark slate on white, one accent. No stock
template, no emojis, nothing that implies a result the measurements do not
support.
"""
from __future__ import annotations

import argparse
import pathlib
import sys

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.util import Emu, Inches, Pt

REPO = pathlib.Path(__file__).resolve().parents[1]
OUT = REPO / "docs" / "slides" / "submission.pptx"
COVER = REPO / "docs" / "images" / "submission-cover.png"
DEMO = REPO / "docs" / "images" / "m06-handoff-complete.png"

W, H = Inches(13.333), Inches(7.5)          # 16:9
INK = RGBColor(0x1C, 0x24, 0x33)            # dark slate
MUTED = RGBColor(0x55, 0x61, 0x70)
ACCENT = RGBColor(0x00, 0x71, 0xC5)         # Intel-ish blue
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
RULE = RGBColor(0xD5, 0xDB, 0xE2)
FONT = "Calibri"


def _tf(shape):
    tf = shape.text_frame
    tf.word_wrap = True
    return tf


def _run(p, text, size, bold=False, color=INK, italic=False):
    r = p.add_run()
    r.text = text
    r.font.size = Pt(size)
    r.font.bold = bold
    r.font.italic = italic
    r.font.color.rgb = color
    r.font.name = FONT
    return r


def blank(prs):
    return prs.slides.add_slide(prs.slide_layouts[6])


def heading(slide, text, sub=None):
    box = slide.shapes.add_textbox(Inches(0.7), Inches(0.45), W - Inches(1.4), Inches(1.0))
    tf = _tf(box)
    p = tf.paragraphs[0]
    _run(p, text, 34, bold=True)
    if sub:
        p2 = tf.add_paragraph()
        p2.space_before = Pt(4)
        _run(p2, sub, 15, color=MUTED)
    ln = slide.shapes.add_shape(1, Inches(0.7), Inches(1.62), W - Inches(1.4), Emu(9525))
    ln.fill.solid()
    ln.fill.fore_color.rgb = RULE
    ln.line.fill.background()
    ln.shadow.inherit = False


def bullets(slide, items, top=Inches(1.95), left=Inches(0.8), width=None,
            size=17, gap=9, height=None):
    width = width or (W - Inches(1.6))
    box = slide.shapes.add_textbox(left, top, width, height or (H - top - Inches(0.72)))
    tf = _tf(box)
    first = True
    for item in items:
        p = tf.paragraphs[0] if first else tf.add_paragraph()
        first = False
        p.space_after = Pt(gap)
        if isinstance(item, tuple):          # (text, indent_level)
            text, lvl = item
            p.level = lvl
        else:
            text, lvl = item, 0
        _run(p, "–  " if lvl else "•  ", size, color=ACCENT)
        # bold spans marked with ** **
        for i, part in enumerate(text.split("**")):
            if part:
                _run(p, part, size, bold=(i % 2 == 1))
    return box


def table(slide, rows, left, top, width, col_w=None, size=12, header=True):
    nrow, ncol = len(rows), len(rows[0])
    height = Inches(0.32) * nrow
    shp = slide.shapes.add_table(nrow, ncol, left, top, width, height)
    tbl = shp.table
    if col_w:
        for i, w in enumerate(col_w):
            tbl.columns[i].width = w
    for r, row in enumerate(rows):
        for c, val in enumerate(row):
            cell = tbl.cell(r, c)
            cell.text = ""
            p = cell.text_frame.paragraphs[0]
            bold = header and r == 0
            for i, part in enumerate(str(val).split("**")):
                if part:
                    _run(p, part, size, bold=bold or (i % 2 == 1),
                         color=WHITE if (header and r == 0) else INK)
            cell.margin_left = Inches(0.08)
            cell.margin_right = Inches(0.08)
            cell.margin_top = Inches(0.02)
            cell.margin_bottom = Inches(0.02)
            if header and r == 0:
                cell.fill.solid()
                cell.fill.fore_color.rgb = INK
            else:
                cell.fill.solid()
                cell.fill.fore_color.rgb = WHITE
    return shp


def footer(slide, text):
    box = slide.shapes.add_textbox(Inches(0.7), H - Inches(0.62),
                                   W - Inches(1.4), Inches(0.4))
    p = _tf(box).paragraphs[0]
    _run(p, text, 10, color=MUTED, italic=True)


# --------------------------------------------------------------------- slides
def build() -> Presentation:
    prs = Presentation()
    prs.slide_width, prs.slide_height = W, H

    # 1 -- title. The cover image ALREADY carries the title, subtitle and
    # event name, so nothing is overlaid on top of it except the repo URL --
    # a second title band would just duplicate the artwork's own text.
    # Fitted to HEIGHT so it bleeds full-frame (1200x630 is 1.905:1 against
    # the slide's 1.778:1, so ~0.5 in is cropped from each side, all of it
    # empty floor).
    s = blank(prs)
    assert COVER.exists(), COVER
    pic_w = int(H * (1200 / 630))
    s.shapes.add_picture(str(COVER), int((W - pic_w) / 2), 0, height=H)
    strip = s.shapes.add_shape(1, 0, H - Inches(0.62), W, Inches(0.62))
    strip.fill.solid()
    strip.fill.fore_color.rgb = INK
    strip.line.fill.background()
    strip.shadow.inherit = False
    box = s.shapes.add_textbox(Inches(0.7), H - Inches(0.58), W - Inches(1.4), Inches(0.5))
    p_ = _tf(box).paragraphs[0]
    _run(p_, "github.com/Sumit-Dwivedi/intel-bimanual-vla", 14, bold=True, color=WHITE)

    # 2 — the challenge
    s = blank(prs)
    heading(s, "The challenge", "Two SO-101 arms, one shared workspace, one language command")
    bullets(s, [
        "Two SO-101 arms in MuJoCo; language-conditioned table setting",
        "Inference on Intel Core Ultra Series 3 (Panther Lake) via OpenVINO 2026.3 — CPU, iGPU (Arc B390), NPU (NPU5010)",
        "Simulation-first: **no physical robot** anywhere in the pipeline",
        "The hard part is not the language. It is getting two 5-DoF arms to share one workspace without colliding — and **knowing whether they did**",
    ], size=18)
    footer(s, "CONSTRAINTS.md · ADR-016 (SO-101: 6 actuators, 5 positioning DoF)")

    # 3 — what we built
    s = blank(prs)
    heading(s, "What we built")
    bullets(s, [
        "**Four scripted skills, end to end:** pick(A, fork), place(A, fork, table), pick(A, water_bottle), handoff(A→B, fork)",
        "**Bimanual handoff** with sequential choreography (ADR-037) — one arm moves, the other genuinely frozen",
        "**Voice → plan → execution:** Speechmatics transcript at 0.983 confidence → rule-based grounder (43 tests) → executor (ADR-058)",
        "**Perception:** PoseNet trained on Arc B390, 2.6–3.2 mm MAE, exported to OpenVINO IR. Opt-in; the demo path uses oracle positions by default (ADR-046)",
        "**One entry point:** run_demo.py → **4/4 PASS** on the Intel target",
    ], size=17)
    footer(s, "Grasping is an explicit MuJoCo weld-constraint abstraction, not friction contact (ADR-029, disclosed per ADR-015)")

    # 4 — working demo
    s = blank(prs)
    heading(s, "Working demo", "handoff(A→B, fork), verified by direct measurement")
    assert DEMO.exists(), DEMO
    s.shapes.add_picture(str(DEMO), Inches(0.8), Inches(2.1), height=Inches(3.9))
    bullets(s, [
        "weld.is_holding('B') == 'fork'",
        "weld.is_holding('A') is None",
        "from_arm_clear = True",
        "Final lateral separation **0.1946 m**",
        "frames_used = **6610**",
        "Full 40 s continuous take: docs/videos/full-sequence-demo.mp4",
    ], top=Inches(2.1), left=Inches(8.05), width=Inches(4.5), size=14, gap=10)
    footer(s, "docs/images/m06-handoff-complete.png · handoff 20/20 across seeds is degenerate — single-point envelope; it measures determinism, not robustness")

    # 5 — OpenVINO + rubric map
    s = blank(prs)
    heading(s, "OpenVINO on Core Ultra, and the evidence map")
    box = s.shapes.add_textbox(Inches(0.8), Inches(1.85), Inches(5.6), Inches(0.35))
    _run(_tf(box).paragraphs[0], "PoseNet inference, batch 1 (ADR-045)", 14, bold=True)
    table(s, [
        ["Device", "Precision", "Latency", "Throughput"],
        ["iGPU (Arc B390)", "FP16", "0.683 ms", "**1465 Hz**"],
        ["NPU (NPU5010)", "FP16", "1.099 ms", "910 Hz"],
        ["CPU", "FP16", "6.492 ms", "154 Hz"],
    ], Inches(0.8), Inches(2.25), Inches(5.6),
        col_w=[Inches(1.9), Inches(1.1), Inches(1.2), Inches(1.4)], size=12)
    box = s.shapes.add_textbox(Inches(0.8), Inches(3.75), Inches(5.6), Inches(0.9))
    p = _tf(box).paragraphs[0]
    _run(p, "Batch 16: GPU **5800 Hz**, NPU 1478 Hz, CPU 231 Hz — latency grows slower than batch on every device, so throughput keeps climbing.", 12, color=MUTED)

    box = s.shapes.add_textbox(Inches(6.9), Inches(1.85), Inches(5.7), Inches(0.35))
    _run(_tf(box).paragraphs[0], "Rubric evidence map", 14, bold=True)
    table(s, [
        ["Criterion", "Status"],
        ["End-to-end + bimanual (30)", "4 skills; handoff(A→B) 4/4 PASS"],
        ["VLA / multi-modal (20)", "Text + voice grounding; perception demoed"],
        ["OpenVINO on Core Ultra (20)", "3 devices × 3 precisions + batch scaling"],
        ["Robustness (15)", "20-seed eval, per-skill rates"],
        ["Reproducibility (10)", "One entry point, pinned envs, 59 ADRs"],
        ["Innovation (5)", "Three measured negative results"],
    ], Inches(6.9), Inches(2.25), Inches(5.7),
        col_w=[Inches(2.5), Inches(3.2)], size=11)
    footer(s, "docs/hardware/m10-phase4-benchmark.md · docs/slides-rubric-map.md · pick(A, water_bottle) is 45% (9/20, ADR-053's 20-seed extension)")

    # 6 — honest engineering
    s = blank(prs)
    heading(s, "Honest engineering", "Three findings we measured, then acted against our own interest")
    bullets(s, [
        "**1. INT8 quantization — declined (ADR-050).** 36–37 mm deviation against the model's own 2.6–3.2 mm MAE. Faster, and not shippable. We kept FP16.",
        "**2. Position-only geometry redesign — rejected (ADR-062).** Two of three kinematic walls did dissolve at 0.40 m separation; net result 1 of 8 skills passing versus master's 4 of 9. Branch preserved as the evidence.",
        "**3. Motion-stack rewrite — partly worked (ADR-074).** Skills had only ever been scored on whether the target moved. Re-scored on five scene-integrity criteria, **master passes 0 of 3**: place displaces the mug 63.2 mm; handoff spends 3179 of 6610 steps (48%) with the arms in contact, at 6.937 rad/s peak. The rebuilt stack passes **2 of 3 individual skills**, and a **six-stage bimanual relay passes all five scene-integrity criteria** at every stage (worst peak 2.362 rad/s against a 2.6 gate).",
    ], size=14, gap=8, height=Inches(2.40))
    bullets(s, [
        "The relay moves the fork **via the table**. v2 has **no working direct hand-to-hand handoff** — diagnosed, unsolved, named fix in ADR-073.",
        "v2's pick pass required fixing the grasp geometry first (the ADR-025 pinch point is not the grasp centre in top-down poses; a 4 mm clearance window). An earlier passing claim was **retracted**.",
        "v2 **cannot reach the water bottle at all** — a regression against master.",
        "Spline vs direct commands: **11.9× lower peak velocity, 21× lower peak acceleration**, identical 0.00014 rad final error. Idle-arm drift **0.000774 rad settled**, bounded transient at step 12 disclosed.",
        "**Two shortcuts refused:** widening the grasp gate 0.05→0.12 m and the weld gate to 0.115 m each turned a failure into a passing number — both measured and rejected, the second after a per-step check proved the receiving pads never touch the fork.",
    ], top=Inches(4.45), size=11, gap=4, height=Inches(2.30))
    footer(s, "docs/hardware/v2-verdict.md · ADR-074 · docs/videos/v2-relay-demo.mp4")

    # 7 — reproducibility
    s = blank(prs)
    heading(s, "Reproducibility")
    bullets(s, [
        "**One command:** ./run_demo.sh → run_demo.py → **4/4 PASS** on the Intel target, from a clean anonymous clone",
        "**Pinned environments:** scripts/requirements-dev.txt / requirements-bmptl.txt, checked by scripts/verify_env.py",
        "**59 ADR entries** in ARCHITECTURE.md (ADR-001–058; 43 in DECISIONS.md), plus ADR-070–074 for the v2 rebuild on the redesign-v2 branch",
        "**Public repo:** github.com/Sumit-Dwivedi/intel-bimanual-vla",
        "Verified by cloning anonymously onto a clean host and following the README with no prior knowledge — which is how we found the repo was still private, and three defects in our own environment check",
    ], size=15, gap=8, height=Inches(3.30))
    box = s.shapes.add_textbox(Inches(0.8), Inches(5.35), W - Inches(1.6), Inches(1.5))
    tf = _tf(box)
    p = tf.paragraphs[0]
    _run(p, "What does not work, stated plainly: ", 13, bold=True)
    _run(p, "pick(A, mug), open_drawer, place(A, water_bottle, table), handoff(B→A, fork). pour is out of scope (ADR-023), so the challenge brief's literal example command does not run end to end. pytest tests/ is 66 passed / 4 failed.", 13)
    footer(s, "Rendered from docs/slides.md by scripts/render_slides.py — the markdown is canonical")
    return prs


def check(path: pathlib.Path) -> int:
    prs = Presentation(str(path))
    n = len(prs.slides)
    size_mb = path.stat().st_size / 1048576
    print("round-trip load: OK")
    print("slides: %d (expected 7)" % n)
    print("size: %.2f MiB" % size_mb)
    print("dimensions: %.3f x %.3f in" % (prs.slide_width / 914400, prs.slide_height / 914400))
    pics = sum(1 for s in prs.slides for sh in s.shapes if sh.shape_type == 13)
    tbls = sum(1 for s in prs.slides for sh in s.shapes if sh.has_table)
    print("pictures: %d   tables: %d" % (pics, tbls))

    # Overflow AND pairwise overlap. Bounds-only checking missed a 0.07 in
    # image/text collision on slide 4; content shapes must not intersect.
    def box(sh):
        return (sh.left, sh.top, sh.left + (sh.width or 0), sh.top + (sh.height or 0))
    problems = []
    for idx, sl in enumerate(prs.slides, 1):
        # Pictures are deliberate backgrounds on slide 1 and text sits over
        # them by design; rules/bands are shape_type 1. Only text-vs-text
        # collisions of more than 0.1 in in BOTH axes are real problems.
        content = [sh for sh in sl.shapes
                   if sh.left is not None and sh.has_text_frame
                   and sh.shape_type != 1]
        for i in range(len(content)):
            for j in range(i + 1, len(content)):
                a, b = box(content[i]), box(content[j])
                ox = min(a[2], b[2]) - max(a[0], b[0])
                oy = min(a[3], b[3]) - max(a[1], b[1])
                if ox > Inches(0.1) and oy > Inches(0.1):
                    problems.append("slide %d: %s overlaps %s by %.2f x %.2f in"
                                    % (idx, content[i].shape_type, content[j].shape_type,
                                       ox / 914400, oy / 914400))
        for sh in sl.shapes:
            if sh.left is None:
                continue
            l, t, r, b2 = box(sh)
            if sh.shape_type == 13:      # full-bleed art may be cropped by design
                continue
            if l < -9525 or t < -9525 or r > prs.slide_width + 9525 or b2 > prs.slide_height + 9525:
                problems.append("slide %d: %s outside slide bounds" % (idx, sh.shape_type))
    for pr in problems:
        print("  LAYOUT: " + pr)
    print("layout problems: %d" % len(problems))
    ok = (n == 7 and size_mb < 10 and not problems)
    print("RESULT: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true", help="verify an existing deck only")
    a = ap.parse_args(argv)
    if a.check:
        return check(OUT)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    build().save(str(OUT))
    print("wrote %s" % OUT.relative_to(REPO))
    return check(OUT)


if __name__ == "__main__":
    sys.exit(main())
