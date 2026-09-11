# Submission Checklist

Track every deliverable required by the hackathon platform. Docs-writer
updates status marks as artifacts land. Nothing here is written on Day 5
that wasn't planned from Day 1.

## Basic Information
- [ ] Project title: [DRAFT: Bimanual VLA on Intel Core Ultra — a voice-driven table-setting demo]
- [ ] Short description (1-2 sentences): TODO
- [ ] Long description (~300 words): TODO
- [ ] Technology tags: OpenVINO, MuJoCo, LeRobot, SmolVLA, Speechmatics, Intel Core Ultra
- [ ] Category tags: Robotics, Physical AI, Edge AI

## Cover Image & Presentation
- [ ] Cover image (1200x630 recommended): TODO — screenshot of MuJoCo scene with both arms mid-task
- [ ] Video presentation (demo + narration): TODO — see docs/video-script.md
- [ ] Slide presentation (5-10 slides): TODO — see docs/slides.md

## App Hosting & Repository
- [ ] Public GitHub repo: [URL TODO — currently private, flip to public on Sept 15]
- [ ] Demo application platform: N/A (this is a local sim, not a hosted app) — clarify with lablab if in doubt
- [ ] Application URL: link to GitHub repo README or a recorded demo page

## Rubric Alignment (Intel + platform)
Intel-specific (100 pts):
- [ ] End-to-end task completion & bimanual (30)
- [ ] VLA / multi-modal reasoning (20)
- [ ] OpenVINO & Core Ultra optimization (20) — Day 0 device access proven; **M03 complete
  Sept 12**: PyTorch→IR→compile→infer verified on CPU, GPU and NPU, evidence in
  `benchmarks/ov-smoke-notes.md` (NPU requires static/bounded batch — see DECISIONS.md M03)
- [ ] Robustness across 10 seeds (15)
- [ ] Reproducibility (10)
- [ ] Innovation (5)

Platform-general:
- [ ] Application of Technology
- [ ] Presentation
- [ ] Business Value
- [ ] Originality

Speechmatics bonus:
- [ ] Speech input wired to VLA text pipeline
- [ ] Working demo clip with voice command

## Assets to Produce
- [ ] `docs/video-script.md` — narrated walkthrough script
- [ ] `docs/slides.md` — outline for slide deck
- [ ] `docs/cover-image-brief.md` — what the cover image shows
- [ ] `benchmarks/bmptl-results.md` — device latency table for README
- [ ] `README.md` — judge-facing overview