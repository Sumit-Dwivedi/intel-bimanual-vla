# Project Constraints

## Hard Deadlines
- Hackathon submission: September 16, 2026
- bm-ptl instance expires: September 17, 2026 at 00:15 local (end of Sept 16)
- Kick-off date: [today's date]
- Days remaining: [5 days 10 Hour]

## Hardware
- bm-ptl: Intel Core Ultra 7 358H, Panther Lake, 8-16 cores, 32-64 GB
- OS: Windows 11
- NPU: NPU5010
- IP: 192.168.2.2 (via SSH jump host guest@146.152.207.201)
- Credentials: devcloud / devcloud (must change on first login)

## Developer Context
- Software background, no prior robotics experience
- Simulation-first: no physical robot
- Primary goal: learn robotics + Intel edge deployment
- Secondary goal: submit a defensible, complete entry

## Required Deliverables
1. Public GitHub repository with reproducible setup
2. MuJoCo simulation of dual SO-101 arms on a table-setting task
3. OpenVINO benchmark script running on bm-ptl (CPU / iGPU / NPU)
4. Demo video across 10 randomized seeds
5. Cover image, video presentation, slide presentation
6. Application URL (or repo link)
7. Technical README explaining architecture

## Rubric Weights
- End-to-end task completion & bimanual: 30
- VLA / multi-modal reasoning: 20
- OpenVINO & Core Ultra optimization: 20
- Robustness across 10 seeds: 15
- Reproducibility: 10
- Innovation: 5
Also judged: Application of Technology, Presentation, Business Value, Originality

## Bonus Award
- Speechmatics voice input as front-end to VLA text pipeline
- Standard model, laptop-side, non-blocking
- Drop if timeline slips

## Out of Scope
- Winning the top prize
- Custom novel architectures
- Training a VLA from scratch
- Physical hardware
- Windows-only tooling that blocks Linux teammates (single-dev project, moot)

## Fallback Strategy
If learned policy is not working by end of Day 3, ship a scripted IK-based
controller as the "policy." Rubric rewards complete pipeline over half-working ML.

## Compute
- Local Windows laptop: dev, VS Code, Claude Code, git, mic capture
- bm-ptl (Windows): OpenVINO conversion, NPU/iGPU benchmark, demo recording
- Kaggle: GPU training (30 hrs/week free, CLI-driven)
- Not using: Colab, physical hardware, Intel Tiber training nodes