# Contributing

## Scope

This repository is a submission for the Intel Physical AI Online Challenge
at the AI Infra Summit Hackathon (Sept 10-16, 2026). It is **not currently
accepting external code contributions** — the submission window is short,
the scope is deliberately narrow and already cut down once (see
`ARCHITECTURE.md` / `DECISIONS.md` for the ADRs that record what was cut and
why), and the maintainer needs to control exactly what lands before judging
closes. This may change after the hackathon concludes and the submission is
scored.

If you're reading this as a judge or a fellow participant rather than a
prospective contributor, the rest of this file still applies to you for
reproduction and issue reporting.

## Reproducing the demo

`run_demo.sh` at the repo root is the intended entry point for reproducing
the demo end-to-end. Before running it, check your environment against
`scripts/requirements-dev.txt` (laptop) or `scripts/requirements-bmptl.txt`
(Intel target — CPU/iGPU/NPU), and run `scripts/verify_env.py` to confirm
the install matches. `SUBMISSION.md` records, without embellishment, which
skill/arm/object combinations currently execute end-to-end and which do
not, and why. `pytest tests/test_skills.py` is the authoritative pass/fail
signal for the scripted skills — as of this writing it reports 4 passed / 4
failed; do not assume a clean run.

## Reporting issues

If you find a bug, a broken reproduction step, or a discrepancy between a
claim in `README.md` / `SUBMISSION.md` and what the code actually does,
please open a GitHub issue on this repository. Include the exact command
you ran, the output you got, and — if relevant — which device (CPU/iGPU/NPU)
or machine you ran it on. Several documented behaviors in this repo differ
between machines (see `DECISIONS.md`), so that detail matters.

## Decision log

Architectural and scope decisions are recorded as ADRs in `ARCHITECTURE.md`
and mirrored with one-line summaries in `DECISIONS.md`. If you're wondering
"why does it work this way, and not some other way," check there first.

## Contact

The maintainer is reachable via GitHub issues on this repository. There is
no other support channel for this project.
