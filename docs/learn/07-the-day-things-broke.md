# 07 — The Day Things Broke

*M06 day, Sept 12, 2026 — the day we built the controller. Notes 01–06 describe designs.
This one describes what happened when a design met physics.*

**~10:40.** M06a launched: four scripted skills — `open_drawer`, `pick`, `place`,
`handoff` — on the IK loop note 06 described.

**~11:20.** All four failed. Two causes diagnosed: the drawer sat under a solid tabletop
with no cutout, and IK was aimed at `gripperframe`, an upstream site ~8 cm from where the
jaws actually pinch. Both real. Neither was the real problem.

**~11:55–12:42.** A reachability probe ran *before* any skill was re-tested — which is
what made it cheap — and found what nobody had thought to look for: at rest the two arms
interpenetrated by up to **6 cm**, with 29 of 34 contacts arm-against-arm. ADR-021 placed
the bases 0.50 m apart assuming a 0.10 m shared band, but the upstream default joint
angles extended both arms fully into it. A `home` keyframe fixed it: **0 cross-arm
contacts** (ADR-026). The envelope was then re-measured from a *valid* pose — the old one
had been measured from a colliding pose and was worthless.

**~13:08–13:44.** With a valid pose the next layer surfaced: IK is collision-blind *by
design* (ADR-024 chose position-only solving). It was solving *through* the tabletop.
Waypoint staging — approach, descend, grip, retreat (ADR-027) — fixed it. That is exactly
the pattern note 06 sketched before any of this was built.

**~13:44–15:48.** A four-fix grasp ladder: jaw friction, grasp point, closure force, fine
collision geometry. All failed. Worse, they were applied cumulatively, so Fix B's broken
offset blocked C and D from ever executing, and Fix D silently **disabled jaw collision
entirely** — kept because plate-z looked "neutral", which masked that the gripper could no
longer touch anything.

**~15:48.** The *user* — not an agent — brought external evidence: a practitioner blog
post on this exact arm, and MuJoCo issue #239.

**~16:00–16:25.** Finger-pad primitives landed (ADR-028) and the root cause finally had a
name: MuJoCo collapses a mesh geom to its **convex hull** unless told otherwise, and the
SO-101's C-shaped jaws became solid blobs overlapping permanently — measured negative at
*every* joint angle, **−0.0345 m** closed to **−0.0206 m** at best. Nothing could ever sit
between them. With box pads, jaw opening became monotonic in joint angle.

**~16:30–17:00.** A target-prop exemption let `pick(A, fork)` reach GRIP for the first
time, every waypoint passing.

**~17:00–18:50.** Grip diagnostics: the pads land *below* the table surface and never
touch the fork; raising the target 4 mm moved the pad 0.1 mm while pushing IK past
convergence. Structural limit reached.

## What to take from this

- **Measurement over reasoning.** Nearly every confident geometric inference today was
  wrong — including two of mine, corrected by probes. The probes were right every time.
- **Order matters.** Running the reachability probe before the skill re-runs is what kept
  the wrong drawer position from costing an afternoon.
- **Pre-committed gates worked.** `pour` was dropped cleanly at the 17:30 gate with no
  re-litigation, exactly as the cut ladder specified.
- **Honest reporting enabled diagnosis.** Every agent reported failures with numbers
  rather than softening them; that is the only reason the causes were findable.
- **Cumulative fixes hide each other.** Revert-on-regression is not bureaucracy — Fix B
  masked two untested hypotheses and Fix D introduced an invisible regression.

**Try this.** Open `docs/hardware/m06-reachability-probe.md` and read the first envelope
next to the ADR-026 re-measured one. Same script, same targets, different rest pose — and
the second table's collision column tells the truth the first one's baseline was hiding.
