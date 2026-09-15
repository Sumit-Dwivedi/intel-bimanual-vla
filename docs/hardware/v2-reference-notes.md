# v2 reference notes — read from source, for verifying agent claims

Orchestrator's own reading of the four references in the v2 brief. Kept out of
the repo while Stage 1's builder owns DECISIONS.md / ARCHITECTURE.md; fold in
at a later stage boundary.

## Lab 7 — geometric IK

- **1b θ1**: project target onto the ground plane, trigonometry. Explicit
  warning, verbatim: *"if you do not account for the frame's offset in the
  x-axis correctly, the robot will not line up exactly with the block!"*
  → matches the measured **−0.027 m lateral pinch offset** in our MJCF.
- **1c wrist position**: `gw4 = gwt @ np.linalg.inv(g4t)`, where
  `g4t = g45(0) @ g5t`.
- **1d θ2, θ3**: law of cosines on the triangle between frames at joints 1, 2,
  3. Lab does **not** state an elbow-up/elbow-down preference — so that is OUR
  choice to make and document.
- **1e θ4**: aligns the end-effector z-axis with world z by projecting both
  onto the x-y plane and taking the angle between them. (The brief's
  `θ4 = -(θ2+θ3)+const` is an equivalent planar shortcut, not Lab 7's wording.)
- **1f θ5**: **`θ5 = θ1`** — opposite joint axes cause a double negation of
  `θ5 = -θ1`. The brief's `θ1 + yaw` is this plus a yaw extension.

## Lab 8 — pick/place phasing

- Pick: **approach (above) → descend → close → lift**. Place: approach →
  descend → open → retract.
- **Approach height is 0.03 m**, NOT the brief's 0.08 m.
- Warning that matters for us: *"the workspace narrows significantly at
  increasing z-heights."* So an 0.08 m hover may be LESS reachable than 0.03 m
  — check both in the Stage 1 workspace map before Stage 4 commits to 0.08.
- Home = all joint angles zero. **This is a SINGLE-arm home.** Our dual-arm
  scene measures 19 cross-arm contacts, deepest −0.0597 m, at all-zeros
  (ADR-026 recorded the same −0.0597 m). Lab 8's home does not transfer.
- Timing: 1.0 s descend/gripper, 2.0 s longer moves — matches the brief.
- Hardware notes: verify IK on hardware first; −0.015 m z-offset for the base
  platform; log target vs actual joint positions for tracking error.

## Lab 9 — cubic splines

- Lab leaves the coefficients as a student exercise; the brief supplies them.
  **Verified independently — they are correct:**
  `a0=q_start, a1=0, a2=3Δ/T², a3=-2Δ/T³`
  gives q(0)=q_start, q(T)=q_target, q'(0)=q'(T)=0.
- Evaluation with clamped t:
  `pos = a0 + a1*tlim + a2*tlim**2 + a3*tlim**3`
  `vel = a1 + 2*a2*tlim + 3*a3*tlim**2`
- Closed forms to CHECK Stage 2b against, rather than trusting the report:
  - **peak velocity = 1.5 * Δ / T**, at t = T/2
  - **peak acceleration = 6 * Δ / T²**, at t = 0 and t = T
- Comparison is direct commands vs linear interp vs cubic; splines win, but the
  lab gives only visual graphs, **no quantitative metric**. So "Lab 9 shows what
  the difference should look like" cannot be cited for a number.

## ggando — same arm, same symptoms

- Gripper overshot and failed to centre: wrong target frame, plus
  **asymmetric gripper (one fixed finger, one moving) needing offset
  compensation**. Independent corroboration of our −0.027 m lateral offset.
- Wobbling object: mesh-to-mesh collisions give unstable single contact points
  (MuJoCo issue #239). Fix: small box geoms at the fingertips. → this is our
  ADR-028 finger-pad primitives, which the brief correctly keeps.
- Adopted the same 4-step pick sequence from course material to replace
  "janky motion".
- **Naming trap:** ggando says target the "gripperframe" (their TCP at the
  fingertips). In OUR MJCF `armX_gripperframe` is a DIFFERENT site, 0.0888 m
  from the pinch point (ADR-025). Same concept, clashing names — do not let an
  agent "fix" our target to the gripperframe site on ggando's authority.

## Orchestrator corrections to the brief (all measured on bm-ptl)

1. `model.opt.timestep = 0.002 s`. Stage 5's "every 6th step at 30 fps =
   real motion speed" is wrong: stride 6 → **0.360x** real speed. Real time at
   30 fps needs **stride 17** (16.67 exact).
2. Stage 3's all-zeros start already fails its own contact gate: **19 cross-arm
   contacts, deepest −0.0597 m**.
3. The brief has the home poses backwards. Current HOME is
   `shoulder_lift=-1.2, elbow_flex=-1.6` — already FOLDED. All-zeros is the
   EXTENDED pose that collides.
4. "ADR-058's position-only workspace" is `redesign` branch ADR-058. On
   master / redesign-v2, ADR-058 is Speechmatics voice. Genuine collision —
   always qualify by branch.
