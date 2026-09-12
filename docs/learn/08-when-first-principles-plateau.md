# 08 — When First Principles Plateau

*The transferable lesson from note 07. Reasoning from first principles is the right
default, right up until it silently stops working.*

**The signals.** You are past the plateau when: a third consecutive fix cycle produces no
success; each fix reveals a new layer rather than closing the problem; the metric you care
about moves the *wrong* way; and fixes keep being proposed confidently while nothing
physical changes. Any one is noise. Together they mean your model of the system is wrong
in a way more reasoning will not repair, because every new inference is drawn from the
same wrong model.

**What to do instead.** Instrument first, fix second — a probe costs minutes and settles
what an argument cannot. Search for practitioners who hit the same wall; someone has
usually already paid for this lesson. Read the tool's own issue tracker, not just its
docs, because known defects live there long before they reach documentation. And ask
whether the problem is your solution's *shape* rather than its *parameters* — that is the
question tuning can never answer for you.

**The time cost, stated defensibly.** The convex-hull defect only became *identifiable*
after waypoint staging landed at ~13:44. From then until the external evidence arrived at
~15:48 is roughly **two hours** spent on gripper-side fixes that could not have worked:
all four tuned *how hard the jaws squeeze*, when the jaws could not close on anything at
all. That is the honest number. Not eight hours, not four — the earlier work found and
fixed three genuine, independent defects (drawer placement, IK pinch point, the resting
inter-arm collision). Two hours, and here is exactly why.

**Try this.** Before the next fix you propose, write down the one measurement that would
prove it wrong. If you cannot name one, you are tuning parameters on a shape that is
already broken.
