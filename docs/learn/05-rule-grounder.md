# 05 — Rule Grounder: Lowering English to a Typed Plan

*Written before M05 is built. This is the design about to be built, not a result.*

**Problem.** A `CommandEvent` carries one English sentence; the Coordinator needs ordered
steps, each bound to a specific arm. M05 is the front end that bridges them.

**Key concept: a tiny DSL front end.** `ground(CommandEvent, SceneBelief) -> TaskPlan`
(ARCHITECTURE.md component table) lowers text to typed IR — a `TaskPlan` of `SkillCall`s
(`language/skills.py`) over a fixed vocabulary: `open_drawer`, `pick`, `place`, `handoff`,
`pour` (ADR-001's two tiers, ADR-011's core sequence), matched by `rule_grounder.py`. The
brief's own command becomes `open_drawer(arm=A) → pick(plate, A) → place(plate, table, A)
→ pick(mug, B) → pour(into=mug, A)`. Arm is a first-class field — `handoff` is how an
object crosses arms — and M05 done-when 3 forbids `None`.

**Why rules.** PLAN.md M05 calls this "the deterministic floor... the piece that must never
fail on demo day". ADR-003 keeps `VlmGrounder` as a stretch behind the same `Grounder`
interface: a floor with an optional upgrade, not a rejection of models. A judge can read
`docs/command-grammar.md`; nobody audits a prompt that way.

**Brittle, defensibly.** It fails loudly — done-when 2 requires a typed
`UngroundedCommandError`, never a partial plan. A clean refusal beats executing three of
five steps on stage.

**Try this.** Hand-write the plan for "pick up the mug and pour water into it." Which arm?
That ambiguity is what the grammar must resolve or reject.
