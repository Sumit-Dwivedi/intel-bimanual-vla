# 04 — CommandSource: One Seam, Two Implementations

*Written before M04 is built. This is the design about to be built and why, not a result.*

**Problem.** A command can arrive as a typed string or as speech, and nothing downstream
should care which. M04 draws that line once, so voice becomes a plug rather than a rewrite.

**Key concept: Strategy plus dependency injection.** `CommandSource` is the strategy
interface — `poll() -> CommandEvent | None` and `close()` — with `TextCommandSource` and a
stub `VoiceCommandSource` as interchangeable implementations (`PLAN.md` M04 outputs). The
caller is *handed* a source and just calls `poll()`; it never asks which kind it got, so
there is no `isinstance` branch waiting to sprout a third arm.

**Why the VLA never sees audio.** ADR-002 in `ARCHITECTURE.md`: everything past the seam
consumes a `CommandEvent` carrying `text`, never a buffer, sample rate or codec — as the
package docstring in `src/bimanual/command/__init__.py` already states. This is enforced,
not merely preferred: M04 done-when 2 greps `src/bimanual/language` and
`src/bimanual/policy` for `audio|pcm|wav|microphone` and requires zero matches. Voice is a
droppable bonus (M15), so the payoff is that dropping it costs one class, not a refactor.

**Try this.** Before reading `text_source.py`, write the caller loop you'd want. If it
names a concrete source anywhere but construction, the abstraction has leaked.
