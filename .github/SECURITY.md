# Security Policy

This is a hackathon research submission (Intel Physical AI Online Challenge,
AI Infra Summit Hackathon, Sept 10-16 2026), not production software. It
runs entirely as a local MuJoCo simulation — there is no deployed service,
no hosted endpoint, no physical robot attached to this code, and no user
data collected anywhere in the pipeline.

Being plain about scale: there is no security team behind this project, no
on-call rotation, and no service-level agreement for responding to reports.
It is one person's submission, and this file exists to say that honestly
rather than to imply a formal disclosure process that does not exist.

## Reporting a concern

If you find something that looks like a security issue — a dependency with
a known CVE in `scripts/requirements-*.txt`, code that executes untrusted
input unsafely, or credentials accidentally committed to the history —
please open a GitHub issue on this repository describing what you found.
Please do not expect a guaranteed response time.

## Scope

Because this project has no deployed surface, most conventional web/API
security concerns (auth, injection over a network boundary, etc.) do not
apply. The categories actually worth flagging here are: secrets or tokens
committed to the repository, unsafe deserialization or `eval`-style code
paths, and known-vulnerable pinned dependencies.
