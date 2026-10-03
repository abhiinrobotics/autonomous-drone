# DECISIONS (append-only log; never delete entries, supersede them)

Rule: log every non-obvious choice (architecture, parameters, tradeoffs, assumptions, deviations from PLAN.md/rulebook). Newest at the bottom. If a decision changes, add a new entry and mark the old one `Superseded by D-xxx`.

Entry template:

## D-001 — <short title>
- Date / Part: YYYY-MM-DD / Part N
- Status: Active | Superseded by D-xxx | Reverted
- Context: why this came up (1-3 lines)
- Decision: what we chose
- Alternatives: what was rejected and why
- Consequences: impact, risks, what must be re-checked (e.g. on hardware)
- Evidence: test/commit/measurement that supports it (or "assumption — unverified")

---
