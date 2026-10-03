---
description: Execute one Part of PLAN.md end-to-end (e.g. /part 2)
argument-hint: <part number>
---
Execute Part $ARGUMENTS of PLAN.md.
1. Read docs/STATUS.md and ONLY that Part in PLAN.md, plus the rule docs from the CLAUDE.md routing table that apply.
2. Restate the acceptance test and list the files you will create/change (brief). If a needed fact is missing, ask once, concisely.
3. Implement in small steps; add tests with the code. Run `make lint typecheck test` (and the SITL acceptance test when the part defines one).
4. Fix failures yourself; use the sitl-debugger agent for SITL logs instead of reading raw logs.
5. Use the code-reviewer agent on the final diff and fix high-severity findings.
6. Stop when the acceptance test passes. Update docs/STATUS.md and commit. Do not start the next Part.
