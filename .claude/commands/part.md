---
description: Execute one Part of PLAN.md end-to-end (e.g. /part 2)
argument-hint: <part number>
---
Execute Part $ARGUMENTS of PLAN.md.
1. Read docs/STATUS.md and ONLY that Part in PLAN.md, plus the rule docs from the CLAUDE.md routing table that apply. Skim the tail of docs/DECISIONS.md and docs/IMPLEMENTATION.md for prior context.
2. Restate the acceptance test and list the files you will create/change (brief). If a needed fact is missing, ask once, concisely.
3. Implement in small steps; add tests with the code. Run `make lint typecheck test` (and the SITL acceptance test when the part defines one).
4. Fix failures yourself; use the sitl-debugger agent for SITL logs instead of reading raw logs.
5. Log each non-obvious decision/assumption in docs/DECISIONS.md as you make it.
6. Use the code-reviewer agent on the final diff and fix high-severity findings.
7. Append the Part entry to docs/IMPLEMENTATION.md (files built, verify commands, status table with real evidence, deviations, gaps). Update docs/STATUS.md pointer and commit.
8. Stop when the acceptance test passes. Do not start the next Part.
