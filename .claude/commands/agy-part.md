---
description: Have agy implement a Part, then Claude Code reviews it (e.g. /agy-part 2)
argument-hint: <part number>
---
Orchestrate Part $ARGUMENTS. agy writes code; you review. Do NOT edit source files yourself.
1. Ensure git is clean, then `git checkout -b part-$ARGUMENTS`.
2. Run agy headless (one Bash call, long timeout):
   `agy -p "Read CLAUDE.md, then execute Part $ARGUMENTS of PLAN.md exactly as .claude/commands/part.md describes. Log decisions in docs/DECISIONS.md and the Part entry in docs/IMPLEMENTATION.md. Stop when the acceptance test passes." --mode accept-edits --effort high --print-timeout 3000s > logs/agy_part$ARGUMENTS.log 2>&1`
3. Do not read the agy log raw. `tail -30` only, or use the sitl-debugger agent.
4. Run `make lint typecheck test`, plus the Part's SITL acceptance test.
5. Use the code-reviewer agent on `git diff main`. Check the diff against PLAN.md, docs/rules/ and pitfalls.md.
6. Verify docs/IMPLEMENTATION.md: every VERIFIED row must match a command you re-ran. Downgrade any that don't.
7. If problems are found, write a numbered punch list and re-run agy with `-c` (continue) and only that list. Max 3 rounds.
8. When clean, commit and report: pass/fail per item, remaining risks. Do not merge.
