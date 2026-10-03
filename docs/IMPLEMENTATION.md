# IMPLEMENTATION LOG (append-only per Part; update status cells in place)

Purpose: exact record of what was built, how to run it, and whether it actually works.
Rule: a row may be marked VERIFIED only after the command was actually run; record the command and the result. Otherwise UNVERIFIED. If it fails: BROKEN + one-line cause.

Entry template:

## Part N — <title>  (commit <hash>, YYYY-MM-DD)
### Built
- `path/to/file.py` — what it does (1 line each)
### How to run / verify
- `command` -> expected result
### Status
| Item | Status (VERIFIED / UNVERIFIED / BROKEN) | Evidence (command + result, seeds, date) |
|---|---|---|
### Deviations from PLAN.md
- (none) | what changed and the matching D-xxx
### Known gaps / TODO
- ...
### Changes after initial implementation
- YYYY-MM-DD <hash>: what changed, why (D-xxx), re-verified? yes/no

---
