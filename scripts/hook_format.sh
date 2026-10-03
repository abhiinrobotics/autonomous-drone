#!/usr/bin/env bash
# PostToolUse hook: format + lint the edited Python file. Silent on success; exit 2 feeds issues back to Claude.
f=$(python3 -c "import sys,json;print(json.load(sys.stdin).get(\"tool_input\",{}).get(\"file_path\",\"\"))" 2>/dev/null)
case "$f" in *.py) ;; *) exit 0 ;; esac
command -v ruff >/dev/null 2>&1 || exit 0
ruff format -q "$f"
ruff check --fix -q "$f" 1>&2 || exit 2
