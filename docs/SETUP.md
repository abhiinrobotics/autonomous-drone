# Setup (human-facing)

## 1. Project
```bash
cd ~/aerothon_2026            # unzip the scaffold here
git init && git add -A && git commit -m "chore: scaffold"
python3 -m venv .venv && source .venv/bin/activate
make install                  # deps + links Gazebo's Python bindings into the venv
sudo apt install libzbar0 ffmpeg   # QR decoding + video
npm i -g pyright              # or: pip install pyright  (needed by the LSP plugin)
claude
```
Gazebo (`gz sim`, Harmonic) and a built PX4 SITL (`make -C ~/PX4-Autopilot px4_sitl`) must
already be present; `make check-env` reports what is missing. `make install` symlinks the
apt-installed `gz` Python package into the venv — the bindings are not on PyPI, and the
project pins `protobuf==3.20.3` because they are generated against the pre-4.x API.
Run every tool through `make` (it clears the ROS `PYTHONPATH` that otherwise breaks pytest).

First message: `/part 0`. After a Part passes: `/handoff`, then `/clear`, then `/part <next>`.

## 2. Plugins (install inside Claude Code)
| Plugin | Why | Command |
|---|---|---|
| pyright-lsp | Type errors / go-to-definition right after edits; fewer wrong-signature bugs, fewer reread tokens | `/plugin install pyright-lsp@claude-plugins-official` |
| context7 | Up-to-date docs for pymavlink, OpenCV, Gazebo/gz, PX4 versions instead of guessing from memory | `/plugin install context7@claude-plugins-official` |
| commit-commands | Clean commits with one command | `/plugin install commit-commands@claude-plugins-official` |
| code-review (optional) | Extra multi-agent review before milestones; costs more tokens, use per Part not per edit | `/plugin install code-review@claude-plugins-official` |
Already built in this repo (no install): ruff format/lint hook on every Python edit, `code-reviewer`, `sitl-debugger`, `vision-tester` agents, `/part /status /review /sitl-debug /handoff` commands, permission allow/deny rules. Marketplace names change; if one is missing run `/plugin` and browse.

## 3. Saving tokens
- One Part per session; `/handoff` then `/clear`. State lives in `docs/STATUS.md`, not in chat history.
- Use `/model` to pick Opus for architecture-heavy Parts (2 FlightLink, 5 search/planning, 7 return nav) and Sonnet for the rest (0, 1 generators, 3 tests, 8-9). Plan mode (Shift+Tab) at the start of a Part for a cheap review of the approach.
- Never let it open the PDFs (denied in settings); specs are condensed in `docs/spec/`.
- Logs go through `sitl-debugger` (summaries), not into the main context. Keep `pytest -q`.
- Keep CLAUDE.md short; detail lives in `docs/rules/*` loaded on demand.
- Images: only ask it to view a frame when debugging vision; save frames to `artifacts/`.
