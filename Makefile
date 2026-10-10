# ROS 2 Humble is installed system-wide and exports PYTHONPATH=/opt/ros/humble/...;
# its pytest entry-point plugins then load into our runs and crash on a missing
# dep (lark). Every target runs with PYTHONPATH cleared and with the repo venv's
# interpreter, so results do not depend on whether the venv is activated.
VENV ?= .venv
PY := env -u PYTHONPATH $(VENV)/bin/python

.PHONY: help venv install lint format typecheck test test-sitl check-env sitl-clean
help: ; @grep -E "^[a-z-]+:" Makefile | sed "s/:.*//"
venv: ; test -x $(VENV)/bin/python || python3 -m venv $(VENV)
install: venv ; $(PY) -m pip install -e ".[dev]"
lint: ; $(PY) -m ruff check .
format: ; $(PY) -m ruff format .
typecheck: ; $(PY) -m mypy aero scripts tests
test: ; $(PY) -m pytest -m "not sitl"
test-sitl: ; $(PY) -m pytest -m sitl
check-env: ; $(PY) scripts/check_env.py
# Last resort after a run was hard-killed (SIGKILL/OOM): PX4 and gz are started
# in their own session, so nothing else reaps them.
sitl-clean: ; -pkill -f "px4_sitl_default/bin/px4"; -pkill -x gz; -pkill -f "gz sim"
