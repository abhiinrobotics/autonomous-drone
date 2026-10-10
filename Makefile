# ROS 2 Humble is installed system-wide and exports PYTHONPATH=/opt/ros/humble/...;
# its pytest entry-point plugins then load into our runs and crash on a missing
# dep (lark). Every target runs with PYTHONPATH cleared and with the repo venv's
# interpreter, so results do not depend on whether the venv is activated.
VENV ?= .venv
SEED ?= 1
PY := env -u PYTHONPATH $(VENV)/bin/python

.PHONY: help venv install gz-link lint format typecheck test test-sitl check-env sitl-clean make-world capture-part1
help: ; @grep -E "^[a-z-]+:" Makefile | sed "s/:.*//"
venv: ; test -x $(VENV)/bin/python || python3 -m venv $(VENV)
install: venv gz-link ; $(PY) -m pip install -e ".[dev]"
# Explicit source trees: the repo root can hold scratch files that are not
# ours to police (and mypy uses the same list).
SRC := aero scripts tests
lint: ; $(PY) -m ruff check $(SRC)
format: ; $(PY) -m ruff format $(SRC)
typecheck: ; $(PY) -m mypy $(SRC)
test: ; $(PY) -m pytest -m "not sitl"
test-sitl: ; $(PY) -m pytest -m sitl
check-env: ; $(PY) scripts/check_env.py
make-world: ; $(PY) scripts/make_world.py --seed $(SEED)
capture-part1: ; $(PY) scripts/capture_part1.py --seed $(SEED)
# Gazebo's Python bindings are an apt package in the system dist-packages. Link
# just that one package into the venv (not all of dist-packages, which would
# re-import the ROS tree this Makefile deliberately hides).
gz-link: venv
	@$(VENV)/bin/python -c "import gz.transport13" 2>/dev/null && echo "gz bindings already linked" || \
	ln -sfn /usr/lib/python3/dist-packages/gz "$$($(VENV)/bin/python -c 'import site;print(site.getsitepackages()[0])')/gz"

# Last resort after a run was hard-killed (SIGKILL/OOM): PX4 and gz are started
# in their own session, so nothing else reaps them. Match on the executable name
# (-x), never the command line: a -f pattern also matches the shell running it.
sitl-clean: ; -pkill -x px4; -pkill -x gz; -pkill -x ruby
