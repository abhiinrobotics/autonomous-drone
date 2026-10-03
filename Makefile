.PHONY: help install lint format typecheck test test-sitl
help: ; @grep -E "^[a-z-]+:" Makefile | sed "s/:.*//"
install: ; pip install -e ".[dev]"
lint: ; ruff check .
format: ; ruff format .
typecheck: ; mypy aero
test: ; pytest -m "not sitl"
test-sitl: ; pytest -m sitl
