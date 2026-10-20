# Top-level entry point. `make test` runs everything.
#
#   make test        C++ unit tests + host tests (golden vectors, integration)
#   make test-cpp    just the firmware core (no Python needed)
#   make test-host   just the host side (compiles the C++ core)
#   make demo        the no-hardware message-exchange demonstration
#   make autonomy    the Task 4 mock-autonomy mission against the simulator
#   make clean
#
# Needs a C++17 compiler and Python 3.9+. Nothing to install: the host tests
# compile the real firmware sources rather than mocking them.

PYTHON ?= python3

.PHONY: test test-cpp test-host demo autonomy clean

test: test-cpp test-host
	@echo
	@echo "================================================"
	@echo " ALL TESTS PASSED"
	@echo "================================================"

test-cpp:
	@echo "=== C++ firmware core ==="
	@$(MAKE) --no-print-directory -C tests/cpp test

test-host:
	@echo
	@echo "=== host side (Jetson) ==="
	@$(PYTHON) -m unittest discover -s tests/host -v

demo:
	@$(PYTHON) host/demo.py

autonomy:
	@$(PYTHON) host/autonomy_sim.py --mock

clean:
	@$(MAKE) --no-print-directory -C tests/cpp clean
	@find . -name __pycache__ -type d -prune -exec rm -rf {} + 2>/dev/null || true
	@echo "cleaned"
