# =============================================================================
# Makefile for UAV-UGV Cooperative Localization
# =============================================================================
# Common tasks for C++ simulation + Python analysis/plotting tools
# Usage: make <target> [VARIABLE=...]

.PHONY: help configure build run clean \
        python-venv python-install python-plot python-clean \
        all dev-setup

# ----------------------------- Configuration -----------------------------
BUILD_DIR      ?= build
OUTPUT_DIR     ?= simulation_output/my_run   # override with: make python-plot OUTPUT_DIR=...
VENV_DIR       ?= venv
PYTHON         ?= python

# Cross-platform path to Python inside the venv
ifeq ($(OS),Windows_NT)
    VENV_PYTHON := $(VENV_DIR)\Scripts\python.exe
else
    VENV_PYTHON := $(VENV_DIR)/bin/python
endif

EXECUTABLE     ?= $(BUILD_DIR)/uav_ugv_coop_localization$(if $(filter Windows_NT,$(OS)),.exe,)

# ----------------------------- Help -------------------------------------
help:  ## Show this help message
	@echo "=== UAV-UGV Coop Localization Makefile ==="
	@echo "Available targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "Examples:"
	@echo "  make dev-setup              # full dev environment"
	@echo "  make build                  # build C++ executable"
	@echo "  make run                    # run simulation (creates output)"
	@echo "  make python-plot OUTPUT_DIR=simulation_output/my_run"
	@echo "  make -j                     # parallel build (faster)"

# ----------------------------- C++ Targets -----------------------------
configure:  ## Run CMake configure (Ninja + Release)
	cmake -B $(BUILD_DIR) -S . -G Ninja -DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_CXX_COMPILER=$(shell which g++-15 2>/dev/null || echo g++) \
		-DCMAKE_C_COMPILER=$(shell which gcc-15 2>/dev/null || echo gcc)

build: configure  ## Build the C++ executable
	cmake --build $(BUILD_DIR) --config Release

run: build  ## Build + run the simulation
	@echo "Running simulation → outputs will appear in $(OUTPUT_DIR)/"
	$(EXECUTABLE)

clean:  ## Remove build artifacts (C++ only)
	rm -rf $(BUILD_DIR)

# ----------------------------- Python Targets -----------------------------
python-venv:  ## Create Python virtual environment
	$(PYTHON) -m venv $(VENV_DIR)

python-install: python-venv  ## Install Python package in editable mode + dev tools
	$(VENV_PYTHON) -m pip install --upgrade pip
	$(VENV_PYTHON) -m pip install -e .
	@echo "Python package installed successfully"

python-plot: python-install  ## Run Plotly visualization (requires OUTPUT_DIR)
	$(VENV_PYTHON) -m analysis.cli --output-dir $(OUTPUT_DIR)

python-clean:  ## Remove Python cache and build artifacts
	rm -rf $(VENV_DIR) *.egg-info dist/ build/ __pycache__ analysis/__pycache__ .pytest_cache

# ----------------------------- Convenience Targets -----------------------------
dev-setup: python-venv python-install  ## One-command full dev setup (C++ + Python)
	@echo "Development environment ready!"
	@echo "Next steps:"
	@echo "  make build     # build C++"
	@echo "  ./$(EXECUTABLE)  # or just 'make run'"
	@echo "  make python-plot OUTPUT_DIR=simulation_output/<your-subdir>"

all: build python-install  ## Build C++ + install Python package

# Default target
.DEFAULT_GOAL := help