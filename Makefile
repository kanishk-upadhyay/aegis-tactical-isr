# ==============================================================================
# AEGIS-TACTICAL-ISR: Developer & Operator Makefile
# Indian Army Internship Programme (AICTE) // ADB & DG AAD
# ==============================================================================

PYTHON ?= python3
VENV ?= .venv
VENV_PYTHON ?= $(VENV)/bin/python
VENV_PYTEST ?= $(VENV)/bin/pytest

.PHONY: help venv install test benchmark demo serve docker-build docker-run clean

help:
	@echo "================================================================================"
	@echo "   AEGIS-TACTICAL-ISR COMMAND TARGETS"
	@echo "================================================================================"
	@echo "  make install     Create .venv and install all dependencies"
	@echo "  make test        Run complete pytest test suite (19 unit/integration tests)"
	@echo "  make benchmark   Run quantitative mission benchmark scorecard (ADB/AAD)"
	@echo "  make demo        Run one-click terminal simulation (30 frames)"
	@echo "  make serve       Start real-time FastAPI C2 HUD backend on http://localhost:8000"
	@echo "  make docker-build Build security-hardened edge deployment container"
	@echo "  make docker-run   Run containerized C2 server on port 8000"
	@echo "  make clean       Remove temporary Python cache and build files"
	@echo "================================================================================"

venv:
	@test -d $(VENV) || $(PYTHON) -m venv $(VENV)

install: venv
	$(VENV_PYTHON) -m pip install --upgrade pip
	$(VENV_PYTHON) -m pip install -r requirements.txt

test:
	$(VENV_PYTEST) tests/ -v --durations=10

benchmark:
	$(VENV_PYTHON) scripts/benchmark_scenarios.py --export-markdown reports/benchmark_report.md

demo:
	$(VENV_PYTHON) scripts/demo_simulation.py --frames 30

serve:
	$(VENV_PYTHON) scripts/demo_simulation.py --serve

docker-build:
	docker build -t aegis-tactical-isr:latest .

docker-run:
	docker run -p 8000:8000 aegis-tactical-isr:latest

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	find . -type f -name "*.pyo" -delete
