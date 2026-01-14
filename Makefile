# Ripple Environment Makefile
# Convenience targets for common validation/build tasks.
#
# Notes:
# - This repo is primarily Nix/Pixi/Bash driven.
# - The Makefile is intentionally "portable by default": it skips Nix-only
#   steps when `nix` isn't available (e.g., Windows host outside WSL).

# Use Git-for-Windows bash when available (avoid PATH collisions with non-exe
# files named 'bash' and avoid WSL path translation quirks).
SHELL := bash
ifeq ($(OS),Windows_NT)
	GIT_BASH := C:/PROGRA~1/Git/usr/bin/bash.exe
	ifneq ($(wildcard $(GIT_BASH)),)
		SHELL := $(GIT_BASH)
	endif
endif

.SHELLFLAGS := -l -e -o pipefail -c

.PHONY: help doctor install build test test-e2e deploy security monitor dev clean

COMPOSE :=
ifneq ($(shell command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1 && echo yes || true),)
COMPOSE := docker compose
else ifneq ($(shell command -v docker-compose >/dev/null 2>&1 && echo yes || true),)
COMPOSE := docker-compose
endif

help:
	@echo "Ripple Environment Build Automation"
	@echo "==================================="
	@echo "Available targets:"
	@echo "  doctor      - Show toolchain versions"
	@echo "  install     - Install JS deps (npm)"
	@echo "  build       - Build JS artifacts (swc)"
	@echo "  test        - Deterministic local checks (no cluster required)"
	@echo "  test-e2e    - Run full e2e validation script"
	@echo "  security    - Run security audit script"
	@echo "  monitor     - Start observability stack (Docker)"
	@echo "  dev         - Start default docker compose stack (if present)"
	@echo "  deploy      - Run deploy script"
	@echo "  clean       - Remove local build artifacts"

doctor:
	@echo "PWD=$$(pwd)"
	@command -v node >/dev/null 2>&1 && node --version || echo "node: (not found)"
	@command -v npm >/dev/null 2>&1 && npm --version || echo "npm: (not found)"
	@command -v cargo >/dev/null 2>&1 && cargo --version || echo "cargo: (not found)"
	@command -v python >/dev/null 2>&1 && python --version || true
	@command -v python3 >/dev/null 2>&1 && python3 --version || true
	@command -v docker >/dev/null 2>&1 && docker --version || echo "docker: (not found)"
	@if [ -n "$(COMPOSE)" ]; then $(COMPOSE) version || true; else echo "docker compose: (not found)"; fi
	@command -v nix >/dev/null 2>&1 && nix --version || echo "nix: (not found)"

install:
	@echo "Installing JS dependencies (npm)…"
	@if [ -f package-lock.json ]; then npm ci; else npm install; fi

build:
	@echo "Building JS artifacts (swc)…"
	@npm run -s swc:build
	@echo "NOTE: Nix builds are performed in CI/WSL via nix flake check/build."

test:
	@echo "Running deterministic local checks…"
	@bash ./scripts/validate-configs.sh
	@npm run -s swc:build
	@npm run -s agents:test
	@if [ -d rust ] && command -v cargo >/dev/null 2>&1; then cargo test --manifest-path rust/Cargo.toml; else echo "Skipping cargo tests (rust/ missing or cargo not found)"; fi
	@if command -v nix >/dev/null 2>&1; then nix flake check; else echo "Skipping nix flake check (nix not found)"; fi

test-e2e:
	@bash ./scripts/validate-e2e.sh

deploy:
	@bash ./scripts/deploy.sh

security:
	@bash ./scripts/security-audit.sh

monitor:
	@echo "Starting observability stack…"
	@if [ -z "$(COMPOSE)" ]; then echo "WARNING: docker compose not found; skipping"; exit 0; fi
	@$(COMPOSE) -f docker/docker-compose.observability.yml up -d

dev:
	@echo "Starting development docker compose stack…"
	@if [ -z "$(COMPOSE)" ]; then echo "WARNING: docker compose not found; skipping"; exit 0; fi
	@if [ -f docker/docker-compose.yml ]; then $(COMPOSE) -f docker/docker-compose.yml up -d; \
	elif [ -f docker-compose.yml ]; then $(COMPOSE) -f docker-compose.yml up -d; \
	else echo "No docker-compose.yml found (checked docker/docker-compose.yml and docker-compose.yml)"; fi

clean:
	@echo "Cleaning local build artifacts…"
	@rm -rf dist .tmp .tmp2 tmp testResults.xml 2>/dev/null || true
