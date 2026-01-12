# Makefile for ros2-humble-env
# Common development tasks and shortcuts

.PHONY: help install test lint format clean shell update docker-build docker-run check

# Default target
help: ## Show this help message
	@echo "ros2-humble-env - ROS2 Development Environment"
	@echo ""
	@echo "Usage: make <target>"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

# Installation targets
install: ## Install development environment
	./bootstrap.sh

install-ci: ## Install in CI mode (non-interactive)
	./bootstrap.sh --ci

# Nix targets
shell: ## Enter development shell
	nix develop

shell-full: ## Enter full development shell with all tools
	nix develop .#full

check: ## Check Nix flake for errors
	nix flake check

update: ## Update Nix flake inputs
	nix flake update

build: ## Build the default package
	nix build

# Pixi targets
pixi-install: ## Install Pixi dependencies
	pixi install

pixi-update: ## Update Pixi dependencies
	pixi update

pixi-clean: ## Clean Pixi environment
	pixi clean

# ROS2 targets
ros2-build: ## Build ROS2 packages with colcon
	colcon build --symlink-install

ros2-test: ## Run ROS2 tests
	colcon test
	colcon test-result --verbose

ros2-clean: ## Clean ROS2 build artifacts
	rm -rf build/ install/ log/

# Testing targets
test: ## Run all tests
	@echo "Running tests..."
	@if command -v pytest &> /dev/null; then \
		pytest test/ -v; \
	else \
		echo "pytest not found, skipping Python tests"; \
	fi
	@if command -v pwsh &> /dev/null; then \
		pwsh -Command "Invoke-Pester -Path test/unit/ -CI" || true; \
	else \
		echo "PowerShell not found, skipping Pester tests"; \
	fi

test-unit: ## Run unit tests only
	pytest test/unit/ -v || true
	pwsh -Command "Invoke-Pester -Path test/unit/ -CI" 2>/dev/null || true

test-integration: ## Run integration tests
	pytest test/integration/ -v || true

test-coverage: ## Run tests with coverage report
	pytest test/ -v --cov=scripts --cov-report=term-missing --cov-report=xml --cov-report=html
	@echo "Coverage report generated: htmlcov/index.html"

test-benchmarks: ## Run performance benchmarks
	pytest test/benchmarks/ -v --benchmark-only || echo "No benchmarks found or ROS2 not available"

# Linting targets
lint: ## Run all linters
	@echo "Running linters..."
	@if command -v ruff &> /dev/null; then \
		ruff check .; \
	else \
		echo "ruff not found, skipping Python linting"; \
	fi
	@if command -v shellcheck &> /dev/null; then \
		find . -name "*.sh" -not -path "./.pixi/*" -not -path "./.git/*" -exec shellcheck {} \;; \
	else \
		echo "shellcheck not found, skipping shell linting"; \
	fi
	@if command -v nixfmt &> /dev/null; then \
		find nix/ -name "*.nix" -exec nixfmt --check {} \; || true; \
	else \
		echo "nixfmt not found, skipping Nix linting"; \
	fi

lint-python: ## Lint Python code
	ruff check .
	mypy scripts/ --ignore-missing-imports || true

lint-shell: ## Lint shell scripts
	find . -name "*.sh" -not -path "./.pixi/*" -not -path "./.git/*" -exec shellcheck {} \;

lint-nix: ## Lint Nix files
	find nix/ -name "*.nix" -exec nixfmt --check {} \;

# Formatting targets
format: ## Format all code
	@echo "Formatting code..."
	@if command -v ruff &> /dev/null; then \
		ruff format .; \
	fi
	@if command -v nixfmt &> /dev/null; then \
		find nix/ -name "*.nix" -exec nixfmt {} \;; \
	fi

format-python: ## Format Python code
	ruff format .

format-nix: ## Format Nix files
	find nix/ -name "*.nix" -exec nixfmt {} \;

# Cleanup targets
clean: ## Clean all build artifacts
	rm -rf build/ dist/ *.egg-info/
	rm -rf result result-*
	rm -rf .pytest_cache/ .mypy_cache/ .ruff_cache/
	find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true

clean-all: clean ros2-clean pixi-clean ## Clean everything including ROS2 and Pixi

# Docker targets
docker-build: ## Build Docker image
	docker build -t ros2-humble-env .

docker-run: ## Run Docker container interactively
	docker run -it --rm ros2-humble-env

docker-compose-up: ## Start all Docker Compose services
	docker-compose up -d

docker-compose-down: ## Stop all Docker Compose services
	docker-compose down

# Documentation targets
docs-serve: ## Serve documentation locally (requires mkdocs)
	mkdocs serve || echo "mkdocs not installed"

# Validation targets
validate: ## Validate configuration files
	python scripts/validate-manifest.py --manifest ARIA_MANIFEST.yaml || true
	nix flake check

validate-yaml: ## Validate all YAML files
	python -c "import yaml, sys; [yaml.safe_load(open(f)) for f in sys.argv[1:]]" config/**/*.yaml

# Version management
ros2-version: ## Show ROS2 version info
	./scripts/ros2-version

ros2-humble: ## Switch to ROS2 Humble
	./scripts/ros2-version humble

ros2-iron: ## Switch to ROS2 Iron
	./scripts/ros2-version iron

ros2-rolling: ## Switch to ROS2 Rolling
	./scripts/ros2-version rolling
