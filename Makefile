# FlexNetOS Makefile
# Production automation and deployment

.PHONY: help install build test deploy clean

# Default target
help:
	@echo "FlexNetOS Build Automation"
	@echo "=========================="
	@echo "Available targets:"
	@echo "  help         - Show this help message"
	@echo "  install      - Install all dependencies"
	@echo "  build        - Build all services"
	@echo "  test         - Run tests and validation"
	@echo "  deploy       - Deploy to production"
	@echo "  clean        - Clean build artifacts"
	@echo "  security     - Run security audit"
	@echo "  monitor      - Start monitoring stack"

# Install dependencies
install:
	@echo "Installing FlexNetOS dependencies..."
	nix develop
	docker-compose pull

# Build all services
build:
	@echo "Building FlexNetOS services..."
	nix build .#default
	docker-compose build

# Run tests
test:
	@echo "Running FlexNetOS tests..."
	nix flake check
	cargo test --manifest-path rust/Cargo.toml
	./scripts/validate-configs.sh

# Deploy to production
deploy:
	@echo "Deploying FlexNetOS to production..."
	./scripts/deploy.sh

# Clean build artifacts
clean:
	@echo "Cleaning FlexNetOS build artifacts..."
	nix-collect-garbage -d
	docker system prune -f

# Security audit
security:
	@echo "Running FlexNetOS security audit..."
	./scripts/security-audit.sh

# Start monitoring
monitor:
	@echo "Starting FlexNetOS monitoring..."
	docker-compose -f docker/prometheus.yml up -d
	docker-compose -f docker/grafana.yml up -d

# Development environment
dev:
	@echo "Starting FlexNetOS development environment..."
	docker-compose up -d
