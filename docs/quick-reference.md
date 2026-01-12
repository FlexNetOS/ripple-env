---
title: Quick Reference
description: Quick reference for common commands and configuration
tags:
  - reference
  - commands
---

# Quick Reference

A quick reference guide for the most commonly used commands and configuration.

## Shell Commands

### Development Shell

```bash
# Enter development shell (default)
nix develop

# Shell variants
nix develop .#full      # All features
nix develop .#cuda      # CUDA-enabled
nix develop .#humble    # ROS2 Humble (explicit)
nix develop .#iron      # ROS2 Iron
nix develop .#rolling   # ROS2 Rolling (dev)
```

### Building

```bash
# Build all packages
cb                          # alias for colcon build --symlink-install

# Build specific package
colcon build --packages-select my_package

# Clean build
rm -rf build install log && cb
```

### Testing

```bash
# Run all tests
ct                          # alias for colcon test

# Test specific package
colcon test --packages-select my_package

# View test results
colcon test-result --verbose
```

### ROS2 Commands

```bash
# Source workspace
source install/setup.bash

# List packages
ros2 pkg list

# Run a node
ros2 run package_name node_name

# Launch
ros2 launch package_name launch_file.py
```

## Pixi Commands

```bash
# Install dependencies
pixi install

# Add package
pixi add package-name

# Use specific environment
pixi run -e cuda python script.py
pixi run -e aios python script.py

# Update lock file
pixi update
```

## Service Management

### AI Services

```bash
# LocalAI
localai start
localai stop

# AGiXT
agixt up
agixt down

# MindsDB
mindsdb start
```

### Infrastructure

```bash
# Temporal
temporal-ctl start

# Start all Docker services
docker compose up -d

# View logs
docker compose logs -f
```

## Security

```bash
# Generate SBOM
sbom-generate

# Security audit
sbom-audit

# mTLS setup
step-ca-init
```

## Nix Commands

```bash
# Check flake
nix flake check

# Show outputs
nix flake show

# Update inputs
nix flake update

# Build specific output
nix build .#package-name
```

## Git Workflow

```bash
# Commit with conventional format
git commit -m "feat: add new feature"
git commit -m "fix: resolve bug"
git commit -m "docs: update documentation"

# Create PR branch
git checkout -b feature/my-feature
git push -u origin feature/my-feature
```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DISTRO` | ROS2 distribution | `humble` |
| `COLCON_HOME` | Colcon config dir | `~/.colcon` |
| `LOCALAI_PORT` | LocalAI port | `8080` |
| `TEMPORAL_HOST` | Temporal address | `localhost:7233` |
| `NATS_URL` | NATS server URL | `nats://localhost:4222` |

## Ports Reference

| Service | Port | Protocol |
|---------|------|----------|
| LocalAI | 8080 | HTTP |
| AGiXT | 8501 | HTTP |
| MindsDB | 47334 | HTTP |
| Prometheus | 9090 | HTTP |
| Grafana | 3000 | HTTP |
| Temporal | 7233 | gRPC |
| NATS | 4222 | TCP |
| Neo4j Bolt | 7687 | Bolt |
| Neo4j HTTP | 7474 | HTTP |
