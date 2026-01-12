---
title: Shell Commands
description: Available shell commands and aliases
tags:
  - shell
  - commands
  - development
---

# Shell Commands

ripple-env provides numerous shell commands and aliases to streamline development.

## Build Commands

| Command | Description | Full Command |
|---------|-------------|--------------|
| `cb` | Build all packages | `colcon build --symlink-install` |
| `ct` | Run tests | `colcon test` |
| `ctr` | Test results | `colcon test-result --verbose` |

## ROS2 Commands

| Command | Description |
|---------|-------------|
| `ros2 run` | Run a node |
| `ros2 launch` | Launch a launch file |
| `ros2 topic list` | List topics |
| `ros2 service list` | List services |
| `ros2 node list` | List nodes |

## Service Commands

| Command | Description |
|---------|-------------|
| `localai start` | Start LocalAI server |
| `localai stop` | Stop LocalAI server |
| `agixt up` | Start AGiXT |
| `agixt down` | Stop AGiXT |
| `temporal-ctl start` | Start Temporal |

## Security Commands

| Command | Description |
|---------|-------------|
| `sbom-generate` | Generate SBOM |
| `sbom-audit` | Run security audit |
| `step-ca-init` | Initialize step-ca |

## Nix Commands

| Command | Description |
|---------|-------------|
| `nix develop` | Enter dev shell |
| `nix build` | Build a derivation |
| `nix flake check` | Check flake |
| `nix flake show` | Show flake outputs |

## Pixi Commands

| Command | Description |
|---------|-------------|
| `pixi install` | Install dependencies |
| `pixi add` | Add a package |
| `pixi run` | Run in environment |
| `pixi update` | Update lock file |

## Aliases

These aliases are available in the development shell:

```bash
# Colcon aliases
alias cb='colcon build --symlink-install'
alias ct='colcon test'
alias ctr='colcon test-result --verbose'

# Git aliases
alias gs='git status'
alias gd='git diff'
alias ga='git add'
alias gc='git commit'
```

## Creating Custom Commands

Add custom commands to your shell by editing `~/.bashrc` or `~/.zshrc`, or add them to the Nix flake for project-wide availability.
