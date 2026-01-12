---
title: Reference
description: Technical reference documentation
tags:
  - reference
  - technical
---

# Reference

Technical reference documentation for ripple-env configuration and requirements.

## Quick Links

| Reference | Description |
|-----------|-------------|
| [Network Requirements](network.md) | Network dependencies and firewall rules |
| [AI Resources](ai-resources.md) | AI service resource requirements |
| [Channel Strategy](channel-strategy.md) | Conda channel priority configuration |
| [Dependency Conflicts](conflicts.md) | Resolving version conflicts |

## Documentation

<div class="grid cards" markdown>

-   :material-network:{ .lg .middle } __Network Requirements__

    ---

    Network dependencies, ports, and firewall configuration

    [:octicons-arrow-right-24: Network](network.md)

-   :material-memory:{ .lg .middle } __AI Resources__

    ---

    CPU, memory, and GPU requirements for AI services

    [:octicons-arrow-right-24: Resources](ai-resources.md)

-   :material-package-variant:{ .lg .middle } __Channel Strategy__

    ---

    Conda/Pixi channel priority and resolution

    [:octicons-arrow-right-24: Channels](channel-strategy.md)

-   :material-alert-circle:{ .lg .middle } __Dependency Conflicts__

    ---

    Resolving version conflicts and incompatibilities

    [:octicons-arrow-right-24: Conflicts](conflicts.md)

</div>

## Configuration Files

| File | Purpose |
|------|---------|
| `flake.nix` | Main Nix flake configuration |
| `pixi.toml` | Pixi/Conda package definitions |
| `.envrc` | direnv configuration |
| `docker-compose.yml` | Docker service definitions |
| `mkdocs.yml` | Documentation configuration |

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ROS_DISTRO` | ROS2 distribution | `humble` |
| `COLCON_HOME` | Colcon configuration directory | `~/.colcon` |
| `LOCALAI_PORT` | LocalAI server port | `8080` |
| `TEMPORAL_HOST` | Temporal server address | `localhost:7233` |
