# ROS2 Humble Environment Documentation

Welcome to the comprehensive documentation for ripple-env, a cross-platform ROS2 development environment built with Nix and Pixi.

## Quick Navigation

| Getting Started | Development | Deployment |
|----------------|-------------|------------|
| [Installation](GETTING_STARTED.md) | [ROS2 Commands](PYTHON-ENVIRONMENTS.md) | [Docker Services](docker-compose.md) |
| [Troubleshooting](TROUBLESHOOTING.md) | [AI Integration](LOCALAI-MODELS.md) | [Edge Deployment](EDGE_DEPLOYMENT.md) |
| [ARM64 Setup](arm64-deployment.md) | [Testing](../test/) | [WSL2 Build](WSL2_BUILD_PIPELINE.md) |

---

## Documentation Structure

### Getting Started

Start here if you're new to the project:

- **[GETTING_STARTED.md](GETTING_STARTED.md)** - Installation and first steps
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions
- **[arm64-deployment.md](arm64-deployment.md)** - ARM64 platform guide (Raspberry Pi, Jetson)

### Core Development

Day-to-day development guides:

| Document | Description |
|----------|-------------|
| [PYTHON-ENVIRONMENTS.md](PYTHON-ENVIRONMENTS.md) | Python environment management (Nix vs Pixi) |
| [ROS2_STATE_PACKAGES.md](ROS2_STATE_PACKAGES.md) | ROS2 state management packages |
| [CONFLICTS.md](CONFLICTS.md) | Dependency conflict resolution |

### AI & Machine Learning

AI integration and inference:

| Document | Description |
|----------|-------------|
| [LOCALAI-MODELS.md](LOCALAI-MODELS.md) | LocalAI model configuration |
| [INFERENCE_SETUP.md](INFERENCE_SETUP.md) | LLM inference setup |
| [MINDSDB_QUICKSTART.md](MINDSDB_QUICKSTART.md) | MindsDB ML platform |
| [GENAI_TOOLBOX_INSTALL.md](GENAI_TOOLBOX_INSTALL.md) | GenAI toolbox |

### Infrastructure

Service deployment and management:

| Document | Description |
|----------|-------------|
| [docker-compose.md](docker-compose.md) | Docker service architecture |
| [OBSERVABILITY-QUICK-START.md](OBSERVABILITY-QUICK-START.md) | Prometheus, Grafana setup |
| [DISTRIBUTED-TRACING.md](DISTRIBUTED-TRACING.md) | Distributed tracing |

### Security

Security configuration and best practices:

| Document | Description |
|----------|-------------|
| [MTLS_SETUP.md](MTLS_SETUP.md) | Mutual TLS configuration |
| [MTLS-IMPLEMENTATION-CHECKLIST.md](MTLS-IMPLEMENTATION-CHECKLIST.md) | mTLS checklist |
| [../SECURITY.md](../SECURITY.md) | Security policy |

### Real-Time & Hardware

Real-time systems and hardware integration:

| Document | Description |
|----------|-------------|
| [realtime-setup.md](realtime-setup.md) | PREEMPT_RT kernel setup |
| Hardware Interfaces | GPIO, CAN, serial in nix/packages/hardware.nix |

### Architecture

System design and decisions:

| Document | Description |
|----------|-------------|
| [NIX_FLAKE_MODULARIZATION.md](NIX_FLAKE_MODULARIZATION.md) | Flake architecture |
| [MANUS_ARIA_ORCHESTRATOR.md](MANUS_ARIA_ORCHESTRATOR.md) | ARIA orchestrator |
| [adr/](adr/) | Architecture Decision Records |

### Platform-Specific

Platform-specific guides:

| Platform | Document |
|----------|----------|
| Windows/WSL2 | [WSL2_BUILD_PIPELINE.md](WSL2_BUILD_PIPELINE.md) |
| Containers | [KATA_CONTAINERS_INSTALL.md](KATA_CONTAINERS_INSTALL.md) |
| Sandbox | [SANDBOX_RUNTIME_INSTALL.md](SANDBOX_RUNTIME_INSTALL.md) |

---

## Architecture Diagrams

### Component Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     ripple-env                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│  │   Nix Flake  │  │    Pixi      │  │   Docker     │         │
│  │   (System)   │  │  (Python)    │  │  (Services)  │         │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘         │
│         │                 │                 │                  │
│         ▼                 ▼                 ▼                  │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │                   Development Shell                       │ │
│  │  • ROS2 Humble    • AI Tools      • Infrastructure       │ │
│  │  • Build Tools    • Security      • Observability        │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Service Architecture

See [docker-compose.md](docker-compose.md) for detailed service diagrams.

---

## Quick Reference

### Common Commands

```bash
# Enter development shell
nix develop

# Build ROS2 packages
cb    # alias for: colcon build --symlink-install

# Run tests
ct    # alias for: colcon test

# Start services
agixt up        # Start AGiXT
localai start   # Start LocalAI
temporal-ctl start  # Start Temporal

# Security
sbom-generate   # Generate SBOM
sbom-audit      # Full security audit
```

### Shell Options

```bash
nix develop           # Default shell
nix develop .#full    # Full-featured shell
nix develop .#cuda    # CUDA-enabled shell
nix develop .#humble  # ROS2 Humble (explicit)
nix develop .#iron    # ROS2 Iron
nix develop .#rolling # ROS2 Rolling (dev)
```

---

## External Resources

### ROS2
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Design](https://design.ros2.org/)

### Nix
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Nix Flakes](https://nixos.wiki/wiki/Flakes)
- [Home Manager Options](https://home-manager-options.extranix.com/)

### Pixi
- [Pixi Documentation](https://pixi.sh/)
- [RoboStack](https://robostack.github.io/)

---

## Contributing

- Read [CONTRIBUTING.md](../CONTRIBUTING.md) before submitting changes
- Follow the [commit message format](../CLAUDE.md#commit-message-format)
- Update documentation when adding features

## Support

- [GitHub Issues](https://github.com/FlexNetOS/ripple-env/issues)
- [GitHub Discussions](https://github.com/FlexNetOS/ripple-env/discussions)
- See [SECURITY.md](../SECURITY.md) for security issues
