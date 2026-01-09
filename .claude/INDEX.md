# Documentation Index

## Quick Navigation

| Document | Description |
|----------|-------------|
| [README](../README.md) | Project overview and getting started |
| [AGENT](./AGENT.md) | Agentic system architecture and capabilities |
| [CLAUDE](./CLAUDE.md) | Claude Code specific instructions |
| [RULES](./RULES.md) | Guidelines and coding standards |
| [SKILL](./SKILL.md) | Available skills and tool reference |

### Configuration

| File | Description |
|------|-------------|
| [settings.json](./settings.json) | Permissions, hooks, and context configuration |
| [.mcp.json](../.mcp.json) | MCP server configuration |
| [.editorconfig](../.editorconfig) | Editor consistency settings |
| [schema.json](./config/schema.json) | Configuration validation schema |

### Skills (Structured)

| Skill | Description |
|-------|-------------|
| [ROS2 Development](./skills/ros2-development/README.md) | Building, testing, and running ROS2 packages |
| [DevOps](./skills/devops/README.md) | CI/CD, GitHub workflows, automation |
| [Nix Environment](./skills/nix-environment/README.md) | Flakes, home-manager, environment management |
| [AI Assistants](./skills/ai-assistants/README.md) | AI-powered development tools (aichat, aider) |
| [Distributed Systems](./skills/distributed-systems/README.md) | NATS messaging, Temporal workflows |
| [Rust Tooling](./skills/rust-tooling/README.md) | PyO3 bindings, sqlx database access |
| [Observability](./skills/observability/README.md) | Prometheus, OpenTelemetry, monitoring |
| [LLM Evaluation](./skills/llm-evaluation/README.md) | promptfoo, TruLens, LLM testing |

### Agent Roles

#### Core Domain Agents

| Agent | Description |
|-------|-------------|
| [Coordinator](./agents/coordinator.md) | Multi-agent task routing and orchestration |
| [Robotics Agent](./agents/robotics-agent.md) | ROS2 development specialist |
| [DevOps Agent](./agents/devops-agent.md) | CI/CD and infrastructure specialist |
| [Nix Agent](./agents/nix-agent.md) | Environment configuration specialist |

#### Architecture & Analysis Agents

| Agent | Description |
|-------|-------------|
| [Architect Agent](./agents/architect-agent.md) | Full-stack architecture and framework design |
| [Pre-Verify Agent](./agents/pre-verify-agent.md) | Pre-implementation verification and validation |
| [Cross-Analysis Agent](./agents/cross-analysis-agent.md) | Codebase analysis and pattern discovery |

### Slash Commands

| Command | Description |
|---------|-------------|
| [/build](./commands/build.md) | Build ROS2 packages with colcon |
| [/test](./commands/test.md) | Run package tests |
| [/flake-check](./commands/flake-check.md) | Validate Nix flake |
| [/update-deps](./commands/update-deps.md) | Update project dependencies |

## Project Structure

```
ros2-humble-env/
├── .claude/                          # Agent configuration
│   ├── INDEX.md                      # This file - documentation navigation
│   ├── AGENT.md                      # Agent system architecture
│   ├── CLAUDE.md                     # Claude Code instructions
│   ├── RULES.md                      # Rules and guidelines
│   ├── SKILL.md                      # Skills reference (summary)
│   ├── settings.json                 # Permissions, hooks, context config
│   ├── settings.local.json           # Local overrides (gitignored)
│   ├── skills/                       # Structured skill definitions
│   │   ├── ros2-development/
│   │   │   └── README.md
│   │   ├── devops/
│   │   │   └── README.md
│   │   ├── nix-environment/
│   │   │   └── README.md
│   │   ├── ai-assistants/
│   │   │   └── README.md
│   │   ├── distributed-systems/
│   │   │   └── README.md
│   │   ├── rust-tooling/
│   │   │   └── README.md
│   │   ├── observability/
│   │   │   └── README.md
│   │   └── llm-evaluation/
│   │       └── README.md
│   ├── commands/                     # Slash command definitions
│   │   ├── build.md
│   │   ├── test.md
│   │   ├── flake-check.md
│   │   └── update-deps.md
│   ├── agents/                       # Agent role definitions
│   │   ├── coordinator.md            # Multi-agent orchestration
│   │   ├── robotics-agent.md         # ROS2 specialist
│   │   ├── devops-agent.md           # CI/CD specialist
│   │   ├── nix-agent.md              # Environment specialist
│   │   ├── architect-agent.md        # Full-stack architecture
│   │   ├── pre-verify-agent.md       # Pre-implementation validation
│   │   └── cross-analysis-agent.md   # Codebase analysis
│   ├── hooks/                        # Hook scripts
│   │   └── session-start.sh          # Workspace initialization
│   └── config/
│       └── schema.json               # Configuration validation schema
│
├── .mcp.json                         # MCP server configuration
├── .editorconfig                     # Editor consistency settings
│
├── .github/
│   ├── workflows/
│   │   └── bootstrap-test.yml        # CI workflow
│   └── docs/
│       └── self-hosted-runner-setup.md
│
├── lib/                              # Nix library functions
│   ├── default.nix                   # Utility functions
│   └── system.nix                    # System builder helpers
│
├── modules/                          # Home-manager configuration modules
│   ├── common/                       # Cross-platform configurations
│   │   ├── default.nix               # Module aggregator
│   │   ├── direnv.nix                # direnv configuration
│   │   ├── git.nix                   # Git configuration
│   │   ├── packages.nix              # Common packages and aliases
│   │   ├── ai/                       # AI-powered development tools
│   │   │   ├── default.nix           # AI module aggregator
│   │   │   ├── aichat.nix            # Provider-agnostic AI CLI
│   │   │   └── aider.nix             # AI pair programming with Git
│   │   ├── nix/
│   │   │   └── default.nix           # Nix settings and caches
│   │   ├── editor/
│   │   │   └── default.nix           # Helix editor with LSPs
│   │   └── shell/
│   │       ├── default.nix           # Shell module aggregator
│   │       ├── bash.nix              # Bash configuration
│   │       ├── zsh.nix               # Zsh configuration
│   │       ├── nushell.nix           # Nushell configuration
│   │       ├── starship.nix          # Starship prompt
│   │       └── zoxide.nix            # Zoxide smart cd
│   │
│   ├── linux/                        # Linux-specific configurations
│   │   ├── default.nix               # Linux module aggregator
│   │   ├── packages.nix              # Linux packages (debug, serial)
│   │   ├── docker.nix                # Docker/container support
│   │   ├── udev.nix                  # Device rules for robotics
│   │   ├── users.nix                 # User configuration
│   │   └── systemd.nix               # Systemd services
│   │
│   └── macos/                        # macOS-specific configurations
│       ├── default.nix               # macOS module aggregator
│       ├── packages.nix              # macOS packages (GNU tools)
│       ├── homebrew.nix              # Homebrew integration
│       ├── system.nix                # macOS system preferences
│       └── shell.nix                 # macOS shell specifics
│
├── bootstrap.sh                      # Linux/macOS setup script
├── bootstrap.ps1                     # Windows PowerShell setup script
├── flake.nix                         # Main Nix flake configuration
├── flake.lock                        # Locked Nix dependencies
├── pixi.toml                         # Pixi/Conda configuration
├── pixi.lock                         # Locked Pixi dependencies
├── .envrc                            # direnv configuration
└── README.md                         # Main project documentation
```

## Getting Started

### New Users

1. Start with [README.md](../README.md) for installation
2. Review [SKILL.md](./SKILL.md) for available commands
3. Check [RULES.md](./RULES.md) for coding standards

### Contributors

1. Read [CLAUDE.md](./CLAUDE.md) for project conventions
2. Follow [RULES.md](./RULES.md) for contribution guidelines
3. Understand [AGENT.md](./AGENT.md) for system architecture

### Developers

1. Study module structure in `modules/`
2. Review `flake.nix` for package configuration
3. Examine `lib/` for utility functions

## Topic Index

### Installation
- [Quick Start](../README.md#quick-start)
- [Windows/WSL2 Setup](../README.md#windows-installation-recommended-for-windows-users)
- [Linux/macOS Setup](../README.md#linux--macos-installation)
- [Manual Installation](../README.md#manual-installation)

### Configuration
- [Nix Flake](../flake.nix)
- [Pixi Packages](../pixi.toml)
- [direnv Setup](../.envrc)
- [Module System](./CLAUDE.md#module-structure)

### Development
- [Build Commands](./SKILL.md#ros2-development)
- [Package Management](./SKILL.md#pixi-package-management)
- [Code Style](./RULES.md#code-rules)
- [Testing](./RULES.md#testing-requirements)

### CI/CD
- [Workflow Configuration](../.github/workflows/bootstrap-test.yml)
- [Self-Hosted Runner](../.github/docs/self-hosted-runner-setup.md)

### Troubleshooting
- [Common Issues](../README.md#troubleshooting)
- [Nix Debugging](./CLAUDE.md#debugging)

## External Resources

### ROS2
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [RoboStack Packages](https://robostack.github.io/humble.html)

### Nix
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [NixOS Wiki](https://wiki.nixos.org/)
- [Home Manager](https://nix-community.github.io/home-manager/)
- [Flake Parts](https://flake.parts/)

### Tools
- [Pixi Documentation](https://pixi.sh)
- [Helix Editor](https://helix-editor.com/)
- [Starship Prompt](https://starship.rs/)
- [Nushell](https://www.nushell.sh/)

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025 | Initial release with ROS2 Humble support |

## Contributing

See [RULES.md](./RULES.md) for contribution guidelines and coding standards.

## License

This project is licensed under the MIT License. See [LICENSE](../LICENSE) for details.
