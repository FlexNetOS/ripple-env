# Documentation Index

## Quick Navigation

| Document | Description |
|----------|-------------|
| [README](../README.md) | Project overview and getting started |
| [AGENT](./AGENT.md) | Agentic system architecture and capabilities |
| [CLAUDE](./CLAUDE.md) | Claude Code specific instructions |
| [RULES](./RULES.md) | Guidelines and coding standards |
| [SKILL](./SKILL.md) | Available skills and tool reference |

## Project Structure

```
ros2-humble-env/
├── .claude/                    # Agent configuration
│   ├── INDEX.md               # This file
│   ├── AGENT.md               # Agent system docs
│   ├── CLAUDE.md              # Claude instructions
│   ├── RULES.md               # Rules and guidelines
│   └── SKILL.md               # Skills reference
│
├── .github/
│   ├── workflows/
│   │   └── bootstrap-test.yml # CI workflow
│   └── docs/
│       └── self-hosted-runner-setup.md
│
├── lib/                        # Nix library
│   ├── default.nix            # Utility functions
│   └── system.nix             # System helpers
│
├── modules/                    # Configuration modules
│   ├── common/                # Cross-platform
│   │   ├── default.nix        # Module aggregator
│   │   ├── direnv.nix         # direnv config
│   │   ├── git.nix            # Git config
│   │   ├── packages.nix       # Common packages
│   │   ├── nix/               # Nix settings
│   │   ├── editor/            # Helix config
│   │   └── shell/             # Shell configs
│   │       ├── bash.nix
│   │       ├── zsh.nix
│   │       ├── nushell.nix
│   │       ├── starship.nix
│   │       └── zoxide.nix
│   │
│   ├── linux/                 # Linux-specific
│   │   ├── default.nix
│   │   ├── docker.nix
│   │   ├── packages.nix
│   │   ├── systemd.nix
│   │   ├── udev.nix
│   │   └── users.nix
│   │
│   └── macos/                 # macOS-specific
│       ├── default.nix
│       ├── homebrew.nix
│       ├── packages.nix
│       ├── shell.nix
│       └── system.nix
│
├── bootstrap.sh               # Linux/macOS setup
├── bootstrap.ps1              # Windows setup
├── flake.nix                  # Main Nix flake
├── flake.lock                 # Locked dependencies
├── pixi.toml                  # Pixi configuration
├── pixi.lock                  # Pixi lock file
├── .envrc                     # direnv configuration
└── README.md                  # Main documentation
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
