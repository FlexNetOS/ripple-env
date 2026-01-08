# Claude Code Instructions

## Project Context

This is a **ROS2 Humble development environment** built with Nix flakes and Pixi. It serves as:

1. A template for robotics projects
2. An agentic system foundation for DevOps, robotics, and life management
3. A cross-platform development environment (Linux, macOS, Windows/WSL2)

## Key Files

| File | Purpose |
|------|---------|
| `flake.nix` | Main Nix flake configuration |
| `pixi.toml` | Pixi/Conda package definitions |
| `bootstrap.sh` | Linux/macOS setup script |
| `bootstrap.ps1` | Windows PowerShell setup script |
| `.envrc` | direnv configuration |
| `modules/` | Home-manager configuration modules |

## Working with This Repository

### Environment Setup

Always ensure the development environment is active:

```bash
# Preferred method
nom develop

# Or via direnv
direnv allow
```

### Building and Testing

```bash
# Build ROS packages
colcon build --symlink-install

# Run tests
colcon test
colcon test-result --verbose

# Check flake
nix flake check
```

### Package Management

```bash
# Add ROS2 packages
pixi add ros-humble-<package-name>

# Add Python packages
pixi add <package-name>

# Update lock files
pixi update
```

## Code Style Guidelines

### Nix
- Use `nixfmt` for formatting
- Prefer `lib.mkOption` with proper types
- Document modules with comments
- Use `flake-parts` patterns

### Python
- Follow PEP 8
- Use type hints
- Document with docstrings

### Shell Scripts
- Use `shellcheck` for linting
- Include help documentation
- Handle errors with `set -e`

### PowerShell
- Use `PSScriptAnalyzer`
- Include comment-based help
- Use approved verbs for functions

## Commit Message Format

```
<type>: <description>

[optional body]

[optional footer]
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

## When Making Changes

1. **Read first**: Always read relevant files before making changes
2. **Understand context**: Check related modules and configurations
3. **Test locally**: Verify changes work in the dev environment
4. **Update docs**: Keep README and documentation in sync
5. **Commit properly**: Use descriptive commit messages

## Important Patterns

### Module Structure

```nix
{ config, lib, pkgs, ... }:
{
  options.module-name = {
    enable = lib.mkEnableOption "description";
  };

  config = lib.mkIf config.module-name.enable {
    # configuration
  };
}
```

### Adding New Tools

1. Add to `flake.nix` in `commonPackages` or `devshells`
2. Or add to appropriate module in `modules/`
3. Update documentation if user-facing

### Adding Skills

Skills are structured knowledge for agents. Create in `.claude/skills/<name>/README.md`:

```markdown
---
name: skill-name
description: What this skill provides
icon: 🔧
category: development
tools:
  - tool1
  - tool2
---

# Skill Title

Documentation content...
```

### Adding Slash Commands

Slash commands provide quick actions. Create in `.claude/commands/<name>.md`:

```markdown
---
name: command-name
description: Command help text
---

Instructions for the command...
```

## Flake Exports

This flake exports modules for use in other projects:

### Home Manager Modules

```nix
{
  inputs.ros2-humble-env.url = "github:FlexNetOS/ros2-humble-env";

  outputs = { ros2-humble-env, ... }: {
    homeConfigurations.user = {
      modules = [
        # Auto-detects platform (Linux/macOS)
        ros2-humble-env.homeManagerModules.default

        # Or specific modules:
        ros2-humble-env.homeManagerModules.common  # Cross-platform only
        ros2-humble-env.homeManagerModules.linux   # Linux-specific
        ros2-humble-env.homeManagerModules.macos   # macOS-specific
      ];
    };
  };
}
```

### NixOS/Darwin Modules

```nix
{
  # For NixOS systems
  nixosConfigurations.host = {
    modules = [ ros2-humble-env.nixosModules.default ];
  };

  # For macOS with nix-darwin
  darwinConfigurations.host = {
    modules = [ ros2-humble-env.darwinModules.default ];
  };
}
```

### Library Functions

```nix
{
  # Access utility functions
  myLib = ros2-humble-env.lib;

  # Available functions:
  # - collectNixFiles: Collect .nix files from directory
  # - isDarwin, isLinux: Platform detection
  # - mkEnableOpt: Create enable option with defaults
  # - deepMerge: Deep merge attribute sets
}
```

### Cross-Platform Considerations

- Check `stdenv.isDarwin` and `stdenv.isLinux`
- Test on multiple platforms when possible
- Use platform-specific modules in `modules/linux/` and `modules/macos/`

## Debugging

### Nix Issues

```bash
# Show flake outputs
nix flake show

# Check for errors
nix flake check

# Build with verbose output
nix build -L
```

### Pixi Issues

```bash
# Clean and reinstall
pixi clean
pixi install

# Show environment
pixi info
```

## Resources

- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Home Manager Options](https://home-manager-options.extranix.com/)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [Pixi Documentation](https://pixi.sh)
