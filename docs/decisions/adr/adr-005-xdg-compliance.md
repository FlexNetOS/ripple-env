# ADR-005: XDG Base Directory Compliance

## Status
Accepted

## Date
2026-01-09

## Context

The XDG Base Directory Specification defines standard locations for user files:
- `$XDG_CONFIG_HOME` (~/.config) - Configuration files
- `$XDG_DATA_HOME` (~/.local/share) - Data files
- `$XDG_STATE_HOME` (~/.local/state) - State files
- `$XDG_CACHE_HOME` (~/.cache) - Cache files

Current state:
- Project uses home-manager but doesn't enable XDG
- No XDG environment variables explicitly set
- Some tools may pollute `$HOME` with dotfiles

Tools evaluated:
- **xdg-ninja** - Audit tool for XDG violations
- **Boxxy** - Sandbox tool to force XDG compliance (Linux-only)
- **home-manager XDG module** - Built-in XDG support

Forces:
- Clean `$HOME` simplifies backup and sync
- ROS2 respects XDG by default (~/.config/ros2/)
- Home-manager has built-in XDG support (zero cost)
- Agentic infrastructure benefits from predictable paths

## Decision

**Enable XDG compliance through three layers:**

### Layer 1: Home-Manager XDG Module (Immediate)
```nix
# modules/common/xdg.nix
{
  xdg.enable = true;
  xdg.userDirs.enable = true;
  xdg.userDirs.createDirectories = true;
}
```

### Layer 2: xdg-ninja for Auditing (High Priority)
```nix
# Via flake input
inputs.xdg-ninja.url = "github:b3nj5m1n/xdg-ninja";

# Add to devShell
packages = [ inputs.xdg-ninja.packages.${system}.default ];
```

### Layer 3: Boxxy for Enforcement (Medium Priority, Linux-only)
```nix
# modules/linux/packages.nix
home.packages = [
  pkgs.boxxy
  pkgs.shadow  # for newuidmap
];
```

## Consequences

### Positive
- Cleaner `$HOME` directory
- Predictable configuration paths for debugging
- Easier backup separation (configs vs cache vs data)
- Aligns with modern Linux practices
- ROS2-compatible by default

### Negative
- Some tools require explicit configuration for XDG
- Boxxy is Linux-only (macOS users miss enforcement)
- Migration may break existing dotfile setups

### Risks
- Breaking existing user configurations
  - **Mitigation**: Document migration steps
- xdg-ninja not in nixpkgs
  - **Mitigation**: Use flake input or custom package
- Boxxy pre-1.0 stability
  - **Mitigation**: Monitor for breaking changes

## XDG Directory Mapping

| Variable | Default | ROS2 Usage |
|----------|---------|------------|
| `XDG_CONFIG_HOME` | ~/.config | ros2/ |
| `XDG_DATA_HOME` | ~/.local/share | ros2/, ament/ |
| `XDG_STATE_HOME` | ~/.local/state | ros2/ logs |
| `XDG_CACHE_HOME` | ~/.cache | colcon/, pip/ |

## Implementation Plan

### Phase 1: Enable home-manager XDG (30 min)
1. Create `modules/common/xdg.nix`
2. Import in `modules/common/default.nix`
3. Test with `nom develop`

### Phase 2: Add xdg-ninja (1-2 hours)
1. Add flake input or custom package
2. Create slash command for audits
3. Document in README

### Phase 3: Add Boxxy (1 hour)
1. Add to `modules/linux/packages.nix`
2. Create default config template
3. Document use cases

## Alternatives Considered

1. **No XDG support**
   - Rejected: Missed opportunity for cleaner setup
   - Rejected: Non-standard configuration paths

2. **Boxxy only (skip xdg-ninja)**
   - Rejected: No audit capability for macOS
   - Rejected: Can't identify violations

3. **Manual XDG configuration**
   - Rejected: home-manager does this automatically
   - Rejected: More work for same result

## References

- [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir/latest/)
- [ArchWiki: XDG Base Directory](https://wiki.archlinux.org/title/XDG_Base_Directory)
- [xdg-ninja GitHub](https://github.com/b3nj5m1n/xdg-ninja)
- [Boxxy GitHub](https://github.com/queer/boxxy)
- [Home-Manager XDG Module](https://nix-community.github.io/home-manager/options.xhtml#opt-xdg.enable)
- Research: XDG compliance research agent report
