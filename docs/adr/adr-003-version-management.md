# ADR-003: Version Management - Nix + Pixi (Skip mise/0install)

## Status
Accepted

## Date
2026-01-09

## Context

The project uses multiple package management tools:
- **Nix flakes** - System packages, reproducible environments
- **Pixi/Conda** - ROS2 Humble via RoboStack, Python packages

Additional tools were evaluated:
- **mise** (formerly rtx) - polyglot runtime version manager
- **0install** - decentralized package manager
- **asdf** - legacy version manager

Forces:
- Nix provides cryptographic reproducibility (locked hashes)
- Pixi handles ROS2/Python ecosystem effectively
- Adding more tools creates complexity and confusion
- Team consistency requires clear tooling boundaries

## Decision

**Continue with Nix + Pixi; do NOT add mise or 0install.**

### Rationale

#### mise Analysis
| Aspect | mise | Nix + Pixi |
|--------|------|------------|
| **Reproducibility** | Hash-based but not cryptographic | Cryptographic hashes |
| **Language Support** | Broad (Node, Python, Ruby, etc.) | Full via nixpkgs |
| **ROS2 Support** | None | Full via RoboStack |
| **Learning Curve** | Low | Medium (already invested) |

**Verdict**: mise would be **redundant**. Nix already manages all system packages with superior reproducibility.

#### 0install Analysis
| Aspect | 0install | Nix + Pixi |
|--------|----------|------------|
| **Decentralization** | Yes (feeds) | No (central registry) |
| **Reproducibility** | Hash-based | Cryptographic |
| **Adoption** | Niche | Widespread (Nix), Growing (Pixi) |
| **ROS2 Support** | None | Full |

**Verdict**: 0install would **conflict** with Nix architecture and add complexity with no benefit.

### Current Architecture

```
┌─────────────────────────────────────────────────────┐
│                  User Environment                    │
├─────────────────────────────────────────────────────┤
│  direnv (.envrc)                                     │
│    └── Automatic environment activation              │
├─────────────────────────────────────────────────────┤
│  Nix Flake (flake.nix)                              │
│    ├── System packages (aichat, helix, git, etc.)   │
│    ├── Development tools (cargo, gcc, cmake)        │
│    ├── LSP servers (pyright, clangd, nil)          │
│    └── Cross-platform modules (home-manager)        │
├─────────────────────────────────────────────────────┤
│  Pixi (pixi.toml)                                   │
│    ├── ROS2 Humble (ros-humble-desktop)             │
│    ├── Python packages (rosdep, colcon)             │
│    └── Node.js (for Neovim plugins)                 │
└─────────────────────────────────────────────────────┘
```

### Responsibility Matrix

| Category | Tool | Examples |
|----------|------|----------|
| System packages | Nix | git, helix, aichat, cargo |
| Development shells | Nix | colcon aliases, env vars |
| ROS2 packages | Pixi | ros-humble-*, rosdep |
| Python packages | Pixi | numpy, pytest, black |
| Node.js | Nix + Pixi | nodejs_22, pnpm |
| Rust toolchain | Nix | cargo, rust-analyzer |

## Consequences

### Positive
- Clear separation of concerns (Nix: system, Pixi: ROS2/Python)
- No tool overlap or confusion
- Simpler onboarding (two tools, not four)
- Maximum reproducibility via Nix flake.lock

### Negative
- Cannot use mise for quick runtime version switches
- Locked to Nix's release cadence for new packages

### Risks
- Nix package may lag behind upstream releases
  - **Mitigation**: Use `pkgs.unstable` for cutting-edge packages
- Pixi/RoboStack may not have all ROS2 packages
  - **Mitigation**: Fall back to rosdep source builds

## Alternatives Considered

1. **Replace Pixi with mise**
   - Rejected: mise cannot provide ROS2 via RoboStack
   - Rejected: Would lose conda ecosystem

2. **Add mise alongside Nix + Pixi**
   - Rejected: Three package managers is excessive
   - Rejected: Overlapping responsibilities

3. **Use 0install for distribution feeds**
   - Rejected: Niche tool, low adoption
   - Rejected: Conflicts with Nix architecture

## References

- [mise Documentation](https://mise.jdx.dev/)
- [0install Documentation](https://docs.0install.net/)
- [Nix vs mise comparison](https://discourse.nixos.org/t/mise-vs-nix/34567)
- Research: Cross-analysis of current codebase
