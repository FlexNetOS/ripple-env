# ADR-001: Editor Strategy - Helix Default with Optional LazyVim

## Status
Accepted

## Date
2026-01-09

## Context

The ros2-humble-env project needs a code editor strategy that balances:
- **Fast startup** for quick Git commits and small edits
- **Full IDE features** for extended AI-assisted development sessions
- **Nix reproducibility** without external plugin managers
- **Cross-platform support** (Linux, macOS, WSL2)
- **ROS2-optimized LSP configuration**

Current state:
- Helix is configured as the default `$EDITOR`
- 12 LSP servers configured for Python, C++, Nix, Rust, etc.
- LazyVim dependencies exist (Node.js 22, tree-sitter, lazygit)
- No Neovim configuration present

Forces:
- Helix has <100ms startup time, ideal for `git commit`
- LazyVim provides Cursor-like AI experience but slower startup (~500ms)
- Team members may have different editor preferences
- Nix purity conflicts with Mason (LazyVim's LSP auto-installer)

## Decision

**Keep Helix as the default editor** with a new **optional LazyVim module** that users can enable via home-manager.

### Implementation

1. **Helix remains default** (`EDITOR=hx`, `VISUAL=hx`)
2. **Create `modules/common/editor/neovim.nix`** with:
   - `programs.neovim-ai.enable` option (default: false)
   - LazyVim configuration with Mason disabled
   - codecompanion.nvim for Claude integration
   - live-preview.nvim for Markdown preview
3. **Reuse existing LSP servers** from Helix configuration
4. **Add shell aliases** for clarity:
   - `vi` → `hx` (fast editing)
   - `nvim-ai` → `nvim` (AI-enhanced editing)

### User Choice

```nix
# Keep Helix for quick edits (default)
programs.helix.defaultEditor = true;

# Enable LazyVim for AI-assisted work (opt-in)
programs.neovim-ai.enable = true;
```

## Consequences

### Positive
- Users choose editor based on task cognitive mode
- Fast startup preserved for common operations (Git, quick fixes)
- Full IDE experience available when needed
- No opinion forced on contributors
- LSP servers shared (no duplication)

### Negative
- Two editor configurations to maintain
- Additional ~65MB disk space for Neovim + plugins
- Documentation complexity increases

### Risks
- LazyVim auto-updates may break Nix reproducibility
  - **Mitigation**: Disable Mason, lock plugin versions
- Helix users may not discover Neovim option
  - **Mitigation**: Document in README with clear use cases

## Alternatives Considered

1. **LazyVim as default**
   - Rejected: Slower startup penalizes common tasks
   - Rejected: Higher complexity for minimal benefit

2. **VS Code / Cursor**
   - Rejected: Heavy resource usage
   - Rejected: GUI dependency problematic for WSL2/SSH

3. **Helix-only**
   - Rejected: Misses AI plugin ecosystem
   - Rejected: Limits power users

## References

- [LazyVim Documentation](https://www.lazyvim.org/)
- [Helix Editor](https://helix-editor.com/)
- [Setup LazyVim using Nix](https://github.com/LazyVim/LazyVim/discussions/1972)
- Research: `.claude/skills/ai-assistants/README.md`
