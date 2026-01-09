# ADR-002: AI Coding Assistants - aichat + aider + Optional Neovim Plugins

## Status
Accepted

## Date
2026-01-09

## Context

Modern AI coding assistance requires tools that support different cognitive modes:
- **Quick queries**: One-off questions about code or concepts
- **Pair programming**: Extended sessions with repository context
- **Deep work**: IDE-integrated AI with inline code generation

Current tooling:
- `aichat` - Lightweight CLI, provider-agnostic
- `aider` - Git-integrated pair programming with auto-commits
- Neovim plugins (codecompanion.nvim, avante.nvim) - IDE-level integration

Forces:
- Different tasks require different AI interaction patterns
- API key management must be secure across tools
- Nix reproducibility needed for development environments
- Claude Code CLI provides terminal-based workflow

## Decision

**Implement a layered AI assistant architecture**:

### Layer 1: Terminal Quick Queries (Default)
```bash
ai "explain ROS2 QoS profiles"         # aichat - instant, no context
ai-code "write a publisher node"       # aichat --role coder
ai-review "check this for issues"      # aichat --role reviewer
```

### Layer 2: Git-Integrated Pair Programming (Default)
```bash
pair src/robot_control/                # aider - full repo context
pair-voice                             # aider --voice
pair-watch                             # aider --watch (auto-commit)
```

### Layer 3: IDE-Integrated AI (Optional, via ADR-001)
```vim
:CodeCompanionChat                     " codecompanion.nvim - chat buffer
:CodeCompanionToggle                   " inline suggestions
```

### API Key Management
```nix
# Secure storage via home-manager
home.sessionVariables = {
  ANTHROPIC_API_KEY = "$(cat ~/.secrets/anthropic-api-key)";
  OPENAI_API_KEY = "$(cat ~/.secrets/openai-api-key)";
};
```

## Consequences

### Positive
- Right tool for each cognitive mode
- Minimal startup overhead for common tasks
- Full IDE experience when needed
- Consistent API key handling across tools
- Git history enriched with AI-assisted commits (aider)

### Negative
- Three separate tools to learn
- API costs may accumulate with heavy usage
- No single unified interface

### Risks
- API key exposure if not properly secured
  - **Mitigation**: Store in `~/.secrets/` with 600 permissions
- aider auto-commits may pollute Git history
  - **Mitigation**: Use `pair-watch` judiciously, review commits

## Plugin Recommendations

### Tier 1: Install by Default
| Tool | Purpose | Package |
|------|---------|---------|
| aichat | Quick queries | `pkgs.aichat` |
| aider | Pair programming | `pkgs.aider-chat` |

### Tier 2: Optional Neovim Plugins
| Plugin | Purpose | Complexity |
|--------|---------|------------|
| codecompanion.nvim | Chat + inline edits | Medium |
| live-preview.nvim | Markdown preview | Low |

### Tier 3: Advanced (Future)
| Plugin | Purpose | Notes |
|--------|---------|-------|
| avante.nvim | Cursor-like experience | Higher complexity |
| claudecode.nvim | Native MCP protocol | Requires Claude CLI |

## Alternatives Considered

1. **Claude Code CLI only**
   - Rejected: Terminal-based, misses IDE integration
   - Partial: Keep as complementary tool

2. **Cursor IDE**
   - Rejected: Proprietary, heavy resources
   - Rejected: Breaks Nix workflow

3. **GitHub Copilot**
   - Rejected: Requires subscription
   - Rejected: Less flexible than multi-provider setup

## References

- [aichat GitHub](https://github.com/sigoden/aichat)
- [aider GitHub](https://github.com/paul-gauthier/aider)
- [codecompanion.nvim](https://github.com/olimorris/codecompanion.nvim)
- [avante.nvim](https://github.com/yetone/avante.nvim)
- Research: Agent research on Claude Neovim integrations
