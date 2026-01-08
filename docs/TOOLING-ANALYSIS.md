# Tooling Deep Research & Cross-Reference Analysis

## Current Stack Summary

| Component | Current Implementation | Status |
|-----------|----------------------|--------|
| Package Manager | Nix Flakes + Pixi (RoboStack) | ✅ Active |
| Dev Shell | numtide/devshell | ✅ Active |
| Env Activation | direnv (`.envrc` with `use flake`) | ✅ Active |
| Python | 3.11.x via Pixi/conda-forge | ✅ Active |
| ROS | ros-humble-desktop via RoboStack | ✅ Active |

---

## Tool Analysis Matrix

### 1. AI-Native Coding Assistants (Vibe Coding Core)

#### Aider (39.4k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | AI pair programming in terminal with Git integration |
| **Key Features** | 100+ languages, repo mapping, auto-commits, voice-to-code |
| **Model Support** | GPT-4o, Claude 3.5/3.7 Sonnet, DeepSeek, Ollama |
| **Conflict Status** | ✅ **NO CONFLICT** - Additive tool |
| **Nix Package** | `pkgs.aider-chat` available in nixpkgs |
| **Integration Notes** | Works alongside any editor; can add to devshell packages |

**Upgrade Path:**
```nix
devshell.packages = with pkgs; [
  pixi
  aider-chat  # Add AI pair programming
];
```

#### Zed (72.5k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | High-performance GPU-accelerated editor with agentic AI |
| **Key Features** | Built-in AI agent panel, edit prediction (Zeta model), multiplayer |
| **Nix Status** | `zed-editor` in nixpkgs 24.11+; official flake available |
| **Conflict Status** | ✅ **NO CONFLICT** - Optional editor choice |
| **Requirements** | Hardware-accelerated Vulkan (use nixGL if needed) |

**Nix Integration:**
```nix
# Available as: pkgs.zed-editor
# Or via flake: github:zed-industries/zed
```

---

### 2. Environment & Tool Management

#### mise (22.7k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Rust-based dev tool version manager (asdf replacement) |
| **Key Features** | 2-5x faster than asdf, no shims, direnv compatible |
| **Conflict Status** | ⚠️ **PARTIAL OVERLAP** with current Nix+Pixi setup |
| **Current Overlap** | Pixi already handles Python/conda versioning |
| **Recommendation** | **NOT NEEDED** - Nix+Pixi provides superior reproducibility |

**Analysis:**
- mise replaces asdf/nvm/pyenv but your current stack (Nix Flakes + Pixi) already provides:
  - Reproducible package versioning via `flake.lock` + `pixi.lock`
  - Environment activation via direnv
  - Cross-platform support
- mise would add complexity without clear benefit

#### direnv (14.5k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Directory-based env var management |
| **Conflict Status** | ✅ **ALREADY IN USE** |
| **Current Config** | `.envrc` contains `use flake` |
| **Enhancement** | Could add `.envrc` per-project variables |

**Status:** Already integrated - no changes needed.

#### DevPod (14.5k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Open-source Codespaces using devcontainer.json |
| **Key Features** | Client-only, any cloud/k8s/local Docker, 5-10x cheaper |
| **Conflict Status** | ✅ **COMPLEMENTARY** - Different use case |
| **Integration** | Could add `.devcontainer/devcontainer.json` for remote dev |

**Upgrade Path:**
```
ros2-humble-env/
├── .devcontainer/
│   └── devcontainer.json  # NEW: Enable DevPod/Codespaces
├── flake.nix
├── pixi.toml
└── ...
```

---

### 3. Dotfile & Home Directory Management

#### chezmoi (17.3k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Cross-platform dotfile manager with templating/secrets |
| **Key Features** | Go templates, 1Password/pass integration, git-native |
| **Conflict Status** | ⚠️ **DIFFERENT SCOPE** - User dotfiles, not project config |
| **Relevance** | Low - This repo is project config, not user dotfiles |

**Recommendation:** Useful for users' personal setups but not for this repository template.

#### home-manager (9.1k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Nix-based declarative home directory management |
| **Key Features** | Full Nix ecosystem, reproducible user environments |
| **Conflict Status** | ✅ **COMPLEMENTARY** - Can coexist with project flakes |
| **Integration** | Users could import this flake into their home-manager config |

**Example home-manager integration:**
```nix
# User's home.nix
{
  imports = [ /* ... */ ];

  # Can add ros2-humble-env as a development shell
  home.shellAliases = {
    ros2-dev = "cd ~/projects/ros2-humble-env && nix develop";
  };
}
```

#### xdg-ninja (3.1k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Audit $HOME for XDG compliance |
| **Requirements** | POSIX shell + glow (for markdown rendering) |
| **Conflict Status** | ✅ **NO CONFLICT** - Diagnostic tool |
| **Relevance** | Medium - Helps users clean cluttered $HOME |

**Nix Package:** `pkgs.xdg-ninja`

#### boxxy (1.7k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | Linux namespace sandbox for XDG non-compliant apps |
| **Platform** | **Linux-only** |
| **Conflict Status** | ✅ **NO CONFLICT** - Additive sandboxing |
| **Requirements** | `newuidmap` must be available |
| **Use Case** | Force apps like tmux to respect XDG paths |

---

### 4. LazyVim + AI CLI Tools

#### live-preview.nvim
| Aspect | Details |
|--------|---------|
| **Description** | Neovim markdown/HTML/SVG live preview |
| **Key Features** | No external deps, pure Lua, sync scrolling, KaTeX, Mermaid |
| **Conflict Status** | ✅ **NO CONFLICT** - Editor plugin |
| **Integration** | Add to user's LazyVim config, not project |

#### peek.nvim
| Aspect | Details |
|--------|---------|
| **Description** | Neovim markdown preview with Deno backend |
| **Requirements** | **Deno** runtime |
| **Conflict Status** | ✅ **NO CONFLICT** - Editor plugin |
| **LazyVim Default** | LazyVim uses markdown-preview.nvim by default |

**Note:** live-preview.nvim is preferred over peek.nvim due to zero external dependencies.

#### aichat (CLI)
| Aspect | Details |
|--------|---------|
| **Description** | All-in-one LLM CLI: Shell assistant, REPL, RAG, agents |
| **Stars** | Growing rapidly (featured tool) |
| **Provider Support** | 20+ providers: OpenAI, Claude, Gemini, Ollama, Groq, etc. |
| **Conflict Status** | ✅ **NO CONFLICT** - Additive CLI tool |
| **Nix Package** | `pkgs.aichat` available |

**Recommendation:** Ship in foundation dev shell as lightweight AI CLI.

```nix
devshell.packages = with pkgs; [
  pixi
  aichat  # Tiny, provider-agnostic AI CLI
];
```

#### cc-mirror
| Aspect | Details |
|--------|---------|
| **Description** | Meta-CLI for isolated Claude Code variants |
| **Key Features** | Multi-provider support, team orchestration, task management |
| **Stars** | 1.1k |
| **Conflict Status** | ✅ **NO CONFLICT** - Optional heavyweight overlay |
| **Recommendation** | Offer as opt-in overlay for Claude-centric workflows |

---

## Recommended Architecture

### Foundation Layer (Default Dev Shell)
```nix
devshell.packages = with pkgs; [
  # Current
  pixi

  # Recommended additions
  aichat        # Lightweight AI CLI (provider-agnostic)
  xdg-ninja     # $HOME hygiene auditing
];
```

### Optional Overlays/Features

| Feature Flag | Tools Added | Use Case |
|--------------|-------------|----------|
| `ai-heavy` | aider-chat, cc-mirror | Deep AI pair programming |
| `editor-zed` | zed-editor | GPU-accelerated AI editor |
| `devcontainer` | devpod | Remote dev environments |
| `sandbox` | boxxy | Linux XDG sandboxing |

### Proposed flake.nix Structure
```nix
{
  outputs = { ... }: {
    devShells = {
      default = { /* base shell with pixi + aichat */ };

      # Feature variants
      ai-heavy = { /* adds aider, cc-mirror */ };
      full = { /* all tools */ };
    };
  };
}
```

---

## Dependency Summary

### Required Dependencies (Already Satisfied)
- Nix with flakes ✅
- Git ✅
- direnv ✅

### New Dependencies by Tool

| Tool | Dependencies | Nix Available |
|------|--------------|---------------|
| aichat | None (static binary) | ✅ `pkgs.aichat` |
| aider | Python 3.8+ | ✅ `pkgs.aider-chat` |
| zed-editor | Vulkan, GPU drivers | ✅ `pkgs.zed-editor` |
| xdg-ninja | glow (optional) | ✅ `pkgs.xdg-ninja` |
| boxxy | newuidmap | ✅ `pkgs.boxxy` |
| peek.nvim | Deno | ✅ `pkgs.deno` |
| live-preview.nvim | None | N/A (Lua plugin) |
| DevPod | Docker/Podman | ✅ `pkgs.devpod` |

---

## Conflicts & Resolutions

| Potential Conflict | Resolution |
|-------------------|------------|
| mise vs Nix+Pixi | **Skip mise** - Nix provides superior reproducibility |
| chezmoi vs home-manager | **Different scope** - chezmoi for portable dotfiles, home-manager for Nix-native |
| Multiple AI tools | **Layer approach** - aichat as default, aider/cc-mirror as opt-in |

---

## Implementation Priority

### Phase 1: Foundation Enhancement
1. Add `aichat` to default devshell
2. Add `xdg-ninja` as optional diagnostic

### Phase 2: AI Tooling Layer
1. Create `ai-heavy` devshell variant with aider
2. Document cc-mirror as external overlay option

### Phase 3: Remote Development
1. Add `.devcontainer/devcontainer.json` for DevPod/Codespaces
2. Ensure Nix works within devcontainer

### Phase 4: Editor Integration
1. Document Zed configuration for Nix users
2. Provide LazyVim plugin recommendations (live-preview.nvim)

---

## Sources

### AI Coding Assistants
- [Aider GitHub](https://github.com/Aider-AI/aider)
- [Aider Documentation](https://aider.chat/docs/)
- [Zed Editor](https://zed.dev/)
- [Zed Agentic Editing](https://zed.dev/agentic)
- [Zed NixOS Wiki](https://wiki.nixos.org/wiki/Zed)

### Environment Tools
- [mise-en-place](https://mise.jdx.dev/)
- [mise vs asdf comparison](https://mise.jdx.dev/dev-tools/comparison-to-asdf.html)
- [DevPod](https://devpod.sh/)
- [DevPod GitHub](https://github.com/loft-sh/devpod)

### Dotfile Management
- [chezmoi](https://www.chezmoi.io/)
- [home-manager GitHub](https://github.com/nix-community/home-manager)
- [xdg-ninja GitHub](https://github.com/b3nj5m1n/xdg-ninja)
- [boxxy GitHub](https://github.com/queer/boxxy)

### AI CLI Tools
- [aichat GitHub](https://github.com/sigoden/aichat)
- [cc-mirror GitHub](https://github.com/numman-ali/cc-mirror)

### Neovim Plugins
- [live-preview.nvim](https://github.com/brianhuster/live-preview.nvim)
- [peek.nvim](https://github.com/toppair/peek.nvim)
- [LazyVim Markdown](http://www.lazyvim.org/extras/lang/markdown)
