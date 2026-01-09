# Tooling Deep Research & Cross-Reference Analysis

## Current Stack Summary

| Component | Current Implementation | Status |
|-----------|----------------------|--------|
| Package Manager | Nix Flakes + Pixi (RoboStack) | ✅ Active |
| Dev Shell | numtide/devshell via flake-parts | ✅ Active |
| Home Manager | nix-community/home-manager modules | ✅ Active |
| Env Activation | direnv (`.envrc` with `use flake`) | ✅ Active |
| Python | 3.11.14+ via Pixi/conda-forge | ✅ Active |
| ROS | ros-humble-desktop via RoboStack | ✅ Active |
| AI Assistants | aichat + aider-chat + Agent Gateway | ✅ **IMPLEMENTED** |
| Editor | Helix with ROS2 LSPs | ✅ Active |
| Shells | bash, zsh, nushell + starship | ✅ Active |
| Multi-platform | Windows (WSL2), Linux, macOS | ✅ Active |

### Current Architecture Details

```
ros2-humble-env/
├── .envrc                      # direnv: `use flake` + local overrides
├── .pixi/
│   └── config.toml             # Pixi: run-post-link-scripts = "insecure"
├── .github/
│   └── workflows/
│       └── bootstrap-test.yml  # CI workflow
├── flake.nix                   # Main flake with devshells + home-manager modules
├── flake.lock                  # Locked: nixpkgs, flake-parts, devshell, home-manager
├── pixi.toml                   # RoboStack + compilation cache + Node.js
├── pixi.lock                   # Locked conda/RoboStack packages
├── bootstrap.sh                # Linux/macOS bootstrap script
├── bootstrap.ps1               # Windows PowerShell bootstrap (WSL2 + NixOS)
├── lib/
│   ├── default.nix             # Library utilities
│   └── system.nix              # System builder helpers
├── modules/
│   ├── common/                 # Cross-platform home-manager modules
│   │   ├── default.nix         # Module aggregator
│   │   ├── direnv.nix          # Enhanced direnv config
│   │   ├── git.nix             # Git configuration
│   │   ├── packages.nix        # Common packages
│   │   ├── ai/                 # AI assistants
│   │   │   ├── aichat.nix      # aichat configuration
│   │   │   └── aider.nix       # aider configuration
│   │   ├── nix/                # Nix settings
│   │   ├── editor/             # Helix editor with ROS2 LSPs
│   │   └── shell/              # Shell configurations
│   │       ├── bash.nix
│   │       ├── zsh.nix
│   │       ├── nushell.nix
│   │       ├── zoxide.nix
│   │       └── starship.nix
│   ├── linux/                  # Linux-specific configurations
│   │   ├── default.nix
│   │   ├── packages.nix
│   │   ├── docker.nix
│   │   ├── udev.nix            # Device rules for robotics
│   │   ├── users.nix
│   │   └── systemd.nix
│   └── macos/                  # macOS-specific configurations
│       ├── default.nix
│       ├── packages.nix
│       ├── homebrew.nix
│       ├── system.nix
│       └── shell.nix
└── docs/
    └── TOOLING-ANALYSIS.md     # This file
```

### Implementation Status vs Analysis Recommendations

| Recommendation | Status | Location |
|---------------|--------|----------|
| Add aichat to devshell | ✅ **DONE** | `flake.nix:132` |
| Add aider-chat to devshell | ✅ **DONE** | `flake.nix:133` |
| Create CI devshell variant | ✅ **DONE** | `flake.nix:269-295` |
| home-manager integration | ✅ **DONE** | `flake.nix:40-61` |
| Multiple shell support | ✅ **DONE** | `modules/common/shell/` |
| Command aliases (cb, ct, ai, pair) | ✅ **DONE** | `flake.nix:230-266` |
| Compilation cache (ccache, sccache) | ✅ **DONE** | `flake.nix:139-141`, `pixi.toml:18-19` |
| Node.js for LazyVim plugins | ✅ **DONE** | `flake.nix:147-148`, `pixi.toml:26-27` |
| Bootstrap scripts | ✅ **DONE** | `bootstrap.sh`, `bootstrap.ps1` |
| Add xdg-ninja | ⏳ Pending | Not yet added |
| Add DevPod/devcontainer | ⏳ Pending | `.devcontainer/` not created |
| Add Zed editor | ⏳ Pending | Not in packages (helix is default) |

### Key Configuration Details

**flake.nix structure (flake-parts + devshell):**
```nix
{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };
  outputs = inputs: flake-parts.lib.mkFlake { inherit inputs; } {
    imports = [ devshell.flakeModule ];
    perSystem = { pkgs, ... }: {
      devshells.default = {
        env = [{ name = "COLCON_DEFAULTS_FILE"; value = "..."; }];
        devshell = {
          packages = with pkgs; [ pixi ];
          startup.activate.text = ''eval "$(pixi shell-hook)"'';
        };
      };
    };
  };
}
```

**Platform-specific handling:**
- macOS: Sets `DYLD_FALLBACK_LIBRARY_PATH` for Pixi libraries
- macOS: Adds `-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON` to colcon cmake-args
- All: Configures `COLCON_DEFAULTS_FILE` with compile_commands.json generation

**Pixi configuration (`.pixi/config.toml`):**
```toml
run-post-link-scripts = "insecure"
```
This enables conda post-link scripts required for ROS package activation.

**pixi.toml dependencies (via RoboStack):**
| Category | Packages |
|----------|----------|
| Python | `python >=3.11.14,<3.12` |
| Build Tools | `compilers`, `cmake >=4.2.0`, `pkg-config`, `make`, `ninja` |
| Compilation Cache | `ccache >=4.10`, `sccache >=0.8` |
| Archive/Network | `tar >=1.34`, `curl >=8.0` |
| Node.js | `nodejs >=22.0` (LTS Jod), `pnpm >=9.0` |
| ROS Tools | `rosdep`, `colcon-common-extensions`, `catkin_tools` |
| ROS Core | `ros-humble-desktop >=0.10.0` |
| Channels | `robostack-humble`, `conda-forge` |

**flake.nix packages (via nixpkgs):**
| Category | Packages |
|----------|----------|
| Core | `pixi`, `git`, `gh` |
| Nix Tools | `nix-output-monitor`, `nix-tree`, `nixfmt-rfc-style`, `nil` |
| Shell Utils | `bat`, `eza`, `fd`, `ripgrep`, `fzf`, `jq`, `yq` |
| Navigation | `zoxide`, `direnv`, `nix-direnv` |
| Shells | `zsh`, `nushell`, `starship` |
| Editor | `helix` |
| AI Assistants | `aichat`, `aider-chat`, `portaudio` (voice) |
| Build Cache | `ccache`, `sccache`, `mold` (fast linker) |
| LazyVim Support | `tree-sitter`, `nodejs_22`, `pnpm`, `lazygit` |
| Linux-only | `inotify-tools`, `strace`, `gdb` |
| macOS-only | `coreutils`, `gnused`, `gawk` |

**ROS2 Jazzy Upgrade Path (2025+):**
RoboStack now supports ROS2 Jazzy via `robostack-jazzy` channel. To add Jazzy as a feature:
```bash
pixi workspace channel add https://prefix.dev/robostack-jazzy --feature jazzy
pixi workspace environment add jazzy --feature jazzy
pixi add ros-jazzy-ros-base --feature jazzy
```

---

## Tool Analysis Matrix

### 1. AI-Native Coding Assistants (Vibe Coding Core)

#### Aider (39.4k ⭐)
| Aspect | Details |
|--------|---------|
| **Description** | AI pair programming in terminal with Git integration |
| **Key Features** | 100+ languages, repo mapping, auto-commits, voice-to-code |
| **Model Support** | Claude 3.7 Sonnet, DeepSeek R1/V3, OpenAI o1/o3-mini/GPT-4o, Grok-4, local models |
| **Conflict Status** | ✅ **NO CONFLICT** - Additive tool |
| **Nix Package** | `pkgs.aider-chat` available in nixpkgs |
| **Integration Notes** | Works alongside any editor; can add to devshell packages |

**Recent Features (Jan 2026):**
- `--thinking-tokens` CLI option for models with thinking capability
- `/architect` mode for planning, `/ask` for questions
- In-code `AI?` comments for inline assistance
- Aider wrote 72% of its own code in recent releases

**Upgrade Path (flake-parts/devshell syntax):**
```nix
devshells.default.devshell.packages = with pkgs; [
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
| **Latest Version** | v0.217.4 (Jan 5, 2026) |

**2025-2026 AI Updates:**
- **Zeta**: Open-source, open-dataset edit prediction model
- **ACP (Agent Connection Protocol)**: Connect any agent to Zed
- **Multibuffer review**: Review agent changes across files
- **Agent following**: Watch agent work live
- Worktree trust mechanism (Jan 7, 2026)

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

**Detailed Overlap Analysis:**

| mise Feature | Current Stack Equivalent |
|--------------|-------------------------|
| Tool versioning | `flake.lock` (Nix) + `pixi.lock` (conda) |
| Python version | `pixi.toml`: `python = ">=3.11.14,<3.12"` |
| Environment activation | `direnv` + `use flake` + `pixi shell-hook` |
| `.tool-versions` | Not needed - versions locked in `*.lock` files |
| Plugin ecosystem | RoboStack channel + nixpkgs |
| Cross-platform | `nix-systems/default` + pixi platforms |

**Conclusion:** mise would add a third layer of tooling without improving reproducibility. The current Nix+Pixi stack provides:
- **Stronger guarantees**: Cryptographic hashes in lock files
- **Atomic rollbacks**: Via Nix generations
- **No shims**: Pixi uses shell-hook, Nix modifies PATH directly

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

**⚠️ Known Issue:** Nix flake builds in devcontainers can be slow (hours vs seconds on bare metal). Use binary caches or pre-built images.

**Recommended devcontainer.json for Nix + ROS2:**
```json
{
  "name": "ros2-humble-env",
  "image": "ghcr.io/lucernae/devcontainer-nix/nix:1",
  "features": {
    "ghcr.io/devcontainers/features/nix:1": {
      "extraNixConfig": "experimental-features = nix-command flakes\nsandbox = true"
    }
  },
  "mounts": [
    "source=nix-store,target=/nix,type=volume"
  ],
  "postCreateCommand": "direnv allow && nix develop --command true",
  "customizations": {
    "vscode": {
      "extensions": ["mkhl.direnv", "jnoortheen.nix-ide"]
    }
  }
}
```

**Best Practices (2025):**
- Mount `/nix` as named volume for caching
- Use direnv + flakes (not Dockerfile RUN commands)
- Pre-populate Nix store from binary cache
- Consider [xtruder/nix-devcontainer](https://github.com/xtruder/nix-devcontainer) for Swiss-army-knife setup

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
| **Latest Version** | v0.30.0 |
| **Provider Support** | 100+ LLMs across 20+ providers: OpenAI, Claude, Gemini, Ollama, Groq, DeepSeek, XAI Grok, etc. |
| **Conflict Status** | ✅ **NO CONFLICT** - Additive CLI tool |
| **Nix Package** | `pkgs.aichat` available |

**Key Features (2025+):**
- Shell auto-completion with AI
- Custom macros for repetitive tasks
- Local API server (OpenAI-compatible)
- LLM Playground/Arena WebUI
- RAG for document integration

**Recommendation:** Ship in foundation dev shell as lightweight AI CLI.

```nix
# In flake.nix perSystem block:
devshells.default.devshell.packages = with pkgs; [
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
# In flake.nix perSystem block:
devshells.default.devshell.packages = with pkgs; [
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

### Proposed flake.nix Structure (flake-parts + devshell)
```nix
{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };

  outputs = inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [ inputs.devshell.flakeModule ];

      perSystem = { pkgs, ... }: {
        # Default shell: foundation tools
        devshells.default = {
          devshell.packages = with pkgs; [ pixi aichat ];
          # ... existing env and startup config
        };

        # AI-heavy variant
        devshells.ai-heavy = {
          devshell.packages = with pkgs; [ pixi aichat aider-chat ];
        };

        # Full toolset
        devshells.full = {
          devshell.packages = with pkgs; [
            pixi aichat aider-chat xdg-ninja
          ] ++ pkgs.lib.optionals pkgs.stdenv.isLinux [ boxxy ];
        };
      };
    };
}
```

**Usage:**
```bash
nix develop              # default shell
nix develop .#ai-heavy   # with aider
nix develop .#full       # all tools
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

## Security Considerations (2025 Best Practices)

### Nix Flake Security

| Practice | Recommendation |
|----------|----------------|
| **Trusted Users** | Adding user to `trusted-users` = root access. Use sparingly. |
| **Binary Caches** | Prefer private caches over `cache.nixos.org` for sensitive projects |
| **Nixpkgs Source** | Always use official NixOS/nixpkgs, not forks |
| **Release Branches** | Use supported branches (stop receiving updates ~7 months after release) |
| **Sandboxing** | Keep `sandbox = true` in nix.conf (default) |

### Flake Input Verification
```nix
# Ensure inputs are from trusted sources
inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";  # Official only
```

### Runtime Sandboxing Options
- **[NixPak](https://github.com/nixpak/nixpak)**: Declarative bwrap wrapper for app sandboxing
- **[Nixwrap](https://github.com/rti/nixwrap)**: Easy sandboxing for common use cases
- **boxxy**: XDG path redirection (already in analysis)

### Pixi Security
The `run-post-link-scripts = "insecure"` setting in `.pixi/config.toml` allows conda post-link scripts to run. This is required for ROS but be aware:
- Only use trusted channels (`robostack-humble`, `conda-forge`)
- Review scripts in unfamiliar packages

### AI Tool Security
| Tool | Consideration |
|------|---------------|
| aichat | API keys stored in `~/.config/aichat/config.yaml` - protect with permissions |
| Aider | Git commits are automatic - review before pushing |
| cc-mirror | Isolated instances help contain API key exposure |

---

## Analysis Update Automation Plan

### Overview

This analysis document should be kept in sync with the codebase. The following automation strategies ensure accuracy and reduce manual maintenance.

### Strategy 1: Git Hooks (Local)

**Pre-commit hook** to validate analysis matches implementation:

```bash
#!/bin/bash
# .git/hooks/pre-commit

# Check if key files changed
if git diff --cached --name-only | grep -qE "(flake.nix|pixi.toml|modules/)"; then
  echo "⚠️  Configuration changed - consider updating docs/TOOLING-ANALYSIS.md"
fi
```

### Strategy 2: GitHub Actions (CI)

**Workflow to auto-generate sections:**

```yaml
# .github/workflows/update-analysis.yml
name: Update Tooling Analysis

on:
  push:
    paths:
      - 'flake.nix'
      - 'pixi.toml'
      - 'modules/**/*.nix'
  schedule:
    - cron: '0 0 1 * *'  # Monthly

jobs:
  update-analysis:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Extract package list from flake.nix
        run: |
          # Parse commonPackages from flake.nix
          grep -A 100 'commonPackages = with pkgs;' flake.nix | \
            grep -oP '^\s+\K\w+' | head -50 > /tmp/nix-packages.txt

      - name: Extract pixi dependencies
        run: |
          # Parse dependencies from pixi.toml
          grep -E '^\w+ = ' pixi.toml | cut -d= -f1 > /tmp/pixi-deps.txt

      - name: Check for tool version updates
        run: |
          # Check GitHub releases for tracked tools
          for tool in aichat aider zed; do
            gh api repos/sigoden/$tool/releases/latest --jq '.tag_name' 2>/dev/null || true
          done

      - name: Update timestamp
        run: |
          sed -i "s/Last updated:.*/Last updated: $(date +'%B %Y')/" docs/TOOLING-ANALYSIS.md
```

### Strategy 3: Claude Code Slash Command

**Create a slash command for on-demand updates:**

```markdown
<!-- .claude/commands/update-analysis.md -->
Analyze the current codebase and update docs/TOOLING-ANALYSIS.md:

1. Read flake.nix and extract:
   - All inputs (nixpkgs version, flake-parts, etc.)
   - All packages in commonPackages, linuxPackages, darwinPackages
   - All devshell variants
   - All exported modules

2. Read pixi.toml and extract:
   - All dependencies with versions
   - Channels configured

3. Scan modules/ directory for:
   - All .nix files and their purposes
   - New modules added since last update

4. Update the following sections:
   - Current Architecture Details (file tree)
   - Implementation Status table
   - Package tables (pixi.toml, flake.nix)

5. Check tool versions against latest releases:
   - aichat: https://github.com/sigoden/aichat/releases
   - aider: https://github.com/Aider-AI/aider/releases
   - zed: https://zed.dev/releases/stable

6. Update "Last updated" timestamp
```

### Strategy 4: Nix-based Verification

**Add a check to flake.nix:**

```nix
checks = {
  analysis-freshness = pkgs.runCommand "check-analysis" {} ''
    # Verify analysis document exists and is recent
    if [ ! -f ${./docs/TOOLING-ANALYSIS.md} ]; then
      echo "ERROR: TOOLING-ANALYSIS.md not found"
      exit 1
    fi

    # Check if key packages are documented
    for pkg in aichat aider-chat helix; do
      if ! grep -q "$pkg" ${./docs/TOOLING-ANALYSIS.md}; then
        echo "WARNING: $pkg not documented in analysis"
      fi
    done

    touch $out
  '';
};
```

### Recommended Update Triggers

| Event | Action |
|-------|--------|
| New package added to flake.nix | Update package tables |
| New module created in modules/ | Update architecture tree |
| pixi.toml dependencies changed | Update pixi tables |
| Monthly schedule | Check for tool version updates |
| Major release | Full review and research update |

### Automation Priority

1. **Immediate**: Add slash command for manual updates
2. **Short-term**: Implement pre-commit hook
3. **Medium-term**: Add GitHub Actions workflow
4. **Long-term**: Nix-based verification in CI

---

## Sources

### AI Coding Assistants
- [Aider GitHub](https://github.com/Aider-AI/aider)
- [Aider Documentation](https://aider.chat/docs/)
- [Aider Release History](https://aider.chat/HISTORY.html)
- [Zed Editor](https://zed.dev/)
- [Zed 2025 Recap](https://zed.dev/2025)
- [Zed Stable Releases](https://zed.dev/releases/stable)
- [Zed NixOS Wiki](https://wiki.nixos.org/wiki/Zed)

### Environment Tools
- [mise-en-place](https://mise.jdx.dev/)
- [mise vs asdf comparison](https://mise.jdx.dev/dev-tools/comparison-to-asdf.html)
- [DevPod](https://devpod.sh/)
- [DevPod GitHub](https://github.com/loft-sh/devpod)
- [lucernae/devcontainer-nix](https://github.com/lucernae/devcontainer-nix)
- [xtruder/nix-devcontainer](https://github.com/xtruder/nix-devcontainer)

### ROS2 + Nix
- [nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay)
- [RoboStack](https://robostack.github.io/)
- [RoboStack ros-jazzy](https://github.com/RoboStack/ros-jazzy)
- [pixi-build-ros backend](https://prefix-dev.github.io/pixi-build-backends/backends/pixi-build-ros/)
- [Using Pixi for ROS 2 Dev (2025)](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/)

### Dotfile Management
- [chezmoi](https://www.chezmoi.io/)
- [home-manager GitHub](https://github.com/nix-community/home-manager)
- [xdg-ninja GitHub](https://github.com/b3nj5m1n/xdg-ninja)
- [boxxy GitHub](https://github.com/queer/boxxy)

### AI CLI Tools
- [aichat GitHub](https://github.com/sigoden/aichat)
- [aichat Docs.rs](https://docs.rs/crate/aichat/latest)
- [cc-mirror GitHub](https://github.com/numman-ali/cc-mirror)

### Neovim Plugins
- [live-preview.nvim](https://github.com/brianhuster/live-preview.nvim)
- [peek.nvim](https://github.com/toppair/peek.nvim)
- [LazyVim Markdown](http://www.lazyvim.org/extras/lang/markdown)

### Security & Best Practices
- [Nix Best Practices at Work](https://determinate.systems/blog/best-practices-for-nix-at-work/)
- [Nix Flake Checker](https://determinate.systems/blog/flake-checker/)
- [NixPak Runtime Sandboxing](https://github.com/nixpak/nixpak)
- [NixOS Security Wiki](https://nixos.wiki/wiki/Security)
- [Nix Flakes Explained (2025)](https://determinate.systems/blog/nix-flakes-explained/)

---

*Last updated: January 2026*
