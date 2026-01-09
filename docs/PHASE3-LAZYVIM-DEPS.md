# Phase 3: LazyVim/Neovim Dependencies

This document captures the dependencies required for implementing LazyVim + Neovim support in Phase 3.

## Required Dependencies

### 1. C Compiler (for nvim-treesitter)

nvim-treesitter requires a C compiler to build parsers. Options:

| Package | Platform | Notes |
|---------|----------|-------|
| `gcc` | Linux | GNU Compiler Collection |
| `clang` | All | LLVM-based, cross-platform |
| `compilers` | pixi | Already in pixi.toml (conda-forge) |

**Current Status**: ✅ `compilers` package in `pixi.toml` provides GCC/Clang.

**Nix Addition Needed**:
```nix
# In flake.nix or packages.nix
gcc
# or
clang
```

### 2. Nerd Fonts (for icons)

LazyVim uses icons that require Nerd Fonts. Options:

| Font | Package | Description |
|------|---------|-------------|
| JetBrains Mono | `nerdfonts` | Recommended for coding |
| Fira Code | `nerdfonts` | Popular ligature font |
| Hack | `nerdfonts` | Clean monospace |

**Nix Package**:
```nix
# Full nerd fonts (large)
nerdfonts

# Or specific fonts (recommended)
(nerdfonts.override { fonts = [ "JetBrainsMono" "FiraCode" "Hack" ]; })
```

**Alternative**: User installs fonts separately (document in README).

### 3. LuaJIT (for Neovim)

Neovim uses LuaJIT for plugin execution.

**Current Status**: ✅ Bundled with `pkgs.neovim` package.

No additional action needed.

### 4. Node.js (for some plugins)

Some plugins (like peek.nvim, copilot.vim) require Node.js.

| Plugin | Requires Node | Purpose |
|--------|---------------|---------|
| peek.nvim | Yes | Markdown preview |
| live-preview.nvim | Yes | Live HTML/MD preview |
| copilot.vim | Yes | GitHub Copilot |
| coc.nvim | Yes | Completion framework |

**Nix Package**:
```nix
nodejs
# or specific version
nodejs_20
```

### 5. Additional LSP Servers

Beyond what Helix uses, LazyVim may need:

| LSP | Language | Package |
|-----|----------|---------|
| lua-language-server | Lua | `lua-language-server` |
| stylua | Lua formatter | `stylua` |
| typescript-language-server | TypeScript | `nodePackages.typescript-language-server` |

### 6. Other Tools

| Tool | Purpose | Package |
|------|---------|---------|
| ripgrep | Telescope/grep | ✅ Already included |
| fd | File finding | ✅ Already included |
| lazygit | Git TUI | `lazygit` |
| tree-sitter | Syntax parsing | `tree-sitter` |

## Implementation Plan

### Option A: Minimal LazyVim Module

```nix
# modules/common/editor/neovim.nix
{ config, lib, pkgs, ... }:
let
  inherit (lib) mkEnableOption mkIf;
in
{
  options.programs.neovim-lazyvim = {
    enable = mkEnableOption "LazyVim Neovim configuration";
  };

  config = mkIf config.programs.neovim-lazyvim.enable {
    programs.neovim = {
      enable = true;
      defaultEditor = false;  # Keep helix as default
      viAlias = false;
      vimAlias = false;
      withNodeJs = true;
      withPython3 = true;
    };

    home.packages = with pkgs; [
      # Build tools for treesitter
      gcc

      # Fonts (user should install separately, but we provide)
      (nerdfonts.override { fonts = [ "JetBrainsMono" "FiraCode" "Hack" ]; })

      # Additional tools
      lazygit
      tree-sitter

      # LSPs for Lua
      lua-language-server
      stylua
    ];
  };
}
```

### Option B: Full LazyVim with nixCats

Use [nixCats](https://nixcats.org/) for proper Nix-native plugin management:

```nix
# Requires nixCats flake input
inputs.nixCats.url = "github:BirdeeHub/nixCats-nvim";
```

### Option C: External LazyVim (Document Only)

Document how users can install LazyVim alongside the devshell:

```bash
# User installs LazyVim manually
git clone https://github.com/LazyVim/starter ~/.config/nvim
```

## Dependencies Summary

| Dependency | Required | Currently Have | Action |
|------------|----------|----------------|--------|
| C/C++ Compiler | ✅ | ✅ (pixi `compilers`) | ✅ Done |
| tar | ✅ | ✅ (pixi + nix) | ✅ Added |
| curl | ✅ | ✅ (pixi + nix) | ✅ Added |
| LuaJIT | ✅ | ✅ (in neovim) | None needed |
| Nerd Fonts | ✅ | ❌ | Add optional package |
| Node.js | ✅ | ✅ (pixi + nix) | ✅ Added nodejs_22 (LTS) |
| pnpm | ✅ | ✅ (pixi + nix) | ✅ Added |
| ripgrep | ✅ | ✅ | None needed |
| fd | ✅ | ✅ | None needed |
| tree-sitter CLI | ✅ | ✅ (nix) | ✅ Added |
| ccache | Optional | ✅ (pixi + nix) | ✅ Added |
| sccache | Optional | ✅ (pixi + nix) | ✅ Added |
| mold (fast linker) | Optional | ✅ (nix) | ✅ Added |
| lazygit | Optional | ✅ (nix) | ✅ Added |

## Already Added Packages

The following packages have been added to support tree-sitter and LazyVim:

```nix
# flake.nix / packages.nix - Already included:
commonPackages = with pkgs; [
  # Archive/Network (explicit for tree-sitter)
  gnutar
  curl
  wget
  unzip
  gzip

  # Build tools & compilation cache
  ccache              # Fast C/C++ compilation cache
  sccache             # Distributed compilation cache (cloud support)
  mold                # Fast modern linker (12x faster than lld)

  # Tree-sitter (for LazyVim/Neovim)
  tree-sitter

  # Node.js ecosystem (for LazyVim plugins)
  nodejs_22           # LTS "Jod" - active until Apr 2027
  nodePackages.pnpm
];
```

```toml
# pixi.toml - Already included:
ccache = ">=4.10,<5"
sccache = ">=0.8,<1"
tar = ">=1.34"
curl = ">=8.0"
nodejs = ">=22.0,<23"    # LTS "Jod" - active until Apr 2027
pnpm = ">=9.0"
```

## Still Needed for Full LazyVim

```nix
# Optional additions for complete LazyVim experience:

# Fonts (optional, large package - ~500MB for single font)
# User should install Nerd Fonts manually or add:
(nerdfonts.override { fonts = [ "JetBrainsMono" "FiraCode" "Hack" ]; })
```

**Note:** lazygit is now included in the devshell.

## Plugins Mentioned in User Notes

From user's Phase 3 request:

1. **live-preview.nvim** - Live preview for markdown/HTML
   - Repo: https://github.com/brianhuster/live-preview.nvim
   - Requires: Node.js, browser

2. **peek.nvim** - Markdown preview in floating window
   - Repo: https://github.com/toppair/peek.nvim
   - Requires: Node.js, deno (optional)

## Font Installation Note

Nerd Fonts are large (~3GB for full package). Recommend:

1. **Per-user install**: User downloads from https://www.nerdfonts.com/
2. **Minimal Nix install**: Only include one font family
3. **System-level**: For NixOS, configure in `fonts.fonts`

## References

- [LazyVim Documentation](https://www.lazyvim.org/)
- [nvim-treesitter Requirements](https://github.com/nvim-treesitter/nvim-treesitter#requirements)
- [nixCats Documentation](https://nixcats.org/)
- [lazy-nix-helper.nvim](https://github.com/b-src/lazy-nix-helper.nvim)
- [Nerd Fonts](https://www.nerdfonts.com/)
