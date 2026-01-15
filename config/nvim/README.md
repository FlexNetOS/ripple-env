# Ripple-env Neovim/LazyVim Configuration

This directory contains a pre-configured Neovim setup optimized for the ripple-env project.

## Features

- **TypeScript/JavaScript**: Enhanced tooling with typescript-tools.nvim
- **Vue 3**: Full Volar support for Vue single-file components
- **Tailwind CSS**: IntelliSense and autocompletion
- **Formatting**: Prettier + ESLint integration
- **Git**: LazyGit integration
- **Project Navigation**: Telescope keymaps for frontend directories
- **Pixi Integration**: Terminal commands that run through pixi

## Installation

### Option A: Symlink (Recommended for dedicated ripple-env work)

```bash
# Backup existing config
mv ~/.config/nvim ~/.config/nvim.backup

# Symlink this config
ln -sf /path/to/ripple-env/config/nvim ~/.config/nvim
```

### Option B: Use NVIM_APPNAME (Multiple configs)

```bash
# Create a named config
ln -sf /path/to/ripple-env/config/nvim ~/.config/ripple-nvim

# Launch with this config
NVIM_APPNAME=ripple-nvim nvim

# Or add an alias
alias rnvim='NVIM_APPNAME=ripple-nvim nvim'
```

### Option C: Merge into existing LazyVim

Copy individual files to your LazyVim config:

```bash
cp lua/plugins/ripple-env.lua ~/.config/nvim/lua/plugins/
cp lua/config/keymaps.lua ~/.config/nvim/lua/config/  # merge if exists
```

## Keybindings

| Key | Action |
|-----|--------|
| `<leader>fr` | Live grep (repo-wide) |
| `<leader>ff` | Find files |
| `<leader>fR` | Grep in React sidebar |
| `<leader>fV` | Grep in Vue sidebar |
| `<leader>fK` | Grep in Vibe Kanban |
| `<leader>tp` | Pixi terminal shell |
| `<leader>ti` | pnpm install (via pixi) |
| `<leader>tb` | pnpm build (via pixi) |
| `<leader>td` | pnpm dev (via pixi) |
| `<leader>pr` | Open React layout |
| `<leader>pv` | Open Vue App |
| `<leader>pk` | Open Vibe Kanban README |
| `<leader>pp` | Open pixi.toml |
| `<leader>gg` | LazyGit |

## Dependencies

See [docs/frontend/LAZYVIM-DEPS.md](../../docs/frontend/LAZYVIM-DEPS.md) for full dependency list.

Required tools (installed via pixi/nix):
- Node.js 22 (LTS)
- pnpm
- ripgrep
- fd
- lazygit
- tree-sitter
- gcc/clang (for treesitter parsers)
