-- Ripple-env project-specific options for LazyVim

-- Use pixi-managed Node.js
vim.g.node_host_prog = vim.fn.exepath("node")

-- Project root detection
vim.g.root_spec = { "pixi.toml", "flake.nix", "package.json", ".git" }

-- Format on save
vim.g.autoformat = true

-- UI customization
vim.opt.relativenumber = true
vim.opt.number = true
vim.opt.signcolumn = "yes"
vim.opt.cursorline = true

-- Tabs/indentation (match project .editorconfig)
vim.opt.tabstop = 2
vim.opt.shiftwidth = 2
vim.opt.expandtab = true

-- Search
vim.opt.ignorecase = true
vim.opt.smartcase = true

-- File handling
vim.opt.undofile = true
vim.opt.swapfile = false

-- Enable clipboard integration
vim.opt.clipboard = "unnamedplus"
