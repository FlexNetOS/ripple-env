-- Ripple-env LazyVim Configuration
-- This file bootstraps LazyVim with project-specific settings
--
-- USAGE:
-- Option A: Symlink this config
--   ln -sf /path/to/ripple-env/config/nvim ~/.config/nvim
--
-- Option B: Use NVIM_APPNAME
--   NVIM_APPNAME=ripple-nvim nvim
--   (with config at ~/.config/ripple-nvim -> /path/to/ripple-env/config/nvim)
--
-- Option C: Merge into existing LazyVim config
--   Copy lua/plugins/ripple-env.lua and lua/config/*.lua to your LazyVim

-- Bootstrap lazy.nvim
local lazypath = vim.fn.stdpath("data") .. "/lazy/lazy.nvim"
if not vim.loop.fs_stat(lazypath) then
  vim.fn.system({
    "git",
    "clone",
    "--filter=blob:none",
    "https://github.com/folke/lazy.nvim.git",
    "--branch=stable",
    lazypath,
  })
end
vim.opt.rtp:prepend(lazypath)

-- Load options first
require("config.options")

-- Setup lazy.nvim with LazyVim
require("lazy").setup({
  spec = {
    -- Import LazyVim
    { "LazyVim/LazyVim", import = "lazyvim.plugins" },
    -- Import LazyVim extras
    { import = "lazyvim.plugins.extras.lang.typescript" },
    { import = "lazyvim.plugins.extras.lang.rust" },
    { import = "lazyvim.plugins.extras.lang.json" },
    { import = "lazyvim.plugins.extras.lang.tailwind" },
    { import = "lazyvim.plugins.extras.formatting.prettier" },
    { import = "lazyvim.plugins.extras.linting.eslint" },
    -- Import project-specific plugins
    { import = "plugins" },
  },
  defaults = {
    lazy = false,
    version = false,
  },
  install = { colorscheme = { "tokyonight", "habamax" } },
  checker = { enabled = true },
  performance = {
    rtp = {
      disabled_plugins = {
        "gzip",
        "tarPlugin",
        "tohtml",
        "tutor",
        "zipPlugin",
      },
    },
  },
})

-- Load keymaps after plugins
require("config.keymaps")
