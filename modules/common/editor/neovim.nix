# Optional AI-enhanced Neovim with LazyVim
# Provides Claude integration and live markdown preview
# Enable with: programs.neovim-ai.enable = true
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkEnableOption mkOption mkIf types;
  cfg = config.programs.neovim-ai;
in
{
  options.programs.neovim-ai = {
    enable = mkEnableOption "AI-enhanced Neovim with LazyVim and Claude integration";

    defaultEditor = mkOption {
      type = types.bool;
      default = false;
      description = "Set Neovim as default EDITOR/VISUAL (overrides Helix)";
    };

    claudeProvider = mkOption {
      type = types.enum [ "codecompanion" "avante" "none" ];
      default = "codecompanion";
      description = ''
        Claude integration plugin:
        - codecompanion: Chat + inline edits (recommended)
        - avante: Cursor-like experience (more complex)
        - none: No Claude integration
      '';
    };

    enableMarkdownPreview = mkOption {
      type = types.bool;
      default = true;
      description = "Enable live-preview.nvim for Markdown/HTML preview";
    };
  };

  config = mkIf cfg.enable {
    programs.neovim = {
      enable = true;
      defaultEditor = cfg.defaultEditor;
      viAlias = false;
      vimAlias = false;
      withNodeJs = true;
      withPython3 = true;

      # Runtime dependencies
      extraPackages = with pkgs; [
        # Lua development (for Neovim config)
        lua-language-server
        stylua

        # Tree-sitter CLI
        tree-sitter
        gcc # For compiling parsers

        # Search tools (used by Telescope)
        ripgrep
        fd

        # Git TUI (LazyVim integration)
        lazygit
      ];

      # LazyVim-compatible configuration
      extraLuaConfig = ''
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

        -- Leader key must be set before lazy
        vim.g.mapleader = " "
        vim.g.maplocalleader = "\\"

        -- Setup lazy.nvim with LazyVim
        require("lazy").setup({
          spec = {
            -- LazyVim core
            {
              "LazyVim/LazyVim",
              import = "lazyvim.plugins",
              opts = {
                colorscheme = "gruvbox",
              },
            },

            -- Gruvbox theme (matches Helix)
            { "ellisonleao/gruvbox.nvim", priority = 1000 },

            -- Disable Mason (use Nix LSP servers)
            { "williamboman/mason.nvim", enabled = false },
            { "williamboman/mason-lspconfig.nvim", enabled = false },

            ${if cfg.claudeProvider == "codecompanion" then ''
            -- Claude integration via codecompanion.nvim
            {
              "olimorris/codecompanion.nvim",
              dependencies = {
                "nvim-lua/plenary.nvim",
                "nvim-treesitter/nvim-treesitter",
              },
              config = function()
                require("codecompanion").setup({
                  strategies = {
                    chat = {
                      adapter = "anthropic",
                    },
                    inline = {
                      adapter = "anthropic",
                    },
                  },
                  adapters = {
                    anthropic = function()
                      return require("codecompanion.adapters").extend("anthropic", {
                        env = {
                          api_key = "ANTHROPIC_API_KEY",
                        },
                      })
                    end,
                  },
                })
              end,
              keys = {
                { "<leader>cc", "<cmd>CodeCompanionChat<cr>", desc = "Claude Chat" },
                { "<leader>ca", "<cmd>CodeCompanionActions<cr>", desc = "Claude Actions" },
                { "<leader>ct", "<cmd>CodeCompanionToggle<cr>", desc = "Toggle Claude" },
              },
            },
            '' else if cfg.claudeProvider == "avante" then ''
            -- Claude integration via avante.nvim (Cursor-like)
            {
              "yetone/avante.nvim",
              event = "VeryLazy",
              build = "make",
              dependencies = {
                "nvim-treesitter/nvim-treesitter",
                "stevearc/dressing.nvim",
                "nvim-lua/plenary.nvim",
                "MunifTanjim/nui.nvim",
              },
              opts = {
                provider = "claude",
                claude = {
                  model = "claude-sonnet-4-20250514",
                },
              },
            },
            '' else "-- No Claude integration enabled"}

            ${if cfg.enableMarkdownPreview then ''
            -- Markdown live preview (zero deps, pure Lua)
            {
              "brianhuster/live-preview.nvim",
              ft = { "markdown", "html", "asciidoc" },
              config = function()
                require("live-preview").setup({
                  port = 5500,
                  browser = "default",
                })
              end,
              keys = {
                { "<leader>mp", "<cmd>LivePreview<cr>", desc = "Markdown Preview" },
                { "<leader>ms", "<cmd>StopPreview<cr>", desc = "Stop Preview" },
              },
            },
            '' else "-- Markdown preview disabled"}
          },

          -- Performance optimizations
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

          -- Don't auto-install (use Nix)
          install = {
            missing = true,  -- Still install plugins, just not LSPs
          },

          checker = {
            enabled = false,  -- Don't check for plugin updates
          },
        })

        -- Additional keymaps for AI workflow
        vim.keymap.set("n", "<leader>ai", function()
          vim.cmd("terminal aichat")
        end, { desc = "Open aichat" })

        vim.keymap.set("n", "<leader>ap", function()
          vim.cmd("terminal aider %")
        end, { desc = "Open aider with current file" })
      '';
    };

    # Shell alias for clarity
    home.shellAliases = {
      nvim-ai = "nvim";
      vim = "nvim"; # Override vim to use AI-enhanced Neovim
    };
  };
}
