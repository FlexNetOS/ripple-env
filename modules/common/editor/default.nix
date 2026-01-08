# Editor configuration
# Helix editor setup with ROS2-relevant language servers
# Based on GustavoWidman/nix and RGBCube/ncc patterns
{
  config,
  lib,
  pkgs,
  ...
}:
let
  inherit (lib) mkDefault;
in
{
  programs.helix = {
    enable = mkDefault true;

    # Default editor setting
    defaultEditor = mkDefault true;

    # Helix configuration
    settings = {
      theme = mkDefault "gruvbox_dark_hard";

      editor = {
        # Line numbers
        line-number = "relative";
        cursorline = true;

        # Cursor shape
        cursor-shape = {
          insert = "bar";
          normal = "block";
          select = "underline";
        };

        # File picker
        file-picker = {
          hidden = false;
          git-ignore = true;
        };

        # Status line
        statusline = {
          left = [
            "mode"
            "spinner"
            "file-name"
            "file-modification-indicator"
          ];
          center = [ ];
          right = [
            "diagnostics"
            "selections"
            "register"
            "position"
            "file-encoding"
          ];
        };

        # Whitespace rendering
        whitespace.render = {
          space = "none";
          tab = "all";
          newline = "none";
        };

        # Indentation guides
        indent-guides = {
          render = true;
          character = "│";
        };

        # LSP settings
        lsp = {
          display-messages = true;
          display-inlay-hints = true;
        };

        # Auto features
        auto-completion = true;
        auto-format = true;
        auto-save = false;

        # Soft wrap for long lines
        soft-wrap.enable = true;

        # Bufferline
        bufferline = "multiple";
      };

      keys = {
        normal = {
          # Quick save
          "C-s" = ":write";

          # Window navigation
          "C-h" = "jump_view_left";
          "C-j" = "jump_view_down";
          "C-k" = "jump_view_up";
          "C-l" = "jump_view_right";

          # Buffer navigation
          "H" = ":buffer-previous";
          "L" = ":buffer-next";

          # Quick close
          "C-q" = ":quit";
        };

        insert = {
          # Escape alternatives
          "j" = {
            "k" = "normal_mode";
          };
        };
      };
    };

    # Language configurations
    languages = {
      language = [
        # Python - essential for ROS2
        {
          name = "python";
          auto-format = true;
          formatter = {
            command = "ruff";
            args = [ "format" "-" ];
          };
          language-servers = [
            "pyright"
            "ruff"
          ];
        }

        # C++ - core ROS2 language
        {
          name = "cpp";
          auto-format = true;
          language-servers = [ "clangd" ];
        }

        # C
        {
          name = "c";
          auto-format = true;
          language-servers = [ "clangd" ];
        }

        # CMake - ROS2 build system
        {
          name = "cmake";
          auto-format = true;
          language-servers = [ "cmake-language-server" ];
        }

        # Nix
        {
          name = "nix";
          auto-format = true;
          formatter = {
            command = "nixfmt";
          };
          language-servers = [ "nil" ];
        }

        # YAML - ROS2 config files
        {
          name = "yaml";
          auto-format = true;
          language-servers = [ "yaml-language-server" ];
        }

        # XML - ROS2 launch files, URDF
        {
          name = "xml";
          auto-format = false;
          language-servers = [ "lemminx" ];
        }

        # Rust
        {
          name = "rust";
          auto-format = true;
          language-servers = [ "rust-analyzer" ];
        }

        # Bash
        {
          name = "bash";
          auto-format = true;
          language-servers = [ "bash-language-server" ];
        }

        # TOML - Cargo, pixi config
        {
          name = "toml";
          auto-format = true;
          language-servers = [ "taplo" ];
        }

        # Markdown
        {
          name = "markdown";
          auto-format = false;
          language-servers = [ "marksman" ];
        }
      ];

      # Language server configurations
      language-server = {
        # Python servers
        pyright = {
          command = "pyright-langserver";
          args = [ "--stdio" ];
          config = { };
        };
        ruff = {
          command = "ruff";
          args = [ "server" ];
        };

        # C/C++ server
        clangd = {
          command = "clangd";
          args = [
            "--background-index"
            "--clang-tidy"
            "--completion-style=detailed"
            "--header-insertion=iwyu"
          ];
        };

        # CMake server
        cmake-language-server = {
          command = "cmake-language-server";
        };

        # Nix server
        nil = {
          command = "nil";
          config.nil = {
            formatting.command = [ "nixfmt" ];
          };
        };

        # YAML server
        yaml-language-server = {
          command = "yaml-language-server";
          args = [ "--stdio" ];
          config.yaml = {
            validate = true;
            schemas = {
              # ROS2 schemas could be added here
            };
          };
        };

        # XML server (for URDF, launch files)
        lemminx = {
          command = "lemminx";
        };

        # Rust server
        rust-analyzer = {
          command = "rust-analyzer";
          config.rust-analyzer = {
            checkOnSave.command = "clippy";
            cargo.features = "all";
          };
        };

        # Bash server
        bash-language-server = {
          command = "bash-language-server";
          args = [ "start" ];
        };

        # TOML server
        taplo = {
          command = "taplo";
          args = [ "lsp" "stdio" ];
        };

        # Markdown server
        marksman = {
          command = "marksman";
          args = [ "server" ];
        };
      };
    };

    # Runtime dependencies (LSP servers)
    extraPackages = with pkgs; [
      # Python
      pyright
      ruff

      # C/C++
      clang-tools # clangd

      # CMake
      cmake-language-server

      # Nix
      nil
      nixfmt-rfc-style

      # YAML
      yaml-language-server

      # XML
      lemminx

      # Rust
      rust-analyzer

      # Bash
      bash-language-server

      # TOML
      taplo

      # Markdown
      marksman
    ];
  };
}
