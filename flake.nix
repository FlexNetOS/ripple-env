{
  description = "ROS2 Humble development environment with Nix flakes and pixi";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";

    # Home-manager for user configuration
    home-manager = {
      url = "github:nix-community/home-manager";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs@{
      self,
      nixpkgs,
      flake-parts,
      systems,
      home-manager,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;

      # Flake-level outputs (not per-system)
      flake = {
        # Export library functions
        lib = (import ./lib { inherit (nixpkgs) lib; inherit inputs; }) // {
          # Home-manager modules are not a standard flake output; expose them under lib
          # to avoid warnings like: "unknown flake output 'homeManagerModules'".
          homeManagerModules = {
            common = ./modules/common;
            linux = ./modules/linux;
            macos = ./modules/macos;

            # Combined module that auto-selects based on platform
            default =
              { config, lib, pkgs, ... }:
              {
                imports = [
                  ./modules/common
                ] ++ lib.optionals pkgs.stdenv.isLinux [
                  ./modules/linux
                ] ++ lib.optionals pkgs.stdenv.isDarwin [
                  ./modules/macos
                ];
              };
          };
        };

        # Export NixOS/Darwin modules (for system-level configuration)
        nixosModules.default = ./modules/linux;
        darwinModules.default = ./modules/macos;
      };

      # Per-system outputs
      perSystem =
        { pkgs, system, ... }:
        let
          inherit (pkgs.lib) optionalString optionals;
          isDarwin = pkgs.stdenv.isDarwin;
          isLinux = pkgs.stdenv.isLinux;

          # Colcon defaults configuration
          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                - -Wno-dev
                ${optionalString isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';

          # Keep the default shell lightweight for fast `direnv` / `nix develop`.
          # Put heavier/optional tools into `devShells.full`.
          basePackages = with pkgs; [
            pixi
            git
            gh

            # Nix tooling
            nixfmt-rfc-style
            nil

            # Useful basics
            curl
            jq
            direnv
            nix-direnv
          ];

          fullExtras = with pkgs; [
            nix-output-monitor
            nix-tree

            # Shell utilities
            bat
            eza
            fd
            ripgrep
            fzf
            yq
            gnutar
            wget
            unzip
            gzip

            # Directory navigation
            zoxide

            # System monitoring
            btop
            htop

            # Shells
            zsh
            nushell

            # Editor
            helix

            # Prompt
            starship

            # AI assistants
            aichat
            aider-chat

            # Audio (for aider voice features)
            portaudio

            # Build tools & compilation cache
            ccache
            sccache
            mold

            # Tree-sitter (for LazyVim/Neovim)
            tree-sitter

            # Node.js ecosystem (for LazyVim plugins)
            nodejs_22
            nodePackages.pnpm

            # Git tools
            lazygit
          ];

          # Linux-specific packages
          linuxPackages = with pkgs; optionals isLinux [
            inotify-tools
            strace
            gdb
          ];

          # macOS-specific packages
          darwinPackages = with pkgs; optionals isDarwin [
            coreutils
            gnused
            gawk
          ];

          # Provide common helper commands as real executables (not shell functions), so they
          # are available when CI uses `nix develop --command ...`.
          coreCommandWrappers = [
            (pkgs.writeShellScriptBin "cb" ''
              exec colcon build --symlink-install "$@"
            '')
            (pkgs.writeShellScriptBin "ct" ''
              exec colcon test "$@"
            '')
            (pkgs.writeShellScriptBin "ctr" ''
              exec colcon test-result --verbose
            '')
            (pkgs.writeShellScriptBin "ros2-env" ''
              env | grep -E '^(ROS|RMW|AMENT|COLCON)' | sort
            '')
            (pkgs.writeShellScriptBin "update-deps" ''
              exec pixi update
            '')
          ];

          aiCommandWrappers = [
            (pkgs.writeShellScriptBin "ai" ''
              exec aichat "$@"
            '')
            (pkgs.writeShellScriptBin "pair" ''
              if command -v aider >/dev/null 2>&1; then
                exec aider "$@"
              elif command -v aider-chat >/dev/null 2>&1; then
                exec aider-chat "$@"
              else
                echo "Neither 'aider' nor 'aider-chat' is available in PATH" >&2
                exit 127
              fi
            '')
          ];

        in
        {
          # Development shell (main entry point)
          # Use standard `devShells` (and avoid the devshell flake module) so `nix flake check`
          # stays warning-free on newer Nix.
          devShells.default = pkgs.mkShell {
            packages = basePackages ++ coreCommandWrappers ++ linuxPackages ++ darwinPackages;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            shellHook = ''
              # Initialize pixi environment
              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook)"
              fi

              # Keep startup fast for non-interactive shells (CI, `nix develop --command ...`).
              if [[ $- == *i* ]]; then
                echo ""
                echo "ROS2 Humble Development Environment"
                echo "=================================="
                echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
                echo ""
              fi
            '';
          };

          # Full-featured shell (slower initial download, more tools)
          devShells.full = pkgs.mkShell {
            packages = basePackages ++ fullExtras ++ coreCommandWrappers ++ aiCommandWrappers ++ linuxPackages ++ darwinPackages;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            shellHook = ''
              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook)"
              fi
            '';
          };

          # Minimal shell for CI
          devShells.ci = pkgs.mkShell {
            packages = with pkgs; [
              pixi
              git
            ];
            COLCON_DEFAULTS_FILE = toString colconDefaults;

            shellHook = ''
              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook)"
              fi
            '';
          };

          # Formatter for nix files
          formatter = pkgs.nixfmt-rfc-style;

          # Check flake
          checks = {
            # Verify the devshell builds
            devshell = self.devShells.${system}.ci;
          };
        };
    };
}
