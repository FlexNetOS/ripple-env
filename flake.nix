{
  description = "ROS2 Humble development environment with Nix flakes and pixi";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";

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
      devshell,
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

          # Common packages for the development shell
          commonPackages = with pkgs; [
            # Core tools
            pixi
            git
            gh

            # Nix tools
            nix-output-monitor
            nix-tree
            nixfmt-rfc-style
            nil

            # Shell utilities
            bat
            eza
            fd
            ripgrep
            fzf
            jq
            yq

            # Archive/Network (explicit for tree-sitter)
            gnutar
            curl
            wget
            unzip
            gzip

            # Directory navigation
            zoxide
            direnv
            nix-direnv

            # System monitoring
            btop
            htop

            # Infrastructure & Monitoring (from GitHub resources research)
            # See docs/GITHUB-RESOURCES.md for full analysis
            prometheus          # Metrics collection for ROS2 DDS
            nats-server         # WAN/multi-site robot messaging
            trippy              # Network diagnostics for DDS traffic
            trivy               # Container/SBOM security scanning
            opa                 # Policy enforcement for ROS2 topics

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

            # AI inference (edge/local models)
            # local-ai           # Uncomment when needed - large package (~2GB)
                                 # Alternative: docker run -p 8080:8080 localai/localai

            # Audio (for aider voice features)
            portaudio

            # Build tools & compilation cache
            ccache              # Fast C/C++ compilation cache
            sccache             # Distributed compilation cache (cloud support)
            mold                # Fast modern linker (12x faster than lld)

            # Tree-sitter (for LazyVim/Neovim)
            tree-sitter

            # Node.js ecosystem (for LazyVim plugins)
            nodejs_22           # LTS "Jod" - active until Apr 2027
            nodePackages.pnpm

            # Git tools
            lazygit             # Git TUI (integrates with LazyVim)
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
          commandWrappers = [
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
            packages = commonPackages ++ commandWrappers ++ linuxPackages ++ darwinPackages;
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

              # Initialize direnv
              eval "$(direnv hook bash)"

              # Initialize zoxide
              eval "$(zoxide init bash)"

              # Initialize starship prompt
              eval "$(starship init bash)"

              # ROS2 environment info
              echo ""
              echo "🤖 ROS2 Humble Development Environment"
              echo "======================================"
              echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
              echo "  Shell: bash (use 'zsh' or 'nu' for other shells)"
              echo ""
              echo "Quick commands:"
              echo "  cb     - colcon build --symlink-install"
              echo "  ct     - colcon test"
              echo "  pixi   - package manager"
              echo ""
              echo "AI assistants:"
              echo "  ai     - AI chat (aichat, lightweight)"
              echo "  pair   - AI pair programming (aider, git-integrated)"
              echo ""
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
            devshell = self.devShells.${system}.default;
          };
        };
    };
}
