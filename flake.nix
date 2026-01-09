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
          inherit (pkgs.lib) optionalString optionals optionalAttrs;
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

            # Python 3.14.2 - Latest stable (for non-ROS2 tools)
            # ROS2 uses Python 3.11 via Pixi/RoboStack (separate environment)
            python313
            python313Packages.pip
            python313Packages.virtualenv

            # Nix tools
            nix-output-monitor
            nix-tree
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

            # Infrastructure & Monitoring (from GitHub resources research)
            # See docs/GITHUB-RESOURCES.md for full analysis
            prometheus          # Metrics collection for ROS2 DDS
            natscli             # NATS CLI for messaging
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
            sccache             # Distributed compilation cache (cloud support)used as a compiler wrapper and avoids compilation when possible. Sccache has the capability to utilize caching in remote storage environments, in local storage.
            mold                # Fast modern linker (12x faster than lld)
            maturin             # Build tool for PyO3 Rust-Python bindings

            # Database tools
            sqlx-cli            # SQL database CLI for migrations and schema management

            # Tree-sitter (for LazyVim/Neovim)
            tree-sitter

            # Node.js ecosystem (for LazyVim plugins & LLM testing)
            nodejs_22           # LTS "Jod" - active until Apr 2027
            nodePackages.pnpm
            # Note: promptfoo (LLM eval/testing) not in nixpkgs - use 'npx promptfoo@latest'

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
            (pkgs.writeShellScriptBin "promptfoo" ''
              # Wrapper for promptfoo LLM testing tool
              # Uses npx to run the latest version without global installation
              exec npx promptfoo@latest "$@"
            '')
          ];

          # CUDA 13.x package set (latest available in nixpkgs)
          # Falls back to default cudaPackages if 13.1 unavailable
          # Only defined on Linux where CUDA is available
          cudaPkgs = if isLinux then
            (pkgs.cudaPackages_13_1 or pkgs.cudaPackages_13 or pkgs.cudaPackages)
          else
            null;

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
              echo "  Python (Nix): ${pkgs.python313.version} (for scripts/tools)"
              echo "  Python (ROS2): 3.11.x via Pixi/RoboStack"
              echo ""
              echo "Quick commands:"
              echo "  cb     - colcon build --symlink-install"
              echo "  ct     - colcon test"
              echo "  pixi   - package manager (ROS2 Python env)"
              echo "  python3.13 - Nix Python for non-ROS tools"
              echo ""
              echo "AI assistants:"
              echo "  ai        - AI chat (aichat, lightweight)"
              echo "  pair      - AI pair programming (aider, git-integrated)"
              echo "  promptfoo - LLM testing & evaluation (robot command parsing)"
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
        }
        # CUDA-enabled shell for GPU workloads (Linux only)
        # Usage: nix develop .#cuda
        # Requires: NVIDIA GPU with drivers installed
        # Binary cache: https://cache.nixos-cuda.org
        // optionalAttrs isLinux {
          devShells.cuda = pkgs.mkShell {
            packages = commonPackages ++ commandWrappers ++ linuxPackages ++ (with pkgs; [
              # CUDA Toolkit 13.x (or latest available)
              # See docs/CONFLICTS.md for version details
              cudaPkgs.cudatoolkit
              cudaPkgs.cudnn
              cudaPkgs.cutensor
              cudaPkgs.nccl
              cudaPkgs.cuda_cudart

              # GCC 13 pinned for CUDA compatibility
              # CUDA requires specific GCC versions for nvcc
              gcc13

              # GPU monitoring
              nvtopPackages.full
            ]);

            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # CUDA environment variables
            CUDA_PATH = "${cudaPkgs.cudatoolkit}";
            # Pin compiler for CUDA compatibility
            CC = "${pkgs.gcc13}/bin/gcc";
            CXX = "${pkgs.gcc13}/bin/g++";

            shellHook = ''
              # Set up LD_LIBRARY_PATH for CUDA libraries
              export LD_LIBRARY_PATH="${cudaPkgs.cudatoolkit}/lib:${cudaPkgs.cudnn}/lib:$LD_LIBRARY_PATH"

              # Initialize pixi environment with CUDA feature
              if [ -f pixi.toml ]; then
                eval "$(pixi shell-hook -e cuda 2>/dev/null || pixi shell-hook)"
              fi

              # Initialize direnv
              eval "$(direnv hook bash)"

              # Initialize zoxide
              eval "$(zoxide init bash)"

              # Initialize starship prompt
              eval "$(starship init bash)"

              # Verify CUDA availability
              if command -v nvidia-smi &> /dev/null; then
                echo ""
                echo "🚀 ROS2 Humble + CUDA Development Environment"
                echo "=============================================="
                echo "  Platform: Linux (${system}) with NVIDIA GPU"
                nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>/dev/null | head -1 | while read line; do
                  echo "  GPU: $line"
                done
                echo "  CUDA: ${cudaPkgs.cudatoolkit.version}"
                echo "  GCC: $(${pkgs.gcc13}/bin/gcc --version | head -1)"
                echo "  Python (Nix): ${pkgs.python313.version}"
                echo "  Python (ROS2): via Pixi 3.11.x"
                echo ""
                echo "PyTorch CUDA verification:"
                echo "  python -c \"import torch; print(torch.cuda.is_available())\""
                echo ""
                echo "AI assistants:"
                echo "  ai        - AI chat (aichat, lightweight)"
                echo "  pair      - AI pair programming (aider, git-integrated)"
                echo "  promptfoo - LLM testing & evaluation (robot command parsing)"
                echo ""
              else
                echo ""
                echo "⚠️  Warning: nvidia-smi not found"
                echo "   CUDA toolkit is available but GPU drivers may not be installed."
                echo "   Install NVIDIA drivers on your host system."
                echo ""
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
