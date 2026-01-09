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

    # Holochain overlay for P2P coordination (BUILDKIT_STARTER_SPEC.md L11)
    # Provides: holochain, hc, lair-keystore
    # NOTE: This overlay is not a flake - it's loaded via fetchFromGitHub in the overlay section
    # See: https://github.com/spartan-holochain-counsel/nix-overlay
    # holochain-nix = {
    #   url = "github:spartan-holochain-counsel/nix-overlay";
    #   inputs.nixpkgs.follows = "nixpkgs";
    # };
    holochain-nix = {
      url = "github:spartan-holochain-counsel/nix-overlay";
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
      holochain-nix,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;

      # Flake-level outputs (not per-system)
      flake = {
        # Export library functions
        lib =
          (import ./lib {
            inherit (nixpkgs) lib;
            inherit inputs;
          })
          // {
            # Home-manager modules are not a standard flake output; expose them under lib
            # to avoid warnings like: "unknown flake output 'homeManagerModules'".
            homeManagerModules = {
              common = ./modules/common;
              linux = ./modules/linux;
              macos = ./modules/macos;

              # Combined module that auto-selects based on platform
              default =
                {
                  config,
                  lib,
                  pkgs,
                  ...
                }:
                {
                  imports = [
                    ./modules/common
                  ]
                  ++ lib.optionals pkgs.stdenv.isLinux [
                    ./modules/linux
                  ]
                  ++ lib.optionals pkgs.stdenv.isDarwin [
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
          # Configure nixpkgs with allowUnfree for packages like vault (BSL license)
          # Define the holochain source
          # Note: Using commit hash as the upstream repo has no tagged releases yet
          holochainSrc = inputs.nixpkgs.legacyPackages.${system}.fetchFromGitHub {
            owner = "spartan-holochain-counsel";
            repo = "nix-overlay";
            rev = "2a321bc7d6d94f169c6071699d9a89acd55039bb";  # Latest commit as of 2026-01-09
            sha256 = "sha256-LZkgXdLY+C+1CxynKzsdtM0g4gC0NJjPP3d24pHPyIU=";
          };

          # Apply Holochain overlay for P2P coordination
          pkgs = import inputs.nixpkgs {
            inherit system;
            config.allowUnfree = true;
            overlays = [
              # Holochain overlay - import the overlay function from the repository
              (import "${holochainSrc}/holochain-overlay/default.nix")
            ];
          };

          # Holochain packages from overlay (P0 - MANDATORY per BUILDKIT_STARTER_SPEC.md)
          holochainPackages = with pkgs; [
            holochain       # Holochain conductor (agent-centric P2P)
            hc              # Holochain dev CLI (scaffold/package/run)
            lair-keystore   # Secure keystore for Holochain agent keys
          ];

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

            # Python 3.13.x - Latest stable (for non-ROS2 tools)
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
            syft                # SBOM generation
            grype               # Vulnerability scanning from SBOM
            cosign              # Container image signing and verification
            opa                 # Policy enforcement for ROS2 topics
            kubectl             # Kubernetes CLI for cluster management
            helm                # Kubernetes package manager
            kustomize           # Kubernetes configuration management
            containerd          # Container runtime
            
            # Additional tools from BUILDKIT_STARTER_SPEC
            neovim              # Editor
            sqlite              # Local database

            # Secrets Management (see docs/GITHUB-RESOURCES.md)
            # Note: Vault uses BSL license - requires NIXPKGS_ALLOW_UNFREE=1
            # For dev mode: vault server -dev -dev-root-token-id root
            vault               # HashiCorp Vault for secrets management

            # P2P & Content-Addressed Storage (BUILDKIT_STARTER_SPEC.md L10-11)
            kubo                # IPFS implementation (content-addressed storage)

            # Supply Chain Security (BUILDKIT_STARTER_SPEC.md L18)
            syft                # SBOM generation (anchore)
            grype               # Vulnerability scanning from SBOM (anchore)
            cosign              # Container/artifact signing (sigstore)

            # PKI Automation (BUILDKIT_STARTER_SPEC.md L5)
            step-cli            # smallstep CLI for mTLS/PKI

            # Isolation & Sandboxing (BUILDKIT_STARTER_SPEC.md L3)
            firecracker         # MicroVM for untrusted workloads
            runc                # OCI container runtime (low-level)

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

            # AI inference (edge/local models) - LocalAI
            # Alternative: docker run -p 8080:8080 localai/localai
            # OpenAI-compatible API server for local LLM inference
            # Supports: GGUF, GGML, Safetensors models
            # P2P federation for multi-robot distributed inference
            # See: docs/adr/adr-006-agixt-integration.md
            local-ai

            # Audio (for aider voice features)
            portaudio

            # Build tools & compilation cache
            ccache              # Fast C/C++ compilation cache
            sccache             # Distributed compilation cache with cloud support
            mold                # Fast modern linker (12x faster than lld)
            maturin             # Build tool for PyO3 Rust-Python bindings

            # Rust toolchain (for AGiXT Rust SDK and ROS2 bridges)
            # See: rust/agixt-bridge/Cargo.toml
            cargo               # Rust package manager
            rustc               # Rust compiler
            rust-analyzer       # Rust LSP for editors
            rustfmt             # Rust code formatter
            clippy              # Rust linter

            # Database tools
            sqlx-cli # SQL database CLI for migrations and schema management

            # Tree-sitter (for LazyVim/Neovim)
            tree-sitter

            # Node.js ecosystem (for LazyVim plugins & LLM testing)
            nodejs_22 # LTS "Jod" - active until Apr 2027
            nodePackages.pnpm
            # Note: promptfoo (LLM eval/testing) not in nixpkgs - use 'npx promptfoo@latest'

            # Git tools
            lazygit             # Git TUI (integrates with LazyVim)

            # Remote development
            devpod              # Client-only devcontainer environments (any backend)
          ];

          # Linux-specific packages
          linuxPackages = with pkgs; optionals isLinux [
            inotify-tools
            strace
            gdb

            # Container security (sandboxing untrusted ROS2 packages)
            # Usage: docker run --runtime=runsc ...
            # See docs/GITHUB-RESOURCES.md for gVisor integration guide
            gvisor              # OCI runtime for container sandboxing
          ];

          # macOS-specific packages
          darwinPackages =
            with pkgs;
            optionals isDarwin [
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
              # aider-chat is the nixpkgs package name for aider
              # Check for aider-chat first (nixpkgs), then aider (pip install)
              if command -v aider-chat >/dev/null 2>&1; then
                exec aider-chat "$@"
              elif command -v aider >/dev/null 2>&1; then
                exec aider "$@"
              else
                echo "Neither 'aider-chat' nor 'aider' is available in PATH" >&2
                echo "Install via: nix develop .#full (includes aider-chat)" >&2
                exit 127
              fi
            '')
            (pkgs.writeShellScriptBin "promptfoo" ''
              # Wrapper for promptfoo LLM testing tool
              # Uses npx to run the latest version without global installation
              exec npx promptfoo@latest "$@"
            '')
            # LocalAI management commands
            (pkgs.writeShellScriptBin "localai" ''
              # LocalAI server management
              # Usage: localai [start|stop|status|models]
              LOCALAI_MODELS="''${LOCALAI_MODELS_PATH:-$HOME/.local/share/localai/models}"
              mkdir -p "$LOCALAI_MODELS"

              case "''${1:-start}" in
                start)
                  echo "Starting LocalAI on port 8080..."
                  echo "Models directory: $LOCALAI_MODELS"
                  exec local-ai --models-path "$LOCALAI_MODELS" --address ":8080" "''${@:2}"
                  ;;
                stop)
                  pkill -f "local-ai" && echo "LocalAI stopped" || echo "LocalAI not running"
                  ;;
                status)
                  if pgrep -f "local-ai" > /dev/null; then
                    echo "LocalAI is running"
                    curl -s http://localhost:8080/readyz && echo " - API ready"
                  else
                    echo "LocalAI is not running"
                  fi
                  ;;
                models)
                  echo "Available models in $LOCALAI_MODELS:"
                  ls -la "$LOCALAI_MODELS" 2>/dev/null || echo "  (none)"
                  ;;
                *)
                  echo "Usage: localai [start|stop|status|models]"
                  echo "  start  - Start LocalAI server (port 8080)"
                  echo "  stop   - Stop LocalAI server"
                  echo "  status - Check if LocalAI is running"
                  echo "  models - List available models"
                  ;;
              esac
            '')
            # AGiXT Docker Compose management
            (pkgs.writeShellScriptBin "agixt" ''
              # AGiXT management via Docker Compose
              # Usage: agixt [up|down|logs|status|shell]
              AGIXT_DIR="''${AGIXT_DIR:-$PWD}"
              COMPOSE_FILE="$AGIXT_DIR/docker-compose.agixt.yml"

              if [ ! -f "$COMPOSE_FILE" ]; then
                echo "Error: docker-compose.agixt.yml not found in $AGIXT_DIR"
                echo "Create it first or set AGIXT_DIR to the correct location"
                exit 1
              fi

              case "''${1:-status}" in
                up)
                  echo "Starting AGiXT services..."
                  docker compose -f "$COMPOSE_FILE" up -d "''${@:2}"
                  echo ""
                  echo "AGiXT API: http://localhost:7437"
                  echo "AGiXT UI:  http://localhost:3437"
                  ;;
                down)
                  echo "Stopping AGiXT services..."
                  docker compose -f "$COMPOSE_FILE" down "''${@:2}"
                  ;;
                logs)
                  docker compose -f "$COMPOSE_FILE" logs -f "''${@:2}"
                  ;;
                status)
                  docker compose -f "$COMPOSE_FILE" ps
                  ;;
                shell)
                  docker compose -f "$COMPOSE_FILE" exec agixt /bin/bash
                  ;;
                *)
                  echo "Usage: agixt [up|down|logs|status|shell]"
                  echo "  up     - Start AGiXT services (API, UI, DB, S3)"
                  echo "  down   - Stop AGiXT services"
                  echo "  logs   - Follow AGiXT logs"
                  echo "  status - Show service status"
                  echo "  shell  - Open shell in AGiXT container"
                  ;;
              esac
            '')
            # AIOS Agent OS kernel management
            # Usage: aios [start|stop|status|install]
            # AIOS runs as a FastAPI server on port 8000
            # See: https://github.com/agiresearch/AIOS
            (pkgs.writeShellScriptBin "aios" ''
              # AIOS Agent Kernel management
              # Requires: pixi run -e aios ...
              AIOS_DIR="''${AIOS_DIR:-$HOME/.local/share/aios}"
              AIOS_CONFIG="''${AIOS_CONFIG:-$AIOS_DIR/config/config.yaml}"
              AIOS_PORT="''${AIOS_PORT:-8000}"

              case "''${1:-status}" in
                install)
                  echo "Installing AIOS Agent Kernel..."
                  mkdir -p "$AIOS_DIR"
                  if [ ! -d "$AIOS_DIR/AIOS" ]; then
                    git clone https://github.com/agiresearch/AIOS.git "$AIOS_DIR/AIOS"
                  else
                    echo "AIOS already installed at $AIOS_DIR/AIOS"
                    echo "To update: cd $AIOS_DIR/AIOS && git pull"
                  fi
                  echo ""
                  echo "Install Cerebrum SDK:"
                  echo "  pip install aios-agent-sdk"
                  echo ""
                  echo "Configure API keys in: $AIOS_DIR/AIOS/aios/config/config.yaml"
                  ;;
                start)
                  echo "Starting AIOS Kernel on port $AIOS_PORT..."
                  if [ ! -d "$AIOS_DIR/AIOS" ]; then
                    echo "AIOS not installed. Run: aios install"
                    exit 1
                  fi
                  cd "$AIOS_DIR/AIOS"
                  echo "Using pixi AIOS environment..."
                  pixi run -e aios python -m uvicorn runtime.launch:app --host 0.0.0.0 --port "$AIOS_PORT" "''${@:2}"
                  ;;
                stop)
                  pkill -f "uvicorn runtime.launch:app" && echo "AIOS Kernel stopped" || echo "AIOS Kernel not running"
                  ;;
                status)
                  if pgrep -f "uvicorn runtime.launch:app" > /dev/null; then
                    echo "AIOS Kernel is running on port $AIOS_PORT"
                    curl -s "http://localhost:$AIOS_PORT/docs" > /dev/null && echo "  API ready: http://localhost:$AIOS_PORT/docs"
                  else
                    echo "AIOS Kernel is not running"
                    echo "  Install: aios install"
                    echo "  Start:   aios start"
                  fi
                  ;;
                config)
                  echo "AIOS Configuration:"
                  echo "  Directory: $AIOS_DIR"
                  echo "  Config:    $AIOS_CONFIG"
                  echo "  Port:      $AIOS_PORT"
                  if [ -f "$AIOS_CONFIG" ]; then
                    echo ""
                    cat "$AIOS_CONFIG"
                  fi
                  ;;
                *)
                  echo "Usage: aios [install|start|stop|status|config]"
                  echo "  install - Clone AIOS repository and setup"
                  echo "  start   - Start AIOS Kernel server (port $AIOS_PORT)"
                  echo "  stop    - Stop AIOS Kernel server"
                  echo "  status  - Check if AIOS is running"
                  echo "  config  - Show AIOS configuration"
                  echo ""
                  echo "Environment variables:"
                  echo "  AIOS_DIR    - Installation directory (default: ~/.local/share/aios)"
                  echo "  AIOS_PORT   - Server port (default: 8000)"
                  echo ""
                  echo "Requires pixi AIOS environment: pixi run -e aios ..."
                  ;;
              esac
            '')
            (pkgs.writeShellScriptBin "vault-dev" ''
              # Start HashiCorp Vault in development mode
              # - Auto-unsealed, in-memory storage
              # - Root token: root (for dev only!)
              # - TLS enabled with auto-generated certs
              echo "Starting Vault in dev mode..."
              echo "  Address: https://127.0.0.1:8200"
              echo "  Token: root"
              echo ""
              exec vault server -dev -dev-root-token-id root -dev-tls "$@"
            '')
            (pkgs.writeShellScriptBin "neonctl" ''
              # Wrapper for Neon serverless Postgres CLI
              # Uses npx to run the latest version without global installation
              # See: https://neon.tech/docs/reference/cli-reference
              exec npx neonctl@latest "$@"
            '')
            (pkgs.writeShellScriptBin "agentgateway-install" ''
              # Install AgentGateway from source
              # Requires Rust toolchain (cargo)
              echo "Installing AgentGateway..."
              echo "  Repository: github.com/agentgateway/agentgateway"
              echo ""
              if ! command -v cargo >/dev/null 2>&1; then
                echo "Error: Rust/Cargo not found. Install Rust first:" >&2
                echo "  curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh" >&2
                exit 1
              fi
              cargo install --git https://github.com/agentgateway/agentgateway agentgateway "$@"
              echo ""
              echo "AgentGateway installed! Run with: agentgateway --help"
            '')
          ];

          # CUDA 13.x package set (latest available in nixpkgs)
          # Falls back to default cudaPackages if 13.1 unavailable
          cudaPkgs = pkgs.cudaPackages_13_1 or pkgs.cudaPackages_13 or pkgs.cudaPackages;

        in
        {
          # Development shell (main entry point)
          # Use standard `devShells` (and avoid the devshell flake module) so `nix flake check`
          # stays warning-free on newer Nix.
          devShells.default = pkgs.mkShell {
            # Include Holochain in default shell - P2P coordination is mandatory
            packages = basePackages ++ holochainPackages ++ coreCommandWrappers ++ linuxPackages ++ darwinPackages;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            shellHook = ''
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              # These are called by conda post-link scripts but don't exist in plain bash
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              # Initialize pixi environment
              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
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
            packages =
              basePackages
              ++ fullExtras
              ++ holochainPackages  # P0: Holochain coordination layer
              ++ coreCommandWrappers
              ++ aiCommandWrappers
              ++ linuxPackages
              ++ darwinPackages;
            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            shellHook = ''
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
              fi
              # Initialize direnv
              eval "$(direnv hook bash)"

              # Initialize zoxide
              eval "$(zoxide init bash)"

              # Initialize starship prompt
              eval "$(starship init bash)"

              # ROS2 environment info
              echo ""
              echo "ROS2 Humble Development Environment (Full)"
              echo "==========================================="
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
              echo "AI infrastructure:"
              echo "  localai   - LocalAI server management (start|stop|status|models)"
              echo "  agixt     - AGiXT platform via Docker (up|down|logs|status)"
              echo "  aios      - AIOS Agent Kernel management (install|start|stop|status)"
              echo ""
              echo "Holochain (P2P coordination):"
              echo "  holochain - Holochain conductor"
              echo "  hc        - Holochain dev CLI"
              echo ""
              echo "Rust (AGiXT SDK):"
              echo "  cargo build -p agixt-bridge  # Build AGiXT-ROS2 bridge"
              echo ""
            '';
          };

          # CUDA-enabled shell for GPU workloads
          # Usage: nix develop .#cuda
          # Requires: NVIDIA GPU with drivers installed
          # Binary cache: https://cache.nixos-cuda.org
          devShells.cuda = pkgs.mkShell {
            packages = basePackages ++ fullExtras ++ holochainPackages ++ coreCommandWrappers ++ aiCommandWrappers ++ linuxPackages ++ (with pkgs; [
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
              # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp
              mkdir -p "$TMPDIR" 2>/dev/null || true

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              # Set up LD_LIBRARY_PATH for CUDA libraries
              export LD_LIBRARY_PATH="${cudaPkgs.cudatoolkit}/lib:${cudaPkgs.cudnn}/lib:$LD_LIBRARY_PATH"

              # Initialize pixi environment with CUDA feature
              if [ -f pixi.toml ]; then
                eval "$(pixi shell-hook -e cuda 2>/dev/null || pixi shell-hook 2>/dev/null)" || true
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
                echo "ROS2 Humble + CUDA Development Environment"
                echo "==========================================="
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

          # NOTE: Legacy devshells.default was removed - it required the devshell
          # flake-parts module which isn't imported. Use devShells.default instead.
          # The command aliases (cb, ct, ctr, etc.) are available as executable commands in devShells.default.

          # Identity & Auth shell for Keycloak/Vaultwarden development (Linux only)
          # Usage: nix develop .#identity
          # Heavy dependencies: Java 21, PostgreSQL
          devShells.identity = pkgs.mkShell {
            packages = basePackages ++ coreCommandWrappers ++ linuxPackages ++ (with pkgs; [
              # Identity & Access Management
              keycloak             # OAuth2/OIDC identity provider (Java 21)
              vaultwarden          # Bitwarden-compatible password manager (Rust)

              # Database backends
              postgresql_15        # PostgreSQL for Keycloak/Vaultwarden
              sqlite               # SQLite for lightweight Vaultwarden

              # Java runtime (required by Keycloak)
              jdk21_headless       # Java 21 LTS (headless for servers)

              # Database tools
              pgcli                # PostgreSQL CLI with autocomplete
            ]);

            COLCON_DEFAULTS_FILE = toString colconDefaults;
            EDITOR = "hx";
            VISUAL = "hx";

            # Java environment
            JAVA_HOME = "${pkgs.jdk21_headless}";

            shellHook = ''
              # Ensure TMPDIR is valid
              export TMPDIR=''${TMPDIR:-/tmp}
              [ -d "$TMPDIR" ] || export TMPDIR=/tmp

              # Define stub functions for RoboStack activation scripts
              noa_add_path() { :; }
              export -f noa_add_path 2>/dev/null || true

              if [ -f pixi.toml ]; then
                ${optionalString isDarwin ''
                  export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                ''}
                eval "$(pixi shell-hook 2>/dev/null)" || true
              fi

              echo ""
              echo "🔐 Identity & Auth Development Environment"
              echo "=========================================="
              echo "  Platform: Linux (${system})"
              echo "  Java: $(java -version 2>&1 | head -1)"
              echo "  PostgreSQL: ${pkgs.postgresql_15.version}"
              echo ""
              echo "Available services:"
              echo "  keycloak        - OAuth2/OIDC identity provider"
              echo "  vaultwarden     - Bitwarden-compatible password manager"
              echo ""
              echo "Quick start:"
              echo "  # Start PostgreSQL (for Keycloak)"
              echo "  initdb -D ./pgdata && pg_ctl -D ./pgdata start"
              echo ""
              echo "  # Start Keycloak in dev mode"
              echo "  keycloak start-dev --http-port=8080"
              echo ""
              echo "  # Start Vaultwarden (SQLite)"
              echo "  vaultwarden"
              echo ""
            '';
          };
        };
    };
}
