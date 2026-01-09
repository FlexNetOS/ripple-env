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

          # ROS2-specific command wrappers
          ros2CommandWrappers = [
            # Clean ROS2 build artifacts
            (pkgs.writeShellScriptBin "ros2-clean" ''
              # Clean ROS2/colcon build artifacts
              # Usage: ros2-clean [--all]
              set -e

              if [ "$1" = "--all" ] || [ "$1" = "-a" ]; then
                echo "[ros2-clean] Removing all build artifacts..."
                rm -rf build/ install/ log/ .colcon_build_status
                echo "[ros2-clean] Cleaned: build/, install/, log/"
              else
                echo "[ros2-clean] Removing build and install directories..."
                rm -rf build/ install/
                echo "[ros2-clean] Cleaned: build/, install/"
                echo "[ros2-clean] Use --all to also remove log/"
              fi
            '')
            # Show ROS2 workspace information
            (pkgs.writeShellScriptBin "ros2-ws" ''
              # Display ROS2 workspace information
              echo "ROS2 Workspace Information"
              echo "=========================="
              echo ""

              # ROS2 environment
              echo "Environment:"
              if [ -n "$ROS_DISTRO" ]; then
                echo "  ROS_DISTRO: $ROS_DISTRO"
              else
                echo "  ROS_DISTRO: (not set)"
              fi
              if [ -n "$RMW_IMPLEMENTATION" ]; then
                echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
              fi
              if [ -n "$ROS_DOMAIN_ID" ]; then
                echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
              fi
              echo ""

              # Packages
              echo "Packages:"
              if [ -d "src" ]; then
                pkg_count=$(find src -name "package.xml" 2>/dev/null | wc -l)
                echo "  Source packages: $pkg_count"
                if [ "$pkg_count" -gt 0 ] && [ "$pkg_count" -le 20 ]; then
                  find src -name "package.xml" -exec dirname {} \; | sed 's|.*/||' | sort | sed 's/^/    - /'
                fi
              else
                echo "  No src/ directory found"
              fi
              echo ""

              # Build status
              echo "Build Status:"
              if [ -d "build" ]; then
                built_count=$(ls -1 build/ 2>/dev/null | wc -l)
                echo "  Built packages: $built_count"
              else
                echo "  Not built yet (run 'cb' to build)"
              fi
              if [ -d "install" ]; then
                echo "  Install directory: exists"
              fi
              echo ""

              # Pixi environment
              echo "Pixi Environment:"
              if [ -f "pixi.toml" ]; then
                echo "  pixi.toml: found"
                if [ -d ".pixi/envs/default" ]; then
                  echo "  Environment: initialized"
                else
                  echo "  Environment: not initialized (run 'pixi install')"
                fi
              else
                echo "  pixi.toml: not found"
              fi
            '')
            # List ROS2 topics with filtering
            (pkgs.writeShellScriptBin "ros2-topics" ''
              # List ROS2 topics with optional filtering
              # Usage: ros2-topics [filter]
              if ! command -v ros2 >/dev/null 2>&1; then
                echo "Error: ros2 command not found" >&2
                echo "Make sure ROS2 environment is sourced" >&2
                exit 1
              fi

              if [ -n "$1" ]; then
                echo "ROS2 Topics matching '$1':"
                ros2 topic list | grep -i "$1" || echo "  (no matches)"
              else
                echo "ROS2 Topics:"
                ros2 topic list
              fi
            '')
            # Quick ROS2 node inspection
            (pkgs.writeShellScriptBin "ros2-nodes" ''
              # List ROS2 nodes with optional filtering
              # Usage: ros2-nodes [filter]
              if ! command -v ros2 >/dev/null 2>&1; then
                echo "Error: ros2 command not found" >&2
                echo "Make sure ROS2 environment is sourced" >&2
                exit 1
              fi

              if [ -n "$1" ]; then
                echo "ROS2 Nodes matching '$1':"
                ros2 node list | grep -i "$1" || echo "  (no matches)"
              else
                echo "ROS2 Nodes:"
                ros2 node list
              fi
            '')
          ];

          # Infrastructure command wrappers (IPFS, NATS, Prometheus)
          infraCommandWrappers = [
            # IPFS (kubo) management
            (pkgs.writeShellScriptBin "ipfs-ctl" ''
              # IPFS node management via kubo
              # Usage: ipfs-ctl [init|start|stop|status|id]
              IPFS_PATH="''${IPFS_PATH:-$HOME/.ipfs}"

              case "''${1:-status}" in
                init)
                  if [ -d "$IPFS_PATH" ]; then
                    echo "IPFS already initialized at $IPFS_PATH"
                  else
                    echo "Initializing IPFS node..."
                    ipfs init
                    echo ""
                    echo "IPFS initialized. Start with: ipfs-ctl start"
                  fi
                  ;;
                start)
                  if [ ! -d "$IPFS_PATH" ]; then
                    echo "IPFS not initialized. Run: ipfs-ctl init" >&2
                    exit 1
                  fi
                  echo "Starting IPFS daemon..."
                  echo "  API: http://127.0.0.1:5001"
                  echo "  Gateway: http://127.0.0.1:8080"
                  exec ipfs daemon "''${@:2}"
                  ;;
                stop)
                  if pkill -f "ipfs daemon"; then
                    echo "IPFS daemon stopped"
                  else
                    echo "IPFS daemon not running"
                  fi
                  ;;
                status)
                  if pgrep -f "ipfs daemon" > /dev/null; then
                    echo "IPFS daemon is running"
                    ipfs id 2>/dev/null | head -5
                  else
                    echo "IPFS daemon is not running"
                    [ -d "$IPFS_PATH" ] && echo "  Node initialized at: $IPFS_PATH"
                  fi
                  ;;
                id)
                  ipfs id
                  ;;
                *)
                  echo "Usage: ipfs-ctl [init|start|stop|status|id]"
                  echo "  init   - Initialize IPFS node"
                  echo "  start  - Start IPFS daemon"
                  echo "  stop   - Stop IPFS daemon"
                  echo "  status - Check daemon status"
                  echo "  id     - Show node identity"
                  ;;
              esac
            '')
            # NATS server management
            (pkgs.writeShellScriptBin "nats-ctl" ''
              # NATS server management
              # Usage: nats-ctl [start|stop|status|pub|sub]
              NATS_PORT="''${NATS_PORT:-4222}"
              NATS_HTTP_PORT="''${NATS_HTTP_PORT:-8222}"

              case "''${1:-status}" in
                start)
                  echo "Starting NATS server..."
                  echo "  Client: nats://127.0.0.1:$NATS_PORT"
                  echo "  HTTP:   http://127.0.0.1:$NATS_HTTP_PORT"
                  exec nats-server -p "$NATS_PORT" -m "$NATS_HTTP_PORT" "''${@:2}"
                  ;;
                stop)
                  if pkill -f "nats-server"; then
                    echo "NATS server stopped"
                  else
                    echo "NATS server not running"
                  fi
                  ;;
                status)
                  if pgrep -f "nats-server" > /dev/null; then
                    echo "NATS server is running"
                    curl -s "http://127.0.0.1:$NATS_HTTP_PORT/varz" 2>/dev/null | jq -r '.server_id // "Connected"' || echo "  (monitoring endpoint unavailable)"
                  else
                    echo "NATS server is not running"
                  fi
                  ;;
                pub)
                  # Publish a message: nats-ctl pub <subject> <message>
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: nats-ctl pub <subject> <message>" >&2
                    exit 1
                  fi
                  nats pub "$2" "$3"
                  ;;
                sub)
                  # Subscribe to a subject: nats-ctl sub <subject>
                  if [ -z "$2" ]; then
                    echo "Usage: nats-ctl sub <subject>" >&2
                    exit 1
                  fi
                  nats sub "$2"
                  ;;
                *)
                  echo "Usage: nats-ctl [start|stop|status|pub|sub]"
                  echo "  start        - Start NATS server"
                  echo "  stop         - Stop NATS server"
                  echo "  status       - Check server status"
                  echo "  pub <s> <m>  - Publish message to subject"
                  echo "  sub <s>      - Subscribe to subject"
                  echo ""
                  echo "Environment:"
                  echo "  NATS_PORT      - Client port (default: 4222)"
                  echo "  NATS_HTTP_PORT - Monitoring port (default: 8222)"
                  ;;
              esac
            '')
            # Prometheus management
            (pkgs.writeShellScriptBin "prom-ctl" ''
              # Prometheus server management
              # Usage: prom-ctl [start|stop|status|config]
              PROM_PORT="''${PROM_PORT:-9090}"
              PROM_CONFIG="''${PROM_CONFIG:-prometheus.yml}"
              PROM_DATA="''${PROM_DATA:-./prometheus-data}"

              case "''${1:-status}" in
                start)
                  if [ ! -f "$PROM_CONFIG" ]; then
                    echo "Creating default prometheus.yml..."
                    cat > "$PROM_CONFIG" << 'PROMCFG'
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']

  # ROS2 metrics (if ros2_prometheus_exporter is running)
  # - job_name: 'ros2'
  #   static_configs:
  #     - targets: ['localhost:9100']
PROMCFG
                    echo "Created $PROM_CONFIG"
                  fi
                  mkdir -p "$PROM_DATA"
                  echo "Starting Prometheus..."
                  echo "  Web UI: http://127.0.0.1:$PROM_PORT"
                  echo "  Config: $PROM_CONFIG"
                  exec prometheus --config.file="$PROM_CONFIG" --storage.tsdb.path="$PROM_DATA" --web.listen-address=":$PROM_PORT" "''${@:2}"
                  ;;
                stop)
                  if pkill -f "prometheus.*--config.file"; then
                    echo "Prometheus stopped"
                  else
                    echo "Prometheus not running"
                  fi
                  ;;
                status)
                  if pgrep -f "prometheus.*--config.file" > /dev/null; then
                    echo "Prometheus is running"
                    curl -s "http://127.0.0.1:$PROM_PORT/-/ready" && echo " - Ready"
                  else
                    echo "Prometheus is not running"
                  fi
                  ;;
                config)
                  if [ -f "$PROM_CONFIG" ]; then
                    cat "$PROM_CONFIG"
                  else
                    echo "Config file not found: $PROM_CONFIG"
                    echo "Run 'prom-ctl start' to create a default config"
                  fi
                  ;;
                *)
                  echo "Usage: prom-ctl [start|stop|status|config]"
                  echo "  start  - Start Prometheus server"
                  echo "  stop   - Stop Prometheus server"
                  echo "  status - Check server status"
                  echo "  config - Show configuration"
                  echo ""
                  echo "Environment:"
                  echo "  PROM_PORT   - Web UI port (default: 9090)"
                  echo "  PROM_CONFIG - Config file (default: prometheus.yml)"
                  echo "  PROM_DATA   - Data directory (default: ./prometheus-data)"
                  ;;
              esac
            '')
          ];

          # Security command wrappers (SBOM, vulnerability scanning, signing)
          securityCommandWrappers = [
            # Generate SBOM (Software Bill of Materials)
            (pkgs.writeShellScriptBin "sbom" ''
              # Generate SBOM using syft
              # Usage: sbom [target] [--format <format>]
              # Formats: json, spdx-json, cyclonedx-json, table (default)
              set -e

              TARGET="''${1:-.}"
              FORMAT="''${2:-table}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: sbom [target] [format]"
                echo ""
                echo "Generate Software Bill of Materials using syft"
                echo ""
                echo "Arguments:"
                echo "  target  - Directory, container image, or archive (default: .)"
                echo "  format  - Output format: table, json, spdx-json, cyclonedx-json"
                echo ""
                echo "Examples:"
                echo "  sbom                      # Scan current directory"
                echo "  sbom ./src json           # Scan src/ as JSON"
                echo "  sbom alpine:latest        # Scan container image"
                echo "  sbom . cyclonedx-json     # CycloneDX format for compliance"
                exit 0
              fi

              echo "Generating SBOM for: $TARGET" >&2
              echo "Format: $FORMAT" >&2
              echo "" >&2

              syft "$TARGET" -o "$FORMAT"
            '')
            # Vulnerability scanning
            (pkgs.writeShellScriptBin "vuln-scan" ''
              # Scan for vulnerabilities using grype or trivy
              # Usage: vuln-scan [target] [--tool <grype|trivy>]
              set -e

              TARGET="''${1:-.}"
              TOOL="''${2:-grype}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: vuln-scan [target] [tool]"
                echo ""
                echo "Scan for vulnerabilities in code, containers, or SBOMs"
                echo ""
                echo "Arguments:"
                echo "  target  - Directory, container image, or SBOM file (default: .)"
                echo "  tool    - Scanner to use: grype (default) or trivy"
                echo ""
                echo "Examples:"
                echo "  vuln-scan                     # Scan current directory with grype"
                echo "  vuln-scan . trivy             # Scan with trivy"
                echo "  vuln-scan alpine:latest       # Scan container image"
                echo "  vuln-scan sbom.json grype     # Scan from SBOM"
                exit 0
              fi

              echo "Scanning: $TARGET" >&2
              echo "Tool: $TOOL" >&2
              echo "" >&2

              case "$TOOL" in
                grype)
                  grype "$TARGET"
                  ;;
                trivy)
                  trivy fs "$TARGET"
                  ;;
                *)
                  echo "Unknown tool: $TOOL (use grype or trivy)" >&2
                  exit 1
                  ;;
              esac
            '')
            # Sign artifacts with cosign
            (pkgs.writeShellScriptBin "sign-artifact" ''
              # Sign container images or blobs with cosign
              # Usage: sign-artifact <image|file> [--key <key>]
              set -e

              if [ -z "$1" ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: sign-artifact <target> [--key <keyfile>]"
                echo ""
                echo "Sign container images or files using cosign (sigstore)"
                echo ""
                echo "Arguments:"
                echo "  target  - Container image or file to sign"
                echo "  --key   - Private key file (optional, uses keyless by default)"
                echo ""
                echo "Examples:"
                echo "  sign-artifact myregistry/myimage:v1.0     # Keyless signing"
                echo "  sign-artifact myimage --key cosign.key    # Key-based signing"
                echo "  sign-artifact artifact.tar.gz             # Sign a file"
                echo ""
                echo "Verify with:"
                echo "  cosign verify <image>"
                echo "  cosign verify-blob --signature <sig> <file>"
                exit 0
              fi

              TARGET="$1"
              shift

              if [ "$1" = "--key" ] && [ -n "$2" ]; then
                echo "Signing with key: $2" >&2
                cosign sign --key "$2" "$TARGET"
              else
                echo "Using keyless signing (OIDC)" >&2
                echo "You will be prompted to authenticate via browser" >&2
                cosign sign "$TARGET"
              fi
            '')
            # PKI certificate generation with step-cli
            (pkgs.writeShellScriptBin "pki-cert" ''
              # Generate certificates using step-cli
              # Usage: pki-cert <command> [args]
              case "''${1:-help}" in
                ca-init)
                  # Initialize a local CA
                  echo "Initializing local Certificate Authority..."
                  step ca init --name "ROS2-Dev-CA" --provisioner admin --dns localhost --address ":9000"
                  ;;
                create)
                  # Create a certificate: pki-cert create <name> <san>
                  if [ -z "$2" ]; then
                    echo "Usage: pki-cert create <name> [san...]" >&2
                    exit 1
                  fi
                  NAME="$2"
                  shift 2
                  echo "Creating certificate for: $NAME"
                  step certificate create "$NAME" "$NAME.crt" "$NAME.key" --san "$NAME" "$@"
                  echo ""
                  echo "Created: $NAME.crt, $NAME.key"
                  ;;
                inspect)
                  # Inspect a certificate
                  if [ -z "$2" ]; then
                    echo "Usage: pki-cert inspect <cert-file>" >&2
                    exit 1
                  fi
                  step certificate inspect "$2"
                  ;;
                *)
                  echo "Usage: pki-cert <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  ca-init           - Initialize a local Certificate Authority"
                  echo "  create <n> [san]  - Create certificate for name with SANs"
                  echo "  inspect <cert>    - Inspect a certificate file"
                  echo ""
                  echo "Examples:"
                  echo "  pki-cert ca-init"
                  echo "  pki-cert create robot1 --san robot1.local --san 192.168.1.10"
                  echo "  pki-cert inspect robot1.crt"
                  ;;
              esac
            '')
          ];

          # Development workflow command wrappers
          devCommandWrappers = [
            # Format all Nix files
            (pkgs.writeShellScriptBin "fmt-nix" ''
              # Format all Nix files in the repository
              echo "Formatting Nix files..."
              find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} +
              echo "Done."
            '')
            # Run all checks
            (pkgs.writeShellScriptBin "dev-check" ''
              # Run all development checks
              # Usage: dev-check [--fix]
              set -e
              FIX=""
              [ "$1" = "--fix" ] && FIX="1"

              echo "Running development checks..."
              echo "=============================="
              FAILED=0

              # Nix formatting
              echo ""
              echo "[1/4] Checking Nix formatting..."
              if [ -n "$FIX" ]; then
                find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} + && echo "  Formatted."
              else
                if find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt --check {} + 2>/dev/null; then
                  echo "  OK"
                else
                  echo "  FAIL - Run with --fix to auto-format"
                  FAILED=1
                fi
              fi

              # Nix flake check
              echo ""
              echo "[2/4] Checking Nix flake..."
              if command -v nix >/dev/null 2>&1; then
                if nix flake check --no-build 2>&1 | head -20; then
                  echo "  OK"
                else
                  echo "  FAIL"
                  FAILED=1
                fi
              else
                echo "  SKIP (nix not available)"
              fi

              # Pixi lock check
              echo ""
              echo "[3/4] Checking pixi.lock..."
              if [ -f pixi.toml ]; then
                if [ -f pixi.lock ]; then
                  echo "  OK (lock file exists)"
                else
                  echo "  FAIL - Run 'pixi install' to create lock file"
                  FAILED=1
                fi
              else
                echo "  SKIP (no pixi.toml)"
              fi

              # ROS2 build check
              echo ""
              echo "[4/4] Checking ROS2 packages..."
              if [ -d "src" ] && command -v colcon >/dev/null 2>&1; then
                if colcon build --packages-select-build-failed 2>&1 | grep -q "No packages"; then
                  echo "  OK (all packages build)"
                else
                  echo "  WARN - Some packages may have build issues"
                fi
              else
                echo "  SKIP (no src/ or colcon unavailable)"
              fi

              echo ""
              echo "=============================="
              if [ "$FAILED" -eq 0 ]; then
                echo "All checks passed!"
              else
                echo "Some checks failed. Run with --fix to auto-fix where possible."
                exit 1
              fi
            '')
            # Git pre-commit helper
            (pkgs.writeShellScriptBin "pre-commit" ''
              # Run pre-commit checks
              # Install as git hook: ln -sf $(which pre-commit) .git/hooks/pre-commit
              echo "Running pre-commit checks..."

              # Check for large files
              LARGE_FILES=$(git diff --cached --name-only | xargs -I{} sh -c 'test -f "{}" && du -k "{}" | awk "\$1 > 1024 {print \$2}"' 2>/dev/null)
              if [ -n "$LARGE_FILES" ]; then
                echo "Warning: Large files (>1MB) staged for commit:" >&2
                echo "$LARGE_FILES" | sed 's/^/  /' >&2
                echo "Consider using Git LFS for large files." >&2
              fi

              # Check for secrets patterns
              if git diff --cached | grep -E "(password|secret|api_key|private_key)\s*[:=]" >/dev/null 2>&1; then
                echo "Warning: Possible secrets detected in staged changes" >&2
                echo "Please review before committing." >&2
              fi

              # Format staged Nix files
              STAGED_NIX=$(git diff --cached --name-only --diff-filter=ACM | grep '\.nix$' || true)
              if [ -n "$STAGED_NIX" ]; then
                echo "Formatting staged Nix files..."
                echo "$STAGED_NIX" | xargs nixfmt
                echo "$STAGED_NIX" | xargs git add
              fi

              echo "Pre-commit checks complete."
            '')
          ];

          # CUDA 13.x package set (latest available in nixpkgs)
          # Falls back to default cudaPackages if 13.1 unavailable
          cudaPkgs = pkgs.cudaPackages_13_1 or pkgs.cudaPackages_13 or pkgs.cudaPackages;

        in
        {
          devshells.default = {
            env = [
              {
                name = "COLCON_DEFAULTS_FILE";
                value = toString colconDefaults;
              }
            ];
            devshell = {
              packages = with pkgs; [
                pixi
              ];
              startup.activate.text = ''
                # ROS2 Humble Environment Activation Script
                # This script initializes the pixi-managed ROS2 development environment

                _ros2_activate() {
                  # Check if pixi.toml exists in the current directory
                  if [ ! -f pixi.toml ]; then
                    echo "[ros2-env] INFO: pixi.toml not found in $(pwd), skipping pixi activation" >&2
                    return 0
                  fi

                  # Verify pixi command is available
                  if ! command -v pixi >/dev/null 2>&1; then
                    echo "[ros2-env] ERROR: pixi command not found in PATH" >&2
                    echo "[ros2-env] Please ensure pixi is installed and available" >&2
                    return 1
                  fi

                  # Check if pixi environment has been initialized
                  if [ ! -d ".pixi/envs/default" ]; then
                    echo "[ros2-env] WARNING: Pixi environment not initialized" >&2
                    echo "[ros2-env] Run 'pixi install' to set up the environment" >&2
                    return 1
                  fi

                  ${optionalString isDarwin ''
                    # macOS: Configure dynamic library fallback path for ROS2
                    local pixi_lib_path="$PWD/.pixi/envs/default/lib"
                    if [ -d "$pixi_lib_path" ]; then
                      export DYLD_FALLBACK_LIBRARY_PATH="$pixi_lib_path:''${DYLD_FALLBACK_LIBRARY_PATH:-}"
                    else
                      echo "[ros2-env] WARNING: Pixi library path not found: $pixi_lib_path" >&2
                      echo "[ros2-env] Some ROS2 libraries may not load correctly" >&2
                    fi
                  ''}

                  # Generate and execute pixi shell-hook
                  # Note: Only capture stdout for eval; let stderr pass through for warnings
                  # Use a temp file to capture stderr for error reporting if command fails
                  local pixi_hook pixi_stderr_file
                  pixi_stderr_file=$(mktemp)
                  if ! pixi_hook=$(pixi shell-hook 2>"$pixi_stderr_file"); then
                    echo "[ros2-env] ERROR: Failed to generate pixi shell-hook" >&2
                    if [ -s "$pixi_stderr_file" ]; then
                      echo "[ros2-env] pixi error: $(cat "$pixi_stderr_file")" >&2
                    fi
                    rm -f "$pixi_stderr_file"
                    return 1
                  fi
                  # Show any pixi warnings but don't include them in eval
                  if [ -s "$pixi_stderr_file" ]; then
                    cat "$pixi_stderr_file" >&2
                  fi
                  rm -f "$pixi_stderr_file"

                  # Validate shell-hook output is not empty
                  if [ -z "$pixi_hook" ]; then
                    echo "[ros2-env] ERROR: pixi shell-hook returned empty output" >&2
                    echo "[ros2-env] This may indicate a pixi configuration issue" >&2
                    return 1
                  fi

                  # Execute the shell-hook to activate the environment
                  if ! eval "$pixi_hook"; then
                    echo "[ros2-env] ERROR: Failed to evaluate pixi shell-hook" >&2
                    return 1
                  fi

                  echo "[ros2-env] ROS2 Humble environment activated successfully" >&2
                  return 0
                }

                # Run activation and capture result
                _ros2_activate
                _ros2_activate_result=$?

                # Clean up the function to avoid polluting the shell namespace
                unset -f _ros2_activate

                # Return the activation result (non-fatal to allow shell to continue)
                if [ $_ros2_activate_result -ne 0 ]; then
                  echo "[ros2-env] Environment activation failed (exit code: $_ros2_activate_result)" >&2
                  echo "[ros2-env] The shell will continue, but ROS2 tools may not work correctly" >&2
                fi
                unset _ros2_activate_result
              '';
              motd = "";
            };
          # Development shell (main entry point)
          # Use standard `devShells` (and avoid the devshell flake module) so `nix flake check`
          # stays warning-free on newer Nix.
          devShells.default = pkgs.mkShell {
            # Include Holochain in default shell - P2P coordination is mandatory
            packages = basePackages ++ holochainPackages ++ coreCommandWrappers ++ ros2CommandWrappers ++ devCommandWrappers ++ linuxPackages ++ darwinPackages;
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
              ++ ros2CommandWrappers
              ++ infraCommandWrappers
              ++ securityCommandWrappers
              ++ devCommandWrappers
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
              echo "  cb          - colcon build --symlink-install"
              echo "  ct          - colcon test"
              echo "  ros2-clean  - Clean build artifacts (--all for logs too)"
              echo "  ros2-ws     - Show workspace info"
              echo "  ros2-topics - List ROS2 topics"
              echo "  ros2-nodes  - List ROS2 nodes"
              echo ""
              echo "Development:"
              echo "  dev-check   - Run all checks (--fix to auto-fix)"
              echo "  fmt-nix     - Format all Nix files"
              echo "  pre-commit  - Git pre-commit checks"
              echo ""
              echo "AI assistants:"
              echo "  ai          - AI chat (aichat, lightweight)"
              echo "  pair        - AI pair programming (aider, git-integrated)"
              echo "  promptfoo   - LLM testing & evaluation"
              echo ""
              echo "AI infrastructure:"
              echo "  localai     - LocalAI server (start|stop|status|models)"
              echo "  agixt       - AGiXT platform (up|down|logs|status)"
              echo "  aios        - AIOS Kernel (install|start|stop|status)"
              echo ""
              echo "Infrastructure:"
              echo "  ipfs-ctl    - IPFS node (init|start|stop|status)"
              echo "  nats-ctl    - NATS server (start|stop|pub|sub)"
              echo "  prom-ctl    - Prometheus (start|stop|config)"
              echo "  vault-dev   - HashiCorp Vault dev mode"
              echo ""
              echo "Security:"
              echo "  sbom        - Generate SBOM (syft)"
              echo "  vuln-scan   - Vulnerability scan (grype/trivy)"
              echo "  sign-artifact - Sign with cosign"
              echo "  pki-cert    - PKI certificates (step-cli)"
              echo ""
              echo "Holochain (P2P):"
              echo "  holochain   - Holochain conductor"
              echo "  hc          - Holochain dev CLI"
              echo ""
            '';
          };

          # CUDA-enabled shell for GPU workloads
          # Usage: nix develop .#cuda
          # Requires: NVIDIA GPU with drivers installed
          # Binary cache: https://cache.nixos-cuda.org
          devShells.cuda = pkgs.mkShell {
            packages = basePackages ++ fullExtras ++ holochainPackages ++ coreCommandWrappers ++ ros2CommandWrappers ++ infraCommandWrappers ++ securityCommandWrappers ++ devCommandWrappers ++ aiCommandWrappers ++ linuxPackages ++ (with pkgs; [
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
