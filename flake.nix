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
    # NOTE: This overlay is loaded via fetchFromGitHub (not as flake input) because:
    #   1. The upstream repo is not a proper flake
    #   2. fetchFromGitHub allows pinning to a specific commit
    #   3. Avoids flake.lock sync issues
    # See: https://github.com/spartan-holochain-counsel/nix-overlay
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
            # P0: sandbox-runtime - Process sandbox for MCP/tool execution
            # BUILDKIT_STARTER_SPEC.md Layer 3 (Isolation) and Layer 8 (Tool Execution)
            (pkgs.writeShellScriptBin "sandbox-runtime" ''
              # Sandbox-runtime wrapper
              # Checks for installation, provides install instructions if missing
              SANDBOX_BIN="''${SANDBOX_RUNTIME_PATH:-$HOME/.cargo/bin/sandbox-runtime}"

              if [ -x "$SANDBOX_BIN" ]; then
                exec "$SANDBOX_BIN" "$@"
              elif command -v sandbox-runtime >/dev/null 2>&1; then
                exec sandbox-runtime "$@"
              else
                echo "sandbox-runtime not found. Install with:" >&2
                echo "" >&2
                echo "  # Option 1: Build from source (recommended)" >&2
                echo "  git clone https://github.com/anthropic-experimental/sandbox-runtime.git" >&2
                echo "  cd sandbox-runtime && cargo build --release" >&2
                echo "  cargo install --path ." >&2
                echo "" >&2
                echo "  # Option 2: Install via cargo" >&2
                echo "  cargo install --git https://github.com/anthropic-experimental/sandbox-runtime" >&2
                echo "" >&2
                echo "See: docs/SANDBOX_RUNTIME_INSTALL.md" >&2
                exit 127
              fi
            '')
            # P0: Kata Containers management wrapper
            # BUILDKIT_STARTER_SPEC.md Layer 3 (Isolation)
            (pkgs.writeShellScriptBin "kata" ''
              # Kata Containers wrapper
              # Provides installation guidance and runtime management

              case "''${1:-status}" in
                install)
                  echo "Installing Kata Containers..."
                  echo ""
                  echo "Kata Containers requires system-level installation."
                  echo ""
                  echo "# Ubuntu/Debian:"
                  echo "  sudo apt-get install -y kata-containers"
                  echo ""
                  echo "# Fedora:"
                  echo "  sudo dnf install -y kata-containers"
                  echo ""
                  echo "# From official releases (check https://github.com/kata-containers/kata-containers/releases):"
                  echo "  # Get latest version"
                  echo "  KATA_VERSION=\$(curl -s https://api.github.com/repos/kata-containers/kata-containers/releases/latest | grep tag_name | cut -d '\"' -f 4)"
                  echo "  # Download and install"
                  echo "  curl -fsSL https://github.com/kata-containers/kata-containers/releases/download/\$KATA_VERSION/kata-static-\$KATA_VERSION-amd64.tar.xz | sudo tar -C / -xJf -"
                  echo ""
                  echo "# Configure Docker to use Kata:"
                  echo "  sudo mkdir -p /etc/docker"
                  echo "  cat <<EOF | sudo tee /etc/docker/daemon.json"
                  echo '  {'
                  echo '    "runtimes": {'
                  echo '      "kata": {'
                  echo '        "path": "/usr/bin/kata-runtime"'
                  echo '      }'
                  echo '    }'
                  echo '  }'
                  echo "  EOF"
                  echo "  sudo systemctl restart docker"
                  echo ""
                  echo "See: docs/KATA_CONTAINERS_INSTALL.md"
                  ;;
                status)
                  if command -v kata-runtime >/dev/null 2>&1; then
                    echo "Kata Containers: INSTALLED"
                    kata-runtime --version
                    echo ""
                    echo "Docker runtime check:"
                    docker info 2>/dev/null | grep -i kata || echo "  Kata not configured as Docker runtime"
                  else
                    echo "Kata Containers: NOT INSTALLED"
                    echo "Run 'kata install' for installation instructions."
                  fi
                  ;;
                check)
                  if command -v kata-runtime >/dev/null 2>&1; then
                    echo "Running Kata check-config..."
                    kata-runtime check
                  else
                    echo "kata-runtime not found"
                    exit 1
                  fi
                  ;;
                test)
                  echo "Testing Kata runtime with Docker..."
                  docker run --rm --runtime=kata alpine cat /etc/os-release
                  ;;
                *)
                  echo "Usage: kata [install|status|check|test]"
                  echo "  install - Show installation instructions"
                  echo "  status  - Check if Kata is installed"
                  echo "  check   - Run kata-runtime check-config"
                  echo "  test    - Test Kata with Docker"
                  ;;
              esac
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

          # Workflow management command wrappers
          workflowCommandWrappers = [
            # Shared helper function for opening URLs
            (pkgs.writeShellScriptBin "open-url-helper" ''
              # Helper to open URLs in the default browser
              URL="$1"
              if [ -z "$URL" ]; then
                echo "Error: No URL provided" >&2
                exit 1
              fi
              
              echo "Opening: $URL"
              if command -v xdg-open >/dev/null 2>&1; then
                xdg-open "$URL"
              elif command -v open >/dev/null 2>&1; then
                open "$URL"
              else
                echo "Open in browser: $URL"
              fi
            '')
            # GitHub issues query tool
            (pkgs.writeShellScriptBin "gh-issues" ''
              # Query and manage GitHub issues
              # Usage: gh-issues [list|search|create|view|close]

              if ! command -v gh >/dev/null 2>&1; then
                echo "Error: gh CLI not found" >&2
                echo "Install with: nix develop" >&2
                exit 1
              fi

              case "''${1:-list}" in
                list)
                  # List issues with optional state filter
                  STATE="''${2:-open}"
                  LIMIT="''${3:-20}"
                  
                  # Validate STATE parameter
                  if [[ ! "$STATE" =~ ^(open|closed|all)$ ]]; then
                    echo "Error: STATE must be one of: open, closed, all" >&2
                    exit 1
                  fi
                  
                  # Validate LIMIT parameter
                  if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
                    echo "Error: LIMIT must be a positive integer" >&2
                    exit 1
                  fi
                  
                  echo "Issues ($STATE):"
                  gh issue list --state "$STATE" --limit "$LIMIT"
                  ;;
                search)
                  # Search issues by keyword
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues search <query> [state]" >&2
                    exit 1
                  fi
                  STATE="''${3:-all}"
                  echo "Searching issues for: $2"
                  gh issue list --search "$2" --state "$STATE"
                  ;;
                labels)
                  # List issues by label
                  if [ -z "$2" ]; then
                    echo "Available labels:"
                    gh label list
                  else
                    echo "Issues with label '$2':"
                    gh issue list --label "$2"
                  fi
                  ;;
                assigned)
                  # List issues assigned to user
                  USER="''${2:-@me}"
                  echo "Issues assigned to $USER:"
                  gh issue list --assignee "$USER"
                  ;;
                create)
                  # Create a new issue interactively
                  shift
                  gh issue create "$@"
                  ;;
                view)
                  # View issue details
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues view <issue-number>" >&2
                    exit 1
                  fi
                  gh issue view "$2"
                  ;;
                close)
                  # Close an issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues close <issue-number> [reason]" >&2
                    exit 1
                  fi
                  REASON="''${3:-completed}"
                  gh issue close "$2" --reason "$REASON"
                  ;;
                reopen)
                  # Reopen an issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues reopen <issue-number>" >&2
                    exit 1
                  fi
                  gh issue reopen "$2"
                  ;;
                comment)
                  # Add comment to issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues comment <issue-number> [message]" >&2
                    exit 1
                  fi
                  shift
                  ISSUE="$1"
                  shift
                  if [ -n "$1" ]; then
                    gh issue comment "$ISSUE" --body "$*"
                  else
                    gh issue comment "$ISSUE"
                  fi
                  ;;
                stats)
                  # Show issue statistics
                  echo "Issue Statistics"
                  echo "================"
                  echo ""
                  echo "Open issues:   $(gh issue list --state open --json number --jq 'length')"
                  # Note: Using limit of 1000 for closed issues - counts may be inaccurate if more exist
                  echo "Closed issues: $(gh issue list --state closed --limit 1000 --json number --jq 'length') (max 1000)"
                  echo ""
                  echo "By label: (this may take a while for repositories with many labels)"
                  # Note: Queries GitHub for each label sequentially - performance scales with label count
                  gh label list --json name --jq '.[].name' | while read -r label; do
                    count=$(gh issue list --label "$label" --json number --jq 'length' 2>/dev/null || echo "0")
                    [ "$count" != "0" ] && echo "  $label: $count"
                  done
                  ;;
                *)
                  echo "Usage: gh-issues <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  list [state] [limit]     - List issues (default: open, 20)"
                  echo "  search <query> [state]   - Search issues by keyword"
                  echo "  labels [label]           - List labels or issues with label"
                  echo "  assigned [user]          - Issues assigned to user (@me default)"
                  echo "  create                   - Create new issue interactively"
                  echo "  view <num>               - View issue details"
                  echo "  close <num> [reason]     - Close issue (completed|not_planned)"
                  echo "  reopen <num>             - Reopen closed issue"
                  echo "  comment <num> [msg]      - Add comment to issue"
                  echo "  stats                    - Show issue statistics"
                  ;;
              esac
            '')
            # Database query tool
            (pkgs.writeShellScriptBin "db-query" ''
              # Query databases (PostgreSQL/SQLite)
              # Usage: db-query [pg|sqlite] <database> [query]

              DB_TYPE="''${1:-help}"

              case "$DB_TYPE" in
                pg|postgres)
                  # PostgreSQL query
                  DB_NAME="''${2:-postgres}"
                  DB_HOST="''${DB_HOST:-localhost}"
                  DB_PORT="''${DB_PORT:-5432}"
                  DB_USER="''${DB_USER:-postgres}"

                  if [ -z "$3" ]; then
                    # Interactive mode
                    echo "Connecting to PostgreSQL: $DB_NAME@$DB_HOST:$DB_PORT"
                    if command -v pgcli >/dev/null 2>&1; then
                      pgcli -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME"
                    else
                      psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME"
                    fi
                  else
                    # Execute query - WARNING: For complex queries with special characters,
                    # use interactive mode to avoid shell interpretation issues
                    shift 2
                    # Use printf to properly quote the query
                    QUERY="$*"
                    psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" -c "$QUERY"
                  fi
                  ;;
                sqlite)
                  # SQLite query
                  DB_FILE="''${2:-database.db}"

                  if [ ! -f "$DB_FILE" ]; then
                    echo "Error: SQLite database not found: $DB_FILE" >&2
                    exit 1
                  fi

                  if [ -z "$3" ]; then
                    # Interactive mode
                    echo "Opening SQLite database: $DB_FILE"
                    sqlite3 "$DB_FILE"
                  else
                    # Execute query - WARNING: For complex queries with special characters,
                    # use interactive mode to avoid shell interpretation issues
                    shift 2
                    # Use variable to properly quote the query
                    QUERY="$*"
                    sqlite3 "$DB_FILE" "$QUERY"
                  fi
                  ;;
                temporal)
                  # Query Temporal database
                  DB_NAME="temporal"
                  DB_HOST="''${TEMPORAL_DB_HOST:-localhost}"
                  DB_PORT="''${TEMPORAL_DB_PORT:-5432}"
                  DB_USER="''${TEMPORAL_DB_USER:-temporal}"

                  echo "Connecting to Temporal database..."
                  PGPASSWORD="''${TEMPORAL_DB_PASSWORD:-temporal}" psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" "''${@:2}"
                  ;;
                n8n)
                  # Query n8n database
                  DB_NAME="n8n_db"
                  DB_HOST="''${N8N_DB_HOST:-localhost}"
                  DB_PORT="''${N8N_DB_PORT:-5432}"
                  DB_USER="''${N8N_DB_USER:-n8n}"
                  
                  if [ -z "$N8N_DB_PASSWORD" ]; then
                    echo "Error: N8N_DB_PASSWORD environment variable must be set" >&2
                    exit 1
                  fi

                  echo "Connecting to n8n database..."
                  PGPASSWORD="$N8N_DB_PASSWORD" psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" "''${@:2}"
                  ;;
                tables)
                  # List tables in a database
                  shift
                  if [ "$1" = "pg" ]; then
                    DB_NAME="''${2:-postgres}"
                    DB_HOST="''${DB_HOST:-localhost}"
                    DB_PORT="''${DB_PORT:-5432}"
                    DB_USER="''${DB_USER:-postgres}"
                    psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" -c "\dt"
                  elif [ "$1" = "sqlite" ]; then
                    DB_FILE="''${2:-database.db}"
                    if [ ! -f "$DB_FILE" ]; then
                      echo "Error: SQLite database not found: $DB_FILE" >&2
                      exit 1
                    fi
                    sqlite3 "$DB_FILE" ".tables"
                  else
                    echo "Usage: db-query tables [pg|sqlite] <database>" >&2
                  fi
                  ;;
                *)
                  echo "Usage: db-query <type> <database> [query]"
                  echo ""
                  echo "Database types:"
                  echo "  pg|postgres <db> [query] - PostgreSQL database"
                  echo "  sqlite <file> [query]    - SQLite database file"
                  echo "  temporal [query]         - Temporal workflow database"
                  echo "  n8n [query]              - n8n automation database"
                  echo "  tables <type> <db>       - List tables in database"
                  echo ""
                  echo "Environment variables:"
                  echo "  DB_HOST, DB_PORT, DB_USER - PostgreSQL connection"
                  echo "  TEMPORAL_DB_* / N8N_DB_*  - Service-specific settings"
                  echo "  N8N_DB_PASSWORD           - Required for n8n database access"
                  echo ""
                  echo "Security Notes:"
                  echo "  - Use interactive mode (no query argument) for sensitive queries"
                  echo "  - Passwords in PGPASSWORD are visible in process listings"
                  echo "  - Consider using ~/.pgpass for PostgreSQL authentication"
                  echo ""
                  echo "Examples:"
                  echo "  db-query pg mydb 'SELECT * FROM users'"
                  echo "  db-query sqlite ./data.db '.schema'"
                  echo "  db-query temporal 'SELECT * FROM executions LIMIT 10'"
                  ;;
              esac
            '')
            # Temporal workflow management
            (pkgs.writeShellScriptBin "temporal-ctl" ''
              # Temporal workflow management
              # Usage: temporal-ctl [status|workflows|start|query]

              # Validate environment variables
              validate_host() {
                if [[ ! "$1" =~ ^[a-zA-Z0-9.-]+:[0-9]+$ ]] && [[ ! "$1" =~ ^[a-zA-Z0-9.-]+$ ]]; then
                  echo "Error: Invalid host format: $1" >&2
                  exit 1
                fi
              }
              
              validate_url() {
                if [[ ! "$1" =~ ^https?://[a-zA-Z0-9.-]+(:[0-9]+)?(/.*)?$ ]]; then
                  echo "Error: Invalid URL format: $1" >&2
                  exit 1
                fi
              }
              
              validate_namespace() {
                if [[ ! "$1" =~ ^[a-zA-Z0-9._-]+$ ]]; then
                  echo "Error: Invalid namespace format: $1 (must be alphanumeric with .-_)" >&2
                  exit 1
                fi
              }

              TEMPORAL_ADDRESS="''${TEMPORAL_ADDRESS:-localhost:7233}"
              TEMPORAL_NAMESPACE="''${TEMPORAL_NAMESPACE:-default}"
              TEMPORAL_UI="''${TEMPORAL_UI:-http://localhost:8088}"
              
              validate_host "$TEMPORAL_ADDRESS"
              validate_namespace "$TEMPORAL_NAMESPACE"
              validate_url "$TEMPORAL_UI"

              case "''${1:-status}" in
                status)
                  echo "Temporal Status"
                  echo "==============="
                  echo "  Address:   $TEMPORAL_ADDRESS"
                  echo "  Namespace: $TEMPORAL_NAMESPACE"
                  echo "  Web UI:    $TEMPORAL_UI"
                  echo ""
                  if command -v temporal >/dev/null 2>&1; then
                    temporal operator cluster health --address "$TEMPORAL_ADDRESS" 2>/dev/null && echo "  Status: HEALTHY" || echo "  Status: UNAVAILABLE"
                  elif curl -sf "$TEMPORAL_UI" >/dev/null 2>&1; then
                    echo "  Status: UI REACHABLE (CLI not installed)"
                  else
                    echo "  Status: UNAVAILABLE"
                    echo ""
                    echo "Start with: docker compose -f docker-compose.temporal.yml up -d"
                  fi
                  ;;
                workflows|list)
                  # List workflows
                  if command -v temporal >/dev/null 2>&1; then
                    echo "Recent Workflows (namespace: $TEMPORAL_NAMESPACE):"
                    temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" "''${@:2}"
                  else
                    echo "Temporal CLI not installed. Using API..."
                    curl -s "$TEMPORAL_UI/api/v1/namespaces/$TEMPORAL_NAMESPACE/workflows" 2>/dev/null | jq '.executions[:10]' || echo "Unable to query workflows"
                  fi
                  ;;
                running)
                  # List running workflows
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Running'"
                  ;;
                failed)
                  # List failed workflows
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Failed'"
                  ;;
                describe)
                  # Describe a workflow
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl describe <workflow-id>" >&2
                    exit 1
                  fi
                  temporal workflow describe --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
                  ;;
                history)
                  # Get workflow history
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl history <workflow-id>" >&2
                    exit 1
                  fi
                  temporal workflow show --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
                  ;;
                terminate)
                  # Terminate a workflow
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl terminate <workflow-id> [reason]" >&2
                    exit 1
                  fi
                  REASON="''${3:-Terminated via temporal-ctl}"
                  temporal workflow terminate --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2" --reason "$REASON"
                  ;;
                namespaces)
                  # List namespaces
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal operator namespace list --address "$TEMPORAL_ADDRESS"
                  ;;
                ui)
                  # Open Temporal UI
                  open-url-helper "$TEMPORAL_UI"
                  ;;
                *)
                  echo "Usage: temporal-ctl <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  status                - Check Temporal service status"
                  echo "  workflows|list        - List recent workflows"
                  echo "  running               - List running workflows"
                  echo "  failed                - List failed workflows"
                  echo "  describe <id>         - Describe workflow by ID"
                  echo "  history <id>          - Show workflow event history"
                  echo "  terminate <id> [msg]  - Terminate a workflow"
                  echo "  namespaces            - List namespaces"
                  echo "  ui                    - Open Temporal Web UI"
                  echo ""
                  echo "Environment:"
                  echo "  TEMPORAL_ADDRESS   - Server address (default: localhost:7233)"
                  echo "  TEMPORAL_NAMESPACE - Namespace (default: default)"
                  echo "  TEMPORAL_UI        - Web UI URL (default: http://localhost:8088)"
                  echo ""
                  echo "Security Note:"
                  echo "  Use HTTPS URLs for TEMPORAL_UI in production environments"
                  ;;
              esac
            '')
            # n8n workflow management
            (pkgs.writeShellScriptBin "n8n-ctl" ''
              # n8n workflow automation management
              # Usage: n8n-ctl [status|workflows|executions|start|stop]
              
              # Validate environment variables
              validate_host() {
                if [[ ! "$1" =~ ^[a-zA-Z0-9.-]+$ ]]; then
                  echo "Error: Invalid host format: $1" >&2
                  exit 1
                fi
              }
              
              validate_port() {
                if ! [[ "$1" =~ ^[0-9]+$ ]] || [ "$1" -lt 1 ] || [ "$1" -gt 65535 ]; then
                  echo "Error: Invalid port number: $1" >&2
                  exit 1
                fi
              }
              
              validate_workflow_id() {
                if [[ ! "$1" =~ ^[a-zA-Z0-9_-]+$ ]]; then
                  echo "Error: Invalid workflow ID format: $1" >&2
                  exit 1
                fi
              }

              N8N_HOST="''${N8N_HOST:-localhost}"
              N8N_PORT="''${N8N_PORT:-5678}"
              
              validate_host "$N8N_HOST"
              validate_port "$N8N_PORT"
              
              N8N_URL="http://$N8N_HOST:$N8N_PORT"
              N8N_API_KEY="''${N8N_API_KEY:-}"

              # API helper
              n8n_api() {
                if [ -n "$N8N_API_KEY" ]; then
                  curl -sf -H "X-N8N-API-KEY: $N8N_API_KEY" "$N8N_URL/api/v1$1"
                else
                  curl -sf "$N8N_URL/api/v1$1"
                fi
              }
              
              # API helper with method and data
              n8n_api_request() {
                local method="$1"
                local path="$2"
                local data="$3"
                
                if [ -n "$N8N_API_KEY" ]; then
                  curl -sf -X "$method" -H "Content-Type: application/json" \
                    -H "X-N8N-API-KEY: $N8N_API_KEY" \
                    -d "$data" "$N8N_URL/api/v1$path"
                else
                  curl -sf -X "$method" -H "Content-Type: application/json" \
                    -d "$data" "$N8N_URL/api/v1$path"
                fi
              }

              case "''${1:-status}" in
                status)
                  echo "n8n Status"
                  echo "=========="
                  echo "  URL: $N8N_URL"
                  echo ""
                  echo "  Security Note: Use HTTPS URLs in production environments"
                  echo "                 Set N8N_URL to https://... for encrypted communication"
                  echo ""
                  if curl -sf "$N8N_URL/healthz" >/dev/null 2>&1; then
                    echo "  Status: HEALTHY"
                    echo ""
                    echo "  Workflows: $(n8n_api '/workflows' 2>/dev/null | jq '.data | length' || echo 'N/A')"
                  else
                    echo "  Status: UNAVAILABLE"
                    echo ""
                    echo "Start with: docker compose -f docker-compose.automation.yml up -d"
                  fi
                  ;;
                workflows|list)
                  echo "n8n Workflows:"
                  n8n_api '/workflows' | jq -r '.data[] | "  [\(.active | if . then "ON" else "OFF" end)] \(.id): \(.name)"' 2>/dev/null || echo "Unable to list workflows"
                  ;;
                executions)
                  # List recent executions
                  LIMIT="''${2:-10}"
                  
                  # Validate LIMIT parameter
                  if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
                    echo "Error: LIMIT must be a positive integer" >&2
                    exit 1
                  fi
                  
                  echo "Recent Executions (last $LIMIT):"
                  n8n_api "/executions?limit=$LIMIT" | jq -r '.data[] | "  [\(.status)] \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list executions"
                  ;;
                failed)
                  # List failed executions
                  echo "Failed Executions:"
                  n8n_api '/executions?status=error&limit=20' | jq -r '.data[] | "  \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list failed executions"
                  ;;
                activate)
                  # Activate a workflow
                  if [ -z "$2" ]; then
                    echo "Usage: n8n-ctl activate <workflow-id>" >&2
                    exit 1
                  fi
                  
                  validate_workflow_id "$2"
                  
                  n8n_api_request "PATCH" "/workflows/$2" '{"active": true}' | jq '.active' 2>/dev/null
                  echo "Workflow $2 activated"
                  ;;
                deactivate)
                  # Deactivate a workflow
                  if [ -z "$2" ]; then
                    echo "Usage: n8n-ctl deactivate <workflow-id>" >&2
                    exit 1
                  fi
                  
                  validate_workflow_id "$2"
                  
                  n8n_api_request "PATCH" "/workflows/$2" '{"active": false}' | jq '.active' 2>/dev/null
                  echo "Workflow $2 deactivated"
                  ;;
                ui)
                  # Open n8n UI
                  open-url-helper "$N8N_URL"
                  ;;
                *)
                  echo "Usage: n8n-ctl <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  status              - Check n8n service status"
                  echo "  workflows|list      - List all workflows"
                  echo "  executions [limit]  - List recent executions"
                  echo "  failed              - List failed executions"
                  echo "  activate <id>       - Activate a workflow"
                  echo "  deactivate <id>     - Deactivate a workflow"
                  echo "  ui                  - Open n8n Web UI"
                  echo ""
                  echo "Environment:"
                  echo "  N8N_HOST    - n8n host (default: localhost)"
                  echo "  N8N_PORT    - n8n port (default: 5678)"
                  echo "  N8N_API_KEY - API key for authentication"
                  echo ""
                  echo "Security Notes:"
                  echo "  - Use HTTPS URLs in production (e.g., N8N_URL=https://...)"
                  echo "  - Always set N8N_API_KEY for authenticated access"
                  ;;
              esac
            '')
            # Workflow status dashboard
            (pkgs.writeShellScriptBin "workflow-status" ''
              # Combined workflow status dashboard
              # Usage: workflow-status [all|github|temporal|n8n|docker]

              check_service() {
                local name="$1"
                local url="$2"
                local status
                if curl -sf --connect-timeout 2 "$url" >/dev/null 2>&1; then
                  status="UP"
                else
                  status="DOWN"
                fi
                printf "  %-20s %s\n" "$name:" "$status"
              }

              case "''${1:-all}" in
                all)
                  echo "========================================"
                  echo "       Workflow Infrastructure Status"
                  echo "========================================"
                  echo ""

                  echo "Services:"
                  check_service "Temporal" "http://localhost:8088"
                  check_service "n8n" "http://localhost:5678/healthz"
                  check_service "Prometheus" "http://localhost:9090/-/ready"
                  check_service "NATS" "http://localhost:8222/varz"
                  check_service "Keycloak" "http://localhost:8080"
                  check_service "Vault" "http://localhost:8200/v1/sys/health"
                  echo ""

                  echo "GitHub Issues:"
                  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
                    OPEN=$(gh issue list --state open --json number --jq 'length' 2>/dev/null || echo "?")
                    printf "  %-20s %s\n" "Open issues:" "$OPEN"
                  else
                    echo "  (gh CLI not authenticated)"
                  fi
                  echo ""

                  echo "Docker Containers:"
                  if command -v docker >/dev/null 2>&1; then
                    RUNNING=$(docker ps -q 2>/dev/null | wc -l)
                    printf "  %-20s %s\n" "Running:" "$RUNNING"
                  else
                    echo "  (docker not available)"
                  fi
                  echo ""

                  echo "========================================"
                  ;;
                github)
                  echo "GitHub Status"
                  echo "============="
                  if command -v gh >/dev/null 2>&1; then
                    echo ""
                    echo "Issues:"
                    echo "  Open:   $(gh issue list --state open --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo "  Closed: $(gh issue list --state closed --limit 1000 --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo ""
                    echo "Pull Requests:"
                    echo "  Open:   $(gh pr list --state open --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo "  Merged: $(gh pr list --state merged --limit 100 --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo ""
                    echo "Recent Activity:"
                    gh issue list --limit 5 2>/dev/null | head -5
                  else
                    echo "gh CLI not available"
                  fi
                  ;;
                temporal)
                  exec temporal-ctl status
                  ;;
                n8n)
                  exec n8n-ctl status
                  ;;
                docker)
                  echo "Docker Container Status"
                  echo "======================="
                  if command -v docker >/dev/null 2>&1; then
                    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "Docker not running"
                  else
                    echo "Docker not available"
                  fi
                  ;;
                *)
                  echo "Usage: workflow-status [all|github|temporal|n8n|docker]"
                  echo ""
                  echo "Commands:"
                  echo "  all      - Show all services status (default)"
                  echo "  github   - GitHub issues and PRs status"
                  echo "  temporal - Temporal workflow engine status"
                  echo "  n8n      - n8n automation platform status"
                  echo "  docker   - Docker containers status"
                  ;;
              esac
            '')
            # Codebase database loader and query tool
            (pkgs.writeShellScriptBin "codebase-db" ''
              # Load codebase files into SQLite database for querying
              # Usage: codebase-db [init|load|query|search|files|stats]

              DB_FILE="''${CODEBASE_DB:-./codebase.db}"
              CODEBASE_ROOT="''${CODEBASE_ROOT:-.}"

              # File patterns to include (common source files)
              INCLUDE_PATTERNS="*.nix *.py *.rs *.go *.js *.ts *.tsx *.jsx *.sh *.bash *.zsh *.yml *.yaml *.toml *.json *.md *.txt *.cmake CMakeLists.txt *.xml package.xml *.launch *.launch.py *.msg *.srv *.action Dockerfile *.dockerfile docker-compose*.yml *.rego"

              # Directories to exclude
              EXCLUDE_DIRS=".git .pixi node_modules target build install log __pycache__ .cache .npm .cargo"

              init_db() {
                echo "Initializing codebase database: $DB_FILE"
                sqlite3 "$DB_FILE" << 'SCHEMA'
                  DROP TABLE IF EXISTS files;
                  DROP TABLE IF EXISTS file_lines;

                  CREATE TABLE files (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    path TEXT UNIQUE NOT NULL,
                    filename TEXT NOT NULL,
                    extension TEXT,
                    size_bytes INTEGER,
                    line_count INTEGER,
                    content TEXT,
                    loaded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                  );

                  CREATE TABLE file_lines (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    file_id INTEGER NOT NULL,
                    line_number INTEGER NOT NULL,
                    content TEXT,
                    FOREIGN KEY (file_id) REFERENCES files(id)
                  );

                  CREATE INDEX idx_files_path ON files(path);
                  CREATE INDEX idx_files_filename ON files(filename);
                  CREATE INDEX idx_files_extension ON files(extension);
                  CREATE INDEX idx_file_lines_file_id ON file_lines(file_id);
                  CREATE INDEX idx_file_lines_content ON file_lines(content);

                  CREATE VIRTUAL TABLE IF NOT EXISTS files_fts USING fts5(
                    path, filename, content, content='files', content_rowid='id'
                  );

                  CREATE TRIGGER files_ai AFTER INSERT ON files BEGIN
                    INSERT INTO files_fts(rowid, path, filename, content)
                    VALUES (new.id, new.path, new.filename, new.content);
                  END;
SCHEMA
                echo "Database initialized."
              }

              load_files() {
                echo "Loading files from: $CODEBASE_ROOT"
                echo "Database: $DB_FILE"
                echo ""

                # Build find command with exclusions
                EXCLUDE_ARGS=""
                for dir in $EXCLUDE_DIRS; do
                  EXCLUDE_ARGS="$EXCLUDE_ARGS -path '*/$dir' -prune -o -path '*/$dir/*' -prune -o"
                done

                # Build include patterns
                INCLUDE_ARGS=""
                first=1
                for pattern in $INCLUDE_PATTERNS; do
                  if [ $first -eq 1 ]; then
                    INCLUDE_ARGS="-name '$pattern'"
                    first=0
                  else
                    INCLUDE_ARGS="$INCLUDE_ARGS -o -name '$pattern'"
                  fi
                done

                # Find and load files
                count=0
                eval "find '$CODEBASE_ROOT' $EXCLUDE_ARGS \\( $INCLUDE_ARGS \\) -type f -print" 2>/dev/null | while read -r filepath; do
                  # Skip binary files
                  if file "$filepath" | grep -q "binary"; then
                    continue
                  fi

                  # Get file info
                  filename=$(basename "$filepath")
                  extension="''${filename##*.}"
                  [ "$extension" = "$filename" ] && extension=""
                  size=$(stat -c%s "$filepath" 2>/dev/null || stat -f%z "$filepath" 2>/dev/null || echo 0)

                  # Read content (limit to 1MB per file)
                  if [ "$size" -lt 1048576 ]; then
                    content=$(cat "$filepath" 2>/dev/null | sed "s/'/''/g")
                    line_count=$(echo "$content" | wc -l)

                    # Insert into database
                    sqlite3 "$DB_FILE" "INSERT OR REPLACE INTO files (path, filename, extension, size_bytes, line_count, content) VALUES ('$filepath', '$filename', '$extension', $size, $line_count, '$content');"

                    # Insert lines for detailed searching
                    file_id=$(sqlite3 "$DB_FILE" "SELECT id FROM files WHERE path='$filepath';")
                    sqlite3 "$DB_FILE" "DELETE FROM file_lines WHERE file_id=$file_id;"

                    line_num=1
                    echo "$content" | while IFS= read -r line; do
                      escaped_line=$(echo "$line" | sed "s/'/''/g")
                      sqlite3 "$DB_FILE" "INSERT INTO file_lines (file_id, line_number, content) VALUES ($file_id, $line_num, '$escaped_line');"
                      line_num=$((line_num + 1))
                    done

                    count=$((count + 1))
                    echo "  Loaded: $filepath ($line_count lines)"
                  else
                    echo "  Skipped (too large): $filepath"
                  fi
                done

                echo ""
                echo "Loading complete. Run 'codebase-db stats' to see summary."
              }

              case "''${1:-help}" in
                init)
                  init_db
                  ;;
                load)
                  if [ ! -f "$DB_FILE" ]; then
                    init_db
                  fi
                  load_files
                  ;;
                reload)
                  init_db
                  load_files
                  ;;
                query|q)
                  # Execute arbitrary SQL query
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db query '<SQL>'" >&2
                    echo ""
                    echo "Examples:"
                    echo "  codebase-db query 'SELECT path FROM files WHERE extension=\"nix\"'"
                    echo "  codebase-db query 'SELECT * FROM files WHERE content LIKE \"%TODO%\"'"
                    exit 1
                  fi
                  shift
                  sqlite3 -header -column "$DB_FILE" "$*"
                  ;;
                search|s)
                  # Full-text search in file contents
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db search <pattern>" >&2
                    exit 1
                  fi
                  shift
                  PATTERN="$*"
                  echo "Searching for: $PATTERN"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT f.path, fl.line_number, fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 100;"
                  ;;
                grep|g)
                  # Search with context (like grep)
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db grep <pattern> [context_lines]" >&2
                    exit 1
                  fi
                  PATTERN="$2"
                  CONTEXT="''${3:-2}"
                  echo "Searching for: $PATTERN (context: $CONTEXT lines)"
                  echo ""
                  sqlite3 "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number || ': ' || fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 50;" | while read -r line; do
                    echo "$line"
                  done
                  ;;
                files|ls)
                  # List files, optionally filtered by extension
                  EXT="$2"
                  if [ -n "$EXT" ]; then
                    echo "Files with extension: $EXT"
                    sqlite3 -column "$DB_FILE" \
                      "SELECT path, line_count, size_bytes FROM files WHERE extension='$EXT' ORDER BY path;"
                  else
                    echo "All indexed files:"
                    sqlite3 -column "$DB_FILE" \
                      "SELECT path, extension, line_count FROM files ORDER BY path;"
                  fi
                  ;;
                functions|fn)
                  # Find function definitions (basic pattern matching)
                  LANG="''${2:-all}"
                  echo "Function definitions ($LANG):"
                  echo ""
                  case "$LANG" in
                    py|python)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'py' AND fl.content LIKE '%def %(%'
                         ORDER BY f.path, fl.line_number;"
                      ;;
                    sh|bash)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension IN ('sh', 'bash') AND (fl.content LIKE '%() {%' OR fl.content LIKE '%function %')
                         ORDER BY f.path, fl.line_number;"
                      ;;
                    nix)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'nix' AND fl.content LIKE '% = %{%' AND fl.content LIKE '%:%'
                         ORDER BY f.path, fl.line_number
                         LIMIT 50;"
                      ;;
                    *)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, substr(fl.content, 1, 80)
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE fl.content LIKE '%def %' OR fl.content LIKE '%function %' OR fl.content LIKE '%fn %' OR fl.content LIKE '%func %'
                         ORDER BY f.path, fl.line_number
                         LIMIT 100;"
                      ;;
                  esac
                  ;;
                todos)
                  # Find TODO/FIXME comments
                  echo "TODO/FIXME items:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%TODO%' OR fl.content LIKE '%FIXME%' OR fl.content LIKE '%XXX%' OR fl.content LIKE '%HACK%'
                     ORDER BY f.path, fl.line_number;"
                  ;;
                issues)
                  # Find potential issues (errors, warnings in code)
                  echo "Potential issues (error handling, warnings):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%error%' OR fl.content LIKE '%Error%' OR fl.content LIKE '%WARNING%' OR fl.content LIKE '%WARN%' OR fl.content LIKE '%panic%' OR fl.content LIKE '%fail%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 100;"
                  ;;
                stats)
                  # Show database statistics
                  echo "Codebase Database Statistics"
                  echo "============================"
                  echo ""
                  echo "Database: $DB_FILE"
                  echo ""
                  sqlite3 "$DB_FILE" << 'STATS'
                    SELECT 'Total files:' as metric, COUNT(*) as value FROM files
                    UNION ALL
                    SELECT 'Total lines:', SUM(line_count) FROM files
                    UNION ALL
                    SELECT 'Total size (bytes):', SUM(size_bytes) FROM files;
STATS
                  echo ""
                  echo "Files by extension:"
                  sqlite3 -column "$DB_FILE" \
                    "SELECT extension, COUNT(*) as count, SUM(line_count) as lines
                     FROM files
                     WHERE extension != ''
                     GROUP BY extension
                     ORDER BY count DESC
                     LIMIT 20;"
                  ;;
                export)
                  # Export search results to file
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db export <pattern> <output-file>" >&2
                    exit 1
                  fi
                  PATTERN="$2"
                  OUTPUT="$3"
                  echo "Exporting matches for '$PATTERN' to $OUTPUT..."
                  sqlite3 "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number || ': ' || fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number;" > "$OUTPUT"
                  echo "Exported $(wc -l < "$OUTPUT") matches."
                  ;;
                shell)
                  # Open interactive SQLite shell
                  echo "Opening SQLite shell for: $DB_FILE"
                  echo "Tables: files, file_lines, files_fts"
                  echo ""
                  sqlite3 "$DB_FILE"
                  ;;
                *)
                  echo "Usage: codebase-db <command> [args]"
                  echo ""
                  echo "Database Commands:"
                  echo "  init              - Initialize empty database"
                  echo "  load              - Load/update files into database"
                  echo "  reload            - Reinitialize and reload all files"
                  echo "  stats             - Show database statistics"
                  echo "  shell             - Open interactive SQLite shell"
                  echo ""
                  echo "Search Commands:"
                  echo "  search <pattern>  - Full-text search in file contents"
                  echo "  grep <pattern>    - Search with file:line output"
                  echo "  query '<SQL>'     - Execute arbitrary SQL query"
                  echo "  files [ext]       - List indexed files (optionally by extension)"
                  echo "  functions [lang]  - Find function definitions"
                  echo "  todos             - Find TODO/FIXME comments"
                  echo "  issues            - Find potential error handling issues"
                  echo "  export <p> <f>    - Export search results to file"
                  echo ""
                  echo "Environment:"
                  echo "  CODEBASE_DB   - Database file (default: ./codebase.db)"
                  echo "  CODEBASE_ROOT - Root directory to scan (default: .)"
                  echo ""
                  echo "Examples:"
                  echo "  codebase-db load                    # Load all files"
                  echo "  codebase-db search 'error handling' # Search for pattern"
                  echo "  codebase-db files nix               # List all .nix files"
                  echo "  codebase-db todos                   # Find all TODOs"
                  echo "  codebase-db query 'SELECT * FROM files WHERE path LIKE \"%flake%\"'"
                  ;;
              esac
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
              ++ workflowCommandWrappers
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
            packages = basePackages ++ fullExtras ++ holochainPackages ++ coreCommandWrappers ++ ros2CommandWrappers ++ infraCommandWrappers ++ securityCommandWrappers ++ devCommandWrappers ++ workflowCommandWrappers ++ aiCommandWrappers ++ linuxPackages ++ (with pkgs; [
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
