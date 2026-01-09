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

    # P2-017: Agentic DevOps automation layer (BUILDKIT_STARTER_SPEC.md L301)
    # Provides: Autonomous DevOps and remediation capabilities
    # See: https://github.com/agenticsorg/devops
    agenticsorg-devops = {
      url = "github:agenticsorg/devops";
      flake = false;  # Not a flake, just source reference
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

          # P2-017: AgenticsOrg DevOps automation layer reference
          # Source is available at: inputs.agenticsorg-devops
          # Usage: Provides autonomous DevOps and remediation capabilities
          # Documentation: ${inputs.agenticsorg-devops}/README.md
          agenticsorgDevopsSrc = inputs.agenticsorg-devops;

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
          # P3-006: Holochain reference tools (Phase 3 - Development tooling)
          #
          # The 'hc' CLI provides comprehensive development commands:
          #   - hc sandbox    : Generate and run test networks for development
          #   - hc scaffold   : Generate DNA, zome, and entry type templates
          #   - hc dna        : DNA operations (init, pack, unpack)
          #   - hc app        : hApp bundle operations (pack, unpack)
          #   - hc web-app    : Web hApp operations
          #
          # For additional launch capabilities:
          #   - Use 'hc sandbox' for local development environments
          #   - For production: holochain conductor with conductor.yaml config
          #   - Alternative: Install @holochain/hc-spin via npm for enhanced DX
          #
          # References:
          #   - Holochain Developer Docs: https://developer.holochain.org
          #   - hc CLI source: https://github.com/holochain/holochain (crates/hc)
          #   - Nix overlay: https://github.com/spartan-holochain-counsel/nix-overlay
          holochainPackages = with pkgs; [
            holochain       # Holochain conductor (agent-centric P2P runtime)
            hc              # Holochain dev CLI (scaffold/sandbox/pack/launch)
            lair-keystore   # Secure keystore for Holochain agent keys (cryptographic identity)
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
            jujutsu              # P2-013: Modern Git-compatible VCS (jj command)

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
            # Layer 3: 33% -> 100% coverage (P0-001, P0-002)
            firecracker         # MicroVM for untrusted workloads
            runc                # OCI container runtime (low-level)
            kata-runtime        # P0-002: Kata Containers runtime (lightweight VMs)

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

            # P3-002: prompt-cache - LLM Semantic Caching Proxy (Go binary)
            # See: https://github.com/messkan/prompt-cache
            # Description: Drop-in, provider-agnostic LLM proxy for semantic caching
            # Features:
            #   - Reduces LLM costs by up to 80% via semantic similarity detection
            #   - Sub-millisecond cached responses
            #   - Embedded BadgerDB for persistent storage
            #   - Multi-provider support: OpenAI, Mistral, Claude (Anthropic)
            # Installation (NOT in nixpkgs - install from source):
            #   Option 1 (Docker Compose - Recommended):
            #     git clone https://github.com/messkan/prompt-cache.git
            #     cd prompt-cache && docker-compose up -d
            #   Option 2 (From source):
            #     git clone https://github.com/messkan/prompt-cache.git
            #     cd prompt-cache && make run
            # Usage:
            #   # Point your OpenAI client to the proxy:
            #   client = OpenAI(base_url="http://localhost:8080/v1", api_key="your-key")
            # NOTE: Requires Go 1.25+ to build from source
            # NOTE: Complementary to P3-001 (vCache) - use together for best results

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
            # P3-002: PromptCache LLM Proxy management
            (pkgs.writeShellScriptBin "prompt-cache" ''
              # PromptCache semantic caching proxy management
              # Usage: prompt-cache [install|up|down|logs|status]
              PROMPTCACHE_DIR="''${PROMPTCACHE_DIR:-$HOME/.local/share/prompt-cache}"
              PROMPTCACHE_PORT="''${PROMPTCACHE_PORT:-8080}"

              case "''${1:-status}" in
                install)
                  echo "Installing PromptCache..."
                  mkdir -p "$PROMPTCACHE_DIR"
                  if [ ! -d "$PROMPTCACHE_DIR/prompt-cache" ]; then
                    git clone https://github.com/messkan/prompt-cache.git "$PROMPTCACHE_DIR/prompt-cache"
                    echo ""
                    echo "PromptCache installed to: $PROMPTCACHE_DIR/prompt-cache"
                    echo ""
                    echo "Configure environment variables:"
                    echo "  OPENAI_API_KEY        - Your OpenAI API key"
                    echo "  VOYAGE_API_KEY        - Your Voyage AI API key (for Claude embeddings)"
                    echo "  PROMPTCACHE_PORT      - Server port (default: 8080)"
                    echo ""
                    echo "Start with: prompt-cache up"
                  else
                    echo "PromptCache already installed at $PROMPTCACHE_DIR/prompt-cache"
                    echo "To update: cd $PROMPTCACHE_DIR/prompt-cache && git pull"
                  fi
                  ;;
                up)
                  if [ ! -d "$PROMPTCACHE_DIR/prompt-cache" ]; then
                    echo "PromptCache not installed. Run: prompt-cache install"
                    exit 1
                  fi
                  echo "Starting PromptCache on port $PROMPTCACHE_PORT..."
                  cd "$PROMPTCACHE_DIR/prompt-cache"
                  if [ -f "docker-compose.yml" ]; then
                    docker compose up -d "''${@:2}"
                    echo ""
                    echo "PromptCache API: http://localhost:$PROMPTCACHE_PORT/v1"
                    echo "Point your OpenAI client to this URL for semantic caching"
                  else
                    echo "Starting from source (requires Go 1.25+)..."
                    make run &
                    echo "PromptCache API: http://localhost:$PROMPTCACHE_PORT/v1"
                  fi
                  ;;
                down)
                  if [ -f "$PROMPTCACHE_DIR/prompt-cache/docker-compose.yml" ]; then
                    cd "$PROMPTCACHE_DIR/prompt-cache"
                    docker compose down "''${@:2}"
                  else
                    pkill -f "prompt-cache" && echo "PromptCache stopped" || echo "PromptCache not running"
                  fi
                  ;;
                logs)
                  if [ -f "$PROMPTCACHE_DIR/prompt-cache/docker-compose.yml" ]; then
                    cd "$PROMPTCACHE_DIR/prompt-cache"
                    docker compose logs -f "''${@:2}"
                  else
                    echo "No docker-compose setup found. Check process logs manually."
                  fi
                  ;;
                status)
                  if pgrep -f "prompt-cache" > /dev/null || docker ps | grep -q "prompt-cache"; then
                    echo "PromptCache is running on port $PROMPTCACHE_PORT"
                    curl -s "http://localhost:$PROMPTCACHE_PORT/health" > /dev/null 2>&1 && echo "  API ready: http://localhost:$PROMPTCACHE_PORT/v1" || echo "  API not responding"
                  else
                    echo "PromptCache is not running"
                    echo "  Install: prompt-cache install"
                    echo "  Start:   prompt-cache up"
                  fi
                  ;;
                *)
                  echo "Usage: prompt-cache [install|up|down|logs|status]"
                  echo "  install - Clone PromptCache repository"
                  echo "  up      - Start PromptCache proxy (port $PROMPTCACHE_PORT)"
                  echo "  down    - Stop PromptCache proxy"
                  echo "  logs    - Follow PromptCache logs (Docker only)"
                  echo "  status  - Check if PromptCache is running"
                  echo ""
                  echo "Integration with vCache (P3-001):"
                  echo "  Use PromptCache as a proxy + vCache for application-level caching"
                  echo "  PromptCache: Provider-agnostic proxy with BadgerDB persistence"
                  echo "  vCache: Python library with guaranteed error bounds"
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
            # P0-001: sandbox-runtime - Process sandbox for MCP/tool execution
            # BUILDKIT_STARTER_SPEC.md Layer 3 (Isolation) and Layer 8 (Tool Execution)
            # Uses Anthropic's sandbox-runtime via npx for on-demand execution
            (pkgs.writeShellScriptBin "sandbox-runtime" ''
              # Sandbox-runtime wrapper (P0-001)
              # Anthropic's sandbox for enforcing filesystem and network restrictions
              # Runs via npx to use the latest version from npm

              # Check if globally installed first (faster)
              if command -v sandbox-runtime >/dev/null 2>&1; then
                exec sandbox-runtime "$@"
              fi

              # Fall back to npx for on-demand execution
              # Package: @anthropic-ai/sandbox-runtime
              if command -v npx >/dev/null 2>&1; then
                exec npx --yes @anthropic-ai/sandbox-runtime "$@"
              else
                echo "Error: sandbox-runtime requires Node.js and npx" >&2
                echo "" >&2
                echo "Installation options:" >&2
                echo "" >&2
                echo "  1. Global install (recommended):" >&2
                echo "     npm install -g @anthropic-ai/sandbox-runtime" >&2
                echo "" >&2
                echo "  2. Use via npx (available in this environment):" >&2
                echo "     npx @anthropic-ai/sandbox-runtime <command>" >&2
                echo "" >&2
                echo "Platform requirements:" >&2
                echo "  - Linux: bubblewrap, socat, ripgrep" >&2
                echo "  - macOS: ripgrep (uses native sandbox-exec)" >&2
                echo "" >&2
                echo "Layer 3 Isolation - P0-001 (Critical)" >&2
                exit 127
              fi
            '')
            # P0-007: genai-toolbox (MCP Toolbox for Databases)
            # BUILDKIT_STARTER_SPEC.md Layer 8 (Tool Execution)
            # MCP server for AI tool integration with database access
            (pkgs.writeShellScriptBin "mcp-toolbox" ''
              # MCP Toolbox (genai-toolbox) wrapper
              # Provides the Model Context Protocol server for database tool execution
              # See: https://github.com/googleapis/genai-toolbox

              # Check if installed via home-manager module
              if command -v mcp-toolbox >/dev/null 2>&1; then
                exec mcp-toolbox "$@"
              else
                echo "MCP Toolbox (genai-toolbox) not found." >&2
                echo "" >&2
                echo "Installation options:" >&2
                echo "" >&2
                echo "  1. Enable home-manager module (recommended):" >&2
                echo "     programs.genai-toolbox.enable = true;" >&2
                echo "" >&2
                echo "  2. Install via Homebrew:" >&2
                echo "     brew install mcp-toolbox" >&2
                echo "" >&2
                echo "  3. Install via Go:" >&2
                echo "     go install github.com/googleapis/genai-toolbox@v0.25.0" >&2
                echo "" >&2
                echo "  4. Download binary from releases:" >&2
                echo "     https://github.com/googleapis/genai-toolbox/releases" >&2
                echo "" >&2
                echo "  5. Use NPX:" >&2
                echo "     npx @toolbox-sdk/server" >&2
                echo "" >&2
                echo "Layer 8 Tool Execution - P0-007 (Critical)" >&2
                exit 127
              fi
            '')
            # P2-009: sublinear-time-solver - Fast solver via npx
            # BUILDKIT_STARTER_SPEC.md Layer 8 (Tool Execution)
            # See: https://github.com/ruvnet/sublinear-time-solver
            (pkgs.writeShellScriptBin "sublinear-solver" ''
              # Sublinear-time solver wrapper (P2-009)
              # Rust + WASM sublinear-time solver for asymmetric diagonally dominant systems
              # Runs via npx for on-demand execution
              if command -v npx >/dev/null 2>&1; then
                exec npx sublinear-time-solver "$@"
              else
                echo "ERROR: npx not found" >&2
                echo "" >&2
                echo "sublinear-time-solver requires Node.js and npx." >&2
                echo "" >&2
                echo "Install Node.js:" >&2
                echo "  - Via Nix: Available in this environment" >&2
                echo "  - Via pixi: Already configured" >&2
                echo "  - System: https://nodejs.org" >&2
                echo "" >&2
                echo "Usage: sublinear-solver [command] [options]" >&2
                echo "  sublinear-solver mcp       # Start MCP server" >&2
                echo "  sublinear-solver --help    # Show help" >&2
                echo "" >&2
                echo "Layer 8 Tool Execution - P2-009" >&2
                exit 127
              fi
            '')
            # P0-002: Kata Containers management wrapper
            # BUILDKIT_STARTER_SPEC.md Layer 3 (Isolation)
            # Kata Containers runtime available via nixpkgs (kata-runtime)
            (pkgs.writeShellScriptBin "kata" ''
              # Kata Containers wrapper (P0-002)
              # Management interface for Kata Containers lightweight VM runtime
              # Runtime provided by nixpkgs package: kata-runtime

              case "''${1:-status}" in
                status)
                  echo "Kata Containers Status"
                  echo "======================"
                  if command -v kata-runtime >/dev/null 2>&1; then
                    echo "Runtime: AVAILABLE"
                    kata-runtime --version
                    echo ""
                    echo "Docker Configuration:"
                    if command -v docker >/dev/null 2>&1; then
                      if docker info 2>/dev/null | grep -i kata >/dev/null; then
                        echo "  Status: Configured as Docker runtime"
                      else
                        echo "  Status: Not configured (run 'kata config-docker')"
                      fi
                    else
                      echo "  Docker: Not installed"
                    fi
                  else
                    echo "Runtime: NOT AVAILABLE"
                    echo ""
                    echo "Kata Containers is available via nixpkgs in this environment."
                    echo "If kata-runtime is not in PATH, ensure you're in the nix shell:"
                    echo "  nix develop .#full"
                  fi
                  ;;
                check)
                  if command -v kata-runtime >/dev/null 2>&1; then
                    echo "Running Kata runtime checks..."
                    kata-runtime check
                  else
                    echo "Error: kata-runtime not found in PATH" >&2
                    echo "Ensure you're in the nix development environment" >&2
                    exit 1
                  fi
                  ;;
                config-docker)
                  if ! command -v docker >/dev/null 2>&1; then
                    echo "Error: Docker not installed" >&2
                    exit 1
                  fi
                  echo "Configuring Docker to use Kata Containers..."
                  echo ""
                  KATA_PATH=$(command -v kata-runtime)
                  echo "Detected kata-runtime at: $KATA_PATH"
                  echo ""
                  echo "Add this to /etc/docker/daemon.json (requires sudo):"
                  echo ""
                  cat << DOCKERCFG
{
  "runtimes": {
    "kata": {
      "path": "$KATA_PATH"
    }
  }
}
DOCKERCFG
                  echo ""
                  echo "Then restart Docker:"
                  echo "  sudo systemctl restart docker"
                  ;;
                test)
                  if ! command -v docker >/dev/null 2>&1; then
                    echo "Error: Docker not installed" >&2
                    exit 1
                  fi
                  echo "Testing Kata runtime with Docker..."
                  echo "Command: docker run --rm --runtime=kata alpine cat /etc/os-release"
                  echo ""
                  docker run --rm --runtime=kata alpine cat /etc/os-release
                  ;;
                version)
                  if command -v kata-runtime >/dev/null 2>&1; then
                    kata-runtime --version
                  else
                    echo "kata-runtime not found" >&2
                    exit 1
                  fi
                  ;;
                *)
                  echo "Usage: kata <command>"
                  echo ""
                  echo "Commands:"
                  echo "  status        - Show Kata Containers status and configuration"
                  echo "  check         - Run kata-runtime system checks"
                  echo "  config-docker - Show Docker configuration for Kata"
                  echo "  test          - Test Kata runtime with Docker"
                  echo "  version       - Show kata-runtime version"
                  echo ""
                  echo "Environment:"
                  echo "  Kata runtime provided by nixpkgs (kata-runtime package)"
                  echo "  Ensure you're in the nix development environment:"
                  echo "    nix develop .#full"
                  echo ""
                  echo "Layer 3 Isolation - P0-002 (Critical)"
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
            # P1-012: SWC compiler wrapper
            # Fast TypeScript/JavaScript compiler written in Rust
            # 10x faster than tsc for transpilation and bundling
            (pkgs.writeShellScriptBin "swc" ''
              # SWC (Speedy Web Compiler) - Fast TypeScript/JavaScript compiler
              # Usage: swc <input> [options]
              # Examples:
              #   swc src/index.ts -o dist/index.js
              #   swc src/ -d dist/
              #
              # Uses npx to avoid 150MB node_modules bloat
              # See: https://github.com/NixOS/nixpkgs/issues/195677
              exec npx -y @swc/cli@latest "$@"
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
                # Allow hostname:port or just hostname format
                if [[ "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?(:[0-9]+)?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid host format: $1" >&2
                exit 1
              }
              
              validate_url() {
                # Basic URL validation for http/https
                if [[ "$1" =~ ^https?://[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?(:[0-9]+)?(/.*)?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid URL format: $1" >&2
                exit 1
              }
              
              validate_namespace() {
                if [[ "$1" =~ ^[a-zA-Z0-9._-]+$ ]]; then
                  return 0
                fi
                echo "Error: Invalid namespace format: $1 (must be alphanumeric with .-_)" >&2
                exit 1
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
                # Hostname format validation
                if [[ "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid host format: $1" >&2
                exit 1
              }
              
              validate_port() {
                if [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ]; then
                  return 0
                fi
                echo "Error: Invalid port number: $1 (must be 1-65535)" >&2
                exit 1
              }
              
              validate_workflow_id() {
                # Require alphanumeric start, allow hyphens/underscores in the middle
                if [[ "$1" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
                  return 0
                fi
                echo "Error: Invalid workflow ID format: $1 (must start with alphanumeric)" >&2
                exit 1
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
                  echo "  Security Note: For production, configure N8N_HOST to point to HTTPS endpoints"
                  echo "                 (e.g., N8N_HOST=n8n.example.com N8N_PORT=443)"
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
                  echo "  - Configure N8N_HOST to use HTTPS endpoints in production"
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
              # Load ALL codebase files into SQLite database for comprehensive querying
              # Designed for configuration workspaces - indexes everything
              # Usage: codebase-db [init|load|query|search|files|stats|configs|deps|envvars|...]

              DB_FILE="''${CODEBASE_DB:-./codebase.db}"
              CODEBASE_ROOT="''${CODEBASE_ROOT:-.}"

              # Directories to exclude (only truly non-content dirs)
              EXCLUDE_DIRS=".git node_modules __pycache__ .cache .npm .cargo .mypy_cache .pytest_cache"

              # Max file size to load (5MB - generous for config files)
              MAX_FILE_SIZE=5242880

              init_db() {
                echo "Initializing codebase database: $DB_FILE"
                sqlite3 "$DB_FILE" << 'SCHEMA'
                  DROP TABLE IF EXISTS files;
                  DROP TABLE IF EXISTS file_lines;
                  DROP TABLE IF EXISTS files_fts;
                  DROP TABLE IF EXISTS metadata;

                  CREATE TABLE files (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    path TEXT UNIQUE NOT NULL,
                    relative_path TEXT NOT NULL,
                    filename TEXT NOT NULL,
                    extension TEXT,
                    directory TEXT NOT NULL,
                    size_bytes INTEGER,
                    line_count INTEGER,
                    content TEXT,
                    file_type TEXT,
                    is_config INTEGER DEFAULT 0,
                    is_hidden INTEGER DEFAULT 0,
                    loaded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                  );

                  CREATE TABLE file_lines (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    file_id INTEGER NOT NULL,
                    line_number INTEGER NOT NULL,
                    content TEXT,
                    trimmed_content TEXT,
                    indent_level INTEGER DEFAULT 0,
                    FOREIGN KEY (file_id) REFERENCES files(id)
                  );

                  CREATE TABLE metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                  );

                  CREATE INDEX idx_files_path ON files(path);
                  CREATE INDEX idx_files_relative_path ON files(relative_path);
                  CREATE INDEX idx_files_filename ON files(filename);
                  CREATE INDEX idx_files_extension ON files(extension);
                  CREATE INDEX idx_files_directory ON files(directory);
                  CREATE INDEX idx_files_file_type ON files(file_type);
                  CREATE INDEX idx_files_is_config ON files(is_config);
                  CREATE INDEX idx_file_lines_file_id ON file_lines(file_id);
                  CREATE INDEX idx_file_lines_content ON file_lines(content);
                  CREATE INDEX idx_file_lines_trimmed ON file_lines(trimmed_content);

                  CREATE VIRTUAL TABLE files_fts USING fts5(
                    path, filename, content, tokenize='porter unicode61'
                  );

                  CREATE VIRTUAL TABLE lines_fts USING fts5(
                    content, tokenize='porter unicode61'
                  );
SCHEMA
                echo "Database initialized with enhanced schema."
              }

              # Determine file type based on extension/name
              get_file_type() {
                local file="$1"
                local ext="$2"
                local name=$(basename "$file")
                case "$name" in
                  Dockerfile*|*.dockerfile) echo "dockerfile" ;;
                  Makefile|GNUmakefile) echo "makefile" ;;
                  CMakeLists.txt) echo "cmake" ;;
                  *.nix) echo "nix" ;;
                  *.py) echo "python" ;;
                  *.sh|*.bash|*.zsh) echo "shell" ;;
                  *.ps1|*.psm1) echo "powershell" ;;
                  *.yml|*.yaml) echo "yaml" ;;
                  *.json) echo "json" ;;
                  *.toml) echo "toml" ;;
                  *.xml) echo "xml" ;;
                  *.md|*.markdown) echo "markdown" ;;
                  *.rst) echo "rst" ;;
                  *.txt) echo "text" ;;
                  *.ini|*.cfg|*.conf) echo "ini" ;;
                  *.env*|.env*) echo "env" ;;
                  *.lock) echo "lockfile" ;;
                  *.rs) echo "rust" ;;
                  *.go) echo "go" ;;
                  *.js|*.mjs|*.cjs) echo "javascript" ;;
                  *.ts|*.tsx) echo "typescript" ;;
                  *.lua) echo "lua" ;;
                  *.vim) echo "vim" ;;
                  *.css|*.scss|*.sass) echo "css" ;;
                  *.html|*.htm) echo "html" ;;
                  *.sql) echo "sql" ;;
                  *.rego) echo "rego" ;;
                  *.hcl|*.tf) echo "hcl" ;;
                  *) echo "other" ;;
                esac
              }

              # Check if file is a config file
              is_config_file() {
                local file="$1"
                local name=$(basename "$file")
                case "$name" in
                  *.nix|*.toml|*.yaml|*.yml|*.json|*.ini|*.cfg|*.conf|*.env*|.env*|\
                  *.xml|*.hcl|*.tf|flake.lock|Cargo.lock|package-lock.json|pixi.lock|\
                  .gitignore|.gitattributes|.editorconfig|.envrc|.pre-commit*|\
                  Dockerfile*|docker-compose*|Makefile|CMakeLists.txt|*.cmake|\
                  tsconfig.json|jsconfig.json|*.config.js|*.config.ts|\
                  pyproject.toml|setup.py|setup.cfg|requirements*.txt|\
                  Gemfile|Rakefile|*.gemspec|Cargo.toml|go.mod|go.sum)
                    echo 1 ;;
                  *) echo 0 ;;
                esac
              }

              load_files() {
                echo "Loading ALL files from: $CODEBASE_ROOT"
                echo "Database: $DB_FILE"
                echo "Max file size: $MAX_FILE_SIZE bytes"
                echo ""

                # Record start time
                start_time=$(date +%s)

                # Build find exclusions
                EXCLUDE_ARGS=""
                for dir in $EXCLUDE_DIRS; do
                  EXCLUDE_ARGS="$EXCLUDE_ARGS -path '*/$dir' -prune -o -path '*/$dir/*' -prune -o"
                done

                # Find ALL files (no extension filtering)
                count=0
                skipped=0
                binary=0

                eval "find '$CODEBASE_ROOT' $EXCLUDE_ARGS -type f -print" 2>/dev/null | while read -r filepath; do
                  filename=$(basename "$filepath")
                  relative_path="''${filepath#$CODEBASE_ROOT/}"
                  directory=$(dirname "$relative_path")
                  extension="''${filename##*.}"
                  [ "$extension" = "$filename" ] && extension=""
                  is_hidden=0
                  [[ "$filename" == .* ]] && is_hidden=1

                  # Get file size
                  size=$(stat -c%s "$filepath" 2>/dev/null || stat -f%z "$filepath" 2>/dev/null || echo 0)

                  # Skip files that are too large
                  if [ "$size" -gt "$MAX_FILE_SIZE" ]; then
                    echo "  Skipped (>5MB): $relative_path"
                    skipped=$((skipped + 1))
                    continue
                  fi

                  # Skip binary files
                  if file "$filepath" 2>/dev/null | grep -qE "binary|executable|ELF|Mach-O|PE32|archive|compressed"; then
                    binary=$((binary + 1))
                    continue
                  fi

                  # Get file type and config status
                  file_type=$(get_file_type "$filepath" "$extension")
                  is_config=$(is_config_file "$filepath")

                  # Read content
                  content=$(cat "$filepath" 2>/dev/null | sed "s/'/''/g" || echo "")
                  line_count=$(echo "$content" | wc -l | tr -d ' ')

                  # Insert file record
                  sqlite3 "$DB_FILE" "INSERT OR REPLACE INTO files
                    (path, relative_path, filename, extension, directory, size_bytes, line_count, content, file_type, is_config, is_hidden)
                    VALUES ('$filepath', '$relative_path', '$filename', '$extension', '$directory', $size, $line_count, '$content', '$file_type', $is_config, $is_hidden);"

                  # Get file ID and insert into FTS
                  file_id=$(sqlite3 "$DB_FILE" "SELECT id FROM files WHERE path='$filepath';")
                  sqlite3 "$DB_FILE" "INSERT INTO files_fts(rowid, path, filename, content) VALUES ($file_id, '$filepath', '$filename', '$content');"

                  # Clear old lines and insert new
                  sqlite3 "$DB_FILE" "DELETE FROM file_lines WHERE file_id=$file_id;"

                  line_num=1
                  echo "$content" | while IFS= read -r line; do
                    escaped_line=$(echo "$line" | sed "s/'/''/g")
                    trimmed=$(echo "$line" | sed 's/^[[:space:]]*//' | sed "s/'/''/g")
                    indent=$(echo "$line" | sed 's/[^[:space:]].*//' | wc -c)
                    indent=$((indent - 1))
                    sqlite3 "$DB_FILE" "INSERT INTO file_lines (file_id, line_number, content, trimmed_content, indent_level)
                      VALUES ($file_id, $line_num, '$escaped_line', '$trimmed', $indent);"
                    line_num=$((line_num + 1))
                  done

                  # Insert lines into FTS
                  sqlite3 "$DB_FILE" "INSERT INTO lines_fts(rowid, content)
                    SELECT id, content FROM file_lines WHERE file_id=$file_id;"

                  count=$((count + 1))
                  echo "  [$file_type] $relative_path ($line_count lines)"
                done

                end_time=$(date +%s)
                duration=$((end_time - start_time))

                # Store metadata
                sqlite3 "$DB_FILE" "INSERT OR REPLACE INTO metadata (key, value) VALUES
                  ('root_path', '$CODEBASE_ROOT'),
                  ('load_time', '$duration'),
                  ('last_loaded', datetime('now'));"

                echo ""
                echo "Loading complete in ''${duration}s. Run 'codebase-db stats' for summary."
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

                # === CONFIGURATION QUERY TOOLS ===

                configs)
                  # List all configuration files
                  echo "Configuration Files:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, size_bytes
                     FROM files WHERE is_config = 1
                     ORDER BY file_type, relative_path;"
                  ;;
                dirs)
                  # List directories with file counts
                  echo "Directories:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT directory, COUNT(*) as files, SUM(line_count) as lines, SUM(size_bytes) as bytes
                     FROM files
                     GROUP BY directory
                     ORDER BY files DESC;"
                  ;;
                types)
                  # List files grouped by type
                  TYPE="$2"
                  if [ -n "$TYPE" ]; then
                    echo "Files of type: $TYPE"
                    sqlite3 -header -column "$DB_FILE" \
                      "SELECT relative_path, line_count, size_bytes
                       FROM files WHERE file_type = '$TYPE'
                       ORDER BY relative_path;"
                  else
                    echo "File types:"
                    sqlite3 -header -column "$DB_FILE" \
                      "SELECT file_type, COUNT(*) as count, SUM(line_count) as lines
                       FROM files GROUP BY file_type ORDER BY count DESC;"
                  fi
                  ;;
                hidden)
                  # List hidden files
                  echo "Hidden Files:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count
                     FROM files WHERE is_hidden = 1
                     ORDER BY relative_path;"
                  ;;
                envvars)
                  # Find environment variable definitions and usages
                  echo "Environment Variables:"
                  echo ""
                  echo "=== Definitions (export VAR=) ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%export %=%'
                        OR fl.content LIKE '%ENV %=%'
                        OR fl.content LIKE '%environment%=%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  echo ""
                  echo "=== Usages (\$VAR or \${VAR}) ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%\$%' AND fl.content NOT LIKE '%#%\$%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                secrets)
                  # Find potential secrets (API keys, passwords, tokens)
                  echo "Potential Secrets (REVIEW CAREFULLY):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 120)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE (fl.content LIKE '%password%' OR fl.content LIKE '%PASSWORD%'
                        OR fl.content LIKE '%secret%' OR fl.content LIKE '%SECRET%'
                        OR fl.content LIKE '%api_key%' OR fl.content LIKE '%API_KEY%'
                        OR fl.content LIKE '%apikey%' OR fl.content LIKE '%APIKEY%'
                        OR fl.content LIKE '%token%' OR fl.content LIKE '%TOKEN%'
                        OR fl.content LIKE '%private_key%' OR fl.content LIKE '%PRIVATE_KEY%'
                        OR fl.content LIKE '%credential%' OR fl.content LIKE '%CREDENTIAL%'
                        OR fl.content LIKE '%aws_access%' OR fl.content LIKE '%AWS_ACCESS%')
                       AND f.filename NOT LIKE '%.md'
                       AND f.filename NOT LIKE '%.txt'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                imports)
                  # Find import/require statements
                  LANG="''${2:-all}"
                  echo "Import statements ($LANG):"
                  echo ""
                  case "$LANG" in
                    py|python)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'py' AND (fl.content LIKE 'import %' OR fl.content LIKE 'from %import%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    nix)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'nix' AND (fl.content LIKE '%import %' OR fl.content LIKE '%inputs.%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    js|ts|javascript|typescript)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension IN ('js', 'ts', 'tsx', 'jsx')
                           AND (fl.content LIKE '%import %' OR fl.content LIKE '%require(%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    *)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE fl.content LIKE '%import %' OR fl.content LIKE '%require(%' OR fl.content LIKE '%include %'
                         ORDER BY f.relative_path, fl.line_number
                         LIMIT 200;"
                      ;;
                  esac
                  ;;
                deps)
                  # Find dependency definitions
                  echo "Dependencies:"
                  echo ""
                  echo "=== Package definitions ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE (f.filename IN ('package.json', 'Cargo.toml', 'go.mod', 'pyproject.toml', 'requirements.txt', 'Gemfile', 'pixi.toml', 'flake.nix')
                        OR f.filename LIKE 'requirements%.txt')
                       AND fl.trimmed_content != ''
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 200;"
                  ;;
                ports)
                  # Find port numbers in configuration
                  echo "Port Numbers:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%port%'
                        OR fl.content LIKE '%PORT%'
                        OR fl.content LIKE '%:80%'
                        OR fl.content LIKE '%:443%'
                        OR fl.content LIKE '%:8080%'
                        OR fl.content LIKE '%:3000%'
                        OR fl.content LIKE '%:5000%'
                        OR fl.content LIKE '%:9090%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                urls)
                  # Find URLs and endpoints
                  echo "URLs and Endpoints:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 150)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%http://%'
                        OR fl.content LIKE '%https://%'
                        OR fl.content LIKE '%localhost%'
                        OR fl.content LIKE '%127.0.0.1%'
                        OR fl.content LIKE '%0.0.0.0%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 150;"
                  ;;
                paths)
                  # Find path definitions
                  echo "Path Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%PATH%'
                        OR fl.content LIKE '%path =%'
                        OR fl.content LIKE '%path:%'
                        OR fl.content LIKE '%/usr/%'
                        OR fl.content LIKE '%/etc/%'
                        OR fl.content LIKE '%/home/%'
                        OR fl.content LIKE '%/var/%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                services)
                  # Find service definitions
                  echo "Service Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%service%'
                        OR fl.content LIKE '%Service%'
                        OR fl.content LIKE '%systemd%'
                        OR fl.content LIKE '%ExecStart%'
                        OR fl.content LIKE '%container%'
                        OR fl.content LIKE '%docker%'
                        OR fl.content LIKE '%podman%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                options)
                  # Find option/setting definitions (especially for Nix)
                  echo "Option/Setting Definitions:"
                  echo ""
                  echo "=== Nix Options ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE f.extension = 'nix'
                       AND (fl.content LIKE '%mkOption%' OR fl.content LIKE '%mkEnableOption%' OR fl.content LIKE '%options.%')
                     ORDER BY f.relative_path, fl.line_number;"
                  echo ""
                  echo "=== General Settings ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%enable%=%'
                        OR fl.content LIKE '%enabled%:%'
                        OR fl.content LIKE '%config.%=%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                modules)
                  # Find module definitions
                  echo "Module Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%module%'
                        OR fl.content LIKE '%Module%'
                        OR fl.content LIKE '%MODULE%'
                        OR fl.content LIKE '%{ config,%'
                        OR fl.content LIKE '%{ lib,%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                fts)
                  # Full-text search using FTS5 (more powerful)
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db fts <query>" >&2
                    echo ""
                    echo "FTS5 query syntax:"
                    echo "  'word1 word2'     - Both words (AND)"
                    echo "  'word1 OR word2'  - Either word"
                    echo "  '\"exact phrase\"'  - Exact phrase"
                    echo "  'word*'           - Prefix match"
                    echo "  'word1 NOT word2' - Exclude word2"
                    exit 1
                  fi
                  shift
                  QUERY="$*"
                  echo "FTS Search: $QUERY"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT path, filename, snippet(files_fts, 2, '>>>', '<<<', '...', 50) as match
                     FROM files_fts
                     WHERE files_fts MATCH '$QUERY'
                     ORDER BY rank
                     LIMIT 50;"
                  ;;
                context)
                  # Show context around a specific line
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db context <file-pattern> <line-number> [context-lines]" >&2
                    exit 1
                  fi
                  FILE_PATTERN="$2"
                  LINE_NUM="$3"
                  CTX="''${4:-5}"
                  START=$((LINE_NUM - CTX))
                  END=$((LINE_NUM + CTX))
                  [ $START -lt 1 ] && START=1
                  echo "Context for $FILE_PATTERN:$LINE_NUM (±$CTX lines):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT fl.line_number || ': ' || fl.content
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE f.relative_path LIKE '%$FILE_PATTERN%'
                       AND fl.line_number BETWEEN $START AND $END
                     ORDER BY fl.line_number;"
                  ;;
                diff)
                  # Compare two files
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db diff <file1-pattern> <file2-pattern>" >&2
                    exit 1
                  fi
                  FILE1="$2"
                  FILE2="$3"
                  echo "Comparing files matching: $FILE1 vs $FILE2"
                  echo ""
                  sqlite3 "$DB_FILE" "SELECT content FROM files WHERE relative_path LIKE '%$FILE1%' LIMIT 1;" > /tmp/cdb_diff1.txt
                  sqlite3 "$DB_FILE" "SELECT content FROM files WHERE relative_path LIKE '%$FILE2%' LIMIT 1;" > /tmp/cdb_diff2.txt
                  diff -u /tmp/cdb_diff1.txt /tmp/cdb_diff2.txt || true
                  rm -f /tmp/cdb_diff1.txt /tmp/cdb_diff2.txt
                  ;;
                duplicates)
                  # Find duplicate content
                  echo "Files with identical content:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT GROUP_CONCAT(relative_path, ', ') as files, line_count, size_bytes
                     FROM files
                     GROUP BY content
                     HAVING COUNT(*) > 1
                     ORDER BY size_bytes DESC
                     LIMIT 20;"
                  ;;
                large)
                  # Find largest files
                  LIMIT="''${2:-20}"
                  echo "Largest files (top $LIMIT):"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, size_bytes
                     FROM files
                     ORDER BY size_bytes DESC
                     LIMIT $LIMIT;"
                  ;;
                recent)
                  # Files loaded in current session (by ID, since we don't track mtime)
                  LIMIT="''${2:-20}"
                  echo "Recently loaded files (last $LIMIT):"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, loaded_at
                     FROM files
                     ORDER BY id DESC
                     LIMIT $LIMIT;"
                  ;;
                tree)
                  # Show directory tree structure
                  echo "Directory Tree:"
                  echo ""
                  sqlite3 "$DB_FILE" \
                    "SELECT DISTINCT directory FROM files ORDER BY directory;" | while read -r dir; do
                    depth=$(echo "$dir" | tr -cd '/' | wc -c)
                    indent=$(printf '%*s' $((depth * 2)) '')
                    name=$(basename "$dir")
                    count=$(sqlite3 "$DB_FILE" "SELECT COUNT(*) FROM files WHERE directory = '$dir';")
                    echo "''${indent}📁 $name/ ($count files)"
                  done
                  ;;

                stats)
                  # Show comprehensive database statistics
                  echo "═══════════════════════════════════════════════════════"
                  echo "           CODEBASE DATABASE STATISTICS"
                  echo "═══════════════════════════════════════════════════════"
                  echo ""
                  echo "Database: $DB_FILE"
                  echo ""
                  echo "── Overview ──────────────────────────────────────────"
                  sqlite3 "$DB_FILE" << 'STATS'
                    SELECT 'Total files:' as metric, COUNT(*) as value FROM files
                    UNION ALL SELECT 'Config files:', COUNT(*) FROM files WHERE is_config = 1
                    UNION ALL SELECT 'Hidden files:', COUNT(*) FROM files WHERE is_hidden = 1
                    UNION ALL SELECT 'Total lines:', SUM(line_count) FROM files
                    UNION ALL SELECT 'Total size (KB):', ROUND(SUM(size_bytes) / 1024.0, 2) FROM files
                    UNION ALL SELECT 'Directories:', COUNT(DISTINCT directory) FROM files;
STATS
                  echo ""
                  echo "── Files by Type ─────────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT file_type as type, COUNT(*) as files, SUM(line_count) as lines, ROUND(SUM(size_bytes)/1024.0, 1) as kb
                     FROM files GROUP BY file_type ORDER BY files DESC LIMIT 15;"
                  echo ""
                  echo "── Files by Extension ────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT extension as ext, COUNT(*) as files, SUM(line_count) as lines
                     FROM files WHERE extension != '' GROUP BY extension ORDER BY files DESC LIMIT 15;"
                  echo ""
                  echo "── Top Directories ───────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT directory as dir, COUNT(*) as files, SUM(line_count) as lines
                     FROM files GROUP BY directory ORDER BY files DESC LIMIT 10;"
                  echo ""
                  # Show metadata if available
                  if sqlite3 "$DB_FILE" "SELECT 1 FROM metadata LIMIT 1;" 2>/dev/null | grep -q 1; then
                    echo "── Load Metadata ─────────────────────────────────────"
                    sqlite3 -column "$DB_FILE" "SELECT key, value FROM metadata;"
                  fi
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
                  echo "Loads ALL files in the workspace into SQLite for comprehensive querying."
                  echo ""
                  echo "DATABASE COMMANDS:"
                  echo "  init              - Initialize empty database"
                  echo "  load              - Load ALL files into database"
                  echo "  reload            - Reinitialize and reload all files"
                  echo "  stats             - Show database statistics"
                  echo "  shell             - Open interactive SQLite shell"
                  echo ""
                  echo "SEARCH COMMANDS:"
                  echo "  search <pattern>  - Search in file contents (LIKE)"
                  echo "  fts <query>       - Full-text search (FTS5 with ranking)"
                  echo "  grep <pattern>    - Search with file:line output"
                  echo "  query '<SQL>'     - Execute arbitrary SQL query"
                  echo "  context <f> <ln>  - Show context around a line"
                  echo "  export <p> <f>    - Export search results to file"
                  echo ""
                  echo "FILE LISTING:"
                  echo "  files [ext]       - List files (optionally by extension)"
                  echo "  types [type]      - List files by type (nix, yaml, shell...)"
                  echo "  configs           - List all configuration files"
                  echo "  dirs              - List directories with file counts"
                  echo "  hidden            - List hidden files"
                  echo "  tree              - Show directory tree structure"
                  echo "  large [n]         - Show largest files"
                  echo "  recent [n]        - Show recently loaded files"
                  echo "  duplicates        - Find files with identical content"
                  echo ""
                  echo "CODE ANALYSIS:"
                  echo "  functions [lang]  - Find function definitions"
                  echo "  imports [lang]    - Find import/require statements"
                  echo "  modules           - Find module definitions"
                  echo "  todos             - Find TODO/FIXME comments"
                  echo "  issues            - Find error handling patterns"
                  echo "  diff <f1> <f2>    - Compare two files"
                  echo ""
                  echo "CONFIGURATION ANALYSIS:"
                  echo "  envvars           - Find environment variables"
                  echo "  secrets           - Find potential secrets (passwords, keys)"
                  echo "  deps              - Find dependency definitions"
                  echo "  ports             - Find port numbers"
                  echo "  urls              - Find URLs and endpoints"
                  echo "  paths             - Find path definitions"
                  echo "  services          - Find service definitions"
                  echo "  options           - Find option definitions (Nix mkOption)"
                  echo ""
                  echo "ENVIRONMENT:"
                  echo "  CODEBASE_DB   - Database file (default: ./codebase.db)"
                  echo "  CODEBASE_ROOT - Root directory to scan (default: .)"
                  echo ""
                  echo "EXAMPLES:"
                  echo "  codebase-db load                     # Load entire workspace"
                  echo "  codebase-db fts 'error AND handling' # FTS5 boolean search"
                  echo "  codebase-db configs                  # List all config files"
                  echo "  codebase-db types nix                # List all Nix files"
                  echo "  codebase-db envvars                  # Find env variables"
                  echo "  codebase-db secrets                  # Audit for secrets"
                  echo "  codebase-db context flake.nix 100 10 # Show lines 90-110"
                  echo "  codebase-db query 'SELECT * FROM files WHERE is_config=1'"
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

            # P2-017: AgenticsOrg DevOps source path
            AGENTICSORG_DEVOPS_SRC = toString agenticsorgDevopsSrc;

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

            # Layer 3 Isolation Configuration (ARIA P0-001, P0-002)
            # DEFAULT_ISOLATION: Primary isolation runtime for containers/workloads
            # Options: "firecracker" (microVM), "kata" (lightweight VM), "sandbox-runtime" (process-level)
            DEFAULT_ISOLATION = "firecracker";

            # TOOL_ISOLATION: Isolation for MCP/AI tool execution
            # Options: "sandbox-runtime" (recommended), "kata", "none"
            TOOL_ISOLATION = "sandbox-runtime";

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
              echo "Layer 3 Isolation (ARIA Audit):"
              echo "  DEFAULT_ISOLATION: $DEFAULT_ISOLATION"
              echo "  TOOL_ISOLATION:    $TOOL_ISOLATION"
              echo "  firecracker        - MicroVM isolation (ready)"
              echo "  kata               - Kata Containers (check: kata status)"
              echo "  sandbox-runtime    - Process sandbox (check: sandbox-runtime --version)"
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
              echo "Holochain (P2P) - P3-006 Reference Tools:"
              echo "  holochain       - Holochain conductor runtime"
              echo "  hc              - Holochain dev CLI"
              echo "    hc sandbox    - Generate and run test networks"
              echo "    hc scaffold   - Generate DNA/zome templates"
              echo "    hc dna        - DNA operations (init/pack/unpack)"
              echo "    hc app        - hApp bundle operations"
              echo "  lair-keystore   - Secure cryptographic keystore"
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
