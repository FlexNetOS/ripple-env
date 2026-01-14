# AI and tool execution command wrappers
# Includes: LocalAI, AGiXT, AIOS, sandbox-runtime, MCP tools
# Extracted from flake.nix for modular organization
{ pkgs, ... }:

[
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

              # Compose file resolution (supports both canonical + legacy layouts)
              if [ -f "$AGIXT_DIR/docker/agixt.yml" ]; then
                COMPOSE_FILE="$AGIXT_DIR/docker/agixt.yml"
              elif [ -f "$AGIXT_DIR/agixt.yml" ]; then
                COMPOSE_FILE="$AGIXT_DIR/agixt.yml"
              elif [ -f "$AGIXT_DIR/docker-compose.agixt.yml" ]; then
                COMPOSE_FILE="$AGIXT_DIR/docker-compose.agixt.yml"
              elif [ -f "$AGIXT_DIR/docker/docker-compose.agixt.yml" ]; then
                COMPOSE_FILE="$AGIXT_DIR/docker/docker-compose.agixt.yml"
              else
                echo "Error: AGiXT compose file not found under $AGIXT_DIR"
                echo "Tried:"
                echo "  - $AGIXT_DIR/docker/agixt.yml"
                echo "  - $AGIXT_DIR/agixt.yml"
                echo "  - $AGIXT_DIR/docker-compose.agixt.yml"
                echo "  - $AGIXT_DIR/docker/docker-compose.agixt.yml"
                echo "Set AGIXT_DIR to the ripple-env repo root (recommended) or the docker/ directory."
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
          ]
