# Common shell configuration shared across all shells
{ pkgs, lib, colconDefaults, isDarwin, system, ... }:

{
  # Common environment variables
  env = {
    COLCON_DEFAULTS_FILE = toString colconDefaults;
    EDITOR = "hx";
    VISUAL = "hx";
  };

  # Base shell hook shared by all shells
  baseShellHook = ''
    # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
    export TMPDIR=''${TMPDIR:-/tmp}
    [ -d "$TMPDIR" ] || export TMPDIR=/tmp
    mkdir -p "$TMPDIR" 2>/dev/null || true

    # Define stub functions for RoboStack activation scripts
    noa_add_path() { :; }
    export -f noa_add_path 2>/dev/null || true

    # Initialize pixi environment
    if [ -f pixi.toml ]; then
      ${lib.optionalString isDarwin ''
        export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
      ''}
      eval "$(pixi shell-hook 2>/dev/null)" || true
    fi
  '';

  # Default shell hook (minimal output)
  defaultShellHook = ''
    # Keep startup fast for non-interactive shells (CI, `nix develop --command ...`).
    if [[ $- == *i* ]]; then
      echo ""
      echo "ROS2 Humble Development Environment"
      echo "=================================="
      echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
      echo ""
    fi
  '';

  # Full shell hook (verbose output with all help)
  fullShellHook = pythonVersion: ''
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
    echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
    echo "  Python (Nix): ${pythonVersion} (for scripts/tools)"
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

  # CUDA shell hook
  cudaShellHook = cudaVersion: gccPath: pythonVersion: ''
    # Set up LD_LIBRARY_PATH for CUDA libraries
    export LD_LIBRARY_PATH="$CUDA_PATH/lib:$CUDNN_PATH/lib:$LD_LIBRARY_PATH"

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
      echo "  CUDA: ${cudaVersion}"
      echo "  GCC: $(${gccPath}/bin/gcc --version | head -1)"
      echo "  Python (Nix): ${pythonVersion}"
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
      echo "Warning: nvidia-smi not found"
      echo "   CUDA toolkit is available but GPU drivers may not be installed."
      echo "   Install NVIDIA drivers on your host system."
      echo ""
    fi
  '';

  # Identity shell hook
  identityShellHook = javaHome: postgresVersion: ''
    echo ""
    echo "Identity & Auth Development Environment"
    echo "========================================"
    echo "  Platform: Linux (${system})"
    echo "  Java: $(java -version 2>&1 | head -1)"
    echo "  PostgreSQL: ${postgresVersion}"
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
}
