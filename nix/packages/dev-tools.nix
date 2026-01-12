# Full development tools (heavier packages for devShells.full)
# These are optional extras beyond base packages
#
# Feature flags allow excluding heavy dependencies:
#   packages.devTools                    # All tools (default)
#   packages.devToolsMinimal             # Without heavy VMs/k8s
#
# Usage with flags:
#   import ./dev-tools.nix { inherit pkgs lib; withKubernetes = false; }
{ pkgs, lib ? pkgs.lib, withKubernetes ? true, withHeavyVMs ? true, withAI ? true, ... }:

let
  # Base development tools (always included)
  baseTools = with pkgs; [
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
    prometheus          # Metrics collection for ROS2 DDS
    natscli             # NATS CLI for messaging
    nats-server         # WAN/multi-site robot messaging
    trippy              # Network diagnostics for DDS traffic
    trivy               # Container/SBOM security scanning
    opa                 # Policy enforcement for ROS2 topics
    syft                # SBOM generation
    grype               # Vulnerability scanning from SBOM
    cosign              # Container image signing and verification

    # Additional tools from BUILDKIT_STARTER_SPEC
    neovim              # Editor
    sqlite              # Local database

    # Secrets Management
    vault               # HashiCorp Vault for secrets management

    # P2P & Content-Addressed Storage
    kubo                # IPFS implementation (content-addressed storage)

    # PKI Automation
    step-cli            # smallstep CLI for mTLS/PKI

    # Container runtime (lightweight)
    runc                # OCI container runtime (low-level)

    # Shells
    zsh
    nushell

    # Editor
    helix

    # Prompt
    starship

    # Audio (for aider voice features)
    portaudio

    # Build tools & compilation cache
    ccache              # Fast C/C++ compilation cache
    sccache             # Distributed compilation cache with cloud support
    mold                # Fast modern linker (12x faster than lld)
    maturin             # Build tool for PyO3 Rust-Python bindings

    # Rust toolchain (for AGiXT Rust SDK and ROS2 bridges)
    cargo               # Rust package manager
    rustc               # Rust compiler
    rust-analyzer       # Rust LSP for editors
    rustfmt             # Rust code formatter
    clippy              # Rust linter

    # Database tools
    sqlx-cli            # SQL database CLI for migrations and schema management

    # Tree-sitter (for LazyVim/Neovim)
    tree-sitter

    # Node.js ecosystem (for LazyVim plugins & LLM testing)
    nodejs_22           # LTS "Jod" - active until Apr 2027
    nodePackages.pnpm

    # Git tools
    lazygit             # Git TUI (integrates with LazyVim)

    # Remote development
    devpod              # Client-only devcontainer environments (any backend)
  ];

  # Kubernetes tools (~100MB+ combined)
  kubernetesTools = with pkgs; [
    kubectl             # Kubernetes CLI for cluster management (~50MB)
    helm                # Kubernetes package manager (~50MB)
    kustomize           # Kubernetes configuration management
    containerd          # Container runtime
  ];

  # Heavy VM/Isolation tools (~150MB+ combined)
  heavyVMTools = with pkgs; [
    firecracker         # MicroVM for untrusted workloads (~50MB)
    kata-runtime        # Kata Containers runtime (~100MB)
  ];

  # AI/ML tools (~500MB+ combined)
  aiTools = with pkgs; [
    aichat              # AI chat assistant
    aider-chat          # AI pair programming

    # AI inference (edge/local models) - LocalAI
    # OpenAI-compatible API server for local LLM inference
    # Supports: GGUF, GGML, Safetensors models
    local-ai            # (~500MB+)
  ];

in
# Combine based on feature flags
baseTools
++ lib.optionals withKubernetes kubernetesTools
++ lib.optionals withHeavyVMs heavyVMTools
++ lib.optionals withAI aiTools
