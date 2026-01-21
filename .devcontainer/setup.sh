#!/usr/bin/env bash
# Devcontainer setup script for ripple-env (NixOS-based)
# This script runs as postCreateCommand to set up the workspace environment
#
# All tools are pre-installed via Nix in the Dockerfile.
# This script handles workspace-specific configuration.
#
# Tool Management:
# - Nix: Provides Rust toolchain (cargo, rustc), system tools, and dev utilities
# - Pixi: Manages Node.js (>=22.0), Python, pnpm, and ROS2 dependencies
# - Direnv: Auto-loads the Nix development environment
# - Home-manager: User configuration management (optional)
set -euo pipefail

echo "==> Setting up ripple-env workspace..."

# =============================================================================
# 1. Verify Nix Environment
# =============================================================================
echo "==> Verifying Nix environment..."

# Check core tools are available
declare -a tools=(
    "nix"
    "git"
    "gh"
    "direnv"
    "pixi"
    "nu"
    "cargo"
    "rustc"
    "node"
    "pnpm"
    "grpcurl"
    "protoc"
    "wasm-pack"
    "kubectl"
    "home-manager"
    "docker"
)

for cmd in "${tools[@]}"; do
    if command -v "$cmd" >/dev/null 2>&1; then
        echo "    ✓ $cmd"
    else
        echo "    ✗ $cmd (missing or optional)"
    fi
done

# =============================================================================
# 2. Install Pixi if not present (fallback)
# =============================================================================
if ! command -v pixi >/dev/null 2>&1; then
    echo "==> Installing Pixi..."
    curl -fsSL https://pixi.sh/install.sh | bash
    export PATH="$HOME/.pixi/bin:$PATH"
fi

# =============================================================================
# 3. Configure Direnv for workspace
# =============================================================================
echo "==> Configuring direnv..."

# Ensure direnv config exists
mkdir -p "$HOME/.config/direnv"
if [ ! -f "$HOME/.config/direnv/direnv.toml" ]; then
    cat > "$HOME/.config/direnv/direnv.toml" << 'EOF'
[whitelist]
prefix = ["/workspaces"]
EOF
fi

# Allow direnv in workspace
if [ -f ".envrc" ]; then
    direnv allow . 2>/dev/null || echo "    Note: Run 'direnv allow' manually if needed"
fi

# =============================================================================
# 4. Initialize Nix flake (if available)
# =============================================================================
if [ -f "flake.nix" ]; then
    echo "==> Nix flake detected"
    echo "    Run 'nix develop' to enter the development shell"
    echo "    Or wait for direnv to auto-load the environment"
fi

# =============================================================================
# 5. Initialize Pixi environment (if available)
# =============================================================================
if [ -f "pixi.toml" ]; then
    echo "==> Pixi project detected"
    echo "    Run 'pixi install' to set up Python/Node.js/ROS2 environment"
    echo "    Node.js and pnpm are managed by pixi for cross-platform consistency"
    # Optionally auto-install (uncomment if desired):
    # pixi install
fi

# =============================================================================
# 6. Home-manager configuration
# =============================================================================
echo "==> Checking Home-manager configuration..."

mkdir -p "$HOME/.config/home-manager"
mkdir -p "$HOME/.config/ripple-env"
mkdir -p "$HOME/.cache/ripple-env"

if [ -f "home.nix" ] && [ ! -f "$HOME/.config/home-manager/home.nix" ]; then
    echo "    Copying home.nix template to $HOME/.config/home-manager/"
    cp home.nix "$HOME/.config/home-manager/home.nix" 2>/dev/null || true
fi

if command -v home-manager >/dev/null 2>&1; then
    echo "    ✓ Home-manager available"
    echo "    Run 'home-manager switch' to apply user configuration"
else
    echo "    ⚠ Home-manager not found (optional)"
fi

# =============================================================================
# 7. Git configuration
# =============================================================================
echo "==> Configuring Git..."

# Set up git LFS
git lfs install --skip-repo 2>/dev/null || true

# Trust the workspace directory
git config --global --add safe.directory /workspaces 2>/dev/null || true
git config --global --add safe.directory "$(pwd)" 2>/dev/null || true

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "=========================================="
echo "  NixOS Devcontainer Ready!"
echo "=========================================="
echo ""
echo "Available tools (via Nix):"
echo "  - nix $(nix --version 2>/dev/null | head -1 || echo 'installed')"
echo "  - git $(git --version 2>/dev/null | cut -d' ' -f3 || echo 'installed')"
echo "  - gh $(gh --version 2>/dev/null | head -1 | cut -d' ' -f3 || echo 'installed')"
echo "  - direnv $(direnv --version 2>/dev/null || echo 'installed')"
command -v pixi >/dev/null 2>&1 && echo "  - pixi $(pixi --version 2>/dev/null || echo 'installed')"
echo "  - nushell $(nu --version 2>/dev/null || echo 'installed')"
echo "  - rust $(rustc --version 2>/dev/null || echo 'installed')"
echo "  - cargo $(cargo --version 2>/dev/null || echo 'installed')"
echo "  - node $(node --version 2>/dev/null || echo 'installed')"
command -v npm >/dev/null 2>&1 && echo "  - npm $(npm --version 2>/dev/null || echo 'installed')"
echo "  - pnpm $(pnpm --version 2>/dev/null || echo 'installed')"
command -v home-manager >/dev/null 2>&1 && echo "  - home-manager $(home-manager --version 2>/dev/null || echo 'installed')"
echo ""
echo "Tool Management:"
echo "  - Nix:     Rust toolchain, system tools, dev utilities"
echo "  - Pixi:    Node.js (>=22.0), Python, pnpm, ROS2 dependencies"
echo "  - Direnv:  Auto-loads Nix environment from .envrc"
echo "  - Home-manager: User configuration (home.nix)"
echo ""
echo "Additional tooling:"
echo "  - Distributed systems: grpcurl, protoc, wasm-pack, websocat"
echo "  - Hardware: v4l-utils, can-utils, socat, minicom"
echo "  - Simulation: ogre, mesa, bullet, ffmpeg"
echo "  - Security: trivy, syft, grype, vault, cosign"
echo "  - Infrastructure: prometheus, nats-server, kubo"
echo "  - AI/ML: aichat, aider-chat, local-ai (optional)"
echo ""
echo "Available shells:"
echo "  - zsh (default)"
echo "  - bash"
echo "  - nushell (nu)"
echo ""
echo "Quick start:"
echo "  1. direnv will auto-load the Nix environment"
echo "  2. Run 'pixi install' for Python/Node.js/ROS2 packages"
echo "  3. Or run 'nix develop' for the full Nix shell"
echo "  4. Run 'home-manager switch' to apply user configuration"
echo ""
