#!/usr/bin/env bash
# Devcontainer setup script for ripple-env (NixOS-based)
# This script runs as postCreateCommand to set up the workspace environment
#
# All tools are pre-installed via Nix in the Dockerfile.
# This script handles workspace-specific configuration.
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
)

for cmd in "${tools[@]}"; do
    if command -v "$cmd" >/dev/null 2>&1; then
        echo "    ✓ $cmd"
    else
        echo "    ✗ $cmd (missing)"
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
    echo "    Run 'pixi install' to set up Python/ROS2 environment"
    # Optionally auto-install (uncomment if desired):
    # pixi install
fi

# =============================================================================
# 6. Git configuration
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
echo "  2. Run 'pixi install' for Python/ROS2 packages"
echo "  3. Or run 'nix develop' for the full Nix shell"
echo ""
