#!/usr/bin/env bash
# Devcontainer setup script for ripple-env
# This script runs as postCreateCommand to set up the development environment
set -euo pipefail

echo "==> Setting up ripple-env devcontainer..."

# Ensure TMPDIR exists and is writable
export TMPDIR="${TMPDIR:-/tmp}"
sudo mkdir -p "$TMPDIR" && sudo chmod 1777 "$TMPDIR"

# =============================================================================
# 1. Configure Nix
# =============================================================================
echo "==> Configuring Nix..."

# Source Nix profile if available
NIX_PROFILE="/nix/var/nix/profiles/default/etc/profile.d/nix.sh"
if [ -f "$NIX_PROFILE" ]; then
    # shellcheck source=/dev/null
    . "$NIX_PROFILE"
fi

# Verify Nix is available
if command -v nix >/dev/null 2>&1; then
    echo "    Nix version: $(nix --version)"
else
    echo "    ERROR: Nix not found in PATH"
    exit 1
fi

# Install essential Nix tools
echo "==> Installing Nix development tools..."
nix profile install \
    nixpkgs#direnv \
    nixpkgs#nix-direnv \
    nixpkgs#nil \
    nixpkgs#nixfmt-rfc-style \
    nixpkgs#nix-output-monitor \
    nixpkgs#nix-tree \
    2>/dev/null || echo "    Some Nix tools may already be installed"

# =============================================================================
# 2. Install Pixi
# =============================================================================
echo "==> Installing Pixi package manager..."

export PIXI_HOME="${PIXI_HOME:-$HOME/.pixi}"
mkdir -p "$PIXI_HOME"

if ! command -v pixi >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | bash
    export PATH="$HOME/.pixi/bin:$PATH"
fi

if command -v pixi >/dev/null 2>&1; then
    echo "    Pixi version: $(pixi --version)"
else
    echo "    WARNING: Pixi installation may have failed"
fi

# =============================================================================
# 3. Configure Shell Integration
# =============================================================================
echo "==> Configuring shell integration..."

# Configure .bashrc
BASHRC="$HOME/.bashrc"
if [ -f "$BASHRC" ]; then
    # Add TMPDIR export
    grep -qxF 'export TMPDIR=/tmp' "$BASHRC" || echo 'export TMPDIR=/tmp' >> "$BASHRC"

    # Add Nix profile
    grep -qF 'nix.sh' "$BASHRC" || echo '[ -f /nix/var/nix/profiles/default/etc/profile.d/nix.sh ] && . /nix/var/nix/profiles/default/etc/profile.d/nix.sh' >> "$BASHRC"

    # Add Pixi to PATH
    grep -qF '.pixi/bin' "$BASHRC" || echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> "$BASHRC"

    # Add direnv hook
    grep -qF 'direnv hook bash' "$BASHRC" || echo 'eval "$(direnv hook bash)"' >> "$BASHRC"
fi

# Configure .zshrc
ZSHRC="$HOME/.zshrc"
if [ -f "$ZSHRC" ]; then
    # Add TMPDIR export
    grep -qxF 'export TMPDIR=/tmp' "$ZSHRC" || echo 'export TMPDIR=/tmp' >> "$ZSHRC"

    # Add Nix profile
    grep -qF 'nix.sh' "$ZSHRC" || echo '[ -f /nix/var/nix/profiles/default/etc/profile.d/nix.sh ] && . /nix/var/nix/profiles/default/etc/profile.d/nix.sh' >> "$ZSHRC"

    # Add Pixi to PATH
    grep -qF '.pixi/bin' "$ZSHRC" || echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> "$ZSHRC"

    # Add direnv hook
    grep -qF 'direnv hook zsh' "$ZSHRC" || echo 'eval "$(direnv hook zsh)"' >> "$ZSHRC"
fi

# Configure Nushell (if installed)
NU_CONFIG_DIR="$HOME/.config/nushell"
if command -v nu >/dev/null 2>&1; then
    echo "==> Configuring Nushell..."
    mkdir -p "$NU_CONFIG_DIR"

    # Create env.nu if it doesn't exist
    NU_ENV="$NU_CONFIG_DIR/env.nu"
    if [ ! -f "$NU_ENV" ]; then
        cat > "$NU_ENV" << 'EOF'
# Nushell Environment Configuration for ripple-env

# Set TMPDIR
$env.TMPDIR = "/tmp"

# Add Pixi to PATH
$env.PATH = ($env.PATH | prepend $"($env.HOME)/.pixi/bin")

# Add Nix profile to PATH
$env.PATH = ($env.PATH | prepend $"($env.HOME)/.nix-profile/bin")

# Source Nix environment (workaround for Nushell)
if ($"($env.HOME)/.nix-profile/etc/profile.d/nix.sh" | path exists) {
    # Nix vars are set by the devcontainer environment
}
EOF
    fi

    # Create config.nu if it doesn't exist
    NU_CONFIG="$NU_CONFIG_DIR/config.nu"
    if [ ! -f "$NU_CONFIG" ]; then
        cat > "$NU_CONFIG" << 'EOF'
# Nushell Configuration for ripple-env

$env.config = {
    show_banner: false

    # History configuration
    history: {
        max_size: 10000
        sync_on_enter: true
        file_format: "sqlite"
    }

    # Completions
    completions: {
        case_sensitive: false
        quick: true
        partial: true
        algorithm: "fuzzy"
    }

    # Table display
    table: {
        mode: rounded
        index_mode: always
    }
}

# Aliases
alias ll = ls -l
alias la = ls -la
alias ndev = nix develop
alias pxi = pixi

# Quick directory navigation
def --env z [dir: string] {
    cd $dir
}
EOF
    fi

    echo "    Nushell version: $(nu --version)"
fi

# =============================================================================
# 4. Allow direnv in workspace
# =============================================================================
echo "==> Allowing direnv in workspace..."
if [ -f ".envrc" ] && command -v direnv >/dev/null 2>&1; then
    direnv allow . 2>/dev/null || echo "    Note: Run 'direnv allow' manually if needed"
fi

# =============================================================================
# 5. Initialize Pixi environment (optional, can be slow)
# =============================================================================
if [ -f "pixi.toml" ] && command -v pixi >/dev/null 2>&1; then
    echo "==> Pixi project detected. Run 'pixi install' to set up the environment."
    echo "    (Skipping auto-install to save time during container creation)"
fi

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "=========================================="
echo "  Devcontainer setup complete!"
echo "=========================================="
echo ""
echo "Available tools:"
command -v nix >/dev/null 2>&1 && echo "  - Nix:     $(nix --version)"
command -v pixi >/dev/null 2>&1 && echo "  - Pixi:    $(pixi --version)"
command -v direnv >/dev/null 2>&1 && echo "  - Direnv:  $(direnv --version)"
command -v nu >/dev/null 2>&1 && echo "  - Nushell: $(nu --version)"
echo ""
echo "Available shells:"
echo "  - bash (default)"
echo "  - zsh"
command -v nu >/dev/null 2>&1 && echo "  - nushell (nu)"
echo ""
echo "Quick start:"
echo "  1. Run 'nix develop' to enter the Nix shell"
echo "  2. Or run 'pixi install && pixi shell' for Pixi environment"
echo "  3. Or just start coding - direnv will auto-load the environment"
echo ""
