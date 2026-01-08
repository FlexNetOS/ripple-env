#!/usr/bin/env bash
#
# bootstrap.sh - Complete end-to-end installation for ros2-humble-env
#
# This script installs all dependencies and sets up the development environment:
# - Nix with flakes enabled (using experimental installer)
# - direnv for automatic environment activation
# - nom for better nix output
# - pixi for package management
# - zsh and nushell shells
# - git and gh cli
#
# Usage: ./bootstrap.sh [--ci] [--skip-shells]
#   --ci          : Run in CI mode (non-interactive, skip optional components)
#   --skip-shells : Skip installing zsh and nushell
#

set -euo pipefail

# Some environments (including certain dotfiles) set TMPDIR to a path that may
# not exist. The Determinate Systems Nix installer uses mktemp and will fail if
# TMPDIR points to a missing directory.
: "${TMPDIR:=/tmp}"
if [ ! -d "$TMPDIR" ]; then
    TMPDIR=/tmp
fi
export TMPDIR

# Keep TMP/TEMP consistent with TMPDIR for downstream tools.
export TMP="${TMPDIR}"
export TEMP="${TMPDIR}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
CI_MODE=false
SKIP_SHELLS=false
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Parse arguments
for arg in "$@"; do
    case $arg in
        --ci)
            CI_MODE=true
            shift
            ;;
        --skip-shells)
            SKIP_SHELLS=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--ci] [--skip-shells]"
            echo "  --ci          : Run in CI mode (non-interactive)"
            echo "  --skip-shells : Skip installing zsh and nushell"
            exit 0
            ;;
    esac
done

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_command() {
    command -v "$1" &> /dev/null
}

source_nix_profile() {
    # nix-daemon.sh guards itself with __ETC_PROFILE_NIX_SOURCED. In some
    # environments that variable may already be set, which can prevent PATH
    # updates and make `nix` appear "not found".
    unset __ETC_PROFILE_NIX_SOURCED 2>/dev/null || true

    if [ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    elif [ -f "$HOME/.nix-profile/etc/profile.d/nix.sh" ]; then
        . "$HOME/.nix-profile/etc/profile.d/nix.sh"
    fi

    # As a last resort, ensure the default profile bin path is available.
    if ! check_command nix && [ -x /nix/var/nix/profiles/default/bin/nix ]; then
        export PATH="/nix/var/nix/profiles/default/bin:$PATH"
    fi
}

ensure_nix_daemon() {
    # If Nix was installed in multi-user mode but the daemon isn't running (e.g.
    # inside containers without systemd), nix commands will fail with permission
    # errors or missing daemon socket.
    if [ -S /nix/var/nix/daemon-socket/socket ]; then
        return 0
    fi

    log_warn "Nix daemon socket not found; attempting to start nix-daemon"

    # Try systemd first (if present).
    if check_command systemctl; then
        sudo systemctl start nix-daemon 2>/dev/null || true
    fi

    if [ -S /nix/var/nix/daemon-socket/socket ]; then
        return 0
    fi

    # Fall back to starting the daemon directly.
    sudo mkdir -p /nix/var/nix/daemon-socket
    sudo chmod 755 /nix/var/nix/daemon-socket
    (sudo /nix/var/nix/profiles/default/bin/nix-daemon --daemon >/tmp/nix-daemon.log 2>&1 &) || true
    sleep 1

    if [ -S /nix/var/nix/daemon-socket/socket ]; then
        log_success "Nix daemon is running"
        return 0
    fi

    log_warn "Failed to start nix-daemon automatically. If nix commands fail, check /tmp/nix-daemon.log"
}

# Detect OS and architecture
detect_system() {
    log_info "Detecting system..."

    OS="$(uname -s)"
    ARCH="$(uname -m)"

    case "$OS" in
        Linux)
            if [ -f /etc/os-release ]; then
                . /etc/os-release
                DISTRO="$ID"
            else
                DISTRO="unknown"
            fi
            ;;
        Darwin)
            DISTRO="macos"
            ;;
        *)
            log_error "Unsupported operating system: $OS"
            exit 1
            ;;
    esac

    log_info "System: $OS ($DISTRO) - $ARCH"
}

# Install Nix using the experimental installer
install_nix() {
    if check_command nix; then
        log_info "Nix is already installed"
        nix --version
        return 0
    fi

    log_info "Installing Nix with experimental installer..."

    if [ "$CI_MODE" = true ]; then
        # CI mode: use non-interactive installation
        curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | \
            sh -s -- install --no-confirm
    else
        curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | \
            sh -s -- install
    fi

    source_nix_profile
    ensure_nix_daemon

    log_success "Nix installed successfully"
}

# Enable flakes if not already enabled
enable_flakes() {
    log_info "Checking flakes configuration..."

    local nix_conf_dir="$HOME/.config/nix"
    local nix_conf="$nix_conf_dir/nix.conf"

    mkdir -p "$nix_conf_dir"

    if [ -f "$nix_conf" ]; then
        if grep -q "experimental-features.*flakes" "$nix_conf"; then
            log_info "Flakes already enabled"
            return 0
        fi
    fi

    # Add flakes configuration
    echo "experimental-features = nix-command flakes" >> "$nix_conf"
    log_success "Flakes enabled in $nix_conf"
}

# Install direnv
install_direnv() {
    if check_command direnv; then
        log_info "direnv is already installed"
        direnv --version
        return 0
    fi

    log_info "Installing direnv via nix..."
    nix profile install nixpkgs#direnv

    log_success "direnv installed successfully"
}

# Install nom (nix output monitor)
install_nom() {
    if check_command nom; then
        log_info "nom is already installed"
        nom --version
        return 0
    fi

    log_info "Installing nom via nix..."
    nix profile install nixpkgs#nix-output-monitor

    log_success "nom installed successfully"
}

# Install git if not present
install_git() {
    if check_command git; then
        log_info "git is already installed"
        git --version
        return 0
    fi

    log_info "Installing git via nix..."
    nix profile install nixpkgs#git

    log_success "git installed successfully"
}

# Install GitHub CLI
install_gh() {
    if check_command gh; then
        log_info "gh cli is already installed"
        gh --version
        return 0
    fi

    log_info "Installing gh cli via nix..."
    nix profile install nixpkgs#gh

    log_success "gh cli installed successfully"
}

# Install zsh
install_zsh() {
    if [ "$SKIP_SHELLS" = true ]; then
        log_info "Skipping zsh installation (--skip-shells)"
        return 0
    fi

    if check_command zsh; then
        log_info "zsh is already installed"
        zsh --version
        return 0
    fi

    log_info "Installing zsh via nix..."
    nix profile install nixpkgs#zsh

    log_success "zsh installed successfully"
}

# Install nushell
install_nushell() {
    if [ "$SKIP_SHELLS" = true ]; then
        log_info "Skipping nushell installation (--skip-shells)"
        return 0
    fi

    if check_command nu; then
        log_info "nushell is already installed"
        nu --version
        return 0
    fi

    log_info "Installing nushell via nix..."
    nix profile install nixpkgs#nushell

    log_success "nushell installed successfully"
}

# Setup direnv hook for shells
setup_direnv_hooks() {
    log_info "Setting up direnv hooks..."

    # Bash hook
    local bashrc="$HOME/.bashrc"
    if [ -f "$bashrc" ]; then
        if ! grep -q 'eval "$(direnv hook bash)"' "$bashrc"; then
            echo 'eval "$(direnv hook bash)"' >> "$bashrc"
            log_info "Added direnv hook to .bashrc"
        fi
    fi

    # Zsh hook (if zsh config exists)
    local zshrc="$HOME/.zshrc"
    if [ -f "$zshrc" ]; then
        if ! grep -q 'eval "$(direnv hook zsh)"' "$zshrc"; then
            echo 'eval "$(direnv hook zsh)"' >> "$zshrc"
            log_info "Added direnv hook to .zshrc"
        fi
    fi

    log_success "direnv hooks configured"
}

# Verify the flake and development environment
verify_environment() {
    log_info "Verifying development environment..."

    cd "$SCRIPT_DIR"

    # Check if flake.nix exists
    if [ ! -f "flake.nix" ]; then
        log_error "flake.nix not found in $SCRIPT_DIR"
        exit 1
    fi

    # Verify flake (fail hard if it doesn't evaluate)
    # NOTE: `--all-systems` can be significantly slower. Keep it for CI, but
    # default to the current system for interactive/local usage.
    log_info "Checking flake..."
    local flake_check_args=(--no-build)
    if [ "$CI_MODE" = true ]; then
        flake_check_args+=(--all-systems)
    fi

    if ! nix flake check "${flake_check_args[@]}"; then
        log_error "nix flake check failed"
        log_info "flake output (for debugging):"
        nix flake show || true
        exit 1
    fi

    # Build the devshell (this validates the configuration)
    log_info "Building development shell..."
    if check_command nom; then
        nom develop --command echo "Development shell verified successfully"
    else
        nix develop --command echo "Development shell verified successfully"
    fi

    log_success "Environment verification complete"
}

# Verify pixi is available in the environment
verify_pixi() {
    log_info "Verifying pixi setup..."

    cd "$SCRIPT_DIR"

    # Check if pixi.toml exists
    if [ ! -f "pixi.toml" ]; then
        log_error "pixi.toml not found in $SCRIPT_DIR"
        exit 1
    fi

    # Run pixi install within the nix environment
    log_info "Running pixi install..."
    nix develop --command pixi install

    log_success "pixi setup verified"
}

# Print summary
print_summary() {
    echo ""
    echo "========================================"
    echo -e "${GREEN}Bootstrap Complete!${NC}"
    echo "========================================"
    echo ""
    echo "Installed components:"
    echo "  - Nix (with flakes enabled)"
    echo "  - direnv"
    echo "  - nom (nix-output-monitor)"
    echo "  - git"
    echo "  - gh (GitHub CLI)"
    if [ "$SKIP_SHELLS" = false ]; then
        echo "  - zsh"
        echo "  - nushell"
    fi
    echo ""
    echo "To enter the development environment:"
    echo "  cd $SCRIPT_DIR"
    echo "  direnv allow"
    echo ""
    echo "Or manually:"
    echo "  nix develop"
    echo "  # (optional) nom develop"
    echo ""
}

# Main execution
main() {
    echo "========================================"
    echo "ros2-humble-env Bootstrap Script"
    echo "========================================"
    echo ""

    detect_system

    # Install core dependencies
    install_nix
    enable_flakes

    # Source nix again to ensure it's available
    source_nix_profile
    ensure_nix_daemon

    # Install tools via nix
    install_direnv
    install_nom
    install_git
    install_gh
    install_zsh
    install_nushell

    # Setup shell integrations
    if [ "$CI_MODE" = true ]; then
        log_info "CI mode: skipping shell rc modifications"
    else
        setup_direnv_hooks
    fi

    # Verify environment
    verify_environment
    verify_pixi

    print_summary
}

main "$@"
