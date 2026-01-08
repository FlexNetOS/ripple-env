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

    # Source nix profile
    if [ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    elif [ -f "$HOME/.nix-profile/etc/profile.d/nix.sh" ]; then
        . "$HOME/.nix-profile/etc/profile.d/nix.sh"
    fi

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

    # Verify flake
    log_info "Checking flake..."
    nix flake check --no-build 2>/dev/null || nix flake show

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
    echo "  nom develop"
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
    if [ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    elif [ -f "$HOME/.nix-profile/etc/profile.d/nix.sh" ]; then
        . "$HOME/.nix-profile/etc/profile.d/nix.sh"
    fi

    # Install tools via nix
    install_direnv
    install_nom
    install_git
    install_gh
    install_zsh
    install_nushell

    # Setup shell integrations
    setup_direnv_hooks

    # Verify environment
    verify_environment
    verify_pixi

    print_summary
}

main "$@"
