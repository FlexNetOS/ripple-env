#!/usr/bin/env bash
set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

print_success() {
    echo -e "${GREEN}SUCCESS: $1${NC}"
}

print_info() {
    echo -e "${YELLOW}INFO: $1${NC}"
}

# Check if Nix is installed
check_nix() {
    if ! command -v nix &> /dev/null; then
        print_error "Nix is not installed or not in PATH"
        echo ""
        echo "To install Nix, run one of the following:"
        echo ""
        echo "1. Experimental installer (recommended):"
        echo "   curl --proto '=https' --tlsv1.2 -sSf -L https://artifacts.nixos.org/experimental-installer | sh -s -- install"
        echo ""
        echo "2. Official installer:"
        echo "   sh <(curl -L https://nixos.org/nix/install) --daemon"
        echo ""
        return 1
    fi
    
    # Check if flakes are enabled
    if ! nix eval --expr '1 + 1' &> /dev/null; then
        print_error "Nix is installed but flakes are not enabled"
        echo ""
        echo "To enable flakes, add the following to your Nix configuration:"
        echo "  experimental-features = nix-command flakes"
        echo ""
        echo "Location depends on your system:"
        echo "  - NixOS: /etc/nixos/configuration.nix"
        echo "  - macOS/Linux: ~/.config/nix/nix.conf or /etc/nix/nix.conf"
        echo ""
        return 1
    fi
    
    print_success "Nix is installed and flakes are enabled"
    return 0
}

# Check if Pixi will be available in the dev shell
check_pixi_in_shell() {
    print_info "Pixi will be provided by the Nix dev shell"
    return 0
}

# Check and recommend direnv
check_direnv() {
    if command -v direnv &> /dev/null; then
        print_success "direnv is installed"
        return 0
    else
        print_info "direnv is not installed (recommended for automatic environment activation)"
        echo ""
        echo "To install direnv:"
        echo "  - macOS: brew install direnv"
        echo "  - Ubuntu/Debian: sudo apt install direnv"
        echo "  - Fedora: sudo dnf install direnv"
        echo "  - Arch: sudo pacman -S direnv"
        echo ""
        echo "After installation, add to your shell config:"
        echo "  - bash: echo 'eval \"\$(direnv hook bash)\"' >> ~/.bashrc"
        echo "  - zsh: echo 'eval \"\$(direnv hook zsh)\"' >> ~/.zshrc"
        echo ""
        return 1
    fi
}

# Main bootstrap process
main() {
    echo "=========================================="
    echo "  ROS2 Humble Environment Bootstrap"
    echo "=========================================="
    echo ""
    
    # Check prerequisites
    if ! check_nix; then
        exit 1
    fi
    
    check_pixi_in_shell
    
    echo ""
    
    # Check for direnv (recommended)
    DIRENV_AVAILABLE=0
    if check_direnv; then
        DIRENV_AVAILABLE=1
    fi
    
    echo ""
    print_info "Entering Nix dev shell and running pixi install..."
    echo ""
    
    # Enter the dev shell and run pixi install
    nix develop --command bash -c '
        set -euo pipefail
        
        # Verify pixi is available
        if ! command -v pixi &> /dev/null; then
            echo -e "'"${RED}"'ERROR: pixi is not available in the dev shell'"${NC}"'" >&2
            exit 1
        fi
        
        echo "Pixi version: $(pixi --version)"
        echo ""
        echo "Running pixi install..."
        pixi install
        
        echo ""
        echo -e "'"${GREEN}"'Bootstrap completed successfully!'"${NC}"'"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "  Next Steps"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
    '
    
    exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        # Provide next steps based on whether direnv is available
        if [ $DIRENV_AVAILABLE -eq 1 ]; then
            echo "RECOMMENDED: Use direnv for automatic environment activation"
            echo ""
            echo "  1. Allow direnv for this directory:"
            echo "     direnv allow"
            echo ""
            echo "  2. The environment will activate automatically when you cd into this directory"
            echo ""
            echo "Or manually enter the environment:"
            echo "  nix develop"
        else
            echo "To enter the development environment:"
            echo ""
            echo "  nix develop"
            echo ""
            echo "RECOMMENDED: Install direnv for automatic environment activation"
            echo "  (see installation instructions above)"
        fi
        echo ""
        print_success "Setup complete!"
    else
        echo ""
        print_error "Bootstrap failed with exit code $exit_code"
        exit $exit_code
    fi
}

# Run main function
main "$@"
