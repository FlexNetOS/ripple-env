#!/usr/bin/env bash
# =============================================================================
# Home Manager Setup Script for ripple-env
# =============================================================================
# This script sets up Home Manager for user environment configuration
# and provides an easy way to manage dotfiles and user packages.
#
# Usage:
#   ./scripts/setup-home-manager.sh           # Interactive setup
#   ./scripts/setup-home-manager.sh --auto    # Automatic setup
#   ./scripts/setup-home-manager.sh --help    # Show help
#
# What this does:
#   1. Checks if home-manager is available
#   2. Creates home-manager configuration directory
#   3. Copies template configuration
#   4. Guides through initial setup
#   5. Applies the configuration
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
HOME_MANAGER_CONFIG_DIR="$HOME/.config/home-manager"
HOME_MANAGER_CONFIG="$HOME_MANAGER_CONFIG_DIR/home.nix"
REPO_HOME_NIX="home.nix"  # Template in repo root

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if home-manager is available
check_home_manager() {
    if command -v home-manager >/dev/null 2>&1; then
        log_success "home-manager is available: $(home-manager --version)"
        return 0
    else
        log_error "home-manager is not available"
        log_info "Please install home-manager first:"
        log_info "  nix profile install nixpkgs#home-manager"
        log_info "  OR"
        log_info "  nix develop .#default  # If home-manager is in flake"
        return 1
    fi
}

# Create home-manager configuration directory
setup_config_dir() {
    log_info "Setting up home-manager configuration directory..."
    
    if [[ ! -d "$HOME_MANAGER_CONFIG_DIR" ]]; then
        mkdir -p "$HOME_MANAGER_CONFIG_DIR"
        log_success "Created directory: $HOME_MANAGER_CONFIG_DIR"
    else
        log_info "Directory already exists: $HOME_MANAGER_CONFIG_DIR"
    fi
}

# Copy template configuration
copy_template() {
    if [[ -f "$REPO_HOME_NIX" ]]; then
        if [[ -f "$HOME_MANAGER_CONFIG" ]]; then
            log_warning "Existing home.nix found at $HOME_MANAGER_CONFIG"
            read -p "Backup existing config? (y/N): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                backup_file="$HOME_MANAGER_CONFIG.backup.$(date +%Y%m%d_%H%M%S)"
                cp "$HOME_MANAGER_CONFIG" "$backup_file"
                log_success "Backed up to: $backup_file"
            fi
            
            read -p "Overwrite with template? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                log_info "Keeping existing configuration"
                return 0
            fi
        fi
        
        cp "$REPO_HOME_NIX" "$HOME_MANAGER_CONFIG"
        log_success "Copied template to: $HOME_MANAGER_CONFIG"
        
        # Customize template for current user
        customize_template
    else
        log_error "Template file not found: $REPO_HOME_NIX"
        log_info "Creating minimal configuration instead..."
        create_minimal_config
    fi
}

# Customize template for current user
customize_template() {
    log_info "Customizing template for current user..."
    
    local username=$(whoami)
    local useremail="${username}@$(hostname)"
    
    # Replace placeholders in template
    sed -i "s/user@example.com/$useremail/g" "$HOME_MANAGER_CONFIG"
    sed -i "s/Ripple Environment User/$username/g" "$HOME_MANAGER_CONFIG"
    
    log_success "Template customized for user: $username"
}

# Create minimal configuration if template is missing
create_minimal_config() {
    log_info "Creating minimal home-manager configuration..."
    
    cat > "$HOME_MANAGER_CONFIG" << 'EOF'
{ config, pkgs, lib, ... }:

{
  programs.home-manager.enable = true;
  home.stateVersion = "24.11";
  home.username = builtins.getEnv "USER";
  home.homeDirectory = builtins.getEnv "HOME";
  
  # Basic packages
  home.packages = with pkgs; [
    git
    curl
    htop
    tree
  ];
  
  # Basic shell configuration
  programs.bash.enable = true;
  programs.git.enable = true;
  programs.git.userName = "$(whoami)";
  programs.git.userEmail = "$(whoami)@$(hostname)";
}
EOF
    
    log_success "Created minimal configuration"
}

# Validate configuration
validate_config() {
    log_info "Validating home-manager configuration..."
    
    if home-manager build; then
        log_success "Configuration is valid"
        return 0
    else
        log_error "Configuration validation failed"
        return 1
    fi
}

# Apply configuration
apply_config() {
    log_info "Applying home-manager configuration..."
    
    if home-manager switch; then
        log_success "Home Manager configuration applied successfully!"
        return 0
    else
        log_error "Failed to apply configuration"
        return 1
    fi
}

# Interactive setup
interactive_setup() {
    log_info "Starting interactive Home Manager setup..."
    echo
    
    echo "This script will help you set up Home Manager for ripple-env."
    echo "Home Manager allows you to:"
    echo "  - Manage user packages and dotfiles"
    echo "  - Configure development tools"
    echo "  - Keep your environment consistent across systems"
    echo
    
    read -p "Continue with setup? (Y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        setup_config_dir
        copy_template
        
        echo
        read -p "Validate configuration before applying? (Y/n): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            if validate_config; then
                read -p "Apply configuration now? (Y/n): " -n 1 -r
                echo
                if [[ ! $REPLY =~ ^[Nn]$ ]]; then
                    apply_config
                else
                    log_info "Configuration ready but not applied"
                    log_info "Run 'home-manager switch' to apply later"
                fi
            else
                log_error "Configuration validation failed"
                log_info "Please fix the configuration and try again"
            fi
        else
            apply_config
        fi
    else
        log_info "Setup cancelled"
    fi
}

# Automatic setup
auto_setup() {
    log_info "Starting automatic Home Manager setup..."
    
    setup_config_dir
    copy_template
    
    if validate_config; then
        apply_config
    else
        log_error "Automatic setup failed - configuration validation failed"
        log_info "Please run interactive setup to fix issues"
        exit 1
    fi
}

# Show help
show_help() {
    cat << EOF
Home Manager Setup Script for ripple-env

Usage: $0 [OPTIONS]

Options:
  --auto, -a        Automatic setup (no prompts)
  --help, -h        Show this help message
  --validate        Validate existing configuration
  --apply           Apply existing configuration

Examples:
  $0                # Interactive setup
  $0 --auto         # Automatic setup
  $0 --validate     # Validate current config
  $0 --apply        # Apply current config

What this does:
  1. Checks if home-manager is available
  2. Creates ~/.config/home-manager/ directory
  3. Copies home.nix template
  4. Customizes for current user
  5. Validates configuration
  6. Applies configuration

After setup:
  - Edit ~/.config/home-manager/home.nix to customize
  - Run 'home-manager switch' to apply changes
  - Run 'home-manager --help' for more options

EOF
}

# Validate existing configuration
validate_existing() {
    if [[ -f "$HOME_MANAGER_CONFIG" ]]; then
        log_info "Validating existing configuration..."
        if validate_config; then
            log_success "Configuration is valid"
        else
            log_error "Configuration has issues"
            exit 1
        fi
    else
        log_error "No configuration found at $HOME_MANAGER_CONFIG"
        exit 1
    fi
}

# Apply existing configuration
apply_existing() {
    if [[ -f "$HOME_MANAGER_CONFIG" ]]; then
        log_info "Applying existing configuration..."
        if apply_config; then
            log_success "Configuration applied successfully"
        else
            log_error "Failed to apply configuration"
            exit 1
        fi
    else
        log_error "No configuration found at $HOME_MANAGER_CONFIG"
        exit 1
    fi
}

# Main function
main() {
    # Parse arguments
    local auto_mode=false
    local validate_mode=false
    local apply_mode=false
    
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --auto|-a)
                auto_mode=true
                shift
                ;;
            --validate)
                validate_mode=true
                shift
                ;;
            --apply)
                apply_mode=true
                shift
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # Check prerequisites
    if ! check_home_manager; then
        exit 1
    fi
    
    # Execute requested action
    if [[ "$validate_mode" == true ]]; then
        validate_existing
    elif [[ "$apply_mode" == true ]]; then
        apply_existing
    elif [[ "$auto_mode" == true ]]; then
        auto_setup
    else
        interactive_setup
    fi
}

# Run main function
main "$@"