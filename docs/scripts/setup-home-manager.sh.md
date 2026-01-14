# Script Contract: setup-home-manager.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/setup-home-manager.sh`

---

## Purpose

Interactive Home Manager setup script that configures user environment management for ripple-env. Creates configuration directory, copies and customizes template `home.nix`, validates configuration, and applies settings. Supports both interactive mode (with prompts) and automatic mode (no prompts). Manages dotfiles, user packages, and development tool configuration across systems.

---

## Invocation

```bash
./scripts/setup-home-manager.sh [OPTIONS]
```

**Options:**
- (no options) - Interactive setup with prompts
- `--auto`, `-a` - Automatic setup without prompts
- `--validate` - Validate existing configuration
- `--apply` - Apply existing configuration
- `--help`, `-h` - Show help message

**Examples:**
```bash
./scripts/setup-home-manager.sh              # Interactive setup
./scripts/setup-home-manager.sh --auto       # Automatic setup
./scripts/setup-home-manager.sh --validate   # Validate current config
./scripts/setup-home-manager.sh --apply      # Apply current config
```

**Requirements:**
- `home-manager` command available
- `nix` installed
- Nix flake configured with home-manager

---

## Outputs

**Standard Output (interactive setup - first time):**
```
[INFO] Starting interactive Home Manager setup...

This script will help you set up Home Manager for ripple-env.
Home Manager allows you to:
  - Manage user packages and dotfiles
  - Configure development tools
  - Keep your environment consistent across systems

Continue with setup? (Y/n): y

[INFO] Setting up home-manager configuration directory...
[SUCCESS] Created directory: /home/user/.config/home-manager

[INFO] Customizing template for current user...
[SUCCESS] Template customized for user: username

[SUCCESS] Copied template to: /home/user/.config/home-manager/home.nix

Validate configuration before applying? (Y/n): y

[INFO] Validating home-manager configuration...
[SUCCESS] Configuration is valid

Apply configuration now? (Y/n): y

[INFO] Applying home-manager configuration...
[SUCCESS] Home Manager configuration applied successfully!
```

**Standard Output (existing configuration found):**
```
[INFO] Setting up home-manager configuration directory...
[INFO] Directory already exists: /home/user/.config/home-manager

[WARNING] Existing home.nix found at /home/user/.config/home-manager/home.nix
Backup existing config? (y/N): y
[SUCCESS] Backed up to: /home/user/.config/home-manager/home.nix.backup.20260113_103045

Overwrite with template? (y/N): n
[INFO] Keeping existing configuration
```

**Standard Output (automatic mode):**
```
[INFO] Starting automatic Home Manager setup...
[INFO] Setting up home-manager configuration directory...
[SUCCESS] Created directory: /home/user/.config/home-manager
[INFO] Customizing template for current user...
[SUCCESS] Template customized for user: username
[SUCCESS] Copied template to: /home/user/.config/home-manager/home.nix
[INFO] Validating home-manager configuration...
[SUCCESS] Configuration is valid
[INFO] Applying home-manager configuration...
[SUCCESS] Home Manager configuration applied successfully!
```

**Standard Output (validation only):**
```
[INFO] Validating existing configuration...
[SUCCESS] Configuration is valid
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (configuration applied or validated) |
| `1` | Failure (home-manager not available, validation failed, no config found) |

---

## Side Effects

### Configuration Directory (line 71)

**Evidence:**
```bash
setup_config_dir() {
    log_info "Setting up home-manager configuration directory..."

    if [[ ! -d "$HOME_MANAGER_CONFIG_DIR" ]]; then
        mkdir -p "$HOME_MANAGER_CONFIG_DIR"
        log_success "Created directory: $HOME_MANAGER_CONFIG_DIR"
    else
        log_info "Directory already exists: $HOME_MANAGER_CONFIG_DIR"
    fi
}
```

**Creates:** `~/.config/home-manager/` directory

### Configuration File (line 99)

**Evidence:**
```bash
cp "$REPO_HOME_NIX" "$HOME_MANAGER_CONFIG"
log_success "Copied template to: $HOME_MANAGER_CONFIG"
```

**Creates:** `~/.config/home-manager/home.nix`

### Backup Files (line 86)

**Evidence:**
```bash
backup_file="$HOME_MANAGER_CONFIG.backup.$(date +%Y%m%d_%H%M%S)"
cp "$HOME_MANAGER_CONFIG" "$backup_file"
log_success "Backed up to: $backup_file"
```

**Creates:** `~/.config/home-manager/home.nix.backup.YYYYMMDD_HHMMSS`

### Template Customization (lines 119-120)

**Evidence:**
```bash
sed -i "s/user@example.com/$useremail/g" "$HOME_MANAGER_CONFIG"
sed -i "s/Ripple Environment User/$username/g" "$HOME_MANAGER_CONFIG"
```

**Modifies:** `home.nix` with current username and email

### Home Manager Application (line 174)

**Evidence:**
```bash
if home-manager switch; then
    log_success "Home Manager configuration applied successfully!"
    return 0
fi
```

**Installs:** User packages, creates symlinks for dotfiles, configures programs

---

## Safety Classification

**🟡 CAUTION** - Creates files, modifies configuration, installs packages.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**
- Configuration directory creation is idempotent (line 70)
- Template copy prompts before overwriting (line 82)
- Backup creation uses timestamps (line 86)
- Home Manager application is idempotent (line 174)

---

## Home Manager Check

**Evidence:** Lines 51-64

```bash
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
```

**Installation methods:**
1. `nix profile install nixpkgs#home-manager` - Standalone installation (line 59)
2. `nix develop` - Via project flake (line 61)

**Exits if not available** (line 344)

---

## Configuration Template

### Template Copy (lines 78-109)

**Evidence:**
```bash
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
```

**Template source:** `home.nix` in project root (line 33)

**Interactive prompts:**
1. Backup existing config? (line 84)
2. Overwrite with template? (line 91)

**Fallback:** Creates minimal config if template missing (line 107)

### Template Customization (lines 111-123)

**Evidence:**
```bash
customize_template() {
    log_info "Customizing template for current user..."

    local username=$(whoami)
    local useremail="${username}@$(hostname)"

    # Replace placeholders in template
    sed -i "s/user@example.com/$useremail/g" "$HOME_MANAGER_CONFIG"
    sed -i "s/Ripple Environment User/$username/g" "$HOME_MANAGER_CONFIG"

    log_success "Template customized for user: $username"
}
```

**Placeholders replaced:**
- `user@example.com` → `username@hostname`
- `Ripple Environment User` → current username

**sed -i:** In-place editing (lines 119-120)

### Minimal Configuration (lines 125-155)

**Evidence:**
```bash
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
```

**Minimal packages:**
- git - Version control
- curl - HTTP client
- htop - Process monitor
- tree - Directory viewer

**Basic programs:**
- bash - Shell
- git - With username/email

**State version:** 24.11 (Nix 24.11 release) (line 134)

---

## Configuration Validation

**Evidence:** Lines 157-168

```bash
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
```

**Command:** `home-manager build`

**Purpose:** Dry-run build without activating configuration

**Exit codes:**
- `0` - Configuration valid
- `1` - Syntax error, missing attribute, or other issue

---

## Configuration Application

**Evidence:** Lines 170-181

```bash
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
```

**Command:** `home-manager switch`

**Effects:**
1. Builds configuration
2. Creates generation (rollback point)
3. Symlinks dotfiles to `~/.config/`, `~/.bashrc`, etc.
4. Installs user packages
5. Configures programs

**Rollback:** `home-manager generations` → `home-manager switch --switch-generation <N>`

---

## Interactive Setup

**Evidence:** Lines 183-224

```bash
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
```

**Prompts:**
1. Continue with setup? (line 195)
2. Validate configuration? (line 202)
3. Apply configuration? (line 206)

**Flow:**
```
Start → Prompt 1 → Setup config dir → Copy template →
Prompt 2 → Validate → Prompt 3 → Apply → Done
```

**Default answers:** Y (Yes) for all prompts

---

## Automatic Setup

**Evidence:** Lines 226-240

```bash
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
```

**No prompts** - Fully automated

**Exits on validation failure** (line 238)

**Usage:** CI/CD, automated provisioning

---

## Validate/Apply Existing

### Validate Existing (lines 277-291)

**Evidence:**
```bash
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
```

**Use case:** Check configuration after manual edits

### Apply Existing (lines 293-307)

**Evidence:**
```bash
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
```

**Use case:** Apply after manual edits without full setup

---

## Argument Parsing

**Evidence:** Lines 309-356

```bash
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
```

**Modes (mutually exclusive):**
1. `--validate` - Validate existing (line 348)
2. `--apply` - Apply existing (line 350)
3. `--auto` - Automatic setup (line 352)
4. (default) - Interactive setup (line 354)

---

## Configuration Location

**Evidence:** Lines 31-33

```bash
HOME_MANAGER_CONFIG_DIR="$HOME/.config/home-manager"
HOME_MANAGER_CONFIG="$HOME_MANAGER_CONFIG_DIR/home.nix"
REPO_HOME_NIX="home.nix"  # Template in repo root
```

**Paths:**
- Configuration directory: `~/.config/home-manager/`
- Configuration file: `~/.config/home-manager/home.nix`
- Template source: `home.nix` (project root)

---

## Post-Setup Workflow

**Evidence:** Lines 269-272

```bash
echo "After setup:"
echo "  - Edit ~/.config/home-manager/home.nix to customize"
echo "  - Run 'home-manager switch' to apply changes"
echo "  - Run 'home-manager --help' for more options"
```

**Common edits:**
```nix
# Add packages
home.packages = with pkgs; [
  git curl htop tree
  neovim ripgrep fd
  python3 nodejs
];

# Configure git
programs.git = {
  enable = true;
  userName = "Your Name";
  userEmail = "you@example.com";
  aliases = {
    st = "status";
    co = "checkout";
  };
};

# Configure shell
programs.bash = {
  enable = true;
  shellAliases = {
    ll = "ls -lah";
    gs = "git status";
  };
};
```

**Apply changes:**
```bash
# Validate first
home-manager build

# Then apply
home-manager switch

# Rollback if needed
home-manager generations
home-manager switch --switch-generation 42
```

---

## Troubleshooting

### Home Manager Not Available

**Symptoms:** "home-manager is not available" error (line 57)

**Fix:**
```bash
# Method 1: Install via nix profile
nix profile install nixpkgs#home-manager

# Method 2: Use project flake
nix develop

# Verify
home-manager --version
```

### Template Not Found

**Symptoms:** "Template file not found" error (line 105)

**Fix:**
```bash
# Check if home.nix exists in project root
ls -l home.nix

# Or let script create minimal config
# (script automatically creates minimal config as fallback)
```

### Validation Failed

**Symptoms:** "Configuration validation failed" error (line 165)

**Debug:**
```bash
# Check syntax errors
home-manager build 2>&1

# Common issues:
# - Missing semicolons
# - Undefined options
# - Type mismatches

# Example fix:
nano ~/.config/home-manager/home.nix
# Fix syntax, save

# Retry
home-manager build
```

### sed Command Failed on macOS

**Symptoms:** sed -i errors (lines 119-120)

**Fix (BSD sed on macOS):**
```bash
# Script uses GNU sed syntax
# Install GNU sed via Homebrew
brew install gnu-sed

# Or modify script to use BSD syntax:
sed -i '' "s/pattern/replacement/g" file
```

### Permission Denied

**Symptoms:** Cannot write to ~/.config/home-manager/

**Fix:**
```bash
# Check directory permissions
ls -ld ~/.config/home-manager/

# Fix ownership
sudo chown -R $USER:$USER ~/.config/home-manager/

# Fix permissions
chmod 755 ~/.config/home-manager/
chmod 644 ~/.config/home-manager/home.nix
```

---

## References

### Source Code
- **Main script:** `scripts/setup-home-manager.sh` (360 lines)
- **Home Manager check:** lines 51-64
- **Config directory setup:** lines 66-76
- **Template copy:** lines 78-109
- **Template customization:** lines 111-123
- **Minimal config:** lines 125-155
- **Validation:** lines 157-168
- **Application:** lines 170-181
- **Interactive setup:** lines 183-224
- **Automatic setup:** lines 226-240
- **Main function:** lines 309-356

### Related Files
- **Template:** `home.nix` (project root)
- **Configuration directory:** `~/.config/home-manager/`
- **Configuration file:** `~/.config/home-manager/home.nix`

### External Resources
- [Home Manager Manual](https://nix-community.github.io/home-manager/)
- [Home Manager Options](https://home-manager-options.extranix.com/)
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Home Manager GitHub](https://github.com/nix-community/home-manager)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 41/60 contracts complete (68.3%)
