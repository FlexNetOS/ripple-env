#!/usr/bin/env bash
#
# bootstrap.sh - Complete end-to-end installation for ripple-env
#
# This script installs all dependencies and sets up the development environment:
# - Nix with flakes enabled (using experimental installer)
# - direnv for automatic environment activation
# - nix-output-monitor (nom) for nicer output (optional; nix remains the primary CLI)
# - pixi for package management
# - zsh and nushell shells
# - git and gh cli
#
# Usage: ./bootstrap.sh [--ci] [--skip-shells] [--verify] [--profile PROFILE] [--resume] [--clean]
#   --ci          : Run in CI mode (non-interactive, skip optional components)
#   --skip-shells : Skip installing zsh and nushell
#   --verify      : Run post-install verification using ARIA manifest
#   --profile     : Verification profile (minimal, ci, default, full)
#   --resume      : Resume from last saved state (if available)
#   --clean       : Clean state and start fresh
#   --log-file    : Write logs to specified file (default: /tmp/bootstrap-<timestamp>.log)
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

# Logging functions (defined early for use in argument parsing)
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

log_debug() {
    if [ "${DEBUG:-false}" = true ]; then
        echo -e "${BLUE}[DEBUG]${NC} $1"
    fi
}

# Timestamped logging for file output
log_with_timestamp() {
    local level="$1"
    local message="$2"
    local timestamp
    timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "[$timestamp] [$level] $message"
}

# Write to log file if configured
write_log() {
    local level="$1"
    local message="$2"
    if [ -n "${LOG_FILE:-}" ]; then
        log_with_timestamp "$level" "$message" >> "$LOG_FILE"
    fi
}

# Combined logging function
log() {
    local level="$1"
    local message="$2"

    case "$level" in
        INFO)    log_info "$message" ;;
        SUCCESS) log_success "$message" ;;
        WARN)    log_warn "$message" ;;
        ERROR)   log_error "$message" ;;
        DEBUG)   log_debug "$message" ;;
    esac

    write_log "$level" "$message"
}

# ============================================================================
# RETRY LOGIC WITH EXPONENTIAL BACKOFF
# ============================================================================

# Retry a command with exponential backoff
# Usage: retry_with_backoff <max_attempts> <initial_delay> <command...>
retry_with_backoff() {
    local max_attempts="$1"
    local initial_delay="$2"
    shift 2

    local attempt=1
    local delay="$initial_delay"

    while [ $attempt -le "$max_attempts" ]; do
        log DEBUG "Attempt $attempt/$max_attempts: $*"

        if "$@"; then
            return 0
        fi

        local exit_code=$?

        if [ $attempt -eq "$max_attempts" ]; then
            log ERROR "Command failed after $max_attempts attempts: $*"
            return $exit_code
        fi

        log WARN "Attempt $attempt failed (exit code: $exit_code). Retrying in ${delay}s..."
        sleep "$delay"

        # Exponential backoff: double the delay for next attempt (max 60s)
        delay=$((delay * 2))
        if [ $delay -gt 60 ]; then
            delay=60
        fi

        attempt=$((attempt + 1))
    done
}

# Retry network operations (curl, etc.) with appropriate backoff
# Default: 4 attempts, starting with 2s delay
retry_network() {
    retry_with_backoff "${NETWORK_RETRY_ATTEMPTS:-4}" "${NETWORK_RETRY_DELAY:-2}" "$@"
}

# ============================================================================
# STATE PERSISTENCE
# ============================================================================

STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/ripple-env"
STATE_FILE="$STATE_DIR/bootstrap.state"

# Bootstrap stages (in order)
STAGES=(
    "detect_system"
    "install_nix"
    "enable_flakes"
    "install_direnv"
    "install_nom"
    "install_git"
    "install_gh"
    "install_zsh"
    "install_nushell"
    "setup_direnv_hooks"
    "verify_environment"
    "verify_pixi"
    "verify_manifest"
)

init_state() {
    mkdir -p "$STATE_DIR"

    if [ ! -f "$STATE_FILE" ]; then
        cat > "$STATE_FILE" << EOF
# Bootstrap state file - do not edit manually
# Created: $(date -Iseconds)
LAST_COMPLETED_STAGE=""
STARTED_AT="$(date -Iseconds)"
LAST_UPDATED="$(date -Iseconds)"
BOOTSTRAP_VERSION="2.0"
EOF
    fi

    # shellcheck source=/dev/null
    source "$STATE_FILE"
}

save_state() {
    local stage="$1"
    cat > "$STATE_FILE" << EOF
# Bootstrap state file - do not edit manually
# Created: ${STARTED_AT:-$(date -Iseconds)}
LAST_COMPLETED_STAGE="$stage"
STARTED_AT="${STARTED_AT:-$(date -Iseconds)}"
LAST_UPDATED="$(date -Iseconds)"
BOOTSTRAP_VERSION="2.0"
EOF
    log DEBUG "State saved: $stage"
}

clear_state() {
    rm -f "$STATE_FILE"
    log INFO "Bootstrap state cleared"
}

get_last_stage() {
    if [ -f "$STATE_FILE" ]; then
        # shellcheck source=/dev/null
        source "$STATE_FILE"
        echo "${LAST_COMPLETED_STAGE:-}"
    fi
}

should_skip_stage() {
    local stage="$1"
    local last_completed="${LAST_COMPLETED_STAGE:-}"

    if [ -z "$last_completed" ]; then
        return 1  # Don't skip - no previous state
    fi

    if [ "$RESUME_MODE" != true ]; then
        return 1  # Don't skip - not in resume mode
    fi

    # Find indices of both stages
    local last_idx=-1
    local current_idx=-1
    local idx=0

    for s in "${STAGES[@]}"; do
        if [ "$s" = "$last_completed" ]; then
            last_idx=$idx
        fi
        if [ "$s" = "$stage" ]; then
            current_idx=$idx
        fi
        idx=$((idx + 1))
    done

    # Skip if current stage comes before or equals last completed
    if [ $current_idx -le $last_idx ]; then
        log INFO "Skipping already completed stage: $stage"
        return 0
    fi

    return 1
}

# ============================================================================
# CLEANUP AND ERROR HANDLING
# ============================================================================

CLEANUP_TASKS=()

register_cleanup() {
    CLEANUP_TASKS+=("$1")
}

run_cleanup() {
    log INFO "Running cleanup tasks..."
    for task in "${CLEANUP_TASKS[@]}"; do
        log DEBUG "Cleanup: $task"
        eval "$task" || true
    done
}

# Trap for cleanup on error
cleanup_on_error() {
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        log ERROR "Bootstrap failed with exit code: $exit_code"
        log ERROR "Last completed stage: $(get_last_stage)"
        log INFO "To resume from this point, run: $0 --resume"
        if [ -n "${LOG_FILE:-}" ]; then
            log INFO "See log file for details: $LOG_FILE"
        fi
        run_cleanup
    fi
}

trap cleanup_on_error EXIT

# ============================================================================
# CONFIGURATION
# ============================================================================

# Configuration
CI_MODE=false
SKIP_SHELLS=false
RUN_VERIFY=false
VERIFY_PROFILE="default"
RESUME_MODE=false
CLEAN_MODE=false
DEBUG=${DEBUG:-false}
LOG_FILE=""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST_FILE="$SCRIPT_DIR/ARIA_MANIFEST.yaml"
VERIFY_SCRIPT="$SCRIPT_DIR/scripts/verify-components.sh"

# Network retry configuration
NETWORK_RETRY_ATTEMPTS=${NETWORK_RETRY_ATTEMPTS:-4}
NETWORK_RETRY_DELAY=${NETWORK_RETRY_DELAY:-2}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --ci)
            CI_MODE=true
            VERIFY_PROFILE="ci"
            shift
            ;;
        --skip-shells)
            SKIP_SHELLS=true
            shift
            ;;
        --verify)
            RUN_VERIFY=true
            shift
            ;;
        --profile)
            VERIFY_PROFILE="$2"
            shift 2
            ;;
        --resume)
            RESUME_MODE=true
            shift
            ;;
        --clean)
            CLEAN_MODE=true
            shift
            ;;
        --debug)
            DEBUG=true
            shift
            ;;
        --log-file)
            LOG_FILE="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --ci            Run in CI mode (non-interactive)"
            echo "  --skip-shells   Skip installing zsh and nushell"
            echo "  --verify        Run post-install verification using ARIA manifest"
            echo "  --profile NAME  Verification profile (minimal, ci, default, full)"
            echo "  --resume        Resume from last saved state"
            echo "  --clean         Clean state and start fresh"
            echo "  --debug         Enable debug output"
            echo "  --log-file PATH Write logs to specified file"
            echo ""
            echo "Environment variables:"
            echo "  NETWORK_RETRY_ATTEMPTS  Number of retry attempts for network operations (default: 4)"
            echo "  NETWORK_RETRY_DELAY     Initial delay in seconds between retries (default: 2)"
            echo "  DEBUG                   Enable debug output (default: false)"
            echo ""
            echo "State files are stored in: \${XDG_STATE_HOME:-\$HOME/.local/state}/ripple-env/"
            exit 0
            ;;
        *)
            log_warn "Unknown argument: $1"
            shift
            ;;
    esac
done

# Initialize log file if not specified
if [ -z "$LOG_FILE" ] && [ "$CI_MODE" = false ]; then
    LOG_FILE="/tmp/bootstrap-$(date +%Y%m%d-%H%M%S).log"
fi

# Initialize logging
if [ -n "$LOG_FILE" ]; then
    mkdir -p "$(dirname "$LOG_FILE")"
    log INFO "Logging to: $LOG_FILE"
fi

# Handle clean mode
if [ "$CLEAN_MODE" = true ]; then
    clear_state
fi

# Initialize state tracking
init_state

# Report resume status
if [ "$RESUME_MODE" = true ]; then
    last_stage=$(get_last_stage)
    if [ -n "$last_stage" ]; then
        log INFO "Resuming from stage: $last_stage"
    else
        log INFO "No previous state found, starting fresh"
    fi
fi

check_command() {
    command -v "$1" &> /dev/null
}

source_nix_profile() {
    # nix-daemon.sh guards itself with __ETC_PROFILE_NIX_SOURCED. In some
    # environments that variable may already be set, which can prevent PATH
    # updates and make `nix` appear "not found".
    unset __ETC_PROFILE_NIX_SOURCED 2>/dev/null || true

    # Multi-user installs typically provide nix-daemon.sh; single-user installs
    # provide nix.sh.
    if [ -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
    elif [ -f /nix/var/nix/profiles/default/etc/profile.d/nix.sh ]; then
        . /nix/var/nix/profiles/default/etc/profile.d/nix.sh
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
    #
    # In single-user installs (common in devcontainers/Codespaces), there is no
    # nix-daemon socket and that's fine—don't warn or try to start it.
    if [ ! -f /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh ] \
        && [ -f /nix/var/nix/profiles/default/etc/profile.d/nix.sh ] \
        && [ ! -S /nix/var/nix/daemon-socket/socket ]; then
        log DEBUG "Single-user Nix detected; skipping nix-daemon checks"
        return 0
    fi

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
    if should_skip_stage "detect_system"; then
        # Even when skipping, we need to set the variables
        OS="$(uname -s)"
        ARCH="$(uname -m)"
        case "$ARCH" in
            x86_64|amd64) ARCH="x86_64" ;;
            aarch64|arm64) ARCH="aarch64" ;;
        esac
        case "$OS" in
            Linux)
                if [ -f /etc/os-release ]; then
                    # shellcheck source=/dev/null
                    . /etc/os-release
                    DISTRO="$ID"
                else
                    DISTRO="unknown"
                fi
                ;;
            Darwin)
                DISTRO="macos"
                ;;
        esac
        return 0
    fi

    log_info "Detecting system..."

    OS="$(uname -s)"
    ARCH="$(uname -m)"

    # Validate architecture
    case "$ARCH" in
        x86_64|amd64)
            ARCH="x86_64"
            ;;
        aarch64|arm64)
            ARCH="aarch64"
            ;;
        *)
            log_error "Unsupported architecture: $ARCH"
            log_error "Supported architectures: x86_64, aarch64"
            exit 1
            ;;
    esac

    case "$OS" in
        Linux)
            if [ -f /etc/os-release ]; then
                # shellcheck source=/dev/null
                . /etc/os-release
                DISTRO="$ID"

                # Validate Ubuntu version if applicable
                if [ "$ID" = "ubuntu" ]; then
                    UBUNTU_VERSION="${VERSION_ID:-0}"
                    UBUNTU_MAJOR="${UBUNTU_VERSION%%.*}"
                    if [ "$UBUNTU_MAJOR" -lt 20 ] 2>/dev/null; then
                        log_warn "Ubuntu $VERSION_ID detected. Ubuntu 20.04+ is recommended for best compatibility."
                    fi
                fi
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
    save_state "detect_system"
}

# Download Nix installer script with retry
download_nix_installer() {
    local installer_path="$1"
    curl --proto '=https' --tlsv1.2 -sSf -L \
        --connect-timeout 30 \
        --max-time 300 \
        -o "$installer_path" \
        https://install.determinate.systems/nix
}

# Install Nix using the experimental installer
install_nix() {
    if should_skip_stage "install_nix"; then
        return 0
    fi

    if check_command nix; then
        log_info "Nix is already installed"
        nix --version
        save_state "install_nix"
        return 0
    fi

    log_info "Installing Nix with experimental installer..."

    # Create a temporary file for the installer script
    local installer_script
    installer_script="$(mktemp)"
    register_cleanup "rm -f '$installer_script'"

    # Download the installer with retry logic
    log_info "Downloading Nix installer (with retry on failure)..."
    if ! retry_network download_nix_installer "$installer_script"; then
        log_error "Failed to download Nix installer after multiple attempts"
        log_error "Please check your network connection and try again"
        return 1
    fi

    # Verify the installer was downloaded
    if [ ! -s "$installer_script" ]; then
        log_error "Downloaded installer script is empty"
        return 1
    fi

    log_info "Running Nix installer..."
    if [ "$CI_MODE" = true ]; then
        # CI mode: use non-interactive installation
        if ! sh "$installer_script" -- install --no-confirm; then
            log_error "Nix installation failed"
            log_error "Check the installer output above for details"
            return 1
        fi
    else
        if ! sh "$installer_script" -- install; then
            log_error "Nix installation failed"
            log_error "Check the installer output above for details"
            return 1
        fi
    fi

    # Cleanup installer
    rm -f "$installer_script"

    source_nix_profile
    ensure_nix_daemon

    # Verify Nix is now available
    if ! check_command nix; then
        log_error "Nix installation completed but 'nix' command not found in PATH"
        log_error "Try sourcing your shell profile or restarting your shell"
        return 1
    fi

    log_success "Nix installed successfully"
    save_state "install_nix"
}

# Enable flakes if not already enabled
enable_flakes() {
    if should_skip_stage "enable_flakes"; then
        return 0
    fi

    log_info "Checking flakes configuration..."

    local nix_conf_dir="$HOME/.config/nix"
    local nix_conf="$nix_conf_dir/nix.conf"

    mkdir -p "$nix_conf_dir"

    # Add flakes configuration (idempotent)
    if [ ! -f "$nix_conf" ]; then
        touch "$nix_conf"
    fi

    if grep -qE '^\s*experimental-features\s*=.*\bflakes\b' "$nix_conf"; then
        log_info "Flakes already enabled"
    else
        echo "experimental-features = nix-command flakes" >> "$nix_conf"
        log_success "Flakes enabled in $nix_conf"
    fi

    # Allow per-flake nixConfig (needed to pick up caches/keys from flake.nix)
    if grep -qE '^\s*accept-flake-config\s*=\s*true\s*$' "$nix_conf"; then
        log_info "accept-flake-config already enabled"
    else
        echo "accept-flake-config = true" >> "$nix_conf"
        log_success "Enabled accept-flake-config in $nix_conf"
    fi

    save_state "enable_flakes"
}

# Helper function for nix profile install with retry
nix_profile_install() {
    local package="$1"
    nix profile install "$package"
}

# Install direnv
install_direnv() {
    if should_skip_stage "install_direnv"; then
        return 0
    fi

    if check_command direnv; then
        log_info "direnv is already installed"
        direnv --version
        save_state "install_direnv"
        return 0
    fi

    log_info "Installing direnv via nix..."
    if ! retry_network nix_profile_install nixpkgs#direnv; then
        log_error "Failed to install direnv after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command direnv; then
        log_error "direnv installation completed but command not found"
        return 1
    fi

    log_success "direnv installed successfully"
    save_state "install_direnv"
}

# Install nom (nix output monitor)
install_nom() {
    if should_skip_stage "install_nom"; then
        return 0
    fi

    if check_command nom; then
        log_info "nom is already installed"
        nom --version
        save_state "install_nom"
        return 0
    fi

    log_info "Installing nom via nix..."
    if ! retry_network nix_profile_install nixpkgs#nix-output-monitor; then
        log_error "Failed to install nom after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command nom; then
        log_error "nom installation completed but command not found"
        return 1
    fi

    log_success "nom installed successfully"
    save_state "install_nom"
}

# Install git if not present
install_git() {
    if should_skip_stage "install_git"; then
        return 0
    fi

    if check_command git; then
        log_info "git is already installed"
        git --version
        save_state "install_git"
        return 0
    fi

    log_info "Installing git via nix..."
    if ! retry_network nix_profile_install nixpkgs#git; then
        log_error "Failed to install git after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command git; then
        log_error "git installation completed but command not found"
        return 1
    fi

    log_success "git installed successfully"
    save_state "install_git"
}

# Install GitHub CLI
install_gh() {
    if should_skip_stage "install_gh"; then
        return 0
    fi

    if check_command gh; then
        log_info "gh cli is already installed"
        gh --version
        save_state "install_gh"
        return 0
    fi

    log_info "Installing gh cli via nix..."
    if ! retry_network nix_profile_install nixpkgs#gh; then
        log_error "Failed to install gh cli after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command gh; then
        log_error "gh cli installation completed but command not found"
        return 1
    fi

    log_success "gh cli installed successfully"
    save_state "install_gh"
}

# Install zsh
install_zsh() {
    if should_skip_stage "install_zsh"; then
        return 0
    fi

    if [ "$SKIP_SHELLS" = true ]; then
        log_info "Skipping zsh installation (--skip-shells)"
        save_state "install_zsh"
        return 0
    fi

    if check_command zsh; then
        log_info "zsh is already installed"
        zsh --version
        save_state "install_zsh"
        return 0
    fi

    log_info "Installing zsh via nix..."
    if ! retry_network nix_profile_install nixpkgs#zsh; then
        log_error "Failed to install zsh after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command zsh; then
        log_error "zsh installation completed but command not found"
        return 1
    fi

    log_success "zsh installed successfully"
    save_state "install_zsh"
}

# Install nushell
install_nushell() {
    if should_skip_stage "install_nushell"; then
        return 0
    fi

    if [ "$SKIP_SHELLS" = true ]; then
        log_info "Skipping nushell installation (--skip-shells)"
        save_state "install_nushell"
        return 0
    fi

    if check_command nu; then
        log_info "nushell is already installed"
        nu --version
        save_state "install_nushell"
        return 0
    fi

    log_info "Installing nushell via nix..."
    if ! retry_network nix_profile_install nixpkgs#nushell; then
        log_error "Failed to install nushell after multiple attempts"
        return 1
    fi

    # Verify installation
    if ! check_command nu; then
        log_error "nushell installation completed but command not found"
        return 1
    fi

    log_success "nushell installed successfully"
    save_state "install_nushell"
}

# Setup direnv hook for shells
setup_direnv_hooks() {
    if should_skip_stage "setup_direnv_hooks"; then
        return 0
    fi

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
    save_state "setup_direnv_hooks"
}

# Verify the flake and development environment
verify_environment() {
    if should_skip_stage "verify_environment"; then
        return 0
    fi

    log_info "Verifying development environment..."

    cd "$SCRIPT_DIR"

    # Check if flake.nix exists
    if [ ! -f "flake.nix" ]; then
        log_error "flake.nix not found in $SCRIPT_DIR"
        return 1
    fi

    # Verify flake (fail hard if it doesn't evaluate)
    # NOTE: `--all-systems` can be significantly slower. Keep it for CI, but
    # default to the current system for interactive/local usage.
    log_info "Checking flake..."
    local flake_check_args=(--no-build)
    if [ "$CI_MODE" = true ]; then
        flake_check_args+=(--all-systems)
    fi

    if ! nix flake check --accept-flake-config "${flake_check_args[@]}"; then
        log_error "nix flake check failed"
        log_info "flake output (for debugging):"
        nix flake show --accept-flake-config || true
        return 1
    fi

    # Build the devshell (this validates the configuration)
    log_info "Building development shell..."
    if ! nix develop --accept-flake-config --command echo "Development shell verified successfully"; then
        log_error "Failed to build development shell"
        return 1
    fi

    log_success "Environment verification complete"
    save_state "verify_environment"
}

# Verify pixi is available in the environment
verify_pixi() {
    if should_skip_stage "verify_pixi"; then
        return 0
    fi

    log_info "Verifying pixi setup..."

    cd "$SCRIPT_DIR"

    # Check if pixi.toml exists
    if [ ! -f "pixi.toml" ]; then
        log_error "pixi.toml not found in $SCRIPT_DIR"
        return 1
    fi

    # Run pixi install within the nix environment
    log_info "Running pixi install..."
    if ! nix develop --accept-flake-config --command pixi install; then
        log_error "pixi install failed"
        return 1
    fi

    log_success "pixi setup verified"
    save_state "verify_pixi"
}

# Verify components using ARIA manifest (Single Source of Truth)
verify_manifest_components() {
    if should_skip_stage "verify_manifest"; then
        return 0
    fi

    log_info "Running ARIA manifest-based verification (profile: $VERIFY_PROFILE)..."

    cd "$SCRIPT_DIR"

    # Check if manifest exists
    if [ ! -f "$MANIFEST_FILE" ]; then
        log_warn "ARIA_MANIFEST.yaml not found, skipping manifest verification"
        save_state "verify_manifest"
        return 0
    fi

    # Validate manifest first (if validation script exists)
    local validate_script="$SCRIPT_DIR/scripts/validate-manifest.py"
    if [ -f "$validate_script" ]; then
        log_info "Validating ARIA_MANIFEST.yaml..."
        # Run using pixi's Python so dependencies are consistent and available.
        if nix develop --command pixi run python "$validate_script" --manifest "$MANIFEST_FILE" 2>/dev/null; then
            log_success "Manifest validation passed"
        else
            log_warn "Manifest validation had issues (non-fatal)"
        fi
    fi

    # Generate verification script if generator exists
    local generator_script="$SCRIPT_DIR/scripts/generate-verification.py"
    if [ -f "$generator_script" ]; then
        log_info "Generating verification script from manifest..."
        nix develop --command pixi run python "$generator_script" --manifest "$MANIFEST_FILE" --output "$VERIFY_SCRIPT" 2>/dev/null || true
    fi

    # Run verification script if it exists
    if [ -f "$VERIFY_SCRIPT" ] && [ -x "$VERIFY_SCRIPT" ]; then
        log_info "Running component verification..."
        nix develop --command bash -c "$VERIFY_SCRIPT --profile $VERIFY_PROFILE" || {
            log_warn "Some verifications failed (non-fatal in bootstrap)"
            save_state "verify_manifest"
            return 0
        }
        log_success "Component verification complete"
    else
        # Fallback to basic inline verification
        log_info "Running basic component verification..."
        verify_basic_components
    fi

    save_state "verify_manifest"
}

# Basic component verification (fallback if no manifest/scripts)
verify_basic_components() {
    local failed=0

    log_info "Checking core components..."

    # Nix infrastructure
    if check_command nix; then
        log_success "  nix: $(nix --version)"
    else
        log_error "  nix: NOT FOUND"
        ((failed++))
    fi

    if check_command direnv; then
        log_success "  direnv: $(direnv --version)"
    else
        log_warn "  direnv: NOT FOUND"
    fi

    if check_command git; then
        log_success "  git: $(git --version | head -1)"
    else
        log_warn "  git: NOT FOUND"
    fi

    if check_command gh; then
        log_success "  gh: $(gh --version | head -1)"
    else
        log_warn "  gh: NOT FOUND"
    fi

    # Check in nix develop environment
    log_info "Checking devshell components..."
    nix develop --command bash -c '
        echo "  pixi: $(pixi --version 2>/dev/null || echo NOT FOUND)"
        echo "  jq: $(jq --version 2>/dev/null || echo NOT FOUND)"
        echo "  curl: $(curl --version 2>/dev/null | head -1 || echo NOT FOUND)"
    ' 2>/dev/null || log_warn "Could not verify devshell components"

    if [ $failed -gt 0 ]; then
        log_error "Some required components are missing"
        return 1
    fi

    log_success "Basic component verification complete"
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
    echo "  # (Preferred option) nom develop"
    echo ""
    echo "Preferred Option (nicer output):"
    echo "  nom develop"
    echo ""
    echo "To run component verification:"
    echo "  ./bootstrap.sh --verify --profile default"
    echo "  # Profiles: minimal, ci, default, full"
    echo ""
    if [ -n "${LOG_FILE:-}" ] && [ -f "$LOG_FILE" ]; then
        echo "Log file: $LOG_FILE"
        echo ""
    fi
    echo "State file: $STATE_FILE"
    echo "To clean state and start fresh: ./bootstrap.sh --clean"
    echo ""
}

# Main execution
main() {
    echo "========================================"
    echo "ripple-env Bootstrap Script"
    echo "========================================"
    echo ""

    if [ -n "${LOG_FILE:-}" ]; then
        log INFO "Bootstrap started at $(date -Iseconds)"
        log INFO "Arguments: $*"
        log INFO "Working directory: $SCRIPT_DIR"
    fi

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
        save_state "setup_direnv_hooks"
    else
        setup_direnv_hooks
    fi

    # Verify environment
    verify_environment
    verify_pixi

    # Run manifest-based verification if requested or in CI mode
    if [ "$RUN_VERIFY" = true ] || [ "$CI_MODE" = true ]; then
        verify_manifest_components
    else
        # Mark as complete even if not run
        save_state "verify_manifest"
    fi

    # Clear state on successful completion
    clear_state
    log INFO "Bootstrap completed successfully at $(date -Iseconds)"

    print_summary

    # Disable the error trap since we succeeded
    trap - EXIT
}

main "$@"
