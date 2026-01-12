#!/usr/bin/env bash
# deploy-airgapped.sh - Deploy ripple-env in air-gapped environment
#
# This script sets up ripple-env on a machine without internet access
# using pre-prepared offline packages from prepare-offline.sh.
#
# Usage:
#   ./scripts/deploy-airgapped.sh [OPTIONS]
#
# Options:
#   --cache-dir DIR      Location of offline cache (default: /var/cache/ripple-env)
#   --bundle FILE        Extract from offline bundle archive
#   --setup-registry     Start local Docker registry
#   --import-images      Import Docker images from cache
#   --configure-nix      Configure Nix for offline use
#   --configure-pixi     Configure Pixi for offline use
#   --all                Perform all setup steps (default)
#   --verify             Verify offline setup
#   --help               Show this help message

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Default configuration
CACHE_DIR="${RIPPLE_CACHE_DIR:-/var/cache/ripple-env}"
BUNDLE_FILE=""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Action flags
DO_SETUP_REGISTRY=false
DO_IMPORT_IMAGES=false
DO_CONFIGURE_NIX=false
DO_CONFIGURE_PIXI=false
DO_ALL=true
DO_VERIFY=false

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            --cache-dir)
                CACHE_DIR="$2"
                shift 2
                ;;
            --bundle)
                BUNDLE_FILE="$2"
                shift 2
                ;;
            --setup-registry)
                DO_SETUP_REGISTRY=true
                DO_ALL=false
                shift
                ;;
            --import-images)
                DO_IMPORT_IMAGES=true
                DO_ALL=false
                shift
                ;;
            --configure-nix)
                DO_CONFIGURE_NIX=true
                DO_ALL=false
                shift
                ;;
            --configure-pixi)
                DO_CONFIGURE_PIXI=true
                DO_ALL=false
                shift
                ;;
            --all)
                DO_ALL=true
                shift
                ;;
            --verify)
                DO_VERIFY=true
                shift
                ;;
            --help)
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
}

show_help() {
    head -25 "$0" | tail -20 | sed 's/^# //' | sed 's/^#//'
}

# Extract offline bundle if provided
extract_bundle() {
    if [[ -n "$BUNDLE_FILE" ]]; then
        if [[ ! -f "$BUNDLE_FILE" ]]; then
            log_error "Bundle file not found: $BUNDLE_FILE"
            exit 1
        fi

        log_info "Extracting offline bundle: $BUNDLE_FILE"

        # Create cache directory parent
        mkdir -p "$(dirname "$CACHE_DIR")"

        # Extract bundle
        tar -xzf "$BUNDLE_FILE" -C "$(dirname "$CACHE_DIR")"

        log_success "Bundle extracted to $CACHE_DIR"
    fi
}

# Verify cache directory exists
verify_cache() {
    if [[ ! -d "$CACHE_DIR" ]]; then
        log_error "Cache directory not found: $CACHE_DIR"
        log_error "Please run prepare-offline.sh first or provide --bundle"
        exit 1
    fi

    log_info "Using cache directory: $CACHE_DIR"
}

# Start local Docker registry
setup_registry() {
    log_info "Setting up local Docker registry..."

    # Check if Docker is available
    if ! command -v docker &> /dev/null; then
        log_error "Docker is required but not installed"
        exit 1
    fi

    # Check if registry is already running
    if docker ps --format '{{.Names}}' | grep -q '^ripple-registry$'; then
        log_info "Local registry already running"
        return
    fi

    # Check if we have the registry image cached
    local registry_tar="$CACHE_DIR/docker-images/registry_2.tar"
    if [[ -f "$registry_tar" ]]; then
        log_info "Loading registry image from cache..."
        docker load -i "$registry_tar"
    else
        log_warn "Registry image not in cache, attempting to use existing..."
    fi

    # Start the registry
    docker run -d \
        --name ripple-registry \
        --restart always \
        -p 5000:5000 \
        -v "$CACHE_DIR/docker-registry:/var/lib/registry" \
        registry:2 || {
            log_warn "Failed to start registry, it may already be running"
        }

    log_success "Local registry running at localhost:5000"
}

# Import Docker images from cache
import_images() {
    log_info "Importing Docker images from cache..."

    local docker_cache="$CACHE_DIR/docker-images"

    if [[ ! -d "$docker_cache" ]]; then
        log_warn "Docker image cache not found: $docker_cache"
        return
    fi

    # Load each image tarball
    for tar_file in "$docker_cache"/*.tar; do
        if [[ -f "$tar_file" ]]; then
            log_info "Loading $(basename "$tar_file")..."
            if docker load -i "$tar_file"; then
                log_success "Loaded $(basename "$tar_file")"
            else
                log_warn "Failed to load $(basename "$tar_file")"
            fi
        fi
    done

    # Optionally push to local registry
    if docker ps --format '{{.Names}}' | grep -q '^ripple-registry$'; then
        log_info "Pushing images to local registry..."

        # Get list of loaded images
        local images
        images=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -v 'localhost:5000')

        for image in $images; do
            local local_image="localhost:5000/${image##*/}"
            log_info "Tagging and pushing $image -> $local_image"
            docker tag "$image" "$local_image" 2>/dev/null || true
            docker push "$local_image" 2>/dev/null || true
        done
    fi

    log_success "Docker images imported"
}

# Configure Nix for offline use
configure_nix() {
    log_info "Configuring Nix for offline use..."

    local nix_cache="$CACHE_DIR/nix-store"

    if [[ ! -d "$nix_cache" ]]; then
        log_warn "Nix cache not found: $nix_cache"
        return
    fi

    # Create/update nix.conf
    local nix_conf_dir="$HOME/.config/nix"
    mkdir -p "$nix_conf_dir"

    local nix_conf="$nix_conf_dir/nix.conf"

    # Backup existing config
    if [[ -f "$nix_conf" ]]; then
        cp "$nix_conf" "$nix_conf.bak.$(date +%s)"
    fi

    # Write offline configuration
    cat > "$nix_conf" << EOF
# Ripple-env offline mode configuration
# Generated by deploy-airgapped.sh on $(date)

# Use local binary cache only
substituters = file://$nix_cache

# Disable fallback to source builds (optional, remove if you want source builds)
# fallback = false

# Trust the local cache
require-sigs = false

# Disable network access hints
connect-timeout = 1

# Experimental features (for flakes)
experimental-features = nix-command flakes
EOF

    log_success "Nix configured for offline use"
    log_info "Configuration written to: $nix_conf"
}

# Configure Pixi for offline use
configure_pixi() {
    log_info "Configuring Pixi for offline use..."

    local conda_cache="$CACHE_DIR/conda-channels"

    # Create pixi config directory
    local pixi_conf_dir="$HOME/.config/pixi"
    mkdir -p "$pixi_conf_dir"

    local pixi_conf="$pixi_conf_dir/config.toml"

    # Backup existing config
    if [[ -f "$pixi_conf" ]]; then
        cp "$pixi_conf" "$pixi_conf.bak.$(date +%s)"
    fi

    # Write offline configuration
    cat > "$pixi_conf" << EOF
# Ripple-env offline mode configuration
# Generated by deploy-airgapped.sh on $(date)

[mirrors]
# Use local channel mirrors
"https://conda.anaconda.org/conda-forge" = ["file://$conda_cache/conda-forge"]
"https://conda.anaconda.org/robostack-humble" = ["file://$conda_cache/robostack-humble"]

[network]
# Disable network access
offline = true
EOF

    # Set environment variable
    export PIXI_OFFLINE=1

    log_success "Pixi configured for offline use"
    log_info "Configuration written to: $pixi_conf"
}

# Configure environment for offline use
configure_environment() {
    log_info "Configuring environment variables..."

    local env_file="$HOME/.config/ripple-env/offline.env"
    mkdir -p "$(dirname "$env_file")"

    cat > "$env_file" << EOF
# Ripple-env offline mode environment
# Source this file: source ~/.config/ripple-env/offline.env

# Enable offline mode
export RIPPLE_OFFLINE=1
export RIPPLE_CACHE_DIR="$CACHE_DIR"

# Nix offline
export NIX_CONFIG="substituters = file://$CACHE_DIR/nix-store"

# Pixi/Conda offline
export PIXI_OFFLINE=1
export CONDA_OFFLINE=1

# HuggingFace offline
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export HF_HOME="$CACHE_DIR/models/huggingface"

# Docker offline hints
export DOCKER_BUILDKIT=0

# Disable telemetry
export DO_NOT_TRACK=1
export PIXI_NO_UPDATE_CHECK=1
EOF

    log_success "Environment file created: $env_file"
    log_info "Add to shell: echo 'source $env_file' >> ~/.bashrc"
}

# Verify offline setup
verify_setup() {
    log_info "Verifying offline setup..."

    local errors=0

    # Check Nix cache
    if [[ -d "$CACHE_DIR/nix-store" ]]; then
        local store_paths
        store_paths=$(find "$CACHE_DIR/nix-store" -maxdepth 1 -name "*.narinfo" 2>/dev/null | wc -l)
        log_info "Nix cache: $store_paths packages available"
    else
        log_warn "Nix cache not found"
        ((errors++))
    fi

    # Check Docker images
    if [[ -d "$CACHE_DIR/docker-images" ]]; then
        local image_count
        image_count=$(find "$CACHE_DIR/docker-images" -name "*.tar" 2>/dev/null | wc -l)
        log_info "Docker images: $image_count images cached"
    else
        log_warn "Docker image cache not found"
        ((errors++))
    fi

    # Check Conda cache
    if [[ -d "$CACHE_DIR/conda-channels" ]]; then
        log_info "Conda channels: $(ls -1 "$CACHE_DIR/conda-channels" 2>/dev/null | wc -l) channels"
    else
        log_warn "Conda channel cache not found"
        ((errors++))
    fi

    # Check models
    if [[ -d "$CACHE_DIR/models" ]]; then
        local model_size
        model_size=$(du -sh "$CACHE_DIR/models" 2>/dev/null | cut -f1)
        log_info "Model cache: $model_size total"
    else
        log_warn "Model cache not found"
    fi

    # Check network connectivity (should fail in air-gapped)
    log_info "Testing network isolation..."
    if curl -s --connect-timeout 2 https://cache.nixos.org > /dev/null 2>&1; then
        log_warn "Network access detected - not truly air-gapped"
    else
        log_info "Network isolated (as expected for air-gapped)"
    fi

    # Summary
    if [[ $errors -eq 0 ]]; then
        log_success "Offline setup verification complete"
    else
        log_warn "Verification completed with $errors warnings"
    fi
}

# Main execution
main() {
    parse_args "$@"

    log_info "Deploying ripple-env in air-gapped mode"

    # Extract bundle if provided
    extract_bundle

    # Verify cache exists
    verify_cache

    if $DO_ALL || $DO_SETUP_REGISTRY; then
        setup_registry
    fi

    if $DO_ALL || $DO_IMPORT_IMAGES; then
        import_images
    fi

    if $DO_ALL || $DO_CONFIGURE_NIX; then
        configure_nix
    fi

    if $DO_ALL || $DO_CONFIGURE_PIXI; then
        configure_pixi
    fi

    if $DO_ALL; then
        configure_environment
    fi

    if $DO_VERIFY || $DO_ALL; then
        verify_setup
    fi

    log_success "Air-gapped deployment complete!"
    log_info ""
    log_info "To activate offline mode:"
    log_info "  source ~/.config/ripple-env/offline.env"
    log_info ""
    log_info "Then start development:"
    log_info "  cd $PROJECT_ROOT"
    log_info "  nix develop"
}

main "$@"
