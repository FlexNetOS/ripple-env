#!/usr/bin/env bash
# prepare-offline.sh - Prepare packages for offline/air-gapped installation
#
# This script downloads all required packages and stores them locally
# for later offline use. Run this on a machine with internet access,
# then transfer the cache to your air-gapped environment.
#
# Usage:
#   ./scripts/prepare-offline.sh [OPTIONS]
#
# Options:
#   --cache-dir DIR    Base cache directory (default: /var/cache/ripple-env)
#   --nix              Prepare Nix packages only
#   --pixi             Prepare Pixi/Conda packages only
#   --docker           Prepare Docker images only
#   --models           Prepare AI models only
#   --all              Prepare everything (default)
#   --minimal          Minimal packages for basic operation
#   --full             Full packages including all optional components
#   --help             Show this help message

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default configuration
CACHE_DIR="${RIPPLE_CACHE_DIR:-/var/cache/ripple-env}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Component flags
PREPARE_NIX=false
PREPARE_PIXI=false
PREPARE_DOCKER=false
PREPARE_MODELS=false
PREPARE_ALL=true
PACKAGE_SET="minimal"

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
            --nix)
                PREPARE_NIX=true
                PREPARE_ALL=false
                shift
                ;;
            --pixi)
                PREPARE_PIXI=true
                PREPARE_ALL=false
                shift
                ;;
            --docker)
                PREPARE_DOCKER=true
                PREPARE_ALL=false
                shift
                ;;
            --models)
                PREPARE_MODELS=true
                PREPARE_ALL=false
                shift
                ;;
            --all)
                PREPARE_ALL=true
                shift
                ;;
            --minimal)
                PACKAGE_SET="minimal"
                shift
                ;;
            --full)
                PACKAGE_SET="full"
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
    head -30 "$0" | tail -25 | sed 's/^# //' | sed 's/^#//'
}

# Create cache directory structure
setup_cache_dirs() {
    log_info "Setting up cache directories in $CACHE_DIR"

    mkdir -p "$CACHE_DIR"/{nix-store,conda-channels,docker-images,models,git-mirrors,git-bundles}

    # Set permissions for shared use
    chmod -R 755 "$CACHE_DIR"

    log_success "Cache directories created"
}

# Prepare Nix packages
prepare_nix() {
    log_info "Preparing Nix packages..."

    local nix_cache="$CACHE_DIR/nix-store"

    # Build the devshell and copy closure to cache
    log_info "Building devshell closure..."
    nix build "$PROJECT_ROOT#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').default" \
        --out-link "$CACHE_DIR/result-devshell"

    # Copy the closure to local cache
    log_info "Copying Nix store closure to cache..."
    nix copy --to "file://$nix_cache" "$CACHE_DIR/result-devshell"

    # Build additional shells based on package set
    if [[ "$PACKAGE_SET" == "full" ]]; then
        for shell in minimal ros2 identity; do
            log_info "Building $shell shell..."
            if nix build "$PROJECT_ROOT#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').$shell" \
                --out-link "$CACHE_DIR/result-$shell" 2>/dev/null; then
                nix copy --to "file://$nix_cache" "$CACHE_DIR/result-$shell"
            else
                log_warn "Shell $shell not available, skipping"
            fi
        done
    fi

    # Generate nix-cache-info
    cat > "$nix_cache/nix-cache-info" << EOF
StoreDir: /nix/store
WantMassQuery: 1
Priority: 30
EOF

    # Clean up result links
    rm -f "$CACHE_DIR"/result-*

    log_success "Nix packages prepared in $nix_cache"
}

# Prepare Pixi/Conda packages
prepare_pixi() {
    log_info "Preparing Pixi/Conda packages..."

    local conda_cache="$CACHE_DIR/conda-channels"

    # Check if pixi is available
    if ! command -v pixi &> /dev/null; then
        log_warn "pixi not found, skipping Pixi package preparation"
        return
    fi

    # Install packages to populate cache
    log_info "Installing Pixi packages to populate cache..."
    cd "$PROJECT_ROOT"
    pixi install

    # Get pixi cache location
    local pixi_cache_dir
    pixi_cache_dir=$(pixi info --json 2>/dev/null | jq -r '.cache_dir // empty' || echo "$HOME/.cache/rattler")

    # Copy conda packages to our cache
    if [[ -d "$pixi_cache_dir" ]]; then
        log_info "Copying Pixi cache to $conda_cache..."
        cp -r "$pixi_cache_dir"/* "$conda_cache/" 2>/dev/null || true
    fi

    # Create channel index files
    for channel in conda-forge robostack-humble; do
        mkdir -p "$conda_cache/$channel"
        log_info "Channel $channel prepared"
    done

    log_success "Pixi/Conda packages prepared in $conda_cache"
}

# Docker images to cache
DOCKER_IMAGES_MINIMAL=(
    "redis:7-alpine"
    "postgres:17.2-alpine"
    "nats:2.10.24-alpine"
)

DOCKER_IMAGES_FULL=(
    "${DOCKER_IMAGES_MINIMAL[@]}"
    "localai/localai:v2.24.2-aio-cpu"
    "temporalio/auto-setup:1.26.2"
    "temporalio/ui:2.34.0"
    "minio/minio:RELEASE.2025-09-07T16-13-09Z"
    "grafana/grafana:11.4.0"
    "prom/prometheus:v2.48.0"
    "hashicorp/vault:1.18"
    "quay.io/keycloak/keycloak:26.0"
    "neo4j:5-community"
    "kong:3.9.0-alpine"
    "nginx:1.27.3-alpine"
)

# Prepare Docker images
prepare_docker() {
    log_info "Preparing Docker images..."

    local docker_cache="$CACHE_DIR/docker-images"

    # Check if docker is available
    if ! command -v docker &> /dev/null; then
        log_warn "docker not found, skipping Docker image preparation"
        return
    fi

    # Select image list based on package set
    local -a images
    if [[ "$PACKAGE_SET" == "full" ]]; then
        images=("${DOCKER_IMAGES_FULL[@]}")
    else
        images=("${DOCKER_IMAGES_MINIMAL[@]}")
    fi

    # Pull and save each image
    for image in "${images[@]}"; do
        log_info "Pulling $image..."
        if docker pull "$image"; then
            # Create safe filename
            local filename
            filename=$(echo "$image" | tr '/:' '_')
            log_info "Saving $image to $docker_cache/$filename.tar..."
            docker save "$image" -o "$docker_cache/$filename.tar"
            log_success "Saved $image"
        else
            log_warn "Failed to pull $image, skipping"
        fi
    done

    # Create manifest file
    log_info "Creating image manifest..."
    cat > "$docker_cache/manifest.json" << EOF
{
  "created": "$(date -Iseconds)",
  "package_set": "$PACKAGE_SET",
  "images": [
$(printf '    "%s",\n' "${images[@]}" | sed '$ s/,$//')
  ]
}
EOF

    log_success "Docker images prepared in $docker_cache"
}

# AI Models to cache
AI_MODELS_MINIMAL=()

AI_MODELS_FULL=(
    "https://huggingface.co/TheBloke/Mistral-7B-Instruct-v0.2-GGUF/resolve/main/mistral-7b-instruct-v0.2.Q4_K_M.gguf"
)

# Prepare AI models
prepare_models() {
    log_info "Preparing AI models..."

    local models_cache="$CACHE_DIR/models"
    mkdir -p "$models_cache"/{huggingface,localai}

    # Select model list based on package set
    local -a models
    if [[ "$PACKAGE_SET" == "full" ]]; then
        models=("${AI_MODELS_FULL[@]}")
    else
        models=("${AI_MODELS_MINIMAL[@]}")
    fi

    if [[ ${#models[@]} -eq 0 ]]; then
        log_info "No models to download for $PACKAGE_SET package set"
        log_success "Models cache prepared (empty)"
        return
    fi

    # Download each model
    for model_url in "${models[@]}"; do
        local filename
        filename=$(basename "$model_url")
        local dest="$models_cache/localai/$filename"

        if [[ -f "$dest" ]]; then
            log_info "Model $filename already exists, skipping"
            continue
        fi

        log_info "Downloading $filename..."
        if curl -L -o "$dest" "$model_url"; then
            log_success "Downloaded $filename"
        else
            log_warn "Failed to download $filename"
        fi
    done

    # Create model manifest
    cat > "$models_cache/manifest.json" << EOF
{
  "created": "$(date -Iseconds)",
  "package_set": "$PACKAGE_SET",
  "models": [
$(printf '    "%s",\n' "${models[@]}" | sed '$ s/,$//')
  ]
}
EOF

    log_success "AI models prepared in $models_cache"
}

# Create offline bundle archive
create_bundle() {
    log_info "Creating offline bundle archive..."

    local bundle_name
    bundle_name="ripple-env-offline-${PACKAGE_SET}-$(date +%Y%m%d).tar.gz"
    local bundle_path="$PROJECT_ROOT/$bundle_name"

    # Create archive (excluding large model files if minimal)
    if [[ "$PACKAGE_SET" == "minimal" ]]; then
        tar -czf "$bundle_path" \
            -C "$(dirname "$CACHE_DIR")" \
            --exclude='*.gguf' \
            --exclude='*.tar' \
            "$(basename "$CACHE_DIR")"
    else
        tar -czf "$bundle_path" \
            -C "$(dirname "$CACHE_DIR")" \
            "$(basename "$CACHE_DIR")"
    fi

    log_success "Offline bundle created: $bundle_path"
    log_info "Bundle size: $(du -h "$bundle_path" | cut -f1)"
}

# Main execution
main() {
    parse_args "$@"

    log_info "Preparing offline packages for ripple-env"
    log_info "Cache directory: $CACHE_DIR"
    log_info "Package set: $PACKAGE_SET"

    setup_cache_dirs

    if $PREPARE_ALL || $PREPARE_NIX; then
        prepare_nix
    fi

    if $PREPARE_ALL || $PREPARE_PIXI; then
        prepare_pixi
    fi

    if $PREPARE_ALL || $PREPARE_DOCKER; then
        prepare_docker
    fi

    if $PREPARE_ALL || $PREPARE_MODELS; then
        prepare_models
    fi

    # Create bundle if preparing all
    if $PREPARE_ALL; then
        create_bundle
    fi

    log_success "Offline preparation complete!"
    log_info ""
    log_info "Next steps:"
    log_info "  1. Transfer $CACHE_DIR to your air-gapped machine"
    log_info "  2. Set environment: export RIPPLE_OFFLINE=1"
    log_info "  3. Configure local caches (see config/offline.yaml)"
    log_info "  4. Run: nix develop"
}

main "$@"
