#!/usr/bin/env bash
# wsl-cleanup.sh - Disk cleanup utility for WSL2/NixOS environment
#
# Usage:
#   ./scripts/wsl-cleanup.sh [OPTIONS]
#
# Options:
#   --all         Run all cleanup operations
#   --nix         Clean Nix store only
#   --docker      Clean Docker only
#   --pixi        Clean Pixi only
#   --build       Clean build artifacts only
#   --aggressive  More aggressive cleanup (removes all generations)
#   --dry-run     Show what would be cleaned without doing it
#   --report      Show disk usage report only
#   -h, --help    Show this help message

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default options
CLEAN_NIX=false
CLEAN_DOCKER=false
CLEAN_PIXI=false
CLEAN_BUILD=false
AGGRESSIVE=false
DRY_RUN=false
REPORT_ONLY=false

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

show_help() {
    head -20 "$0" | tail -17 | sed 's/^# //' | sed 's/^#//'
    exit 0
}

show_disk_report() {
    echo ""
    echo "==========================================="
    echo "        WSL2 Disk Usage Report            "
    echo "==========================================="
    echo ""

    echo "Filesystem Overview:"
    df -h / 2>/dev/null | tail -1 | awk '{print "  Total: " $2 ", Used: " $3 " (" $5 "), Available: " $4}'
    echo ""

    echo "Component Breakdown:"
    printf "  %-25s %s\n" "Nix Store:" "$(du -sh /nix/store 2>/dev/null | cut -f1 || echo 'N/A')"
    printf "  %-25s %s\n" "Pixi Cache:" "$(du -sh ~/.pixi 2>/dev/null | cut -f1 || echo 'N/A')"
    printf "  %-25s %s\n" "Home Directory:" "$(du -sh ~ 2>/dev/null | cut -f1 || echo 'N/A')"

    if command -v docker &>/dev/null && docker info &>/dev/null 2>&1; then
        echo ""
        echo "Docker Usage:"
        docker system df 2>/dev/null | sed 's/^/  /' || echo "  Docker not available"
    fi

    # Check for build directories
    if [ -d "$HOME/ripple-env/build" ]; then
        printf "  %-25s %s\n" "Build Artifacts:" "$(du -sh ~/ripple-env/build 2>/dev/null | cut -f1 || echo 'N/A')"
    fi

    # Check for AI models
    if [ -d "$HOME/models" ]; then
        printf "  %-25s %s\n" "AI Models:" "$(du -sh ~/models 2>/dev/null | cut -f1 || echo 'N/A')"
    fi

    echo ""
    echo "Largest Directories (top 10):"
    du -h ~/* 2>/dev/null | sort -hr | head -10 | sed 's/^/  /'

    echo ""
}

clean_nix() {
    log_info "Cleaning Nix store..."

    if [ "$DRY_RUN" = true ]; then
        log_info "[DRY-RUN] Would run: nix store gc"
        nix store gc --dry-run 2>/dev/null || true
        return
    fi

    # Get size before
    local before
    before=$(du -sb /nix/store 2>/dev/null | cut -f1 || echo 0)

    if [ "$AGGRESSIVE" = true ]; then
        log_info "Running aggressive cleanup (removing all old generations)..."
        nix-collect-garbage -d 2>/dev/null || true
    else
        log_info "Running standard garbage collection..."
        nix store gc 2>/dev/null || true
    fi

    # Optimize store (deduplicate)
    log_info "Optimizing Nix store..."
    nix store optimise 2>/dev/null || true

    # Get size after
    local after
    after=$(du -sb /nix/store 2>/dev/null | cut -f1 || echo 0)
    local freed=$((before - after))
    local freed_mb=$((freed / 1024 / 1024))

    if [ $freed_mb -gt 0 ]; then
        log_success "Freed ${freed_mb} MB from Nix store"
    else
        log_info "Nix store already optimized"
    fi
}

clean_docker() {
    if ! command -v docker &>/dev/null; then
        log_warning "Docker not installed, skipping..."
        return
    fi

    if ! docker info &>/dev/null 2>&1; then
        log_warning "Docker daemon not running, skipping..."
        return
    fi

    log_info "Cleaning Docker..."

    if [ "$DRY_RUN" = true ]; then
        log_info "[DRY-RUN] Would run: docker system prune -a -f"
        docker system df
        return
    fi

    # Get size before
    local before
    before=$(docker system df --format '{{.Size}}' 2>/dev/null | head -1 || echo "0B")

    if [ "$AGGRESSIVE" = true ]; then
        log_info "Running aggressive Docker cleanup (including volumes)..."
        docker system prune -a -f --volumes 2>/dev/null || true
    else
        log_info "Running standard Docker cleanup..."
        docker system prune -a -f 2>/dev/null || true
    fi

    local after
    after=$(docker system df --format '{{.Size}}' 2>/dev/null | head -1 || echo "0B")
    log_success "Docker cleaned. Before: $before, After: $after"
}

clean_pixi() {
    if ! command -v pixi &>/dev/null; then
        log_warning "Pixi not installed, skipping..."
        return
    fi

    log_info "Cleaning Pixi cache..."

    if [ "$DRY_RUN" = true ]; then
        log_info "[DRY-RUN] Would run: pixi clean"
        du -sh ~/.pixi 2>/dev/null || true
        return
    fi

    local before
    before=$(du -sb ~/.pixi 2>/dev/null | cut -f1 || echo 0)

    pixi clean 2>/dev/null || true

    local after
    after=$(du -sb ~/.pixi 2>/dev/null | cut -f1 || echo 0)
    local freed=$((before - after))
    local freed_mb=$((freed / 1024 / 1024))

    if [ $freed_mb -gt 0 ]; then
        log_success "Freed ${freed_mb} MB from Pixi cache"
    else
        log_info "Pixi cache already clean"
    fi
}

clean_build() {
    log_info "Cleaning build artifacts..."

    local freed_total=0

    # Clean ROS2/colcon build directories
    local build_dirs=(
        "$HOME/ripple-env/build"
        "$HOME/ripple-env/install"
        "$HOME/ripple-env/log"
    )

    for dir in "${build_dirs[@]}"; do
        if [ -d "$dir" ]; then
            local size
            size=$(du -sb "$dir" 2>/dev/null | cut -f1 || echo 0)
            if [ "$DRY_RUN" = true ]; then
                log_info "[DRY-RUN] Would remove: $dir ($(du -sh "$dir" | cut -f1))"
            else
                rm -rf "$dir"
                log_info "Removed: $dir"
                freed_total=$((freed_total + size))
            fi
        fi
    done

    # Clean ccache if it exists
    if command -v ccache &>/dev/null; then
        if [ "$DRY_RUN" = true ]; then
            log_info "[DRY-RUN] Would clear ccache"
            ccache -s 2>/dev/null | grep "cache size" || true
        else
            ccache --clear 2>/dev/null || true
            log_info "Cleared ccache"
        fi
    fi

    if [ "$DRY_RUN" = false ]; then
        local freed_mb
        freed_mb=$((freed_total / 1024 / 1024))
        if [ $freed_mb -gt 0 ]; then
            log_success "Freed ${freed_mb} MB from build artifacts"
        fi
    fi
}

run_all_cleanup() {
    local start_free
    start_free=$(df / 2>/dev/null | tail -1 | awk '{print $4}')

    clean_nix
    clean_docker
    clean_pixi
    clean_build

    local end_free
    end_free=$(df / 2>/dev/null | tail -1 | awk '{print $4}')
    local freed=$((end_free - start_free))
    local freed_mb=$((freed / 1024))

    echo ""
    if [ "$DRY_RUN" = false ] && [ $freed_mb -gt 0 ]; then
        log_success "Total freed: ${freed_mb} MB"
    fi
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --all)
            CLEAN_NIX=true
            CLEAN_DOCKER=true
            CLEAN_PIXI=true
            CLEAN_BUILD=true
            shift
            ;;
        --nix)
            CLEAN_NIX=true
            shift
            ;;
        --docker)
            CLEAN_DOCKER=true
            shift
            ;;
        --pixi)
            CLEAN_PIXI=true
            shift
            ;;
        --build)
            CLEAN_BUILD=true
            shift
            ;;
        --aggressive)
            AGGRESSIVE=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --report)
            REPORT_ONLY=true
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            log_error "Unknown option: $1"
            show_help
            ;;
    esac
done

# Main execution
echo ""
echo "==========================================="
echo "     WSL2 Disk Cleanup Utility            "
echo "==========================================="

if [ "$DRY_RUN" = true ]; then
    log_warning "DRY-RUN MODE - No changes will be made"
fi

if [ "$REPORT_ONLY" = true ]; then
    show_disk_report
    exit 0
fi

# If no specific option selected, show report and prompt
if [ "$CLEAN_NIX" = false ] && [ "$CLEAN_DOCKER" = false ] && \
   [ "$CLEAN_PIXI" = false ] && [ "$CLEAN_BUILD" = false ]; then
    show_disk_report
    echo ""
    log_info "No cleanup options specified. Use --all for full cleanup or -h for help."
    exit 0
fi

# Run selected cleanups
if [ "$CLEAN_NIX" = true ]; then clean_nix; fi
if [ "$CLEAN_DOCKER" = true ]; then clean_docker; fi
if [ "$CLEAN_PIXI" = true ]; then clean_pixi; fi
if [ "$CLEAN_BUILD" = true ]; then clean_build; fi

echo ""
show_disk_report
log_success "Cleanup complete!"
