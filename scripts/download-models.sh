#!/usr/bin/env bash
# =============================================================================
# LocalAI Model Download Script
# =============================================================================
# BUILDKIT_STARTER_SPEC.md Layer 10: Multi-Model Inference
#
# Downloads GGUF models for LocalAI MOE (Multi-Model Fan-Out) inference.
# Models are defined in manifests/distributed/inference_policy.yaml
#
# Usage:
#   ./scripts/download-models.sh           # Download all required models
#   ./scripts/download-models.sh --all     # Include optional models
#   ./scripts/download-models.sh --list    # List models without downloading
#   ./scripts/download-models.sh --verify  # Verify existing models
#
# Requirements:
#   - curl or wget
#   - ~15GB disk space for required models
#   - ~20GB disk space for all models (including optional)
# =============================================================================

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODELS_DIR="${PROJECT_ROOT}/data/localai/models"
HF_BASE_URL="https://huggingface.co"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Model definitions from inference_policy.yaml
# Format: name|filename|repo|size_bytes|required
declare -a MODELS=(
    # Required models (min_parallel: 5)
    "gemma-3n-E2B-it|gemma-3n-E2B-it-UD-Q4_K_XL.gguf|unsloth/gemma-3n-E2B-it-GGUF|2100000000|true"
    "Phi-4-mini-reasoning|Phi-4-mini-reasoning-UD-Q4_K_XL.gguf|unsloth/Phi-4-mini-reasoning-GGUF|2500000000|true"
    "DeepSeek-R1-Distill-Qwen-1.5B|DeepSeek-R1-Distill-Qwen-1.5B-Q8_0.gguf|unsloth/DeepSeek-R1-Distill-Qwen-1.5B-GGUF|1600000000|true"
    "gemma-3-4b-it-qat|gemma-3-4b-it-qat-UD-Q4_K_XL.gguf|unsloth/gemma-3-4b-it-qat-GGUF|2800000000|true"
    "Qwen3-4B|Qwen3-4B-Q4_K_M.gguf|unsloth/Qwen3-4B-GGUF|2700000000|true"
    # Optional models
    "gemma-3-1b-it|gemma-3-1b-it-BF16.gguf|unsloth/gemma-3-1b-it-GGUF|2000000000|false"
    "Qwen3-0.6B|Qwen3-0.6B-BF16.gguf|unsloth/Qwen3-0.6B-GGUF|1200000000|false"
)

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $*"; }
log_success() { echo -e "${GREEN}[OK]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# Print usage
usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Download GGUF models for LocalAI MOE inference.

Options:
    --all       Download all models (including optional)
    --list      List models without downloading
    --verify    Verify existing models
    --clean     Remove all downloaded models
    --help      Show this help message

Models Directory: ${MODELS_DIR}

Required Models (5):
    - gemma-3n-E2B-it         General purpose
    - Phi-4-mini-reasoning    Reasoning specialist
    - DeepSeek-R1-Distill     Code specialist
    - gemma-3-4b-it-qat       General quantized
    - Qwen3-4B                Code/General

Optional Models (2):
    - gemma-3-1b-it           Lightweight general
    - Qwen3-0.6B              Lightweight general
EOF
}

# Format bytes to human readable
format_size() {
    local size=$1
    if ((size >= 1073741824)); then
        echo "$(echo "scale=1; $size/1073741824" | bc)GB"
    elif ((size >= 1048576)); then
        echo "$(echo "scale=1; $size/1048576" | bc)MB"
    else
        echo "${size}B"
    fi
}

# Check for download tool
get_download_cmd() {
    if command -v curl &>/dev/null; then
        echo "curl"
    elif command -v wget &>/dev/null; then
        echo "wget"
    else
        log_error "Neither curl nor wget found. Please install one of them."
        exit 1
    fi
}

# Download a file with progress
download_file() {
    local url=$1
    local output=$2
    local download_cmd
    download_cmd=$(get_download_cmd)

    if [[ "$download_cmd" == "curl" ]]; then
        curl -L --progress-bar -o "$output" "$url"
    else
        wget --progress=bar:force -O "$output" "$url"
    fi
}

# List all models
list_models() {
    echo ""
    echo "LocalAI MOE Models"
    echo "=================="
    echo ""
    printf "%-30s %-45s %-10s %-10s\n" "Name" "Filename" "Size" "Required"
    printf "%-30s %-45s %-10s %-10s\n" "----" "--------" "----" "--------"

    for model_def in "${MODELS[@]}"; do
        IFS='|' read -r name filename repo size required <<< "$model_def"
        local size_human
        size_human=$(format_size "$size")
        local req_str="No"
        [[ "$required" == "true" ]] && req_str="Yes"

        # Check if already downloaded
        local status=""
        if [[ -f "${MODELS_DIR}/${filename}" ]]; then
            status=" [Downloaded]"
        fi

        printf "%-30s %-45s %-10s %-10s%s\n" "$name" "$filename" "$size_human" "$req_str" "$status"
    done
    echo ""
}

# Verify downloaded models
verify_models() {
    log_info "Verifying downloaded models..."
    echo ""

    local required_count=0
    local required_found=0
    local optional_count=0
    local optional_found=0

    for model_def in "${MODELS[@]}"; do
        IFS='|' read -r name filename repo size required <<< "$model_def"
        local filepath="${MODELS_DIR}/${filename}"

        if [[ "$required" == "true" ]]; then
            ((required_count++))
        else
            ((optional_count++))
        fi

        if [[ -f "$filepath" ]]; then
            local actual_size
            actual_size=$(stat -c%s "$filepath" 2>/dev/null || stat -f%z "$filepath" 2>/dev/null)

            # Allow 10% variance in file size (HF files may vary)
            local min_size=$((size * 9 / 10))

            if ((actual_size >= min_size)); then
                log_success "$name: $(format_size "$actual_size")"
                if [[ "$required" == "true" ]]; then
                    ((required_found++))
                else
                    ((optional_found++))
                fi
            else
                log_warn "$name: Size mismatch (expected ~$(format_size "$size"), got $(format_size "$actual_size"))"
            fi
        else
            if [[ "$required" == "true" ]]; then
                log_error "$name: Not found (REQUIRED)"
            else
                log_warn "$name: Not found (optional)"
            fi
        fi
    done

    echo ""
    echo "Summary:"
    echo "  Required: ${required_found}/${required_count}"
    echo "  Optional: ${optional_found}/${optional_count}"

    if ((required_found >= 5)); then
        log_success "MOE minimum model requirement satisfied (5+ models)"
        return 0
    else
        log_error "MOE requires at least 5 models. Please download more."
        return 1
    fi
}

# Download models
download_models() {
    local include_optional=${1:-false}

    # Create models directory
    mkdir -p "$MODELS_DIR"

    log_info "Models directory: ${MODELS_DIR}"
    log_info "Include optional: ${include_optional}"
    echo ""

    local downloaded=0
    local skipped=0
    local failed=0

    for model_def in "${MODELS[@]}"; do
        IFS='|' read -r name filename repo size required <<< "$model_def"

        # Skip optional models unless --all flag
        if [[ "$required" == "false" && "$include_optional" == "false" ]]; then
            log_info "Skipping optional model: $name"
            continue
        fi

        local filepath="${MODELS_DIR}/${filename}"
        local url="${HF_BASE_URL}/${repo}/resolve/main/${filename}"

        # Skip if already exists
        if [[ -f "$filepath" ]]; then
            local actual_size
            actual_size=$(stat -c%s "$filepath" 2>/dev/null || stat -f%z "$filepath" 2>/dev/null)
            log_success "Already exists: $name ($(format_size "$actual_size"))"
            ((skipped++))
            continue
        fi

        echo ""
        log_info "Downloading: $name"
        log_info "  Source: $url"
        log_info "  Target: $filepath"
        log_info "  Expected size: $(format_size "$size")"
        echo ""

        # Download to temp file first
        local temp_file="${filepath}.download"

        if download_file "$url" "$temp_file"; then
            mv "$temp_file" "$filepath"
            log_success "Downloaded: $name"
            ((downloaded++))
        else
            log_error "Failed to download: $name"
            rm -f "$temp_file"
            ((failed++))
        fi
    done

    echo ""
    echo "Download Summary:"
    echo "  Downloaded: $downloaded"
    echo "  Skipped (existing): $skipped"
    echo "  Failed: $failed"

    if ((failed > 0)); then
        log_error "Some downloads failed. Re-run script to retry."
        return 1
    fi

    return 0
}

# Clean all models
clean_models() {
    log_warn "This will remove all models from ${MODELS_DIR}"
    read -p "Are you sure? [y/N] " -n 1 -r
    echo ""

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "${MODELS_DIR:?}"
        log_success "Models directory cleaned"
    else
        log_info "Cancelled"
    fi
}

# Main
main() {
    local action="download"
    local include_optional=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --all)
                include_optional=true
                shift
                ;;
            --list)
                action="list"
                shift
                ;;
            --verify)
                action="verify"
                shift
                ;;
            --clean)
                action="clean"
                shift
                ;;
            --help|-h)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done

    echo ""
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║           LocalAI Model Download Script                        ║"
    echo "║           BUILDKIT_STARTER_SPEC.md Layer 10                    ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo ""

    case $action in
        list)
            list_models
            ;;
        verify)
            verify_models
            ;;
        clean)
            clean_models
            ;;
        download)
            download_models "$include_optional"
            echo ""
            verify_models
            ;;
    esac
}

main "$@"
