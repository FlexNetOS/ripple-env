#!/usr/bin/env bash
# upgrade-python-deps.sh - Python dependency upgrade automation
#
# This script helps upgrade Python versions and their coupled dependencies
# while maintaining compatibility constraints.
#
# Usage:
#   ./scripts/upgrade-python-deps.sh [command] [options]
#
# Commands:
#   check        Check for available upgrades
#   pytorch      Upgrade PyTorch ecosystem
#   python       Show Python upgrade paths
#   all          Check all upgrade possibilities
#
# Examples:
#   ./scripts/upgrade-python-deps.sh check
#   ./scripts/upgrade-python-deps.sh pytorch 2.6
#   ./scripts/upgrade-python-deps.sh python

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

PIXI_TOML="pixi.toml"

info() { echo -e "${BLUE}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; }
success() { echo -e "${GREEN}[OK]${NC} $*"; }

# PyTorch version coupling table
declare -A PYTORCH_COUPLING=(
    ["2.6"]="torchvision:0.21 torchaudio:2.6"
    ["2.5"]="torchvision:0.20 torchaudio:2.5"
    ["2.4"]="torchvision:0.19 torchaudio:2.4"
    ["2.3"]="torchvision:0.18 torchaudio:2.3"
    ["2.2"]="torchvision:0.17 torchaudio:2.2"
    ["2.1"]="torchvision:0.16 torchaudio:2.1"
)

# Python compatibility constraints
declare -A PYTHON_CONSTRAINTS=(
    ["robostack-humble"]="3.11"
    ["aios"]="3.10-3.11"
    ["pytorch"]="3.9-3.12"
    ["transformers"]="3.9-3.12"
)

# Check if pixi.toml exists
check_pixi_toml() {
    if [[ ! -f "$PIXI_TOML" ]]; then
        error "pixi.toml not found in current directory"
        exit 1
    fi
}

# Get current versions from pixi.toml
get_current_versions() {
    info "Reading current versions from pixi.toml..."

    # Extract Python version constraint
    PYTHON_VER=$(grep -E '^python\s*=' "$PIXI_TOML" | head -1 | sed 's/.*"\(.*\)".*/\1/' || echo "unknown")
    echo "  Python: $PYTHON_VER"

    # Extract PyTorch version
    PYTORCH_VER=$(grep -E '^pytorch\s*=' "$PIXI_TOML" | head -1 | grep -oP '>=\K[0-9]+\.[0-9]+' || echo "unknown")
    echo "  PyTorch: $PYTORCH_VER"

    # Extract TorchVision version
    TORCHVISION_VER=$(grep -E '^torchvision\s*=' "$PIXI_TOML" | head -1 | grep -oP '>=\K[0-9]+\.[0-9]+' || echo "unknown")
    echo "  TorchVision: $TORCHVISION_VER"

    # Extract TorchAudio version
    TORCHAUDIO_VER=$(grep -E '^torchaudio\s*=' "$PIXI_TOML" | head -1 | grep -oP '>=\K[0-9]+\.[0-9]+' || echo "unknown")
    echo "  TorchAudio: $TORCHAUDIO_VER"
}

# Check for available PyTorch upgrades
check_pytorch_upgrades() {
    info "Checking PyTorch upgrade possibilities..."

    echo ""
    echo "Current PyTorch ecosystem:"
    get_current_versions

    echo ""
    echo "Available PyTorch versions with coupling requirements:"
    echo ""
    printf "%-10s %-15s %-15s %-20s\n" "PyTorch" "TorchVision" "TorchAudio" "Status"
    printf "%s\n" "----------------------------------------------------------------"

    for version in $(echo "${!PYTORCH_COUPLING[@]}" | tr ' ' '\n' | sort -rV); do
        local deps="${PYTORCH_COUPLING[$version]}"
        local tv_ver=$(echo "$deps" | grep -oP 'torchvision:\K[0-9.]+')
        local ta_ver=$(echo "$deps" | grep -oP 'torchaudio:\K[0-9.]+')

        local status=""
        if [[ "$version" == "$PYTORCH_VER" ]]; then
            status="${GREEN}(current)${NC}"
        elif [[ $(echo -e "$version\n$PYTORCH_VER" | sort -V | head -1) == "$PYTORCH_VER" ]]; then
            status="${CYAN}(upgrade available)${NC}"
        else
            status="${YELLOW}(downgrade)${NC}"
        fi

        printf "%-10s %-15s %-15s " "$version.x" "$tv_ver.x" "$ta_ver.x"
        echo -e "$status"
    done
}

# Generate upgrade commands for PyTorch
suggest_pytorch_upgrade() {
    local target_version="${1:-}"

    if [[ -z "$target_version" ]]; then
        echo ""
        echo "Usage: $0 pytorch <version>"
        echo ""
        echo "Available versions:"
        for v in $(echo "${!PYTORCH_COUPLING[@]}" | tr ' ' '\n' | sort -rV); do
            echo "  $v"
        done
        return 1
    fi

    if [[ -z "${PYTORCH_COUPLING[$target_version]:-}" ]]; then
        error "Unknown PyTorch version: $target_version"
        echo "Available: ${!PYTORCH_COUPLING[*]}"
        return 1
    fi

    local deps="${PYTORCH_COUPLING[$target_version]}"
    local tv_ver=$(echo "$deps" | grep -oP 'torchvision:\K[0-9.]+')
    local ta_ver=$(echo "$deps" | grep -oP 'torchaudio:\K[0-9.]+')

    echo ""
    info "Upgrade instructions for PyTorch $target_version"
    echo ""
    echo "1. Update pixi.toml with these versions:"
    echo ""
    echo -e "${CYAN}# PyPI dependencies section:${NC}"
    echo "pytorch = \">=$target_version,<$(echo "$target_version + 0.1" | bc)\""
    echo "torchvision = \">=$tv_ver,<$(echo "$tv_ver + 0.01" | bc)\""
    echo "torchaudio = \">=$ta_ver,<$(echo "$ta_ver + 0.1" | bc)\""
    echo ""
    echo -e "${CYAN}# CUDA feature section (if applicable):${NC}"
    echo "pytorch = { version = \">=$target_version,<$(echo "$target_version + 0.1" | bc)\", channel = \"pytorch\", build = \"*cuda*\" }"
    echo "torchvision = { version = \">=$tv_ver,<$(echo "$tv_ver + 0.01" | bc)\", channel = \"pytorch\", build = \"*cuda*\" }"
    echo "torchaudio = { version = \">=$ta_ver,<$(echo "$ta_ver + 0.1" | bc)\", channel = \"pytorch\", build = \"*cuda*\" }"
    echo ""
    echo "2. Update lock file:"
    echo "   pixi update"
    echo ""
    echo "3. Verify installation:"
    echo "   pixi run python -c \"import torch; print(torch.__version__)\""
    echo "   ./scripts/check-python-deps.sh"
}

# Show Python upgrade paths
show_python_paths() {
    echo ""
    info "Python Version Upgrade Paths"
    echo ""
    echo "Current constraints by component:"
    echo ""
    printf "%-20s %-20s %-30s\n" "Component" "Python Version" "Notes"
    printf "%s\n" "----------------------------------------------------------------------"
    printf "%-20s %-20s %-30s\n" "RoboStack Humble" "3.11.x only" "conda-forge constraint"
    printf "%-20s %-20s %-30s\n" "AIOS" "3.10-3.11" "No 3.12+ support"
    printf "%-20s %-20s %-30s\n" "PyTorch" "3.9-3.12" "Official wheel support"
    printf "%-20s %-20s %-30s\n" "Transformers" "3.9-3.13" "Broad compatibility"
    printf "%-20s %-20s %-30s\n" "Nix (scripts)" "3.13.x" "Latest stable"

    echo ""
    echo "Recommended upgrade paths:"
    echo ""
    echo "1. ${GREEN}Stay on Python 3.11 (current, recommended)${NC}"
    echo "   - Full RoboStack compatibility"
    echo "   - AIOS compatible"
    echo "   - All ML libraries supported"
    echo ""
    echo "2. ${YELLOW}Upgrade to Python 3.12 (partial)${NC}"
    echo "   - Requires building ROS2 from source"
    echo "   - AIOS will NOT work"
    echo "   - Better performance (10-15%)"
    echo ""
    echo "3. ${CYAN}Wait for ROS2 Jazzy + RoboStack update${NC}"
    echo "   - Full Python 3.12 support expected"
    echo "   - Target: RoboStack Jazzy release"
    echo ""

    echo "To upgrade Python version in default environment:"
    echo "  Edit pixi.toml: python = \">=3.12,<3.13\""
    echo "  Then run: pixi update"
    echo ""
    echo "Note: AIOS environment must remain on Python 3.11"
}

# Check all upgrades
check_all() {
    check_pixi_toml

    echo "=========================================="
    echo "Python Dependency Upgrade Analysis"
    echo "=========================================="

    get_current_versions
    echo ""
    check_pytorch_upgrades
    echo ""
    show_python_paths
}

# Main
main() {
    local cmd="${1:-check}"

    case "$cmd" in
        check)
            check_pixi_toml
            check_pytorch_upgrades
            ;;
        pytorch)
            check_pixi_toml
            suggest_pytorch_upgrade "${2:-}"
            ;;
        python)
            show_python_paths
            ;;
        all)
            check_all
            ;;
        -h|--help|help)
            echo "Usage: $0 [command] [options]"
            echo ""
            echo "Commands:"
            echo "  check        Check for available PyTorch upgrades"
            echo "  pytorch VER  Show upgrade instructions for PyTorch version"
            echo "  python       Show Python upgrade paths and constraints"
            echo "  all          Run all checks"
            echo ""
            echo "Examples:"
            echo "  $0 check"
            echo "  $0 pytorch 2.6"
            echo "  $0 python"
            ;;
        *)
            error "Unknown command: $cmd"
            echo "Run '$0 --help' for usage"
            exit 1
            ;;
    esac
}

main "$@"
