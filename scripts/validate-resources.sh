#!/usr/bin/env bash
# Resource Validation Script for AI Services
#
# Validates system resources before starting AI services.
# Checks RAM, CPU, GPU, and disk space requirements.
#
# Usage:
#   ./validate-resources.sh [OPTIONS]
#
# Options:
#   --profile <name>    Profile to validate against (lightweight|standard|gpu|full)
#   --check-only        Only check resources, don't suggest fixes
#   --json              Output results as JSON
#   --verbose           Show detailed information
#   --help              Show this help message
#
# Exit codes:
#   0 - All requirements met
#   1 - Some requirements not met (warnings)
#   2 - Critical requirements not met (errors)

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
PROFILE="standard"
CHECK_ONLY=false
JSON_OUTPUT=false
VERBOSE=false

# Profile requirements (in MB for RAM, GB for disk)
declare -A PROFILE_RAM_MIN=(
    ["lightweight"]=4096
    ["standard"]=16384
    ["gpu"]=16384
    ["full"]=32768
)

declare -A PROFILE_RAM_REC=(
    ["lightweight"]=8192
    ["standard"]=32768
    ["gpu"]=32768
    ["full"]=65536
)

declare -A PROFILE_CPU_MIN=(
    ["lightweight"]=2
    ["standard"]=4
    ["gpu"]=4
    ["full"]=8
)

declare -A PROFILE_DISK_MIN=(
    ["lightweight"]=20
    ["standard"]=50
    ["gpu"]=50
    ["full"]=100
)

declare -A PROFILE_VRAM_MIN=(
    ["lightweight"]=0
    ["standard"]=0
    ["gpu"]=8192
    ["full"]=16384
)

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --check-only)
            CHECK_ONLY=true
            shift
            ;;
        --json)
            JSON_OUTPUT=true
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        --help)
            head -30 "$0" | tail -25
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate profile
if [[ ! -v "PROFILE_RAM_MIN[$PROFILE]" ]]; then
    echo "Invalid profile: $PROFILE"
    echo "Valid profiles: lightweight, standard, gpu, full"
    exit 1
fi

# Results storage
declare -a ERRORS=()
declare -a WARNINGS=()
declare -a PASSED=()

# Utility functions
log_info() {
    if [[ "$JSON_OUTPUT" != "true" ]]; then
        echo -e "${BLUE}[INFO]${NC} $1"
    fi
}

log_pass() {
    PASSED+=("$1")
    if [[ "$JSON_OUTPUT" != "true" ]]; then
        echo -e "${GREEN}[PASS]${NC} $1"
    fi
}

log_warn() {
    WARNINGS+=("$1")
    if [[ "$JSON_OUTPUT" != "true" ]]; then
        echo -e "${YELLOW}[WARN]${NC} $1"
    fi
}

log_error() {
    ERRORS+=("$1")
    if [[ "$JSON_OUTPUT" != "true" ]]; then
        echo -e "${RED}[FAIL]${NC} $1"
    fi
}

log_verbose() {
    if [[ "$VERBOSE" == "true" && "$JSON_OUTPUT" != "true" ]]; then
        echo -e "       $1"
    fi
}

# Get system information
get_total_ram_mb() {
    if [[ "$(uname)" == "Darwin" ]]; then
        sysctl -n hw.memsize | awk '{print int($1/1024/1024)}'
    else
        grep MemTotal /proc/meminfo | awk '{print int($2/1024)}'
    fi
}

get_available_ram_mb() {
    if [[ "$(uname)" == "Darwin" ]]; then
        # macOS: approximate available memory
        vm_stat | awk '/Pages free/ {free=$3} /Pages inactive/ {inactive=$3} END {print int((free+inactive)*4096/1024/1024)}'
    else
        grep MemAvailable /proc/meminfo | awk '{print int($2/1024)}'
    fi
}

get_swap_mb() {
    if [[ "$(uname)" == "Darwin" ]]; then
        sysctl -n vm.swapusage | awk '{print int($2/1024/1024)}'
    else
        grep SwapTotal /proc/meminfo | awk '{print int($2/1024)}'
    fi
}

get_cpu_cores() {
    if [[ "$(uname)" == "Darwin" ]]; then
        sysctl -n hw.ncpu
    else
        nproc
    fi
}

get_cpu_model() {
    if [[ "$(uname)" == "Darwin" ]]; then
        sysctl -n machdep.cpu.brand_string 2>/dev/null || echo "Apple Silicon"
    else
        grep "model name" /proc/cpuinfo | head -1 | cut -d: -f2 | xargs
    fi
}

get_disk_free_gb() {
    local path="${1:-.}"
    df -BG "$path" 2>/dev/null | tail -1 | awk '{print int($4)}' || echo "0"
}

get_nvidia_gpu_info() {
    if command -v nvidia-smi &>/dev/null; then
        nvidia-smi --query-gpu=name,memory.total --format=csv,noheader,nounits 2>/dev/null || echo ""
    else
        echo ""
    fi
}

get_nvidia_vram_mb() {
    if command -v nvidia-smi &>/dev/null; then
        nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits 2>/dev/null | head -1 || echo "0"
    else
        echo "0"
    fi
}

get_nvidia_driver_version() {
    if command -v nvidia-smi &>/dev/null; then
        nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 || echo ""
    else
        echo ""
    fi
}

get_cuda_version() {
    if command -v nvidia-smi &>/dev/null; then
        nvidia-smi --query-gpu=cuda_version --format=csv,noheader 2>/dev/null | head -1 || echo ""
    else
        echo ""
    fi
}

check_docker_gpu() {
    if docker info 2>/dev/null | grep -q "Runtimes.*nvidia"; then
        echo "true"
    else
        echo "false"
    fi
}

# Check functions
check_ram() {
    local total_ram
    local available_ram
    local swap
    local min_ram="${PROFILE_RAM_MIN[$PROFILE]}"
    local rec_ram="${PROFILE_RAM_REC[$PROFILE]}"

    total_ram=$(get_total_ram_mb)
    available_ram=$(get_available_ram_mb)
    swap=$(get_swap_mb)

    log_verbose "Total RAM: ${total_ram}MB"
    log_verbose "Available RAM: ${available_ram}MB"
    log_verbose "Swap: ${swap}MB"

    if [[ $total_ram -lt $min_ram ]]; then
        log_error "Insufficient RAM: ${total_ram}MB < ${min_ram}MB minimum for $PROFILE profile"
        if [[ "$CHECK_ONLY" != "true" ]]; then
            log_verbose "Suggestion: Add more RAM or use 'lightweight' profile"
        fi
    elif [[ $total_ram -lt $rec_ram ]]; then
        log_warn "RAM below recommended: ${total_ram}MB < ${rec_ram}MB for optimal $PROFILE performance"
    else
        log_pass "RAM: ${total_ram}MB (minimum: ${min_ram}MB, recommended: ${rec_ram}MB)"
    fi

    if [[ $swap -lt 4096 && $total_ram -lt $rec_ram ]]; then
        log_warn "Low swap space: ${swap}MB. Consider enabling 8-16GB swap for headroom"
    fi
}

check_cpu() {
    local cores
    local cpu_model
    local min_cores="${PROFILE_CPU_MIN[$PROFILE]}"

    cores=$(get_cpu_cores)
    cpu_model=$(get_cpu_model)

    log_verbose "CPU: $cpu_model"
    log_verbose "Cores: $cores"

    if [[ $cores -lt $min_cores ]]; then
        log_error "Insufficient CPU cores: $cores < $min_cores minimum for $PROFILE profile"
    else
        log_pass "CPU: $cores cores (minimum: $min_cores)"
    fi
}

check_gpu() {
    local vram
    local gpu_info
    local driver_version
    local cuda_version
    local docker_gpu
    local min_vram="${PROFILE_VRAM_MIN[$PROFILE]}"

    gpu_info=$(get_nvidia_gpu_info)
    vram=$(get_nvidia_vram_mb)
    driver_version=$(get_nvidia_driver_version)
    cuda_version=$(get_cuda_version)
    docker_gpu=$(check_docker_gpu)

    if [[ -z "$gpu_info" ]]; then
        if [[ $min_vram -gt 0 ]]; then
            log_error "No NVIDIA GPU detected. $PROFILE profile requires ${min_vram}MB VRAM"
            if [[ "$CHECK_ONLY" != "true" ]]; then
                log_verbose "Suggestion: Use 'standard' or 'lightweight' profile for CPU-only mode"
            fi
        else
            log_pass "GPU: Not required for $PROFILE profile (CPU-only mode)"
        fi
        return
    fi

    log_verbose "GPU: $gpu_info"
    log_verbose "VRAM: ${vram}MB"
    log_verbose "Driver: $driver_version"
    log_verbose "CUDA: $cuda_version"
    log_verbose "Docker GPU support: $docker_gpu"

    if [[ $vram -lt $min_vram && $min_vram -gt 0 ]]; then
        log_error "Insufficient VRAM: ${vram}MB < ${min_vram}MB minimum for $PROFILE profile"
    elif [[ $min_vram -gt 0 ]]; then
        log_pass "GPU: ${gpu_info} with ${vram}MB VRAM (minimum: ${min_vram}MB)"
    else
        log_pass "GPU: ${gpu_info} with ${vram}MB VRAM (optional for $PROFILE profile)"
    fi

    # Check Docker GPU support
    if [[ "$docker_gpu" != "true" && $min_vram -gt 0 ]]; then
        log_warn "Docker NVIDIA runtime not detected. GPU acceleration may not work in containers"
        if [[ "$CHECK_ONLY" != "true" ]]; then
            log_verbose "Install nvidia-container-toolkit and restart Docker"
        fi
    fi

    # Check CUDA version compatibility
    if [[ -n "$cuda_version" ]]; then
        local cuda_major
        cuda_major=$(echo "$cuda_version" | cut -d. -f1)
        if [[ $cuda_major -lt 11 ]]; then
            log_warn "CUDA $cuda_version may be too old. CUDA 11.0+ recommended"
        fi
    fi
}

check_disk() {
    local script_dir
    local data_dir
    local free_gb
    local min_disk="${PROFILE_DISK_MIN[$PROFILE]}"

    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    data_dir="$(dirname "$script_dir")/data"

    # Check main project directory
    free_gb=$(get_disk_free_gb "$script_dir")
    log_verbose "Free disk space: ${free_gb}GB"

    if [[ $free_gb -lt $min_disk ]]; then
        log_error "Insufficient disk space: ${free_gb}GB < ${min_disk}GB minimum for $PROFILE profile"
        if [[ "$CHECK_ONLY" != "true" ]]; then
            log_verbose "Free up disk space or use external storage for models"
        fi
    else
        log_pass "Disk space: ${free_gb}GB free (minimum: ${min_disk}GB)"
    fi

    # Check data directory if exists
    if [[ -d "$data_dir" ]]; then
        local data_free
        data_free=$(get_disk_free_gb "$data_dir")
        if [[ $data_free -lt 15 ]]; then
            log_warn "Low space in data directory: ${data_free}GB. Models require ~15-20GB"
        fi
    fi
}

check_docker() {
    if ! command -v docker &>/dev/null; then
        log_error "Docker not found. Required for AI services"
        return
    fi

    if ! docker info &>/dev/null; then
        log_error "Docker daemon not running or not accessible"
        return
    fi

    log_pass "Docker: Available and running"

    # Check Docker Compose
    if docker compose version &>/dev/null; then
        local compose_version
        compose_version=$(docker compose version --short 2>/dev/null || echo "unknown")
        log_pass "Docker Compose: v$compose_version"
    else
        log_warn "Docker Compose plugin not found. Install docker-compose-plugin"
    fi

    # Check Docker memory limit
    local docker_mem
    docker_mem=$(docker info --format '{{.MemTotal}}' 2>/dev/null | awk '{print int($1/1024/1024)}' || echo "0")
    if [[ $docker_mem -gt 0 ]]; then
        log_verbose "Docker memory limit: ${docker_mem}MB"
        local min_ram="${PROFILE_RAM_MIN[$PROFILE]}"
        if [[ $docker_mem -lt $((min_ram / 2)) ]]; then
            log_warn "Docker memory limit may be too low for $PROFILE profile"
        fi
    fi
}

check_network() {
    # Check if agentic-network exists
    if docker network ls --format '{{.Name}}' 2>/dev/null | grep -q "^agentic-network$"; then
        log_pass "Docker network: agentic-network exists"
    else
        log_warn "Docker network 'agentic-network' not found. Create with: docker network create agentic-network"
    fi
}

check_ports() {
    local ports=(8080 7437 3437 5432 9000 9001 8000 6379 3000)
    local port_names=(
        "8080:LocalAI"
        "7437:AGiXT API"
        "3437:AGiXT UI"
        "5432:PostgreSQL"
        "9000:MinIO API"
        "9001:MinIO Console"
        "8000:AIOS Kernel"
        "6379:Redis"
        "3000:Embedding Service"
    )

    local conflicts=()
    for port_info in "${port_names[@]}"; do
        local port="${port_info%%:*}"
        local name="${port_info#*:}"
        if lsof -i ":$port" &>/dev/null || ss -tuln 2>/dev/null | grep -q ":$port "; then
            conflicts+=("$port ($name)")
        fi
    done

    if [[ ${#conflicts[@]} -gt 0 ]]; then
        log_warn "Ports in use: ${conflicts[*]}"
        if [[ "$CHECK_ONLY" != "true" ]]; then
            log_verbose "Stop conflicting services or configure alternative ports"
        fi
    else
        log_pass "Required ports: All available"
    fi
}

# Output JSON results
output_json() {
    local total_ram available_ram swap cpu_cores
    local gpu_info vram driver cuda disk_free

    total_ram=$(get_total_ram_mb)
    available_ram=$(get_available_ram_mb)
    swap=$(get_swap_mb)
    cpu_cores=$(get_cpu_cores)
    gpu_info=$(get_nvidia_gpu_info)
    vram=$(get_nvidia_vram_mb)
    driver=$(get_nvidia_driver_version)
    cuda=$(get_cuda_version)
    disk_free=$(get_disk_free_gb)

    cat <<EOF
{
  "profile": "$PROFILE",
  "system": {
    "ram_total_mb": $total_ram,
    "ram_available_mb": $available_ram,
    "swap_mb": $swap,
    "cpu_cores": $cpu_cores,
    "gpu": {
      "name": "$gpu_info",
      "vram_mb": ${vram:-0},
      "driver": "$driver",
      "cuda": "$cuda"
    },
    "disk_free_gb": $disk_free
  },
  "requirements": {
    "ram_min_mb": ${PROFILE_RAM_MIN[$PROFILE]},
    "ram_rec_mb": ${PROFILE_RAM_REC[$PROFILE]},
    "cpu_min": ${PROFILE_CPU_MIN[$PROFILE]},
    "disk_min_gb": ${PROFILE_DISK_MIN[$PROFILE]},
    "vram_min_mb": ${PROFILE_VRAM_MIN[$PROFILE]}
  },
  "results": {
    "passed": ${#PASSED[@]},
    "warnings": ${#WARNINGS[@]},
    "errors": ${#ERRORS[@]}
  },
  "status": "$(if [[ ${#ERRORS[@]} -gt 0 ]]; then echo "fail"; elif [[ ${#WARNINGS[@]} -gt 0 ]]; then echo "warn"; else echo "pass"; fi)"
}
EOF
}

# Main execution
main() {
    if [[ "$JSON_OUTPUT" != "true" ]]; then
        echo ""
        echo "========================================"
        echo "  AI Resource Validation"
        echo "  Profile: $PROFILE"
        echo "========================================"
        echo ""
    fi

    check_ram
    check_cpu
    check_gpu
    check_disk
    check_docker
    check_network
    check_ports

    if [[ "$JSON_OUTPUT" == "true" ]]; then
        output_json
    else
        echo ""
        echo "========================================"
        echo "  Summary"
        echo "========================================"
        echo -e "  ${GREEN}Passed:${NC}   ${#PASSED[@]}"
        echo -e "  ${YELLOW}Warnings:${NC} ${#WARNINGS[@]}"
        echo -e "  ${RED}Errors:${NC}   ${#ERRORS[@]}"
        echo ""

        if [[ ${#ERRORS[@]} -gt 0 ]]; then
            echo -e "${RED}Some requirements not met. Review errors above.${NC}"
            echo ""
        elif [[ ${#WARNINGS[@]} -gt 0 ]]; then
            echo -e "${YELLOW}All minimum requirements met with some warnings.${NC}"
            echo ""
        else
            echo -e "${GREEN}All requirements met for $PROFILE profile!${NC}"
            echo ""
        fi
    fi

    # Exit with appropriate code
    if [[ ${#ERRORS[@]} -gt 0 ]]; then
        exit 2
    elif [[ ${#WARNINGS[@]} -gt 0 ]]; then
        exit 1
    else
        exit 0
    fi
}

main "$@"
