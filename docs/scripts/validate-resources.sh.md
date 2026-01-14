# Script Contract: validate-resources.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/validate-resources.sh`

---

## Purpose

Validate system resources (RAM, CPU, GPU, disk) against AI service requirements before deployment. Supports four profiles (lightweight, standard, gpu, full) with configurable thresholds. Checks Docker availability, GPU drivers, network configuration, and port conflicts. Designed for pre-deployment validation and capacity planning.

---

## Invocation

```bash
./scripts/validate-resources.sh [OPTIONS]
```

**Options:**
- `--profile <name>` - Profile to validate against: `lightweight`, `standard`, `gpu`, `full` (default: `standard`)
- `--check-only` - Only check resources, don't suggest fixes
- `--json` - Output results as JSON (for CI/CD integration)
- `--verbose` - Show detailed system information
- `--help` - Show help message

**Examples:**
```bash
./scripts/validate-resources.sh                        # Standard profile
./scripts/validate-resources.sh --profile gpu          # GPU profile
./scripts/validate-resources.sh --json                 # JSON output for CI
./scripts/validate-resources.sh --verbose --profile full  # Full profile with details
```

---

## Outputs

**Standard Output (success):**
```
========================================
  AI Resource Validation
  Profile: standard
========================================

[PASS] RAM: 32768MB (minimum: 16384MB, recommended: 32768MB)
[PASS] CPU: 8 cores (minimum: 4)
[PASS] GPU: Not required for standard profile (CPU-only mode)
[PASS] Disk space: 120GB free (minimum: 50GB)
[PASS] Docker: Available and running
[PASS] Docker Compose: v2.20.2
[PASS] Docker network: agentic-network exists
[PASS] Required ports: All available

========================================
  Summary
========================================
  Passed:   8
  Warnings: 0
  Errors:   0

All requirements met for standard profile!
```

**Standard Output (with warnings):**
```
========================================
  AI Resource Validation
  Profile: gpu
========================================

[PASS] RAM: 16384MB (minimum: 16384MB, recommended: 32768MB)
[WARN] RAM below recommended: 16384MB < 32768MB for optimal gpu performance
[PASS] CPU: 4 cores (minimum: 4)
[PASS] GPU: NVIDIA GeForce RTX 3080 with 10240MB VRAM (minimum: 8192MB)
[WARN] Docker NVIDIA runtime not detected. GPU acceleration may not work in containers
[PASS] Disk space: 80GB free (minimum: 50GB)
[PASS] Docker: Available and running
[PASS] Docker Compose: v2.20.2
[WARN] Docker network 'agentic-network' not found. Create with: docker network create agentic-network
[WARN] Ports in use: 8080 (LocalAI), 5432 (PostgreSQL)

========================================
  Summary
========================================
  Passed:   6
  Warnings: 4
  Errors:   0

All minimum requirements met with some warnings.
```

**Standard Output (errors):**
```
========================================
  AI Resource Validation
  Profile: full
========================================

[FAIL] Insufficient RAM: 8192MB < 32768MB minimum for full profile
[FAIL] Insufficient CPU cores: 2 < 8 minimum for full profile
[PASS] GPU: Not required for full profile (CPU-only mode)
[FAIL] Insufficient disk space: 15GB < 100GB minimum for full profile
[PASS] Docker: Available and running
[PASS] Docker Compose: v2.20.2
[WARN] Docker network 'agentic-network' not found. Create with: docker network create agentic-network
[PASS] Required ports: All available

========================================
  Summary
========================================
  Passed:   4
  Warnings: 1
  Errors:   3

Some requirements not met. Review errors above.
```

**JSON Output (--json):**
```json
{
  "profile": "standard",
  "system": {
    "ram_total_mb": 32768,
    "ram_available_mb": 28000,
    "swap_mb": 8192,
    "cpu_cores": 8,
    "gpu": {
      "name": "",
      "vram_mb": 0,
      "driver": "",
      "cuda": ""
    },
    "disk_free_gb": 120
  },
  "requirements": {
    "ram_min_mb": 16384,
    "ram_rec_mb": 32768,
    "cpu_min": 4,
    "disk_min_gb": 50,
    "vram_min_mb": 0
  },
  "results": {
    "passed": 8,
    "warnings": 0,
    "errors": 0
  },
  "status": "pass"
}
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all requirements met) |
| `1` | Warning (minimum requirements met, but some warnings) |
| `2` | Error (critical requirements not met) |

---

## Side Effects

**None** - Read-only resource checks.

---

## Safety Classification

**🟢 SAFE** - Read-only validation, no modifications.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Profile Requirements

**Evidence:** Lines 37-71

```bash
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
```

### Profile Breakdown

| Profile | RAM Min | RAM Rec | CPU Min | Disk Min | VRAM Min | Use Case |
|---------|---------|---------|---------|----------|----------|----------|
| **lightweight** | 4GB | 8GB | 2 cores | 20GB | 0MB | Development, testing |
| **standard** | 16GB | 32GB | 4 cores | 50GB | 0MB | Production CPU-only |
| **gpu** | 16GB | 32GB | 4 cores | 50GB | 8GB | GPU-accelerated ML |
| **full** | 32GB | 64GB | 8 cores | 100GB | 16GB | Large models, training |

---

## System Information Functions

### RAM Detection (lines 150-165)

**Evidence:**
```bash
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
```

**Platform-specific:**
- macOS: `sysctl hw.memsize`, `vm_stat` (lines 151-152, 160-161)
- Linux: `/proc/meminfo` (lines 154, 163)

### CPU Detection (lines 175-189)

**Evidence:**
```bash
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
```

### GPU Detection (lines 196-226)

**Evidence:**
```bash
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
```

**NVIDIA-only:** Checks for `nvidia-smi` availability

### Docker GPU Support (lines 228-234)

**Evidence:**
```bash
check_docker_gpu() {
    if docker info 2>/dev/null | grep -q "Runtimes.*nvidia"; then
        echo "true"
    else
        echo "false"
    fi
}
```

**Detection:** Checks for NVIDIA runtime in `docker info`

---

## Validation Checks

### RAM Check (lines 237-266)

**Evidence:**
```bash
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
```

**Three-tier validation:**
1. Error if below minimum (line 252)
2. Warning if below recommended (line 257)
3. Pass if meets recommended (line 260)

**Swap check:** Warns if < 4GB swap and RAM < recommended (line 263)

### CPU Check (lines 268-284)

**Evidence:**
```bash
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
```

### GPU Check (lines 286-342)

**Evidence:**
```bash
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
```

**GPU detection logic:**
1. If no GPU and required → Error (line 301)
2. If no GPU and optional → Pass (line 307)
3. If GPU present → Check VRAM (line 318)
4. Check Docker GPU support (line 327)
5. Check CUDA version (line 335)

### Disk Check (lines 344-374)

**Evidence:**
```bash
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
```

**Two checks:**
1. Project directory free space (line 354)
2. Data directory free space if exists (line 367)

### Docker Check (lines 376-408)

**Evidence:**
```bash
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
```

**Checks:**
1. Docker command available (line 377)
2. Docker daemon running (line 382)
3. Docker Compose plugin (line 390)
4. Docker memory limit (line 399)

### Network Check (lines 410-417)

**Evidence:**
```bash
check_network() {
    # Check if agentic-network exists
    if docker network ls --format '{{.Name}}' 2>/dev/null | grep -q "^agentic-network$"; then
        log_pass "Docker network: agentic-network exists"
    else
        log_warn "Docker network 'agentic-network' not found. Create with: docker network create agentic-network"
    fi
}
```

### Port Conflict Check (lines 419-450)

**Evidence:**
```bash
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
```

**Checked ports:**
- 8080: LocalAI
- 7437: AGiXT API
- 3437: AGiXT UI
- 5432: PostgreSQL
- 9000/9001: MinIO API/Console
- 8000: AIOS Kernel
- 6379: Redis
- 3000: Embedding Service

**Detection:** Uses `lsof` or `ss` (line 437)

---

## JSON Output

**Evidence:** Lines 452-498

```bash
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
```

**Structure:**
- System information (current state)
- Requirements (profile thresholds)
- Results (pass/warn/error counts)
- Status (fail/warn/pass)

---

## Exit Code Logic

**Evidence:** Lines 543-550

```bash
# Exit with appropriate code
if [[ ${#ERRORS[@]} -gt 0 ]]; then
    exit 2
elif [[ ${#WARNINGS[@]} -gt 0 ]]; then
    exit 1
else
    exit 0
fi
```

**Three-tier exit codes:**
- 2: Errors (critical requirements not met)
- 1: Warnings (minimum met, but concerns)
- 0: Success (all requirements met)

---

## Usage Patterns

### Pre-Deployment Validation
```bash
# Before starting services
./scripts/validate-resources.sh --profile gpu --verbose

# If passed, proceed
docker compose up -d
```

### CI/CD Integration
```yaml
# .github/workflows/deploy.yml
- name: Validate Resources
  run: ./scripts/validate-resources.sh --json --profile standard > resources.json

- name: Check Validation Result
  run: |
    if [ $(jq -r .status resources.json) != "pass" ]; then
      echo "Resource validation failed"
      exit 1
    fi
```

### Profile Selection
```bash
# Development laptop (limited resources)
./scripts/validate-resources.sh --profile lightweight

# Production server (CPU-only)
./scripts/validate-resources.sh --profile standard

# GPU server (ML inference)
./scripts/validate-resources.sh --profile gpu

# ML training cluster
./scripts/validate-resources.sh --profile full
```

---

## References

### Source Code
- **Main script:** `scripts/validate-resources.sh` (554 lines)
- **Profile requirements:** lines 37-71
- **System detection:** lines 150-234
- **Validation checks:** lines 237-450
- **JSON output:** lines 452-498
- **Main execution:** lines 500-553

### Related Files
- **Deployment scripts:** `scripts/deploy*.sh` (call this for validation)
- **Docker configs:** Various `docker-compose*.yml` files

### External Resources
- [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-container-toolkit)
- [Docker Resource Constraints](https://docs.docker.com/config/containers/resource_constraints/)
- [CUDA Compatibility](https://docs.nvidia.com/deploy/cuda-compatibility/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 37/60 contracts complete
