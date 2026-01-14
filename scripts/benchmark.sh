#!/usr/bin/env bash
#
# Performance benchmarking script for ripple-env
#
# Establishes baseline performance metrics for capacity planning and regression detection.
# Measures bootstrap time, build time, validation time, API latency, and resource usage.
#
# Usage:
#   ./scripts/benchmark.sh [--profile PROFILE] [--output JSON|TEXT] [--save]
#
# Profiles:
#   quick    - Fast benchmarks (~5 min): bootstrap, build, validation
#   standard - All benchmarks (~15 min): adds API latency tests
#   full     - Comprehensive (~30 min): adds load testing
#
# Output:
#   JSON format suitable for CI/CD integration and trending
#   TEXT format for human-readable results
#
# Exit Codes:
#   0 - Success
#   1 - Warning (some benchmarks failed)
#   2 - Critical error (cannot run benchmarks)

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PROFILE="${PROFILE:-standard}"
OUTPUT_FORMAT="${OUTPUT_FORMAT:-text}"
RESULTS_FILE="${RESULTS_FILE:-benchmark-results-$(date +%Y%m%d-%H%M%S).json}"
SAVE_RESULTS=false

# Benchmark targets (from PRODUCTION_SCALE.md)
TARGET_BOOTSTRAP_TIME=1800    # 30 minutes
TARGET_BUILD_TIME=600         # 10 minutes
TARGET_VALIDATION_TIME=300    # 5 minutes
TARGET_API_LATENCY_P95=100    # 100ms
TARGET_INFERENCE_LATENCY=2000 # 2 seconds

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Results storage
declare -A RESULTS

log_info() {
    echo -e "${GREEN}[INFO]${NC} $*"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*" >&2
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_bench() {
    echo -e "${BLUE}[BENCH]${NC} $*"
}

usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Performance benchmarking for ripple-env baseline metrics.

OPTIONS:
    --profile PROFILE   Benchmark profile: quick, standard, full (default: standard)
    --output FORMAT     Output format: text, json (default: text)
    --save              Save results to file: $RESULTS_FILE
    --help              Show this help message

PROFILES:
    quick      Fast benchmarks (~5 min)
               - Build time
               - Validation time

    standard   Standard benchmarks (~15 min)
               - All quick benchmarks
               - Bootstrap simulation
               - API health checks

    full       Comprehensive benchmarks (~30 min)
               - All standard benchmarks
               - API latency tests (P50/P95/P99)
               - Inference latency (if LocalAI running)
               - Resource usage profiling

TARGETS (from docs/PRODUCTION_SCALE.md):
    Bootstrap time:  <$TARGET_BOOTSTRAP_TIME seconds (30 min)
    Build time:      <$TARGET_BUILD_TIME seconds (10 min)
    Validation time: <$TARGET_VALIDATION_TIME seconds (5 min)
    API latency P95: <$TARGET_API_LATENCY_P95 ms
    Inference:       <$TARGET_INFERENCE_LATENCY ms

EXAMPLES:
    # Quick benchmark
    $0 --profile quick

    # Standard with JSON output
    $0 --output json

    # Full benchmark and save results
    $0 --profile full --save

See docs/PRODUCTION_SCALE.md for baseline requirements.
EOF
}

check_prerequisites() {
    log_info "Checking prerequisites..."

    local missing=()

    command -v pixi >/dev/null 2>&1 || missing+=("pixi")
    command -v colcon >/dev/null 2>&1 || missing+=("colcon")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required tools: ${missing[*]}"
        return 2
    fi

    # Optional tools
    if [[ "$PROFILE" == "full" ]]; then
        if ! command -v curl >/dev/null 2>&1; then
            log_warn "curl not found - API latency tests will be skipped"
        fi
    fi

    return 0
}

benchmark_build_time() {
    log_bench "Benchmarking ROS2 build time..."

    cd "$REPO_ROOT"

    # Clean build to ensure fair benchmark
    if [[ -d "build" ]]; then
        rm -rf build install log
    fi

    local start_time=$(date +%s)

    if colcon build --symlink-install --executor sequential 2>&1 | tee /tmp/benchmark-build.log; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))

        RESULTS[build_time_seconds]=$duration
        RESULTS[build_time_status]=$([ $duration -lt $TARGET_BUILD_TIME ] && echo "PASS" || echo "FAIL")

        log_bench "Build time: ${duration}s (target: <${TARGET_BUILD_TIME}s)"

        # Count packages built
        local packages=$(grep -c "Finished <<< " /tmp/benchmark-build.log || echo "0")
        RESULTS[build_packages_count]=$packages
        log_bench "Packages built: $packages"
    else
        log_error "Build failed"
        RESULTS[build_time_seconds]=-1
        RESULTS[build_time_status]="ERROR"
        return 1
    fi

    return 0
}

benchmark_validation_time() {
    log_bench "Benchmarking E2E validation time..."

    if [[ ! -f "$SCRIPT_DIR/validate-e2e.sh" ]]; then
        log_warn "validate-e2e.sh not found - skipping"
        RESULTS[validation_time_seconds]=-1
        RESULTS[validation_time_status]="SKIPPED"
        return 0
    fi

    local start_time=$(date +%s)

    if "$SCRIPT_DIR/validate-e2e.sh" >/tmp/benchmark-validation.log 2>&1; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))

        RESULTS[validation_time_seconds]=$duration
        RESULTS[validation_time_status]=$([ $duration -lt $TARGET_VALIDATION_TIME ] && echo "PASS" || echo "FAIL")

        log_bench "Validation time: ${duration}s (target: <${TARGET_VALIDATION_TIME}s)"
    else
        log_warn "Validation completed with warnings/errors"
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        RESULTS[validation_time_seconds]=$duration
        RESULTS[validation_time_status]="WARN"
    fi

    return 0
}

benchmark_bootstrap_simulation() {
    log_bench "Benchmarking bootstrap simulation..."

    # Simulate key bootstrap steps (don't actually re-bootstrap)
    log_bench "  Measuring pixi install time..."

    local start_time=$(date +%s)

    # Remove pixi cache and reinstall
    rm -rf .pixi 2>/dev/null || true

    if pixi install --frozen 2>&1 | tee /tmp/benchmark-pixi.log; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))

        RESULTS[pixi_install_seconds]=$duration
        log_bench "  Pixi install: ${duration}s"
    else
        log_warn "  Pixi install failed"
        RESULTS[pixi_install_seconds]=-1
    fi

    # Estimate full bootstrap time (bootstrap includes more steps)
    # From bootstrap.sh: Nix install (~5-10 min) + pixi (~2-5 min) + shells (~1-2 min)
    local estimated_bootstrap=$((duration * 3))
    RESULTS[bootstrap_estimated_seconds]=$estimated_bootstrap
    RESULTS[bootstrap_estimated_status]=$([ $estimated_bootstrap -lt $TARGET_BOOTSTRAP_TIME ] && echo "PASS" || echo "FAIL")

    log_bench "Estimated full bootstrap: ~${estimated_bootstrap}s (target: <${TARGET_BOOTSTRAP_TIME}s)"

    return 0
}

benchmark_api_health() {
    log_bench "Benchmarking API health checks..."

    if ! command -v curl >/dev/null 2>&1; then
        log_warn "curl not found - skipping API tests"
        return 0
    fi

    # Check common service ports from PORTS.md
    local services=(
        "vault:8200:/v1/sys/health"
        "keycloak:8080:/health/ready"
        "nats:4222:"
        "prometheus:9090/-/ready"
    )

    local healthy=0
    local total=0

    for service_spec in "${services[@]}"; do
        IFS=':' read -r name port path <<< "$service_spec"
        total=$((total + 1))

        local url="http://localhost:${port}${path}"

        if curl -sf "$url" -m 5 >/dev/null 2>&1; then
            log_bench "  ✓ $name is healthy"
            healthy=$((healthy + 1))
        else
            log_bench "  ✗ $name is not responding"
        fi
    done

    RESULTS[services_healthy]=$healthy
    RESULTS[services_total]=$total
    RESULTS[services_health_percent]=$(awk "BEGIN {printf \"%.0f\", ($healthy/$total)*100}")

    log_bench "Services healthy: $healthy/$total (${RESULTS[services_health_percent]}%)"

    return 0
}

benchmark_api_latency() {
    log_bench "Benchmarking API latency (P50/P95/P99)..."

    if ! command -v curl >/dev/null 2>&1; then
        log_warn "curl not found - skipping latency tests"
        return 0
    fi

    # Test Vault health endpoint (most critical service)
    local endpoint="http://localhost:8200/v1/sys/health"
    local iterations=100

    log_bench "  Running $iterations requests to Vault API..."

    local latencies=()
    for ((i=1; i<=iterations; i++)); do
        local start=$(date +%s%3N)
        if curl -sf "$endpoint" -m 2 >/dev/null 2>&1; then
            local end=$(date +%s%3N)
            local latency=$((end - start))
            latencies+=($latency)
        fi
    done

    if [[ ${#latencies[@]} -eq 0 ]]; then
        log_warn "  No successful requests - cannot calculate latency"
        return 0
    fi

    # Sort and calculate percentiles
    IFS=$'\n' latencies=($(sort -n <<<"${latencies[*]}"))

    local p50_idx=$((${#latencies[@]} * 50 / 100))
    local p95_idx=$((${#latencies[@]} * 95 / 100))
    local p99_idx=$((${#latencies[@]} * 99 / 100))

    RESULTS[api_latency_p50_ms]=${latencies[$p50_idx]}
    RESULTS[api_latency_p95_ms]=${latencies[$p95_idx]}
    RESULTS[api_latency_p99_ms]=${latencies[$p99_idx]}
    RESULTS[api_latency_status]=$([ ${latencies[$p95_idx]} -lt $TARGET_API_LATENCY_P95 ] && echo "PASS" || echo "FAIL")

    log_bench "  P50: ${latencies[$p50_idx]}ms"
    log_bench "  P95: ${latencies[$p95_idx]}ms (target: <${TARGET_API_LATENCY_P95}ms)"
    log_bench "  P99: ${latencies[$p99_idx]}ms"

    return 0
}

benchmark_inference_latency() {
    log_bench "Benchmarking inference latency (LocalAI)..."

    # Check if LocalAI is running
    if ! curl -sf http://localhost:8080/v1/models -m 2 >/dev/null 2>&1; then
        log_warn "LocalAI not running - skipping inference benchmark"
        RESULTS[inference_latency_ms]=-1
        RESULTS[inference_latency_status]="SKIPPED"
        return 0
    fi

    log_bench "  Running test inference request..."

    local start=$(date +%s%3N)

    # Simple completion request
    if curl -sf http://localhost:8080/v1/completions \
        -H "Content-Type: application/json" \
        -d '{"prompt": "Hello", "max_tokens": 5}' \
        -m 10 >/dev/null 2>&1; then

        local end=$(date +%s%3N)
        local latency=$((end - start))

        RESULTS[inference_latency_ms]=$latency
        RESULTS[inference_latency_status]=$([ $latency -lt $TARGET_INFERENCE_LATENCY ] && echo "PASS" || echo "FAIL")

        log_bench "  Inference latency: ${latency}ms (target: <${TARGET_INFERENCE_LATENCY}ms)"
    else
        log_warn "  Inference request failed"
        RESULTS[inference_latency_ms]=-1
        RESULTS[inference_latency_status]="ERROR"
    fi

    return 0
}

collect_system_metrics() {
    log_bench "Collecting system resource metrics..."

    # CPU cores
    RESULTS[system_cpu_cores]=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo "unknown")

    # RAM (in GB)
    if [[ -f /proc/meminfo ]]; then
        RESULTS[system_ram_gb]=$(awk '/MemTotal/ {printf "%.1f", $2/1024/1024}' /proc/meminfo)
    else
        RESULTS[system_ram_gb]="unknown"
    fi

    # Disk space available (in GB)
    RESULTS[system_disk_available_gb]=$(df -BG "$REPO_ROOT" | awk 'NR==2 {print $4}' | sed 's/G//')

    log_bench "  CPU: ${RESULTS[system_cpu_cores]} cores"
    log_bench "  RAM: ${RESULTS[system_ram_gb]} GB"
    log_bench "  Disk: ${RESULTS[system_disk_available_gb]} GB available"

    return 0
}

output_results() {
    local format="${1:-text}"

    if [[ "$format" == "json" ]]; then
        output_json
    else
        output_text
    fi
}

output_text() {
    echo ""
    echo "========================================="
    echo "  Benchmark Results"
    echo "========================================="
    echo ""
    echo "Profile: $PROFILE"
    echo "Timestamp: $(date --iso-8601=seconds)"
    echo ""

    echo "Build Performance:"
    echo "  Build time: ${RESULTS[build_time_seconds]:-N/A}s [${RESULTS[build_time_status]:-N/A}]"
    echo "  Packages: ${RESULTS[build_packages_count]:-N/A}"
    echo ""

    echo "Validation Performance:"
    echo "  Validation time: ${RESULTS[validation_time_seconds]:-N/A}s [${RESULTS[validation_time_status]:-N/A}]"
    echo ""

    if [[ "${RESULTS[pixi_install_seconds]:-}" ]]; then
        echo "Bootstrap Performance:"
        echo "  Pixi install: ${RESULTS[pixi_install_seconds]}s"
        echo "  Estimated bootstrap: ${RESULTS[bootstrap_estimated_seconds]}s [${RESULTS[bootstrap_estimated_status]}]"
        echo ""
    fi

    if [[ "${RESULTS[services_healthy]:-}" ]]; then
        echo "Service Health:"
        echo "  Healthy: ${RESULTS[services_healthy]}/${RESULTS[services_total]} (${RESULTS[services_health_percent]}%)"
        echo ""
    fi

    if [[ "${RESULTS[api_latency_p95_ms]:-}" ]]; then
        echo "API Latency:"
        echo "  P50: ${RESULTS[api_latency_p50_ms]}ms"
        echo "  P95: ${RESULTS[api_latency_p95_ms]}ms [${RESULTS[api_latency_status]}]"
        echo "  P99: ${RESULTS[api_latency_p99_ms]}ms"
        echo ""
    fi

    if [[ "${RESULTS[inference_latency_ms]:-}" != "-1" ]]; then
        echo "Inference Performance:"
        echo "  Latency: ${RESULTS[inference_latency_ms]}ms [${RESULTS[inference_latency_status]}]"
        echo ""
    fi

    echo "System Resources:"
    echo "  CPU: ${RESULTS[system_cpu_cores]} cores"
    echo "  RAM: ${RESULTS[system_ram_gb]} GB"
    echo "  Disk: ${RESULTS[system_disk_available_gb]} GB available"
    echo ""
    echo "========================================="
}

output_json() {
    cat << EOF
{
  "benchmark_profile": "$PROFILE",
  "timestamp": "$(date --iso-8601=seconds)",
  "build": {
    "time_seconds": ${RESULTS[build_time_seconds]:-null},
    "status": "${RESULTS[build_time_status]:-null}",
    "packages_count": ${RESULTS[build_packages_count]:-null}
  },
  "validation": {
    "time_seconds": ${RESULTS[validation_time_seconds]:-null},
    "status": "${RESULTS[validation_time_status]:-null}"
  },
  "bootstrap": {
    "pixi_install_seconds": ${RESULTS[pixi_install_seconds]:-null},
    "estimated_total_seconds": ${RESULTS[bootstrap_estimated_seconds]:-null},
    "status": "${RESULTS[bootstrap_estimated_status]:-null}"
  },
  "services": {
    "healthy": ${RESULTS[services_healthy]:-null},
    "total": ${RESULTS[services_total]:-null},
    "health_percent": ${RESULTS[services_health_percent]:-null}
  },
  "api_latency": {
    "p50_ms": ${RESULTS[api_latency_p50_ms]:-null},
    "p95_ms": ${RESULTS[api_latency_p95_ms]:-null},
    "p99_ms": ${RESULTS[api_latency_p99_ms]:-null},
    "status": "${RESULTS[api_latency_status]:-null}"
  },
  "inference": {
    "latency_ms": ${RESULTS[inference_latency_ms]:-null},
    "status": "${RESULTS[inference_latency_status]:-null}"
  },
  "system": {
    "cpu_cores": "${RESULTS[system_cpu_cores]:-unknown}",
    "ram_gb": "${RESULTS[system_ram_gb]:-unknown}",
    "disk_available_gb": "${RESULTS[system_disk_available_gb]:-unknown}"
  }
}
EOF
}

save_results() {
    if [[ "$SAVE_RESULTS" == "true" ]]; then
        log_info "Saving results to: $RESULTS_FILE"
        output_json > "$RESULTS_FILE"
        log_info "Results saved"
    fi
}

main() {
    log_info "Starting performance benchmarks"
    log_info "Profile: $PROFILE"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --profile)
                PROFILE="$2"
                shift 2
                ;;
            --output)
                OUTPUT_FORMAT="$2"
                shift 2
                ;;
            --save)
                SAVE_RESULTS=true
                shift
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 2
                ;;
        esac
    done

    # Validate profile
    if [[ ! "$PROFILE" =~ ^(quick|standard|full)$ ]]; then
        log_error "Invalid profile: $PROFILE (must be quick, standard, or full)"
        exit 2
    fi

    check_prerequisites || exit $?
    collect_system_metrics

    # Run benchmarks based on profile
    case "$PROFILE" in
        quick)
            benchmark_build_time
            benchmark_validation_time
            ;;
        standard)
            benchmark_build_time
            benchmark_validation_time
            benchmark_bootstrap_simulation
            benchmark_api_health
            ;;
        full)
            benchmark_build_time
            benchmark_validation_time
            benchmark_bootstrap_simulation
            benchmark_api_health
            benchmark_api_latency
            benchmark_inference_latency
            ;;
    esac

    # Output results
    output_results "$OUTPUT_FORMAT"
    save_results

    log_info "Benchmarking complete"
    exit 0
}

main "$@"
