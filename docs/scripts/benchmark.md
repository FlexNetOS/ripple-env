# benchmark.sh

## Purpose

Establish baseline performance metrics for capacity planning and regression detection through automated benchmarking of build time, validation time, API latency, and resource usage.

## Invocation

```bash
./scripts/benchmark.sh [--profile PROFILE] [--output FORMAT] [--save]
```

## Inputs

### Arguments
- `--profile PROFILE` - Benchmark profile: `quick`, `standard`, `full` (default: standard)
- `--output FORMAT` - Output format: `text`, `json` (default: text)
- `--save` - Save results to timestamped JSON file
- `--help` - Show help message

### Profiles

| Profile | Duration | Benchmarks Included |
|---------|----------|---------------------|
| **quick** | ~5 min | Build time, Validation time |
| **standard** | ~15 min | Quick + Bootstrap simulation, API health |
| **full** | ~30 min | Standard + API latency (P50/P95/P99), Inference latency |

### Environment Variables
- `PROFILE` - Default profile (default: standard)
- `OUTPUT_FORMAT` - Default output format (default: text)
- `RESULTS_FILE` - Output filename (default: benchmark-results-YYYYMMDD-HHMMSS.json)

## Outputs

### Text Format (Human-Readable)
```
=========================================
  Benchmark Results
=========================================

Profile: standard
Timestamp: 2026-01-14T12:00:00+00:00

Build Performance:
  Build time: 450s [PASS]
  Packages: 25

Validation Performance:
  Validation time: 180s [PASS]

...
```

### JSON Format (Machine-Readable)
```json
{
  "benchmark_profile": "standard",
  "timestamp": "2026-01-14T12:00:00+00:00",
  "build": {
    "time_seconds": 450,
    "status": "PASS",
    "packages_count": 25
  },
  "validation": {
    "time_seconds": 180,
    "status": "PASS"
  },
  ...
}
```

### Exit Codes
- `0` - Success (all benchmarks completed)
- `1` - Warning (some benchmarks failed but overall success)
- `2` - Critical error (cannot run benchmarks, missing prerequisites)

### Saved Files (with --save)
- `benchmark-results-YYYYMMDD-HHMMSS.json` - JSON results file

## Side Effects

### File System
- Creates temporary log files in `/tmp/benchmark-*.log`
- Removes and rebuilds ROS2 workspace (clean build for fairness)
- Removes `.pixi/` directory for bootstrap simulation
- Creates JSON results file if `--save` flag used

### Resource Usage
- CPU: High during build/validation benchmarks
- RAM: Moderate (build artifacts in memory)
- Disk: ~2GB temporary space for clean builds
- Time: 5-30 minutes depending on profile

### Network
- API latency tests: 100+ HTTP requests to localhost services
- Inference tests: 1 HTTP request to LocalAI (if running)

## Safety Classification

- **Safety**: caution (destructive clean builds, removes .pixi/)
- **Idempotent**: yes (can be run multiple times, results may vary)

## Idempotency

Script is idempotent with expected variance:
- Clean build ensures consistent starting state
- Results will vary based on system load
- Multiple runs provide statistical confidence
- Saved results are timestamped (no overwriting)

## Dependencies

### Required Tools
- `bash` (4.0+)
- `pixi` - Dependency management
- `colcon` - ROS2 build system
- `date` - Timestamp generation

### Optional Tools
- `curl` - API latency benchmarks (full profile)
- `nproc` or `sysctl` - CPU core detection
- `awk` - Result calculation

### Services (for Full Profile)
- Vault (localhost:8200)
- Keycloak (localhost:8080)
- NATS (localhost:4222)
- Prometheus (localhost:9090)
- LocalAI (localhost:8080) - optional for inference tests

## Failure Modes

### MISSING_PREREQUISITES
**Error**: "Missing required tools: pixi colcon"
**Solution**: Install with `pixi add colcon-common-extensions`

### BUILD_FAILED
**Exit Code**: 1 (warning)
**Impact**: Build benchmark marked as ERROR, other benchmarks continue
**Solution**: Check build logs at `/tmp/benchmark-build.log`

### VALIDATION_FAILED
**Exit Code**: 1 (warning)
**Impact**: Validation benchmark marked as WARN/ERROR, other benchmarks continue
**Solution**: Check validation logs at `/tmp/benchmark-validation.log`

### SERVICES_NOT_RUNNING
**Impact**: API health/latency tests skipped or fail
**Solution**: Start services with `docker compose up -d`

### LOCALAI_NOT_RUNNING
**Impact**: Inference latency test skipped
**Solution**: Start LocalAI or skip with `--profile standard`

### DISK_FULL
**Error**: "No space left on device"
**Solution**: Free up disk space (need ~2GB for clean builds)

## Examples

### Quick Benchmark (5 min)
```bash
./scripts/benchmark.sh --profile quick
```
Output: Build time and validation time only.

### Standard Benchmark with JSON
```bash
./scripts/benchmark.sh --output json
```
Output: JSON format suitable for CI/CD integration.

### Full Benchmark and Save Results
```bash
./scripts/benchmark.sh --profile full --save
```
Output: Comprehensive benchmarks saved to timestamped JSON file.

### CI/CD Integration
```bash
# Run benchmark in CI and save results
./scripts/benchmark.sh --profile standard --output json --save

# Check for regressions
CURRENT_BUILD_TIME=$(jq '.build.time_seconds' benchmark-results-*.json | tail -1)
if (( CURRENT_BUILD_TIME > 600 )); then
  echo "Build time regression detected: ${CURRENT_BUILD_TIME}s > 600s"
  exit 1
fi
```

### Baseline Establishment
```bash
# Run multiple times for statistical confidence
for i in {1..5}; do
  ./scripts/benchmark.sh --profile full --save
  sleep 60  # Cool down between runs
done

# Analyze results
jq '.build.time_seconds' benchmark-results-*.json | awk '{sum+=$1; count++} END {print "Average:", sum/count}'
```

## Performance Targets

Targets from `docs/PRODUCTION_SCALE.md`:

| Metric | Target | Status Field |
|--------|--------|--------------|
| Bootstrap time | <1800s (30 min) | `bootstrap.status` |
| Build time | <600s (10 min) | `build.status` |
| Validation time | <300s (5 min) | `validation.status` |
| API latency P95 | <100ms | `api_latency.status` |
| Inference latency | <2000ms (2s) | `inference.status` |

Status values: `PASS`, `FAIL`, `WARN`, `ERROR`, `SKIPPED`

## Result Interpretation

### PASS
All benchmarks meet or exceed targets. System performance is adequate.

### FAIL
One or more benchmarks exceed targets. Consider:
- Resource allocation (CPU/RAM)
- Optimization (see `docs/cookbooks/PERFORMANCE_TUNING.md`)
- Load on system during benchmarking

### WARN
Benchmarks completed with warnings (e.g., validation errors). Investigate logs.

### ERROR
Benchmark failed to run. Check prerequisites and logs.

### SKIPPED
Benchmark not applicable (e.g., LocalAI not running).

## Trending and Regression Detection

### Save Results Over Time
```bash
# Weekly benchmark
0 2 * * 1 /path/to/benchmark.sh --profile standard --save
```

### Detect Regressions
```bash
# Compare current vs baseline
BASELINE_BUILD_TIME=450
CURRENT_BUILD_TIME=$(jq '.build.time_seconds' latest-results.json)

REGRESSION_PERCENT=$(awk "BEGIN {printf \"%.1f\", (($CURRENT_BUILD_TIME - $BASELINE_BUILD_TIME) / $BASELINE_BUILD_TIME) * 100}")

if (( $(echo "$REGRESSION_PERCENT > 10" | bc -l) )); then
  echo "Build time regression: +${REGRESSION_PERCENT}%"
fi
```

## References

- `docs/PRODUCTION_SCALE.md` - Performance targets and resource baselines
- `docs/cookbooks/PERFORMANCE_TUNING.md` - Optimization procedures
- `scripts/validate-e2e.sh` - End-to-end validation script
- `scripts/validate-resources.sh` - Resource validation script
