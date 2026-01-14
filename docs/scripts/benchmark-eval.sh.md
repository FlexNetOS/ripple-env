# Script Contract: benchmark-eval.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/benchmark-eval.sh`

---

## Purpose

Nix flake evaluation performance benchmarking tool. Measures evaluation time for key flake operations (flake show, devshell activation) and compares against target thresholds. Helps detect performance regressions in flake configuration as modularization progresses.

---

## Invocation

```bash
./scripts/benchmark-eval.sh [OPTIONS]
```

**Options:**
- `--verbose` - Show detailed output
- `--json` - Output results as JSON
- `--help` - Show help

**Examples:**
```bash
./scripts/benchmark-eval.sh                # Run all benchmarks
./scripts/benchmark-eval.sh --verbose      # Detailed timing info
./scripts/benchmark-eval.sh --json         # JSON output for CI
```

---

## Side Effects

**None** - Read-only benchmarking.

---

## Safety Classification

**🟢 SAFE** - Read-only evaluation benchmarks.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can run repeatedly.

---

## Key Features

### Benchmark Function (lines 29-62)

```bash
benchmark() {
    local name="$1"
    local target="$2"
    local cmd="$3"

    log_info "Benchmarking: $name"
    log_info "Target: <${target}s"

    # Run command and time it
    start_time=$(date +%s.%N)

    if [[ "$VERBOSE" == "true" ]]; then
        eval "$cmd"
    else
        eval "$cmd" > /dev/null 2>&1
    fi

    end_time=$(date +%s.%N)
    duration=$(echo "$end_time - $start_time" | bc)

    # Check against target
    if (( $(echo "$duration < $target" | bc -l) )); then
        log_success "$name: ${duration}s (target: <${target}s) ✓"
        echo "$name|$duration|$target|pass" >> "$RESULTS_FILE"
    else
        log_warn "$name: ${duration}s (target: <${target}s) ✗ SLOW"
        echo "$name|$duration|$target|fail" >> "$RESULTS_FILE"
    fi

    echo ""
}
```

**Key features:**
- Millisecond-precision timing via `date +%s.%N`
- Pass/fail against target thresholds
- Results logged to temp file for JSON output

### Benchmark Targets (lines 64-108)

**1. Flake Show (lines 71-73):**
```bash
benchmark "flake show" 10 \
    "nix flake show --all-systems"
```
**Target:** <10 seconds
**Purpose:** Measure time to evaluate all flake outputs

**2. Minimal Devshell (lines 75-77):**
```bash
benchmark "minimal shell" 5 \
    "nix develop .#minimal --command echo 'ok'"
```
**Target:** <5 seconds
**Purpose:** Measure minimal shell activation time

**3. Default Devshell (lines 79-81):**
```bash
benchmark "default shell" 15 \
    "nix develop --command echo 'ok'"
```
**Target:** <15 seconds
**Purpose:** Measure full devshell activation time

### JSON Output (lines 83-108)

```bash
if [[ "$JSON_OUTPUT" == "true" ]]; then
    echo "{"
    echo "  \"timestamp\": \"$(date -Iseconds)\","
    echo "  \"results\": ["

    local first=true
    while IFS='|' read -r name duration target status; do
        if [[ "$first" == "true" ]]; then
            first=false
        else
            echo ","
        fi

        echo "    {"
        echo "      \"name\": \"$name\","
        echo "      \"duration\": $duration,"
        echo "      \"target\": $target,"
        echo "      \"status\": \"$status\""
        echo -n "    }"
    done < "$RESULTS_FILE"

    echo ""
    echo "  ]"
    echo "}"
fi
```

**Example JSON output:**
```json
{
  "timestamp": "2026-01-13T10:30:45-08:00",
  "results": [
    {
      "name": "flake show",
      "duration": 8.234,
      "target": 10,
      "status": "pass"
    },
    {
      "name": "minimal shell",
      "duration": 3.891,
      "target": 5,
      "status": "pass"
    },
    {
      "name": "default shell",
      "duration": 12.456,
      "target": 15,
      "status": "pass"
    }
  ]
}
```

---

## Performance Targets

| Benchmark | Target | Rationale |
|-----------|--------|-----------|
| flake show | <10s | Quick overview of flake outputs |
| minimal shell | <5s | Fast activation for simple tasks |
| default shell | <15s | Acceptable for full environment |

**Modularization Goal:**
As `flake.nix` is modularized (per `docs/nix/NIX_FLAKE_MODULARIZATION.md`), these benchmarks should improve or remain constant. Any regression indicates excessive module complexity or circular dependencies.

---

## CI Integration

```yaml
# .github/workflows/benchmark.yml
- name: Benchmark Nix Evaluation
  run: |
    ./scripts/benchmark-eval.sh --json > benchmark-results.json

- name: Check Performance Targets
  run: |
    jq -e '.results[] | select(.status == "fail") | error("Performance regression detected")' benchmark-results.json
```

---

## References

- **Main script:** `scripts/benchmark-eval.sh` (108 lines)
- **Benchmark function:** lines 29-62
- **Targets:** lines 64-81
- **JSON output:** lines 83-108
- **Related:** `docs/nix/NIX_FLAKE_MODULARIZATION.md` (modularization plan)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 56/60 (93.3%)
