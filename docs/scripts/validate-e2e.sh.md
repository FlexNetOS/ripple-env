# Script Contract: validate-e2e.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/validate-e2e.sh`

---

## Purpose

Local end-to-end validation script that mirrors GitHub Actions CI/CD workflow. Validates configuration files, package definitions, Docker Compose structure, OPA policies, script syntax, and documentation. Designed for pre-commit checks and local testing before pushing to repository.

---

## Invocation

```bash
./scripts/validate-e2e.sh
```

**No arguments required** - Runs all validation phases automatically.

**Requirements:**
- Docker Compose v2 or v1 (optional, skips if unavailable)
- Python 3 with venv support (optional, skips YAML/TOML parsing if unavailable)
- Nix (optional, skips flake check if unavailable)
- Bash 4.0+ (for associative arrays)

---

## Outputs

**Standard Output (success):**
```
=============================================================================
END-TO-END VALIDATION
=============================================================================

[INFO] Phase 1: Configuration Validation

[PASS] Compose config: docker-compose.yml
[PASS] Compose config: docker-compose.identity.yml
[PASS] YAML: .github/workflows/ci.yml
[PASS] TOML: pixi.toml
[PASS] Nix: flake check

[INFO] Phase 2: Package Verification

[PASS] Nix package: kubectl
[PASS] Nix package: helm
[PASS] Pixi package: jupyterlab
[PASS] Pixi package: mlflow

[INFO] Phase 3: Docker Compose Structure Validation

[PASS] Docker Compose services: docker-compose.yml
[PASS] Service: prometheus
[PASS] Service: grafana
[PASS] Service: nats

[INFO] Phase 4: OPA Policy Validation

[PASS] OPA policy: authz.rego exists
[PASS] OPA policy: has package declaration
[PASS] OPA policy: has default deny

[INFO] Phase 5: Script Validation

[PASS] Script syntax: scripts/deploy.sh
[PASS] Script executable: scripts/deploy.sh

[INFO] Phase 6: Documentation Check

[PASS] README.md exists
[PASS] FINAL_REPORT.md exists
[PASS] docs/ROS2_STATE_PACKAGES.md exists

=============================================================================
VALIDATION RESULTS
=============================================================================

Total Tests: 45
Passed: 45
Failed: 0

✅ ALL TESTS PASSED

RESULT: PASS
WHY: All 45 validation tests passed
NEXT: Ready for deployment or CI/CD
```

**Standard Output (with failures):**
```
=============================================================================
END-TO-END VALIDATION
=============================================================================

[INFO] Phase 1: Configuration Validation

[PASS] Compose config: docker-compose.yml
[FAIL] Compose config: docker-compose.identity.yml
[WARN] nix not found; skipping 'nix flake check'

...

=============================================================================
VALIDATION RESULTS
=============================================================================

Total Tests: 45
Passed: 40
Failed: 5

❌ SOME TESTS FAILED

RESULT: FAIL
WHY: 5 out of 45 tests failed
NEXT: Fix failing tests before proceeding
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all tests passed) |
| `1` | Failure (one or more tests failed) |

---

## Side Effects

**None** - Read-only validation script.

**Temporary venv:** Creates `.tmp/e2e-venv/` for Python dependencies (lines 62-94), but isolated from global Python.

---

## Safety Classification

**🟢 SAFE** - Read-only validation, no modifications.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Validation Phases

### Phase 1: Configuration Validation (lines 123-176)

**Evidence:**
```bash
log_info "Phase 1: Configuration Validation"

# Docker Compose validation
COMPOSE_FILES=(docker-compose*.yml docker/docker-compose*.yml docker/*.yml)
for f in "${COMPOSE_FILES[@]}"; do
    if [ -f "$f" ]; then
        if [ ${#COMPOSE[@]} -gt 0 ]; then
            run_test "Compose config: $f" "${COMPOSE[*]} -f '$f' config"
        else
            log_warn "Docker Compose not available; skipping compose validation for $f"
        fi
    fi
done

# Non-compose YAML validation via PyYAML
for f in .github/workflows/*.yml; do
    if [ -f "$f" ]; then
        if [ "$PY_CHECKS_READY" -eq 1 ]; then
            run_test "YAML: $f" "$VENV_PY -c \"import yaml; yaml.safe_load(open('$f', 'r', encoding='utf-8'))\""
        fi
    fi
done

# TOML validation
for f in pixi.toml; do
    if [ -f "$f" ]; then
        if [ "$PY_CHECKS_READY" -eq 1 ]; then
            run_test "TOML: $f" "$VENV_PY -c \"import toml; toml.load(open('$f', 'r', encoding='utf-8'))\""
        fi
    fi
done

# Nix flake validation
if [ -f "flake.nix" ]; then
    if command -v nix >/dev/null 2>&1; then
        run_test "Nix: flake check" "nix flake check"
    fi
fi
```

**Checks:**
1. Docker Compose files: `docker compose -f <file> config` (lines 130-139)
2. GitHub workflow YAML: PyYAML parsing (lines 146-154)
3. pixi.toml: TOML parsing (lines 157-165)
4. flake.nix: `nix flake check` (lines 168-174)

**Graceful degradation:** Skips checks if tools unavailable (Docker Compose, Python, Nix)

### Phase 2: Package Verification (lines 178-201)

**Evidence:**
```bash
log_info "Phase 2: Package Verification"

# Check required packages in flake.nix
if command -v nix >/dev/null 2>&1; then
    REQUIRED_NIX_PKGS="kubectl helm trivy syft grype cosign containerd neovim"
    for pkg in $REQUIRED_NIX_PKGS; do
        run_test "Nix package: $pkg" "grep -q '$pkg' flake.nix"
    done
else
    log_warn "nix not found; skipping Nix package verification"
fi

# Check required packages in pixi.toml
REQUIRED_PIXI_PKGS="jupyterlab mlflow transformers esbuild datafusion"
for pkg in $REQUIRED_PIXI_PKGS; do
    run_test "Pixi package: $pkg" "grep -q '$pkg' pixi.toml"
done
```

**Nix packages checked (line 186):**
- kubectl, helm - Kubernetes tools
- trivy, syft, grype, cosign - Security scanning
- containerd - Container runtime
- neovim - Text editor

**Pixi packages checked (line 195):**
- jupyterlab - Notebooks
- mlflow - ML experiment tracking
- transformers - NLP models
- esbuild - JavaScript bundler
- datafusion - Query engine

### Phase 3: Docker Compose Structure Validation (lines 203-232)

**Evidence:**
```bash
log_info "Phase 3: Docker Compose Structure Validation"

# Check Docker Compose files have required sections
for f in docker-compose*.yml docker/docker-compose*.yml; do
    if [ -f "$f" ]; then
        run_test "Docker Compose services: $f" "grep -q 'services:' '$f'"
    fi
done

# Check specific services exist
OBS_COMPOSE="$(resolve_compose_file docker-compose.observability.yml)"
MSG_COMPOSE="$(resolve_compose_file docker-compose.messaging.yml)"
AUTO_COMPOSE="$(resolve_compose_file docker-compose.automation.yml)"
EDGE_COMPOSE="$(resolve_compose_file docker-compose.edge.yml)"
LOCALAI_COMPOSE="$(resolve_compose_file docker-compose.localai.yml)"

run_test "Service: prometheus" "test -f '$OBS_COMPOSE' && grep -q 'prometheus' '$OBS_COMPOSE'"
run_test "Service: grafana" "test -f '$OBS_COMPOSE' && grep -q 'grafana' '$OBS_COMPOSE'"
run_test "Service: nats" "test -f '$MSG_COMPOSE' && grep -q 'nats' '$MSG_COMPOSE'"
run_test "Service: temporal" "test -f '$MSG_COMPOSE' && grep -q 'temporal' '$MSG_COMPOSE'"
run_test "Service: opa" "test -f '$AUTO_COMPOSE' && grep -q 'opa' '$AUTO_COMPOSE'"
run_test "Service: n8n" "test -f '$AUTO_COMPOSE' && grep -q 'n8n' '$AUTO_COMPOSE'"
run_test "Service: kong" "test -f '$EDGE_COMPOSE' && grep -q 'kong' '$EDGE_COMPOSE'"
run_test "Service: localai" "test -f '$LOCALAI_COMPOSE' && grep -q 'localai' '$LOCALAI_COMPOSE'"
```

**Services verified:**
- Observability: prometheus, grafana
- Messaging: nats, temporal
- Automation: opa, n8n
- Edge: kong
- AI: localai

**Compose file resolution (lines 43-51):** Checks `docker/` subdirectory first, then root

### Phase 4: OPA Policy Validation (lines 234-246)

**Evidence:**
```bash
log_info "Phase 4: OPA Policy Validation"

if [ -f "config/opa/policies/authz.rego" ]; then
    run_test "OPA policy: authz.rego exists" "test -f config/opa/policies/authz.rego"
    run_test "OPA policy: has package declaration" "grep -q 'package ros2.authz' config/opa/policies/authz.rego"
    run_test "OPA policy: has default deny" "grep -q 'default allow := false' config/opa/policies/authz.rego"
fi
```

**Checks:**
1. Policy file exists (line 241)
2. Package declaration present (line 242)
3. Default deny rule present (line 243)

### Phase 5: Script Validation (lines 248-265)

**Evidence:**
```bash
log_info "Phase 5: Script Validation"

for script in scripts/*.sh; do
    if [ -f "$script" ]; then
        run_test "Script syntax: $script" "bash -n $script"
        if is_windows_like; then
            log_warn "Skipping executable-bit check on Windows: $script"
        else
            run_test "Script executable: $script" "test -x $script"
        fi
    fi
done
```

**Checks:**
1. Bash syntax validation: `bash -n` (line 256)
2. Executable bit check: `test -x` (line 260, skipped on Windows)

**Windows detection (lines 28-33):** MINGW/MSYS/CYGWIN patterns

### Phase 6: Documentation Check (lines 267-277)

**Evidence:**
```bash
log_info "Phase 6: Documentation Check"

run_test "README.md exists" "test -f README.md"
run_test "FINAL_REPORT.md exists" "test -f docs/reports/FINAL_REPORT.md"
run_test "docs/ROS2_STATE_PACKAGES.md exists" "test -f docs/ROS2_STATE_PACKAGES.md"
```

**Required documentation:**
- README.md (project overview)
- FINAL_REPORT.md (audit report)
- ROS2_STATE_PACKAGES.md (state management docs)

---

## Test Tracking System

**Evidence:** Lines 96-115

```bash
# Results tracking
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

run_test() {
    local name="$1"
    local cmd="$2"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))

    if eval "$cmd" > /dev/null 2>&1; then
        log_success "$name"
        PASSED_TESTS=$((PASSED_TESTS + 1))
        return 0
    else
        log_error "$name"
        FAILED_TESTS=$((FAILED_TESTS + 1))
        return 1
    fi
}
```

**Behavior:**
- Increments `TOTAL_TESTS` counter (line 104)
- Evaluates command silently (line 106)
- Increments `PASSED_TESTS` or `FAILED_TESTS` (lines 108, 112)
- Returns 0 (pass) or 1 (fail)

---

## Python Virtual Environment

**Evidence:** Lines 53-94

```bash
PYTHON=${PYTHON:-}
if [ -z "$PYTHON" ]; then
    if command -v python3 >/dev/null 2>&1; then
        PYTHON=python3
    else
        PYTHON=python
    fi
fi

VENV_DIR=".tmp/e2e-venv"
VENV_PY=""
ensure_python_deps() {
    if [ -z "$PYTHON" ] || ! command -v "$PYTHON" >/dev/null 2>&1; then
        log_warn "Python not found; skipping YAML/TOML parsing checks"
        return 1
    fi

    if [ ! -d "$VENV_DIR" ]; then
        mkdir -p "$(dirname "$VENV_DIR")" || true
        "$PYTHON" -m venv "$VENV_DIR" >/dev/null 2>&1 || {
            log_warn "Unable to create venv at $VENV_DIR; skipping YAML/TOML parsing checks"
            return 1
        }
    fi

    if [ -f "$VENV_DIR/Scripts/python.exe" ]; then
        VENV_PY="$VENV_DIR/Scripts/python.exe"
    elif [ -f "$VENV_DIR/bin/python" ]; then
        VENV_PY="$VENV_DIR/bin/python"
    else
        log_warn "Venv python not found; skipping YAML/TOML parsing checks"
        return 1
    fi

    "$VENV_PY" -m pip -q install -r scripts/requirements.txt >/dev/null 2>&1 || {
        log_warn "Failed installing python deps from scripts/requirements.txt; skipping YAML/TOML parsing checks"
        return 1
    }

    return 0
}
```

**Purpose:** Isolated venv prevents global Python pollution

**Python detection:** python3 → python fallback (lines 54-60)

**Platform-aware paths:**
- Windows: `.tmp/e2e-venv/Scripts/python.exe` (line 79)
- Unix: `.tmp/e2e-venv/bin/python` (line 81)

**Dependencies:** Installs from `scripts/requirements.txt` (line 88)

**Graceful degradation:** Returns 1 if venv creation fails, allows script to continue

---

## Docker Compose Detection

**Evidence:** Lines 35-41

```bash
# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi
```

**Preference order:**
1. `docker compose` (v2 plugin) - lines 37-38
2. `docker-compose` (v1 standalone) - lines 39-40

**Empty array:** If neither available, `${#COMPOSE[@]} -eq 0` (checked at line 133)

---

## Results Summary

**Evidence:** Lines 282-305

```bash
echo "============================================================================="
echo "VALIDATION RESULTS"
echo "============================================================================="
echo ""
echo "Total Tests: $TOTAL_TESTS"
echo -e "Passed: ${GREEN}$PASSED_TESTS${NC}"
echo -e "Failed: ${RED}$FAILED_TESTS${NC}"
echo ""

if [ $FAILED_TESTS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED${NC}"
    echo ""
    echo "RESULT: PASS"
    echo "WHY: All $TOTAL_TESTS validation tests passed"
    echo "NEXT: Ready for deployment or CI/CD"
    exit 0
else
    echo -e "${RED}❌ SOME TESTS FAILED${NC}"
    echo ""
    echo "RESULT: FAIL"
    echo "WHY: $FAILED_TESTS out of $TOTAL_TESTS tests failed"
    echo "NEXT: Fix failing tests before proceeding"
    exit 1
fi
```

**Clear verdict:** RESULT, WHY, NEXT sections (lines 294-303)

**Color-coded summary:** Green for pass, red for fail (lines 287-288)

---

## Integration with CI/CD

**Purpose:** Mirrors GitHub Actions workflow locally

**Usage in pre-commit hook:**
```bash
#!/bin/bash
# .git/hooks/pre-commit
./scripts/validate-e2e.sh || {
    echo "Validation failed! Commit blocked."
    exit 1
}
```

**Usage in CI:**
```yaml
# .github/workflows/ci.yml
- name: Validate Configuration
  run: ./scripts/validate-e2e.sh
```

---

## References

### Source Code
- **Main script:** `scripts/validate-e2e.sh` (306 lines)
- **Test runner:** lines 101-115
- **Python venv:** lines 53-94
- **Compose detection:** lines 35-41
- **Phase 1 (Config):** lines 123-176
- **Phase 2 (Packages):** lines 178-201
- **Phase 3 (Compose):** lines 203-232
- **Phase 4 (OPA):** lines 234-246
- **Phase 5 (Scripts):** lines 248-265
- **Phase 6 (Docs):** lines 267-277
- **Results:** lines 282-305

### Related Files
- **Python dependencies:** `scripts/requirements.txt` (PyYAML, toml)
- **OPA policy:** `config/opa/policies/authz.rego`
- **Documentation:** `README.md`, `docs/reports/FINAL_REPORT.md`, `docs/ROS2_STATE_PACKAGES.md`

### External Resources
- [Docker Compose Config Command](https://docs.docker.com/compose/reference/config/)
- [PyYAML](https://pyyaml.org/)
- [Nix Flake Check](https://nixos.org/manual/nix/stable/command-ref/new-cli/nix3-flake-check.html)
- [OPA Policies](https://www.openpolicyagent.org/docs/latest/policy-language/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 36/60 contracts complete (60%)
