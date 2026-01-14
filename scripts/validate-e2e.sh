#!/usr/bin/env bash
# =============================================================================
# Local End-to-End Validation Script
# =============================================================================
# Run this script to validate the entire stack locally before pushing.
# This mirrors the GitHub Actions workflow for local testing.
# =============================================================================

set -euo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[FAIL]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

is_windows_like() {
    case "$(uname -s 2>/dev/null || echo unknown)" in
        MINGW*|MSYS*|CYGWIN*) return 0 ;;
        *) return 1 ;;
    esac
}

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi

resolve_compose_file() {
    # usage: resolve_compose_file docker-compose.foo.yml
    local base="$1"
    if [ -f "docker/$base" ]; then
        echo "docker/$base"
    else
        echo "$base"
    fi
}

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
    # Use an isolated venv so we don't mutate global Python installs.
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

echo "============================================================================="
echo "END-TO-END VALIDATION"
echo "============================================================================="
echo ""

# =============================================================================
# Phase 1: Configuration Validation
# =============================================================================
log_info "Phase 1: Configuration Validation"
echo ""

# YAML validation
# 1) Compose files: validate using docker compose config (more accurate than generic YAML parsing)
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

# 2) Non-compose YAML: validate via Python (PyYAML) when available.
PY_CHECKS_READY=0
if ensure_python_deps; then
    PY_CHECKS_READY=1
fi
for f in .github/workflows/*.yml; do
    if [ -f "$f" ]; then
        if [ "$PY_CHECKS_READY" -eq 1 ]; then
            run_test "YAML: $f" "$VENV_PY -c \"import yaml; yaml.safe_load(open('$f', 'r', encoding='utf-8'))\""
        else
            log_warn "Skipping YAML parse check (missing python deps): $f"
        fi
    fi
done

# TOML validation
for f in pixi.toml; do
    if [ -f "$f" ]; then
        if [ "$PY_CHECKS_READY" -eq 1 ]; then
            run_test "TOML: $f" "$VENV_PY -c \"import toml; toml.load(open('$f', 'r', encoding='utf-8'))\""
        else
            log_warn "Skipping TOML parse check (missing python deps): $f"
        fi
    fi
done

# Nix validation
if [ -f "flake.nix" ]; then
    if command -v nix >/dev/null 2>&1; then
        run_test "Nix: flake check" "nix flake check"
    else
        log_warn "nix not found; skipping 'nix flake check'"
    fi
fi

echo ""

# =============================================================================
# Phase 2: Package Verification
# =============================================================================
log_info "Phase 2: Package Verification"
echo ""

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

echo ""

# =============================================================================
# Phase 3: Docker Compose Validation
# =============================================================================
log_info "Phase 3: Docker Compose Structure Validation"
echo ""


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

echo ""

# =============================================================================
# Phase 4: OPA Policy Validation
# =============================================================================
log_info "Phase 4: OPA Policy Validation"
echo ""

if [ -f "config/opa/policies/authz.rego" ]; then
    run_test "OPA policy: authz.rego exists" "test -f config/opa/policies/authz.rego"
    run_test "OPA policy: has package declaration" "grep -q 'package ros2.authz' config/opa/policies/authz.rego"
    run_test "OPA policy: has default deny" "grep -q 'default allow := false' config/opa/policies/authz.rego"
fi

echo ""

# =============================================================================
# Phase 5: Script Validation
# =============================================================================
log_info "Phase 5: Script Validation"
echo ""

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

echo ""

# =============================================================================
# Phase 6: Documentation Check
# =============================================================================
log_info "Phase 6: Documentation Check"
echo ""

run_test "README.md exists" "test -f README.md"
run_test "FINAL_REPORT.md exists" "test -f docs/reports/FINAL_REPORT.md"
run_test "docs/ROS2_STATE_PACKAGES.md exists" "test -f docs/ROS2_STATE_PACKAGES.md"

echo ""

# =============================================================================
# Results Summary
# =============================================================================
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
