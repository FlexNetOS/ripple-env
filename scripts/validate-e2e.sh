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
for f in docker-compose*.yml .github/workflows/*.yml; do
    if [ -f "$f" ]; then
        run_test "YAML: $f" "python3 -c \"import yaml; yaml.safe_load(open('$f'))\""
    fi
done

# TOML validation
for f in pixi.toml; do
    if [ -f "$f" ]; then
        run_test "TOML: $f" "python3 -c \"import toml; toml.load(open('$f'))\""
    fi
done

# Nix validation
if [ -f "flake.nix" ]; then
    run_test "Nix: flake.nix syntax" "python3 -c \"
content = open('flake.nix').read()
assert content.count('{') == content.count('}'), 'Unbalanced braces'
assert 'inputs' in content and 'outputs' in content, 'Missing inputs/outputs'
\""
fi

echo ""

# =============================================================================
# Phase 2: Package Verification
# =============================================================================
log_info "Phase 2: Package Verification"
echo ""

# Check required packages in flake.nix
REQUIRED_NIX_PKGS="kubectl helm trivy syft grype cosign containerd neovim"
for pkg in $REQUIRED_NIX_PKGS; do
    run_test "Nix package: $pkg" "grep -q '$pkg' flake.nix"
done

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
for f in docker-compose*.yml; do
    if [ -f "$f" ]; then
        run_test "Docker Compose services: $f" "grep -q 'services:' $f"
    fi
done

# Check specific services exist
run_test "Service: prometheus" "grep -q 'prometheus' docker-compose.observability.yml"
run_test "Service: grafana" "grep -q 'grafana' docker-compose.observability.yml"
run_test "Service: nats" "grep -q 'nats' docker-compose.messaging.yml"
run_test "Service: temporal" "grep -q 'temporal' docker-compose.messaging.yml"
run_test "Service: opa" "grep -q 'opa' docker-compose.automation.yml"
run_test "Service: n8n" "grep -q 'n8n' docker-compose.automation.yml"
run_test "Service: kong" "grep -q 'kong' docker-compose.edge.yml"
run_test "Service: localai" "grep -q 'localai' docker-compose.localai.yml"

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
        run_test "Script executable: $script" "test -x $script"
    fi
done

echo ""

# =============================================================================
# Phase 6: Documentation Check
# =============================================================================
log_info "Phase 6: Documentation Check"
echo ""

run_test "README.md exists" "test -f README.md"
run_test "FINAL_REPORT.md exists" "test -f FINAL_REPORT.md"
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
