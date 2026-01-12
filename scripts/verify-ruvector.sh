#!/usr/bin/env bash
# =============================================================================
# RuVector Integration Verification Script
# =============================================================================
# BUILDKIT_STARTER_SPEC.md Layer 10: State & Storage (P1-001)
#
# Verifies that RuVector is properly installed and configured:
#   - Rust dependencies in Cargo.toml
#   - Docker Compose configuration
#   - Environment variables
#   - Pixi feature flags
#   - Nix packages
#
# Usage:
#   chmod +x scripts/verify-ruvector.sh
#   ./scripts/verify-ruvector.sh
# =============================================================================

set -e

echo "========================================"
echo "RuVector Integration Verification"
echo "========================================"
echo ""

PASSED=0
FAILED=0
WARNINGS=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
pass() {
    echo -e "${GREEN}✓${NC} $1"
    PASSED=$((PASSED + 1))
}

fail() {
    echo -e "${RED}✗${NC} $1"
    FAILED=$((FAILED + 1))
}

warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    WARNINGS=$((WARNINGS + 1))
}

info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# =============================================================================
# 1. Check Rust Dependencies
# =============================================================================

echo "[1/6] Checking Rust dependencies..."
echo ""

if [ -f "rust/Cargo.toml" ]; then
    if grep -q 'ruvector.*=.*{.*git.*=.*"https://github.com/ruvnet/ruvector"' rust/Cargo.toml; then
        pass "ruvector git dependency found in Cargo.toml"
    else
        fail "ruvector git dependency not found in Cargo.toml"
    fi

    if grep -q 'redis.*=.*{.*version.*=.*"0.27"' rust/Cargo.toml; then
        pass "redis client dependency found in Cargo.toml"
    else
        fail "redis client dependency not found in Cargo.toml"
    fi
else
    fail "rust/Cargo.toml not found"
fi

echo ""

# =============================================================================
# 2. Check Docker Compose Configuration
# =============================================================================

echo "[2/6] Checking Docker Compose configuration..."
echo ""

if [ -f "docker-compose.ruvector.yml" ]; then
    pass "docker-compose.ruvector.yml exists"

    if command -v docker >/dev/null 2>&1; then
        if docker compose -f docker-compose.ruvector.yml config --quiet 2>/dev/null; then
            pass "docker-compose.ruvector.yml syntax is valid"
        else
            fail "docker-compose.ruvector.yml has syntax errors"
        fi
    else
        warn "Docker not available, skipping compose validation"
    fi
else
    fail "docker-compose.ruvector.yml not found"
fi

if [ -f "docker-compose.state.yml" ] || [ -f "docker/docker-compose.state.yml" ]; then
    pass "docker-compose.state.yml exists (includes Redis)"
else
    warn "docker-compose.state.yml not found"
fi

echo ""

# =============================================================================
# 3. Check Environment Configuration
# =============================================================================

echo "[3/6] Checking environment configuration..."
echo ""

if [ -f ".env.state.example" ]; then
    pass ".env.state.example exists"

    if grep -q "RUVECTOR_ENDPOINT" .env.state.example; then
        pass "RUVECTOR_ENDPOINT variable defined"
    else
        fail "RUVECTOR_ENDPOINT variable not defined"
    fi

    if grep -q "RUVECTOR_GNN_ENABLED" .env.state.example; then
        pass "RUVECTOR_GNN_ENABLED variable defined"
    else
        fail "RUVECTOR_GNN_ENABLED variable not defined"
    fi

    if grep -q "VECTOR_STORE" .env.state.example; then
        pass "VECTOR_STORE feature flag defined"
    else
        fail "VECTOR_STORE feature flag not defined"
    fi
else
    fail ".env.state.example not found"
fi

echo ""

# =============================================================================
# 4. Check Pixi Feature Flags
# =============================================================================

echo "[4/6] Checking Pixi feature flags..."
echo ""

if [ -f "pixi.toml" ]; then
    if grep -q '\[feature\.vectordb-ruvector\]' pixi.toml; then
        pass "vectordb-ruvector feature defined in pixi.toml"
    else
        fail "vectordb-ruvector feature not defined in pixi.toml"
    fi

    if grep -q '\[feature\.vectordb-chromadb\]' pixi.toml; then
        pass "vectordb-chromadb feature defined in pixi.toml"
    else
        warn "vectordb-chromadb feature not defined (A/B testing unavailable)"
    fi

    if grep -q 'vectordb-ruvector.*=.*{.*features.*=.*\["vectordb-ruvector"\]' pixi.toml; then
        pass "vectordb-ruvector environment defined"
    else
        fail "vectordb-ruvector environment not defined"
    fi
else
    fail "pixi.toml not found"
fi

echo ""

# =============================================================================
# 5. Check Nix Packages
# =============================================================================

echo "[5/6] Checking Nix packages..."
echo ""

if [ -f "nix/packages/distributed.nix" ]; then
    pass "nix/packages/distributed.nix exists"

    if grep -q "grpcurl" nix/packages/distributed.nix; then
        pass "gRPC tools included in distributed.nix"
    else
        warn "gRPC tools not found in distributed.nix"
    fi

    if grep -q "wasm-pack" nix/packages/distributed.nix; then
        pass "WASM tools included in distributed.nix"
    else
        warn "WASM tools not found in distributed.nix"
    fi
else
    fail "nix/packages/distributed.nix not found"
fi

if [ -f "nix/packages/default.nix" ]; then
    if grep -q "distributed" nix/packages/default.nix; then
        pass "distributed packages imported in default.nix"
    else
        fail "distributed packages not imported in default.nix"
    fi
else
    fail "nix/packages/default.nix not found"
fi

echo ""

# =============================================================================
# 6. Check Service Status (if running)
# =============================================================================

echo "[6/6] Checking service status..."
echo ""

if command -v docker >/dev/null 2>&1; then
    if docker compose -f docker-compose.ruvector.yml ps 2>/dev/null | grep -q "ruvector-primary.*Up"; then
        pass "RuVector primary node is running"

        # Test health endpoint
        if curl -sf http://localhost:8000/health >/dev/null 2>&1; then
            pass "RuVector health check passed"
        else
            warn "RuVector health check failed (service may still be starting)"
        fi
    else
        info "RuVector not running (start with: docker compose -f docker-compose.ruvector.yml up -d)"
    fi
else
    info "Docker not available, skipping service status check"
fi

echo ""

# =============================================================================
# Summary
# =============================================================================

echo "========================================"
echo "Verification Summary"
echo "========================================"
echo ""
echo -e "Passed:   ${GREEN}$PASSED${NC}"
echo -e "Failed:   ${RED}$FAILED${NC}"
echo -e "Warnings: ${YELLOW}$WARNINGS${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}RuVector integration verification passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Start RuVector: docker compose -f docker-compose.ruvector.yml up -d"
    echo "  2. Test health: curl http://localhost:8000/health"
    echo "  3. Access gRPC: grpcurl -plaintext localhost:8001 list"
    echo "  4. View metrics: curl http://localhost:8002/metrics"
    exit 0
else
    echo -e "${RED}RuVector integration verification failed.${NC}"
    echo ""
    echo "Review the errors above and consult:"
    echo "  - docs/implementation/L10_STATE_STORAGE_IMPLEMENTATION.md"
    echo "  - docs/audits/RUVECTOR_QUDAG_AUDIT_REPORT.md"
    exit 1
fi
