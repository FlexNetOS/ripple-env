#!/usr/bin/env bash
# =============================================================================
# Layer 10 State & Storage Verification Script
# =============================================================================
# Verifies that P0-005 (Redis), P1-001 (ruvector), and P1-006 (MinIO)
# are properly installed and configured.
#
# Usage:
#   chmod +x scripts/verify-state-storage.sh
#   ./scripts/verify-state-storage.sh
# =============================================================================

set -e

echo "========================================"
echo "Layer 10 State & Storage Verification"
echo "========================================"
echo ""

PASSED=0
FAILED=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper functions
pass() {
    echo -e "${GREEN}✓${NC} $1"
    ((PASSED++))
}

fail() {
    echo -e "${RED}✗${NC} $1"
    ((FAILED++))
}

warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# =============================================================================
# 1. Check Files Exist
# =============================================================================

echo "[1/5] Checking configuration files..."
echo ""

if [ -f "docker-compose.state.yml" ]; then
    pass "docker-compose.state.yml exists"
else
    fail "docker-compose.state.yml not found"
fi

if [ -f ".env.state.example" ]; then
    pass ".env.state.example exists"
else
    fail ".env.state.example not found"
fi

if [ -f "rust/Cargo.toml" ]; then
    if grep -q "ruvector" rust/Cargo.toml; then
        pass "ruvector dependency found in Cargo.toml"
    else
        fail "ruvector not found in Cargo.toml"
    fi

    if grep -q "redis" rust/Cargo.toml; then
        pass "redis dependency found in Cargo.toml"
    else
        fail "redis not found in Cargo.toml"
    fi
else
    fail "rust/Cargo.toml not found"
fi

echo ""

# =============================================================================
# 2. Check Docker Availability
# =============================================================================

echo "[2/5] Checking Docker..."
echo ""

if command -v docker >/dev/null 2>&1; then
    pass "Docker is installed"
    DOCKER_VERSION=$(docker --version)
    echo "   Version: $DOCKER_VERSION"

    if docker compose version >/dev/null 2>&1; then
        pass "Docker Compose is available"
    else
        fail "Docker Compose not found"
    fi
else
    fail "Docker not found in PATH"
    warn "Install Docker to run Redis and MinIO services"
fi

echo ""

# =============================================================================
# 3. Validate Docker Compose File
# =============================================================================

echo "[3/5] Validating docker-compose.state.yml..."
echo ""

if command -v docker >/dev/null 2>&1; then
    if docker compose -f docker-compose.state.yml config --quiet 2>/dev/null; then
        pass "docker-compose.state.yml syntax is valid"
    else
        fail "docker-compose.state.yml has syntax errors"
        docker compose -f docker-compose.state.yml config 2>&1 | head -10
    fi

    # Check network
    if docker network inspect agentic-network >/dev/null 2>&1; then
        pass "Docker network 'agentic-network' exists"
    else
        warn "Docker network 'agentic-network' not found"
        echo "   Create with: docker network create agentic-network"
    fi
else
    warn "Skipping Docker Compose validation (Docker not available)"
fi

echo ""

# =============================================================================
# 4. Check Service Status
# =============================================================================

echo "[4/5] Checking service status..."
echo ""

if command -v docker >/dev/null 2>&1; then
    # Check if services are running
    if docker compose -f docker-compose.state.yml ps redis 2>/dev/null | grep -q "Up"; then
        pass "Redis is running"

        # Test Redis connection
        if docker compose -f docker-compose.state.yml exec redis redis-cli ping 2>/dev/null | grep -q "PONG"; then
            pass "Redis responds to PING"
        else
            warn "Redis is running but not responding"
        fi
    else
        warn "Redis is not running"
        echo "   Start with: docker compose -f docker-compose.state.yml up -d redis"
    fi

    if docker compose -f docker-compose.state.yml ps minio 2>/dev/null | grep -q "Up"; then
        pass "MinIO is running"

        # Test MinIO health
        if curl -sf http://localhost:9000/minio/health/live >/dev/null 2>&1; then
            pass "MinIO health check passed"
        else
            warn "MinIO is running but health check failed"
        fi
    else
        warn "MinIO is not running"
        echo "   Start with: docker compose -f docker-compose.state.yml up -d minio mc"
    fi
else
    warn "Skipping service status check (Docker not available)"
fi

echo ""

# =============================================================================
# 5. Check ruvector Installation
# =============================================================================

echo "[5/5] Checking ruvector..."
echo ""

if command -v cargo >/dev/null 2>&1; then
    pass "Cargo is installed"

    # Check if ruvector-cli is installed
    if command -v ruvector-cli >/dev/null 2>&1; then
        pass "ruvector-cli is installed"
        RUVECTOR_VERSION=$(ruvector-cli --version 2>/dev/null || echo "unknown")
        echo "   Version: $RUVECTOR_VERSION"
    else
        warn "ruvector-cli not installed"
        echo "   Install with: cargo install ruvector-cli"
    fi

    # Check if ruvector library is accessible
    cd rust
    if cargo tree 2>/dev/null | grep -q "ruvector"; then
        pass "ruvector library is in dependency tree"
    else
        warn "ruvector library not resolved (may need network access to GitHub)"
    fi
    cd ..
else
    fail "Cargo not found"
    warn "Install Rust toolchain to use ruvector"
fi

echo ""

# =============================================================================
# Summary
# =============================================================================

echo "========================================"
echo "Verification Summary"
echo "========================================"
echo ""
echo -e "Passed: ${GREEN}$PASSED${NC}"
echo -e "Failed: ${RED}$FAILED${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}All checks passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Start services: docker compose -f docker-compose.state.yml up -d"
    echo "  2. Test Redis: docker compose -f docker-compose.state.yml exec redis redis-cli ping"
    echo "  3. Access MinIO Console: http://localhost:9001 (minioadmin/minioadmin)"
    echo "  4. Install ruvector CLI: cargo install ruvector-cli"
    exit 0
else
    echo -e "${RED}Some checks failed.${NC}"
    echo ""
    echo "Review the errors above and consult L10_STATE_STORAGE_IMPLEMENTATION.md"
    exit 1
fi
