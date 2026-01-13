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

# Support both historical and current locations.
STATE_COMPOSE_FILE="docker-compose.state.yml"
if [ -f "docker/docker-compose.state.yml" ]; then
    STATE_COMPOSE_FILE="docker/docker-compose.state.yml"
fi

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

if [ -f "$STATE_COMPOSE_FILE" ]; then
    pass "$STATE_COMPOSE_FILE exists"
else
    fail "docker-compose.state.yml not found (expected $STATE_COMPOSE_FILE)"
fi

if [ -f ".env.state.example" ]; then
    pass ".env.state.example exists"
else
    fail ".env.state.example not found"
fi

# RuVector is currently consumed via npm/npx (embedded/local DB CLI).
# The CLI also includes a `server` subcommand with HTTP/gRPC options; if you
# plan to use it as a networked service, verify it by starting it and probing
# an endpoint (e.g. GET /health) in your target version/environment.
if [ -f ".npmrc" ]; then
    pass ".npmrc exists (repo-local npm config)"
else
    warn ".npmrc not found (npx may be broken if global npm config points to a missing drive)"
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
    if docker compose -f "$STATE_COMPOSE_FILE" config --quiet 2>/dev/null; then
        pass "$STATE_COMPOSE_FILE syntax is valid"
    else
        fail "$STATE_COMPOSE_FILE has syntax errors"
        docker compose -f "$STATE_COMPOSE_FILE" config 2>&1 | head -10
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
    if docker compose -f "$STATE_COMPOSE_FILE" ps redis 2>/dev/null | grep -q "Up"; then
        pass "Redis is running"

        # Test Redis connection
        if docker compose -f "$STATE_COMPOSE_FILE" exec redis redis-cli ping 2>/dev/null | grep -q "PONG"; then
            pass "Redis responds to PING"
        else
            warn "Redis is running but not responding"
        fi
    else
        warn "Redis is not running"
        echo "   Start with: docker compose -f $STATE_COMPOSE_FILE up -d redis"
    fi

    if docker compose -f "$STATE_COMPOSE_FILE" ps minio 2>/dev/null | grep -q "Up"; then
        pass "MinIO is running"

        # Test MinIO health
        if curl -sf http://localhost:9000/minio/health/live >/dev/null 2>&1; then
            pass "MinIO health check passed"
        else
            warn "MinIO is running but health check failed"
        fi
    else
        warn "MinIO is not running"
        echo "   Start with: docker compose -f $STATE_COMPOSE_FILE up -d minio mc"
    fi
else
    warn "Skipping service status check (Docker not available)"
fi

echo ""

# =============================================================================
# 5. Check ruvector Installation
# =============================================================================

echo "[5/5] Checking ruvector (npm/npx CLI)..."
echo ""

if command -v node >/dev/null 2>&1; then
    pass "Node.js is installed"
else
    fail "Node.js not found"
    warn "Install Node.js to use ruvector via npx"
fi

if command -v npx >/dev/null 2>&1; then
    pass "npx is available"

    # Some environments have a global npm prefix/cache pointing to a missing drive.
    # Use repo-local locations for this script's npx invocations.
    REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    mkdir -p "$REPO_ROOT/.npm-cache" "$REPO_ROOT/.npm-prefix" >/dev/null 2>&1 || true
    export NPM_CONFIG_CACHE="${NPM_CONFIG_CACHE:-$REPO_ROOT/.npm-cache}"
    export NPM_CONFIG_PREFIX="${NPM_CONFIG_PREFIX:-$REPO_ROOT/.npm-prefix}"

    # Attempt to run ruvector CLI. This may download the package on first run.
    if RUVECTOR_VERSION=$(npx --yes ruvector --version 2>/dev/null); then
        pass "ruvector CLI is runnable via npx"
        echo "   Version: $RUVECTOR_VERSION"

        if npx --yes ruvector doctor >/dev/null 2>&1; then
            pass "ruvector doctor passed"
        else
            warn "ruvector doctor reported issues"
        fi
    else
        warn "ruvector CLI did not run via npx"
        echo "   Hint: if npm prefix/cache points to a missing drive, use scripts/ruvector.(ps1|sh) or set NPM_CONFIG_PREFIX/NPM_CONFIG_CACHE"
    fi
else
    fail "npx not found"
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
    echo "  1. Start services: docker compose -f $STATE_COMPOSE_FILE up -d"
    echo "  2. Test Redis: docker compose -f $STATE_COMPOSE_FILE exec redis redis-cli ping"
    echo "  3. Access MinIO Console: http://localhost:9001 (minioadmin/minioadmin)"
    echo "  4. Run ruvector CLI: npx --yes ruvector --help"
    exit 0
else
    echo -e "${RED}Some checks failed.${NC}"
    echo ""
    echo "Review the errors above and consult L10_STATE_STORAGE_IMPLEMENTATION.md"
    exit 1
fi
