#!/usr/bin/env bash
# =============================================================================
# QuDAG Integration Verification Script
# =============================================================================
# BUILDKIT_STARTER_SPEC.md Layer 11: P2P Coordination (P3-011)
#
# Verifies that QuDAG is properly installed and configured:
#   - Rust dependencies in Cargo.toml
#   - Docker Compose configuration
#   - Environment variables
#   - Pixi feature flags
#   - MCP server configuration
#   - Network configuration
#
# Usage:
#   chmod +x scripts/verify-qudag.sh
#   ./scripts/verify-qudag.sh
# =============================================================================

set -e

echo "========================================"
echo "QuDAG Integration Verification"
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

echo "[1/8] Checking Rust dependencies..."
echo ""

if [ -f "rust/Cargo.toml" ]; then
    if grep -q 'qudag-core.*=.*{.*git.*=.*"https://github.com/ruvnet/qudag"' rust/Cargo.toml; then
        pass "qudag-core dependency found in Cargo.toml"
    else
        fail "qudag-core dependency not found in Cargo.toml"
    fi

    if grep -q 'qudag-crypto.*=.*{.*git.*=.*"https://github.com/ruvnet/qudag"' rust/Cargo.toml; then
        pass "qudag-crypto dependency found in Cargo.toml"
    else
        fail "qudag-crypto dependency not found in Cargo.toml"
    fi

    if grep -q 'qudag-network.*=.*{.*git.*=.*"https://github.com/ruvnet/qudag"' rust/Cargo.toml; then
        pass "qudag-network dependency found in Cargo.toml"
    else
        fail "qudag-network dependency not found in Cargo.toml"
    fi

    if grep -q 'qudag-mcp.*=.*{.*git.*=.*"https://github.com/ruvnet/qudag"' rust/Cargo.toml; then
        pass "qudag-mcp dependency found in Cargo.toml"
    else
        fail "qudag-mcp dependency not found in Cargo.toml"
    fi

    # Check crypto libraries
    if grep -q 'blake3.*=.*"1.5"' rust/Cargo.toml; then
        pass "blake3 hash library found"
    else
        warn "blake3 hash library not found"
    fi

    if grep -q 'chacha20poly1305.*=.*"0.10"' rust/Cargo.toml; then
        pass "chacha20poly1305 cipher found"
    else
        warn "chacha20poly1305 cipher not found"
    fi
else
    fail "rust/Cargo.toml not found"
fi

echo ""

# =============================================================================
# 2. Check Docker Compose Configuration
# =============================================================================

echo "[2/8] Checking Docker Compose configuration..."
echo ""

if [ -f "docker-compose.qudag.yml" ]; then
    pass "docker-compose.qudag.yml exists"

    if command -v docker >/dev/null 2>&1; then
        if docker compose -f docker-compose.qudag.yml config --quiet 2>/dev/null; then
            pass "docker-compose.qudag.yml syntax is valid"
        else
            fail "docker-compose.qudag.yml has syntax errors"
        fi
    else
        warn "Docker not available, skipping compose validation"
    fi

    # Check for services
    if grep -q "qudag-node:" docker-compose.qudag.yml; then
        pass "qudag-node service defined"
    else
        fail "qudag-node service not defined"
    fi

    if grep -q "qudag-mcp:" docker-compose.qudag.yml; then
        pass "qudag-mcp service defined"
    else
        warn "qudag-mcp service not defined (MCP profile only)"
    fi
else
    fail "docker-compose.qudag.yml not found"
fi

echo ""

# =============================================================================
# 3. Check Environment Configuration
# =============================================================================

echo "[3/8] Checking environment configuration..."
echo ""

if [ -f ".env.qudag.example" ]; then
    pass ".env.qudag.example exists"

    if grep -q "QUDAG_CRYPTO_SUITE" .env.qudag.example; then
        pass "QUDAG_CRYPTO_SUITE variable defined"
    else
        fail "QUDAG_CRYPTO_SUITE variable not defined"
    fi

    if grep -q "QUDAG_CONSENSUS_ALGORITHM" .env.qudag.example; then
        pass "QUDAG_CONSENSUS_ALGORITHM variable defined"
    else
        fail "QUDAG_CONSENSUS_ALGORITHM variable not defined"
    fi

    if grep -q "QUDAG_MCP_ENABLED" .env.qudag.example; then
        pass "QUDAG_MCP_ENABLED variable defined"
    else
        fail "QUDAG_MCP_ENABLED variable not defined"
    fi

    if grep -q "QUDAG_ONION_ENABLED" .env.qudag.example; then
        pass "QUDAG_ONION_ENABLED variable defined"
    else
        fail "QUDAG_ONION_ENABLED variable not defined"
    fi
else
    fail ".env.qudag.example not found"
fi

echo ""

# =============================================================================
# 4. Check Pixi Feature Flags
# =============================================================================

echo "[4/8] Checking Pixi feature flags..."
echo ""

if [ -f "pixi.toml" ]; then
    if grep -q '\[feature\.qudag\]' pixi.toml; then
        pass "qudag feature defined in pixi.toml"
    else
        fail "qudag feature not defined in pixi.toml"
    fi

    if grep -q 'websockets.*>=.*11.0' pixi.toml; then
        pass "websockets dependency found for MCP WebSocket"
    else
        warn "websockets dependency not found"
    fi

    if grep -q 'mcp.*>=.*1.0' pixi.toml; then
        pass "mcp SDK dependency found"
    else
        warn "mcp SDK dependency not found"
    fi

    if grep -q 'qudag.*=.*{.*features.*=.*\["qudag"\]' pixi.toml; then
        pass "qudag environment defined"
    else
        fail "qudag environment not defined"
    fi
else
    fail "pixi.toml not found"
fi

echo ""

# =============================================================================
# 5. Check MCP Server Configuration
# =============================================================================

echo "[5/8] Checking MCP server configuration..."
echo ""

if [ -f "manifests/mcp/qudag-server.json" ]; then
    pass "manifests/mcp/qudag-server.json exists"

    if command -v jq >/dev/null 2>&1; then
        # Validate JSON syntax
        if jq . manifests/mcp/qudag-server.json >/dev/null 2>&1; then
            pass "qudag-server.json is valid JSON"

            # Check for required tools
            TOOLS_COUNT=$(jq '.tools | length' manifests/mcp/qudag-server.json)
            if [ "$TOOLS_COUNT" -ge 10 ]; then
                pass "MCP server has $TOOLS_COUNT tools defined"
            else
                warn "MCP server has only $TOOLS_COUNT tools (expected >= 10)"
            fi

            # Check for resources
            RESOURCES_COUNT=$(jq '.resources | length' manifests/mcp/qudag-server.json)
            if [ "$RESOURCES_COUNT" -ge 5 ]; then
                pass "MCP server has $RESOURCES_COUNT resources defined"
            else
                warn "MCP server has only $RESOURCES_COUNT resources (expected >= 5)"
            fi
        else
            fail "qudag-server.json has JSON syntax errors"
        fi
    else
        warn "jq not available, skipping JSON validation"
    fi
else
    fail "manifests/mcp/qudag-server.json not found"
fi

echo ""

# =============================================================================
# 6. Check Network Configuration
# =============================================================================

echo "[6/8] Checking network configuration..."
echo ""

if [ -f "manifests/qudag/networks.json" ]; then
    pass "manifests/qudag/networks.json exists"

    if command -v jq >/dev/null 2>&1; then
        if jq . manifests/qudag/networks.json >/dev/null 2>&1; then
            pass "networks.json is valid JSON"

            # Check for network definitions
            if jq -e '.networks.local' manifests/qudag/networks.json >/dev/null 2>&1; then
                pass "local network defined"
            else
                fail "local network not defined"
            fi

            if jq -e '.networks.testnet' manifests/qudag/networks.json >/dev/null 2>&1; then
                pass "testnet network defined"
            else
                fail "testnet network not defined"
            fi
        else
            fail "networks.json has JSON syntax errors"
        fi
    else
        warn "jq not available, skipping JSON validation"
    fi
else
    fail "manifests/qudag/networks.json not found"
fi

echo ""

# =============================================================================
# 7. Check Documentation
# =============================================================================

echo "[7/8] Checking documentation..."
echo ""

if [ -f "docs/implementation/QUDAG_IMPLEMENTATION.md" ]; then
    pass "QUDAG_IMPLEMENTATION.md exists"
else
    fail "QUDAG_IMPLEMENTATION.md not found"
fi

if [ -f "docs/audits/RUVECTOR_QUDAG_AUDIT_REPORT.md" ]; then
    pass "RUVECTOR_QUDAG_AUDIT_REPORT.md exists"
else
    warn "RUVECTOR_QUDAG_AUDIT_REPORT.md not found"
fi

echo ""

# =============================================================================
# 8. Check Service Status (if running)
# =============================================================================

echo "[8/8] Checking service status..."
echo ""

if command -v docker >/dev/null 2>&1; then
    if docker compose -f docker-compose.qudag.yml ps 2>/dev/null | grep -q "qudag-node.*Up"; then
        pass "QuDAG node is running"

        # Test health endpoint
        if curl -sf http://localhost:9000/health >/dev/null 2>&1; then
            pass "QuDAG health check passed"
        else
            warn "QuDAG health check failed (service may still be starting)"
        fi

        # Test MCP endpoint
        if curl -sf http://localhost:9000/mcp >/dev/null 2>&1; then
            pass "QuDAG MCP endpoint accessible"
        else
            warn "QuDAG MCP endpoint not accessible"
        fi
    else
        info "QuDAG not running (start with: docker compose -f docker-compose.qudag.yml up -d)"
    fi

    # Check network
    if docker network inspect agentic-network >/dev/null 2>&1; then
        pass "agentic-network Docker network exists"
    else
        warn "agentic-network not found (create with: docker network create agentic-network)"
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
    echo -e "${GREEN}QuDAG integration verification passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Create network: docker network create agentic-network"
    echo "  2. Start QuDAG: docker compose -f docker-compose.qudag.yml up -d"
    echo "  3. Test health: curl http://localhost:9000/health"
    echo "  4. Test MCP: curl -X POST http://localhost:9000/mcp -H 'Content-Type: application/json' -d '{\"jsonrpc\":\"2.0\",\"method\":\"tools/list\",\"id\":1}'"
    echo "  5. Connect to testnet: docker compose -f docker-compose.qudag.yml --profile testnet up -d"
    exit 0
else
    echo -e "${RED}QuDAG integration verification failed.${NC}"
    echo ""
    echo "Review the errors above and consult:"
    echo "  - docs/implementation/QUDAG_IMPLEMENTATION.md"
    echo "  - docs/audits/RUVECTOR_QUDAG_AUDIT_REPORT.md"
    exit 1
fi
