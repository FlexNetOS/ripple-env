#!/usr/bin/env bash
# Verify mTLS setup for ARIA
# This script checks that all components are properly configured

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "🔐 ARIA mTLS Setup Verification"
echo "==============================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

pass() {
    echo -e "${GREEN}✓${NC} $1"
}

fail() {
    echo -e "${RED}✗${NC} $1"
}

warn() {
    echo -e "${YELLOW}⚠${NC} $1"
}

ERRORS=0

echo "1. Checking step-cli installation..."
if command -v step &> /dev/null; then
    VERSION=$(step version 2>&1 | head -1)
    pass "step-cli installed: $VERSION"
else
    fail "step-cli not found. Run: nix develop"
    ERRORS=$((ERRORS + 1))
fi
echo ""

echo "2. Checking Step-CA configuration files..."
if [ -f "$PROJECT_ROOT/config/step-ca/ca.json" ]; then
    pass "CA configuration exists: config/step-ca/ca.json"
else
    fail "CA configuration missing: config/step-ca/ca.json"
    ERRORS=$((ERRORS + 1))
fi

if [ -f "$PROJECT_ROOT/config/step-ca/defaults.json" ]; then
    pass "Client defaults exist: config/step-ca/defaults.json"
else
    fail "Client defaults missing: config/step-ca/defaults.json"
    ERRORS=$((ERRORS + 1))
fi
echo ""

echo "3. Checking PKI certificates..."
if [ -f "$PROJECT_ROOT/config/step-ca/pki/root_ca.crt" ]; then
    pass "Root CA certificate exists"
    if command -v step &> /dev/null; then
        EXPIRY=$(step certificate inspect "$PROJECT_ROOT/config/step-ca/pki/root_ca.crt" --format json 2>/dev/null | jq -r '.validity.end // "unknown"')
        echo "   Valid until: $EXPIRY"
    fi
else
    warn "Root CA not initialized. Run: ./scripts/init-step-ca.sh"
fi

if [ -f "$PROJECT_ROOT/config/step-ca/pki/intermediate_ca.crt" ]; then
    pass "Intermediate CA certificate exists"
else
    warn "Intermediate CA not initialized. Run: ./scripts/init-step-ca.sh"
fi
echo ""

echo "4. Checking Step-CA secrets..."
if [ -f "$PROJECT_ROOT/config/step-ca/secrets/password.txt" ]; then
    pass "CA password file exists"
    PERMS=$(stat -c %a "$PROJECT_ROOT/config/step-ca/secrets/password.txt" 2>/dev/null || stat -f %A "$PROJECT_ROOT/config/step-ca/secrets/password.txt" 2>/dev/null || echo "unknown")
    if [ "$PERMS" == "600" ]; then
        pass "Password file has correct permissions (600)"
    else
        warn "Password file permissions are $PERMS (should be 600)"
    fi
else
    warn "CA password not generated. Run: ./scripts/init-step-ca.sh"
fi
echo ""

echo "5. Checking Step-CA service..."
if command -v docker &> /dev/null; then
    if docker ps --format '{{.Names}}' | grep -q "^step-ca$"; then
        pass "Step-CA container is running"

        # Check health
        HEALTH=$(docker inspect --format='{{.State.Health.Status}}' step-ca 2>/dev/null || echo "unknown")
        if [ "$HEALTH" == "healthy" ]; then
            pass "Step-CA is healthy"
        else
            warn "Step-CA health status: $HEALTH"
        fi
    else
        warn "Step-CA container not running. Start with:"
        echo "   docker-compose -f docker-compose.identity.yml up -d step-ca"
    fi
else
    warn "Docker not available - cannot check Step-CA service"
fi
echo ""

echo "6. Checking documentation..."
if [ -f "$PROJECT_ROOT/docs/MTLS_SETUP.md" ]; then
    pass "mTLS documentation exists: docs/MTLS_SETUP.md"
else
    fail "mTLS documentation missing"
    ERRORS=$((ERRORS + 1))
fi
echo ""

echo "7. Checking service certificate directories..."
CERT_DIR="$PROJECT_ROOT/data/certs"
if [ -d "$CERT_DIR" ]; then
    pass "Certificate directory exists: $CERT_DIR"

    # Count certificates
    CERT_COUNT=$(find "$CERT_DIR" -name "*.crt" -type f | wc -l)
    if [ "$CERT_COUNT" -gt 0 ]; then
        pass "Found $CERT_COUNT service certificate(s)"
    else
        warn "No service certificates found. Generate with:"
        echo "   ./scripts/generate-service-certs.sh"
    fi
else
    warn "Certificate directory not created: $CERT_DIR"
fi
echo ""

echo "8. Checking docker-compose configuration..."
if grep -q "step-ca:" "$PROJECT_ROOT/docker-compose.identity.yml"; then
    pass "Step-CA service configured in docker-compose.identity.yml"
else
    fail "Step-CA not configured in docker-compose.identity.yml"
    ERRORS=$((ERRORS + 1))
fi
echo ""

echo "==============================="
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✓ All checks passed!${NC}"
    echo ""
    echo "🚀 Quick Start:"
    echo "   # Initialize CA (if not done):"
    echo "   ./scripts/init-step-ca.sh"
    echo ""
    echo "   # Start Step-CA:"
    echo "   docker-compose -f docker-compose.identity.yml up -d step-ca"
    echo ""
    echo "   # Generate service certificates:"
    echo "   ./scripts/generate-service-certs.sh"
    echo ""
    echo "   # Enable mTLS in docker-compose files and restart services"
    echo ""
else
    echo -e "${RED}✗ Found $ERRORS error(s)${NC}"
    echo ""
    echo "Please fix the errors above before proceeding."
    exit 1
fi
