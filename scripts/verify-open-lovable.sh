#!/usr/bin/env bash
# =============================================================================
# Open-Lovable Verification Script (P2-012)
# =============================================================================
# This script verifies the open-lovable integration with the ARIA UI stack
# Usage: ./scripts/verify-open-lovable.sh
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
PASSED=0
FAILED=0
WARNINGS=0

# Helper functions
print_header() {
    echo ""
    echo -e "${BLUE}============================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}============================================================${NC}"
    echo ""
}

print_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

print_pass() {
    echo -e "${GREEN}[PASS]${NC} $1"
    PASSED=$((PASSED + 1))
}

print_fail() {
    echo -e "${RED}[FAIL]${NC} $1"
    FAILED=$((FAILED + 1))
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    WARNINGS=$((WARNINGS + 1))
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Change to repository root
cd "$(dirname "$0")/.." || exit 1

# Support both historical and current compose locations.
UI_COMPOSE_FILE="docker-compose.ui.yml"
if [ -f "docker/docker-compose.ui.yml" ]; then
    UI_COMPOSE_FILE="docker/docker-compose.ui.yml"
fi

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi

print_header "P2-012: Open-Lovable Integration Verification"

# =============================================================================
# Test 1: File Existence
# =============================================================================
print_header "Test 1: File Existence"

print_test "Checking Dockerfile..."
if [ -f "config/dockerfiles/Dockerfile.open-lovable" ]; then
    print_pass "Dockerfile exists at config/dockerfiles/Dockerfile.open-lovable"
else
    print_fail "Dockerfile not found at config/dockerfiles/Dockerfile.open-lovable"
fi

print_test "Checking docker-compose.ui.yml..."
if [ -f "$UI_COMPOSE_FILE" ]; then
    print_pass "$UI_COMPOSE_FILE exists"
else
    print_fail "docker-compose.ui.yml not found (checked: $UI_COMPOSE_FILE)"
fi

print_test "Checking .env.example..."
if [ -f ".env.example" ]; then
    print_pass ".env.example exists"
else
    print_fail ".env.example not found"
fi

print_test "Checking documentation..."
if [ -f "docs/P2-012-OPEN-LOVABLE-INTEGRATION.md" ]; then
    print_pass "Documentation exists at docs/P2-012-OPEN-LOVABLE-INTEGRATION.md"
else
    print_fail "Documentation not found"
fi

# =============================================================================
# Test 2: Configuration Validation
# =============================================================================
print_header "Test 2: Configuration Validation"

print_test "Checking for open-lovable service in docker-compose.ui.yml..."
if [ -f "$UI_COMPOSE_FILE" ] && grep -q "open-lovable:" "$UI_COMPOSE_FILE"; then
    print_pass "open-lovable service defined in $UI_COMPOSE_FILE"
else
    print_fail "open-lovable service not found in $UI_COMPOSE_FILE"
fi

print_test "Checking for Dockerfile build configuration..."
if [ -f "$UI_COMPOSE_FILE" ] && grep -q "dockerfile: Dockerfile.open-lovable" "$UI_COMPOSE_FILE"; then
    print_pass "Dockerfile build configuration found"
else
    print_fail "Dockerfile build configuration not found"
fi

print_test "Checking for port mapping (3211:3000)..."
if [ -f "$UI_COMPOSE_FILE" ] && grep -q "3211:3000" "$UI_COMPOSE_FILE"; then
    print_pass "Port mapping 3211:3000 configured"
else
    print_fail "Port mapping not found"
fi

print_test "Checking for LocalAI dependency..."
if [ -f "$UI_COMPOSE_FILE" ]; then
    # LocalAI is often started separately on the shared external network.
    # Accept either an explicit depends_on OR an OPENAI_* URL pointing at `http://localai:8080`.
    if grep -A 12 "open-lovable:" "$UI_COMPOSE_FILE" | grep -q "depends_on" && \
       grep -A 20 "open-lovable:" "$UI_COMPOSE_FILE" | grep -q "localai"; then
        print_pass "LocalAI dependency configured (depends_on)"
    elif grep -A 40 "open-lovable:" "$UI_COMPOSE_FILE" | grep -Eq "OPENAI_(BASE_URL|PROXY_URL).*localai:8080"; then
        print_pass "LocalAI configured via OPENAI_BASE_URL/OPENAI_PROXY_URL (external service)"
    else
        print_fail "LocalAI dependency/config not found (expected depends_on or OPENAI_* pointing to localai:8080)"
    fi
else
    print_fail "Compose file missing; cannot verify LocalAI configuration"
fi

print_test "Checking for volumes..."
if [ -f "$UI_COMPOSE_FILE" ] && grep -q "open-lovable-data:" "$UI_COMPOSE_FILE" && \
   grep -q "open-lovable-projects:" "$UI_COMPOSE_FILE"; then
    print_pass "Volumes configured (open-lovable-data, open-lovable-projects)"
else
    print_fail "Volumes not properly configured"
fi

print_test "Checking environment variables in .env.example..."
if grep -q "OPEN_LOVABLE_VERSION" .env.example && \
   grep -q "OPENAI_API_KEY" .env.example && \
   grep -q "OPENAI_BASE_URL" .env.example; then
    print_pass "Environment variables configured in .env.example"
else
    print_fail "Environment variables missing in .env.example"
fi

# =============================================================================
# Test 3: Docker Network
# =============================================================================
print_header "Test 3: Docker Network"

print_test "Checking if agentic-network exists..."
if docker network inspect agentic-network &>/dev/null; then
    print_pass "agentic-network exists"
else
    print_warn "agentic-network does not exist - run: docker network create agentic-network"
fi

# =============================================================================
# Test 4: Docker Compose Validation
# =============================================================================
print_header "Test 4: Docker Compose Validation"

print_test "Validating docker-compose.ui.yml syntax..."
if [ ${#COMPOSE[@]} -gt 0 ] && [ -f "$UI_COMPOSE_FILE" ] && "${COMPOSE[@]}" -f "$UI_COMPOSE_FILE" config &>/dev/null; then
    print_pass "$UI_COMPOSE_FILE syntax is valid"
else
    print_fail "$UI_COMPOSE_FILE has syntax errors or Docker Compose is unavailable"
    if [ ${#COMPOSE[@]} -gt 0 ] && [ -f "$UI_COMPOSE_FILE" ]; then
        "${COMPOSE[@]}" -f "$UI_COMPOSE_FILE" config || true
    fi
fi

# =============================================================================
# Test 5: Optional Runtime Tests (if service is running)
# =============================================================================
print_header "Test 5: Runtime Tests (Optional)"

print_test "Checking if open-lovable container exists..."
if docker ps -a --format '{{.Names}}' | grep -q "^open-lovable$"; then
    print_pass "open-lovable container exists"

    print_test "Checking if open-lovable container is running..."
    if docker ps --format '{{.Names}}' | grep -q "^open-lovable$"; then
        print_pass "open-lovable container is running"

        print_test "Checking health status..."
        HEALTH_STATUS=$(docker inspect --format='{{.State.Health.Status}}' open-lovable 2>/dev/null || echo "unknown")
        if [ "$HEALTH_STATUS" = "healthy" ]; then
            print_pass "Container is healthy"
        elif [ "$HEALTH_STATUS" = "starting" ]; then
            print_warn "Container is still starting..."
        elif [ "$HEALTH_STATUS" = "unknown" ]; then
            print_warn "Health status unknown (may not have health check)"
        else
            print_fail "Container is unhealthy: $HEALTH_STATUS"
        fi

        print_test "Checking port accessibility..."
        if curl -s -o /dev/null -w "%{http_code}" http://localhost:3211 &>/dev/null; then
            HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3211)
            if [ "$HTTP_CODE" = "200" ] || [ "$HTTP_CODE" = "301" ] || [ "$HTTP_CODE" = "302" ]; then
                print_pass "Application is accessible on http://localhost:3211 (HTTP $HTTP_CODE)"
            else
                print_warn "Application returned HTTP $HTTP_CODE"
            fi
        else
            print_fail "Application is not accessible on http://localhost:3211"
        fi

        print_test "Checking logs for errors..."
        ERROR_COUNT=$(docker logs open-lovable 2>&1 | grep -ci "error")
        if [ "$ERROR_COUNT" -eq 0 ]; then
            print_pass "No errors found in logs"
        else
            print_warn "Found $ERROR_COUNT error messages in logs"
        fi
    else
        print_warn "Container exists but is not running - run: ${COMPOSE[*]:-docker compose} -f $UI_COMPOSE_FILE up -d open-lovable"
    fi
else
    print_info "Container does not exist yet - build and start with:"
    print_info "  ${COMPOSE[*]:-docker compose} -f $UI_COMPOSE_FILE up -d --build open-lovable"
fi

print_test "Checking if LocalAI is running..."
if docker ps --format '{{.Names}}' | grep -q "localai"; then
    print_pass "LocalAI container is running"
else
    print_warn "LocalAI is not running - open-lovable requires LocalAI"
    print_info "  Start LocalAI: docker-compose -f docker-compose.localai.yml up -d"
fi

# =============================================================================
# Test 6: Volume Inspection
# =============================================================================
print_header "Test 6: Volume Inspection"

print_test "Checking for open-lovable volumes..."
VOLUME_COUNT=$(docker volume ls --format '{{.Name}}' | grep -c "open-lovable" || true)
if [ "$VOLUME_COUNT" -gt 0 ]; then
    print_pass "Found $VOLUME_COUNT open-lovable volumes"
    docker volume ls --format 'table {{.Name}}\t{{.Driver}}' | grep "open-lovable" || true
else
    print_info "No volumes created yet (volumes are created on first run)"
fi

# =============================================================================
# Summary
# =============================================================================
print_header "Verification Summary"

echo ""
echo -e "${GREEN}Passed:${NC}   $PASSED"
echo -e "${RED}Failed:${NC}   $FAILED"
echo -e "${YELLOW}Warnings:${NC} $WARNINGS"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All critical tests passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Ensure LocalAI is running: ${COMPOSE[*]:-docker compose} -f docker-compose.localai.yml up -d"
    echo "  2. Build and start open-lovable: ${COMPOSE[*]:-docker compose} -f $UI_COMPOSE_FILE up -d --build open-lovable"
    echo "  3. Access the application: http://localhost:3211"
    echo "  4. Check logs: ${COMPOSE[*]:-docker compose} -f $UI_COMPOSE_FILE logs -f open-lovable"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please review the output above.${NC}"
    echo ""
    exit 1
fi
