#!/bin/bash
# =============================================================================
# Observability Stack Verification Script
# =============================================================================
# This script verifies that P1-008 (netdata) and P1-009 (umami) are properly
# configured and running in the observability stack.
#
# Usage: ./scripts/verify-observability.sh
# =============================================================================

set -e

ERRORS=0

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================================================="
echo "  ARIA Observability Stack Verification (P1-008 & P1-009)"
echo "========================================================================="
echo ""

# Function to check if service is running
check_service() {
    local service_name=$1
    local container_name=$2

    echo -n "Checking ${service_name}... "

    if docker ps --filter "name=${container_name}" --filter "status=running" | grep -q "${container_name}"; then
        echo -e "${GREEN}✓ Running${NC}"
        return 0
    else
        echo -e "${RED}✗ Not running${NC}"
        return 1
    fi
}

# Function to check endpoint availability
check_endpoint() {
    local service_name=$1
    local url=$2
    local expected_code=${3:-200}

    echo -n "Checking ${service_name} endpoint (${url})... "

    if curl -f -s -o /dev/null -w "%{http_code}" "${url}" | grep -q "${expected_code}"; then
        echo -e "${GREEN}✓ Accessible${NC}"
        return 0
    else
        echo -e "${RED}✗ Not accessible${NC}"
        return 1
    fi
}

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}✗ Docker is not installed or not in PATH${NC}"
    exit 1
fi

echo "=== Docker Services ==="
echo ""

# Check Netdata (P1-008)
if ! check_service "Netdata" "netdata"; then
    ERRORS=$((ERRORS + 1))
fi

# Check Umami (P1-009)
if ! check_service "Umami" "umami"; then
    ERRORS=$((ERRORS + 1))
fi
if ! check_service "Umami Database" "umami-db"; then
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "=== Service Endpoints ==="
echo ""

# Check Netdata endpoint
if ! check_endpoint "Netdata Dashboard" "http://localhost:19999"; then
    ERRORS=$((ERRORS + 1))
fi

# Check Umami endpoint
if ! check_endpoint "Umami Dashboard" "http://localhost:3001/api/heartbeat"; then
    ERRORS=$((ERRORS + 1))
fi

# Check Grafana (should include Netdata datasource)
if ! check_endpoint "Grafana" "http://localhost:3000/api/health"; then
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "=== Environment Variables ==="
echo ""

# Check if .env file exists
if [ -f .env ]; then
    echo -e "${GREEN}✓ .env file exists${NC}"

    # Check for required Umami variables
    if grep -q "UMAMI_DB_USER" .env && \
       grep -q "UMAMI_DB_PASSWORD" .env && \
       grep -q "UMAMI_APP_SECRET" .env; then
        echo -e "${GREEN}✓ Umami environment variables configured${NC}"
    else
        echo -e "${YELLOW}⚠ Some Umami environment variables missing${NC}"
    fi

    # Check for Netdata variable (optional)
    if grep -q "NETDATA_CLAIM_TOKEN" .env; then
        echo -e "${GREEN}✓ Netdata claim token configured${NC}"
    else
        echo -e "${YELLOW}⚠ Netdata claim token not configured (optional)${NC}"
    fi
else
    echo -e "${YELLOW}⚠ .env file not found - copy from .env.example${NC}"
fi

echo ""
echo "=== Docker Volumes ==="
echo ""

# Check Netdata volumes
if docker volume ls | grep -q "netdata-config"; then
    echo -e "${GREEN}✓ Netdata volumes created${NC}"
else
    echo -e "${YELLOW}⚠ Netdata volumes not found${NC}"
fi

# Check Umami volume
if docker volume ls | grep -q "umami-data"; then
    echo -e "${GREEN}✓ Umami volume created${NC}"
else
    echo -e "${YELLOW}⚠ Umami volume not found${NC}"
fi

echo ""
echo "=== Service Details ==="
echo ""

echo "Netdata (P1-008):"
echo "  - URL: http://localhost:19999"
echo "  - Container: netdata"
echo "  - Image: netdata/netdata:v2.1.0"
echo "  - Features: Real-time system monitoring, Docker metrics"
echo ""

echo "Umami (P1-009):"
echo "  - URL: http://localhost:3001"
echo "  - Container: umami"
echo "  - Image: ghcr.io/umami-software/umami:postgresql-latest"
echo "  - Database: PostgreSQL (umami-db)"
echo "  - Features: Privacy-focused web analytics"
echo ""

echo "=== Next Steps ==="
echo ""
echo "1. Access Netdata dashboard:"
echo "   http://localhost:19999"
echo ""
echo "2. Access Umami dashboard:"
echo "   http://localhost:3001"
echo "   Default login: admin / umami (change on first login)"
echo ""
echo "3. Configure Netdata Cloud (optional):"
echo "   - Get claim token from https://app.netdata.cloud"
echo "   - Add to .env: NETDATA_CLAIM_TOKEN=<your-token>"
OBS_COMPOSE_FILE="docker-compose.observability.yml"
if [ -f "docker/docker-compose.observability.yml" ]; then
    OBS_COMPOSE_FILE="docker/docker-compose.observability.yml"
fi
echo "   - Restart: docker compose -f $OBS_COMPOSE_FILE restart netdata"
echo ""
echo "4. View metrics in Grafana:"
echo "   http://localhost:3000"
echo "   - Netdata datasource: http://localhost:19999/api/v1/allmetrics?format=prometheus"
echo ""

echo "========================================================================="
echo "  Verification Complete"
echo "========================================================================="

if [ "$ERRORS" -gt 0 ]; then
    echo -e "${YELLOW}⚠ Completed with $ERRORS failing check(s).${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All checks passed.${NC}"
