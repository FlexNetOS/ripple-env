#!/bin/bash
# =============================================================================
# Observability Stack Deployment Script (P1-008 & P1-009)
# =============================================================================
# This script deploys netdata and umami services to the observability stack.
#
# Usage: ./scripts/deploy-observability.sh
# =============================================================================

set -e

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================================================="
echo "  ARIA Observability Stack Deployment (P1-008 & P1-009)"
echo "========================================================================="
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}✗ Docker is not installed or not in PATH${NC}"
    exit 1
fi

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo -e "${RED}✗ Docker Compose is not available (docker compose / docker-compose)${NC}"
    exit 1
fi

resolve_compose_file() {
    # usage: resolve_compose_file docker-compose.observability.yml
    local base="$1"
    if [ -f "docker/$base" ]; then
        echo "docker/$base"
    else
        echo "$base"
    fi
}

OBS_COMPOSE_FILE="$(resolve_compose_file docker-compose.observability.yml)"

echo -e "${GREEN}✓ Docker is available${NC}"
echo ""

# Step 1: Check for .env file
echo "Step 1: Environment Configuration"
echo "-----------------------------------"
if [ -f .env ]; then
    echo -e "${GREEN}✓ .env file exists${NC}"
else
    echo -e "${YELLOW}⚠ .env file not found${NC}"
    echo "  Creating from .env.example..."
    cp .env.example .env
    echo -e "${GREEN}✓ Created .env file${NC}"
    echo ""
    echo -e "${YELLOW}⚠ WARNING: Using default credentials!${NC}"
    echo "  Generate secure secrets with:"
    echo "    openssl rand -base64 32"
    echo ""
fi

# Step 2: Check/Generate secrets
echo "Step 2: Security Configuration"
echo "-------------------------------"

# Check if secrets are still default
if grep -q "UMAMI_APP_SECRET=changeme" .env 2>/dev/null || \
   ! grep -q "UMAMI_APP_SECRET" .env 2>/dev/null; then
    echo -e "${YELLOW}⚠ Generating secure secrets...${NC}"

    gen_secret() {
        if command -v openssl >/dev/null 2>&1; then
            openssl rand -base64 32
        elif command -v python3 >/dev/null 2>&1; then
            python3 -c "import secrets,base64; print(base64.b64encode(secrets.token_bytes(32)).decode())"
        elif command -v python >/dev/null 2>&1; then
            python -c "import secrets,base64; print(base64.b64encode(secrets.token_bytes(32)).decode())"
        else
            return 1
        fi
    }

    UMAMI_APP_SECRET="$(gen_secret)" || { echo -e "${RED}✗ Unable to generate secrets (need openssl or python)${NC}"; exit 1; }
    UMAMI_DB_PASSWORD="$(gen_secret)" || { echo -e "${RED}✗ Unable to generate secrets (need openssl or python)${NC}"; exit 1; }

    # Update or append to .env
    if grep -q "UMAMI_APP_SECRET" .env; then
        sed -i "s|UMAMI_APP_SECRET=.*|UMAMI_APP_SECRET=${UMAMI_APP_SECRET}|" .env
    else
        echo "UMAMI_APP_SECRET=${UMAMI_APP_SECRET}" >> .env
    fi

    if grep -q "UMAMI_DB_PASSWORD" .env; then
        sed -i "s|UMAMI_DB_PASSWORD=.*|UMAMI_DB_PASSWORD=${UMAMI_DB_PASSWORD}|" .env
    else
        echo "UMAMI_DB_PASSWORD=${UMAMI_DB_PASSWORD}" >> .env
    fi

    echo -e "${GREEN}✓ Secure secrets generated and saved to .env${NC}"
else
    echo -e "${GREEN}✓ Secrets already configured${NC}"
fi

# Protect .env file
chmod 600 .env
echo -e "${GREEN}✓ .env file permissions set to 600${NC}"
echo ""

# Step 3: Create Docker network
echo "Step 3: Docker Network"
echo "----------------------"
if docker network inspect agentic-network >/dev/null 2>&1; then
    echo -e "${GREEN}✓ agentic-network already exists${NC}"
else
    docker network create agentic-network
    echo -e "${GREEN}✓ Created agentic-network${NC}"
fi
echo ""

# Step 4: Pull images
echo "Step 4: Pulling Docker Images"
echo "-----------------------------"
echo "Pulling netdata..."
"${COMPOSE[@]}" -f "$OBS_COMPOSE_FILE" pull netdata

echo "Pulling umami..."
"${COMPOSE[@]}" -f "$OBS_COMPOSE_FILE" pull umami umami-db

echo -e "${GREEN}✓ All images pulled${NC}"
echo ""

# Step 5: Deploy services
echo "Step 5: Deploying Services"
echo "--------------------------"
echo "Starting observability stack..."
"${COMPOSE[@]}" -f "$OBS_COMPOSE_FILE" up -d

echo -e "${GREEN}✓ Services started${NC}"
echo ""

# Step 6: Wait for services
echo "Step 6: Waiting for Services"
echo "----------------------------"
echo "Waiting 30 seconds for services to initialize..."
sleep 30

# Step 7: Verify deployment
echo ""
echo "Step 7: Verification"
echo "--------------------"

# Check netdata
echo -n "Checking Netdata... "
if docker ps | grep -q "netdata"; then
    echo -e "${GREEN}✓ Running${NC}"
else
    echo -e "${RED}✗ Not running${NC}"
fi

# Check umami
echo -n "Checking Umami... "
if docker ps | grep -q "umami"; then
    echo -e "${GREEN}✓ Running${NC}"
else
    echo -e "${RED}✗ Not running${NC}"
fi

# Check umami-db
echo -n "Checking Umami Database... "
if docker ps | grep -q "umami-db"; then
    echo -e "${GREEN}✓ Running${NC}"
else
    echo -e "${RED}✗ Not running${NC}"
fi

echo ""
echo "========================================================================="
echo "  Deployment Complete!"
echo "========================================================================="
echo ""
echo "Service URLs:"
echo "  Netdata (P1-008):  http://localhost:19999"
echo "  Umami (P1-009):    http://localhost:3001"
echo "  Grafana:           http://localhost:3000"
echo ""
echo "Default Credentials:"
echo "  Umami:   admin / umami  (⚠️ CHANGE IMMEDIATELY!)"
echo "  Grafana: admin / admin  (⚠️ CHANGE IMMEDIATELY!)"
echo ""
echo "Next Steps:"
echo "  1. Access Netdata: http://localhost:19999"
echo "  2. Login to Umami: http://localhost:3001"
echo "  3. Change default passwords"
echo "  4. Run verification: ./scripts/verify-observability.sh"
echo ""
echo "Documentation:"
echo "  - Implementation Report: docs/P1-008-009-IMPLEMENTATION.md"
echo "  - Quick Start Guide: docs/OBSERVABILITY-QUICK-START.md"
echo "  - Completion Summary: P1-008-009-COMPLETION-SUMMARY.md"
echo ""
echo "========================================================================="
