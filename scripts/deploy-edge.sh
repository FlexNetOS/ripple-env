#!/bin/bash
# Edge Services Deployment Script
# Deploys P0-003 (Kong Gateway) and P0-004 (AgentGateway)
# Author: L4 Edge & Agent Traffic Domain Team Lead
# Version: 1.0.0

set -e

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
COMPOSE_FILE="$PROJECT_ROOT/docker-compose.edge.yml"

# Helper functions
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_failure() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# Check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"

    # Check if Docker is installed
    if ! command -v docker &> /dev/null; then
        print_failure "Docker is not installed"
        exit 1
    fi
    print_success "Docker is installed: $(docker --version)"

    # Check if Docker Compose is available
    if ! docker compose version &> /dev/null; then
        print_failure "Docker Compose is not available"
        exit 1
    fi
    print_success "Docker Compose is available: $(docker compose version)"

    # Check if docker-compose.edge.yml exists
    if [ ! -f "$COMPOSE_FILE" ]; then
        print_failure "docker-compose.edge.yml not found at $COMPOSE_FILE"
        exit 1
    fi
    print_success "docker-compose.edge.yml found"
}

# Create network
create_network() {
    print_header "Creating Docker Network"

    if docker network ls | grep -q agentic-network; then
        print_info "agentic-network already exists"
    else
        docker network create agentic-network
        print_success "agentic-network created"
    fi
}

# Create config directory
create_config() {
    print_header "Creating Configuration Directories"

    mkdir -p "$PROJECT_ROOT/config/agentgateway"
    print_success "AgentGateway config directory created"

    if [ ! -f "$PROJECT_ROOT/config/agentgateway/config.yaml" ]; then
        print_warning "config.yaml not found - AgentGateway will use defaults"
    else
        print_success "config.yaml found"
    fi
}

# Deploy services
deploy_services() {
    print_header "Deploying Edge Services"

    print_info "Starting services with docker-compose..."
    cd "$PROJECT_ROOT"

    # Pull images first
    print_info "Pulling images..."
    docker compose -f docker-compose.edge.yml pull

    # Start services
    print_info "Starting services..."
    docker compose -f docker-compose.edge.yml up -d

    print_success "Services started"
}

# Wait for services to be healthy
wait_for_services() {
    print_header "Waiting for Services to be Healthy"

    local max_wait=120
    local elapsed=0

    print_info "Waiting for Kong database to be healthy..."
    while [ $elapsed -lt $max_wait ]; do
        if docker inspect --format='{{.State.Health.Status}}' kong-database 2>/dev/null | grep -q "healthy"; then
            print_success "Kong database is healthy"
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [ $elapsed -ge $max_wait ]; then
        print_failure "Timeout waiting for Kong database"
        return 1
    fi

    print_info "Waiting for Kong gateway to be healthy..."
    elapsed=0
    while [ $elapsed -lt $max_wait ]; do
        if docker inspect --format='{{.State.Health.Status}}' kong 2>/dev/null | grep -q "healthy"; then
            print_success "Kong gateway is healthy"
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [ $elapsed -ge $max_wait ]; then
        print_failure "Timeout waiting for Kong gateway"
        return 1
    fi

    print_info "Waiting for AgentGateway to be healthy..."
    elapsed=0
    while [ $elapsed -lt $max_wait ]; do
        if docker inspect --format='{{.State.Health.Status}}' agentgateway 2>/dev/null | grep -q "healthy"; then
            print_success "AgentGateway is healthy"
            break
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done

    if [ $elapsed -ge $max_wait ]; then
        print_failure "Timeout waiting for AgentGateway"
        return 1
    fi

    print_success "All services are healthy"
}

# Show service status
show_status() {
    print_header "Service Status"

    cd "$PROJECT_ROOT"
    docker compose -f docker-compose.edge.yml ps
}

# Show access information
show_access_info() {
    print_header "Access Information"

    echo "Kong Gateway (P0-003):"
    echo "  - Admin API:  http://localhost:8001"
    echo "  - Proxy:      http://localhost:8000"
    echo "  - Manager UI: http://localhost:8002"
    echo ""
    echo "AgentGateway (P0-004):"
    echo "  - Main API:   http://localhost:8090"
    echo "  - Admin API:  http://localhost:8091"
    echo "  - Metrics:    http://localhost:8092/metrics"
    echo ""
    echo "Konga Admin UI:"
    echo "  - Web UI:     http://localhost:1337"
    echo ""
    echo "Quick Tests:"
    echo "  - Kong status:        curl http://localhost:8001/status"
    echo "  - AgentGateway health: curl http://localhost:8090/health"
    echo ""
    echo "Verification:"
    echo "  - Run verification script: $SCRIPT_DIR/verify-edge.sh"
}

# Main deployment flow
main() {
    print_header "ARIA Edge Services Deployment"
    echo "P0-003: Kong Gateway"
    echo "P0-004: AgentGateway"
    echo "Date: $(date)"
    echo ""

    check_prerequisites
    create_network
    create_config
    deploy_services

    print_info "Waiting 10 seconds for services to initialize..."
    sleep 10

    wait_for_services
    show_status
    show_access_info

    print_header "Deployment Complete"
    print_success "Edge services deployed successfully!"
    echo ""
    print_info "Next steps:"
    echo "  1. Run verification: $SCRIPT_DIR/verify-edge.sh"
    echo "  2. Configure Kong routes for your services"
    echo "  3. Set up AgentGateway routing rules"
    echo "  4. Access Konga UI at http://localhost:1337 to manage Kong"
    echo ""
    print_info "To stop services: docker-compose -f docker-compose.edge.yml down"
    print_info "To view logs: docker-compose -f docker-compose.edge.yml logs -f"
}

# Run main function
main "$@"
