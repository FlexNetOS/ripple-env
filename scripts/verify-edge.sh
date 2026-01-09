#!/bin/bash
# Edge Services Verification Script
# Verifies P0-003 (Kong Gateway) and P0-004 (AgentGateway) deployment
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
KONG_ADMIN_URL="${KONG_ADMIN_URL:-http://localhost:8001}"
KONG_PROXY_URL="${KONG_PROXY_URL:-http://localhost:8000}"
KONG_MANAGER_URL="${KONG_MANAGER_URL:-http://localhost:8002}"
AGENTGATEWAY_API_URL="${AGENTGATEWAY_API_URL:-http://localhost:8090}"
AGENTGATEWAY_ADMIN_URL="${AGENTGATEWAY_ADMIN_URL:-http://localhost:8091}"
AGENTGATEWAY_METRICS_URL="${AGENTGATEWAY_METRICS_URL:-http://localhost:8092}"
KONGA_URL="${KONGA_URL:-http://localhost:1337}"

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_TOTAL=0

# Helper functions
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_test() {
    echo -e "${YELLOW}[TEST]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((TESTS_PASSED++))
}

print_failure() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((TESTS_FAILED++))
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

run_test() {
    ((TESTS_TOTAL++))
}

# Check if a URL is accessible
check_url() {
    local url=$1
    local expected_code=${2:-200}
    local timeout=${3:-5}

    if curl -s -f -o /dev/null -w "%{http_code}" --max-time "$timeout" "$url" | grep -q "$expected_code"; then
        return 0
    else
        return 1
    fi
}

# Check if a service is running
check_service() {
    local service_name=$1

    if docker ps --format "{{.Names}}" | grep -q "^${service_name}$"; then
        return 0
    else
        return 1
    fi
}

# Check if a service is healthy
check_service_health() {
    local service_name=$1
    local health_status

    health_status=$(docker inspect --format='{{.State.Health.Status}}' "$service_name" 2>/dev/null || echo "none")

    if [ "$health_status" = "healthy" ]; then
        return 0
    elif [ "$health_status" = "none" ]; then
        # Check if service is running (some services don't have health checks)
        if docker ps --format "{{.Names}}" | grep -q "^${service_name}$"; then
            return 0
        fi
    fi
    return 1
}

# Main verification
main() {
    print_header "ARIA Edge Services Verification"
    echo "P0-003: Kong Gateway"
    echo "P0-004: AgentGateway"
    echo "Date: $(date)"
    echo ""

    # Check if Docker network exists
    print_header "Prerequisites Check"

    print_test "Checking if agentic-network exists..."
    run_test
    if docker network ls | grep -q agentic-network; then
        print_success "agentic-network exists"
    else
        print_failure "agentic-network does not exist"
        print_info "Create it with: docker network create agentic-network"
    fi

    # Check Docker Compose file
    print_test "Checking docker-compose.edge.yml..."
    run_test
    if [ -f "docker-compose.edge.yml" ]; then
        print_success "docker-compose.edge.yml found"
    else
        print_failure "docker-compose.edge.yml not found"
    fi

    # Check config directory
    print_test "Checking AgentGateway config directory..."
    run_test
    if [ -d "config/agentgateway" ]; then
        print_success "AgentGateway config directory exists"
        if [ -f "config/agentgateway/config.yaml" ]; then
            print_info "  - config.yaml found"
        else
            print_failure "  - config.yaml not found"
        fi
    else
        print_failure "AgentGateway config directory not found"
    fi

    # Check P0-003: Kong Gateway
    print_header "P0-003: Kong Gateway Verification"

    print_test "Checking Kong database service..."
    run_test
    if check_service "kong-database"; then
        if check_service_health "kong-database"; then
            print_success "Kong database is running and healthy"
        else
            print_failure "Kong database is running but not healthy"
        fi
    else
        print_failure "Kong database is not running"
    fi

    print_test "Checking Kong migrations service..."
    run_test
    if docker ps -a --format "{{.Names}}" | grep -q "kong-migrations"; then
        local status=$(docker inspect --format='{{.State.Status}}' kong-migrations 2>/dev/null || echo "not found")
        if [ "$status" = "exited" ]; then
            local exit_code=$(docker inspect --format='{{.State.ExitCode}}' kong-migrations)
            if [ "$exit_code" = "0" ]; then
                print_success "Kong migrations completed successfully"
            else
                print_failure "Kong migrations failed with exit code $exit_code"
            fi
        else
            print_info "Kong migrations status: $status"
        fi
    else
        print_failure "Kong migrations container not found"
    fi

    print_test "Checking Kong gateway service..."
    run_test
    if check_service "kong"; then
        if check_service_health "kong"; then
            print_success "Kong gateway is running and healthy"
        else
            print_failure "Kong gateway is running but not healthy"
        fi
    else
        print_failure "Kong gateway is not running"
    fi

    print_test "Checking Kong Admin API ($KONG_ADMIN_URL/status)..."
    run_test
    if check_url "$KONG_ADMIN_URL/status"; then
        print_success "Kong Admin API is accessible"
        kong_version=$(curl -s "$KONG_ADMIN_URL" | jq -r '.version' 2>/dev/null || echo "unknown")
        print_info "  - Kong version: $kong_version"
    else
        print_failure "Kong Admin API is not accessible"
    fi

    print_test "Checking Kong Proxy ($KONG_PROXY_URL)..."
    run_test
    if check_url "$KONG_PROXY_URL" "404|200"; then
        print_success "Kong Proxy is accessible"
    else
        print_failure "Kong Proxy is not accessible"
    fi

    print_test "Checking Kong Manager UI ($KONG_MANAGER_URL)..."
    run_test
    if check_url "$KONG_MANAGER_URL"; then
        print_success "Kong Manager UI is accessible"
    else
        print_failure "Kong Manager UI is not accessible"
    fi

    print_test "Checking Konga UI service..."
    run_test
    if check_service "konga"; then
        if check_url "$KONGA_URL"; then
            print_success "Konga UI is running and accessible"
        else
            print_failure "Konga UI is running but not accessible"
        fi
    else
        print_info "Konga UI is not running (optional service)"
    fi

    # Check P0-004: AgentGateway
    print_header "P0-004: AgentGateway Verification"

    print_test "Checking AgentGateway service..."
    run_test
    if check_service "agentgateway"; then
        if check_service_health "agentgateway"; then
            print_success "AgentGateway is running and healthy"
        else
            print_failure "AgentGateway is running but not healthy"
        fi
    else
        print_failure "AgentGateway is not running"
    fi

    print_test "Checking AgentGateway Main API ($AGENTGATEWAY_API_URL/health)..."
    run_test
    if check_url "$AGENTGATEWAY_API_URL/health"; then
        print_success "AgentGateway Main API is accessible"
        ag_status=$(curl -s "$AGENTGATEWAY_API_URL/health" | jq -r '.status' 2>/dev/null || echo "unknown")
        print_info "  - Status: $ag_status"
    else
        print_failure "AgentGateway Main API is not accessible"
    fi

    print_test "Checking AgentGateway Admin API ($AGENTGATEWAY_ADMIN_URL)..."
    run_test
    if check_url "$AGENTGATEWAY_ADMIN_URL" "200|404"; then
        print_success "AgentGateway Admin API is accessible"
    else
        print_failure "AgentGateway Admin API is not accessible"
    fi

    print_test "Checking AgentGateway Metrics ($AGENTGATEWAY_METRICS_URL/metrics)..."
    run_test
    if check_url "$AGENTGATEWAY_METRICS_URL/metrics"; then
        print_success "AgentGateway Metrics endpoint is accessible"
        metric_count=$(curl -s "$AGENTGATEWAY_METRICS_URL/metrics" | grep -c "^agentgateway_" || echo "0")
        print_info "  - Metrics available: $metric_count"
    else
        print_failure "AgentGateway Metrics endpoint is not accessible"
    fi

    # Integration tests
    print_header "Integration Tests"

    print_test "Checking AgentGateway -> Kong integration..."
    run_test
    ag_kong_status=$(curl -s "$AGENTGATEWAY_API_URL/health" 2>/dev/null | jq -r '.upstreams.kong // "unknown"' 2>/dev/null || echo "unknown")
    if [ "$ag_kong_status" = "healthy" ] || [ "$ag_kong_status" = "up" ]; then
        print_success "AgentGateway can reach Kong"
    else
        print_info "AgentGateway -> Kong status: $ag_kong_status"
    fi

    print_test "Checking MCP protocol support..."
    run_test
    mcp_enabled=$(docker exec agentgateway env 2>/dev/null | grep "AG_MCP_ENABLED=true" || echo "")
    if [ -n "$mcp_enabled" ]; then
        print_success "MCP protocol is enabled"
        mcp_transports=$(docker exec agentgateway env 2>/dev/null | grep "AG_MCP_TRANSPORT" | cut -d= -f2 || echo "unknown")
        print_info "  - Transports: $mcp_transports"
    else
        print_failure "MCP protocol is not enabled"
    fi

    # Feature flags verification
    print_header "Feature Flags Verification"

    print_test "Checking KONG_ENABLED feature flag..."
    run_test
    kong_enabled=$(docker exec kong env 2>/dev/null | grep "KONG_ENABLED" | cut -d= -f2 || echo "not set")
    if [ "$kong_enabled" = "true" ]; then
        print_success "KONG_ENABLED: true"
    else
        print_info "KONG_ENABLED: $kong_enabled"
    fi

    print_test "Checking AGENTGATEWAY_ENABLED feature flag..."
    run_test
    ag_enabled=$(docker exec agentgateway env 2>/dev/null | grep "AGENTGATEWAY_ENABLED" | cut -d= -f2 || echo "not set")
    if [ "$ag_enabled" = "true" ]; then
        print_success "AGENTGATEWAY_ENABLED: true"
    else
        print_info "AGENTGATEWAY_ENABLED: $ag_enabled"
    fi

    # Service logs check
    print_header "Service Logs Check"

    print_test "Checking for errors in Kong logs..."
    run_test
    kong_errors=$(docker logs kong 2>&1 | grep -i "error" | tail -5 || echo "")
    if [ -z "$kong_errors" ]; then
        print_success "No recent errors in Kong logs"
    else
        print_info "Recent errors found in Kong logs (last 5):"
        echo "$kong_errors" | sed 's/^/    /'
    fi

    print_test "Checking for errors in AgentGateway logs..."
    run_test
    ag_errors=$(docker logs agentgateway 2>&1 | grep -i "error" | tail -5 || echo "")
    if [ -z "$ag_errors" ]; then
        print_success "No recent errors in AgentGateway logs"
    else
        print_info "Recent errors found in AgentGateway logs (last 5):"
        echo "$ag_errors" | sed 's/^/    /'
    fi

    # Summary
    print_header "Verification Summary"

    echo "Total Tests: $TESTS_TOTAL"
    echo -e "Passed: ${GREEN}$TESTS_PASSED${NC}"
    echo -e "Failed: ${RED}$TESTS_FAILED${NC}"
    echo ""

    if [ $TESTS_FAILED -eq 0 ]; then
        echo -e "${GREEN}All tests passed! Edge services are deployed correctly.${NC}"
        echo ""
        echo "Access Points:"
        echo "  - Kong Admin API:      $KONG_ADMIN_URL"
        echo "  - Kong Proxy:          $KONG_PROXY_URL"
        echo "  - Kong Manager:        $KONG_MANAGER_URL"
        echo "  - Konga UI:            $KONGA_URL"
        echo "  - AgentGateway API:    $AGENTGATEWAY_API_URL"
        echo "  - AgentGateway Admin:  $AGENTGATEWAY_ADMIN_URL"
        echo "  - AgentGateway Metrics: $AGENTGATEWAY_METRICS_URL"
        return 0
    else
        echo -e "${RED}Some tests failed. Please review the errors above.${NC}"
        echo ""
        echo "Troubleshooting:"
        echo "  - Check service logs: docker-compose -f docker-compose.edge.yml logs <service>"
        echo "  - Restart services: docker-compose -f docker-compose.edge.yml restart"
        echo "  - View service status: docker-compose -f docker-compose.edge.yml ps"
        return 1
    fi
}

# Run main function
main "$@"
