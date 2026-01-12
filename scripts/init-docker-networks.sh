#!/usr/bin/env bash
# =============================================================================
# Docker Network Initialization Script
# =============================================================================
# Creates isolated Docker networks for the ARIA platform with proper
# segmentation for security boundaries.
#
# Networks:
#   - identity-network: Identity, auth, and secrets (172.20.0.0/16)
#   - agentic-network: AI/ML and automation services (172.21.0.0/16)
#   - observability-network: Monitoring and logging (172.22.0.0/16)
#   - data-network: Databases and storage (172.23.0.0/16)
#   - messaging-network: Event streaming (172.24.0.0/16)
#
# Usage:
#   ./scripts/init-docker-networks.sh
#   ./scripts/init-docker-networks.sh --clean  # Remove and recreate
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Network definitions
declare -A NETWORKS=(
    ["identity-network"]="172.20.0.0/16:172.20.1.0/24"
    ["agentic-network"]="172.21.0.0/16:172.21.1.0/24"
    ["observability-network"]="172.22.0.0/16:172.22.1.0/24"
    ["data-network"]="172.23.0.0/16:172.23.1.0/24"
    ["messaging-network"]="172.24.0.0/16:172.24.1.0/24"
)

# Network descriptions
declare -A DESCRIPTIONS=(
    ["identity-network"]="Identity, authentication, and secrets management"
    ["agentic-network"]="AI/ML services and workflow automation"
    ["observability-network"]="Monitoring, logging, and tracing"
    ["data-network"]="Databases and persistent storage"
    ["messaging-network"]="NATS and event streaming"
)

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
Docker Network Initialization Script

Usage: $0 [OPTIONS]

Options:
    --clean     Remove existing networks and recreate them
    --remove    Remove all ARIA networks
    --status    Show current network status
    --help      Show this help message

Networks created:
EOF
    for network in "${!NETWORKS[@]}"; do
        IFS=':' read -r subnet ip_range <<< "${NETWORKS[$network]}"
        echo "    $network"
        echo "        Subnet: $subnet"
        echo "        IP Range: $ip_range"
        echo "        Purpose: ${DESCRIPTIONS[$network]}"
        echo ""
    done
}

network_exists() {
    docker network inspect "$1" &>/dev/null
}

create_network() {
    local name=$1
    local config=${NETWORKS[$name]}
    IFS=':' read -r subnet ip_range <<< "$config"

    if network_exists "$name"; then
        log_warn "Network '$name' already exists"
        return 0
    fi

    log_info "Creating network: $name (${DESCRIPTIONS[$name]})"
    docker network create \
        --driver bridge \
        --subnet="$subnet" \
        --ip-range="$ip_range" \
        --label "aria.network.purpose=${DESCRIPTIONS[$name]}" \
        --label "aria.network.security=segmented" \
        "$name"

    log_success "Created network: $name"
}

remove_network() {
    local name=$1

    if ! network_exists "$name"; then
        log_warn "Network '$name' does not exist"
        return 0
    fi

    # Check if network is in use
    local containers
    containers=$(docker network inspect "$name" -f '{{range .Containers}}{{.Name}} {{end}}' 2>/dev/null || echo "")

    if [[ -n "$containers" ]]; then
        log_error "Network '$name' is in use by: $containers"
        log_error "Stop containers before removing network"
        return 1
    fi

    log_info "Removing network: $name"
    docker network rm "$name"
    log_success "Removed network: $name"
}

show_status() {
    echo ""
    echo "=== ARIA Docker Network Status ==="
    echo ""

    for network in "${!NETWORKS[@]}"; do
        if network_exists "$network"; then
            echo -e "${GREEN}✓${NC} $network"
            local subnet
            subnet=$(docker network inspect "$network" -f '{{range .IPAM.Config}}{{.Subnet}}{{end}}')
            local containers
            containers=$(docker network inspect "$network" -f '{{range .Containers}}{{.Name}} {{end}}')
            echo "    Subnet: $subnet"
            echo "    Purpose: ${DESCRIPTIONS[$network]}"
            if [[ -n "$containers" ]]; then
                echo "    Containers: $containers"
            else
                echo "    Containers: (none)"
            fi
        else
            echo -e "${RED}✗${NC} $network (not created)"
        fi
        echo ""
    done
}

create_all_networks() {
    log_info "Creating ARIA Docker networks..."
    echo ""

    for network in "${!NETWORKS[@]}"; do
        create_network "$network"
    done

    echo ""
    log_success "All networks created successfully"
    echo ""
    show_status
}

remove_all_networks() {
    log_info "Removing ARIA Docker networks..."
    echo ""

    local failed=0
    for network in "${!NETWORKS[@]}"; do
        if ! remove_network "$network"; then
            failed=1
        fi
    done

    if [[ $failed -eq 1 ]]; then
        log_error "Some networks could not be removed"
        exit 1
    fi

    echo ""
    log_success "All networks removed successfully"
}

clean_and_create() {
    log_info "Cleaning and recreating ARIA Docker networks..."
    echo ""

    # First, try to remove all networks
    for network in "${!NETWORKS[@]}"; do
        if network_exists "$network"; then
            remove_network "$network" || true
        fi
    done

    echo ""

    # Then create all networks
    create_all_networks
}

# Main
main() {
    case "${1:-}" in
        --help|-h)
            show_help
            ;;
        --clean)
            clean_and_create
            ;;
        --remove)
            remove_all_networks
            ;;
        --status)
            show_status
            ;;
        "")
            create_all_networks
            ;;
        *)
            log_error "Unknown option: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
