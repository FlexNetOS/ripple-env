#!/usr/bin/env bash
# =============================================================================
# FlexStack Docker Compose Helper Script
# =============================================================================
# Simplified management of Docker Compose profiles for FlexStack services.
#
# Usage:
#   ./scripts/flexstack.sh up --profile dev
#   ./scripts/flexstack.sh down
#   ./scripts/flexstack.sh status
#   ./scripts/flexstack.sh logs localai
#
# =============================================================================

set -euo pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DOCKER_DIR="$PROJECT_DIR/docker"
COMPOSE_FILE="$DOCKER_DIR/docker-compose.yml"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default values
PROFILES=()
DETACH=true
FOLLOW_LOGS=false
VERBOSE=false

# Profile presets
declare -A PROFILE_PRESETS=(
    ["minimal"]="core"
    ["dev"]="dev"
    ["development"]="dev"
    ["ai"]="core ai"
    ["observability"]="observability"
    ["monitoring"]="observability"
    ["identity"]="core identity"
    ["auth"]="core identity"
    ["data"]="core data"
    ["edge"]="edge"
    ["gateway"]="edge"
    ["messaging"]="core messaging"
    ["workflows"]="core messaging"
    ["automation"]="core automation"
    ["full"]="full"
    ["all"]="full"
)

# Profile descriptions
declare -A PROFILE_DESCRIPTIONS=(
    ["core"]="PostgreSQL, Redis, MinIO (4GB RAM)"
    ["dev"]="Core + LocalAI (12GB RAM)"
    ["ai"]="LocalAI, AGiXT, MindsDB (16GB RAM)"
    ["observability"]="Prometheus, Grafana, Loki, Tempo (8GB RAM)"
    ["identity"]="Keycloak, Vault, Step-CA (4GB RAM)"
    ["data"]="Neo4j, MindsDB (6GB RAM)"
    ["edge"]="Kong, AgentGateway (2GB RAM)"
    ["messaging"]="NATS, Temporal (4GB RAM)"
    ["automation"]="n8n, OPA (2GB RAM)"
    ["full"]="All services (32GB+ RAM)"
)

# =============================================================================
# Helper Functions
# =============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

usage() {
    cat << EOF
${CYAN}FlexStack Docker Compose Helper${NC}

Usage: $(basename "$0") <command> [options]

Commands:
    up          Start services with specified profiles
    down        Stop and remove services
    stop        Stop services without removing
    restart     Restart services
    status      Show service status
    logs        Show service logs
    ps          List running services
    exec        Execute command in a service
    validate    Validate resources for profiles
    list        List available profiles

Options:
    -p, --profile <name>    Profile to use (can be repeated)
    -f, --follow            Follow logs after starting
    -d, --detach            Run in detached mode (default: true)
    -v, --verbose           Verbose output
    -h, --help              Show this help message

Profile Presets:
    minimal     Core infrastructure only
    dev         Development stack (core + AI)
    ai          Full AI/ML stack
    monitoring  Observability stack
    auth        Identity services
    full        All services

Examples:
    $(basename "$0") up --profile dev
    $(basename "$0") up --profile core --profile ai
    $(basename "$0") up --profile full -f
    $(basename "$0") logs localai
    $(basename "$0") status

EOF
}

list_profiles() {
    echo -e "${CYAN}Available Profiles:${NC}"
    echo ""
    for profile in core dev ai observability identity data edge messaging automation full; do
        desc="${PROFILE_DESCRIPTIONS[$profile]:-No description}"
        printf "  ${GREEN}%-15s${NC} %s\n" "$profile" "$desc"
    done
    echo ""
    echo -e "${CYAN}Profile Presets (aliases):${NC}"
    echo ""
    for preset in minimal development monitoring auth gateway workflows all; do
        profiles="${PROFILE_PRESETS[$preset]}"
        printf "  ${YELLOW}%-15s${NC} -> %s\n" "$preset" "$profiles"
    done
}

resolve_profile() {
    local profile="$1"
    if [[ -n "${PROFILE_PRESETS[$profile]:-}" ]]; then
        echo "${PROFILE_PRESETS[$profile]}"
    else
        echo "$profile"
    fi
}

build_profile_args() {
    local args=""
    for profile in "${PROFILES[@]}"; do
        # Resolve preset if applicable
        resolved=$(resolve_profile "$profile")
        for p in $resolved; do
            args="$args --profile $p"
        done
    done
    echo "$args"
}

check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker is not installed or not in PATH"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        log_error "Docker daemon is not running"
        exit 1
    fi
}

check_compose_file() {
    if [[ ! -f "$COMPOSE_FILE" ]]; then
        log_error "Compose file not found: $COMPOSE_FILE"
        exit 1
    fi
}

create_network() {
    if ! docker network inspect flexstack-network &> /dev/null; then
        log_info "Creating flexstack-network..."
        docker network create flexstack-network 2>/dev/null || true
    fi
}

# =============================================================================
# Commands
# =============================================================================

cmd_up() {
    check_docker
    check_compose_file
    create_network

    if [[ ${#PROFILES[@]} -eq 0 ]]; then
        log_warn "No profile specified, using 'dev' profile"
        PROFILES=("dev")
    fi

    local profile_args=$(build_profile_args)

    log_info "Starting FlexStack with profiles: ${PROFILES[*]}"

    local detach_arg=""
    if [[ "$DETACH" == true ]]; then
        detach_arg="-d"
    fi

    if [[ "$VERBOSE" == true ]]; then
        log_info "Running: docker compose -f $COMPOSE_FILE $profile_args up $detach_arg"
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args up $detach_arg

    if [[ "$DETACH" == true ]]; then
        log_success "Services started in detached mode"
        echo ""
        # shellcheck disable=SC2086
        docker compose -f "$COMPOSE_FILE" $profile_args ps

        if [[ "$FOLLOW_LOGS" == true ]]; then
            echo ""
            log_info "Following logs (Ctrl+C to stop)..."
            # shellcheck disable=SC2086
            docker compose -f "$COMPOSE_FILE" $profile_args logs -f
        fi
    fi
}

cmd_down() {
    check_docker
    check_compose_file

    log_info "Stopping FlexStack services..."

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args down

    log_success "Services stopped"
}

cmd_stop() {
    check_docker
    check_compose_file

    log_info "Stopping FlexStack services..."

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args stop

    log_success "Services stopped"
}

cmd_restart() {
    check_docker
    check_compose_file

    log_info "Restarting FlexStack services..."

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args restart "${@}"

    log_success "Services restarted"
}

cmd_status() {
    check_docker
    check_compose_file

    echo -e "${CYAN}FlexStack Service Status${NC}"
    echo ""

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args ps --format "table {{.Name}}\t{{.Status}}\t{{.Ports}}"
}

cmd_logs() {
    check_docker
    check_compose_file

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    local follow_arg=""
    if [[ "$FOLLOW_LOGS" == true ]]; then
        follow_arg="-f"
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args logs $follow_arg "${@}"
}

cmd_ps() {
    check_docker
    check_compose_file

    local profile_args=""
    if [[ ${#PROFILES[@]} -gt 0 ]]; then
        profile_args=$(build_profile_args)
    fi

    # shellcheck disable=SC2086
    docker compose -f "$COMPOSE_FILE" $profile_args ps "${@}"
}

cmd_exec() {
    check_docker
    check_compose_file

    if [[ $# -lt 2 ]]; then
        log_error "Usage: $(basename "$0") exec <service> <command>"
        exit 1
    fi

    local service="$1"
    shift

    docker compose -f "$COMPOSE_FILE" exec "$service" "${@}"
}

cmd_validate() {
    if [[ -x "$SCRIPT_DIR/validate-resources.sh" ]]; then
        if [[ ${#PROFILES[@]} -gt 0 ]]; then
            "$SCRIPT_DIR/validate-resources.sh" --profile "${PROFILES[0]}"
        else
            "$SCRIPT_DIR/validate-resources.sh"
        fi
    else
        log_warn "validate-resources.sh not found or not executable"
    fi
}

# =============================================================================
# Main
# =============================================================================

main() {
    if [[ $# -eq 0 ]]; then
        usage
        exit 0
    fi

    local command=""
    local extra_args=()

    while [[ $# -gt 0 ]]; do
        case "$1" in
            up|down|stop|restart|status|logs|ps|exec|validate|list)
                command="$1"
                shift
                ;;
            -p|--profile)
                if [[ -n "${2:-}" ]]; then
                    PROFILES+=("$2")
                    shift 2
                else
                    log_error "--profile requires an argument"
                    exit 1
                fi
                ;;
            -f|--follow)
                FOLLOW_LOGS=true
                shift
                ;;
            -d|--detach)
                DETACH=true
                shift
                ;;
            --no-detach)
                DETACH=false
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -h|--help)
                usage
                exit 0
                ;;
            -*)
                log_error "Unknown option: $1"
                usage
                exit 1
                ;;
            *)
                extra_args+=("$1")
                shift
                ;;
        esac
    done

    case "$command" in
        up)
            cmd_up
            ;;
        down)
            cmd_down
            ;;
        stop)
            cmd_stop
            ;;
        restart)
            cmd_restart "${extra_args[@]}"
            ;;
        status)
            cmd_status
            ;;
        logs)
            cmd_logs "${extra_args[@]}"
            ;;
        ps)
            cmd_ps "${extra_args[@]}"
            ;;
        exec)
            cmd_exec "${extra_args[@]}"
            ;;
        validate)
            cmd_validate
            ;;
        list)
            list_profiles
            ;;
        *)
            log_error "Unknown command: $command"
            usage
            exit 1
            ;;
    esac
}

main "$@"
