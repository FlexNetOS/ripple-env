#!/usr/bin/env bash
# FlexNetOS Deployment Script

set -euo pipefail

echo "Deploying FlexNetOS..."

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo "Error: Docker Compose not found (docker compose / docker-compose)"
    exit 1
fi

resolve_compose_file() {
    # usage: resolve_compose_file docker-compose.yml
    local base="$1"
    if [ -f "docker/$base" ]; then
        echo "docker/$base"
    else
        echo "$base"
    fi
}

STACK_COMPOSE_FILE="$(resolve_compose_file docker-compose.yml)"

if [ ! -f "$STACK_COMPOSE_FILE" ]; then
    echo "Error: stack compose file not found (expected docker/docker-compose.yml or docker-compose.yml)"
    exit 1
fi

# Start core services first
echo "Starting core services..."
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" pull vault keycloak nats || true
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" up -d vault keycloak nats

# Wait for core services to be ready with health checks
echo "Waiting for core services to be ready..."

# Function to check if a service is healthy
check_service_health() {
    local service=$1
    local max_attempts=30
    local attempt=0

    while [ $attempt -lt $max_attempts ]; do
        # Get the container ID for this service (if any)
        local container_id
        container_id="$("${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" ps -q "$service" 2>/dev/null | head -n1 || true)"

        if [ -n "${container_id:-}" ]; then
            # Try to read explicit health status (if a health check is defined)
            local health_status
            health_status="$(docker inspect -f '{{if .State.Health}}{{.State.Health.Status}}{{end}}' "$container_id" 2>/dev/null || true)"

            if [ -n "$health_status" ]; then
                if [ "$health_status" = "healthy" ]; then
                    echo "$service is ready"
                    return 0
                fi
            else
                # No health check defined; fall back to container state
                local container_state
                container_state="$(docker inspect -f '{{.State.Status}}' "$container_id" 2>/dev/null || true)"
                if [ "$container_state" = "running" ]; then
                    echo "$service is ready"
                    return 0
                fi
            fi
        fi

        attempt=$((attempt + 1))
        sleep 2
    done
    echo "Warning: $service may not be fully ready"
    return 1
}

# Check core services
check_service_health "keycloak" || true
check_service_health "vault" || true
check_service_health "nats" || true

# Start remaining services
echo "Starting remaining services..."
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" up -d

echo "FlexNetOS deployment completed!"
echo "Access Grafana: http://localhost:3000"
echo "Access Prometheus: http://localhost:9090"
echo "Access Kong Gateway: http://localhost:8000"
