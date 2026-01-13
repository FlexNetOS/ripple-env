#!/usr/bin/env bash
# FlexNetOS Deployment Script

set -euo pipefail

echo "Deploying FlexNetOS..."

# Start core services first
echo "Starting core services..."
docker-compose -f docker/holochain.yml up -d
docker-compose -f docker/nats.yml up -d
docker-compose -f docker/vault.yml up -d

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
        container_id="$(docker-compose -f "docker/$service.yml" ps -q 2>/dev/null | head -n1 || true)"

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
check_service_health "holochain"
check_service_health "nats"
check_service_health "vault"

# Start remaining services
services=("kong" "agixt" "localai" "prometheus" "grafana" "keycloak" "minio" "ipfs" "tensorzero" "lobe-chat")

for service in "${services[@]}"; do
    echo "Starting $service..."
    docker-compose -f docker/$service.yml up -d
done

echo "FlexNetOS deployment completed!"
echo "Access Grafana: http://localhost:3000"
echo "Access Prometheus: http://localhost:9090"
echo "Access Kong Gateway: http://localhost:8000"
