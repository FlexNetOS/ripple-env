#!/usr/bin/env bash
# FlexNetOS Deployment Script

set -euo pipefail

echo "Deploying FlexNetOS..."

# Start core services first
echo "Starting core services..."
docker-compose -f docker/holochain.yml up -d
docker-compose -f docker/nats.yml up -d
docker-compose -f docker/vault.yml up -d

# Wait for core services to be ready
echo "Waiting for core services..."
sleep 30

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
