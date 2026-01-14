# Edge Services Quick Start Guide

## Deploy in 3 Steps

### Step 1: Create Network
```bash
docker network create agentic-network
```

### Step 2: Deploy Services
```bash
cd /home/user/ripple-env
./scripts/deploy-edge.sh
```

### Step 3: Verify Deployment
```bash
./scripts/verify-edge.sh
```

## Access Services

| Service | URL | Description |
|---------|-----|-------------|
| Kong Admin API | http://localhost:8001 | Configure Kong |
| Kong Proxy | http://localhost:8000 | API Gateway |
| Kong Manager | http://localhost:8002 | Web UI |
| Konga UI | http://localhost:1337 | Admin Dashboard |
| AgentGateway API | http://localhost:8090 | Agent Gateway |
| AgentGateway Metrics | http://localhost:8092/metrics | Prometheus |

## Quick Tests

```bash
# Kong status
curl http://localhost:8001/status

# AgentGateway health
curl http://localhost:8090/health

# View logs
docker compose -f docker/docker-compose.edge.yml logs -f
```

## Common Commands

```bash
# Start services
docker compose -f docker/docker-compose.edge.yml up -d

# Stop services
docker compose -f docker/docker-compose.edge.yml down

# Restart services
docker compose -f docker/docker-compose.edge.yml restart

# View status
docker compose -f docker/docker-compose.edge.yml ps

# View logs
docker compose -f docker/docker-compose.edge.yml logs -f <service>
```

## Feature Flags

Both services are enabled by default:
- KONG_ENABLED=true
- AGENTGATEWAY_ENABLED=true

## Documentation

For detailed information, see [EDGE_DEPLOYMENT.md](./EDGE_DEPLOYMENT.md)
