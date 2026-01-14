# ARIA Edge Services Deployment Guide

**Domain**: L4 Edge & Agent Traffic
**Priority**: P0-003 (Kong Gateway), P0-004 (AgentGateway)
**Version**: 1.0.0
**Date**: 2026-01-09

## Overview

This guide covers the deployment and verification of edge services layer:

- **P0-003: Kong Gateway** - North-South API Gateway for external traffic
- **P0-004: AgentGateway** - East-West gateway for agent/MCP traffic plane

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     External Clients                        │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
            ┌─────────────────┐
            │  Kong Gateway   │ (P0-003)
            │  Port: 8000     │ - North-South traffic
            │  Admin: 8001    │ - API rate limiting
            │  Manager: 8002  │ - Authentication
            └────────┬────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
        ▼                         ▼
┌───────────────┐         ┌──────────────┐
│ AgentGateway  │         │  LocalAI     │
│ (P0-004)      │◄───────►│              │
│ Port: 8090    │         │ Port: 8080   │
│ Admin: 8091   │         └──────────────┘
│ Metrics: 8092 │
└───────────────┘
        │
        └─────► MCP Protocol Support
                (stdio, http)
```

## Components

### Kong Gateway (P0-003)

**Purpose**: North-South API Gateway for external traffic routing

**Services**:
- `kong-database` - PostgreSQL 17.2 database
- `kong-migrations` - Database initialization
- `kong` - Kong Gateway 3.9.0
- `konga-prepare` - Konga database setup
- `konga` - Web UI for Kong management

**Ports**:
- 8000 - Proxy HTTP
- 8443 - Proxy HTTPS
- 8001 - Admin API
- 8444 - Admin API HTTPS
- 8002 - Kong Manager UI
- 1337 - Konga UI

**Features**:
- Request routing and proxying
- Rate limiting
- Authentication/Authorization
- Load balancing
- Health checks
- Logging and monitoring

### AgentGateway (P0-004)

**Purpose**: East-West gateway for agent traffic and MCP protocol support

**Services**:
- `agentgateway` - Agent Gateway 0.2.0

**Ports**:
- 8090 - Main API
- 8091 - Admin API
- 8092 - Prometheus metrics

**Features**:
- MCP (Model Context Protocol) support
- Agent traffic routing
- Circuit breaker
- Rate limiting
- Connection pooling
- Health checks
- Prometheus metrics

## Prerequisites

1. Docker Engine 20.10+
2. Docker Compose v2+
3. Network connectivity
4. Minimum 2GB RAM available
5. Ports 8000-8002, 8090-8092, 1337 available

## Quick Start

### Option 1: Automated Deployment (Recommended)

```bash
# Run the deployment script
cd /home/user/ripple-env
./scripts/deploy-edge.sh

# Verify deployment
./scripts/verify-edge.sh
```

### Option 2: Manual Deployment

```bash
# 1. Create Docker network
docker network create agentic-network

# 2. Create config directory
mkdir -p config/agentgateway

# 3. Start services
docker compose -f docker/docker-compose.edge.yml up -d

# 4. Check status
docker compose -f docker/docker-compose.edge.yml ps

# 5. Verify services
./scripts/verify-edge.sh
```

## Configuration

### Environment Variables

Copy the example environment file:

```bash
cp .env.edge.example .env.edge
```

Edit `.env.edge` to customize:

**Kong Settings**:
```bash
KONG_DB_PASSWORD=changeme
KONG_LOG_LEVEL=info
KONG_ENABLED=true
```

**AgentGateway Settings**:
```bash
AG_LOG_LEVEL=info
AG_MCP_ENABLED=true
AG_RATE_LIMIT_RPS=100
AGENTGATEWAY_ENABLED=true
```

Load environment variables:
```bash
export $(cat .env.edge | xargs)
docker compose -f docker/docker-compose.edge.yml --env-file .env.edge up -d
```

### AgentGateway Configuration

Edit `/home/user/ripple-env/config/agentgateway/config.yaml`:

```yaml
server:
  port: 8090
  admin_port: 8091
  metrics_port: 8092

mcp:
  enabled: true
  transports: [stdio, http]

rate_limit:
  enabled: true
  requests_per_second: 100
  burst: 200
```

## Verification

### Health Checks

```bash
# Kong Gateway
curl http://localhost:8001/status

# AgentGateway
curl http://localhost:8090/health

# Expected response:
# {"status":"healthy","upstreams":{"kong":"healthy","localai":"healthy"}}
```

### Service Status

```bash
# Check all services
docker compose -f docker/docker-compose.edge.yml ps

# Expected output:
# NAME               STATUS              PORTS
# kong-database      healthy             5432/tcp
# kong               healthy             8000-8002, 8443-8444
# agentgateway       healthy             8090-8092
# konga              running             1337
```

### Automated Verification

```bash
# Run comprehensive verification
./scripts/verify-edge.sh

# Expected output:
# ========================================
# ARIA Edge Services Verification
# ========================================
# ...
# Total Tests: 25
# Passed: 25
# Failed: 0
```

## Feature Flags

The following feature flags are configured:

| Flag | Service | Default | Description |
|------|---------|---------|-------------|
| `KONG_ENABLED` | Kong | true | Enable Kong Gateway |
| `AGENTGATEWAY_ENABLED` | AgentGateway | true | Enable AgentGateway |
| `AG_MCP_ENABLED` | AgentGateway | true | Enable MCP protocol |
| `AG_RATE_LIMIT_ENABLED` | AgentGateway | true | Enable rate limiting |
| `AG_CIRCUIT_BREAKER_ENABLED` | AgentGateway | true | Enable circuit breaker |

## Kong Configuration

### Add a Service

```bash
curl -i -X POST http://localhost:8001/services \
  --data name=localai \
  --data url=http://localai:8080
```

### Add a Route

```bash
curl -i -X POST http://localhost:8001/services/localai/routes \
  --data paths[]=/ai \
  --data methods[]=GET \
  --data methods[]=POST
```

### Test the Route

```bash
curl http://localhost:8000/ai/v1/models
```

### Using Konga UI

1. Access http://localhost:1337
2. Create admin account on first login
3. Add Kong connection:
   - Name: Kong
   - Kong Admin URL: http://kong:8001
4. Start managing Kong through the UI

## AgentGateway Usage

### Health Check

```bash
curl http://localhost:8090/health
```

### Metrics

```bash
curl http://localhost:8092/metrics
```

**Key Metrics**:
- `agentgateway_requests_total` - Total requests
- `agentgateway_request_duration_seconds` - Request latency
- `agentgateway_upstream_requests_total` - Upstream requests
- `agentgateway_circuit_breaker_state` - Circuit breaker status

### MCP Protocol

AgentGateway supports Model Context Protocol (MCP):

**Transports**: stdio, http
**Max Message Size**: 10MB
**Timeout**: 60s

## Monitoring

### View Logs

```bash
# All services
docker compose -f docker/docker-compose.edge.yml logs -f

# Specific service
docker compose -f docker/docker-compose.edge.yml logs -f kong
docker compose -f docker/docker-compose.edge.yml logs -f agentgateway
```

### Service Metrics

```bash
# AgentGateway metrics
curl http://localhost:8092/metrics | grep agentgateway_

# Kong status
curl http://localhost:8001/status | jq
```

## Troubleshooting

### Services Not Starting

```bash
# Check service status
docker compose -f docker/docker-compose.edge.yml ps

# Check logs
docker compose -f docker/docker-compose.edge.yml logs

# Restart services
docker compose -f docker/docker-compose.edge.yml restart
```

### Kong Database Issues

```bash
# Check database health
docker inspect kong-database | jq '.[0].State.Health'

# Run migrations manually
docker compose -f docker/docker-compose.edge.yml run --rm kong-migrations

# Reset database (WARNING: destroys data)
docker compose -f docker/docker-compose.edge.yml down -v
docker compose -f docker/docker-compose.edge.yml up -d
```

### Network Issues

```bash
# Check network exists
docker network ls | grep agentic-network

# Recreate network
docker network create agentic-network

# Check network connectivity
docker exec kong ping -c 3 kong-database
docker exec agentgateway ping -c 3 kong
```

### Port Conflicts

If ports are already in use, modify in `.env.edge`:

```bash
KONG_PROXY_PORT=18000
KONG_ADMIN_PORT=18001
AGENTGATEWAY_API_PORT=18090
```

## Maintenance

### Update Services

```bash
# Pull latest images
docker compose -f docker/docker-compose.edge.yml pull

# Restart with new images
docker compose -f docker/docker-compose.edge.yml up -d
```

### Backup Configuration

```bash
# Backup Kong configuration
curl http://localhost:8001/config > kong-config-backup.json

# Backup AgentGateway config
tar -czf agentgateway-config-backup.tar.gz config/agentgateway/
```

### Clean Up

```bash
# Stop services
docker compose -f docker/docker-compose.edge.yml down

# Remove volumes (WARNING: destroys data)
docker compose -f docker/docker-compose.edge.yml down -v

# Remove network
docker network rm agentic-network
```

## Performance Tuning

### Kong Performance

Edit environment variables in `docker/docker-compose.edge.yml`:

```yaml
KONG_NGINX_WORKER_PROCESSES: 4
KONG_UPSTREAM_KEEPALIVE_POOL_SIZE: 100
KONG_UPSTREAM_KEEPALIVE_MAX_REQUESTS: 1000
```

### AgentGateway Performance

Edit `.env.edge`:

```bash
AG_MAX_CONNECTIONS=5000
AG_RATE_LIMIT_RPS=500
AG_RATE_LIMIT_BURST=1000
AG_UPSTREAM_TIMEOUT=60s
```

## Security Considerations

1. **Change default passwords** in production
2. **Enable TLS** for Kong proxy and admin API
3. **Restrict admin API access** to trusted networks only
4. **Enable Kong authentication plugins** for your APIs
5. **Monitor rate limits** and adjust as needed
6. **Review logs regularly** for suspicious activity

## Integration with Other Services

### With LocalAI

AgentGateway is pre-configured to route to LocalAI:

```bash
# Through Kong
curl http://localhost:8000/ai/v1/models

# Through AgentGateway
curl http://localhost:8090/v1/models
```

### With Observability Stack

Metrics are exposed at:
- Kong: http://localhost:8001/metrics
- AgentGateway: http://localhost:8092/metrics

Configure Prometheus to scrape these endpoints.

## Support

For issues or questions:
1. Check logs: `docker compose logs <service>`
2. Run verification: `./scripts/verify-edge.sh`
3. Review documentation
4. Check Kong documentation: https://docs.konghq.com/
5. Check service health: `docker inspect <service>`

## References

- Kong Gateway: https://konghq.com/
- Konga: https://github.com/pantsel/konga
- Model Context Protocol (MCP): https://modelcontextprotocol.io/
- Docker Compose: https://docs.docker.com/compose/

## Changelog

### Version 1.0.0 (2026-01-09)
- Initial deployment configuration
- P0-003: Kong Gateway deployment
- P0-004: AgentGateway deployment
- Comprehensive verification scripts
- Environment variable templates
- Deployment automation
