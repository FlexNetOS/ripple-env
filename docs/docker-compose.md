# Docker Compose Service Architecture

This document describes the Docker Compose services, their dependencies, and how to use them.

## Service Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           agentic-network                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │ LocalAI     │     │   AGiXT     │     │  Temporal   │              │
│  │  :8080      │ ──► │   :7437     │     │   :7233     │              │
│  └─────────────┘     └──────┬──────┘     └──────┬──────┘              │
│                             │                    │                     │
│                       ┌─────▼─────┐        ┌─────▼─────┐              │
│                       │ PostgreSQL│        │ PostgreSQL│              │
│                       │   :5432   │        │   :5433   │              │
│                       └───────────┘        └───────────┘              │
│                                                                         │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │ Prometheus  │     │   Grafana   │     │    Tempo    │              │
│  │  :9090      │ ◄── │   :3000     │     │   :3200     │              │
│  └─────────────┘     └─────────────┘     └─────────────┘              │
│                                                                         │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐              │
│  │    NATS     │     │     OPA     │     │    n8n      │              │
│  │  :4222      │     │   :8181     │     │   :5678     │              │
│  └─────────────┘     └─────────────┘     └─────────────┘              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## Service Dependencies

### Dependency Matrix

| Service | Depends On | Health Check | Port(s) |
|---------|------------|--------------|---------|
| AGiXT API | postgres, minio | HTTP /api/health | 7437 |
| AGiXT UI | agixt | - | 3437 |
| LocalAI | - | HTTP /readyz | 8080 |
| Temporal | temporal-postgresql | tctl cluster health | 7233 |
| Temporal UI | temporal | HTTP / | 8088 |
| PostgreSQL | - | pg_isready | 5432 |
| NATS | - | HTTP /healthz | 4222, 8222 |
| Prometheus | - | HTTP /-/healthy | 9090 |
| Grafana | prometheus | HTTP /api/health | 3000 |
| OPA | - | HTTP /health | 8181 |
| n8n | - | HTTP /healthz | 5678 |
| Kong | - | HTTP /status | 8001, 8000 |

### Startup Order

For the full stack, services should start in this order:

1. **Databases**: PostgreSQL instances
2. **Messaging**: NATS server
3. **Core Services**: Temporal, OPA, Prometheus
4. **AI Services**: LocalAI, AGiXT
5. **UI/Dashboards**: Grafana, Temporal UI, n8n

## Docker Compose Files

### Core Stacks

| File | Purpose | Services |
|------|---------|----------|
| `docker-compose.localai.yml` | Local LLM inference | LocalAI |
| `docker-compose.agixt.yml` | AI agent platform | AGiXT, PostgreSQL, MinIO |
| `docker-compose.temporal.yml` | Workflow orchestration | Temporal, PostgreSQL |
| `docker-compose.messaging.yml` | Messaging layer | NATS |
| `docker-compose.observability.yml` | Monitoring | Prometheus, Grafana, Tempo |
| `docker-compose.automation.yml` | Automation | n8n, OPA |

### Specialized Stacks

| File | Purpose |
|------|---------|
| `docker-compose.identity.yml` | Keycloak, Vaultwarden |
| `docker-compose.edge.yml` | Kong Gateway |
| `docker-compose.data.yml` | Database services |
| `docker-compose.holochain.yml` | P2P coordination |

## Quick Start

### Create Network

All services share the `agentic-network`:

```bash
docker network create agentic-network
```

### Start Core Stack

```bash
# Observability first (for monitoring)
docker-compose -f docker/docker-compose.observability.yml up -d

# Messaging
docker-compose -f docker/docker-compose.messaging.yml up -d

# Automation
docker-compose -f docker/docker-compose.automation.yml up -d

# AI Services
docker-compose -f docker/docker-compose.localai.yml up -d
docker-compose -f docker/docker-compose.agixt.yml up -d
```

### Using Helper Commands

The Nix shell provides helper commands:

```bash
# Enter development shell
nix develop

# Start services
agixt up       # Start AGiXT stack
localai start  # Start LocalAI
temporal-ctl start  # Start Temporal
```

## Health Checks

All services include health checks. Check status:

```bash
# Using the health-check script
./scripts/health-check.sh http://localhost:8080/readyz 5 5 "LocalAI"
./scripts/health-check.sh http://localhost:7437/api/health 5 5 "AGiXT"
./scripts/health-check.sh http://localhost:9090/-/healthy 5 5 "Prometheus"
```

### Health Check Configuration

Example health check in docker-compose:

```yaml
services:
  myservice:
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s      # Check every 30 seconds
      timeout: 10s       # Timeout after 10 seconds
      retries: 3         # Fail after 3 retries
      start_period: 60s  # Grace period for startup
```

## Service Dependencies with Health Conditions

Use `depends_on` with `condition: service_healthy`:

```yaml
services:
  api:
    depends_on:
      database:
        condition: service_healthy
      cache:
        condition: service_started
```

## Environment Variables

### Common Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `POSTGRES_PASSWORD` | PostgreSQL password | (service-specific) |
| `OPENAI_API_BASE` | LLM API endpoint | http://localhost:8080/v1 |
| `OPENAI_API_KEY` | API key for LLM | sk-localai |

### Per-Service Variables

See individual `docker-compose.*.yml` files for service-specific configuration.

## Volumes

### Data Persistence

```yaml
volumes:
  postgres-data:      # PostgreSQL data
  localai-models:     # AI models
  grafana-data:       # Grafana dashboards
  prometheus-data:    # Metrics data
```

### Default Locations

Data is stored in `./data/<service>/`:

```
./data/
├── localai/models/     # AI models
├── postgres/           # Database files
├── grafana/            # Dashboard configs
└── prometheus/         # Metrics
```

## Troubleshooting

### Service Won't Start

```bash
# Check logs
docker-compose -f docker/docker-compose.<service>.yml logs -f

# Check health
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"

# Inspect container
docker inspect <container_name>
```

### Network Issues

```bash
# List networks
docker network ls

# Inspect network
docker network inspect agentic-network

# Recreate network
docker network rm agentic-network
docker network create agentic-network
```

### Port Conflicts

```bash
# Find what's using a port
lsof -i :8080

# Or on Linux
ss -tlnp | grep 8080
```

### Clean Restart

```bash
# Stop all services
docker-compose -f docker/docker-compose.*.yml down

# Remove volumes (CAUTION: data loss)
docker-compose -f docker/docker-compose.*.yml down -v

# Remove network
docker network rm agentic-network

# Start fresh
docker network create agentic-network
docker-compose -f docker/docker-compose.observability.yml up -d
```

## Best Practices

1. **Always use specific image tags** (not `latest`)
2. **Set resource limits** in production
3. **Use health checks** for all services
4. **Configure proper logging** (JSON format)
5. **Use secrets management** for sensitive data
6. **Enable TLS** for production deployments
