# Docker Compose Profiles Guide

**Status**: Active
**Date**: 2026-01-12
**Purpose**: Guide for using Docker Compose profiles to selectively deploy FlexStack services

## Overview

FlexStack uses Docker Compose profiles to organize 30+ services into logical groups. This allows you to:
- Deploy only the services you need
- Reduce resource usage during development
- Scale components independently
- Simplify dependency management

## Quick Start

```bash
# Navigate to docker directory
cd docker

# Start minimal development stack (core + AI)
docker compose --profile dev up -d

# Start with specific profiles
docker compose --profile core --profile ai up -d

# Start everything
docker compose --profile full up -d

# Or use the helper script
./scripts/flexstack.sh up --profile ai
```

## Available Profiles

| Profile | Services | RAM Required | Use Case |
|---------|----------|--------------|----------|
| `core` | PostgreSQL, Redis, MinIO | 4GB | Infrastructure base |
| `dev` | Core + LocalAI | 12GB | Development |
| `ai` | LocalAI, AGiXT, MindsDB | 16GB | AI/ML workloads |
| `observability` | Prometheus, Grafana, Loki, Tempo | 8GB | Monitoring |
| `identity` | Keycloak, Vault, Step-CA | 4GB | Auth & secrets |
| `data` | Neo4j, MindsDB | 6GB | Graph & ML databases |
| `edge` | Kong, AgentGateway | 2GB | API gateway |
| `messaging` | NATS, Temporal | 4GB | Event bus & workflows |
| `automation` | n8n, OPA | 2GB | Workflow automation |
| `full` | All services | 32GB+ | Production-like |

## Profile Details

### Core Profile (`--profile core`)

Essential infrastructure services required by most other profiles.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| PostgreSQL | 5432 | Shared database (multi-tenant) |
| Redis | 6379 | Cache and state store |
| MinIO | 9000/9001 | S3-compatible object storage |

**Resource Requirements:**
- RAM: 4GB minimum
- CPU: 2 cores
- Disk: 10GB

**Usage:**
```bash
docker compose --profile core up -d

# Verify
docker compose ps
psql -h localhost -U postgres -c "SELECT 1"
redis-cli ping
curl http://localhost:9000/minio/health/live
```

### Development Profile (`--profile dev`)

Minimal stack for local development with AI capabilities.

**Includes:** Core + LocalAI

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| *All core services* | - | - |
| LocalAI | 8080 | Local LLM inference |

**Resource Requirements:**
- RAM: 12GB minimum (16GB recommended)
- CPU: 4 cores
- Disk: 30GB (including models)

**Usage:**
```bash
docker compose --profile dev up -d

# Test LocalAI
curl http://localhost:8080/v1/models
```

### AI Profile (`--profile ai`)

Full AI/ML stack for inference and orchestration.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| LocalAI | 8080 | LLM inference server |
| AGiXT API | 7437 | Agent orchestration |
| AGiXT UI | 3437 | Web interface |
| MindsDB | 47334-6 | ML database layer |

**Dependencies:** Requires `core` profile

**Resource Requirements:**
- RAM: 16GB minimum (24GB recommended)
- CPU: 8 cores
- GPU: Optional (8GB+ VRAM for acceleration)
- Disk: 50GB

**Usage:**
```bash
docker compose --profile core --profile ai up -d

# Or with dev profile (includes core)
docker compose --profile dev --profile ai up -d
```

### Observability Profile (`--profile observability`)

Complete monitoring, logging, and tracing stack.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| Prometheus | 9090 | Metrics collection |
| Grafana | 3000 | Dashboards & visualization |
| Loki | 3100 | Log aggregation |
| Promtail | - | Log shipper |
| Tempo | 3200 | Distributed tracing |
| Alertmanager | 9093 | Alert routing |
| Node Exporter | 9100 | Host metrics |
| cAdvisor | 8085 | Container metrics |

**Resource Requirements:**
- RAM: 8GB minimum
- CPU: 4 cores
- Disk: 50GB (for retention)

**Usage:**
```bash
docker compose --profile observability up -d

# Access Grafana
open http://localhost:3000
# Default: admin/admin
```

### Identity Profile (`--profile identity`)

Authentication, authorization, and secrets management.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| Step-CA | 9443 | Certificate Authority |
| Keycloak | 8082/8443 | OIDC provider |
| Vault | 8200 | Secrets management |
| Vaultwarden | 8081 | Password vault |

**Dependencies:** Requires `core` profile (for PostgreSQL)

**Resource Requirements:**
- RAM: 4GB minimum
- CPU: 2 cores
- Disk: 5GB

**Usage:**
```bash
docker compose --profile core --profile identity up -d

# Access Keycloak admin
open http://localhost:8082/admin
# Default: admin/changeme
```

### Data Profile (`--profile data`)

Graph database and data query services.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| Neo4j | 7474/7687 | Graph database |
| MindsDB | 47334-6 | ML database (shared with ai) |

**Resource Requirements:**
- RAM: 6GB minimum
- CPU: 4 cores
- Disk: 20GB

**Usage:**
```bash
docker compose --profile core --profile data up -d

# Access Neo4j Browser
open http://localhost:7474
# Default: neo4j/changeme
```

### Edge Profile (`--profile edge`)

API gateway and agent traffic management.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| Kong Gateway | 8000-8002 | API gateway |
| Kong DB | - | Kong's PostgreSQL |
| AgentGateway | 8090-8092 | MCP traffic plane |

**Resource Requirements:**
- RAM: 2GB minimum
- CPU: 2 cores
- Disk: 2GB

**Usage:**
```bash
docker compose --profile edge up -d

# Verify Kong
curl http://localhost:8001/status

# Access Kong Manager
open http://localhost:8002
```

### Messaging Profile (`--profile messaging`)

Event bus and durable workflow orchestration.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| NATS | 4222/8222 | Event bus |
| Temporal | 7233/7239 | Workflow engine |
| Temporal UI | 8088 | Web interface |

**Dependencies:** Requires `core` profile (for PostgreSQL)

**Resource Requirements:**
- RAM: 4GB minimum
- CPU: 4 cores
- Disk: 10GB

**Usage:**
```bash
docker compose --profile core --profile messaging up -d

# Access Temporal UI
open http://localhost:8088
```

### Automation Profile (`--profile automation`)

Workflow automation and policy management.

**Services:**
| Service | Port | Description |
|---------|------|-------------|
| n8n | 5678 | Workflow automation |
| OPA | 8181 | Policy agent |

**Dependencies:** Requires `core` profile (for PostgreSQL)

**Resource Requirements:**
- RAM: 2GB minimum
- CPU: 2 cores
- Disk: 5GB

**Usage:**
```bash
docker compose --profile core --profile automation up -d

# Access n8n
open http://localhost:5678
# Default: admin/changeme
```

### Full Profile (`--profile full`)

All services for production-like environments.

**Resource Requirements:**
- RAM: 32GB minimum (64GB recommended)
- CPU: 16 cores
- GPU: Recommended (16GB+ VRAM)
- Disk: 100GB

**Usage:**
```bash
docker compose --profile full up -d

# Check all services
docker compose ps
```

## Service Dependencies

```
                    ┌──────────────┐
                    │   Step-CA    │
                    │  (identity)  │
                    └──────┬───────┘
                           │
              ┌────────────┼────────────┐
              ▼            ▼            ▼
        ┌──────────┐ ┌──────────┐ ┌──────────┐
        │ Keycloak │ │  Vault   │ │Vaultwarden│
        │(identity)│ │(identity)│ │(identity) │
        └────┬─────┘ └──────────┘ └───────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────┐
│                      PostgreSQL                          │
│                        (core)                            │
└─────────────────────────────────────────────────────────┘
    ▲           ▲           ▲           ▲           ▲
    │           │           │           │           │
┌───┴───┐  ┌────┴───┐  ┌────┴───┐  ┌────┴───┐  ┌────┴───┐
│ n8n   │  │Temporal│  │MindsDB │  │Keycloak│  │ AGiXT  │
│(auto) │  │ (msg)  │  │(ai/data)│ │(ident) │  │  (ai)  │
└───────┘  └────────┘  └─────────┘ └────────┘  └────┬───┘
                                                     │
┌────────────────────────────────────────────────────┼─────┐
│                         MinIO                      │     │
│                        (core)                      │     │
└────────────────────────────────────────────────────┴─────┘
                                                     │
                                               ┌─────┴─────┐
                                               │  LocalAI  │
                                               │   (ai)    │
                                               └───────────┘
```

## Common Workflows

### Development Workflow

Start minimal stack for development:

```bash
# 1. Start core infrastructure
docker compose --profile dev up -d

# 2. Wait for services to be healthy
docker compose ps

# 3. Add observability when needed
docker compose --profile dev --profile observability up -d
```

### Production-Like Testing

Test full stack locally:

```bash
# 1. Ensure sufficient resources
./scripts/validate-resources.sh --profile full

# 2. Start all services
docker compose --profile full up -d

# 3. Monitor startup
docker compose logs -f
```

### Profile Combinations

Common profile combinations:

```bash
# AI Development with monitoring
docker compose --profile dev --profile ai --profile observability up -d

# Backend services (no AI)
docker compose --profile core --profile identity --profile messaging up -d

# API gateway development
docker compose --profile core --profile edge up -d

# Full observability stack
docker compose --profile core --profile observability up -d
```

## Resource Validation

Before starting profiles, validate system resources:

```bash
# Check resources for specific profile
./scripts/validate-resources.sh --profile ai

# Check for full stack
./scripts/validate-resources.sh --profile full

# List resource requirements
./scripts/validate-resources.sh --list
```

## Troubleshooting

### Service Won't Start

```bash
# Check logs for specific service
docker compose logs -f <service-name>

# Check dependencies
docker compose ps

# Restart specific service
docker compose restart <service-name>
```

### Dependency Issues

If services fail due to missing dependencies:

```bash
# Start dependencies first
docker compose --profile core up -d
sleep 30

# Then add dependent profiles
docker compose --profile core --profile ai up -d
```

### Resource Exhaustion

```bash
# Check container resource usage
docker stats

# Reduce resource limits in docker-compose.yml
# Or use lighter profiles

# Stop resource-heavy services
docker compose stop mindsdb neo4j
```

### Port Conflicts

Check for port conflicts:

```bash
# List ports in use
docker compose ps --format "table {{.Name}}\t{{.Ports}}"

# Or use netstat
netstat -tlnp | grep -E "(8080|9090|3000)"
```

## Migration from Individual Compose Files

If you were using individual docker-compose files:

```bash
# Old way
docker compose -f docker/docker-compose.observability.yml up -d
docker compose -f docker/docker-compose.identity.yml up -d

# New way
docker compose --profile observability --profile identity up -d
```

Individual compose files are still available for isolated testing:

```bash
# Use individual file for isolation
docker compose -f docker/docker-compose.observability.yml up -d
```

## Related Documentation

- [AI_RESOURCE_REQUIREMENTS.md](../ai-ml/AI_RESOURCE_REQUIREMENTS.md) - Detailed AI service resources
- [GETTING_STARTED.md](../getting-started/GETTING_STARTED.md) - Quick start guide
- [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - Common issues and solutions

## References

- [Docker Compose Profiles](https://docs.docker.com/compose/profiles/)
- [Docker Resource Constraints](https://docs.docker.com/config/containers/resource_constraints/)
- [Docker Compose Specification](https://docs.docker.com/compose/compose-file/)
