# Port Mappings Registry

**Status:** Complete
**Last Updated:** 2026-01-14
**Source:** Extracted from `docker/docker-compose*.yml` files

---

## Summary

| Category | Port Range | Service Count |
|----------|------------|---------------|
| Core Infrastructure | 5432, 6379, 9000-9001 | 3 |
| Observability | 3000-3200, 4317-4318, 8080, 8888-8889, 9090-9100, 13133, 19999 | 12 |
| AI/ML Services | 7437, 8080, 47334-47336 | 4 |
| Identity & Security | 8081-8082, 8200, 8443, 9443 | 5 |
| Data Services | 7474, 7687 | 1 |
| Edge/Gateway | 8000-8002, 8090-8092, 8443 | 3 |
| Messaging & Workflows | 4222, 6222, 7233, 7239, 8088, 8222 | 3 |
| Automation | 5678, 8181 | 2 |

---

## Core Infrastructure

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| PostgreSQL | 5432 | TCP | Database connections | `docker-compose.yml` |
| Redis | 6379 | TCP | Cache/state store | `docker-compose.yml` |
| MinIO (API) | 9000 | HTTP | S3-compatible API | `docker-compose.yml` |
| MinIO (Console) | 9001 | HTTP | Web console | `docker-compose.yml` |

---

## Observability Stack

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| Prometheus | 9090 | HTTP | Metrics collection | `docker-compose.observability.yml` |
| Grafana | 3000 | HTTP | Dashboards & visualization | `docker-compose.observability.yml` |
| Loki | 3100 | HTTP | Log aggregation API | `docker-compose.observability.yml` |
| Tempo (HTTP) | 3200 | HTTP | Tracing API | `docker-compose.observability.yml` |
| Tempo (OTLP gRPC) | 4317 | gRPC | OpenTelemetry traces | `docker-compose.observability.yml` |
| Tempo (OTLP HTTP) | 4318 | HTTP | OpenTelemetry traces | `docker-compose.observability.yml` |
| Tempo (gRPC) | 9095 | gRPC | Tempo native protocol | `docker-compose.observability.yml` |
| Tempo (Jaeger gRPC) | 14250 | gRPC | Jaeger-compatible traces | `docker-compose.observability.yml` |
| Tempo (Jaeger HTTP) | 14268 | HTTP | Jaeger-compatible traces | `docker-compose.observability.yml` |
| Alertmanager | 9093 | HTTP | Alert management | `docker-compose.observability.yml` |
| Node Exporter | 9100 | HTTP | Host metrics | `docker-compose.observability.yml` |
| cAdvisor | 8080/8085 | HTTP | Container metrics | `docker-compose.observability.yml` / `docker-compose.yml` |
| OTel Collector (metrics) | 8888 | HTTP | Prometheus metrics | `docker-compose.observability.yml` |
| OTel Collector (exporter) | 8889 | HTTP | Prometheus exporter | `docker-compose.observability.yml` |
| OTel Collector (health) | 13133 | HTTP | Health check | `docker-compose.observability.yml` |
| Netdata | 19999 | HTTP | Real-time monitoring | `docker-compose.observability.yml` |
| Umami | 3001 | HTTP | Product analytics | `docker-compose.observability.yml` |

---

## AI/ML Services

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| LocalAI | 8080 | HTTP | OpenAI-compatible LLM API | `docker-compose.yml` |
| AGiXT API | 7437 | HTTP | Agent orchestration API | `docker-compose.yml` |
| AGiXT UI | 3437 | HTTP | Interactive agent UI | `docker-compose.yml` |
| MindsDB (HTTP) | 47334 | HTTP | ML database API | `docker-compose.yml` |
| MindsDB (MySQL) | 47335 | MySQL | MySQL-compatible protocol | `docker-compose.yml` |
| MindsDB (MongoDB) | 47336 | MongoDB | MongoDB-compatible protocol | `docker-compose.yml` |

---

## Identity & Security

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| Step-CA | 9443 | HTTPS | Certificate Authority API | `docker-compose.yml` |
| Keycloak (HTTP) | 8082 | HTTP | OIDC/OAuth2 provider | `docker-compose.yml` |
| Keycloak (HTTPS) | 8443 | HTTPS | OIDC/OAuth2 provider (TLS) | `docker-compose.yml` |
| Vault | 8200 | HTTP | Secrets management | `docker-compose.yml` |
| Vaultwarden | 8081 | HTTP | Password vault | `docker-compose.yml` |
| Vaultwarden (WebSocket) | 3012 | WS | Live notifications | `docker-compose.yml` |

---

## Data Services

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| Neo4j (HTTP) | 7474 | HTTP | Graph database browser | `docker-compose.yml` |
| Neo4j (Bolt) | 7687 | Bolt | Graph database protocol | `docker-compose.yml` |

---

## Edge/Gateway Services

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| Kong (Proxy HTTP) | 8000 | HTTP | API Gateway proxy | `docker-compose.yml` |
| Kong (Proxy HTTPS) | 8443 | HTTPS | API Gateway proxy (TLS) | `docker-compose.yml` |
| Kong (Admin API) | 8001 | HTTP | Gateway admin API | `docker-compose.yml` |
| Kong (Admin GUI) | 8002 | HTTP | Gateway admin console | `docker-compose.yml` |
| AgentGateway (API) | 8090 | HTTP | Agent traffic plane | `docker-compose.yml` |
| AgentGateway (Admin) | 8091 | HTTP | Admin API | `docker-compose.yml` |
| AgentGateway (Metrics) | 8092 | HTTP | Prometheus metrics | `docker-compose.yml` |

---

## Messaging & Workflows

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| NATS (Client) | 4222 | TCP | NATS client connections | `docker-compose.yml` |
| NATS (Cluster) | 6222 | TCP | NATS cluster routing | `docker-compose.yml` |
| NATS (HTTP) | 8222 | HTTP | NATS monitoring API | `docker-compose.yml` |
| Temporal (gRPC) | 7233 | gRPC | Workflow engine | `docker-compose.yml` |
| Temporal (Metrics) | 7239 | HTTP | Prometheus metrics | `docker-compose.yml` |
| Temporal UI | 8088 | HTTP | Workflow dashboard | `docker-compose.yml` |

---

## Automation

| Service | Port | Protocol | Purpose | Compose File |
|---------|------|----------|---------|--------------|
| n8n | 5678 | HTTP | Workflow automation UI | `docker-compose.yml` |
| OPA | 8181 | HTTP | Policy decision API | `docker-compose.yml` |

---

## Port Conflict Check

### Potential Conflicts

| Port | Services | Resolution |
|------|----------|------------|
| 8080 | LocalAI, cAdvisor | cAdvisor mapped to 8085 in main compose |
| 8443 | Keycloak, Kong | Different networks; both use TLS |
| 4317 | Tempo, OTel Collector | Same service chain; OTel forwards to Tempo |
| 4318 | Tempo, OTel Collector | Same service chain; OTel forwards to Tempo |

### Reserved Ranges

- **3000-3999**: Web UIs and dashboards
- **4000-4999**: Messaging protocols
- **5000-5999**: Automation and internal services
- **6000-6999**: Cluster communication
- **7000-7999**: Databases and workflows
- **8000-8999**: HTTP APIs and gateways
- **9000-9999**: Metrics and monitoring
- **47000-47999**: MindsDB services

---

## Environment Variable Overrides

Many ports can be customized via environment variables in `.env`:

```bash
# Example port overrides (not all services support this)
LOCALAI_PORT=8080
GRAFANA_PORT=3000
KEYCLOAK_PORT=8082
KONG_PROXY_PORT=8000
NATS_CLIENT_PORT=4222
TEMPORAL_PORT=7233
```

---

## Security Considerations

### External Exposure

**Only expose these ports externally in production:**
- Kong Proxy (8000/8443) - API Gateway
- Keycloak (8082/8443) - Identity Provider

**Keep internal only:**
- All database ports (5432, 6379, 7474, 7687)
- Admin APIs (8001, 8181, 8200)
- Metrics endpoints (9090, 9100, etc.)

### Firewall Rules

```bash
# Example: Allow only gateway traffic
ufw allow 8000/tcp  # Kong HTTP
ufw allow 8443/tcp  # Kong HTTPS
ufw deny 5432/tcp   # Block direct Postgres
ufw deny 6379/tcp   # Block direct Redis
```

---

**See Also:**
- [ARCHITECTURE.md](ARCHITECTURE.md) - Network topology
- [CONTAINER_SECURITY.md](CONTAINER_SECURITY.md) - Security guidance
- [PROVIDERS.md](PROVIDERS.md) - Provider configuration
