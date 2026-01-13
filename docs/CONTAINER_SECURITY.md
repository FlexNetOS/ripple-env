# Container Security Guide

This document provides comprehensive security guidelines for the ARIA platform's Docker container stack, including network segmentation, port exposure, mTLS configuration, and security best practices.

## Table of Contents

- [Network Architecture](#network-architecture)
- [Network Segmentation](#network-segmentation)
- [Port Exposure Matrix](#port-exposure-matrix)
- [Security Boundaries](#security-boundaries)
- [Firewall Configuration](#firewall-configuration)
- [mTLS Service Mesh](#mtls-service-mesh)
- [Container Image Security](#container-image-security)
- [Security Scanning](#security-scanning)
- [Hardening Guidelines](#hardening-guidelines)

## Network Architecture

The ARIA platform uses a segmented network architecture to isolate services by function and trust level.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              EXTERNAL NETWORK                                │
│                         (Public/Internet-facing)                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                              ┌───────┴───────┐
                              │   FIREWALL    │
                              │  (ufw/nftables)│
                              └───────┬───────┘
                                      │
┌─────────────────────────────────────────────────────────────────────────────┐
│                              DMZ NETWORK                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                          │
│  │    Kong     │  │   Nginx     │  │   Traefik   │                          │
│  │ (API GW)    │  │  (Reverse)  │  │  (Ingress)  │                          │
│  └─────────────┘  └─────────────┘  └─────────────┘                          │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
        ┌─────────────────────────────┼─────────────────────────────┐
        │                             │                             │
        ▼                             ▼                             ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│  identity-network │   │  agentic-network  │   │ observability-net │
│                   │   │                   │   │                   │
│  • Keycloak       │   │  • LocalAI        │   │  • Grafana        │
│  • Vault          │   │  • AGiXT          │   │  • Prometheus     │
│  • Vaultwarden    │   │  • MindsDB        │   │  • Loki           │
│  • Step-CA        │   │  • Neo4j          │   │  • Tempo          │
│  • OPA            │   │  • N8N            │   │  • Alertmanager   │
│                   │   │  • Temporal       │   │  • OTEL Collector │
└───────────────────┘   └───────────────────┘   └───────────────────┘
        │                             │                             │
        └─────────────────────────────┼─────────────────────────────┘
                                      │
                                      ▼
                        ┌───────────────────┐
                        │   data-network    │
                        │                   │
                        │  • PostgreSQL     │
                        │  • Redis          │
                        │  • MinIO          │
                        │  • ClickHouse     │
                        └───────────────────┘
```

## Network Segmentation

### Network Definitions

| Network | Purpose | Trust Level | Isolation |
|---------|---------|-------------|-----------|
| `identity-network` | Identity, authentication, and secrets management | HIGH | Strict - limited ingress |
| `agentic-network` | AI/ML services, workflow automation | MEDIUM | Moderate - API access only |
| `observability-network` | Monitoring, logging, tracing | MEDIUM | Read-only data collection |
| `data-network` | Persistent data storage | HIGH | No external access |
| `messaging-network` | NATS, event streaming | MEDIUM | Internal pub/sub only |

### Docker Network Configuration

Create the required networks before starting services:

```bash
# Create isolated networks with specific subnets
docker network create --driver bridge \
  --subnet=172.20.0.0/16 \
  --ip-range=172.20.1.0/24 \
  identity-network

docker network create --driver bridge \
  --subnet=172.21.0.0/16 \
  --ip-range=172.21.1.0/24 \
  agentic-network

docker network create --driver bridge \
  --subnet=172.22.0.0/16 \
  --ip-range=172.22.1.0/24 \
  observability-network

docker network create --driver bridge \
  --subnet=172.23.0.0/16 \
  --ip-range=172.23.1.0/24 \
  data-network

docker network create --driver bridge \
  --subnet=172.24.0.0/16 \
  --ip-range=172.24.1.0/24 \
  messaging-network
```

### Network Access Matrix

| Source Network | → identity | → agentic | → observability | → data | → messaging |
|----------------|------------|-----------|-----------------|--------|-------------|
| identity       | ✅ | ✅ (auth) | ✅ (metrics) | ✅ (secrets) | ❌ |
| agentic        | ✅ (auth) | ✅ | ✅ (metrics) | ✅ (storage) | ✅ |
| observability  | ✅ (metrics) | ✅ (metrics) | ✅ | ❌ | ❌ |
| data           | ❌ | ❌ | ✅ (metrics) | ✅ | ❌ |
| messaging      | ❌ | ✅ (events) | ✅ (metrics) | ❌ | ✅ |

## Port Exposure Matrix

### Identity & Access Management (Layer 5)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| Keycloak | 8080 | 8080 | HTTP | Internal/Dev | Use HTTPS (8443) in production |
| Keycloak | 8443 | 8443 | HTTPS | External | mTLS recommended |
| Vault | 8200 | 8200 | HTTP/HTTPS | Internal | Enable TLS, use transit secrets |
| Vaultwarden | 80 | 8081 | HTTP | Internal | HTTPS via reverse proxy |
| Vaultwarden | 3012 | 3012 | WebSocket | Internal | For notifications |
| Step-CA | 9000 | 9000 | HTTPS | Internal | Certificate authority |
| OPA | 8181 | 8181 | HTTP | Internal | Policy decisions |

### AI/ML Services (Layer 12)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| LocalAI | 8080 | 8080* | HTTP | Internal | API key recommended |
| AGiXT | 7437 | 7437 | HTTP | Internal | Authentication required |
| AGiXT-Interactive | 3000 | 3000 | HTTP | Internal | Frontend only |
| MindsDB | 47334 | 47334 | HTTP | Internal | API key required |
| MindsDB | 47335 | 47335 | MySQL | Internal | Use strong credentials |
| MindsDB | 47336 | 47336 | MongoDB | Internal | Use strong credentials |
| Refact | 8008 | 8008 | HTTP | Internal | Code completion API |

*Note: LocalAI port 8080 conflicts with Keycloak - use 8085 in production

### Databases (Layer 10)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| PostgreSQL | 5432 | 5432 | TCP | Internal | Never expose externally |
| Redis | 6379 | 6379 | TCP | Internal | Enable AUTH, disable KEYS |
| Neo4j Browser | 7474 | 7474 | HTTP | Internal | Authentication required |
| Neo4j Bolt | 7687 | 7687 | TCP | Internal | Use TLS connections |
| MinIO | 9000 | 9000 | HTTP | Internal | Use access/secret keys |
| MinIO Console | 9001 | 9001 | HTTP | Internal | Admin only |
| ClickHouse | 8123 | 8123 | HTTP | Internal | Authentication required |
| ClickHouse | 9000 | 9009* | TCP | Internal | Native protocol |

*Note: ClickHouse native port remapped to avoid conflict with MinIO/Step-CA

### Observability (Layer 13)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| Grafana | 3000 | 3000 | HTTP | Internal | OAuth integration recommended |
| Prometheus | 9090 | 9090 | HTTP | Internal | Read-only dashboards |
| Loki | 3100 | 3100 | HTTP | Internal | Push gateway only |
| Tempo | 3200 | 3200 | HTTP | Internal | Trace ingestion |
| Alertmanager | 9093 | 9093 | HTTP | Internal | Webhook security |
| OTEL Collector | 4317 | 4317 | gRPC | Internal | OTLP receiver |
| OTEL Collector | 4318 | 4318 | HTTP | Internal | OTLP HTTP receiver |
| Netdata | 19999 | 19999 | HTTP | Internal | Real-time metrics |

### Workflow & Automation (Layer 8)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| Temporal | 7233 | 7233 | gRPC | Internal | mTLS recommended |
| Temporal UI | 8080 | 8088 | HTTP | Internal | OAuth integration |
| N8N | 5678 | 5678 | HTTP | Internal | Basic auth minimum |
| NATS | 4222 | 4222 | TCP | Internal | TLS + auth tokens |
| NATS Monitor | 8222 | 8222 | HTTP | Internal | Metrics only |

### API Gateway (DMZ)

| Service | Container Port | Host Port | Protocol | Exposure | Security Notes |
|---------|---------------|-----------|----------|----------|----------------|
| Kong | 8000 | 8000 | HTTP | External | Rate limiting enabled |
| Kong | 8443 | 8443 | HTTPS | External | TLS termination |
| Kong Admin | 8001 | 8001 | HTTP | **NEVER** | Bind to localhost only |
| Konga | 1337 | 1337 | HTTP | Internal | Kong management UI |
| Nginx | 80 | 80 | HTTP | External | Redirect to HTTPS |
| Nginx | 443 | 443 | HTTPS | External | TLS termination |

## Security Boundaries

### Trust Zones

```
┌─────────────────────────────────────────────────────────────────┐
│                      ZONE 0: PUBLIC (Untrusted)                  │
│  - Internet traffic                                              │
│  - Rate limited                                                  │
│  - WAF protection                                                │
└─────────────────────────────────────────────────────────────────┘
                                │
                        [API Gateway + TLS]
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ZONE 1: DMZ (Semi-trusted)                  │
│  - Kong API Gateway                                              │
│  - Nginx reverse proxy                                           │
│  - Authentication enforcement                                    │
└─────────────────────────────────────────────────────────────────┘
                                │
                     [Authentication + mTLS]
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ZONE 2: APPLICATION (Trusted)               │
│  - AI/ML services                                                │
│  - Workflow engines                                              │
│  - Application logic                                             │
└─────────────────────────────────────────────────────────────────┘
                                │
                        [mTLS + RBAC]
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────┐
│                      ZONE 3: DATA (Highly Trusted)               │
│  - Databases                                                     │
│  - Secret stores                                                 │
│  - Encryption at rest                                            │
└─────────────────────────────────────────────────────────────────┘
```

### Service-to-Service Authentication

| Source | Destination | Auth Method | Notes |
|--------|-------------|-------------|-------|
| Kong → Keycloak | OIDC | Token validation |
| Any → Vault | Token/AppRole | Dynamic secrets |
| Services → PostgreSQL | Username/Password | Via Vault dynamic creds |
| Services → Redis | Password | AUTH command |
| Services → NATS | Token/TLS | NKey authentication |
| Services → Temporal | mTLS | Certificate-based |

## Firewall Configuration

### UFW Rules (Host-based Firewall)

```bash
#!/bin/bash
# firewall-setup.sh - Configure host firewall for ARIA platform

# Reset UFW
ufw --force reset

# Default policies
ufw default deny incoming
ufw default allow outgoing

# SSH (restrict to specific IPs in production)
ufw allow from 10.0.0.0/8 to any port 22 proto tcp

# API Gateway (public-facing)
ufw allow 80/tcp    # HTTP (redirect to HTTPS)
ufw allow 443/tcp   # HTTPS

# Internal services (restrict to Docker networks)
# Identity services
ufw allow from 172.20.0.0/16 to any port 8080 proto tcp  # Keycloak
ufw allow from 172.20.0.0/16 to any port 8200 proto tcp  # Vault

# Block direct access to admin interfaces
ufw deny 8001/tcp   # Kong Admin
ufw deny 5432/tcp   # PostgreSQL (use Docker network)
ufw deny 6379/tcp   # Redis (use Docker network)

# Enable firewall
ufw --force enable
ufw status verbose
```

### Docker Network Policies (Kubernetes/Swarm)

For production deployments with Kubernetes, apply network policies:

```yaml
# network-policy-identity.yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: identity-network-policy
  namespace: aria
spec:
  podSelector:
    matchLabels:
      network: identity
  policyTypes:
    - Ingress
    - Egress
  ingress:
    - from:
        - namespaceSelector:
            matchLabels:
              name: aria
        - podSelector:
            matchLabels:
              role: api-gateway
      ports:
        - protocol: TCP
          port: 8080
        - protocol: TCP
          port: 8443
  egress:
    - to:
        - podSelector:
            matchLabels:
              network: data
      ports:
        - protocol: TCP
          port: 5432
```

## mTLS Service Mesh

For comprehensive mTLS setup, refer to:

- [mTLS Setup Guide](MTLS_SETUP.md) - Full configuration instructions
- [mTLS Implementation Checklist](MTLS-IMPLEMENTATION-CHECKLIST.md) - Step-by-step verification

### Quick mTLS Overview

```bash
# 1. Initialize Certificate Authority
./scripts/init-step-ca.sh

# 2. Generate service certificates
./scripts/generate-service-certs.sh

# 3. Verify mTLS setup
./scripts/verify-mtls-setup.sh
```

### Service Mesh Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Step-CA (Root CA)                         │
│              Issues certificates for all services                │
└─────────────────────────────────────────────────────────────────┘
                                │
            ┌───────────────────┼───────────────────┐
            ▼                   ▼                   ▼
     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
     │  Keycloak   │◄───►│    Vault    │◄───►│  Temporal   │
     │   (mTLS)    │     │   (mTLS)    │     │   (mTLS)    │
     └─────────────┘     └─────────────┘     └─────────────┘
            │                   │                   │
            └───────────────────┼───────────────────┘
                                │
                    [All traffic encrypted with mTLS]
```

## Container Image Security

### Pinned Image Versions (January 2026)

All production images are pinned to specific versions to prevent supply chain attacks:

| Category | Image | Version | Notes |
|----------|-------|---------|-------|
| **Workflow** | temporalio/auto-setup | 1.28.2 | CVE-2025-14987, CVE-2025-14986 fixed |
| **Workflow** | temporalio/ui | 2.34.0 | Security patches included |
| **Workflow** | temporalio/admin-tools | 1.28.2 | Matches server version |
| **LLMOps** | tensorzero/tensorzero | 2025.10.6 | CalVer pinning |
| **LLMOps** | tensorzero/tensorzero-ui | 2025.10.6 | CalVer pinning |
| **AI/ML** | mindsdb/mindsdb | 26.1.0 | Monthly release pinning |
| **Holochain** | holochain/holochain | 0.6.0 | Latest stable with K2 |
| **Holochain** | holochain/bootstrap-server | 0.6.0 | Matches conductor version |
| **Embedding** | semitechnologies/transformers-inference | 1.8.5 | MiniLM-L6-v2 model (full tag: `semitechnologies/transformers-inference:sentence-transformers-all-MiniLM-L6-v2-1.8.5`) |

#### Images Awaiting Version Pinning

The following images use `:latest` and are marked with TODO comments for version pinning when releases become available:

- `ruvector/ruvector:latest` - Custom vector database (no public releases yet)
- `ruvector/ruvector-ui:latest` - Vector DB UI
- `ruvector/qudag:latest` - Quantum-resistant DAG
- `ruvector/qudag-mcp:latest` - QuDAG MCP gateway
- `ghcr.io/xenova/transformers.js:latest` - Embedding service

#### Locally Built Images

These images are version-controlled via git, not Docker tags:

- `vcache:latest` - Built from `docker/vcache/Dockerfile`
- `aria/open-lovable:latest` - Built from `config/dockerfiles/Dockerfile.open-lovable`

### Image Scanning

Container images are automatically scanned for vulnerabilities:

- **CI/CD Pipeline**: Trivy scans all images on push/PR
- **Weekly Scans**: Scheduled scans catch new CVEs
- **SARIF Reports**: Results uploaded to GitHub Security tab

### Image Hardening Checklist

- [ ] Use minimal base images (alpine, distroless)
- [ ] Pin image versions with SHA256 digests
- [ ] Run as non-root user
- [ ] Remove unnecessary packages
- [ ] Set read-only root filesystem where possible
- [ ] Define resource limits (CPU, memory)
- [ ] Use multi-stage builds
- [ ] Scan for secrets in image layers

### Recommended Image Sources

| Source | Trust Level | Notes |
|--------|-------------|-------|
| Official Docker Hub | High | Verified publishers |
| Bitnami | High | Hardened images |
| Chainguard | Very High | Minimal, signed images |
| Custom builds | Variable | Scan before use |

## Security Scanning

### Automated Scanning

The following scans run automatically:

| Scanner | Target | Frequency | Workflow |
|---------|--------|-----------|----------|
| Trivy | Container images | On push, weekly | `container-security.yml` |
| Trivy | Filesystem | On push, weekly | `security.yml` |
| Trivy | IaC configs | On push | `security.yml` |
| CodeQL | Python code | On push | `security.yml` |
| Grype | SBOM | On push | `security.yml` |
| TruffleHog | Secrets | On push | `security.yml` |

### Manual Security Audit

```bash
# Run local Trivy scan on all images
./scripts/scan-containers.sh

# Check for exposed secrets
trufflehog filesystem .

# Audit Docker Compose files
trivy config docker/

# Check container runtime security
docker scout cves localai/localai:v2.24.2-aio-cpu
```

## Hardening Guidelines

### Container Runtime

```yaml
# Example hardened service configuration
services:
  secure-service:
    image: myservice:v1.0.0@sha256:abc123...
    user: "1000:1000"
    read_only: true
    security_opt:
      - no-new-privileges:true
    cap_drop:
      - ALL
    cap_add:
      - NET_BIND_SERVICE  # Only if needed
    tmpfs:
      - /tmp:noexec,nosuid,nodev
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 2G
        reservations:
          cpus: '0.5'
          memory: 512M
```

### Secret Management

1. **Never** store secrets in docker-compose files
2. Use environment variables from `.env` files (gitignored)
3. Use Docker secrets for swarm deployments
4. Use Vault for dynamic secrets in production
5. Rotate credentials regularly

### Logging and Audit

```yaml
# Enable audit logging
services:
  keycloak:
    environment:
      KC_LOG_LEVEL: INFO
      KC_LOG_CONSOLE_OUTPUT: json
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"
        labels: "service,environment"
```

## Quick Reference Commands

```bash
# List all exposed ports
docker ps --format "table {{.Names}}\t{{.Ports}}"

# Check network connectivity
docker network inspect identity-network

# Verify no containers running as root
docker ps -q | xargs docker inspect --format '{{.Name}} - User: {{.Config.User}}'

# Scan specific image
trivy image --severity HIGH,CRITICAL hashicorp/vault:1.18

# Check for outdated images
docker images --format "{{.Repository}}:{{.Tag}}" | while read img; do
  echo "Checking $img..."
  docker pull "$img" 2>/dev/null && echo "Updated: $img"
done
```

## Related Documentation

- [Getting Started](GETTING_STARTED.md)
- [mTLS Setup Guide](MTLS_SETUP.md)
- [mTLS Implementation Checklist](MTLS-IMPLEMENTATION-CHECKLIST.md)
- [Supply Chain Security](SUPPLY_CHAIN_SECURITY.md)
- [Troubleshooting](TROUBLESHOOTING.md)
