# Providers Module

**Layer:** L2 (Environments)
**Criticality:** MEDIUM
**Surface:** external
**Runtime:** container, cloud
**Status:** verified
**Owner:** ripple-env maintainers

---

## Purpose

This module documents provider integrations, authentication methods, and connectivity verification for all external and internal services.

---

## Provider Categories

### Overview

**Evidence:** `docs/CATALOG.json` providers section

| Category | Provider Count | Purpose |
|----------|----------------|---------|
| AI Inference | 4 | LLM and ML inference |
| Messaging | 2 | Event streaming, workflows |
| Identity | 2 | Authentication, secrets |
| Observability | 3 | Metrics, logs, traces |
| Storage | 3 | Object, relational, cache |
| Kubernetes | 3 | GitOps, deployments |

---

## AI Inference Providers

### LocalAI

**Evidence:** `config/localai/`, `docker/docker-compose.localai.yml`

| Attribute | Value |
|-----------|-------|
| Port | 8080 |
| Auth | None (local) |
| Config | `config/localai/models.yaml` |
| API | OpenAI-compatible |

**Smoke Check:**
```bash
curl http://localhost:8080/v1/models
```

### AGiXT

**Evidence:** `config/agixt/`, `docker/docker-compose.agixt.yml`

| Attribute | Value |
|-----------|-------|
| Port | 7437 |
| Auth | API key (optional) |
| Config | `config/agixt/` |
| API | REST |

**Smoke Check:**
```bash
curl http://localhost:7437/api/health
```

### OpenAI (External)

**Evidence:** Environment variables

| Attribute | Value |
|-----------|-------|
| Endpoint | `api.openai.com` |
| Auth | `OPENAI_API_KEY` |
| Config | N/A (external) |

**Smoke Check:**
```bash
curl -H "Authorization: Bearer $OPENAI_API_KEY" \
  https://api.openai.com/v1/models
```

### Anthropic (External)

**Evidence:** Environment variables

| Attribute | Value |
|-----------|-------|
| Endpoint | `api.anthropic.com` |
| Auth | `ANTHROPIC_API_KEY` |
| Config | N/A (external) |

**Smoke Check:**
```bash
curl -H "x-api-key: $ANTHROPIC_API_KEY" \
  https://api.anthropic.com/v1/models
```

---

## Messaging Providers

### NATS

**Evidence:** `docker/docker-compose.messaging.yml`

| Attribute | Value |
|-----------|-------|
| Client Port | 4222 |
| Cluster Port | 6222 |
| Monitor Port | 8222 |
| Auth | Credentials file |

**Smoke Check:**
```bash
# Using nats CLI
nats server ping

# Or curl monitoring
curl http://localhost:8222/healthz
```

### Temporal

**Evidence:** `docker/docker-compose.temporal.yml`, `config/temporal/`

| Attribute | Value |
|-----------|-------|
| gRPC Port | 7233 |
| Web UI Port | 8088 |
| Auth | None (local) |
| Config | `config/temporal/development.yaml` |

**Smoke Check:**
```bash
# Using temporal CLI
temporal operator cluster health

# Or curl health endpoint
curl http://localhost:7233/health
```

---

## Identity Providers

### Keycloak

**Evidence:** `docker/docker-compose.identity.yml`, `config/keycloak/`

| Attribute | Value |
|-----------|-------|
| HTTP Port | 8180 |
| Auth | Admin credentials |
| Config | `config/keycloak/realm.json` |
| Protocols | OIDC, SAML |

**Smoke Check:**
```bash
curl http://localhost:8180/health/ready
```

### HashiCorp Vault

**Evidence:** `docker/docker-compose.identity.yml`, `config/vault/`

| Attribute | Value |
|-----------|-------|
| Port | 8200 |
| Auth | Token, OIDC, AppRole |
| Config | `config/vault/vault.hcl` |

**Smoke Check:**
```bash
curl http://localhost:8200/v1/sys/health
```

---

## Observability Providers

### Prometheus

**Evidence:** `docker/docker-compose.observability.yml`, `config/prometheus/`

| Attribute | Value |
|-----------|-------|
| Port | 9090 |
| Auth | None (local) |
| Config | `config/prometheus/prometheus.yml` |

**Smoke Check:**
```bash
curl http://localhost:9090/-/healthy
```

### Grafana

**Evidence:** `docker/docker-compose.observability.yml`, `config/grafana/`

| Attribute | Value |
|-----------|-------|
| Port | 3000 |
| Auth | Credentials |
| Config | `config/grafana/provisioning/` |

**Smoke Check:**
```bash
curl http://localhost:3000/api/health
```

### Tempo (Jaeger-compatible)

**Evidence:** `docker/docker-compose.observability.yml`, `config/tempo/`

| Attribute | Value |
|-----------|-------|
| Query Port | 3200 |
| Jaeger gRPC | 14250 |
| Jaeger HTTP | 14268 |
| Config | `config/tempo/tempo.yaml` |

**Smoke Check:**
```bash
curl http://localhost:3200/ready
```

---

## Storage Providers

### PostgreSQL

**Evidence:** `docker/docker-compose.data.yml`

| Attribute | Value |
|-----------|-------|
| Port | 5432 |
| Auth | Credentials |
| Databases | temporal, keycloak, bytebase |

**Smoke Check:**
```bash
pg_isready -h localhost -p 5432
```

### Redis

**Evidence:** `docker/docker-compose.caching.yml`

| Attribute | Value |
|-----------|-------|
| Port | 6379 |
| Auth | Password (optional) |
| Purpose | Caching, sessions |

**Smoke Check:**
```bash
redis-cli ping
```

### MinIO (S3-compatible)

**Evidence:** `docker/docker-compose.state.yml`

| Attribute | Value |
|-----------|-------|
| API Port | 9000 |
| Console Port | 9001 |
| Auth | Access key / Secret key |

**Smoke Check:**
```bash
curl http://localhost:9000/minio/health/live
```

---

## Kubernetes Providers

### ArgoCD

**Evidence:** `manifests/argocd/`, `scripts/install-argocd.sh`

| Attribute | Value |
|-----------|-------|
| Port | 443 (via ingress) |
| Auth | Admin credentials |
| Config | `manifests/argocd/` |

**Smoke Check:**
```bash
argocd app list
```

### Argo Workflows

**Evidence:** `manifests/argo-workflows/`, `scripts/setup-argo-workflows.sh`

| Attribute | Value |
|-----------|-------|
| Port | 2746 |
| Auth | Kubeconfig |
| Config | `manifests/argo-workflows/` |

**Smoke Check:**
```bash
argo list
```

### Argo Rollouts

**Evidence:** `scripts/install-argo-rollouts.sh`

| Attribute | Value |
|-----------|-------|
| Port | N/A (controller) |
| Auth | Kubeconfig |

**Smoke Check:**
```bash
kubectl argo rollouts list rollouts
```

---

## Provider Authentication Matrix

| Provider | Dev Auth | Prod Auth | Secret Location |
|----------|----------|-----------|-----------------|
| LocalAI | None | None | N/A |
| AGiXT | Optional key | API key | `.env.local` |
| OpenAI | API key | API key | Vault |
| Anthropic | API key | API key | Vault |
| NATS | None | mTLS | Step CA |
| Temporal | None | mTLS | Step CA |
| Keycloak | Admin creds | OIDC | Vault |
| Vault | Token | OIDC | Bootstrap |
| Prometheus | None | Basic auth | Vault |
| Grafana | Creds | OIDC | Vault |
| PostgreSQL | Creds | mTLS | Vault |
| Redis | Optional | Password | Vault |
| MinIO | Keys | Keys | Vault |
| ArgoCD | Admin | OIDC | Vault |

---

## Connectivity Verification

### Full Stack Check

**Evidence:** `scripts/health-check.sh`

```bash
# Run all provider checks
./scripts/health-check.sh

# Output:
# [OK] LocalAI - http://localhost:8080/v1/models
# [OK] NATS - localhost:4222
# [OK] Temporal - localhost:7233
# [OK] Vault - http://localhost:8200/v1/sys/health
# [OK] Prometheus - http://localhost:9090/-/healthy
# ...
```

### Individual Provider Scripts

| Provider | Verification Script |
|----------|---------------------|
| Argo Workflows | `scripts/verify-argo-workflows.sh` |
| Edge Services | `scripts/verify-edge.sh` |
| JetStream | `scripts/verify-jetstream.sh` |
| MindsDB | `scripts/verify-mindsdb.sh` |
| mTLS | `scripts/verify-mtls-setup.sh` |
| Observability | `scripts/verify-observability.sh` |
| Open Lovable | `scripts/verify-open-lovable.sh` |
| QuDAG | `scripts/verify-qudag.sh` |
| RuVector | `scripts/verify-ruvector.sh` |
| State Storage | `scripts/verify-state-storage.sh` |

---

## Onboarding New Providers

**See:** `docs/cookbooks/PROVIDER_ONBOARDING.md`

1. Add Docker Compose service definition
2. Add configuration files
3. Create verification script
4. Document in CATALOG.json
5. Update PORTS.md
6. Add to health-check.sh

---

## Related Docs

- [PROVIDERS.md](../providers/PROVIDERS.md) - Detailed provider docs
- [PROVIDER_INTEGRATIONS.md](../providers/PROVIDER_INTEGRATIONS.md) - Integration details
- [AUTH_PROVIDERS.md](../providers/AUTH_PROVIDERS.md) - Auth specifics
- [PORTS.md](../api/PORTS.md) - Port mappings
- [API_REFERENCE.md](../api/API_REFERENCE.md) - API documentation
- [cookbooks/PROVIDER_ONBOARDING.md](../cookbooks/PROVIDER_ONBOARDING.md) - Onboarding guide

---

**Last Updated:** 2026-01-14
