# Environment Variable Registry

**Status:** Phase 6 Complete
**Last Updated:** 2026-01-14
**Evidence:** `.env.example`, `.env.*.example`, docker-compose files, scripts

---

## Overview

This document catalogs all environment variables used across the ripple-env project, organized by component and purpose.

---

## Core Environment Variables

### Nix/Flake

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `NIXPKGS_ALLOW_UNFREE` | `1` | No | Allow unfree packages (vault, etc.) | `flake.nix`, CI workflows |
| `NIX_CONFIG` | - | No | Override nix.conf settings | User |

**Evidence:** `.github/workflows/ci.yml` lines 15-17

### Pixi

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `PIXI_HOME` | `~/.pixi` | No | Pixi installation directory | User |
| `CONDA_PREFIX` | - | Auto | Active conda environment path | Pixi |

### ROS2

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `ROS_DISTRO` | `humble` | Yes | ROS2 distribution name | `pixi.toml` |
| `ROS_DOMAIN_ID` | `0` | No | ROS2 domain isolation | User |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | No | ROS2 middleware | User |
| `AMENT_PREFIX_PATH` | - | Auto | ROS2 package search path | Colcon |
| `ROS_LOCALHOST_ONLY` | `0` | No | Restrict to localhost | User |

**Evidence:** `pixi.toml` activation section

---

## AI/ML Environment Variables

### OpenAI/Anthropic

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `OPENAI_API_KEY` | - | Conditional | OpenAI API authentication | User |
| `ANTHROPIC_API_KEY` | - | Conditional | Anthropic API authentication | User |
| `OPENAI_API_BASE` | `https://api.openai.com/v1` | No | Override API endpoint (for LocalAI) | User |

**Evidence:** `.env.example`

### LocalAI

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `LOCALAI_API_KEY` | - | No | LocalAI authentication | `docker-compose.localai.yml` |
| `LOCALAI_MODELS_PATH` | `/models` | No | Model storage path | Docker compose |
| `LOCALAI_THREADS` | `4` | No | Inference threads | Docker compose |
| `LOCALAI_CONTEXT_SIZE` | `2048` | No | Context window size | Docker compose |

**Evidence:** `docker/docker-compose.localai.yml`

### AGiXT

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `AGIXT_API_KEY` | - | Yes (production) | AGiXT API authentication | `.env.agixt.example` |
| `AGIXT_URI` | `http://localhost:7437` | No | AGiXT server URL | Config |
| `AGIXT_HUB_URI` | `http://localhost:7437` | No | AGiXT hub URL | Config |
| `AGIXT_AUTO_UPDATE` | `true` | No | Auto-update agents | Config |
| `AGIXT_DATABASE_TYPE` | `sqlite` | No | Database backend | Config |
| `AGIXT_DATABASE_NAME` | `agixt` | No | Database name | Config |
| `AGIXT_POSTGRES_*` | - | Conditional | PostgreSQL connection | Config |

**Evidence:** `.env.agixt.example`, `docker/docker-compose.agixt.yml`

### TruLens

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `TRULENS_API_KEY` | - | No | TruLens cloud API key | User |
| `TRULENS_DATABASE_URL` | `sqlite:///trulens.db` | No | Evaluation database | User |

**Evidence:** `pixi.toml` llmops feature

---

## Identity/Security Environment Variables

### Keycloak

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `KEYCLOAK_ADMIN` | `admin` | Yes | Admin username | Docker compose |
| `KEYCLOAK_ADMIN_PASSWORD` | - | Yes | Admin password | Docker compose |
| `KC_DB` | `postgres` | No | Database type | Docker compose |
| `KC_DB_URL` | - | Conditional | JDBC URL | Docker compose |
| `KC_HOSTNAME` | `localhost` | No | Public hostname | Docker compose |

**Evidence:** `docker/docker-compose.identity.yml`

### Vault

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `VAULT_ADDR` | `http://127.0.0.1:8200` | Yes | Vault server URL | Scripts |
| `VAULT_TOKEN` | - | Yes | Vault authentication token | Scripts |
| `VAULT_DEV_ROOT_TOKEN_ID` | `dev-token` | Dev only | Dev mode token | Docker compose |
| `VAULT_DEV_LISTEN_ADDRESS` | `0.0.0.0:8200` | Dev only | Dev listen address | Docker compose |

**Evidence:** `docker/docker-compose.identity.yml`, `scripts/deploy.sh`

### Step-CA

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `STEP_CA_PASSWORD` | - | Yes | CA password | `scripts/init-step-ca.sh` |
| `STEP_CA_PROVISIONER_PASSWORD` | - | Yes | Provisioner password | Scripts |
| `STEP_CA_URL` | `https://localhost:9000` | No | CA URL | Config |

**Evidence:** `scripts/init-step-ca.sh`, `scripts/generate-service-certs.sh`

### mTLS

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `MTLS_ENABLED` | `false` | No | Enable mTLS | Config |
| `MTLS_CA_CERT` | - | Conditional | CA certificate path | Config |
| `MTLS_CLIENT_CERT` | - | Conditional | Client certificate path | Config |
| `MTLS_CLIENT_KEY` | - | Conditional | Client key path | Config |

**Evidence:** `docs/MTLS_SETUP.md`

---

## Messaging Environment Variables

### NATS

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `NATS_URL` | `nats://localhost:4222` | No | NATS server URL | Scripts |
| `NATS_USER` | - | Conditional | NATS username | Config |
| `NATS_PASSWORD` | - | Conditional | NATS password | Config |
| `NATS_CREDS_FILE` | - | Conditional | Credentials file path | Config |

**Evidence:** `docker/docker-compose.messaging.yml`

### Temporal

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `TEMPORAL_ADDRESS` | `localhost:7233` | No | Temporal frontend | Config |
| `TEMPORAL_NAMESPACE` | `default` | No | Temporal namespace | Config |
| `TEMPORAL_TLS_CA` | - | Conditional | TLS CA path | Config |

**Evidence:** `docker/docker-compose.temporal.yml`

---

## Observability Environment Variables

### Prometheus

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `PROMETHEUS_CONFIG` | `/etc/prometheus/prometheus.yml` | No | Config path | Docker compose |

### Grafana

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `GF_SECURITY_ADMIN_USER` | `admin` | No | Admin username | Docker compose |
| `GF_SECURITY_ADMIN_PASSWORD` | - | Yes | Admin password | Docker compose |
| `GF_SERVER_ROOT_URL` | `http://localhost:3000` | No | Public URL | Docker compose |
| `GF_INSTALL_PLUGINS` | - | No | Plugins to install | Docker compose |

**Evidence:** `docker/docker-compose.observability.yml`

### OpenTelemetry

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `OTEL_EXPORTER_OTLP_ENDPOINT` | `http://localhost:4317` | No | OTLP endpoint | Config |
| `OTEL_SERVICE_NAME` | - | Recommended | Service name | Config |
| `OTEL_TRACES_SAMPLER` | `parentbased_always_on` | No | Trace sampling | Config |

**Evidence:** `docker/docker-compose.observability.yml`

---

## Storage Environment Variables

### PostgreSQL

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `POSTGRES_USER` | `postgres` | No | Database user | Docker compose |
| `POSTGRES_PASSWORD` | - | Yes | Database password | Docker compose |
| `POSTGRES_DB` | `postgres` | No | Default database | Docker compose |
| `DATABASE_URL` | - | App | Connection string | Apps |

**Evidence:** `docker/docker-compose.data.yml`

### Redis

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `REDIS_URL` | `redis://localhost:6379` | No | Redis connection | Config |
| `REDIS_PASSWORD` | - | No | Redis password | Docker compose |

**Evidence:** `docker/docker-compose.state.yml`

### MinIO

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `MINIO_ROOT_USER` | - | Yes | Access key | Docker compose |
| `MINIO_ROOT_PASSWORD` | - | Yes | Secret key | Docker compose |
| `MINIO_ENDPOINT` | `localhost:9000` | No | API endpoint | Config |

**Evidence:** `docker/docker-compose.state.yml`

### Neo4j

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `NEO4J_AUTH` | `neo4j/password` | No | Auth (user/pass) | Docker compose |
| `NEO4J_URI` | `bolt://localhost:7687` | No | Connection URI | Config |

**Evidence:** `docker/docker-compose.data.yml`

---

## Development Environment Variables

### CI/CD

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `CI` | - | Auto | CI environment flag | GitHub Actions |
| `GITHUB_TOKEN` | - | CI | GitHub API access | GitHub Actions |
| `GITHUB_REPOSITORY` | - | Auto | Repository name | GitHub Actions |

### Scripts

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `SECURITY_AUDIT_REQUIRE_TRIVY` | `0` | No | Fail if Trivy missing | `security-audit.sh` |
| `SECURITY_AUDIT_REQUIRE_SECRETS` | `0` | No | Fail on secret leaks | `security-audit.sh` |
| `SECURITY_AUDIT_GITLEAKS_MODE` | `dir` | No | Gitleaks scan mode | `security-audit.sh` |

**Evidence:** `scripts/security-audit.sh`

---

## QuDAG Environment Variables

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `QUDAG_BACKEND` | `aer_simulator` | No | Quantum backend | `.env.qudag.example` |
| `QUDAG_SHOTS` | `1000` | No | Measurement shots | `.env.qudag.example` |
| `QUDAG_OPTIMIZATION_LEVEL` | `3` | No | Transpiler level | `.env.qudag.example` |

**Evidence:** `.env.qudag.example`, `docker-compose.qudag.yml`

---

## State Storage Environment Variables

| Variable | Default | Required | Purpose | Source |
|----------|---------|----------|---------|--------|
| `STATE_STORAGE_BACKEND` | `redis` | No | Storage backend | `.env.state.example` |
| `STATE_REDIS_URL` | `redis://localhost:6379` | No | Redis URL | `.env.state.example` |
| `STATE_S3_BUCKET` | - | Conditional | S3 bucket name | `.env.state.example` |
| `STATE_S3_ENDPOINT` | - | Conditional | S3 endpoint (MinIO) | `.env.state.example` |

**Evidence:** `.env.state.example`

---

## Environment File Hierarchy

```
.
├── .env                    # Local overrides (gitignored)
├── .env.example            # Template with all variables
├── .env.agixt.example      # AGiXT-specific template
├── .env.qudag.example      # QuDAG-specific template
├── .env.state.example      # State storage template
└── config/
    ├── keycloak/.env       # Keycloak secrets
    ├── vault/.env          # Vault secrets
    └── grafana/.env        # Grafana secrets
```

### Loading Order

1. System environment
2. `.env` file (if exists)
3. Docker Compose `env_file` directives
4. Docker Compose `environment` section (overrides)
5. Command-line `-e` flags (highest priority)

---

## Security Notes

1. **Never commit secrets** — Use `.env.example` as template, create `.env` locally
2. **Use Vault in production** — Inject secrets via Vault Agent or K8s secrets
3. **Rotate credentials** — Regularly rotate API keys and passwords
4. **Principle of least privilege** — Only grant necessary permissions

**Evidence:** `SECURITY.md`, `docs/SECRETS.md`
