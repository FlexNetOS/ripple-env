# Providers & Auth Wiring

**Status:** Phase 6 Complete
**Last Updated:** 2026-01-14
**Evidence:** docker-compose files, config directories, scripts

---

## Overview

This document details external service providers, their authentication mechanisms, and configuration locations.

---

## Provider Matrix

| Provider | Category | Port(s) | Auth Method | Config Location | Status |
|----------|----------|---------|-------------|-----------------|--------|
| OpenAI | AI Inference | - | API Key | `OPENAI_API_KEY` env | External |
| Anthropic | AI Inference | - | API Key | `ANTHROPIC_API_KEY` env | External |
| LocalAI | AI Inference | 8080 | API Key (optional) | `docker-compose.localai.yml` | Local |
| AGiXT | AI Agents | 7437 | API Key | `docker-compose.agixt.yml` | Local |
| Keycloak | Identity | 8080, 8443 | Admin credentials | `docker-compose.identity.yml` | Local |
| Vault | Secrets | 8200 | Token/AppRole | `docker-compose.identity.yml` | Local |
| Step-CA | PKI | 9000 | X.509 certs | `config/step-ca/` | Local |
| NATS | Messaging | 4222, 8222 | Creds file | `docker-compose.messaging.yml` | Local |
| Temporal | Workflows | 7233 | mTLS (optional) | `docker-compose.temporal.yml` | Local |
| Prometheus | Metrics | 9090 | None (local) | `config/prometheus/` | Local |
| Grafana | Dashboards | 3000 | Admin password | `docker-compose.observability.yml` | Local |
| PostgreSQL | Database | 5432 | Username/Password | `docker-compose.data.yml` | Local |
| Redis | Cache | 6379 | Password (optional) | `docker-compose.state.yml` | Local |
| MinIO | Object Store | 9000, 9001 | Access Key/Secret | `docker-compose.state.yml` | Local |
| Neo4j | Graph DB | 7474, 7687 | Username/Password | `docker-compose.data.yml` | Local |

---

## AI Inference Providers

### OpenAI

**Type:** External Cloud API
**Auth:** API Key via environment variable

```bash
# Set API key
export OPENAI_API_KEY="sk-..."

# Verify
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

**Configuration:**
- Environment variable: `OPENAI_API_KEY`
- Base URL override: `OPENAI_API_BASE` (for LocalAI compatibility)

**Evidence:** `.env.example`

### Anthropic

**Type:** External Cloud API
**Auth:** API Key via environment variable

```bash
# Set API key
export ANTHROPIC_API_KEY="sk-ant-..."

# Verify
curl https://api.anthropic.com/v1/messages \
  -H "x-api-key: $ANTHROPIC_API_KEY" \
  -H "anthropic-version: 2023-06-01"
```

**Configuration:**
- Environment variable: `ANTHROPIC_API_KEY`

**Evidence:** `.env.example`

### LocalAI

**Type:** Local Docker Service
**Auth:** Optional API Key

```yaml
# docker-compose.localai.yml
services:
  localai:
    environment:
      LOCALAI_API_KEY: "${LOCALAI_API_KEY:-}"  # Optional
```

**Endpoints:**
- API: `http://localhost:8080/v1/`
- Models: `http://localhost:8080/v1/models`
- Completions: `http://localhost:8080/v1/chat/completions`

**Setup:**
```bash
# Start LocalAI
docker compose -f docker/docker-compose.localai.yml up -d

# Download models
./scripts/download-models.sh

# Verify
curl http://localhost:8080/v1/models
```

**Evidence:** `docker/docker-compose.localai.yml`, `scripts/download-models.sh`

### AGiXT

**Type:** Local Docker Service
**Auth:** API Key (required in production)

```yaml
# docker-compose.agixt.yml
services:
  agixt:
    environment:
      AGIXT_API_KEY: "${AGIXT_API_KEY}"
```

**Endpoints:**
- API: `http://localhost:7437/api/`
- Agents: `http://localhost:7437/api/agent/`
- Chains: `http://localhost:7437/api/chain/`

**Setup:**
```bash
# Copy example env
cp .env.agixt.example .env.agixt

# Edit with your API key
vim .env.agixt

# Start AGiXT
docker compose -f docker/docker-compose.agixt.yml --env-file .env.agixt up -d

# Verify
curl http://localhost:7437/api/health
```

**Evidence:** `docker/docker-compose.agixt.yml`, `.env.agixt.example`

---

## Identity Providers

### Keycloak

**Type:** Local Docker Service
**Auth:** Admin username/password

```yaml
# docker-compose.identity.yml
services:
  keycloak:
    environment:
      KEYCLOAK_ADMIN: admin
      KEYCLOAK_ADMIN_PASSWORD: "${KEYCLOAK_ADMIN_PASSWORD}"
```

**Endpoints:**
- Admin Console: `http://localhost:8080/admin/`
- Realms: `http://localhost:8080/realms/{realm}/`
- Token: `http://localhost:8080/realms/{realm}/protocol/openid-connect/token`

**Client Configuration:**
1. Create realm in Keycloak Admin Console
2. Create client with appropriate grant types
3. Configure redirect URIs
4. Export client secret

**Setup:**
```bash
# Set admin password
export KEYCLOAK_ADMIN_PASSWORD="your-secure-password"

# Start Keycloak
docker compose -f docker/docker-compose.identity.yml up -d keycloak

# Wait for startup (can take 60+ seconds)
./scripts/health-check.sh keycloak

# Access admin console
open http://localhost:8080/admin/
```

**Evidence:** `docker/docker-compose.identity.yml`, `config/keycloak/`

### Vault

**Type:** Local Docker Service
**Auth:** Token or AppRole

```yaml
# docker-compose.identity.yml (dev mode)
services:
  vault:
    environment:
      VAULT_DEV_ROOT_TOKEN_ID: dev-token
      VAULT_DEV_LISTEN_ADDRESS: 0.0.0.0:8200
```

**Endpoints:**
- API: `http://localhost:8200/v1/`
- Secrets (KV): `http://localhost:8200/v1/secret/data/{path}`
- Auth: `http://localhost:8200/v1/auth/{method}/login`

**Setup:**
```bash
# Start Vault (dev mode)
docker compose -f docker/docker-compose.identity.yml up -d vault

# Set environment
export VAULT_ADDR="http://127.0.0.1:8200"
export VAULT_TOKEN="dev-token"

# Verify
vault status

# Store a secret
vault kv put secret/myapp api_key="secret123"

# Retrieve
vault kv get secret/myapp
```

**Production Mode:**
```bash
# Initialize Vault (once)
vault operator init

# Unseal (3 of 5 keys)
vault operator unseal <key1>
vault operator unseal <key2>
vault operator unseal <key3>

# Login
vault login <root-token>
```

**Evidence:** `docker/docker-compose.identity.yml`, `scripts/deploy.sh`

### Step-CA (PKI)

**Type:** Local Docker Service
**Auth:** X.509 Certificates

**Setup:**
```bash
# Initialize CA
./scripts/init-step-ca.sh

# Generate service certificates
./scripts/generate-service-certs.sh

# Certificate locations
ls secrets/certs/
# ca.crt, server.crt, server.key, client.crt, client.key
```

**Configuration:**
```bash
# Environment variables
export STEP_CA_PASSWORD="your-ca-password"
export STEP_CA_PROVISIONER_PASSWORD="your-provisioner-password"
```

**Evidence:** `scripts/init-step-ca.sh`, `scripts/generate-service-certs.sh`, `config/step-ca/`

---

## Messaging Providers

### NATS

**Type:** Local Docker Service
**Auth:** Credentials file or user/password

```yaml
# docker-compose.messaging.yml
services:
  nats:
    ports:
      - "4222:4222"   # Clients
      - "8222:8222"   # Monitoring
```

**Setup (No Auth - Development):**
```bash
# Start NATS
docker compose -f docker/docker-compose.messaging.yml up -d nats

# Verify
nats server info

# Publish/Subscribe
nats sub "test.subject" &
nats pub "test.subject" "Hello NATS"
```

**Setup (With Auth):**
```bash
# Create credentials
nats-server --gen-keys

# Configure in nats.conf
# accounts: { ... }

# Connect with credentials
nats sub "test" --creds /path/to/user.creds
```

**Evidence:** `docker/docker-compose.messaging.yml`, `scripts/init-jetstream.sh`

### Temporal

**Type:** Local Docker Service
**Auth:** mTLS (optional)

```yaml
# docker-compose.temporal.yml
services:
  temporal:
    ports:
      - "7233:7233"
```

**Setup:**
```bash
# Start Temporal
docker compose -f docker/docker-compose.temporal.yml up -d

# Verify
temporal workflow list

# Run a workflow
temporal workflow execute --type MyWorkflow --task-queue my-queue
```

**Evidence:** `docker/docker-compose.temporal.yml`

---

## Observability Providers

### Prometheus

**Type:** Local Docker Service
**Auth:** None (local access only)

**Configuration:**
```yaml
# config/prometheus/prometheus.yml
scrape_configs:
  - job_name: 'services'
    static_configs:
      - targets: ['host.docker.internal:8080']
```

**Setup:**
```bash
# Start Prometheus
docker compose -f docker/docker-compose.observability.yml up -d prometheus

# Access UI
open http://localhost:9090
```

**Evidence:** `docker/docker-compose.observability.yml`, `config/prometheus/`

### Grafana

**Type:** Local Docker Service
**Auth:** Admin username/password

```yaml
# docker-compose.observability.yml
services:
  grafana:
    environment:
      GF_SECURITY_ADMIN_USER: admin
      GF_SECURITY_ADMIN_PASSWORD: "${GRAFANA_ADMIN_PASSWORD}"
```

**Setup:**
```bash
# Set password
export GRAFANA_ADMIN_PASSWORD="your-secure-password"

# Start Grafana
docker compose -f docker/docker-compose.observability.yml up -d grafana

# Access UI (admin / your-password)
open http://localhost:3000
```

**Evidence:** `docker/docker-compose.observability.yml`, `config/grafana/`

---

## Storage Providers

### PostgreSQL

**Type:** Local Docker Service
**Auth:** Username/Password

```yaml
# docker-compose.data.yml
services:
  postgres:
    environment:
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: "${POSTGRES_PASSWORD}"
      POSTGRES_DB: ripple
```

**Connection String:**
```bash
export DATABASE_URL="postgresql://postgres:password@localhost:5432/ripple"
```

**Evidence:** `docker/docker-compose.data.yml`

### Redis

**Type:** Local Docker Service
**Auth:** Password (optional)

```yaml
# docker-compose.state.yml
services:
  redis:
    command: redis-server --requirepass "${REDIS_PASSWORD}"
```

**Connection String:**
```bash
export REDIS_URL="redis://:password@localhost:6379"
```

**Evidence:** `docker/docker-compose.state.yml`

### MinIO

**Type:** Local Docker Service
**Auth:** Access Key / Secret Key

```yaml
# docker-compose.state.yml
services:
  minio:
    environment:
      MINIO_ROOT_USER: "${MINIO_ROOT_USER}"
      MINIO_ROOT_PASSWORD: "${MINIO_ROOT_PASSWORD}"
```

**Setup:**
```bash
# Set credentials
export MINIO_ROOT_USER="minioadmin"
export MINIO_ROOT_PASSWORD="minioadmin"

# Start MinIO
docker compose -f docker/docker-compose.state.yml up -d minio

# Access Console
open http://localhost:9001
```

**Evidence:** `docker/docker-compose.state.yml`

---

## Auth Flow Diagrams

### OAuth2/OIDC Flow (Keycloak)

```mermaid
sequenceDiagram
    participant User
    participant App
    participant Kong
    participant Keycloak
    participant Service

    User->>App: Access Protected Resource
    App->>Kong: Request with no token
    Kong-->>App: 401 Unauthorized
    App->>Keycloak: Redirect to /authorize
    User->>Keycloak: Login
    Keycloak-->>App: Authorization Code
    App->>Keycloak: Exchange code for tokens
    Keycloak-->>App: Access + Refresh tokens
    App->>Kong: Request with Access Token
    Kong->>Keycloak: Validate Token (introspect)
    Keycloak-->>Kong: Token Valid + Claims
    Kong->>Service: Forward request with claims
    Service-->>Kong: Response
    Kong-->>App: Response
    App-->>User: Protected Resource
```

### mTLS Flow (Step-CA)

```mermaid
sequenceDiagram
    participant Client
    participant StepCA
    participant Service

    Client->>StepCA: Request certificate (CSR)
    StepCA-->>Client: Signed certificate

    Client->>Service: TLS Handshake with client cert
    Service->>Service: Validate client cert against CA
    Service-->>Client: TLS Established
    Client->>Service: API Request
    Service-->>Client: Response
```

### Vault Secret Injection

```mermaid
sequenceDiagram
    participant App
    participant VaultAgent
    participant Vault
    participant Secret

    App->>VaultAgent: Start (sidecar)
    VaultAgent->>Vault: Authenticate (AppRole/K8s)
    Vault-->>VaultAgent: Client Token
    VaultAgent->>Vault: Read secret/data/myapp
    Vault-->>VaultAgent: Secret data
    VaultAgent->>App: Inject as env/file
    App->>App: Use secret
```

---

## Provider Dependencies

```mermaid
graph TD
    subgraph "Auth Layer"
        KC[Keycloak]
        VAULT[Vault]
        STEP[Step-CA]
    end

    subgraph "Service Layer"
        KONG[Kong]
        NATS[NATS]
        TEMPORAL[Temporal]
        AGIXT[AGiXT]
    end

    subgraph "Data Layer"
        PG[PostgreSQL]
        REDIS[Redis]
        MINIO[MinIO]
    end

    KC --> PG
    VAULT --> PG
    KONG --> KC
    KONG --> STEP
    AGIXT --> PG
    AGIXT --> KC
    TEMPORAL --> PG
    NATS --> STEP
```

---

## Troubleshooting

### Common Issues

| Issue | Provider | Solution |
|-------|----------|----------|
| 401 Unauthorized | Keycloak | Check token expiry, refresh token |
| Certificate expired | Step-CA | Run `./scripts/rotate-certs.sh` |
| Connection refused | All | Check if container is running |
| Database locked | SQLite | Use PostgreSQL for production |
| Token invalid | Vault | Re-authenticate, check policy |

### Debug Commands

```bash
# Check provider health
./scripts/health-check.sh

# View logs
docker compose logs keycloak
docker compose logs vault

# Test connectivity
curl -v http://localhost:8080/health

# Inspect certificates
openssl x509 -in secrets/certs/server.crt -text -noout
```
