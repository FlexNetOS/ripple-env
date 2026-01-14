# Phase 6: Authentication Providers and Authorization Wiring

**Status:** ✅ Complete
**Phase:** 6/8 - Document providers and auth wiring
**Created:** 2026-01-14
**Diagrams:** 1 (auth-architecture.mmd)

---

## Executive Summary

This phase documents the complete authentication, authorization, and secrets management infrastructure for the ripple-env platform. The architecture implements a **zero-trust security model** with multiple layers of defense:

- **Identity Layer (L5):** 4 specialized providers (Step-CA, Keycloak, Vault, Vaultwarden)
- **Policy Layer:** OPA-based authorization with Rego policies
- **Gateway Layer:** Kong API Gateway with plugin-based security
- **Service Mesh:** mTLS-encrypted service-to-service communication

### Key Architecture Decisions

| Decision | Rationale | Evidence |
|----------|-----------|----------|
| **Multi-provider identity stack** | Separation of concerns (PKI, OIDC, secrets, passwords) | docker/docker-compose.identity.yml:1-233 |
| **Zero-trust mTLS** | All services authenticated via client certificates | docs/MTLS_SETUP.md:1-150 |
| **Policy-based authorization** | Centralized, versioned, testable access control | config/opa/policies/*.rego |
| **Kong Gateway enforcement** | Single policy enforcement point | config/kong/*.yaml |
| **OIDC for humans/agents** | Industry-standard OAuth2/OIDC flows | config/vault/auth-oidc.hcl:1-121 |

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Identity Layer (L5)](#identity-layer-l5)
   - [Step-CA: PKI and Certificate Authority](#step-ca-pki-and-certificate-authority)
   - [Keycloak: OIDC Identity Provider](#keycloak-oidc-identity-provider)
   - [HashiCorp Vault: Secrets Management](#hashicorp-vault-secrets-management)
   - [Vaultwarden: Password Vault](#vaultwarden-password-vault)
3. [Policy Layer](#policy-layer)
   - [Open Policy Agent (OPA)](#open-policy-agent-opa)
   - [RBAC Authorization Policies](#rbac-authorization-policies)
   - [Service Mesh Policies](#service-mesh-policies)
4. [Gateway Layer](#gateway-layer)
   - [Kong Gateway Configuration](#kong-gateway-configuration)
   - [OAuth2 Plugin](#oauth2-plugin)
   - [Rate Limiting](#rate-limiting)
   - [mTLS Verification](#mtls-verification)
5. [Authentication Flows](#authentication-flows)
   - [Human User Authentication (OIDC)](#human-user-authentication-oidc)
   - [AI Agent Authentication (Client Credentials)](#ai-agent-authentication-client-credentials)
   - [Service-to-Service Authentication (mTLS)](#service-to-service-authentication-mtls)
6. [Authorization Flow](#authorization-flow)
7. [Certificate Lifecycle Management](#certificate-lifecycle-management)
8. [Secrets Management Workflows](#secrets-management-workflows)
9. [Integration Examples](#integration-examples)
10. [Operational Procedures](#operational-procedures)
11. [Security Best Practices](#security-best-practices)
12. [Troubleshooting Guide](#troubleshooting-guide)

---

## Architecture Overview

### Visual Architecture

See **docs/graphs/auth-architecture.mmd** for the complete Mermaid diagram showing:

- 3 external actor types (humans, agents, services)
- 4 identity providers with 5 backing stores
- 1 policy engine with 2 policy bundles
- 1 API gateway with 3 security plugins
- 4 service mesh components with mTLS

### Security Layers

```
┌─────────────────────────────────────────────────────┐
│  External Actors (Users, Agents, Services)          │
└─────────────────┬───────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────┐
│  🌐 Gateway Layer (Kong)                            │
│  • OAuth2 validation                                │
│  • mTLS verification                                │
│  • Rate limiting                                    │
└─────────────────┬───────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────┐
│  📋 Policy Layer (OPA)                              │
│  • RBAC authorization                               │
│  • Service mesh policies                            │
└─────────────────┬───────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────┐
│  🔐 Identity Layer (L5)                             │
│  • Step-CA (PKI)                                    │
│  • Keycloak (OIDC)                                  │
│  • Vault (Secrets)                                  │
│  • Vaultwarden (Passwords)                          │
└─────────────────┬───────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────┐
│  🔧 Service Mesh (Temporal, n8n, AGiXT, LocalAI)    │
└─────────────────────────────────────────────────────┘
```

### Trust Model

**Zero-Trust Principles:**
1. **Never trust, always verify:** Every request authenticated and authorized
2. **Least privilege:** Minimal permissions for each actor
3. **Assume breach:** Defense in depth with multiple layers
4. **Verify explicitly:** All identities verified via certificates or tokens

**Implementation:**
- No implicit trust between services
- mTLS required for all service-to-service communication
- Policy enforcement at gateway (single choke point)
- Certificate-based service identity
- Token-based user/agent identity

---

## Identity Layer (L5)

### Step-CA: PKI and Certificate Authority

**Purpose:** Provides Public Key Infrastructure (PKI) and issues X.509 certificates for mTLS.

**Configuration:** `docker/docker-compose.identity.yml:6-26`

```yaml
step-ca:
  image: smallstep/step-ca:0.27.5
  container_name: step-ca
  hostname: step-ca
  networks:
    - agentic-network
  ports:
    - "9000:9000"
  environment:
    DOCKER_STEPCA_INIT_NAME: "ARIA Development CA"
    DOCKER_STEPCA_INIT_DNS_NAMES: "step-ca,localhost"
    DOCKER_STEPCA_INIT_PROVISIONER_NAME: "admin"
    DOCKER_STEPCA_INIT_ADMIN_SUBJECT: "admin"
  volumes:
    - step-ca-data:/home/step
  healthcheck:
    test: ["CMD", "step", "ca", "health"]
    interval: 10s
    timeout: 5s
    retries: 5
```

**Key Features:**
- **Root CA:** 10-year validity, offline storage
- **Intermediate CA:** 5-year validity, online signing
- **Automated renewal:** Certificates renewed before expiration
- **ACME protocol:** Automated certificate management

**Certificate Hierarchy:**
```
Root CA (10 years)
  └── Intermediate CA (5 years)
        ├── Service Certs (90 days)
        ├── Gateway Certs (90 days)
        └── Client Certs (90 days)
```

**Scripts:**
- `scripts/init-step-ca.sh` - Initialize CA with root and intermediate certificates (lines 1-156)
- `scripts/generate-service-certs.sh` - Generate service-specific certificates (lines 1-183)
- `scripts/rotate-certs.sh` - Rotate expiring certificates (lines 1-247)
- `scripts/setup-cert-rotation-cron.sh` - Automate weekly rotation (lines 1-87)
- `scripts/verify-mtls-setup.sh` - Verify CA chain and certificate validity (lines 1-148)

**Documentation:** `docs/MTLS_SETUP.md:1-150`

**Ports:**
- **9000:** CA API (HTTPS)

**Volumes:**
- `step-ca-data:/home/step` - CA configuration and keys

---

### Keycloak: OIDC Identity Provider

**Purpose:** OpenID Connect (OIDC) and OAuth2 identity provider for human users and AI agents.

**Configuration:** `docker/docker-compose.identity.yml:28-78`

```yaml
keycloak:
  image: quay.io/keycloak/keycloak:26.0
  container_name: keycloak
  hostname: keycloak
  networks:
    - agentic-network
  ports:
    - "8080:8080"
    - "8443:8443"
  environment:
    KC_DB: postgres
    KC_DB_URL: jdbc:postgresql://keycloak-postgres:5432/keycloak
    KC_DB_USERNAME: keycloak
    KC_DB_PASSWORD: "${KEYCLOAK_DB_PASSWORD:-changeme}"
    KC_HOSTNAME: localhost
    KC_HOSTNAME_STRICT: "false"
    KC_HTTP_ENABLED: "true"
    KC_HEALTH_ENABLED: "true"
    KC_METRICS_ENABLED: "true"
    KEYCLOAK_ADMIN: admin
    KEYCLOAK_ADMIN_PASSWORD: "${KEYCLOAK_ADMIN_PASSWORD:-admin}"
  command:
    - start-dev
```

**Key Features:**
- **Realm:** `agentic` realm for all services
- **OIDC flows:** Authorization code, implicit, client credentials
- **User federation:** LDAP, Active Directory, custom providers
- **Social login:** Google, GitHub, etc. (configurable)
- **2FA/MFA:** TOTP, WebAuthn support

**Authentication Flows:**
1. **Human Users:** Authorization code flow with PKCE
2. **AI Agents:** Client credentials flow (machine-to-machine)
3. **Services:** Service account tokens

**Client Configuration:**
- **vault:** OIDC client for Vault integration
- **agixt:** Client credentials for AI agents
- **n8n:** OAuth2 client for workflow automation
- **grafana:** OIDC SSO for observability dashboards

**Token Format (JWT):**
```json
{
  "iss": "http://keycloak:8080/realms/agentic",
  "sub": "user-uuid",
  "aud": "vault",
  "exp": 1736899200,
  "iat": 1736895600,
  "azp": "vault",
  "scope": "openid profile email groups",
  "email": "user@example.com",
  "groups": ["/admin", "/developers"],
  "realm_access": {
    "roles": ["admin", "user"]
  }
}
```

**Ports:**
- **8080:** HTTP (development)
- **8443:** HTTPS (production)

**Volumes:**
- `keycloak-data:/opt/keycloak/data` - Realm configuration and themes

**Database:** PostgreSQL (keycloak-postgres service)

---

### HashiCorp Vault: Secrets Management

**Purpose:** Centralized secrets storage, encryption, and dynamic credentials.

**Configuration:** `docker/vault.yml:1-66`

```yaml
vault:
  image: hashicorp/vault:1.18
  container_name: vault
  hostname: vault
  networks:
    - agentic-network
  ports:
    - "8200:8200"
  environment:
    VAULT_DEV_ROOT_TOKEN_ID: "${VAULT_ROOT_TOKEN:-root}"
    VAULT_DEV_LISTEN_ADDRESS: "0.0.0.0:8200"
    VAULT_ADDR: "http://0.0.0.0:8200"
  cap_add:
    - IPC_LOCK
  volumes:
    - vault-data:/vault/file
    - ./config/vault:/vault/config:ro
  command: server -dev
```

**Key Features:**
- **KV v2 secrets engine:** Versioned key-value storage
- **Dynamic secrets:** Database credentials, API keys
- **Encryption as a service:** Transit engine for data encryption
- **PKI integration:** Step-CA backend for certificate management
- **OIDC authentication:** Keycloak integration

**Secrets Hierarchy:**
```
secret/
├── data/llm/          # LLM provider API keys
│   ├── openai
│   ├── anthropic
│   └── localai
├── data/agents/       # Agent-specific secrets
│   ├── agent-001
│   └── agent-002
├── data/databases/    # Database credentials
│   ├── postgres
│   └── redis
└── data/services/     # Service API keys
    ├── temporal
    └── n8n
```

**Access Policies:**

**Agent Policy** (`config/vault/policy-agent.hcl:1-28`):
```hcl
# API keys for LLM providers
path "secret/data/llm/*" {
  capabilities = ["read"]
}

# Agent-specific secrets (full CRUD)
path "secret/data/agents/{{identity.entity.metadata.agent_id}}/*" {
  capabilities = ["read", "create", "update", "delete"]
}

# Encryption operations
path "transit/encrypt/agent-data" {
  capabilities = ["update"]
}

path "transit/decrypt/agent-data" {
  capabilities = ["update"]
}
```

**OIDC Authentication** (`config/vault/auth-oidc.hcl:1-121`):
```hcl
path "auth/oidc/config" {
  capabilities = ["create", "update"]
}

# Keycloak OIDC discovery
oidc_discovery_url = "http://keycloak:8080/realms/agentic"
oidc_client_id = "vault"
oidc_client_secret = "${VAULT_OIDC_CLIENT_SECRET}"
default_role = "reader"

# Map Keycloak groups to Vault policies
oidc_scopes = ["openid", "profile", "email", "groups"]

bound_audiences = ["vault"]
allowed_redirect_uris = [
  "http://localhost:8200/ui/vault/auth/oidc/oidc/callback",
  "http://vault:8200/ui/vault/auth/oidc/oidc/callback"
]

# Role mapping
user_claim = "sub"
groups_claim = "groups"
```

**Scripts:**
- `scripts/init-vault.sh` - Initialize Vault with unsealing (if needed)
- `scripts/populate-vault-secrets.sh` - Populate initial secrets (custom)

**Ports:**
- **8200:** Vault API and UI (HTTP)

**Volumes:**
- `vault-data:/vault/file` - Secret storage (dev mode uses in-memory)
- `./config/vault:/vault/config:ro` - Policy and auth configurations

---

### Vaultwarden: Password Vault

**Purpose:** Bitwarden-compatible password manager for team credentials.

**Configuration:** `docker/docker-compose.identity.yml:80-96`

```yaml
vaultwarden:
  image: vaultwarden/server:1.32.5
  container_name: vaultwarden
  hostname: vaultwarden
  networks:
    - agentic-network
  ports:
    - "8081:80"
  environment:
    DOMAIN: "http://localhost:8081"
    SIGNUPS_ALLOWED: "false"
    ADMIN_TOKEN: "${VAULTWARDEN_ADMIN_TOKEN:-changeme}"
  volumes:
    - vaultwarden-data:/data
```

**Key Features:**
- **Bitwarden-compatible:** Use official Bitwarden clients
- **Team passwords:** Shared credentials for development
- **Collections:** Organize by project or service
- **2FA support:** TOTP, YubiKey, Duo

**Use Cases:**
- Development API keys (non-production)
- Shared service passwords
- Personal developer credentials
- Emergency access credentials

**Ports:**
- **8081:** Web UI and API (HTTP)

**Volumes:**
- `vaultwarden-data:/data` - SQLite database and attachments

---

## Policy Layer

### Open Policy Agent (OPA)

**Purpose:** Policy-based authorization engine for RBAC and service mesh policies.

**Configuration:** `docker/docker-compose.automation.yml:80-105`

```yaml
opa:
  image: openpolicyagent/opa:0.71.0-rootless
  container_name: opa
  hostname: opa
  networks:
    - agentic-network
  ports:
    - "8181:8181"
  volumes:
    - ./config/opa/policies:/policies:ro
  command:
    - "run"
    - "--server"
    - "--addr=0.0.0.0:8181"
    - "/policies"
  healthcheck:
    test: ["CMD", "wget", "-q", "--spider", "http://localhost:8181/health"]
    interval: 10s
    timeout: 5s
    retries: 5
```

**Key Features:**
- **Rego policy language:** Declarative authorization rules
- **Policy bundles:** Versioned, testable policies
- **REST API:** Policy evaluation via HTTP
- **Decision logging:** Audit trail of authorization decisions

**Policy Structure:**
```
config/opa/policies/
├── authz.rego               # RBAC authorization
├── service_mesh.rego        # Service-to-service policies
└── data.json                # Policy data (roles, permissions)
```

**Ports:**
- **8181:** OPA API and health endpoint

**Volumes:**
- `./config/opa/policies:/policies:ro` - Read-only policy bundles

---

### RBAC Authorization Policies

**Purpose:** Role-based access control for users and agents.

**Configuration:** `config/opa/policies/authz.rego:1-67`

```rego
package ros2.authz

import future.keywords.if
import future.keywords.in

# Default deny
default allow := false

# ============================================================================
# Admin Role - Full Access
# ============================================================================

allow if {
    input.user.role == "admin"
}

# ============================================================================
# Developer Role - Read/Write Own Resources
# ============================================================================

allow if {
    input.user.role == "developer"
    input.action in ["read", "write"]
    input.resource.owner == input.user.id
}

allow if {
    input.user.role == "developer"
    input.action == "read"
    input.resource.visibility == "public"
}

# ============================================================================
# Agent Role - Agent-Specific Access
# ============================================================================

allow if {
    input.user.type == "agent"
    input.resource.owner == input.user.id
}

allow if {
    input.user.type == "agent"
    input.action == "read"
    input.resource.agent_readable == true
}

# ============================================================================
# Viewer Role - Read-Only Public Resources
# ============================================================================

allow if {
    input.user.role == "viewer"
    input.action == "read"
    input.resource.visibility == "public"
}

# ============================================================================
# Deny Destructive Actions for Non-Admins
# ============================================================================

deny if {
    input.action == "delete"
    input.user.role != "admin"
}
```

**Input Format:**
```json
{
  "user": {
    "id": "user-uuid",
    "role": "developer",
    "type": "human",
    "groups": ["/developers"]
  },
  "action": "read",
  "resource": {
    "name": "workflow-123",
    "owner": "user-uuid",
    "visibility": "public",
    "agent_readable": true
  }
}
```

**Output Format:**
```json
{
  "result": {
    "allow": true,
    "deny": false
  }
}
```

**Testing:**
```bash
# Test policy locally
opa test config/opa/policies/

# Evaluate policy via API
curl -X POST http://localhost:8181/v1/data/ros2/authz \
  -H "Content-Type: application/json" \
  -d '{
    "input": {
      "user": {"role": "admin"},
      "action": "delete",
      "resource": {"name": "test"}
    }
  }'
```

---

### Service Mesh Policies

**Purpose:** Control service-to-service communication in the mesh.

**Configuration:** `config/opa/policies/service_mesh.rego:1-21`

```rego
package service_mesh

import future.keywords.if

# Default deny all connections
default allow_connection = false

# ============================================================================
# Allowed Service-to-Service Connections
# ============================================================================

# Temporal can connect to NATS
allow_connection if {
    input.source.service == "temporal"
    input.destination.service == "nats"
}

# n8n can connect to AGiXT
allow_connection if {
    input.source.service == "n8n"
    input.destination.service == "agixt"
}

# AGiXT can connect to LocalAI
allow_connection if {
    input.source.service == "agixt"
    input.destination.service == "localai"
}

# All services can connect to Vault
allow_connection if {
    input.destination.service == "vault"
}

# All services can connect to OPA
allow_connection if {
    input.destination.service == "opa"
}

# Observability services can scrape metrics
allow_connection if {
    input.source.service == "prometheus"
    input.destination.port == 9090
}
```

**Input Format:**
```json
{
  "source": {
    "service": "agixt",
    "namespace": "default",
    "cert_cn": "agixt.default.svc"
  },
  "destination": {
    "service": "localai",
    "namespace": "default",
    "port": 8080
  }
}
```

**Integration:**
Kong Gateway calls OPA before forwarding requests:
```bash
# Kong OPA plugin configuration
curl -X POST http://localhost:8001/services/temporal/plugins \
  -d name=opa \
  -d config.policy_uri=http://opa:8181/v1/data/service_mesh/allow_connection
```

---

## Gateway Layer

### Kong Gateway Configuration

**Purpose:** API Gateway with policy enforcement, rate limiting, and mTLS verification.

**Configuration:** `docker/docker-compose.edge.yml:6-45`

```yaml
kong:
  image: kong:3.9-alpine
  container_name: kong
  hostname: kong
  networks:
    - agentic-network
  ports:
    - "8000:8000"   # HTTP proxy
    - "8443:8443"   # HTTPS proxy
    - "8001:8001"   # Admin API
  environment:
    KONG_DATABASE: "off"
    KONG_DECLARATIVE_CONFIG: /kong/kong.yml
    KONG_PROXY_ACCESS_LOG: /dev/stdout
    KONG_ADMIN_ACCESS_LOG: /dev/stdout
    KONG_PROXY_ERROR_LOG: /dev/stderr
    KONG_ADMIN_ERROR_LOG: /dev/stderr
    KONG_ADMIN_LISTEN: 0.0.0.0:8001
    KONG_PROXY_LISTEN: 0.0.0.0:8000, 0.0.0.0:8443 ssl
  volumes:
    - ./config/kong:/kong:ro
    - ./certs:/certs:ro
```

**Plugins:**
1. **OAuth2:** Client credentials and authorization code flows
2. **Rate Limiting:** 100 requests/minute per consumer
3. **mTLS Verification:** Validate client certificates against Step-CA
4. **CORS:** Cross-origin resource sharing
5. **Request Transformer:** Add/remove headers
6. **Response Transformer:** Modify responses

**Ports:**
- **8000:** HTTP proxy (insecure, dev only)
- **8443:** HTTPS proxy (production)
- **8001:** Admin API (internal only)

**Volumes:**
- `./config/kong:/kong:ro` - Declarative configuration
- `./certs:/certs:ro` - TLS certificates for HTTPS

---

### OAuth2 Plugin

**Configuration:** `config/kong/oauth2-plugin.yaml:1-22`

```yaml
plugins:
  - name: oauth2
    config:
      # Token endpoint
      enable_authorization_code: true
      enable_client_credentials: true
      enable_implicit_grant: false
      enable_password_grant: false

      # Scopes
      scopes:
        - read
        - write
        - admin
      mandatory_scope: true

      # Token settings
      token_expiration: 3600        # 1 hour
      refresh_token_ttl: 1209600    # 14 days

      # Endpoints
      provision_key: "provision-key-changeme"
      hide_credentials: true
```

**Grant Types:**

**Client Credentials (AI Agents):**
```bash
curl -X POST https://kong:8443/oauth2/token \
  -d grant_type=client_credentials \
  -d client_id=agixt-client \
  -d client_secret=secret \
  -d scope=read,write
```

**Authorization Code (Human Users):**
```bash
# Step 1: Authorization endpoint
https://kong:8443/oauth2/authorize?
  response_type=code&
  client_id=webapp&
  redirect_uri=http://localhost:3000/callback&
  scope=read,write

# Step 2: Token exchange
curl -X POST https://kong:8443/oauth2/token \
  -d grant_type=authorization_code \
  -d client_id=webapp \
  -d client_secret=secret \
  -d code=AUTH_CODE
```

---

### Rate Limiting

**Configuration:**
```yaml
plugins:
  - name: rate-limiting
    config:
      minute: 100
      hour: 5000
      policy: local
      fault_tolerant: true
      hide_client_headers: false
```

**Per-Consumer Limits:**
```bash
# Create consumer
curl -X POST http://localhost:8001/consumers \
  -d username=agixt-agent-001

# Set custom rate limit
curl -X POST http://localhost:8001/consumers/agixt-agent-001/plugins \
  -d name=rate-limiting \
  -d config.minute=200 \
  -d config.hour=10000
```

**Response Headers:**
```
X-RateLimit-Limit-Minute: 100
X-RateLimit-Remaining-Minute: 99
X-RateLimit-Reset-Minute: 1736896000
```

---

### mTLS Verification

**Configuration:**
```yaml
plugins:
  - name: mtls-auth
    config:
      ca_certificates:
        - /certs/ca.crt
      revocation_check_mode: IGNORE_CA_ERROR
      skip_consumer_lookup: false
      consumer_by: CN
      authenticated_group_by: CN
```

**Certificate Validation:**
1. **Client presents certificate** during TLS handshake
2. **Kong verifies certificate** against Step-CA root
3. **Extract CN (Common Name)** from certificate
4. **Map CN to Kong consumer** for rate limiting and policies
5. **Forward to OPA** for authorization check

**Example Certificate:**
```
Subject: CN=agixt.default.svc, O=ARIA Development CA
Issuer: CN=ARIA Development Intermediate CA
Validity: 2026-01-14 to 2026-04-14 (90 days)
```

---

## Authentication Flows

### Human User Authentication (OIDC)

**Flow:** Authorization Code with PKCE

```
┌──────────┐                                      ┌───────────┐
│  User    │                                      │ Keycloak  │
│ Browser  │                                      │   OIDC    │
└────┬─────┘                                      └─────┬─────┘
     │                                                  │
     │  1. GET /authorize?                             │
     │     response_type=code&                         │
     │     client_id=webapp&                           │
     │     redirect_uri=http://app/callback&           │
     │     code_challenge=SHA256(verifier)             │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │  2. User login form                             │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │  3. POST /authenticate                          │
     │     username=user&password=pass                 │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │  4. 302 Redirect to callback with code          │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │  5. POST /token                                 │
     │     grant_type=authorization_code&              │
     │     code=AUTH_CODE&                             │
     │     code_verifier=VERIFIER                      │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │  6. JWT Access Token + Refresh Token            │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │                                                  │
┌────┴─────┐                                      ┌─────┴─────┐
│  User    │──────── API Calls with JWT ────────>│   Kong    │
│ Browser  │                                      │  Gateway  │
└──────────┘                                      └───────────┘
                                                        │
                                                        │ Validate JWT
                                                        │
                                                        ▼
                                                  ┌───────────┐
                                                  │ Keycloak  │
                                                  │   JWKS    │
                                                  └───────────┘
```

**Evidence:** `config/vault/auth-oidc.hcl:1-121`

**Security Features:**
- **PKCE (Proof Key for Code Exchange):** Prevents authorization code interception
- **State parameter:** CSRF protection
- **Short-lived access tokens:** 1 hour expiration
- **Refresh tokens:** 14-day expiration for token renewal

---

### AI Agent Authentication (Client Credentials)

**Flow:** OAuth2 Client Credentials

```
┌──────────┐                                      ┌───────────┐
│ AI Agent │                                      │ Keycloak  │
│  (AGiXT) │                                      │   OIDC    │
└────┬─────┘                                      └─────┬─────┘
     │                                                  │
     │  1. POST /token                                 │
     │     grant_type=client_credentials&              │
     │     client_id=agixt-client&                     │
     │     client_secret=SECRET&                       │
     │     scope=read,write                            │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │  2. Service Token (JWT)                         │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │                                                  │
┌────┴─────┐                                      ┌─────┴─────┐
│ AI Agent │──────── API Calls with Token ──────>│   Kong    │
│  (AGiXT) │                                      │  Gateway  │
└──────────┘                                      └───────────┘
                                                        │
                                                        │ Validate Token
                                                        │
                                                        ▼
                                                  ┌───────────┐
                                                  │ Keycloak  │
                                                  │   JWKS    │
                                                  └───────────┘
```

**Token Example:**
```json
{
  "iss": "http://keycloak:8080/realms/agentic",
  "sub": "service-account-agixt-client",
  "aud": "kong",
  "exp": 1736899200,
  "iat": 1736895600,
  "azp": "agixt-client",
  "scope": "read write",
  "client_id": "agixt-client"
}
```

**Evidence:** `config/kong/oauth2-plugin.yaml:1-22`

---

### Service-to-Service Authentication (mTLS)

**Flow:** Mutual TLS Handshake

```
┌──────────┐                                      ┌───────────┐
│ Service  │                                      │   Kong    │
│(Temporal)│                                      │  Gateway  │
└────┬─────┘                                      └─────┬─────┘
     │                                                  │
     │  1. TLS Client Hello                            │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │  2. TLS Server Hello + Certificate Request      │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │  3. Client Certificate                          │
     │     (CN=temporal.default.svc)                   │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │                                                  │ Verify cert
     │                                                  │ against CA
     │                                                  │
     │                                                  ▼
     │                                            ┌───────────┐
     │                                            │  Step-CA  │
     │                                            │ (Root CA) │
     │                                            └─────┬─────┘
     │                                                  │
     │  4. TLS Handshake Complete                      │
     │<─────────────────────────────────────────────────┤
     │                                                  │
     │  5. HTTP Request (encrypted)                    │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │                                                  │ OPA Policy
     │                                                  │ Check
     │                                                  │
     │                                                  ▼
     │                                            ┌───────────┐
     │                                            │    OPA    │
     │                                            │  (Allow)  │
     │                                            └─────┬─────┘
     │                                                  │
     │  6. Forward to Target Service                   │
     │                                                  ├────────────>
     │                                                  │        ┌────────┐
     │                                                  │        │  n8n   │
     │                                                  │        └────────┘
```

**Certificate Details:**
```bash
# View service certificate
openssl x509 -in certs/temporal.crt -text -noout

# Output:
# Subject: CN=temporal.default.svc, O=ARIA Development CA
# Issuer: CN=ARIA Development Intermediate CA
# Validity: 2026-01-14 to 2026-04-14 (90 days)
# X509v3 Key Usage: Digital Signature, Key Encipherment
# X509v3 Extended Key Usage: TLS Web Server Authentication, TLS Web Client Authentication
```

**Evidence:** `docs/MTLS_SETUP.md:1-150`

---

## Authorization Flow

**Integrated OIDC + OPA Flow:**

```
┌──────────┐                                      ┌───────────┐
│  Client  │                                      │   Kong    │
│          │                                      │  Gateway  │
└────┬─────┘                                      └─────┬─────┘
     │                                                  │
     │  1. API Request                                 │
     │     Authorization: Bearer JWT_TOKEN             │
     ├─────────────────────────────────────────────────>
     │                                                  │
     │                                                  │ 2. Validate JWT
     │                                                  │
     │                                                  ▼
     │                                            ┌───────────┐
     │                                            │ Keycloak  │
     │                                            │   JWKS    │
     │                                            └─────┬─────┘
     │                                                  │
     │                                                  │ 3. JWT Valid
     │                                                  │
     │                                                  ▼
     │                                            ┌───────────┐
     │                                            │    OPA    │
     │                                            │  Policy   │
     │                                            └─────┬─────┘
     │                                                  │
     │                                                  │ 4. Policy Check
     │                                                  │    Input:
     │                                                  │    - user.role
     │                                                  │    - action
     │                                                  │    - resource
     │                                                  │
     │                                                  │ 5. Allow/Deny
     │                                                  │
     │  6. Forward Request (if allowed)                │
     │                                                  ├────────────>
     │                                                  │        ┌────────┐
     │                                                  │        │Service │
     │                                                  │        └────────┘
     │                                                  │
     │  7. Response                                    │
     │<─────────────────────────────────────────────────┤
```

**Implementation:** Kong OPA plugin configuration

**Evidence:** `config/opa/policies/authz.rego:1-67`

---

## Certificate Lifecycle Management

### Certificate Issuance

**Process:**
1. **Generate private key** for service
2. **Create CSR (Certificate Signing Request)** with service identity
3. **Submit CSR to Step-CA** for signing
4. **Receive signed certificate** from intermediate CA
5. **Deploy certificate** to service container

**Script:** `scripts/generate-service-certs.sh:1-183`

**Example:**
```bash
# Generate certificate for Temporal
./scripts/generate-service-certs.sh temporal

# Output:
# - certs/temporal.key (private key)
# - certs/temporal.crt (signed certificate)
# - certs/ca.crt (CA chain)
```

**Certificate Details:**
```bash
# View certificate info
openssl x509 -in certs/temporal.crt -text -noout

Subject: CN=temporal.default.svc
Issuer: CN=ARIA Development Intermediate CA
Validity: 2026-01-14 to 2026-04-14 (90 days)
```

---

### Certificate Rotation

**Process:**
1. **Check certificate expiration** (warn at 30 days)
2. **Generate new key pair** for service
3. **Request new certificate** from Step-CA
4. **Deploy new certificate** to service
5. **Backup old certificate** for rollback
6. **Restart service** with new certificate

**Script:** `scripts/rotate-certs.sh:1-247`

**Automated Rotation:** `scripts/setup-cert-rotation-cron.sh:1-87`

```bash
# Weekly certificate rotation check (every Sunday at 2 AM)
0 2 * * 0 /path/to/scripts/rotate-certs.sh --auto
```

**Evidence:** `docs/MTLS_SETUP.md:70-120`

---

### Certificate Verification

**Script:** `scripts/verify-mtls-setup.sh:1-148`

**Checks:**
1. **Step-CA running** and accessible
2. **Root CA certificate** present and valid
3. **Intermediate CA certificate** present and valid
4. **Service certificates** present and valid
5. **Certificate chain** verifiable
6. **mTLS handshake** successful

**Example Output:**
```
✅ Step-CA is running
✅ Root CA certificate valid (expires 2036-01-14)
✅ Intermediate CA certificate valid (expires 2031-01-14)
✅ temporal certificate valid (expires 2026-04-14)
✅ n8n certificate valid (expires 2026-04-14)
✅ agixt certificate valid (expires 2026-04-14)
✅ Certificate chain verification successful
✅ mTLS handshake test passed
```

---

## Secrets Management Workflows

### Populating Secrets

**Process:**
1. **Initialize Vault** (dev mode or production)
2. **Enable KV v2 secrets engine** at `secret/`
3. **Create secrets** for each service
4. **Set access policies** for agents and services
5. **Verify secret access** with test credentials

**Script:** Custom `scripts/populate-vault-secrets.sh` (to be created)

**Example:**
```bash
# Enable KV v2 engine
vault secrets enable -path=secret kv-v2

# Create LLM API keys
vault kv put secret/llm/openai api_key="sk-..."
vault kv put secret/llm/anthropic api_key="sk-ant-..."

# Create database credentials
vault kv put secret/databases/postgres \
  username="postgres" \
  password="secure-password"

# Create agent secrets
vault kv put secret/agents/agent-001 \
  api_key="agent-key-001"
```

---

### Accessing Secrets

**From Services:**
```python
# Python example using hvac
import hvac

client = hvac.Client(url='http://vault:8200')
client.token = os.environ['VAULT_TOKEN']

# Read secret
secret = client.secrets.kv.v2.read_secret_version(
    path='llm/openai',
    mount_point='secret'
)

api_key = secret['data']['data']['api_key']
```

**From Agents:**
```bash
# Using Vault CLI
export VAULT_ADDR='http://vault:8200'
export VAULT_TOKEN='agent-token'

vault kv get -field=api_key secret/llm/openai
```

**Evidence:** `config/vault/policy-agent.hcl:1-28`

---

### Secret Rotation

**Process:**
1. **Generate new secret** (API key, password, etc.)
2. **Update secret in Vault** (creates new version)
3. **Update service configuration** to use new secret
4. **Restart service** with new configuration
5. **Verify service operation** with new secret
6. **Revoke old secret** after grace period

**Example:**
```bash
# Update secret (creates version 2)
vault kv put secret/llm/openai api_key="sk-new-key"

# View secret versions
vault kv metadata get secret/llm/openai

# Rollback if needed
vault kv rollback -version=1 secret/llm/openai
```

---

## Integration Examples

### AGiXT Service Integration

**Configuration:**

**1. Service Certificate:**
```bash
# Generate mTLS certificate
./scripts/generate-service-certs.sh agixt

# Mount certificate in Docker Compose
volumes:
  - ./certs/agixt.crt:/certs/agixt.crt:ro
  - ./certs/agixt.key:/certs/agixt.key:ro
  - ./certs/ca.crt:/certs/ca.crt:ro
```

**2. OIDC Client Credentials:**
```bash
# Create Keycloak client
curl -X POST http://localhost:8080/admin/realms/agentic/clients \
  -H "Authorization: Bearer $ADMIN_TOKEN" \
  -d '{
    "clientId": "agixt-client",
    "secret": "agixt-secret",
    "serviceAccountsEnabled": true,
    "authorizationServicesEnabled": true
  }'
```

**3. Vault Policy:**
```hcl
# Allow AGiXT to read LLM API keys
path "secret/data/llm/*" {
  capabilities = ["read"]
}

# Allow AGiXT to manage agent secrets
path "secret/data/agents/*" {
  capabilities = ["read", "create", "update", "delete"]
}
```

**4. Application Code:**
```python
import requests
import hvac
from OpenSSL import crypto

# Get OIDC token
token_response = requests.post(
    'http://keycloak:8080/realms/agentic/protocol/openid-connect/token',
    data={
        'grant_type': 'client_credentials',
        'client_id': 'agixt-client',
        'client_secret': 'agixt-secret',
        'scope': 'read write'
    }
)
access_token = token_response.json()['access_token']

# Access Vault
vault_client = hvac.Client(url='http://vault:8200', token=access_token)
openai_key = vault_client.secrets.kv.v2.read_secret_version(
    path='llm/openai',
    mount_point='secret'
)['data']['data']['api_key']

# Make API call through Kong with mTLS
response = requests.get(
    'https://kong:8443/api/workflows',
    headers={'Authorization': f'Bearer {access_token}'},
    cert=('/certs/agixt.crt', '/certs/agixt.key'),
    verify='/certs/ca.crt'
)
```

---

### Temporal Workflow Integration

**Configuration:**

**1. mTLS Certificates:**
```bash
# Generate certificates for Temporal
./scripts/generate-service-certs.sh temporal

# Configure Temporal with mTLS
temporal:
  tls:
    client:
      certFile: /certs/temporal.crt
      keyFile: /certs/temporal.key
    server:
      requireClientAuth: true
      clientCaFiles: ["/certs/ca.crt"]
```

**2. Service Mesh Policy:**
```rego
# Allow Temporal to connect to NATS
allow_connection if {
    input.source.service == "temporal"
    input.destination.service == "nats"
}
```

**3. Workflow Code:**
```python
from temporalio.client import Client
import ssl

# mTLS context
ssl_context = ssl.create_default_context(cafile='/certs/ca.crt')
ssl_context.load_cert_chain('/certs/temporal.crt', '/certs/temporal.key')

# Connect to Temporal with mTLS
client = await Client.connect(
    "temporal:7233",
    tls=True,
    tls_ssl_context=ssl_context
)

# Execute workflow
result = await client.execute_workflow(
    MyWorkflow.run,
    args=["data"],
    id="workflow-001",
    task_queue="default"
)
```

---

## Operational Procedures

### Daily Operations

**Morning Checklist:**
1. **Check identity services health:**
   ```bash
   docker ps | grep -E '(keycloak|vault|step-ca|vaultwarden)'
   ```

2. **Verify OPA policy bundle loaded:**
   ```bash
   curl http://localhost:8181/v1/policies
   ```

3. **Check Kong Gateway status:**
   ```bash
   curl http://localhost:8001/status
   ```

4. **Review auth logs:**
   ```bash
   docker logs keycloak --since 1h | grep -E '(ERROR|WARN)'
   docker logs vault --since 1h | grep -E '(ERROR|WARN)'
   ```

---

### Weekly Maintenance

**Certificate Rotation Check:**
```bash
# Run automated certificate rotation
./scripts/rotate-certs.sh --auto

# Verify certificates
./scripts/verify-mtls-setup.sh
```

**Policy Update:**
```bash
# Update OPA policies
cd config/opa/policies/
git pull
docker restart opa

# Verify policy bundle loaded
curl http://localhost:8181/v1/policies
```

**Secret Rotation:**
```bash
# Rotate database credentials
vault kv put secret/databases/postgres \
  username="postgres" \
  password="new-secure-password"

# Update services with new credentials
docker-compose -f docker/docker-compose.state-storage.yml restart
```

---

### Incident Response

**Authentication Failure:**

**Symptoms:** Users unable to login, 401/403 errors

**Diagnosis:**
```bash
# Check Keycloak health
curl http://localhost:8080/health/ready

# Check Vault seal status
curl http://localhost:8200/v1/sys/seal-status

# Verify Kong connectivity
curl http://localhost:8001/status

# Check OPA policy evaluation
curl -X POST http://localhost:8181/v1/data/ros2/authz \
  -d '{"input":{"user":{"role":"admin"},"action":"read","resource":{"name":"test"}}}'
```

**Resolution:**
1. **Restart identity services:**
   ```bash
   docker-compose -f docker/docker-compose.identity.yml restart
   ```

2. **Verify network connectivity:**
   ```bash
   docker exec kong ping keycloak
   docker exec kong ping vault
   docker exec kong ping opa
   ```

3. **Check certificate validity:**
   ```bash
   ./scripts/verify-mtls-setup.sh
   ```

---

**Certificate Expiration:**

**Symptoms:** mTLS handshake failures, 503 errors

**Diagnosis:**
```bash
# Check certificate expiration
openssl x509 -in certs/temporal.crt -noout -enddate
openssl x509 -in certs/agixt.crt -noout -enddate
```

**Resolution:**
```bash
# Rotate expiring certificates
./scripts/rotate-certs.sh temporal
./scripts/rotate-certs.sh agixt

# Restart services
docker-compose -f docker/docker-compose.messaging.yml restart temporal
docker-compose -f docker/agixt.yml restart agixt
```

---

**Policy Violation:**

**Symptoms:** 403 Forbidden errors from Kong

**Diagnosis:**
```bash
# Test policy evaluation
curl -X POST http://localhost:8181/v1/data/ros2/authz \
  -H "Content-Type: application/json" \
  -d '{
    "input": {
      "user": {"role": "developer"},
      "action": "delete",
      "resource": {"name": "workflow-123"}
    }
  }'

# Expected result: {"result":{"allow":false,"deny":true}}
```

**Resolution:**
1. **Review policy rules** in `config/opa/policies/authz.rego`
2. **Update policy** if needed
3. **Reload OPA policies:**
   ```bash
   docker restart opa
   ```

---

## Security Best Practices

### Certificate Management

1. **Short-lived certificates:** 90-day validity for all service certificates
2. **Automated rotation:** Weekly cron job checks expiration
3. **Offline root CA:** Root CA key stored offline, only used for intermediate signing
4. **Separate intermediate CA:** Online intermediate CA for day-to-day signing
5. **Certificate backup:** Backup certificates before rotation

### Secret Management

1. **No secrets in code:** All secrets stored in Vault
2. **Least privilege:** Services only access required secrets
3. **Secret rotation:** Regular rotation of API keys and passwords
4. **Audit logging:** Log all secret access for compliance
5. **Encryption at rest:** Vault encrypts all secrets

### Authentication

1. **No plaintext passwords:** All passwords hashed (Keycloak, Vaultwarden)
2. **2FA/MFA:** Enable for admin accounts
3. **Strong password policy:** Minimum 12 characters, complexity requirements
4. **Token expiration:** Short-lived access tokens (1 hour)
5. **Refresh token rotation:** New refresh token on each use

### Authorization

1. **Default deny:** OPA policies default to deny
2. **RBAC enforcement:** Role-based access control for all resources
3. **Policy versioning:** Track policy changes in Git
4. **Policy testing:** Unit tests for OPA policies
5. **Audit logging:** Log all authorization decisions

### Network Security

1. **mTLS everywhere:** No plaintext service communication
2. **Network segmentation:** Separate networks for services
3. **Gateway choke point:** All external traffic through Kong
4. **Rate limiting:** Prevent abuse and DDoS
5. **HTTPS only:** TLS 1.3 for all external connections

---

## Troubleshooting Guide

### Keycloak Issues

**Problem:** Keycloak admin console inaccessible

**Solution:**
```bash
# Check Keycloak health
curl http://localhost:8080/health/ready

# Check logs
docker logs keycloak --tail 100

# Verify database connection
docker exec keycloak-postgres psql -U keycloak -d keycloak -c '\dt'

# Restart Keycloak
docker-compose -f docker/docker-compose.identity.yml restart keycloak
```

---

**Problem:** OIDC token validation fails

**Solution:**
```bash
# Verify JWKS endpoint accessible
curl http://localhost:8080/realms/agentic/protocol/openid-connect/certs

# Check token expiration
echo $JWT_TOKEN | cut -d. -f2 | base64 -d | jq .exp

# Verify clock sync
date
docker exec keycloak date
```

---

### Vault Issues

**Problem:** Vault sealed

**Solution:**
```bash
# Check seal status
curl http://localhost:8200/v1/sys/seal-status

# Unseal Vault (production)
vault operator unseal $UNSEAL_KEY_1
vault operator unseal $UNSEAL_KEY_2
vault operator unseal $UNSEAL_KEY_3

# Dev mode (auto-unseals on restart)
docker-compose -f docker/vault.yml restart vault
```

---

**Problem:** Cannot read secrets

**Solution:**
```bash
# Verify token validity
vault token lookup

# Check policy attached to token
vault token lookup -format=json | jq .data.policies

# Test secret read with proper path
vault kv get -mount=secret llm/openai

# Verify policy allows read
vault policy read agent-policy
```

---

### Step-CA Issues

**Problem:** Certificate issuance fails

**Solution:**
```bash
# Check Step-CA health
curl http://localhost:9000/health

# Verify CA initialized
docker exec step-ca step ca health

# Check CA provisioner
docker exec step-ca step ca provisioner list

# Re-initialize if needed
./scripts/init-step-ca.sh
```

---

**Problem:** mTLS handshake fails

**Solution:**
```bash
# Verify certificate validity
openssl x509 -in certs/temporal.crt -noout -dates

# Check certificate chain
openssl verify -CAfile certs/ca.crt certs/temporal.crt

# Test mTLS connection
curl -v --cert certs/temporal.crt --key certs/temporal.key \
  --cacert certs/ca.crt https://kong:8443/api/health
```

---

### OPA Issues

**Problem:** Policy evaluation returns unexpected result

**Solution:**
```bash
# Test policy directly
opa eval -d config/opa/policies/authz.rego \
  'data.ros2.authz.allow' \
  --input <(echo '{"user":{"role":"admin"},"action":"read","resource":{"name":"test"}}')

# Run policy tests
opa test config/opa/policies/

# Check policy syntax
opa check config/opa/policies/authz.rego
```

---

**Problem:** OPA not loading policies

**Solution:**
```bash
# Check OPA logs
docker logs opa --tail 50

# Verify policy volume mount
docker inspect opa | jq '.[0].Mounts'

# Manually load policies
curl -X PUT http://localhost:8181/v1/policies/authz \
  --data-binary @config/opa/policies/authz.rego
```

---

### Kong Gateway Issues

**Problem:** 502 Bad Gateway

**Solution:**
```bash
# Check Kong status
curl http://localhost:8001/status

# Verify upstream connectivity
docker exec kong ping keycloak
docker exec kong ping vault
docker exec kong ping opa

# Check Kong routes
curl http://localhost:8001/routes

# Review Kong error logs
docker logs kong --tail 100 | grep ERROR
```

---

**Problem:** OAuth2 token validation fails

**Solution:**
```bash
# Verify OAuth2 plugin configured
curl http://localhost:8001/plugins | jq '.data[] | select(.name=="oauth2")'

# Test token introspection
curl -X POST http://localhost:8000/oauth2/introspect \
  -d token=$ACCESS_TOKEN

# Check Keycloak connectivity from Kong
docker exec kong curl http://keycloak:8080/health/ready
```

---

## Summary

This phase documented a **comprehensive zero-trust security architecture** with:

### Identity Layer (L5)
- **Step-CA:** PKI with 10-year root, 5-year intermediate, 90-day service certificates
- **Keycloak:** OIDC provider with RBAC, social login, 2FA support
- **HashiCorp Vault:** KV v2 secrets, dynamic credentials, OIDC integration
- **Vaultwarden:** Bitwarden-compatible password vault for team credentials

### Policy Layer
- **OPA:** Policy-based authorization with Rego policies
- **RBAC Rules:** Admin, developer, viewer, agent roles
- **Service Mesh Policies:** Explicit service-to-service allow list

### Gateway Layer
- **Kong Gateway:** API gateway with OAuth2, mTLS, rate limiting
- **Single Enforcement Point:** All external traffic through Kong
- **Plugin Architecture:** Modular security plugins

### Authentication Flows
- **OIDC (Human Users):** Authorization code with PKCE
- **Client Credentials (AI Agents):** Machine-to-machine OAuth2
- **mTLS (Services):** Certificate-based mutual authentication

### Operations
- **Certificate Lifecycle:** Automated rotation, verification, backup
- **Secrets Management:** Vault-based storage, rotation, audit
- **Policy Management:** Versioned, tested, declarative policies
- **Incident Response:** Runbooks for common failure scenarios

### Security Posture
- **Zero-trust:** No implicit trust, always verify
- **Defense in depth:** Multiple security layers
- **Least privilege:** Minimal permissions by default
- **Audit trail:** Comprehensive logging of auth decisions

**Next Phase:** Phase 7 - Create runbooks and cookbooks for operations
