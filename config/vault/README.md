# HashiCorp Vault Integration for ARIA Platform

This directory contains Vault policies and configuration for secure secret management across the ARIA platform.

> **P0-004 Update (2026-01-15)**: Vault is now configured for **production mode** by default with persistent file storage. Set `VAULT_DEV_MODE=true` in your environment for development/testing.

## Quick Start

### Development Mode (quick testing)
```bash
# Set dev mode
export VAULT_DEV_MODE=true
export VAULT_DEV_ROOT_TOKEN_ID=root

# Start services
docker compose -f docker/docker-compose.identity.yml up -d vault

# Use root token for testing
export VAULT_ADDR='http://localhost:8200'
export VAULT_TOKEN='root'
```

### Production Mode (persistent storage)
```bash
# Ensure VAULT_DEV_MODE is not set or set to false
unset VAULT_DEV_MODE
# OR
export VAULT_DEV_MODE=false

# Start Vault
docker compose -f docker/docker-compose.identity.yml up -d vault

# Initialize Vault (first time only)
export VAULT_ADDR='http://localhost:8200'
vault operator init -key-shares=5 -key-threshold=3

# IMPORTANT: Save the unseal keys and root token securely!
# Example output:
#   Unseal Key 1: xxx
#   Unseal Key 2: xxx
#   ...
#   Initial Root Token: hvs.xxxxx

# Unseal Vault (required after every restart)
vault operator unseal <key1>
vault operator unseal <key2>
vault operator unseal <key3>

# Authenticate
vault login <root-token>
```

## Security Issues Fixed

### SEC-001: Keycloak Credentials (HIGH Severity)
**Status**: RESOLVED
**Files Modified**: `/home/user/ripple-env/docker-compose.identity.yml`

**Before**:
```yaml
KEYCLOAK_ADMIN: admin
KEYCLOAK_ADMIN_PASSWORD: admin
KC_DB_PASSWORD: keycloak
```

**After**:
```yaml
KEYCLOAK_ADMIN: ${KEYCLOAK_ADMIN:-admin}
KEYCLOAK_ADMIN_PASSWORD: ${KEYCLOAK_ADMIN_PASSWORD:-changeme}
KC_DB_PASSWORD: ${KEYCLOAK_DB_PASSWORD:-changeme}
```

### SEC-002: PostgreSQL Passwords (HIGH Severity)
**Status**: RESOLVED
**Files Modified**: All `docker-compose*.yml` files with PostgreSQL databases

**Databases Fixed**:
- Keycloak DB
- MLflow DB
- n8n DB
- Temporal DB
- Kong DB
- Lobe Chat DB
- Umami DB

**Pattern Applied**:
```yaml
POSTGRES_PASSWORD: ${SERVICE_DB_PASSWORD:-changeme}
```

## Files in This Directory

### policy-agent.hcl
Vault policy for agent runtime (AIOS, AGiXT) access to secrets.

### policy-secrets.hcl
Comprehensive Vault policy for all platform secrets including:
- Identity & Access Management
- Database credentials
- API keys and tokens
- Object storage credentials

## Setup Instructions

### 1. Install HashiCorp Vault

#### Using Docker (Development)
```bash
# Start Vault in dev mode (NOT for production!)
docker run -d --name vault \
  --cap-add=IPC_LOCK \
  -p 8200:8200 \
  -e VAULT_DEV_ROOT_TOKEN_ID=root \
  vault:1.15

# Set environment variables
export VAULT_ADDR='http://localhost:8200'
export VAULT_TOKEN='root'
```

#### Using Package Manager (Production)
```bash
# Install Vault
wget -O- https://apt.releases.hashicorp.com/gpg | sudo gpg --dearmor -o /usr/share/keyrings/hashicorp-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/hashicorp-archive-keyring.gpg] https://apt.releases.hashicorp.com $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/hashicorp.list
sudo apt update && sudo apt install vault

# Initialize Vault
vault operator init
vault operator unseal
```

### 2. Enable KV Secrets Engine

```bash
# Enable KV v2 secrets engine
vault secrets enable -path=secret kv-v2

# Verify
vault secrets list
```

### 3. Load Vault Policies

```bash
# Load the secrets policy
vault policy write aria-secrets /home/user/ripple-env/config/vault/policy-secrets.hcl

# Load the agent policy
vault policy write aria-agent /home/user/ripple-env/config/vault/policy-agent.hcl

# Verify
vault policy list
vault policy read aria-secrets
```

### 4. Create Tokens for Services

```bash
# Create a token for platform services
vault token create -policy=aria-secrets -period=24h -display-name="aria-platform"

# Create a token for agent runtime
vault token create -policy=aria-agent -period=24h -display-name="aria-agents"
```

### 5. Store Secrets in Vault

#### Generate Secure Passwords
```bash
# Generate random passwords
openssl rand -base64 32  # For passwords
openssl rand -hex 32     # For tokens
```

#### Store Identity Secrets
```bash
# Keycloak
vault kv put secret/identity/keycloak/admin-password value="<secure-password>"
vault kv put secret/identity/keycloak/db-password value="<secure-password>"

# Vaultwarden
vault kv put secret/identity/vaultwarden/admin-token value="$(openssl rand -base64 32)"
```

#### Store Database Credentials
```bash
# PostgreSQL databases
vault kv put secret/database/postgres/keycloak password="<secure-password>"
vault kv put secret/database/postgres/mlflow password="<secure-password>"
vault kv put secret/database/postgres/n8n password="<secure-password>"
vault kv put secret/database/postgres/temporal password="<secure-password>"
vault kv put secret/database/postgres/kong password="<secure-password>"
vault kv put secret/database/postgres/lobe password="<secure-password>"
vault kv put secret/database/postgres/umami password="<secure-password>"

# ClickHouse
vault kv put secret/database/clickhouse/tensorzero password="<secure-password>"
```

#### Store Storage Credentials
```bash
# MinIO for AGiXT
vault kv put secret/storage/agixt/minio-root-user value="agixt-admin"
vault kv put secret/storage/agixt/minio-root-password value="<secure-password>"

# MinIO for Lobe Chat
vault kv put secret/storage/lobe/minio-root-user value="lobe-admin"
vault kv put secret/storage/lobe/minio-root-password value="<secure-password>"
```

#### Store Application Secrets
```bash
# n8n
vault kv put secret/automation/n8n/admin-password value="<secure-password>"

# Grafana
vault kv put secret/observability/grafana/admin-password value="<secure-password>"

# Umami
vault kv put secret/observability/umami/app-secret value="$(openssl rand -base64 32)"

# Lobe Chat
vault kv put secret/ui/lobe/next-auth-secret value="$(openssl rand -base64 32)"

# K3s
vault kv put secret/k8s/k3s/token value="$(openssl rand -hex 32)"
```

### 6. Retrieve Secrets from Vault

#### Manual Retrieval
```bash
# Read a secret
vault kv get secret/identity/keycloak/admin-password
vault kv get -field=value secret/identity/keycloak/admin-password

# List all secrets
vault kv list secret/
vault kv list secret/database/postgres/
```

#### Export to Environment Variables
```bash
# Export secrets to environment variables
export KEYCLOAK_ADMIN_PASSWORD=$(vault kv get -field=value secret/identity/keycloak/admin-password)
export KEYCLOAK_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/keycloak)
export MLFLOW_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/mlflow)

# Verify
echo $KEYCLOAK_ADMIN_PASSWORD
```

#### Automated Script
```bash
#!/bin/bash
# scripts/load-secrets-from-vault.sh

set -e

# Check Vault is accessible
vault status > /dev/null 2>&1 || {
  echo "Error: Cannot connect to Vault at $VAULT_ADDR"
  exit 1
}

# Export all secrets
export KEYCLOAK_ADMIN_PASSWORD=$(vault kv get -field=value secret/identity/keycloak/admin-password)
export KEYCLOAK_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/keycloak)
export VAULTWARDEN_ADMIN_TOKEN=$(vault kv get -field=value secret/identity/vaultwarden/admin-token)

export CLICKHOUSE_PASSWORD=$(vault kv get -field=value secret/database/clickhouse/tensorzero)
export MLFLOW_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/mlflow)

export N8N_BASIC_AUTH_PASSWORD=$(vault kv get -field=value secret/automation/n8n/admin-password)
export N8N_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/n8n)

export GRAFANA_ADMIN_PASSWORD=$(vault kv get -field=value secret/observability/grafana/admin-password)
export UMAMI_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/umami)
export UMAMI_APP_SECRET=$(vault kv get -field=value secret/observability/umami/app-secret)

export TEMPORAL_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/temporal)

export KONG_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/kong)

export LOBE_DB_PASSWORD=$(vault kv get -field=value secret/database/postgres/lobe)
export LOBE_MINIO_ROOT_USER=$(vault kv get -field=value secret/storage/lobe/minio-root-user)
export LOBE_MINIO_ROOT_PASSWORD=$(vault kv get -field=value secret/storage/lobe/minio-root-password)
export LOBE_NEXT_AUTH_SECRET=$(vault kv get -field=value secret/ui/lobe/next-auth-secret)

export AGIXT_POSTGRES_PASSWORD=$(vault kv get -field=value secret/database/postgres/agixt)
export AGIXT_MINIO_ROOT_USER=$(vault kv get -field=value secret/storage/agixt/minio-root-user)
export AGIXT_MINIO_ROOT_PASSWORD=$(vault kv get -field=value secret/storage/agixt/minio-root-password)

export K3S_TOKEN=$(vault kv get -field=value secret/k8s/k3s/token)

echo "All secrets loaded from Vault successfully!"
```

### 7. Use with Docker Compose

#### Option 1: Environment File
```bash
# Load secrets from Vault
source scripts/load-secrets-from-vault.sh

# Start services (secrets are in environment)
docker-compose -f docker-compose.identity.yml up -d
```

#### Option 2: Direct Substitution
```bash
# Export Vault token
export VAULT_ADDR='http://localhost:8200'
export VAULT_TOKEN='your-vault-token'

# Use vault CLI in docker-compose
KEYCLOAK_ADMIN_PASSWORD=$(vault kv get -field=value secret/identity/keycloak/admin-password) \
  docker-compose -f docker-compose.identity.yml up -d
```

#### Option 3: Vault Agent (Production)
See: https://www.vaultproject.io/docs/agent

## Secret Rotation

### Automated Rotation Script
```bash
#!/bin/bash
# scripts/rotate-secrets.sh

SERVICE=$1
NEW_PASSWORD=$(openssl rand -base64 32)

# Store new password in Vault
vault kv put secret/database/postgres/$SERVICE password="$NEW_PASSWORD"

# Update running container
export ${SERVICE^^}_DB_PASSWORD=$NEW_PASSWORD
docker-compose -f docker-compose.$SERVICE.yml up -d --no-deps ${SERVICE}-db
```

### Rotation Schedule
- Database passwords: Every 90 days
- API keys: Every 180 days
- Admin tokens: Every 30 days

## Security Best Practices

1. **Never commit secrets to version control**
   - Use `.gitignore` for `.env` files
   - Use Vault for all sensitive data

2. **Use principle of least privilege**
   - Each service gets only the secrets it needs
   - Use separate policies for different service tiers

3. **Enable audit logging**
   ```bash
   vault audit enable file file_path=/var/log/vault_audit.log
   ```

4. **Use dynamic database credentials** (Production)
   ```bash
   vault secrets enable database
   vault write database/config/postgres \
     plugin_name=postgresql-database-plugin \
     connection_url="postgresql://{{username}}:{{password}}@localhost:5432/postgres" \
     allowed_roles="*"
   ```

5. **Implement secret versioning**
   - KV v2 automatically versions secrets
   - Rollback if needed: `vault kv rollback -version=1 secret/path`

6. **Monitor secret access**
   ```bash
   # View audit logs
   tail -f /var/log/vault_audit.log | jq
   ```

## Troubleshooting

### Cannot connect to Vault
```bash
# Check Vault status
vault status

# Check network connectivity
curl http://localhost:8200/v1/sys/health
```

### Permission denied reading secret
```bash
# Check your token's policies
vault token lookup

# Read the policy
vault policy read aria-secrets
```

### Secret not found
```bash
# List all secrets
vault kv list secret/

# Check path is correct
vault kv get secret/database/postgres/keycloak
```

## Migration from .env to Vault

1. **Backup existing .env files**
   ```bash
   cp .env .env.backup
   ```

2. **Extract secrets from .env**
   ```bash
   grep -v '^#' .env | grep '=' | while IFS='=' read -r key value; do
     echo "$key=$value"
   done
   ```

3. **Store in Vault**
   ```bash
   # Script to migrate from .env to Vault
   ./scripts/migrate-env-to-vault.sh .env
   ```

4. **Test retrieval**
   ```bash
   source scripts/load-secrets-from-vault.sh
   env | grep PASSWORD
   ```

5. **Remove .env files**
   ```bash
   rm .env .env.backup
   ```

## References

- [HashiCorp Vault Documentation](https://www.vaultproject.io/docs)
- [Vault KV Secrets Engine](https://www.vaultproject.io/docs/secrets/kv/kv-v2)
- [Vault Policies](https://www.vaultproject.io/docs/concepts/policies)
- [OWASP Secrets Management](https://cheatsheetseries.owasp.org/cheatsheets/Secrets_Management_Cheat_Sheet.html)

## Support

For security issues, contact the Security Domain Team Lead.
