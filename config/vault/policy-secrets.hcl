# =============================================================================
# Vault Policy for ARIA Platform Secrets
# =============================================================================
# This policy defines access control for all platform secrets including:
# - Identity & Access Management (Keycloak, Vaultwarden)
# - Database credentials (PostgreSQL, ClickHouse)
# - API keys and tokens
# - Object storage credentials (MinIO)
#
# Usage:
#   vault policy write aria-secrets /home/user/ros2-humble-env/config/vault/policy-secrets.hcl
#   vault token create -policy=aria-secrets
#
# Security Notes:
# - Principle of least privilege
# - Separate paths for different service categories
# - Read-only access for most services
# - Admin secrets require special permissions
# =============================================================================

# -----------------------------------------------------------------------------
# Identity & Access Management Secrets
# -----------------------------------------------------------------------------
# Keycloak admin credentials, Vaultwarden tokens
path "secret/data/identity/keycloak/*" {
  capabilities = ["read"]
}

path "secret/data/identity/vaultwarden/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Database Credentials (PostgreSQL, ClickHouse)
# -----------------------------------------------------------------------------
# All PostgreSQL database credentials
path "secret/data/database/postgres/*" {
  capabilities = ["read"]
}

# ClickHouse credentials for TensorZero
path "secret/data/database/clickhouse/*" {
  capabilities = ["read"]
}

# Allow services to rotate their own database passwords
path "secret/data/database/*/rotate" {
  capabilities = ["create", "update"]
}

# -----------------------------------------------------------------------------
# LLMOps & AI Platform Secrets
# -----------------------------------------------------------------------------
# MLflow database credentials
path "secret/data/llmops/mlflow/*" {
  capabilities = ["read"]
}

# TensorZero credentials
path "secret/data/llmops/tensorzero/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Automation Platform Secrets
# -----------------------------------------------------------------------------
# n8n credentials and API keys
path "secret/data/automation/n8n/*" {
  capabilities = ["read"]
}

# OPA policy signing keys
path "secret/data/automation/opa/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Observability Stack Secrets
# -----------------------------------------------------------------------------
# Grafana admin credentials
path "secret/data/observability/grafana/*" {
  capabilities = ["read"]
}

# Umami analytics database and app secrets
path "secret/data/observability/umami/*" {
  capabilities = ["read"]
}

# Netdata cloud token
path "secret/data/observability/netdata/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Messaging & Workflow Secrets
# -----------------------------------------------------------------------------
# Temporal database credentials
path "secret/data/messaging/temporal/*" {
  capabilities = ["read"]
}

# NATS credentials (if authentication enabled)
path "secret/data/messaging/nats/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Edge Gateway Secrets
# -----------------------------------------------------------------------------
# Kong database credentials
path "secret/data/edge/kong/*" {
  capabilities = ["read"]
}

# Konga admin credentials
path "secret/data/edge/konga/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# UI Platform Secrets
# -----------------------------------------------------------------------------
# Lobe Chat database and authentication
path "secret/data/ui/lobe/*" {
  capabilities = ["read"]
}

# Open-Lovable API keys
path "secret/data/ui/lovable/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Object Storage (MinIO) Credentials
# -----------------------------------------------------------------------------
# AGiXT MinIO credentials
path "secret/data/storage/agixt/*" {
  capabilities = ["read"]
}

# Lobe Chat MinIO credentials
path "secret/data/storage/lobe/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Kubernetes & Orchestration Secrets
# -----------------------------------------------------------------------------
# K3s cluster token
path "secret/data/k8s/k3s/*" {
  capabilities = ["read"]
}

# Argo CD credentials
path "secret/data/k8s/argocd/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Agent Runtime Secrets
# -----------------------------------------------------------------------------
# AIOS and AGiXT agent secrets (from policy-agent.hcl)
path "secret/data/agents/*" {
  capabilities = ["read", "create", "update"]
}

# LLM API keys for agents
path "secret/data/llm/*" {
  capabilities = ["read"]
}

# Tool execution credentials
path "secret/data/tools/*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Admin & Platform Management
# -----------------------------------------------------------------------------
# Platform-wide admin secrets (restricted)
path "secret/data/admin/*" {
  capabilities = ["deny"]
}

# Secret rotation and management (admin only)
path "secret/data/*/rotate" {
  capabilities = ["create", "update"]
}

# -----------------------------------------------------------------------------
# Secret Versioning & Metadata
# -----------------------------------------------------------------------------
# Allow reading secret metadata for auditing
path "secret/metadata/*" {
  capabilities = ["read", "list"]
}

# Deny deletion of secrets (use TTL for expiration)
path "secret/data/*" {
  capabilities = ["read"]
  denied_parameters = {
    "delete" = []
  }
}

# -----------------------------------------------------------------------------
# Dynamic Database Credentials (Optional)
# -----------------------------------------------------------------------------
# If using Vault's database secrets engine for dynamic credentials
path "database/creds/postgres-*" {
  capabilities = ["read"]
}

path "database/creds/temporal-*" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# PKI for TLS Certificates (Optional)
# -----------------------------------------------------------------------------
# Issue certificates for internal service-to-service communication
path "pki/issue/*" {
  capabilities = ["create", "update"]
}

# Read CA certificate
path "pki/ca" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Transit Encryption (Optional)
# -----------------------------------------------------------------------------
# Encrypt/decrypt sensitive data in transit
path "transit/encrypt/aria-*" {
  capabilities = ["update"]
}

path "transit/decrypt/aria-*" {
  capabilities = ["update"]
}

# -----------------------------------------------------------------------------
# Token Management
# -----------------------------------------------------------------------------
# Allow token self-renewal
path "auth/token/renew-self" {
  capabilities = ["update"]
}

# Allow token self-lookup
path "auth/token/lookup-self" {
  capabilities = ["read"]
}

# -----------------------------------------------------------------------------
# Audit & Compliance
# -----------------------------------------------------------------------------
# List all secret paths for auditing (admin only)
path "secret/*" {
  capabilities = ["list"]
}

# =============================================================================
# Secret Path Structure
# =============================================================================
# Recommended structure for storing secrets in Vault:
#
# secret/data/identity/keycloak/admin-password
# secret/data/identity/keycloak/db-password
# secret/data/identity/vaultwarden/admin-token
#
# secret/data/database/postgres/keycloak
# secret/data/database/postgres/mlflow
# secret/data/database/postgres/n8n
# secret/data/database/postgres/temporal
# secret/data/database/postgres/kong
# secret/data/database/postgres/lobe
# secret/data/database/postgres/umami
# secret/data/database/clickhouse/tensorzero
#
# secret/data/storage/agixt/minio-root-user
# secret/data/storage/agixt/minio-root-password
# secret/data/storage/lobe/minio-root-user
# secret/data/storage/lobe/minio-root-password
#
# secret/data/llmops/mlflow/db-password
# secret/data/automation/n8n/admin-password
# secret/data/observability/grafana/admin-password
# secret/data/observability/umami/app-secret
# secret/data/edge/kong/db-password
# secret/data/ui/lobe/next-auth-secret
#
# Example commands to store secrets:
#   vault kv put secret/identity/keycloak/admin-password value="<secure-password>"
#   vault kv put secret/database/postgres/keycloak password="<secure-password>"
#   vault kv put secret/storage/agixt/minio-root-user value="<secure-username>"
# =============================================================================
