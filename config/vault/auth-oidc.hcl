# Vault OIDC Authentication Configuration
# P1-004: Vault-Keycloak OIDC Integration
#
# This configuration enables OIDC authentication for Vault using Keycloak
# as the identity provider.
#
# Usage:
#   vault auth enable oidc
#   vault write auth/oidc/config @auth-oidc.hcl
#
# Prerequisites:
#   1. Keycloak must be running and accessible
#   2. A Vault client must be configured in Keycloak
#   3. Environment variables must be set (see below)

# OIDC Discovery URL for Keycloak
# This is the base URL where Vault will discover OIDC endpoints
# Format: https://<keycloak-host>/realms/<realm-name>
oidc_discovery_url = "http://keycloak:8080/realms/agentic"

# Client ID for Vault in Keycloak
# This should match the client ID configured in Keycloak for Vault
oidc_client_id = "vault"

# Client Secret for Vault in Keycloak
# SECURITY: This should be stored in a secure location and injected via environment variable
# vault write auth/oidc/config oidc_client_secret="${VAULT_OIDC_CLIENT_SECRET}"
# oidc_client_secret = "${VAULT_OIDC_CLIENT_SECRET}"

# Default role for OIDC authentication
default_role = "reader"

# Redirect URIs that Keycloak will accept
# These must match the allowed redirect URIs configured in Keycloak
# Note: Add your actual Vault URL here
oidc_redirect_uris = [
  "http://localhost:8200/ui/vault/auth/oidc/oidc/callback",
  "http://vault:8200/ui/vault/auth/oidc/oidc/callback",
  "http://127.0.0.1:8200/ui/vault/auth/oidc/oidc/callback"
]

# Supported response types
oidc_response_types = ["code"]

# Supported response modes
oidc_response_mode = "query"

# Additional OIDC scopes to request
oidc_scopes = [
  "openid",
  "profile",
  "email",
  "groups"
]

# JWT validation configuration
jwt_validation_pubkeys = []
jwt_supported_algs = ["RS256"]

# Namespace in access token claim
# This allows multi-tenant OIDC configurations
namespace_in_state = false

# Provider configuration
provider_config = {
  provider = "keycloak"
}

# Token TTL and Max TTL
bound_issuer = "http://keycloak:8080/realms/agentic"

# Verbose OIDC logging for debugging (disable in production)
verbose_oidc_logging = false

# Example Role Configuration
# Create roles separately using:
#
# vault write auth/oidc/role/reader \
#   bound_audiences="vault" \
#   allowed_redirect_uris="http://localhost:8200/ui/vault/auth/oidc/oidc/callback" \
#   user_claim="sub" \
#   policies="reader"
#
# vault write auth/oidc/role/admin \
#   bound_audiences="vault" \
#   allowed_redirect_uris="http://localhost:8200/ui/vault/auth/oidc/oidc/callback" \
#   user_claim="sub" \
#   policies="admin" \
#   groups_claim="groups" \
#   bound_claims='{"groups":["vault-admins"]}'

# Environment Variables Required:
# - VAULT_OIDC_CLIENT_SECRET: The client secret from Keycloak
# - VAULT_ADDR: Vault server address
# - KEYCLOAK_URL: Keycloak server address

# To enable this auth method:
# 1. Enable the OIDC auth method:
#    vault auth enable oidc
#
# 2. Configure the OIDC auth method:
#    vault write auth/oidc/config @config/vault/auth-oidc.hcl \
#      oidc_client_secret="${VAULT_OIDC_CLIENT_SECRET}"
#
# 3. Create a role:
#    vault write auth/oidc/role/reader \
#      bound_audiences="vault" \
#      allowed_redirect_uris="http://localhost:8200/ui/vault/auth/oidc/oidc/callback" \
#      user_claim="sub" \
#      policies="reader"
#
# 4. Test authentication:
#    vault login -method=oidc role=reader

# Notes:
# - For production, use HTTPS URLs instead of HTTP
# - Store client secret in Vault's own secure storage or environment variables
# - Configure appropriate policies for each role
# - Consider using group claims for role-based access control
# - Review Keycloak client configuration to ensure redirect URIs match
