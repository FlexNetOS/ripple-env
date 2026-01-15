# Vault Configuration
# P0-004: HashiCorp Vault secrets management configuration
# Updated: 2026-01-15 - Production-ready with file storage

# =============================================================================
# IMPORTANT: This is the PRODUCTION configuration
# For development mode, set VAULT_DEV_MODE=true in docker-compose
# =============================================================================

# Listener configuration
listener "tcp" {
  address     = "0.0.0.0:8200"

  # TLS Configuration (P0-005: mTLS)
  # Uncomment when Step-CA certificates are generated
  # tls_disable = 0
  # tls_cert_file = "/vault/certs/vault.crt"
  # tls_key_file = "/vault/certs/vault.key"
  # tls_client_ca_file = "/vault/certs/ca.crt"
  # tls_require_and_verify_client_cert = true

  # Development fallback (disable in production)
  tls_disable = 1
}

# Storage backend - FILE for persistence
# P0-004: Changed from inmem to file for production
storage "file" {
  path = "/vault/file"
}

# Alternative: Raft storage for HA (uncomment for multi-node)
# storage "raft" {
#   path    = "/vault/raft"
#   node_id = "vault_1"
# }

# Audit logging
audit "file" {
  path = "/vault/logs/audit.log"
  mode = "0644"
}

# UI configuration
ui = true

# Maximum request size (default: 32MB)
# max_request_size = 33554432

# Maximum response size (default: 32MB)
# max_response_size = 33554432

# Default lease TTL for tokens and secrets
default_lease_ttl = "768h"

# Maximum lease TTL for tokens and secrets
max_lease_ttl = "768h"

# Disable cache for development (performance impact)
disable_cache = false

# Enable raw endpoint (for advanced operations)
raw_storage_endpoint = true

# Plugin directory
plugin_directory = "/vault/plugins"