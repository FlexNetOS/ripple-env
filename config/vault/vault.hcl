# Vault Configuration
# P0-003: HashiCorp Vault secrets management configuration

# Listener configuration
listener "tcp" {
  address     = "0.0.0.0:8200"
  tls_disable = 1  # Development only - enable TLS for production
}

# Storage backend
# For development: use in-memory ("inmem")
# For production: use "file" or "consul"
storage "inmem" {}

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