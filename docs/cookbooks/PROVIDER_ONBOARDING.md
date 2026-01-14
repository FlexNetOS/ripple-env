# Provider Onboarding

Step-by-step procedure for integrating a new external service provider into the stack.

## What's in the repo (evidence)

- Existing providers: `docs/PROVIDERS.md` (documents 10+ current providers)
- Docker compose files: `docker/docker-compose.*.yml` (5 compose files)
- Port registry: `docs/PORTS.md` (tracks all service ports)
- Network security: `docs/CONTAINER_SECURITY.md` (network segmentation rules)
- Service catalog: `docs/CATALOG.json` (machine-readable service registry)
- API reference: `docs/API_REFERENCE.md` (documents all service APIs)

## Goal

1. Add new provider to appropriate docker-compose file
2. Prevent port conflicts with existing services
3. Apply correct network segmentation
4. Configure authentication following established patterns
5. Update all documentation indexes

## Quick start

### Step 1: Choose Docker Compose File

Select the appropriate compose file based on service category:

| File | Purpose | Existing Services |
|------|---------|-------------------|
| `docker-compose.identity.yml` | Auth & secrets | Keycloak, Vault |
| `docker-compose.data.yml` | Databases | PostgreSQL, Neo4j |
| `docker-compose.ai.yml` | AI/ML services | LocalAI, MindsDB |
| `docker-compose.observability.yml` | Monitoring | Prometheus, Grafana |
| `docker-compose.messaging.yml` | Event streams | NATS, Temporal |

**Evidence**: `docker/` directory contains these compose files

### Step 2: Check Port Conflicts

Before assigning a port, verify it's not already in use:

```bash
# Check port registry
grep "<your-port>" docs/PORTS.md

# Or use the validation script
./scripts/validate-resources.sh | grep -i port
```

**Reserved Ports** (from `docs/PORTS.md`):
- 3000-3099: UI services
- 5000-5099: API services
- 8000-8099: Internal services
- 9090-9099: Monitoring/metrics

**Next available ports**: Check `docs/PORTS.md` for current assignments

**Evidence**: `docs/PORTS.md` tracks all 9+ critical service ports

### Step 3: Determine Network Placement

Apply network segmentation rules from `docs/CONTAINER_SECURITY.md`:

| Network | Purpose | Access Level |
|---------|---------|--------------|
| `frontend` | Public-facing services | External access |
| `backend` | Internal APIs | Service-to-service only |
| `data` | Databases | Backend services only |
| `monitoring` | Observability | Monitoring tools only |

**Example**:
```yaml
# In docker-compose.<category>.yml
services:
  my-new-service:
    networks:
      - backend  # Internal API, no public access
```

**Evidence**: `docs/CONTAINER_SECURITY.md` documents network topology

### Step 4: Configure Authentication

Follow existing authentication patterns from `docs/PROVIDERS.md`:

**Pattern 1: Keycloak OAuth2** (for user-facing services):
```yaml
environment:
  KEYCLOAK_URL: http://keycloak:8080
  KEYCLOAK_REALM: flexnetos
  KEYCLOAK_CLIENT_ID: my-service
```

**Pattern 2: Vault Secrets** (for service credentials):
```yaml
environment:
  VAULT_ADDR: http://vault:8200
  VAULT_TOKEN: ${VAULT_TOKEN}
```

**Pattern 3: mTLS Certificates** (for service-to-service):
```yaml
volumes:
  - ./data/step-ca/certs:/certs:ro
environment:
  TLS_CERT: /certs/my-service.crt
  TLS_KEY: /certs/my-service.key
```

**Evidence**: `docs/PROVIDERS.md` shows 10+ provider auth patterns

### Step 5: Add Health Check

All services must have health checks for monitoring:

```yaml
services:
  my-new-service:
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s
```

**Evidence**: Existing services in compose files have health checks

### Step 6: Update Documentation

After adding the service, update these files:

```bash
# 1. Add to provider matrix
$EDITOR docs/PROVIDERS.md
# Add row: | My Service | OAuth2 | Keycloak | http://localhost:8080 |

# 2. Add to port registry
$EDITOR docs/PORTS.md
# Add row: | 8080 | my-service | HTTP | Backend API |

# 3. Add to machine-readable catalog
$EDITOR docs/CATALOG.json
# Add service definition in "services" array

# 4. Document API endpoints
$EDITOR docs/API_REFERENCE.md
# Add service API documentation
```

**Evidence**: These files exist and document current providers

### Step 7: Create Script Contract

Document any scripts/commands for the new provider:

```bash
# Create contract document
cat > docs/scripts/my-service-setup.md << 'EOF'
# My Service Setup

## Purpose
Initialize My Service with required configuration

## Invocation
\`\`\`bash
./scripts/my-service-setup.sh [options]
\`\`\`

## Inputs
- Environment: VAULT_TOKEN, MY_SERVICE_CONFIG
- Files: config/my-service.yml

## Outputs
- Exit code: 0=success, 1=warning, 2=error
- Logs: logs/my-service-setup.log

## Safety Classification
- Safety: safe (read-only configuration check)
- Idempotent: yes
- Side effects: None (dry-run by default)

## Dependencies
- curl, jq
- Vault running and unsealed
- Network connectivity to service

## References
- docs/PROVIDERS.md
- docker/docker-compose.*.yml
EOF

# Add to script index
echo "| my-service-setup.sh | Initialize My Service | setup | safe | yes |" >> docs/scripts/INDEX.md
```

**Evidence**: `docs/scripts/INDEX.md` catalogs 60+ script contracts

## Full Example: Adding a New Service

```yaml
# In docker/docker-compose.ai.yml
services:
  embeddings-service:
    image: my-org/embeddings:latest
    container_name: embeddings
    ports:
      - "8085:8080"
    networks:
      - backend
    environment:
      KEYCLOAK_URL: http://keycloak:8080
      VAULT_ADDR: http://vault:8200
    volumes:
      - ./data/embeddings:/data
      - ./data/step-ca/certs:/certs:ro
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3
    depends_on:
      vault:
        condition: service_healthy
      keycloak:
        condition: service_healthy
```

## Validation

After adding the provider, verify integration:

```bash
# 1. Start the service
docker compose -f docker/docker-compose.ai.yml up -d embeddings-service

# 2. Check health
docker compose ps embeddings-service

# 3. Test connectivity
curl http://localhost:8085/health

# 4. Run full E2E validation
./scripts/validate-e2e.sh

# 5. Check for port conflicts
./scripts/validate-resources.sh
```

**Expected**: All validation checks pass (exit code 0)
**Evidence**: `scripts/validate-e2e.sh` tests all services

## Rollback

If the new provider causes issues:

```bash
# Stop the service
docker compose -f docker/docker-compose.*.yml stop my-new-service

# Remove from compose file
git restore docker/docker-compose.*.yml

# Restart stack
docker compose down && docker compose up -d
```

## Common Pitfalls

1. **Port conflicts**: Always check `docs/PORTS.md` first
2. **Network isolation**: Don't expose databases on `frontend` network
3. **Missing health checks**: Services without health checks can't be monitored
4. **Circular dependencies**: Use `depends_on` with `condition: service_healthy`
5. **Documentation lag**: Update all docs in same commit as code changes

## Related docs

- [Providers Matrix](../providers/PROVIDERS.md) - Current provider authentication patterns
- [Port Registry](../api/PORTS.md) - Port assignment tracking
- [Container Security](../security/CONTAINER_SECURITY.md) - Network segmentation rules
- [API Reference](../api/API_REFERENCE.md) - Service API documentation
- [Helm Chart](../HELM_CHART.md) - Kubernetes deployment (if migrating to K8s)
