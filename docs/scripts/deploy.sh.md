# Script Contract: deploy.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/deploy.sh`

---

## Purpose

Deploy FlexNetOS stack using Docker Compose. Starts core services (Vault, Keycloak, NATS) first with health checks, then starts remaining services in the main compose file.

This is the primary deployment script for the complete FlexNetOS stack.

---

## Invocation

```bash
./scripts/deploy.sh
```

**No arguments supported.**

**Requirements:**
- Docker Compose v2 (`docker compose`) or legacy v1 (`docker-compose`)
- Docker daemon running
- Compose file at `docker/docker-compose.yml` or `docker-compose.yml`

**Examples:**
```bash
# Standard deployment
./scripts/deploy.sh

# Run from any directory
cd ripple-env
./scripts/deploy.sh
```

---

## Inputs

### Arguments
None.

### Environment Variables
None explicitly used. Docker Compose will read environment variables for service configuration.

### Configuration Files
| File | Required | Purpose |
|------|----------|---------|
| `docker/docker-compose.yml` | Yes (preferred) | Main stack composition |
| `docker-compose.yml` | Yes (fallback) | Legacy location for compose file |

**Evidence:** lines 19-34

---

## Outputs

### Files Created
None directly. Docker Compose creates:
- Container volumes (persistent data)
- Container networks
- Container logs

### Standard Output
```
Deploying FlexNetOS...
Starting core services...
[docker compose output]
Waiting for core services to be ready...
keycloak is ready
vault is ready
nats is ready
Starting remaining services...
[docker compose output]
FlexNetOS deployment completed!
Access Grafana: http://localhost:3000
Access Prometheus: http://localhost:9090
Access Kong Gateway: http://localhost:8000
```

**Evidence:** lines 6, 37, 42, 89, 92-96

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all services started |
| `1` | Failure - Docker Compose not found or compose file missing |

**Failure Points:**
- Docker Compose not available (lines 14-17)
- Compose file not found (lines 31-34)
- Service startup failure (implicit via `set -e`)

---

## Side Effects

### System Modifications
1. **Starts Docker containers** for all services defined in compose file
   - Core services: vault, keycloak, nats (line 39)
   - Remaining services: All others (line 90)

2. **Pulls Docker images** (if not cached)
   - Core service images (line 38)
   - Evidence: `pull vault keycloak nats || true`

3. **Creates Docker networks** (if defined in compose file)
   - Implicit in `docker compose up -d`

4. **Creates Docker volumes** (if defined in compose file)
   - Persistent data storage for services

5. **Binds host ports** as defined in compose file
   - Evidence: lines 93-96 (Grafana:3000, Prometheus:9090, Kong:8000)

### Network Activity
- Pulls Docker images from registries
- Services may make outbound connections (e.g., Vault initialization)

### State Changes
- Docker daemon state updated with running containers
- Service data persisted to volumes

---

## Safety Classification

**🟡 CAUTION**

**Rationale:**
- Starts multiple Docker containers with network exposure
- Pulls images from external registries
- May conflict with existing containers on same ports
- No confirmation prompt before deployment
- Uses `|| true` to ignore pull failures (line 38)

**Safe for:**
- Development environments
- Fresh installations
- Systems with no port conflicts

**Unsafe for:**
- Production without review
- Systems with existing services on ports 3000, 8000, 9090
- Environments where network isolation is critical

---

## Idempotency

**✅ IDEMPOTENT**

The script can be re-run safely:

1. **Docker Compose behavior**
   - `docker compose up -d` is idempotent
   - Creates containers if missing, no-op if already running
   - Evidence: lines 39, 90

2. **Image pull handling**
   - `pull ... || true` continues even if pull fails
   - Uses cached images if pull fails
   - Evidence: line 38

3. **Health check retry logic**
   - Checks service health up to 30 attempts
   - Non-blocking warnings if service not ready
   - Evidence: lines 44-81

**Note:** Re-running will restart containers if configuration changed.

---

## Dependencies

### Required Tools
| Tool | Purpose | Check |
|------|---------|-------|
| `docker` | Container runtime | lines 10-11 |
| `docker compose` or `docker-compose` | Orchestration | lines 10-17 |

**Detection Logic:**
```bash
# lines 10-17
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo "Error: Docker Compose not found"
    exit 1
fi
```

### Required Services
| Service | Stage | Purpose |
|---------|-------|---------|
| Docker daemon | All | Container runtime |

### Required Files
| File | Evidence |
|------|----------|
| `docker/docker-compose.yml` or `docker-compose.yml` | lines 29-34 |

---

## Failure Modes

### Docker Compose Not Found
**Symptom:** Neither `docker compose` nor `docker-compose` available
**Exit Code:** 1
**Recovery:** Install Docker with Compose plugin
**Evidence:** lines 14-17

**Example:**
```
Error: Docker Compose not found (docker compose / docker-compose)
```

### Compose File Missing
**Symptom:** No compose file at expected locations
**Exit Code:** 1
**Recovery:** Verify working directory, check file exists
**Evidence:** lines 31-34

**Example:**
```
Error: stack compose file not found (expected docker/docker-compose.yml or docker-compose.yml)
```

### Service Startup Failure
**Symptom:** Container fails to start, `docker compose up -d` fails
**Exit Code:** 1 (via `set -e`)
**Recovery:** Check Docker logs, verify configuration
**Evidence:** line 4 (`set -euo pipefail`)

### Health Check Timeout
**Symptom:** Service not ready after 30 attempts (60 seconds)
**Exit Code:** 0 (non-fatal)
**Recovery:** Check service logs, verify health check configuration
**Evidence:** lines 47-80

**Example:**
```
Warning: vault may not be fully ready
```

### Image Pull Failure
**Symptom:** Cannot pull Docker image
**Exit Code:** 0 (ignored via `|| true`)
**Recovery:** Uses cached images if available
**Evidence:** line 38

---

## Health Check Logic

### Function: check_service_health
**Evidence:** lines 45-81

**Process:**
1. Get container ID for service (line 53)
2. Check for explicit health status (lines 57-64)
3. If no health check, check container running state (lines 66-73)
4. Retry up to 30 times with 2-second intervals (lines 47, 77)
5. Return warning if not ready after 60 seconds (line 79)

**Health Check Logic:**
```bash
# lines 57-64
if [ -n "$health_status" ]; then
    if [ "$health_status" = "healthy" ]; then
        echo "$service is ready"
        return 0
    fi
else
    # No health check defined; fall back to container state
    if [ "$container_state" = "running" ]; then
        echo "$service is ready"
        return 0
    fi
fi
```

**Timeout:** 60 seconds (30 attempts × 2 seconds)

**Services Checked:**
- keycloak (line 84)
- vault (line 85)
- nats (line 86)

**Non-blocking:** Health check failures return warnings, not errors (lines 84-86 use `|| true`)

---

## Deployment Flow

### Stage 1: Core Services
**Services:** vault, keycloak, nats
**Evidence:** lines 37-39

```bash
echo "Starting core services..."
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" pull vault keycloak nats || true
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" up -d vault keycloak nats
```

**Rationale:** These services must be ready before dependent services start.

### Stage 2: Health Check Wait
**Evidence:** lines 42-86

Waits for core services to reach healthy/running state with retry logic.

### Stage 3: Remaining Services
**Evidence:** lines 89-90

```bash
echo "Starting remaining services..."
"${COMPOSE[@]}" -f "$STACK_COMPOSE_FILE" up -d
```

**Note:** Starts all services, including already-running core services (no-op for those).

---

## Compose File Resolution

### Function: resolve_compose_file
**Evidence:** lines 19-27

**Logic:**
1. Check if file exists in `docker/` subdirectory (line 22)
2. Return `docker/<filename>` if exists
3. Otherwise return `<filename>` (current directory)

**Usage:**
```bash
# line 29
STACK_COMPOSE_FILE="$(resolve_compose_file docker-compose.yml)"
```

**Search Order:**
1. `docker/docker-compose.yml` (preferred)
2. `docker-compose.yml` (fallback)

---

## Service Endpoints

The script displays access URLs after deployment:

| Service | URL | Evidence |
|---------|-----|----------|
| Grafana | http://localhost:3000 | line 93 |
| Prometheus | http://localhost:9090 | line 94 |
| Kong Gateway | http://localhost:8000 | line 95 |

**Note:** Actual ports depend on compose file configuration. These are default/common values.

---

## Docker Compose Version Support

### Compose v2 (Preferred)
**Command:** `docker compose`
**Check:** `docker compose version`
**Evidence:** lines 10-11

### Compose v1 (Legacy)
**Command:** `docker-compose`
**Check:** `command -v docker-compose`
**Evidence:** lines 12-13

**Preference Logic:**
1. Try v2 (`docker compose`) first
2. Fall back to v1 (`docker-compose`)
3. Error if neither available

---

## Error Handling

### Exit on Error
**Evidence:** line 4
```bash
set -euo pipefail
```

**Behavior:**
- `-e`: Exit immediately if any command fails
- `-u`: Error on undefined variables
- `-o pipefail`: Pipeline fails if any command fails

**Exceptions:**
- Image pull failures ignored (line 38: `|| true`)
- Health check failures ignored (lines 84-86: `|| true`)

### Non-fatal Warnings
Health checks use `|| true` to prevent deployment failure if services are slow to start:
```bash
# lines 84-86
check_service_health "keycloak" || true
check_service_health "vault" || true
check_service_health "nats" || true
```

---

## References

### Source Code
- **Main script:** `scripts/deploy.sh` (96 lines)
- **Compose detection:** lines 9-17
- **File resolution:** lines 19-34
- **Core service deployment:** lines 36-39
- **Health check logic:** lines 44-86
- **Full deployment:** lines 88-90

### Related Files
- **Compose file:** `docker/docker-compose.yml` (required)
- **Specialized deployments:** `scripts/deploy-*.sh` (edge, observability, automation, etc.)

### Related Documentation
- [docs/modules/bootstrap.md](../modules/bootstrap.md) - Golden Path GP-4 (Deploy & Verify)
- [docker/README.md](../../docker/README.md) - Docker compose stack documentation (if exists)

### External Resources
- [Docker Compose Documentation](https://docs.docker.com/compose/)
- [Docker Health Checks](https://docs.docker.com/engine/reference/builder/#healthcheck)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 3/60 contracts complete
