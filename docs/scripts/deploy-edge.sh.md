# Script Contract: deploy-edge.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/deploy-edge.sh`

---

## Purpose

Deploy ARIA Edge Services stack using Docker Compose. Deploys P0-003 (Kong Gateway) and P0-004 (AgentGateway) with health checks, network setup, and configuration directory creation.

This is the specialized deployment script for edge gateway services.

---

## Invocation

```bash
./scripts/deploy-edge.sh
```

**No arguments supported.**

**Requirements:**
- Docker with Compose v2 plugin
- Compose file at `docker/docker-compose.edge.yml` or `docker-compose.edge.yml`
- Docker daemon running

**Examples:**
```bash
# Standard deployment
./scripts/deploy-edge.sh

# Run from project root
cd ripple-env
./scripts/deploy-edge.sh
```

---

## Inputs

### Arguments
None.

### Environment Variables
None explicitly used. Docker Compose reads environment for service configuration.

### Configuration Files
| File | Required | Purpose |
|------|----------|---------|
| `docker/docker-compose.edge.yml` | Yes (preferred) | Edge stack composition |
| `docker-compose.edge.yml` | Yes (fallback) | Legacy compose file location |
| `config/agentgateway/config.yaml` | No (optional) | AgentGateway configuration |

**Evidence:** lines 19-23, 93-97

---

## Outputs

### Files Created
| File/Directory | Purpose |
|----------------|---------|
| `config/agentgateway/` | Configuration directory for AgentGateway |

**Evidence:** lines 90-91

### Standard Output
- Color-coded progress messages
- Prerequisites check results
- Network creation status
- Service deployment progress
- Health check status
- Service status table
- Access URLs and next steps

**Evidence:** lines 26-46 (print functions), 184-238 (access info and summary)

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all services healthy |
| `1` | Failure - prerequisites failed or health check timeout |

**Failure Points:**
- Docker not installed (lines 53-56)
- Docker Compose not available (lines 60-63)
- Compose file not found (lines 67-70)
- Kong database health timeout (lines 135-138)
- Kong gateway health timeout (lines 151-154)
- AgentGateway health timeout (lines 167-170)

---

## Side Effects

### System Modifications
1. **Creates Docker network** (if not exists)
   - Network name: `agentic-network`
   - Evidence: lines 75-84

2. **Creates configuration directories**
   - `config/agentgateway/`
   - Evidence: lines 90-91

3. **Starts Docker containers**
   - Kong database (PostgreSQL)
   - Kong gateway
   - AgentGateway
   - Konga admin UI
   - Evidence: lines 101-116

4. **Pulls Docker images** from registries
   - Evidence: lines 108-109

5. **Binds host ports**
   - Kong Admin API: 8001
   - Kong Proxy: 8000
   - Kong Manager UI: 8002
   - AgentGateway Main API: 8090
   - AgentGateway Admin API: 8091
   - AgentGateway Metrics: 8092
   - Konga Web UI: 1337
   - Evidence: lines 187-198

### Network Activity
- Pulls Docker images from registries
- Creates network connections between containers

### State Changes
- Docker daemon state updated with running containers
- Network `agentic-network` created (persistent)
- Configuration directory created (persistent)

---

## Safety Classification

**🟡 CAUTION**

**Rationale:**
- Starts multiple containers with network exposure
- Creates shared Docker network for inter-service communication
- Binds multiple ports (8000-8002, 8090-8092, 1337)
- No confirmation prompt before deployment
- Health check failures cause script exit (hard failure)

**Safe for:**
- Development environments
- Fresh installations
- Systems with no port conflicts

**Unsafe for:**
- Production without review
- Systems with existing Kong/gateway services
- Environments where port conflicts exist

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**

**Idempotent operations:**
1. **Network creation** - Checks if network exists before creating (lines 78-83)
2. **Directory creation** - `mkdir -p` is idempotent (line 90)
3. **Docker Compose up -d** - Recreates containers if configuration changed

**Non-idempotent operations:**
1. **Health check logic** - Will fail if services already running but unhealthy
2. **No cleanup** - Doesn't remove existing containers before deployment

**Re-run behavior:**
- May restart containers if configuration changed
- Will fail if services exist but are unhealthy

---

## Dependencies

### Required Tools
| Tool | Purpose | Check |
|------|---------|-------|
| `docker` | Container runtime | lines 53-56 |
| `docker compose` | Orchestration | lines 60-63 |

### Required Services
| Service | Stage | Purpose |
|---------|-------|---------|
| Docker daemon | All | Container runtime |

### Required Files
| File | Evidence |
|------|----------|
| `docker/docker-compose.edge.yml` or `docker-compose.edge.yml` | lines 19-23, 67-70 |

---

## Failure Modes

### Docker Not Installed
**Symptom:** `docker` command not found
**Exit Code:** 1
**Recovery:** Install Docker
**Evidence:** lines 53-56

**Example:**
```
[ERROR] Docker is not installed
```

### Docker Compose Not Available
**Symptom:** `docker compose version` fails
**Exit Code:** 1
**Recovery:** Install Docker Compose v2 plugin
**Evidence:** lines 60-63

**Example:**
```
[ERROR] Docker Compose is not available
```

### Compose File Not Found
**Symptom:** Edge compose file missing at expected locations
**Exit Code:** 1
**Recovery:** Verify file exists, check working directory
**Evidence:** lines 67-70

**Example:**
```
[ERROR] Edge compose file not found at docker/docker-compose.edge.yml
```

### Kong Database Health Timeout
**Symptom:** Kong database not healthy after 120 seconds
**Exit Code:** 1
**Recovery:** Check Docker logs, verify database configuration
**Evidence:** lines 125-138

**Example:**
```
[ERROR] Timeout waiting for Kong database
```

### Kong Gateway Health Timeout
**Symptom:** Kong gateway not healthy after 120 seconds
**Exit Code:** 1
**Recovery:** Check Kong logs, verify database connectivity
**Evidence:** lines 140-154

**Example:**
```
[ERROR] Timeout waiting for Kong gateway
```

### AgentGateway Health Timeout
**Symptom:** AgentGateway not healthy after 120 seconds
**Exit Code:** 1
**Recovery:** Check AgentGateway logs, verify configuration
**Evidence:** lines 156-170

**Example:**
```
[ERROR] Timeout waiting for AgentGateway
```

---

## Health Check Logic

### Function: wait_for_services
**Evidence:** lines 119-173

**Timeout:** 120 seconds per service

**Services Checked:**
1. **kong-database** (lines 125-138)
2. **kong** (lines 140-154)
3. **agentgateway** (lines 156-170)

**Check Logic:**
```bash
# lines 127-130
if docker inspect --format='{{.State.Health.Status}}' <container> 2>/dev/null | grep -q "healthy"; then
    print_success "<Service> is healthy"
    break
fi
```

**Process:**
1. Check container health status every 2 seconds
2. Timeout after 120 seconds (60 attempts)
3. Return error code 1 on timeout (hard failure)

**Note:** Unlike deploy.sh, health check failures are fatal (no `|| true`).

---

## Deployment Flow

### Stage 1: Prerequisites Check
**Function:** `check_prerequisites` (lines 49-72)
**Checks:**
- Docker installed
- Docker Compose available
- Compose file exists

### Stage 2: Network Creation
**Function:** `create_network` (lines 75-84)
**Creates:** `agentic-network` Docker network (bridge mode)

### Stage 3: Configuration Setup
**Function:** `create_config` (lines 87-98)
**Creates:** `config/agentgateway/` directory
**Checks:** Optional `config.yaml` presence

### Stage 4: Service Deployment
**Function:** `deploy_services` (lines 101-116)
**Process:**
1. Pull images (`docker compose pull`)
2. Start services (`docker compose up -d`)

### Stage 5: Health Check Wait
**Function:** `wait_for_services` (lines 119-173)
**Initial delay:** 10 seconds (line 221-222)
**Checks:** All three services sequentially

### Stage 6: Status Display
**Function:** `show_status` (lines 176-181)
**Shows:** Container status table

### Stage 7: Access Information
**Function:** `show_access_info` (lines 184-206)
**Shows:** Service URLs, quick tests, verification command

---

## Service Endpoints

| Service | Endpoint | Port | Purpose |
|---------|----------|------|---------|
| Kong Admin API | http://localhost:8001 | 8001 | Gateway administration |
| Kong Proxy | http://localhost:8000 | 8000 | API proxy |
| Kong Manager UI | http://localhost:8002 | 8002 | Web management interface |
| AgentGateway Main | http://localhost:8090 | 8090 | Main API endpoint |
| AgentGateway Admin | http://localhost:8091 | 8091 | Admin operations |
| AgentGateway Metrics | http://localhost:8092/metrics | 8092 | Prometheus metrics |
| Konga Web UI | http://localhost:1337 | 1337 | Kong admin interface |

**Evidence:** lines 187-198

---

## Quick Tests

The script provides quick test commands:

```bash
# Kong status
curl http://localhost:8001/status

# AgentGateway health
curl http://localhost:8090/health
```

**Evidence:** lines 200-202

---

## Next Steps

Displayed at completion (lines 231-238):

1. Run verification: `scripts/verify-edge.sh`
2. Configure Kong routes for services
3. Set up AgentGateway routing rules
4. Access Konga UI at http://localhost:1337

**Management commands:**
- Stop: `docker compose -f docker/docker-compose.edge.yml down`
- Logs: `docker compose -f docker/docker-compose.edge.yml logs -f`

**Evidence:** lines 237-238

---

## Color Output Functions

| Function | Color | Purpose | Evidence |
|----------|-------|---------|----------|
| `print_header` | Blue | Section headers | lines 26-30 |
| `print_success` | Green | Success messages | lines 32-34 |
| `print_failure` | Red | Error messages | lines 36-38 |
| `print_info` | Blue | Information | lines 40-42 |
| `print_warning` | Yellow | Warnings | lines 44-46 |

**ANSI Color Codes:** lines 10-14

---

## Error Handling

### Exit on Error
**Evidence:** line 7
```bash
set -e
```

**Behavior:**
- Exit immediately if any command fails
- Health check timeouts return error code 1
- Script exits on first failure

**No Exceptions:** All failures are fatal (unlike deploy.sh which uses `|| true`).

---

## ARIA Priority References

The script deploys two ARIA P0 (Priority 0 - Critical) components:

**P0-003: Kong Gateway**
- Industry-standard API gateway
- Plugin ecosystem for auth, rate limiting, logging
- Evidence: lines 3, 211

**P0-004: AgentGateway**
- Custom agent traffic routing
- Admin API and metrics endpoints
- Evidence: lines 3, 212

---

## References

### Source Code
- **Main script:** `scripts/deploy-edge.sh` (243 lines)
- **Prerequisites check:** lines 49-72
- **Network creation:** lines 75-84
- **Configuration setup:** lines 87-98
- **Service deployment:** lines 101-116
- **Health checks:** lines 119-173
- **Access information:** lines 184-206

### Related Files
- **Compose file:** `docker/docker-compose.edge.yml` (required)
- **Verification script:** `scripts/verify-edge.sh` (referenced)
- **AgentGateway config:** `config/agentgateway/config.yaml` (optional)

### Related Documentation
- [docs/modules/bootstrap.md](../modules/bootstrap.md) - Golden Path GP-4 (Deploy & Verify)
- ARIA_MANIFEST.yaml - P0-003, P0-004 component definitions

### External Resources
- [Kong Gateway Documentation](https://docs.konghq.com/)
- [Konga Documentation](https://github.com/pantsel/konga)
- [Docker Compose Documentation](https://docs.docker.com/compose/)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 4/60 contracts complete
