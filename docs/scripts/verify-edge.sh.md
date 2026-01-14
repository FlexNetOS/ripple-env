# Script Contract: verify-edge.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-edge.sh`

---

## Purpose

Verify ARIA Edge Services (P0-003 Kong Gateway & P0-004 AgentGateway) deployment. Performs comprehensive testing of container status, health checks, API endpoints, integration points, feature flags, and service logs.

---

## Invocation

```bash
./scripts/verify-edge.sh
```

**Environment Variables:**
- `EDGE_REQUIRE_RUNNING=1` - Fail if services not running (default: 0, skip checks)
- `KONG_ADMIN_URL` - Kong admin API URL (default: http://localhost:8001)
- `KONG_PROXY_URL` - Kong proxy URL (default: http://localhost:8000)
- `KONG_MANAGER_URL` - Kong manager URL (default: http://localhost:8002)
- `AGENTGATEWAY_API_URL` - AgentGateway main API (default: http://localhost:8090)
- `AGENTGATEWAY_ADMIN_URL` - AgentGateway admin API (default: http://localhost:8091)
- `AGENTGATEWAY_METRICS_URL` - AgentGateway metrics (default: http://localhost:8092)
- `KONGA_URL` - Konga UI (default: http://localhost:1337)

---

## Outputs

### Standard Output
- Test results with pass/fail counters
- Service status verification
- API endpoint checks
- Integration test results
- Feature flag status
- Log error summaries
- Access URLs and troubleshooting tips

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | All tests passed or services not running (if EDGE_REQUIRE_RUNNING=0) |
| `1` | Tests failed or services required but not running |

---

## Side Effects

**Read-only:** No system modifications. Only performs checks via:
- Docker inspection commands
- HTTP GET requests (curl)
- Log reading

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no modifications.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly without side effects.

---

## Test Categories

### 1. Prerequisites (3 tests)
- Docker network `agentic-network` exists
- Compose file `docker-compose.edge.yml` valid
- AgentGateway config directory exists

### 2. Kong Gateway (6 tests)
- Kong database service running and healthy
- Kong migrations completed
- Kong gateway running and healthy
- Kong Admin API accessible
- Kong Proxy accessible
- Kong Manager UI accessible
- Konga UI accessible (optional)

### 3. AgentGateway (4 tests)
- AgentGateway service running and healthy
- Main API accessible
- Admin API accessible
- Metrics endpoint accessible

### 4. Integration (2 tests)
- AgentGateway -> Kong connectivity
- MCP protocol support enabled

### 5. Feature Flags (2 tests)
- KONG_ENABLED flag
- AGENTGATEWAY_ENABLED flag

### 6. Log Analysis (2 tests)
- Recent errors in Kong logs
- Recent errors in AgentGateway logs

**Total:** 19 tests

**Evidence:** Lines 42-44 (counters), 72-73 (run_test function)

---

## Key Functions

### check_url(url, expected_code, timeout)
**Evidence:** Lines 76-87
- Validates HTTP endpoint accessibility
- Supports regex for expected codes (e.g., "200|404")
- Default timeout: 5 seconds

### check_service(service_name)
**Evidence:** Lines 90-98
- Checks if Docker container is running

### check_service_health(service_name)
**Evidence:** Lines 101-116
- Checks Docker health status
- Falls back to running status if no health check defined

---

## Skip Logic

**Evidence:** Lines 167-176

If no edge services running and `EDGE_REQUIRE_RUNNING=0`:
- Prints info message
- Suggests start command
- Exits with code 0 (success)

---

## References

### Source Code
- **Main script:** `scripts/verify-edge.sh` (404 lines)
- **Test framework:** lines 42-74
- **Prerequisites:** lines 127-164
- **Kong tests:** lines 179-259
- **AgentGateway tests:** lines 262-302
- **Integration tests:** lines 305-325
- **Feature flags:** lines 328-346
- **Log analysis:** lines 349-369

### Related Files
- **Compose file:** `docker/docker-compose.edge.yml`
- **Deployment script:** `scripts/deploy-edge.sh`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 7/60 contracts complete
