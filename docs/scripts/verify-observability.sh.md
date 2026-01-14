# Script Contract: verify-observability.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-observability.sh`

---

## Purpose

Verify ARIA Observability Stack (P1-008 Netdata & P1-009 Umami) deployment. Checks Docker services, API endpoints, environment variables, volumes, and provides next steps.

---

## Invocation

```bash
./scripts/verify-observability.sh
```

**Environment Variables:**
- `OBSERVABILITY_REQUIRE_RUNNING=1` - Fail if services not running (default: 0, skip checks)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | All checks passed or services not running (if OBSERVABILITY_REQUIRE_RUNNING=0) |
| `1` | Checks failed or services required but not running |

---

## Side Effects

**Read-only:** No system modifications.

---

## Safety Classification

**🟢 SAFE** - Read-only verification.

---

## Idempotency

**✅ FULLY IDEMPOTENT**

---

## Verification Steps

### 1. Docker Services (3 checks)
- Netdata running
- Umami running
- Umami database running

### 2. Service Endpoints (3 checks)
- Netdata dashboard: http://localhost:19999
- Umami heartbeat: http://localhost:3001/api/heartbeat
- Grafana health: http://localhost:3000/api/health

### 3. Environment Variables (3 checks)
- .env file exists
- Umami variables configured (UMAMI_DB_USER, UMAMI_DB_PASSWORD, UMAMI_APP_SECRET)
- Netdata claim token (optional)

### 4. Docker Volumes (2 checks)
- Netdata volumes created
- Umami volume created

### 5. Service Details
- Displays URLs, container names, images, features

### 6. Next Steps
- Access instructions
- Configuration guidance
- Integration tips

---

## Key Functions

### check_service(service_name, container_name)
**Evidence:** Lines 42-55
- Checks if container is running

### check_endpoint(service_name, url, expected_code)
**Evidence:** Lines 58-72
- HTTP endpoint accessibility check

---

## Skip Logic

**Evidence:** Lines 81-92

If no observability services running and `OBSERVABILITY_REQUIRE_RUNNING=0`:
- Prints warning
- Suggests start command
- Exits with code 0

---

## References

### Source Code
- **Main script:** `scripts/verify-observability.sh` (223 lines)
- **Service checks:** lines 94-127
- **Environment checks:** lines 130-154
- **Volume checks:** lines 157-172
- **Next steps:** lines 193-210

### Related Files
- **Compose file:** `docker/docker-compose.observability.yml`
- **Deployment script:** `scripts/deploy-observability.sh`
- **Documentation:** `docs/P1-008-009-IMPLEMENTATION.md`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 8/60 contracts complete
