# Script Contract: deploy-observability.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/deploy-observability.sh`

---

## Purpose

Deploy ARIA Observability Stack (P1-008 Netdata & P1-009 Umami) using Docker Compose. Handles environment file creation, secret generation, network setup, image pulling, and service deployment with verification.

This is the specialized deployment script for observability monitoring services.

---

## Invocation

```bash
./scripts/deploy-observability.sh
```

**No arguments supported.**

**Requirements:**
- Docker with Compose v2 plugin or docker-compose v1
- Compose file at `docker/docker-compose.observability.yml` or `docker-compose.observability.yml`
- openssl or python for secret generation

---

## Inputs

### Environment Variables
None explicitly required. Creates/updates `.env` file with generated secrets.

### Configuration Files
| File | Required | Purpose |
|------|----------|---------|
| `docker/docker-compose.observability.yml` | Yes (preferred) | Observability stack composition |
| `docker-compose.observability.yml` | Yes (fallback) | Legacy compose file location |
| `.env.example` | No (optional) | Template for .env creation |
| `.env` | No (created) | Environment variables for services |

---

## Outputs

### Files Created/Modified
| File | Purpose |
|------|---------|
| `.env` | Created from .env.example if missing, secrets updated |
| `.env` (permissions) | Set to 600 (read/write owner only) |

### Standard Output
Multi-stage deployment progress with color-coded steps:
- Environment configuration
- Security configuration
- Docker network setup
- Image pulling
- Service deployment
- Service verification
- Access information and next steps

**Evidence:** lines 18-211

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - services deployed |
| `1` | Failure - Docker not available, Compose missing, or secret generation failed |

---

## Side Effects

### System Modifications
1. **Creates/modifies .env file** with generated secrets
   - `UMAMI_APP_SECRET` (32-byte base64)
   - `UMAMI_DB_PASSWORD` (32-byte base64)
   - Evidence: lines 58-112

2. **Sets .env file permissions** to 600
   - Evidence: lines 115-116

3. **Creates Docker network** `agentic-network` (if not exists)
   - Evidence: lines 122-127

4. **Pulls Docker images**
   - netdata
   - umami, umami-db
   - Evidence: lines 134-139

5. **Starts Docker containers**
   - netdata (port 19999)
   - umami (port 3001)
   - umami-db (PostgreSQL)
   - Evidence: line 146

6. **Binds host ports:**
   - Netdata: 19999
   - Umami: 3001
   - Grafana: 3000 (referenced)
   - Evidence: lines 192-194

### Network Activity
- Pulls Docker images from registries
- Creates network connections between containers

---

## Safety Classification

**🟡 CAUTION**

**Rationale:**
- Automatically generates and updates secrets in .env file
- Modifies .env file permissions without confirmation
- Uses default credentials for initial setup (insecure until changed)
- Creates/modifies configuration files
- Starts containers with network exposure

**Safe for:**
- Development environments
- Initial setup on fresh systems

**Unsafe for:**
- Production (requires manual credential changes)
- Systems with existing .env configurations (will overwrite secrets)

---

## Idempotency

**✅ MOSTLY IDEMPOTENT**

**Idempotent operations:**
1. **Docker Compose up -d** - Recreates if needed
2. **Network creation** - Checks existence first (line 122)
3. **.env file check** - Only creates if missing (line 58)

**Non-idempotent operations:**
1. **Secret generation** - Regenerates if secrets are default "changeme" (lines 77-109)
2. **.env permission changes** - Always set to 600 (line 115)

**Re-run behavior:**
- Safe to re-run, will regenerate default secrets
- Preserves custom secrets if not "changeme"

---

## Dependencies

### Required Tools
| Tool | Purpose | Check | Evidence |
|------|---------|-------|----------|
| `docker` | Container runtime | lines 24-27 | Required |
| `docker compose` or `docker-compose` | Orchestration | lines 31-38 | Required |
| `openssl` or `python3`/`python` | Secret generation | lines 82-90 | Required for secrets |

**Secret Generation Fallback:** Tries openssl → python3 → python (lines 82-90)

### Required Files
| File | Evidence |
|------|----------|
| `docker/docker-compose.observability.yml` or `docker-compose.observability.yml` | lines 40-50 |

---

## Failure Modes

### Docker Not Available
**Symptom:** `docker` command not found
**Exit Code:** 1
**Recovery:** Install Docker
**Evidence:** lines 24-27

### Docker Compose Not Available
**Symptom:** Neither compose variant available
**Exit Code:** 1
**Recovery:** Install Docker Compose
**Evidence:** lines 36-38

### Secret Generation Failure
**Symptom:** No openssl or python available
**Exit Code:** 1
**Recovery:** Install openssl or python
**Evidence:** lines 93-94

---

## Deployment Flow

### Step 1: Environment Configuration
**Evidence:** lines 56-70

- Check for .env file
- Create from .env.example if missing
- Warn about default credentials

### Step 2: Security Configuration
**Evidence:** lines 73-117

**Secret Generation Function:** lines 81-91
```bash
gen_secret() {
    if command -v openssl >/dev/null 2>&1; then
        openssl rand -base64 32
    elif command -v python3 >/dev/null 2>&1; then
        python3 -c "import secrets,base64; print(base64.b64encode(secrets.token_bytes(32)).decode())"
    elif command -v python >/dev/null 2>&1; then
        python -c "import secrets,base64; print(base64.b64encode(secrets.token_bytes(32)).decode())"
    else
        return 1
    fi
}
```

**Generates:**
- `UMAMI_APP_SECRET` (lines 93-101)
- `UMAMI_DB_PASSWORD` (lines 94, 103-107)

**Updates .env** using sed (lines 98-107)

**Sets permissions** to 600 (lines 115-116)

### Step 3: Docker Network
**Evidence:** lines 120-128

Creates `agentic-network` if not exists.

### Step 4: Pull Images
**Evidence:** lines 131-140

Pulls netdata and umami/umami-db images.

### Step 5: Deploy Services
**Evidence:** lines 143-149

Starts all services with `docker compose up -d`.

### Step 6: Wait for Services
**Evidence:** lines 152-155

Fixed 30-second wait for initialization.

### Step 7: Verification
**Evidence:** lines 158-185

Checks if containers are running:
- netdata (lines 163-168)
- umami (lines 171-176)
- umami-db (lines 179-184)

---

## Service Endpoints

| Service | URL | Port | Priority |
|---------|-----|------|----------|
| Netdata | http://localhost:19999 | 19999 | P1-008 |
| Umami | http://localhost:3001 | 3001 | P1-009 |
| Grafana | http://localhost:3000 | 3000 | Referenced |

**Evidence:** lines 192-194

---

## Default Credentials

**WARNING:** Must be changed immediately after deployment.

| Service | Username | Password |
|---------|----------|----------|
| Umami | admin | umami |
| Grafana | admin | admin |

**Evidence:** lines 197-198

---

## Next Steps

Displayed at completion (lines 200-209):

1. Access Netdata: http://localhost:19999
2. Login to Umami: http://localhost:3001
3. Change default passwords
4. Run verification: `./scripts/verify-observability.sh`

**Documentation references:**
- Implementation Report: `docs/P1-008-009-IMPLEMENTATION.md`
- Quick Start Guide: `docs/OBSERVABILITY-QUICK-START.md`
- Completion Summary: `P1-008-009-COMPLETION-SUMMARY.md`

---

## Secret Management

### Secret Detection Logic
**Evidence:** lines 77-78

```bash
if grep -q "UMAMI_APP_SECRET=changeme" .env 2>/dev/null || \
   ! grep -q "UMAMI_APP_SECRET" .env 2>/dev/null; then
```

**Triggers regeneration if:**
- Secret is set to "changeme"
- Secret variable not present in .env

### Secret Update Logic
**Evidence:** lines 97-107

```bash
if grep -q "UMAMI_APP_SECRET" .env; then
    sed -i "s|UMAMI_APP_SECRET=.*|UMAMI_APP_SECRET=${UMAMI_APP_SECRET}|" .env
else
    echo "UMAMI_APP_SECRET=${UMAMI_APP_SECRET}" >> .env
fi
```

**Behavior:**
- If variable exists: Replace with sed
- If variable missing: Append to file

---

## Compose File Resolution

**Function:** `resolve_compose_file` (lines 40-48)

**Search Order:**
1. `docker/docker-compose.observability.yml` (preferred)
2. `docker-compose.observability.yml` (fallback)

Same logic as deploy.sh.

---

## Error Handling

### Exit on Error
**Evidence:** line 10
```bash
set -e
```

**Behavior:**
- Exit immediately on any command failure
- Secret generation failure is fatal
- Docker/Compose unavailability is fatal

---

## ARIA Priority References

**P1-008: Netdata**
- Real-time system monitoring
- Agent-based metrics collection
- Evidence: lines 3, 19

**P1-009: Umami**
- Privacy-focused web analytics
- PostgreSQL-backed
- Evidence: lines 3, 19

---

## References

### Source Code
- **Main script:** `scripts/deploy-observability.sh` (212 lines)
- **Secret generation:** lines 81-91
- **Environment setup:** lines 56-117
- **Deployment flow:** lines 131-155
- **Verification:** lines 158-185

### Related Files
- **Compose file:** `docker/docker-compose.observability.yml`
- **Verification script:** `scripts/verify-observability.sh`
- **Documentation:** `docs/P1-008-009-IMPLEMENTATION.md`

### External Resources
- [Netdata Documentation](https://learn.netdata.cloud/)
- [Umami Documentation](https://umami.is/docs)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 5/60 contracts complete
