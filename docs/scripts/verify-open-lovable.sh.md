# Script Contract: verify-open-lovable.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-open-lovable.sh`

---

## Purpose

Verify Open-Lovable integration (P2-012, ARIA UI Stack). Checks Dockerfile, Docker Compose configuration, environment variables, network setup, and performs optional runtime tests if service is running. Open-Lovable is a self-hosted AI code generation UI.

---

## Invocation

```bash
./scripts/verify-open-lovable.sh
```

**No arguments or environment variables required.**

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (0 failures) |
| `1` | Failed (1+ failures) |

### Counters
- Passed (green)
- Failed (red)
- Warnings (yellow)

---

## Side Effects

**Minimal:** Read-only verification, no persistent changes.

**Docker Checks:** Non-destructive container and network inspection if Docker available.

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Verification Tests (6)

### Test 1: File Existence (lines 82-110)
- **Dockerfile:** `config/dockerfiles/Dockerfile.open-lovable`
- **Compose file:** `docker-compose.ui.yml` (or `docker/docker-compose.ui.yml`)
- **.env.example:** Environment variable template
- **Documentation:** `docs/P2-012-OPEN-LOVABLE-INTEGRATION.md`

**Evidence:** Lines 85-109.

### Test 2: Configuration Validation (lines 115-169)
- **open-lovable service** defined in compose file
- **Dockerfile build** configuration (line 125)
- **Port mapping:** 3211:3000 (line 132)
- **LocalAI dependency** - either `depends_on` or `OPENAI_BASE_URL` pointing to `localai:8080` (lines 139-152)
- **Volumes:** open-lovable-data, open-lovable-projects (lines 155-160)
- **Environment variables:** OPEN_LOVABLE_VERSION, OPENAI_API_KEY, OPENAI_BASE_URL (lines 163-168)

**Evidence:** Lines 118-168.

**LocalAI Integration:** Accepts either explicit `depends_on: localai` or environment variable `OPENAI_BASE_URL=http://localai:8080` for external service pattern (lines 142-148).

### Test 3: Docker Network (lines 174-181)
- **agentic-network** existence check
- Warning if missing with creation command

**Evidence:** Lines 176-180.

### Test 4: Docker Compose Validation (lines 186-196)
- Syntax validation via `docker compose config`
- Shows errors if validation fails

**Evidence:** Lines 188-195.

### Test 5: Runtime Tests (Optional) (lines 201-256)
**Only runs if open-lovable container exists:**

- **Container status** (running/stopped)
- **Health status** - healthy/starting/unhealthy (lines 211-221)
- **Port accessibility** - HTTP 200/301/302 on localhost:3211 (lines 223-233)
- **Log errors** - Counts error messages in logs (lines 235-241)
- **LocalAI status** - Checks if LocalAI container running (lines 250-256)

**Evidence:** Lines 203-256.

### Test 6: Volume Inspection (lines 261-270)
- Counts open-lovable volumes
- Lists volume names and drivers
- Info message if none found (normal for new installation)

**Evidence:** Lines 263-270.

---

## Compose File Resolution

**Evidence:** Lines 59-67

**Pattern:** Check subdirectory first, fallback to root
```bash
UI_COMPOSE_FILE="docker-compose.ui.yml"
if [ -f "docker/docker-compose.ui.yml" ]; then
    UI_COMPOSE_FILE="docker/docker-compose.ui.yml"
fi
```

**Also checks:** `docker-compose.localai.yml` with same pattern (lines 64-67).

---

## Docker Compose Command Resolution

**Evidence:** Lines 70-75

**Prefers v2 plugin:**
```bash
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi
```

**Usage:** `"${COMPOSE[@]}" -f "$UI_COMPOSE_FILE" config` (lines 189-195).

---

## Key Features

### 1. LocalAI Dependency Detection

**Evidence:** Lines 139-152

**Two patterns supported:**

**Pattern A - Explicit dependency:**
```yaml
services:
  open-lovable:
    depends_on:
      - localai
```

**Pattern B - External service:**
```yaml
services:
  open-lovable:
    environment:
      OPENAI_BASE_URL: http://localai:8080
      # Or OPENAI_PROXY_URL: http://localai:8080
```

**Rationale:** LocalAI may run in separate compose stack on shared network (lines 140-141).

### 2. Health Status Interpretation

**Evidence:** Lines 211-221

```bash
HEALTH_STATUS=$(docker inspect --format='{{.State.Health.Status}}' open-lovable)
```

**Status values:**
- `healthy` - Pass
- `starting` - Warning (still initializing)
- `unknown` - Warning (no healthcheck defined)
- Other - Fail (unhealthy)

### 3. Port Accessibility Check

**Evidence:** Lines 223-233

**Accepts multiple HTTP codes:**
- 200 - OK
- 301 - Redirect
- 302 - Temporary redirect

**Rationale:** Application may redirect to login or setup page.

### 4. Log Error Detection

**Evidence:** Lines 235-241

```bash
ERROR_COUNT=$(docker logs open-lovable 2>&1 | grep -ci "error")
```

**Case-insensitive search** for error messages. Warnings if any found (not failures).

---

## Next Steps

**Displayed on success (lines 286-290):**
```bash
# 1. Ensure LocalAI is running
docker compose -f docker-compose.localai.yml up -d

# 2. Build and start open-lovable
docker compose -f docker-compose.ui.yml up -d --build open-lovable

# 3. Access the application
http://localhost:3211

# 4. Check logs
docker compose -f docker-compose.ui.yml logs -f open-lovable
```

---

## Open-Lovable Context

**P2-012: ARIA UI Stack**
- **Purpose:** Self-hosted AI code generation and chat interface
- **Backend:** LocalAI (replaces OpenAI API)
- **Port:** 3211 (external) → 3000 (internal)
- **Volumes:** Persistent data and projects
- **Network:** agentic-network (shared with LocalAI)

**Integration Points:**
- LocalAI for LLM inference
- ARIA platform for agentic workflows
- Optional NATS integration for events

---

## References

### Source Code
- **Main script:** `scripts/verify-open-lovable.sh` (298 lines)
- **File checks:** lines 82-110
- **Config validation:** lines 115-169
- **Runtime tests:** lines 201-256
- **Volume inspection:** lines 261-270

### Related Files
- **Dockerfile:** `config/dockerfiles/Dockerfile.open-lovable`
- **Compose file:** `docker-compose.ui.yml` (or `docker/docker-compose.ui.yml`)
- **LocalAI compose:** `docker-compose.localai.yml`
- **Environment:** `.env.example`
- **Documentation:** `docs/P2-012-OPEN-LOVABLE-INTEGRATION.md`

### External Resources
- [Open-Lovable GitHub](https://github.com/open-lovable/open-lovable) (inferred)
- [LocalAI](https://localai.io/) - Self-hosted AI inference

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 16/60 contracts complete
