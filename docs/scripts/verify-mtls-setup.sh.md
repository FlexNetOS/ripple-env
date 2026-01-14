# Script Contract: verify-mtls-setup.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-mtls-setup.sh`

---

## Purpose

Verify ARIA mTLS (mutual TLS) setup completeness. Checks step-cli installation, Step-CA configuration files, PKI certificates, secrets, running services, documentation, service certificate directories, and Docker Compose configuration.

---

## Invocation

```bash
./scripts/verify-mtls-setup.sh
```

**Environment Variables:**
- `MTLS_REQUIRE_STEP=1` - Fail if step-cli missing (default: 0, warnings only)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (0 errors) |
| `1` | Failed (1+ errors) |

### Counters
- ERRORS count (incremented on failures)

---

## Side Effects

**Minimal:** Read-only verification, no persistent changes.

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Verification Checks (8)

### Check 1: step-cli Installation (lines 49-61)

**Evidence:**
```bash
if command -v step &> /dev/null; then
    VERSION=$(step version 2>&1 | head -1)
    pass "step-cli installed: $VERSION"
else
    if [ "$MTLS_REQUIRE_STEP" = "1" ]; then
        fail "step-cli not found"
        ERRORS=$((ERRORS + 1))
    else
        warn "step-cli not found (skipping step-cli-specific checks)"
    fi
fi
```

**Behavior:** Warnings unless `MTLS_REQUIRE_STEP=1`.

### Check 2: Step-CA Configuration Files (lines 63-77)

Checks for:
- `config/step-ca/ca.json` - CA server configuration (required)
- `config/step-ca/defaults.json` - Client defaults (required)

**Evidence:** Lines 64-76.

### Check 3: PKI Certificates (lines 79-109)

Checks for:
- `config/step-ca/pki/root_ca.crt` - Root CA certificate
- `config/step-ca/pki/intermediate_ca.crt` - Intermediate CA certificate

**Evidence:** Lines 80-108.

**Enhanced check:** If step-cli available, displays certificate expiry date (lines 82-99).

**JSON parsing strategy:**
```bash
# Prefer Python over jq (better Windows Git Bash compatibility)
if command -v python &> /dev/null; then
    EXPIRY=$(step certificate inspect "$PKI/root_ca.crt" --format json | \
        python - <<'PY'
import json, sys
data = json.load(sys.stdin)
print((data.get('validity', {}) or {}).get('end', 'unknown'))
PY
    )
fi
```

### Check 4: Step-CA Secrets (lines 111-123)

Checks for:
- `config/step-ca/secrets/password.txt` - CA password file
- File permissions (should be 600)

**Evidence:** Lines 112-122.

**Permission check:**
```bash
PERMS=$(stat -c %a "$FILE" 2>/dev/null ||    # Linux
        stat -f %A "$FILE" 2>/dev/null ||    # macOS
        echo "unknown")
```

### Check 5: Step-CA Service (lines 125-144)

Checks:
- Container running (name: step-ca)
- Container health status

**Evidence:** Lines 126-143.

**Health statuses:**
- `healthy` - Pass
- Other - Warning

### Check 6: Documentation (lines 146-153)

Checks for:
- `docs/MTLS_SETUP.md` - mTLS documentation (required)

**Evidence:** Lines 147-152.

### Check 7: Service Certificate Directories (lines 155-171)

Checks:
- `data/certs/` directory exists
- Certificate count (`.crt` files)

**Evidence:** Lines 156-170.

**Displays:** Number of service certificates found.

### Check 8: Docker Compose Configuration (lines 173-180)

Checks:
- Step-CA service defined in `docker-compose.identity.yml`

**Evidence:** Lines 174-179.

**Compose file resolution** (lines 10-13):
```bash
IDENTITY_COMPOSE_FILE="$PROJECT_ROOT/docker-compose.identity.yml"
if [ -f "$PROJECT_ROOT/docker/docker-compose.identity.yml" ]; then
    IDENTITY_COMPOSE_FILE="$PROJECT_ROOT/docker/docker-compose.identity.yml"
fi
```

---

## Compose Command Resolution

**Evidence:** Lines 16-21

**Prefers v2 plugin:**
```bash
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi
```

---

## Success Output

**Evidence:** Lines 183-197

**Displays quick start guide:**
```bash
# Initialize CA (if not done):
./scripts/init-step-ca.sh

# Start Step-CA:
docker compose -f docker-compose.identity.yml up -d step-ca

# Generate service certificates:
./scripts/generate-service-certs.sh

# Enable mTLS in docker-compose files and restart services
```

---

## Failure Output

**Evidence:** Lines 199-202

**Displays error count and instructs to fix errors.**

---

## Key Features

### 1. Graceful step-cli Unavailability

**Evidence:** Lines 54-59

**Default behavior (MTLS_REQUIRE_STEP=0):**
- Warns if step-cli missing
- Skips step-cli-specific checks (expiry dates)
- Continues verification
- Exits 0 if no other errors

**Strict mode (MTLS_REQUIRE_STEP=1):**
- Fails if step-cli missing
- Increments error counter
- May exit 1 if only error

### 2. Cross-Platform stat Command

**Evidence:** Lines 114

**Supports:**
- Linux: `stat -c %a` (format: %a)
- macOS: `stat -f %A` (format: %A)
- Fallback: `unknown`

**Purpose:** File permission checks work on both Linux and macOS.

### 3. Python-First JSON Parsing

**Evidence:** Lines 84-93

**Rationale:**
- Windows Git Bash often has Python
- jq may not be available or runnable
- Python parsing more reliable cross-platform

**Fallback:** If Python unavailable, EXPIRY = "unknown".

### 4. Error Counting

**Evidence:** Throughout script

**Pattern:**
```bash
if [ condition ]; then
    pass "Check passed"
else
    fail "Check failed"
    ERRORS=$((ERRORS + 1))
fi
```

**Summary:** Lines 183-202 check `$ERRORS` for final status.

---

## mTLS Setup Workflow

**Typical sequence:**

1. **Initialize CA:**
   ```bash
   ./scripts/init-step-ca.sh
   ```

2. **Verify setup:**
   ```bash
   ./scripts/verify-mtls-setup.sh
   ```

3. **Start Step-CA:**
   ```bash
   docker compose -f docker-compose.identity.yml up -d step-ca
   ```

4. **Generate service certificates:**
   ```bash
   ./scripts/generate-service-certs.sh
   ```

5. **Enable mTLS in services:**
   - Update docker-compose files with TLS mount points
   - Set environment variables for TLS
   - Restart services

---

## Integration Context

**mTLS (Mutual TLS):**
- Client authenticates server (standard TLS)
- Server authenticates client (mutual)
- Uses X.509 certificates

**ARIA mTLS:**
- Step-CA issues certificates
- Services verify each other's certificates
- Zero-trust architecture
- Short-lived certificates (90 days, auto-renewed)

**Services:**
- Vault ↔ Keycloak
- Kong ↔ Backend services
- NATS ↔ Clients
- MinIO ↔ Applications

---

## References

### Source Code
- **Main script:** `scripts/verify-mtls-setup.sh` (204 lines)
- **step-cli check:** lines 49-61
- **Config checks:** lines 63-77
- **PKI checks:** lines 79-109
- **Secrets checks:** lines 111-123
- **Service checks:** lines 125-144
- **Cert directory check:** lines 155-171

### Related Files
- **Initialization:** `scripts/init-step-ca.sh`
- **Cert generation:** `scripts/generate-service-certs.sh`
- **CA config:** `config/step-ca/ca.json`
- **Defaults:** `config/step-ca/defaults.json`
- **Documentation:** `docs/MTLS_SETUP.md`
- **Compose file:** `docker-compose.identity.yml` (or `docker/docker-compose.identity.yml`)

### External Resources
- [Step-CA](https://smallstep.com/docs/step-ca/)
- [Mutual TLS](https://en.wikipedia.org/wiki/Mutual_authentication#mTLS)
- [X.509 Certificates](https://en.wikipedia.org/wiki/X.509)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 25/60 contracts complete
