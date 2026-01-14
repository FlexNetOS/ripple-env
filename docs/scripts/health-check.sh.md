# Script Contract: health-check.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/health-check.sh`

---

## Purpose

Generic health check script with exponential backoff retry logic for CI/CD workflows. Tests HTTP endpoints with configurable retries and delays. Designed for use in GitHub Actions, GitLab CI, and deployment scripts to wait for services to become healthy.

---

## Invocation

```bash
./health-check.sh <URL> [MAX_RETRIES] [RETRY_DELAY] [SERVICE_NAME]
```

**Arguments:**
- `URL` - Health check endpoint URL (required)
- `MAX_RETRIES` - Maximum retry attempts (default: 5)
- `RETRY_DELAY` - Initial delay between retries in seconds (default: 10)
- `SERVICE_NAME` - Service name for logging (default: extracted from URL)

**Examples:**
```bash
./health-check.sh http://localhost:8080/readyz
./health-check.sh http://localhost:8080/readyz 5 10 "LocalAI"
./health-check.sh https://vault:8200/v1/sys/health 10 5 "Vault"
```

---

## Outputs

**Standard Output (success):**
```
Health check for localhost (http://localhost:8080/readyz)
  Max retries: 5, Initial delay: 10s
✅ localhost is healthy (attempt 1/5)
```

**Standard Output (retry):**
```
Health check for LocalAI (http://localhost:8080/readyz)
  Max retries: 5, Initial delay: 10s
⏳ Attempt 1/5 failed, retrying in 10s...
⏳ Attempt 2/5 failed, retrying in 20s...
✅ LocalAI is healthy (attempt 3/5)
```

**Standard Output (failure):**
```
Health check for LocalAI (http://localhost:8080/readyz)
  Max retries: 5, Initial delay: 10s
⏳ Attempt 1/5 failed, retrying in 10s...
⏳ Attempt 2/5 failed, retrying in 20s...
⏳ Attempt 3/5 failed, retrying in 30s...
⏳ Attempt 4/5 failed, retrying in 40s...
❌ LocalAI failed health check after 5 attempts
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (service healthy) |
| `1` | Failure (max retries exhausted or no URL provided) |

---

## Side Effects

**None** - Read-only HTTP GET requests only.

---

## Safety Classification

**🟢 SAFE** - Read-only health checks, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Retry Logic

### Exponential Backoff (lines 46-49)

**Evidence:**
```bash
# Exponential backoff with jitter
CURRENT_DELAY=$((RETRY_DELAY * i))
echo "⏳ Attempt $i/$MAX_RETRIES failed, retrying in ${CURRENT_DELAY}s..."
sleep "$CURRENT_DELAY"
```

**Formula:** `CURRENT_DELAY = RETRY_DELAY × attempt_number`

**Example (RETRY_DELAY=10):**
- Attempt 1: 10s delay
- Attempt 2: 20s delay
- Attempt 3: 30s delay
- Attempt 4: 40s delay
- Attempt 5: 50s delay

**Total wait time:** 10 + 20 + 30 + 40 + 50 = 150 seconds (2.5 minutes)

**Note:** Comment says "with jitter" but implementation is linear, not exponential. True exponential would be `2^i × RETRY_DELAY`.

---

## curl Configuration

**Evidence:** Line 36

```bash
curl -sf --connect-timeout 5 --max-time 10 "$URL" >/dev/null 2>&1
```

**Flags:**
- `-s` (silent) - No progress bar
- `-f` (fail) - Exit code 22 for HTTP errors (4xx, 5xx)
- `--connect-timeout 5` - 5 seconds to establish connection
- `--max-time 10` - 10 seconds total operation timeout

**Output:** Discarded (`>/dev/null 2>&1`)

**Exit codes:**
- 0 - Success (HTTP 2xx)
- Non-zero - Failure (connection error, timeout, HTTP error)

---

## Service Name Extraction

**Evidence:** Lines 28-30

```bash
if [[ -z "$SERVICE_NAME" ]]; then
    SERVICE_NAME=$(echo "$URL" | sed -E 's|https?://([^/:]+).*|\1|')
fi
```

**Regex:** `https?://([^/:]+).*`
- Captures hostname or IP
- Stops at first `:` (port) or `/` (path)

**Examples:**
- `http://localhost:8080/health` → `localhost`
- `https://vault.aria.local:8200/v1/sys/health` → `vault.aria.local`
- `http://192.168.1.10:9000/ready` → `192.168.1.10`

---

## Usage in CI/CD

### GitHub Actions

```yaml
- name: Wait for LocalAI
  run: |
    ./scripts/health-check.sh http://localhost:8080/readyz 10 5 "LocalAI"
```

### GitLab CI

```yaml
script:
  - ./scripts/health-check.sh http://vault:8200/v1/sys/health 15 10 "Vault"
```

### Docker Compose Healthcheck

```yaml
services:
  localai:
    healthcheck:
      test: ["CMD", "./scripts/health-check.sh", "http://localhost:8080/readyz", "3", "5"]
      interval: 30s
      timeout: 10s
      retries: 3
```

### Deployment Script

```bash
# Start service
docker compose up -d localai

# Wait for healthy
./scripts/health-check.sh http://localhost:8080/readyz 10 10 "LocalAI"

# Proceed with dependent services
docker compose up -d app
```

---

## Typical Retry Configurations

### Fast Services (< 10s startup)

```bash
./health-check.sh http://localhost:8080/health 3 2 "FastAPI"
# Total: 2 + 4 + 6 = 12 seconds max
```

### Medium Services (10-60s startup)

```bash
./health-check.sh http://localhost:8080/health 5 10 "LocalAI"
# Total: 10 + 20 + 30 + 40 + 50 = 150 seconds max
```

### Slow Services (60-300s startup)

```bash
./health-check.sh http://localhost:8200/v1/sys/health 10 15 "Vault"
# Total: 15 + 30 + 45 + 60 + 75 + 90 + 105 + 120 + 135 + 150 = 825 seconds max
```

---

## Health Check Endpoints

**Common patterns:**

### Kubernetes-style
- `/healthz` - Liveness probe
- `/readyz` - Readiness probe
- `/livez` - Liveness (alternative)

### HashiCorp Vault
- `/v1/sys/health` - Overall health
- `/v1/sys/seal-status` - Seal status

### MindsDB
- `/api/status` - API status

### NATS
- `/healthz` - Health endpoint (requires monitoring port)

### Custom services
- `/health` - Generic health
- `/ping` - Simple ping
- `/status` - Status endpoint

---

## Exit Code Handling

**Evidence:** Lines 36-44, 52

**Success path (lines 36-38):**
```bash
if curl -sf ... "$URL" >/dev/null 2>&1; then
    echo "✅ $SERVICE_NAME is healthy (attempt $i/$MAX_RETRIES)"
    exit 0
fi
```

**Failure path (lines 41-44):**
```bash
if [[ $i -eq $MAX_RETRIES ]]; then
    echo "❌ $SERVICE_NAME failed health check after $MAX_RETRIES attempts"
    exit 1
fi
```

**Final exit (line 52):**
```bash
exit 1  # Fallback (should never reach here)
```

---

## Limitations

**HTTP/HTTPS only:** Does not support TCP socket checks, gRPC health checks, or custom protocols.

**No authentication:** Does not support basic auth, bearer tokens, or mTLS.

**No custom headers:** Cannot set custom headers for auth or routing.

**No response validation:** Only checks HTTP status code, not response body.

**Linear backoff:** Comment says "exponential backoff with jitter" but implementation is linear.

---

## Improvements for Production

**True exponential backoff:**
```bash
CURRENT_DELAY=$((RETRY_DELAY * (2 ** (i - 1))))
```

**Jitter (randomization):**
```bash
JITTER=$((RANDOM % RETRY_DELAY))
CURRENT_DELAY=$((CURRENT_DELAY + JITTER))
```

**Custom headers:**
```bash
curl -sf -H "Authorization: Bearer $TOKEN" "$URL"
```

**Response validation:**
```bash
if curl -sf "$URL" | grep -q "status.*ok"; then
    # Healthy
fi
```

---

## Integration

**Used by:**
- CI/CD workflows (.github/workflows/*.yml)
- Deployment scripts (deploy.sh, deploy-edge.sh, etc.)
- Docker Compose healthcheck commands
- Manual service verification

**Depends on:**
- curl (universally available)

---

## References

### Source Code
- **Main script:** `scripts/health-check.sh` (53 lines)
- **curl command:** line 36
- **Retry logic:** lines 35-50
- **Service name extraction:** lines 28-30

### Related Files
- **Deployment scripts:** `scripts/deploy*.sh`
- **CI workflows:** `.github/workflows/*.yml`
- **Docker Compose:** Various compose files with healthchecks

### External Resources
- [curl Documentation](https://curl.se/docs/manpage.html)
- [Kubernetes Probes](https://kubernetes.io/docs/tasks/configure-pod-container/configure-liveness-readiness-startup-probes/)
- [Exponential Backoff](https://en.wikipedia.org/wiki/Exponential_backoff)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 29/60 contracts complete
