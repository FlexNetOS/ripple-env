# Script Contract: verify-state-storage.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-state-storage.sh`

---

## Purpose

Verify Layer 10 State & Storage components (P0-005 Redis, P1-001 RuVector, P1-006 MinIO) are properly installed and configured. Comprehensive 5-stage verification process.

---

## Invocation

```bash
chmod +x scripts/verify-state-storage.sh
./scripts/verify-state-storage.sh
```

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | All checks passed |
| `1` | Some checks failed |

---

## Side Effects

**Mostly read-only:** Creates temporary directories for npm cache/prefix workaround (lines 195-197).

---

## Safety Classification

**🟢 SAFE** - Read-only verification with minimal temp directory creation.

---

## Idempotency

**✅ FULLY IDEMPOTENT**

---

## Verification Stages

### Stage 1: Configuration Files (4 checks)
- `docker-compose.state.yml` exists
- `.env.state.example` exists
- `.npmrc` exists (warning if missing)

### Stage 2: Docker Availability (2 checks)
- Docker installed
- Docker Compose available

### Stage 3: Compose Validation (2 checks)
- Compose file syntax valid
- Docker network `agentic-network` exists

### Stage 4: Service Status (4 checks)
- Redis running
- Redis responds to PING
- MinIO running
- MinIO health check passes

### Stage 5: RuVector Installation (4 checks)
- Node.js installed
- npx available
- ruvector CLI runnable
- ruvector doctor passes

---

## NPM Workaround

**Evidence:** Lines 194-197

Sets repo-local npm cache/prefix to avoid issues with global npm config pointing to missing drives:

```bash
export NPM_CONFIG_CACHE="$REPO_ROOT/.npm-cache"
export NPM_CONFIG_PREFIX="$REPO_ROOT/.npm-prefix"
```

---

## Next Steps

Displayed on success (lines 234-238):
1. Start services: `docker compose -f docker-compose.state.yml up -d`
2. Test Redis: `docker compose -f docker-compose.state.yml exec redis redis-cli ping`
3. Access MinIO Console: http://localhost:9001
4. Run ruvector CLI: `npx --yes ruvector --help`

---

## References

### Source Code
- **Main script:** `scripts/verify-state-storage.sh` (246 lines)
- **File checks:** lines 54-78
- **Docker checks:** lines 85-102
- **Compose validation:** lines 109-129
- **Service status:** lines 137-171
- **RuVector checks:** lines 179-215

### Related Files
- **Compose file:** `docker/docker-compose.state.yml`
- **Documentation:** `docs/implementation/L10_STATE_STORAGE_IMPLEMENTATION.md`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 9/60 contracts complete
