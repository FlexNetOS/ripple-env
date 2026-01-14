# Script Contract: verify-ruvector.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-ruvector.sh`

---

## Purpose

Verify RuVector integration (P1-001, Layer 10 State & Storage). Checks npm/npx CLI availability, environment variables, Pixi feature flags, Nix packages, and performs functional smoke tests. RuVector runs as embedded/local DB via npm CLI.

---

## Invocation

```bash
chmod +x scripts/verify-ruvector.sh
./scripts/verify-ruvector.sh
```

**Environment Variables:**
- `RUVECTOR_REQUIRE_FULL=1` - Enforce GNN and graph modules (default: 0, warnings only)
- `RUVECTOR_SMOKE_DB=1` - Run deep smoke test with temp DB (default: 0, skip)
- `NPM_CONFIG_CACHE` - npm cache location (default: .npm-cache in repo)
- `NPM_CONFIG_PREFIX` - npm prefix location (default: .npm-prefix in repo)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Passed (0 failures) |
| `1` | Failed (1+ failures) |

### Counters
- Passed (green)
- Failed (red)
- Warnings (yellow)

---

## Side Effects

**Minimal:** Creates repo-local npm cache/prefix directories (lines 67-69). Downloads ruvector package on first npx invocation.

**Smoke Test (RUVECTOR_SMOKE_DB=1):** Creates temp directory, test DB, vectors.json, performs insert/search, cleanup (lines 291-325).

---

## Safety Classification

**🟢 SAFE** - Read-only verification, temp files only cleaned up.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Verification Stages (6)

### Stage 1: CLI Availability (lines 75-107)
- Node.js installed
- npx available
- .npmrc exists (optional)
- ruvector CLI runnable via npx
- Version check

### Stage 2: Docker Compose (lines 113-127)
- docker-compose.ruvector.yml NOT present (deprecated)
- docker-compose.state.yml exists

### Stage 3: Environment Config (lines 134-160)
- .env.state.example exists
- RUVECTOR_ENDPOINT defined
- RUVECTOR_GNN_ENABLED defined
- VECTOR_STORE feature flag defined

### Stage 4: Pixi Feature Flags (lines 167-191)
- vectordb-ruvector feature in pixi.toml
- vectordb-chromadb feature (for A/B testing)
- vectordb-ruvector environment defined

### Stage 5: Nix Packages (lines 198-228)
- nix/packages/distributed.nix exists
- gRPC tools included (grpcurl)
- WASM tools included (wasm-pack)
- distributed imported in default.nix

### Stage 6: Functional Smoke Test (lines 235-329)
- `ruvector doctor` passes
- `ruvector info` runs
- `ruvector server --help` available
- GNN module check (warning or fail based on RUVECTOR_REQUIRE_FULL)
- Graph subcommand check (if RUVECTOR_REQUIRE_FULL=1)
- Optional DB test: create, insert, search (if RUVECTOR_SMOKE_DB=1)

---

## NPM Workaround

**Evidence:** Lines 66-69

Sets repo-local npm cache/prefix to avoid global config issues:
```bash
export NPM_CONFIG_CACHE="${NPM_CONFIG_CACHE:-$REPO_ROOT/.npm-cache}"
export NPM_CONFIG_PREFIX="${NPM_CONFIG_PREFIX:-$REPO_ROOT/.npm-prefix}"
```

---

## Smoke Test (Optional)

**Trigger:** `RUVECTOR_SMOKE_DB=1`
**Evidence:** Lines 290-326

**Creates:**
1. Temp directory
2. Test DB (3 dimensions)
3. vectors.json with 3 test vectors
4. Inserts vectors
5. Searches for [1,0,0]
6. Cleans up temp directory

---

## Next Steps

Displayed on success (lines 349-353):
```bash
npx --yes ruvector --help
npx --yes ruvector create ./data/ruvector.db
npx --yes ruvector insert ./data/ruvector.db ./vectors.json
ruvector server --help  # For HTTP/gRPC mode
```

---

## References

### Source Code
- **Main script:** `scripts/verify-ruvector.sh` (363 lines)
- **CLI checks:** lines 75-107
- **Environment checks:** lines 134-160
- **Pixi checks:** lines 167-191
- **Nix checks:** lines 198-228
- **Smoke test:** lines 235-329

### Related Files
- **Wrapper script:** `scripts/ruvector.sh`
- **Environment:** `.env.state.example`
- **Pixi config:** `pixi.toml`
- **Nix packages:** `nix/packages/distributed.nix`
- **Documentation:** `docs/implementation/L10_STATE_STORAGE_IMPLEMENTATION.md`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 12/60 contracts complete
