# Script Contract: ruvector.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/ruvector.sh`

---

## Purpose

RuVector CLI launcher wrapper for WSL/Linux/macOS. Forces repo-local npm cache/prefix to avoid issues with global npm configuration pointing to missing drives or locked-down locations. Ensures `npx ruvector` works reliably.

---

## Invocation

```bash
./scripts/ruvector.sh [RUVECTOR_ARGS...]
```

**Examples:**
```bash
./scripts/ruvector.sh --version
./scripts/ruvector.sh create ./data/ruvector.db
./scripts/ruvector.sh insert ./data/ruvector.db ./vectors.json
./scripts/ruvector.sh search ./data/ruvector.db --vector "[1,0,0]" --top-k 3
./scripts/ruvector.sh --help
```

---

## Inputs

### Arguments
All arguments passed directly to `ruvector` CLI.

### Environment Variables
- `NPM_CONFIG_CACHE` - npm cache location (default: `$REPO_ROOT/.npm-cache`)
- `NPM_CONFIG_PREFIX` - npm prefix location (default: `$REPO_ROOT/.npm-prefix`)

---

## Outputs

### Standard Output
Output from ruvector CLI command.

### Exit Codes
- Exit code from ruvector CLI
- `1` if npx not found

---

## Side Effects

**Creates directories:** `.npm-cache` and `.npm-prefix` in repo root (line 18).

**Executes:** Either local `node_modules/.bin/ruvector` or `npx --yes ruvector`.

---

## Safety Classification

**🟢 SAFE** - Wrapper only, delegates to ruvector CLI.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Creates dirs with `-p`, checks existence.

---

## Execution Logic

**Evidence:** Lines 28-39

1. **No arguments:** Show help
   - Prefers local: `node_modules/.bin/ruvector --help` (lines 29-30)
   - Falls back: `npx --yes ruvector --help` (line 32)

2. **With arguments:** Execute command
   - Prefers local: `node_modules/.bin/ruvector "$@"` (lines 35-36)
   - Falls back: `npx --yes ruvector "$@"` (line 39)

---

## NPM Configuration

**Evidence:** Lines 20-21

```bash
export NPM_CONFIG_CACHE="${NPM_CONFIG_CACHE:-$REPO_ROOT/.npm-cache}"
export NPM_CONFIG_PREFIX="${NPM_CONFIG_PREFIX:-$REPO_ROOT/.npm-prefix}"
```

Overrides global npm config for this session.

---

## Requirements

**npx:** Required (lines 23-26)

```bash
if ! command -v npx >/dev/null 2>&1; then
  echo "npx not found. Install Node.js (includes npm/npx)." >&2
  exit 1
fi
```

---

## References

### Source Code
- **Main script:** `scripts/ruvector.sh` (40 lines)
- **Path setup:** lines 17-21
- **Execution logic:** lines 28-39

### Related Files
- **PowerShell equivalent:** `scripts/ruvector.ps1`
- **Verification:** `scripts/verify-ruvector.sh`

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 13/60 contracts complete
