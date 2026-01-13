#!/usr/bin/env bash
# =============================================================================
# RuVector Integration Verification Script
# =============================================================================
# BUILDKIT_STARTER_SPEC.md Layer 10: State & Storage (P1-001)
#
# Verifies that RuVector is properly installed and configured:
#   - npm/npx CLI availability (primary)
#   - Environment variables
#   - Pixi feature flags
#   - Nix packages
#
# Notes:
#   - RuVector currently runs as an embedded/local DB via the CLI (npm package).
#   - The CLI includes a `server` subcommand with HTTP + gRPC options.
#     If you intend to use it as a networked service, verify runtime behavior by
#     starting the server and probing an endpoint (e.g. GET /health).
#   - This repo may contain a docker-compose.ruvector.yml; treat it as
#     deprecated/experimental unless you have a known-good image.
#
# Usage:
#   chmod +x scripts/verify-ruvector.sh
#   ./scripts/verify-ruvector.sh
# =============================================================================

set -e

echo "========================================"
echo "RuVector Integration Verification"
echo "========================================"
echo ""

PASSED=0
FAILED=0
WARNINGS=0

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
pass() {
    echo -e "${GREEN}✓${NC} $1"
    PASSED=$((PASSED + 1))
}

fail() {
    echo -e "${RED}✗${NC} $1"
    FAILED=$((FAILED + 1))
}

warn() {
    echo -e "${YELLOW}⚠${NC} $1"
    WARNINGS=$((WARNINGS + 1))
}

info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

# Some environments have a global npm prefix/cache pointing to a missing drive.
# Use repo-local locations for this script's npx invocations.
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$REPO_ROOT/.npm-cache" "$REPO_ROOT/.npm-prefix" >/dev/null 2>&1 || true
export NPM_CONFIG_CACHE="${NPM_CONFIG_CACHE:-$REPO_ROOT/.npm-cache}"
export NPM_CONFIG_PREFIX="${NPM_CONFIG_PREFIX:-$REPO_ROOT/.npm-prefix}"

# =============================================================================
# 1. Check CLI Availability (npm/npx)
# =============================================================================

echo "[1/6] Checking RuVector CLI availability (npm/npx)..."
echo ""

if command -v node >/dev/null 2>&1; then
    pass "Node.js is installed"
else
    fail "Node.js not found"
fi

if command -v npx >/dev/null 2>&1; then
    pass "npx is available"
else
    fail "npx not found"
fi

if [ -f ".npmrc" ]; then
    pass ".npmrc exists (repo-local npm config)"
else
    warn ".npmrc not found (npx may fail if global npm config points to a missing drive)"
fi

# Attempt a lightweight CLI invocation. This may download the package on first run.
if command -v npx >/dev/null 2>&1; then
    if RUVECTOR_VERSION=$(npx --yes ruvector --version 2>/dev/null); then
        pass "ruvector CLI is runnable via npx"
        echo "   Version: $RUVECTOR_VERSION"
    else
        warn "ruvector CLI did not run via npx"
        echo "   Hint: use scripts/ruvector.(ps1|sh) or set NPM_CONFIG_PREFIX/NPM_CONFIG_CACHE"
    fi
fi

echo ""

# =============================================================================
# 2. Check Deprecated Docker Compose (informational)
# =============================================================================

echo "[2/6] Checking deprecated Docker Compose artifacts..."
echo ""

if [ -f "docker-compose.ruvector.yml" ]; then
    warn "docker-compose.ruvector.yml exists (deprecated/experimental; RuVector is not primarily run via Docker)"
else
    pass "docker-compose.ruvector.yml not present (ok)"
fi

if [ -f "docker-compose.state.yml" ] || [ -f "docker/docker-compose.state.yml" ]; then
    pass "docker-compose.state.yml exists (includes Redis)"
else
    warn "docker-compose.state.yml not found"
fi

echo ""

# =============================================================================
# 3. Check Environment Configuration
# =============================================================================

echo "[3/6] Checking environment configuration..."
echo ""

if [ -f ".env.state.example" ]; then
    pass ".env.state.example exists"

    if grep -q "RUVECTOR_ENDPOINT" .env.state.example; then
        pass "RUVECTOR_ENDPOINT variable defined"
    else
        fail "RUVECTOR_ENDPOINT variable not defined"
    fi

    if grep -q "RUVECTOR_GNN_ENABLED" .env.state.example; then
        pass "RUVECTOR_GNN_ENABLED variable defined"
    else
        fail "RUVECTOR_GNN_ENABLED variable not defined"
    fi

    if grep -q "VECTOR_STORE" .env.state.example; then
        pass "VECTOR_STORE feature flag defined"
    else
        fail "VECTOR_STORE feature flag not defined"
    fi
else
    fail ".env.state.example not found"
fi

echo ""

# =============================================================================
# 4. Check Pixi Feature Flags
# =============================================================================

echo "[4/6] Checking Pixi feature flags..."
echo ""

if [ -f "pixi.toml" ]; then
    if grep -q '\[feature\.vectordb-ruvector\]' pixi.toml; then
        pass "vectordb-ruvector feature defined in pixi.toml"
    else
        fail "vectordb-ruvector feature not defined in pixi.toml"
    fi

    if grep -q '\[feature\.vectordb-chromadb\]' pixi.toml; then
        pass "vectordb-chromadb feature defined in pixi.toml"
    else
        warn "vectordb-chromadb feature not defined (A/B testing unavailable)"
    fi

    if grep -q 'vectordb-ruvector.*=.*{.*features.*=.*\["vectordb-ruvector"\]' pixi.toml; then
        pass "vectordb-ruvector environment defined"
    else
        fail "vectordb-ruvector environment not defined"
    fi
else
    fail "pixi.toml not found"
fi

echo ""

# =============================================================================
# 5. Check Nix Packages
# =============================================================================

echo "[5/6] Checking Nix packages..."
echo ""

if [ -f "nix/packages/distributed.nix" ]; then
    pass "nix/packages/distributed.nix exists"

    if grep -q "grpcurl" nix/packages/distributed.nix; then
        pass "gRPC tools included in distributed.nix"
    else
        warn "gRPC tools not found in distributed.nix"
    fi

    if grep -q "wasm-pack" nix/packages/distributed.nix; then
        pass "WASM tools included in distributed.nix"
    else
        warn "WASM tools not found in distributed.nix"
    fi
else
    fail "nix/packages/distributed.nix not found"
fi

if [ -f "nix/packages/default.nix" ]; then
    if grep -q "distributed" nix/packages/default.nix; then
        pass "distributed packages imported in default.nix"
    else
        fail "distributed packages not imported in default.nix"
    fi
else
    fail "nix/packages/default.nix not found"
fi

echo ""

# =============================================================================
# 6. Functional smoke test (embedded/local CLI)
# =============================================================================

echo "[6/6] Running embedded/local CLI smoke test..."
echo ""

if command -v npx >/dev/null 2>&1; then
    if npx --yes ruvector doctor >/dev/null 2>&1; then
        pass "ruvector doctor passed"
    else
        warn "ruvector doctor reported issues"
    fi

    if npx --yes ruvector info >/dev/null 2>&1; then
        pass "ruvector info ran"
    else
        warn "ruvector info failed"
    fi

    # Validate the server command is present (does not start the server).
    if npx --yes ruvector server --help >/dev/null 2>&1; then
        pass "ruvector server subcommand is available (HTTP/gRPC options present)"
    else
        warn "ruvector server subcommand not available"
    fi

    # Optional: require "full" feature set (GNN + graph) for elite mode.
    # Enable with: RUVECTOR_REQUIRE_FULL=1
    if [ "${RUVECTOR_REQUIRE_FULL:-0}" = "1" ]; then
        GNN_OUT="$(npx --yes ruvector gnn info 2>&1 || true)"
        if echo "$GNN_OUT" | grep -qi "Not installed"; then
            fail "GNN module is not installed (RUVECTOR_REQUIRE_FULL=1)"
        elif [ -z "$GNN_OUT" ]; then
            fail "GNN check produced no output (RUVECTOR_REQUIRE_FULL=1)"
        elif echo "$GNN_OUT" | grep -qi "error\|failed"; then
            fail "GNN check reported an error (RUVECTOR_REQUIRE_FULL=1)"
        else
            pass "GNN module appears installed"
        fi

        if npx --yes ruvector graph --help >/dev/null 2>&1; then
            pass "Graph subcommand is available"
        else
            fail "Graph subcommand is not usable (RUVECTOR_REQUIRE_FULL=1)"
        fi
    else
        GNN_OUT="$(npx --yes ruvector gnn info 2>&1 || true)"
        if [ -z "$GNN_OUT" ]; then
            warn "GNN check produced no output"
        elif echo "$GNN_OUT" | grep -qi "Not installed"; then
            warn "GNN module not installed (set RUVECTOR_REQUIRE_FULL=1 to enforce)"
        else
            pass "GNN module appears installed"
        fi
    fi

    # Optional deeper test: create/insert/search a tiny local DB.
    # Enabled only when RUVECTOR_SMOKE_DB=1 is set.
    if [ "${RUVECTOR_SMOKE_DB:-0}" = "1" ]; then
        TMPDIR=$(mktemp -d 2>/dev/null || true)
        if [ -n "$TMPDIR" ] && [ -d "$TMPDIR" ]; then
            DB_PATH="$TMPDIR/ruvector-test.db"
            JSON_PATH="$TMPDIR/vectors.json"

            cat > "$JSON_PATH" <<'EOF'
[
  {"id": "a", "vector": [1.0, 0.0, 0.0]},
  {"id": "b", "vector": [0.0, 1.0, 0.0]},
  {"id": "c", "vector": [0.0, 0.0, 1.0]}
]
EOF

            if npx --yes ruvector create "$DB_PATH" --dimension 3 >/dev/null 2>&1; then
                pass "Created local RuVector DB"
            else
                warn "Failed to create local RuVector DB (DB commands may be experimental on some platforms)"
            fi

            if npx --yes ruvector insert "$DB_PATH" "$JSON_PATH" >/dev/null 2>&1; then
                pass "Inserted test vectors"
            else
                warn "Failed to insert test vectors"
            fi

            if npx --yes ruvector search "$DB_PATH" --vector "[1,0,0]" --top-k 1 >/dev/null 2>&1; then
                pass "Search query executed"
            else
                warn "Search query failed"
            fi

            rm -rf "$TMPDIR" >/dev/null 2>&1 || true
        else
            warn "Could not create temp directory for smoke test"
        fi
    fi
else
    warn "Skipping smoke test (npx not available)"
fi

echo ""

# =============================================================================
# Summary
# =============================================================================

echo "========================================"
echo "Verification Summary"
echo "========================================"
echo ""
echo -e "Passed:   ${GREEN}$PASSED${NC}"
echo -e "Failed:   ${RED}$FAILED${NC}"
echo -e "Warnings: ${YELLOW}$WARNINGS${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}RuVector integration verification passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Explore CLI: npx --yes ruvector --help"
    echo "  2. Create a local DB: npx --yes ruvector create ./data/ruvector.db"
    echo "  3. Insert vectors: npx --yes ruvector insert ./data/ruvector.db ./path/to/vectors.json"
    echo "  4. If you need HTTP/gRPC: run 'ruvector server --help' and validate runtime by starting the server and probing /health"
    exit 0
else
    echo -e "${RED}RuVector integration verification failed.${NC}"
    echo ""
    echo "Review the errors above and consult:"
    echo "  - docs/implementation/L10_STATE_STORAGE_IMPLEMENTATION.md"
    echo "  - docs/audits/RUVECTOR_QUDAG_AUDIT_REPORT.md"
    exit 1
fi
