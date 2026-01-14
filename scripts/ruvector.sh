#!/usr/bin/env bash
# RuVector launcher (WSL/Linux/macOS)
#
# Why this exists:
# Some environments have a global npm prefix/cache pointing to a missing drive
# or locked-down location. This wrapper forces repo-local npm cache/prefix so
# `npx ruvector ...` works reliably.
#
# Examples:
#   ./scripts/ruvector.sh --version
#   ./scripts/ruvector.sh create ./data/ruvector.db
#   ./scripts/ruvector.sh insert ./data/ruvector.db ./vectors.json
#   ./scripts/ruvector.sh search ./data/ruvector.db --vector "[1,0,0]" --top-k 3

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$REPO_ROOT/.npm-cache" "$REPO_ROOT/.npm-prefix" >/dev/null 2>&1 || true

export NPM_CONFIG_CACHE="${NPM_CONFIG_CACHE:-$REPO_ROOT/.npm-cache}"
export NPM_CONFIG_PREFIX="${NPM_CONFIG_PREFIX:-$REPO_ROOT/.npm-prefix}"

if ! command -v npx >/dev/null 2>&1; then
  echo "npx not found. Install Node.js (includes npm/npx)." >&2
  exit 1
fi

if [ "$#" -eq 0 ]; then
  if [ -x "$REPO_ROOT/node_modules/.bin/ruvector" ]; then
    exec "$REPO_ROOT/node_modules/.bin/ruvector" --help
  fi
  exec npx --yes ruvector --help
fi

if [ -x "$REPO_ROOT/node_modules/.bin/ruvector" ]; then
  exec "$REPO_ROOT/node_modules/.bin/ruvector" "$@"
fi

exec npx --yes ruvector "$@"
