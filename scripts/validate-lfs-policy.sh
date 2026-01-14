#!/usr/bin/env bash
# Enforce Git LFS size policy:
# Any Git blob stored in normal Git history at/above the threshold must be stored
# via Git LFS (i.e., the Git blob should be an LFS pointer file).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "$REPO_ROOT"

# Default: 25 MiB
MAX_BLOB_BYTES="${LFS_MAX_GIT_BLOB_BYTES:-}"

# If not explicitly set, try to read from hashes.yml (best-effort, no YAML parser).
if [ -z "$MAX_BLOB_BYTES" ] && [ -f "hashes.yml" ]; then
  parsed="$(grep -E '^[[:space:]]*max_git_blob_bytes:[[:space:]]*[0-9]+' hashes.yml | head -n 1 | sed -E 's/.*max_git_blob_bytes:[[:space:]]*([0-9]+).*/\1/')" || true
  if [ -n "${parsed:-}" ]; then
    MAX_BLOB_BYTES="$parsed"
  fi
fi

if [ -z "$MAX_BLOB_BYTES" ]; then
  MAX_BLOB_BYTES="26214400"
fi

if ! command -v git >/dev/null 2>&1; then
  echo "Error: git not found" >&2
  exit 2
fi

failures=0

# Iterate tracked files (NUL-delimited for safety).
while IFS= read -r -d '' path; do
  # Skip empty
  [ -n "$path" ] || continue

  # Determine blob size in current HEAD (or index as fallback).
  size=""
  if size=$(git cat-file -s "HEAD:${path}" 2>/dev/null); then
    :
  elif size=$(git cat-file -s ":${path}" 2>/dev/null); then
    :
  else
    # Could be in sparse checkout or unresolvable; ignore.
    continue
  fi

  # If size is not an integer, ignore.
  case "$size" in
    ''|*[!0-9]*) continue ;;
  esac

  if [ "$size" -ge "$MAX_BLOB_BYTES" ]; then
    echo "ERROR: Git blob too large (>= ${MAX_BLOB_BYTES} bytes): ${path} (blob size ${size})" >&2
    echo "       Store this via Git LFS (add an attribute pattern) and recommit." >&2
    failures=$((failures + 1))
  fi

done < <(git ls-files -z)

if [ "$failures" -gt 0 ]; then
  echo "" >&2
  echo "Git LFS policy violations: $failures" >&2
  exit 1
fi

echo "Git LFS size policy OK (max_git_blob_bytes=${MAX_BLOB_BYTES})."
