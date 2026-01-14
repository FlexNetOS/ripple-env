#!/usr/bin/env bash
# Validate that Git LFS is available when required, and (optionally) that the
# checked-out commit has its LFS objects present locally.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "$REPO_ROOT"

if ! command -v git >/dev/null 2>&1; then
  echo "Error: git not found" >&2
  exit 2
fi

# Decide whether this repo "uses LFS" by looking for filter=lfs in .gitattributes.
USES_LFS=0
if [ -f ".gitattributes" ] && grep -q "filter=lfs" .gitattributes; then
  USES_LFS=1
fi

if [ "$USES_LFS" -eq 0 ]; then
  echo "Git LFS not configured in .gitattributes; skipping."
  exit 0
fi

# Require git-lfs to be installed if repo config indicates LFS usage.
if ! git lfs version >/dev/null 2>&1; then
  echo "Error: git-lfs is required (repo has filter=lfs in .gitattributes), but 'git lfs' is not available." >&2
  echo "Install git-lfs, then run: git lfs install" >&2
  exit 1
fi

# Decide whether to require LFS objects locally.
# Default: require on CI/GitHub Actions; optional locally.
REQUIRE_OBJECTS="${LFS_REQUIRE_OBJECTS:-}"
if [ -z "$REQUIRE_OBJECTS" ]; then
  if [ "${GITHUB_ACTIONS:-}" = "true" ] || [ "${CI:-}" = "true" ] || [ "${CI:-}" = "1" ]; then
    REQUIRE_OBJECTS=1
  else
    REQUIRE_OBJECTS=0
  fi
fi

# If there are no LFS-tracked files in this commit, don't fsck.
LFS_COUNT=0
if git lfs ls-files -n >/dev/null 2>&1; then
  # Count lines (portable)
  LFS_COUNT=$(git lfs ls-files -n | wc -l | tr -d ' ')
fi

if [ "$LFS_COUNT" -eq 0 ]; then
  echo "Git LFS available; no LFS-tracked files in this commit." 
  exit 0
fi

echo "Git LFS available; ${LFS_COUNT} LFS-tracked path(s) in this commit."

if [ "$REQUIRE_OBJECTS" -eq 1 ]; then
  echo "Checking LFS objects (git lfs fsck)…"
  git lfs fsck
  echo "Git LFS objects present."
else
  echo "Skipping LFS object presence check (set LFS_REQUIRE_OBJECTS=1 to enforce)."
fi
