#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/fetch-localai-models.sh <action>

Actions:
  pull     Fetch LocalAI models via git-lfs (downloads into working tree)
  cache    Copy models from working tree to cache
  restore  Copy models from cache back into working tree
  status   Print paths and basic stats

Environment:
  LOCALAI_MODELS_CACHE_DIR   Override cache directory

Notes:
  - This script is designed to be safe and repeatable.
  - For URL-based downloads, extend this script with a verified manifest.
EOF
}

action="${1:-status}"

repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$repo_root" ]]; then
  echo "ERROR: not inside a git repository (unable to resolve repo root)" >&2
  exit 2
fi

models_dir="$repo_root/docker/data/localai/models"
cache_dir="${LOCALAI_MODELS_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/flexnetos/localai/models}"

ensure_dir() {
  local p="$1"
  [[ -d "$p" ]] || mkdir -p "$p"
}

copy_tree() {
  local from="$1"
  local to="$2"
  ensure_dir "$to"

  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete-delay "$from/" "$to/"
  else
    # shellcheck disable=SC2086
    cp -a "$from/." "$to/"
  fi
}

case "$action" in
  pull)
    ( cd "$repo_root" && git lfs pull --include="docker/data/localai/models/**" )
    ;;
  cache)
    ensure_dir "$models_dir"
    ensure_dir "$cache_dir"
    copy_tree "$models_dir" "$cache_dir"
    ;;
  restore)
    ensure_dir "$models_dir"
    ensure_dir "$cache_dir"
    copy_tree "$cache_dir" "$models_dir"
    ;;
  status)
    echo "RepoRoot : $repo_root"
    echo "ModelsDir: $models_dir"
    echo "CacheDir : $cache_dir"
    if [[ -d "$models_dir" ]]; then
      count=$(find "$models_dir" -type f 2>/dev/null | wc -l | tr -d ' ')
      echo "ModelsDir files: $count"
    else
      echo "ModelsDir does not exist yet."
    fi
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown action: $action" >&2
    usage >&2
    exit 2
    ;;
esac
