# Script Contract: fetch-localai-models.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/fetch-localai-models.sh`

---

## Purpose

Fetch, cache, and restore LocalAI models via Git LFS. Provides three operations: pull models from Git LFS into working tree, cache models from working tree to local cache, and restore models from cache back to working tree. Designed for offline workflows and large model management.

---

## Invocation

```bash
scripts/fetch-localai-models.sh <action>
```

**Actions:**
- `pull` - Fetch models via `git lfs pull` (downloads into working tree)
- `cache` - Copy models from working tree to cache directory
- `restore` - Copy models from cache back into working tree
- `status` - Print paths and basic stats (default)
- `-h`, `--help`, `help` - Show usage

**Environment Variables:**
- `LOCALAI_MODELS_CACHE_DIR` - Override cache directory (default: `$XDG_CACHE_HOME/flexnetos/localai/models` or `~/.cache/flexnetos/localai/models`)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success |
| `2` | Error (not in git repo, unknown action) |

### Status Action Output
```
RepoRoot : /path/to/ripple-env
ModelsDir: /path/to/ripple-env/docker/data/localai/models
CacheDir : /home/user/.cache/flexnetos/localai/models
ModelsDir files: 5
```

---

## Side Effects

### Pull Action (lines 53-54)
- **Downloads models** from Git LFS into `docker/data/localai/models/`
- **Command:** `git lfs pull --include="docker/data/localai/models/**"`
- **Size:** Potentially gigabytes (models are large)

### Cache Action (lines 56-59)
- **Copies models** from working tree to cache directory
- **Uses rsync** (if available) or `cp -a`
- **Preserves:** Timestamps, permissions

### Restore Action (lines 61-64)
- **Copies models** from cache to working tree
- **Overwrites** existing models in working tree

---

## Safety Classification

**🟡 CAUTION** - Downloads and copies large files, overwrites on restore.

---

## Idempotency

**✅ IDEMPOTENT (pull, status)** - Can be run repeatedly.

**⚠️ DESTRUCTIVE (cache, restore)** - Overwrites destination with source.

---

## Path Resolution

**Evidence:** Lines 25-32

**Repo root:**
```bash
repo_root="$(git rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$repo_root" ]]; then
  echo "ERROR: not inside a git repository (unable to resolve repo root)" >&2
  exit 2
fi
```

**Models directory:**
```bash
models_dir="$repo_root/docker/data/localai/models"
```

**Cache directory:**
```bash
cache_dir="${LOCALAI_MODELS_CACHE_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/flexnetos/localai/models}"
```

**XDG Base Directory:** Follows freedesktop.org specification.

---

## Actions

### pull (lines 53-54)

**Command:**
```bash
cd "$repo_root" && git lfs pull --include="docker/data/localai/models/**"
```

**Purpose:** Downloads model files tracked by Git LFS into working tree.

**Requirements:**
- Git LFS installed (`git lfs install`)
- Remote with LFS server (GitHub, GitLab, Bitbucket, or custom)
- Network connectivity

**Typical models:**
- `mistral-7b-instruct-v0.1.Q4_K_M.gguf` (4GB)
- `llama-2-7b-chat.Q4_K_M.gguf` (3.8GB)
- `all-MiniLM-L6-v2.gguf` (embeddings, 80MB)

### cache (lines 56-59)

**Command:**
```bash
copy_tree "$models_dir" "$cache_dir"
```

**Purpose:** Create local cache of models for offline use or faster restore.

**Use cases:**
- Air-gapped environments
- Slow network connections
- Frequent clean rebuilds

### restore (lines 61-64)

**Command:**
```bash
copy_tree "$cache_dir" "$models_dir"
```

**Purpose:** Restore models from cache without network access.

**Use cases:**
- After `git clean -fdx`
- New clone without LFS download
- Switching branches

### status (lines 66-75)

**Displays:**
- Repository root path
- Models directory path
- Cache directory path
- File count in models directory

**Evidence:** Lines 67-75.

---

## Copy Strategy

**Evidence:** Lines 39-50

**Function:** `copy_tree(from, to)`

**Prefers rsync:**
```bash
if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete-delay "$from/" "$to/"
else
    cp -a "$from/." "$to/"
fi
```

**rsync flags:**
- `-a` (archive) - Recursive, preserve timestamps/permissions
- `--delete-delay` - Remove extraneous files after transfer (safer)

**Fallback:** `cp -a` for portability.

---

## Workflow Examples

### Initial Setup

```bash
# 1. Clone repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# 2. Pull models from Git LFS
./scripts/fetch-localai-models.sh pull

# 3. Cache models locally (optional)
./scripts/fetch-localai-models.sh cache

# 4. Start LocalAI
docker compose -f docker-compose.localai.yml up -d
```

### Air-Gapped Deployment

```bash
# On internet-connected machine:
./scripts/fetch-localai-models.sh pull
./scripts/fetch-localai-models.sh cache

# Transfer cache directory to air-gapped machine
rsync -avz ~/.cache/flexnetos/localai/models/ \
  airgap-host:~/.cache/flexnetos/localai/models/

# On air-gapped machine:
./scripts/fetch-localai-models.sh restore
```

### After Clean

```bash
# Clean working tree (removes models)
git clean -fdx

# Restore from cache (no network needed)
./scripts/fetch-localai-models.sh restore
```

---

## LocalAI Integration

**Models location:** `docker/data/localai/models/`

**Mount in compose:**
```yaml
services:
  localai:
    volumes:
      - ./docker/data/localai/models:/models:ro
```

**Environment variable:** `LOCALAI_MODELS_PATH` set by `scripts/env-vars.sh` (default: `$HOME/.local/share/localai/models`)

**Note:** Script uses repo-local path (`docker/data/localai/models`), different from `LOCALAI_MODELS_PATH` (user home). Repo-local path is for Docker mounts.

---

## Git LFS Context

**Purpose:** Track large files without bloating git history.

**Setup:**
```bash
# Install Git LFS
git lfs install

# Track model files
git lfs track "docker/data/localai/models/*.gguf"
```

**Storage:** LFS server stores large files, git stores pointers.

**Download:** `git lfs pull` fetches actual files from LFS server.

---

## References

### Source Code
- **Main script:** `scripts/fetch-localai-models.sh` (86 lines)
- **Path resolution:** lines 25-32
- **Copy strategy:** lines 39-50
- **Actions:** lines 52-85

### Related Files
- **PowerShell equivalent:** `scripts/fetch-localai-models.ps1`
- **Environment setup:** `scripts/env-vars.sh`
- **LocalAI compose:** `docker-compose.localai.yml`

### External Resources
- [Git LFS](https://git-lfs.github.com/)
- [LocalAI](https://localai.io/)
- [XDG Base Directory Spec](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 21/60 contracts complete
