# Script Contract: deploy-airgapped.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/deploy-airgapped.sh`

---

## Purpose

Deploy ripple-env in air-gapped environment without internet access. Configures Nix, Pixi, and Docker to use pre-cached offline packages prepared by `prepare-offline.sh`. Sets up local Docker registry, imports images, and configures environment for offline operation.

This is the specialized deployment script for disconnected/secure environments.

---

## Invocation

```bash
./scripts/deploy-airgapped.sh [OPTIONS]
```

**Options:**
- `--cache-dir DIR` - Location of offline cache (default: /var/cache/ripple-env)
- `--bundle FILE` - Extract from offline bundle archive
- `--setup-registry` - Start local Docker registry only
- `--import-images` - Import Docker images from cache only
- `--configure-nix` - Configure Nix for offline use only
- `--configure-pixi` - Configure Pixi for offline use only
- `--all` - Perform all setup steps (default)
- `--verify` - Verify offline setup
- `--help` - Show help message

**Evidence:** lines 7-19, 51-101

---

## Inputs

### Arguments
| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--cache-dir` | No | `/var/cache/ripple-env` or `$RIPPLE_CACHE_DIR` | Offline cache location |
| `--bundle` | No | none | Path to tarball bundle to extract |
| `--setup-registry` | No | false | Only start Docker registry |
| `--import-images` | No | false | Only import Docker images |
| `--configure-nix` | No | false | Only configure Nix |
| `--configure-pixi` | No | false | Only configure Pixi |
| `--all` | No | true (default) | Perform all setup steps |
| `--verify` | No | false | Run verification checks |

### Environment Variables
| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `RIPPLE_CACHE_DIR` | No | `/var/cache/ripple-env` | Override default cache directory |

### Required Files/Directories
| Path | Required | Purpose |
|------|----------|---------|
| `$CACHE_DIR/` | Yes | Root offline cache directory |
| `$CACHE_DIR/nix-store/` | No (optional) | Nix package cache |
| `$CACHE_DIR/docker-images/` | No (optional) | Docker image tarballs |
| `$CACHE_DIR/conda-channels/` | No (optional) | Conda package cache |
| `$CACHE_DIR/models/` | No (optional) | AI model cache |
| Bundle tarball | No (if --bundle) | Compressed offline cache |

---

## Outputs

### Files Created
| File | Purpose |
|------|---------|
| `~/.config/nix/nix.conf` | Nix offline configuration |
| `~/.config/nix/nix.conf.bak.<timestamp>` | Backup of existing nix.conf |
| `~/.config/pixi/config.toml` | Pixi offline configuration |
| `~/.config/pixi/config.toml.bak.<timestamp>` | Backup of existing pixi config |
| `~/.config/ripple-env/offline.env` | Environment variables for offline mode |

**Evidence:** lines 233-341

### Standard Output
- Colored log messages (INFO, SUCCESS, WARN, ERROR)
- Progress for each deployment step
- Verification results
- Activation instructions

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - offline setup complete |
| `1` | Failure - cache missing, Docker unavailable, or setup failed |

---

## Side Effects

### System Modifications
1. **Starts Docker registry container**
   - Container name: `ripple-registry`
   - Port: 5000
   - Volume: `$CACHE_DIR/docker-registry`
   - Restart policy: always
   - Evidence: lines 164-173

2. **Loads Docker images** from tarballs
   - Evidence: lines 188-197

3. **Tags and pushes images** to local registry
   - Format: `localhost:5000/<image-name>`
   - Evidence: lines 200-213

4. **Writes Nix configuration**
   - `~/.config/nix/nix.conf`
   - Sets substituters to local cache
   - Disables signature verification
   - Evidence: lines 241-259

5. **Writes Pixi configuration**
   - `~/.config/pixi/config.toml`
   - Mirrors channels to local cache
   - Enables offline mode
   - Evidence: lines 283-295

6. **Creates offline environment file**
   - `~/.config/ripple-env/offline.env`
   - Sets multiple offline environment variables
   - Evidence: lines 308-337

7. **Extracts bundle archive** (if provided)
   - Evidence: lines 108-125

### Network Activity
None. Script is designed for air-gapped environments.

---

## Safety Classification

**🟢 SAFE**

**Rationale:**
- No network access required (by design)
- Backs up existing configurations before modification
- Creates files in user home directory (no sudo required)
- Docker registry runs as unprivileged container
- All operations are reversible

**Safe for:**
- Air-gapped environments
- Secure/classified networks
- Offline development workstations

**Note:** Assumes offline cache was prepared securely by `prepare-offline.sh`.

---

## Idempotency

**✅ FULLY IDEMPOTENT**

1. **Docker registry check** - Won't start if already running (lines 149-152)
2. **Configuration backups** - Creates timestamped backups (lines 236-238, 278-280)
3. **Image loading** - Safe to reload (docker load handles duplicates)
4. **Network connectivity** - No external dependencies

**Re-run behavior:**
- Safe to run multiple times
- Preserves existing configurations via backup
- Updates configurations to latest offline settings

---

## Dependencies

### Required Tools (Conditional)
| Tool | Required For | Check |
|------|--------------|-------|
| `docker` | Registry setup, image import | lines 143-146 |
| `tar` | Bundle extraction | line 121 |

### Required Directory Structure
```
$CACHE_DIR/
├── nix-store/           # Nix packages (.narinfo files)
├── docker-images/       # Docker image tarballs (*.tar)
│   └── registry_2.tar   # Registry image (for bootstrap)
├── docker-registry/     # Registry volume data
├── conda-channels/      # Conda package cache
│   ├── conda-forge/
│   └── robostack-humble/
└── models/             # AI model cache
    └── huggingface/
```

---

## Failure Modes

### Cache Directory Missing
**Symptom:** Cache directory not found
**Exit Code:** 1
**Recovery:** Run `prepare-offline.sh` first or provide --bundle
**Evidence:** lines 129-133

**Example:**
```
[ERROR] Cache directory not found: /var/cache/ripple-env
[ERROR] Please run prepare-offline.sh first or provide --bundle
```

### Bundle File Missing
**Symptom:** Specified bundle file doesn't exist
**Exit Code:** 1
**Recovery:** Verify bundle path
**Evidence:** lines 110-113

### Docker Not Available
**Symptom:** Docker command not found
**Exit Code:** 1
**Recovery:** Install Docker (requires internet - must be done before air-gap)
**Evidence:** lines 143-146

### Registry Image Missing
**Symptom:** registry:2 image not in cache, container startup fails
**Exit Code:** Non-fatal warning
**Recovery:** Uses existing registry image if available
**Evidence:** lines 154-161, 169-171

---

## Deployment Flow

### Phase 1: Initialization
**Function:** `parse_args` (lines 51-101)
- Parse command-line arguments
- Set action flags

### Phase 2: Bundle Extraction (Optional)
**Function:** `extract_bundle` (lines 108-125)
- Extract offline bundle tarball to cache directory
- Evidence: line 121

### Phase 3: Cache Verification
**Function:** `verify_cache` (lines 128-136)
- Check cache directory exists
- Evidence: lines 129-133

### Phase 4: Registry Setup (Optional/All)
**Function:** `setup_registry` (lines 139-174)
1. Check Docker availability
2. Check if registry already running
3. Load registry image from cache (if available)
4. Start registry container on port 5000

### Phase 5: Image Import (Optional/All)
**Function:** `import_images` (lines 177-216)
1. Load Docker image tarballs
2. Tag images for local registry
3. Push to local registry

### Phase 6: Nix Configuration (Optional/All)
**Function:** `configure_nix` (lines 219-263)
1. Verify Nix cache exists
2. Backup existing nix.conf
3. Write offline configuration

### Phase 7: Pixi Configuration (Optional/All)
**Function:** `configure_pixi` (lines 266-302)
1. Backup existing pixi config
2. Write offline configuration with channel mirrors

### Phase 8: Environment Configuration (All Only)
**Function:** `configure_environment` (lines 305-341)
- Create offline.env with all offline environment variables

### Phase 9: Verification (Optional/All)
**Function:** `verify_setup` (lines 344-400)
- Check cache contents
- Test network isolation
- Report summary

---

## Configuration Files Generated

### Nix Configuration
**File:** `~/.config/nix/nix.conf`
**Evidence:** lines 241-259

```ini
# Use local binary cache only
substituters = file:///var/cache/ripple-env/nix-store

# Trust the local cache
require-sigs = false

# Disable network access hints
connect-timeout = 1

# Experimental features (for flakes)
experimental-features = nix-command flakes
```

### Pixi Configuration
**File:** `~/.config/pixi/config.toml`
**Evidence:** lines 283-295

```toml
[mirrors]
# Use local channel mirrors
"https://conda.anaconda.org/conda-forge" = ["file:///var/cache/ripple-env/conda-channels/conda-forge"]
"https://conda.anaconda.org/robostack-humble" = ["file:///var/cache/ripple-env/conda-channels/robostack-humble"]

[network]
# Disable network access
offline = true
```

### Offline Environment File
**File:** `~/.config/ripple-env/offline.env`
**Evidence:** lines 311-337

```bash
# Enable offline mode
export RIPPLE_OFFLINE=1
export RIPPLE_CACHE_DIR="/var/cache/ripple-env"

# Nix offline
export NIX_CONFIG="substituters = file:///var/cache/ripple-env/nix-store"

# Pixi/Conda offline
export PIXI_OFFLINE=1
export CONDA_OFFLINE=1

# HuggingFace offline
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
export HF_HOME="/var/cache/ripple-env/models/huggingface"

# Docker offline hints
export DOCKER_BUILDKIT=0

# Disable telemetry
export DO_NOT_TRACK=1
export PIXI_NO_UPDATE_CHECK=1
```

---

## Verification Checks

**Function:** `verify_setup` (lines 344-400)

### Checks Performed:
1. **Nix cache** - Count .narinfo files (lines 350-357)
2. **Docker images** - Count .tar files (lines 360-367)
3. **Conda channels** - List channel directories (lines 370-375)
4. **Model cache** - Check size (lines 378-384)
5. **Network isolation** - Test connectivity (fails as expected) (lines 387-392)

**Output Example:**
```
[INFO] Nix cache: 1234 packages available
[INFO] Docker images: 45 images cached
[INFO] Conda channels: 2 channels
[INFO] Model cache: 15G total
[INFO] Network isolated (as expected for air-gapped)
[SUCCESS] Offline setup verification complete
```

---

## Local Docker Registry

**Container Name:** `ripple-registry`
**Image:** `registry:2`
**Port:** `5000`
**Volume:** `$CACHE_DIR/docker-registry:/var/lib/registry`
**Restart Policy:** `always`

**Evidence:** lines 164-173

**Image Naming Convention:**
```
Original: ubuntu:22.04
Tagged:   localhost:5000/ubuntu:22.04
```

**Evidence:** lines 208-211

---

## Activation Instructions

Displayed at completion (lines 440-445):

```bash
# Activate offline mode
source ~/.config/ripple-env/offline.env

# Then start development
cd $PROJECT_ROOT
nix develop
```

---

## Error Handling

### Exit on Error
**Evidence:** line 21
```bash
set -euo pipefail
```

**Behavior:**
- `-e`: Exit on command failure
- `-u`: Error on undefined variables
- `-o pipefail`: Pipeline fails if any command fails

### Non-Fatal Warnings
- Registry image missing (line 160)
- Registry startup failure (line 170)
- Image load failures (line 194)
- Cache component missing (lines 355, 365, 373)

---

## References

### Source Code
- **Main script:** `scripts/deploy-airgapped.sh` (449 lines)
- **Bundle extraction:** lines 108-125
- **Registry setup:** lines 139-174
- **Image import:** lines 177-216
- **Nix configuration:** lines 219-263
- **Pixi configuration:** lines 266-302
- **Environment setup:** lines 305-341
- **Verification:** lines 344-400

### Related Files
- **Preparation script:** `scripts/prepare-offline.sh`
- **Nix config:** `~/.config/nix/nix.conf`
- **Pixi config:** `~/.config/pixi/config.toml`
- **Offline env:** `~/.config/ripple-env/offline.env`

### Related Documentation
- Air-gapped deployment guide (referenced)
- Offline cache preparation (referenced)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 6/60 contracts complete
