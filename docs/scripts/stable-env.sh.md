# Script Contract: stable-env.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/stable-env.sh`

---

## Purpose

Stable environment setup for WSL. Sets conservative resource limits, memory optimizations, and creates safe command wrappers with timeouts to prevent WSL crashes and reloads. Essential for WSL2 stability.

---

## Invocation

```bash
source scripts/stable-env.sh
```

**Note:** Must be sourced (not executed) to export variables and aliases to current shell.

---

## Outputs

**Standard Output:**
```
🛡️ Loading WSL-stable environment...
✅ WSL-stable environment loaded!
   • NIX_BUILD_CORES: 2 (limited for WSL)
   • NIX_MAX_JOBS: 2 (limited for WSL)
   • Memory optimizations: ENABLED
   • Safe commands: pixi-safe, nix-safe, docker-safe
   • Session tools: ripple-save, ripple-restore
   • Environment variables: Loaded (use 'load-env' to reload)

💡 Use 'pixi-safe install' instead of 'pixi install'
```

---

## Side Effects

### Environment Variables Set

**Build Parallelism Limits (WSL stability):**
```bash
NIX_BUILD_CORES=2          # Max 2 cores for Nix builds
NIX_MAX_JOBS=2             # Max 2 parallel Nix jobs
CMAKE_BUILD_PARALLEL_LEVEL=2  # Max 2 parallel CMake jobs
```
**Evidence:** Lines 8-10

**Memory Optimizations:**
```bash
MALLOC_ARENA_MAX=2         # Limit glibc memory arenas
MALLOC_MMAP_THRESHOLD_=131072  # 128KB mmap threshold
MALLOC_TRIM_THRESHOLD_=131072  # 128KB trim threshold
MALLOC_TOP_PAD_=131072         # 128KB top pad
```
**Evidence:** Lines 13-16

**WSL-Specific:**
```bash
WSLENV="NIX_BUILD_CORES/w:NIX_MAX_JOBS/w"  # Pass to Windows
LD_LIBRARY_PATH="/usr/lib/wsl/lib:..."    # NVIDIA WSL GPU libraries
```
**Evidence:** Lines 19, 28

**PATH:**
```bash
PATH="/usr/lib/wsl/lib:/usr/local/bin:/usr/bin:/bin:/home/nixos/.nix-profile/bin:$PATH"
```
**Evidence:** Line 22

### Functions Created

**timeout_pixi(args)** - Pixi with 300s timeout (lines 41-48)
- Adds `--frozen` flag to `install` command
- Prevents lockfile updates during install

**timeout_nix(args)** - Nix with 600s timeout (line 49)

**timeout_docker(args)** - Docker with 120s timeout (line 50)

### Aliases Created

```bash
pixi-safe='timeout_pixi'
nix-safe='timeout_nix'
docker-safe='timeout_docker'
ripple-save='scripts/session-save.sh'
ripple-restore='scripts/session-restore.sh'
load-env='source scripts/env-vars.sh'
```
**Evidence:** Lines 53-58

### Sourced Files

1. **scripts/env-vars.sh** - AI/ML environment variables (lines 31-33)
2. **~/.nix-profile/etc/profile.d/nix.sh** - Nix profile (lines 36-38)

---

## Safety Classification

**🟢 SAFE** - Environment configuration only, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be sourced repeatedly. Last sourcing wins for env vars and aliases.

---

## Key Features

### 1. Resource Limits

**Purpose:** Prevent WSL crashes from memory pressure and excessive parallelism.

**NIX_BUILD_CORES=2:** Limits Nix to 2 CPU cores (line 8)
**NIX_MAX_JOBS=2:** Limits parallel Nix jobs to 2 (line 9)
**CMAKE_BUILD_PARALLEL_LEVEL=2:** Limits CMake parallelism (line 10)

### 2. Memory Optimization

**Purpose:** Reduce glibc memory pressure in WSL.

**MALLOC_ARENA_MAX=2:** Limits per-thread memory arenas (line 13)
- Reduces fragmentation
- Lowers memory usage

**Thresholds:** 128KB for mmap, trim, top pad (lines 14-16)
- Aggressive memory reclamation
- Prevents bloat

### 3. NVIDIA GPU Support

**Purpose:** Make NVIDIA WSL2 GPU libraries discoverable for CUDA and nvidia-smi.

```bash
LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```
**Evidence:** Line 28

**Note:** WSL2 Windows NVIDIA driver exposes user-space libraries at `/usr/lib/wsl/lib`. NixOS doesn't run ldconfig, so explicit LD_LIBRARY_PATH is required.

### 4. Safe Command Wrappers

**pixi-safe:** 300-second timeout, adds `--frozen` to install (lines 41-48)
```bash
timeout_pixi() {
    if [[ "$1" == "install" ]]; then
        timeout 300 pixi install --frozen "${@:2}"
    else
        timeout 300 pixi "$@"
    fi
}
```

**nix-safe:** 600-second timeout (line 49)

**docker-safe:** 120-second timeout (line 50)

---

## Usage Patterns

### Standard Workflow

```bash
# 1. Load stable environment
source scripts/stable-env.sh

# 2. Use safe commands with timeout protection
pixi-safe install --skip vectordb-ruvector
nix-safe develop
docker-safe system prune -f

# 3. Save session before risky operations
ripple-save

# 4. After WSL reload (if needed)
ripple-restore
```

### Why --frozen for pixi install?

**Evidence:** Line 44

Prevents lockfile updates that can:
- Trigger long dependency resolution (timeouts)
- Cause WSL hangs
- Introduce instability

Use `pixi update` explicitly when you want lockfile changes.

---

## Integration

**Loaded by:** bootstrap.ps1 WSL stability guide (referenced in modified version)
**Used with:** session-save.sh, session-restore.sh
**Depends on:** scripts/env-vars.sh (optional, lines 31-33)

---

## References

### Source Code
- **Main script:** `scripts/stable-env.sh` (68 lines)
- **Resource limits:** lines 8-16
- **GPU libraries:** line 28
- **Safe wrappers:** lines 41-50
- **Aliases:** lines 53-58

### Related Files
- **AI/ML vars:** `scripts/env-vars.sh`
- **Session tools:** `scripts/session-save.sh`, `scripts/session-restore.sh`
- **Windows guide:** Referenced in bootstrap.ps1 modifications

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 14/60 contracts complete
