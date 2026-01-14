# Script Contract: fix-wsl-stability.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/fix-wsl-stability.sh`

---

## Purpose

Comprehensive WSL2 stability fix script that addresses common memory pressure, resource contention, and hanging issues. Creates helper scripts (`stable-env.sh`, `session-save.sh`, `session-restore.sh`), configures resource limits for Nix/Docker/Pixi, sets up bash aliases, and generates `.wslconfig` template. Designed for WSL2 environments running NixOS with heavy workloads.

---

## Invocation

```bash
./scripts/fix-wsl-stability.sh
```

**No arguments** - Automatically detects WSL and applies fixes.

**Exit Codes:**
- `0` - Success (fixes applied or not WSL)

---

## Side Effects

### Files Created
- `scripts/stable-env.sh` - Resource-limited environment (lines 92-127)
- `scripts/session-save.sh` - Session state persistence (lines 133-150)
- `scripts/session-restore.sh` - Session recovery (lines 156-181)
- `/tmp/.wslconfig` - WSL configuration template (lines 214-225)
- `/etc/wsl.conf` - WSL2 limits (appended, lines 27-31)
- `/etc/docker/daemon.json` - Docker config (lines 40-56)
- `~/.config/pixi/config.toml` - Pixi config (lines 76-87)

### Configuration Changes
- Appends to `~/.bashrc` - Nix limits + aliases (lines 67-68, 190-207)

---

## Safety Classification

**🟡 CAUTION** - Creates files, modifies system configs, requires sudo.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT** - Appends to files (may create duplicates on re-run).

---

## Key Fixes Applied

### Fix 1: Memory Limits (lines 24-31)
```bash
echo "[wsl2]" | sudo tee -a /etc/wsl.conf
echo "memory=8GB" | sudo tee -a /etc/wsl.conf
echo "processors=4" | sudo tee -a /etc/wsl.conf
echo "swap=2GB" | sudo tee -a /etc/wsl.conf
```

### Fix 2: Docker Config (lines 40-52)
```json
{
  "default-ulimits": {"nofile": {"Soft": 65536, "Hard": 65536}},
  "max-concurrent-downloads": 3,
  "max-concurrent-uploads": 2,
  "storage-driver": "overlay2"
}
```

### Fix 3: Nix Limits (lines 63-68)
```bash
export NIX_BUILD_CORES=2
export NIX_MAX_JOBS=2
```

### Fix 4: Pixi Config (lines 76-87)
```toml
[solve_strategy]
max_attempts = 3
timeout = 300

[repodata]
fetch_threads = 2
```

### Fix 5: Stable Environment Script (lines 92-127)
Sets conservative resource limits, memory optimization, WSL-specific env vars.

### Fix 6: Session Persistence (lines 133-150)
Saves environment variables, working directory, command history to `~/.ripple-sessions/`.

### Fix 7: Session Recovery (lines 156-181)
Restores last directory and loads stable environment after WSL reload.

---

## Bash Aliases Created (lines 190-207)

```bash
alias ripple-enter='source scripts/stable-env.sh && echo "✅ Entered stable ripple environment"'
alias ripple-save='scripts/session-save.sh'
alias ripple-restore='scripts/session-restore.sh'
alias pixi-safe='timeout 300 pixi'
alias nix-safe='timeout 600 nix'
alias docker-safe='timeout 120 docker'
alias pixi-install='pixi-safe install --no-update-lockfile'
alias nix-build='nix-safe build --max-jobs 2 --cores 2'
alias nix-develop='nix-safe develop --max-jobs 2 --cores 2'
```

---

## WSL Configuration Template (lines 214-225)

```ini
[wsl2]
memory=8GB
processors=4
swap=2GB
localhostForwarding=true
nestedVirtualization=false

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

**Installation:**
```bash
cp /tmp/.wslconfig /mnt/c/Users/$USER/.wslconfig
wsl --shutdown
```

---

## References

- **Main script:** `scripts/fix-wsl-stability.sh` (262 lines)
- **Created scripts:** `stable-env.sh` (lines 92-127), `session-save.sh` (lines 133-150), `session-restore.sh` (lines 156-181)
- **Related:** `docs/TROUBLESHOOTING.md` (WSL issues section)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 45/60 (75%)
