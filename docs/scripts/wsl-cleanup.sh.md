# Script Contract: wsl-cleanup.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/wsl-cleanup.sh`

---

## Purpose

Disk cleanup utility for WSL2 environments that frees space from Nix store, Docker, Pixi cache, and build artifacts. Supports selective cleanup, aggressive mode (removes all generations/volumes), dry-run mode, and disk usage reporting. Tracks freed space and provides before/after comparisons.

---

## Invocation

```bash
./scripts/wsl-cleanup.sh [OPTIONS]
```

**Options:**
- `--all` - Run all cleanup operations
- `--nix` - Clean Nix store only
- `--docker` - Clean Docker only
- `--pixi` - Clean Pixi cache only
- `--build` - Clean build artifacts only
- `--aggressive` - More aggressive cleanup (removes all generations/volumes)
- `--dry-run` - Show what would be cleaned
- `--report` - Show disk usage report only
- `-h`, `--help` - Show help

**Examples:**
```bash
./scripts/wsl-cleanup.sh --report              # Disk report
./scripts/wsl-cleanup.sh --all                 # Full cleanup
./scripts/wsl-cleanup.sh --all --aggressive    # Aggressive cleanup
./scripts/wsl-cleanup.sh --nix --docker        # Nix + Docker
./scripts/wsl-cleanup.sh --all --dry-run       # Preview
```

---

## Side Effects

### Nix Store (lines 96-132)
- **Standard:** `nix store gc` + `nix store optimise`
- **Aggressive:** `nix-collect-garbage -d` (removes all old generations)

### Docker (lines 134-168)
- **Standard:** `docker system prune -a -f` (unused images/containers)
- **Aggressive:** `docker system prune -a -f --volumes` (includes volumes)

### Pixi Cache (lines 170-199)
- Runs `pixi clean`

### Build Artifacts (lines 201-245)
- Removes `build/`, `install/`, `log/` directories
- Clears `ccache` if present

---

## Safety Classification

**🔴 DESTRUCTIVE** (aggressive mode) - Can delete volumes, generations, build artifacts.

---

## Idempotency

**❌ NON-IDEMPOTENT** - Deletes data permanently.

---

## Key Features

### Disk Usage Report (lines 57-94)
```
=========================================
        WSL2 Disk Usage Report
=========================================

Filesystem Overview:
  Total: 250G, Used: 180G (72%), Available: 70G

Component Breakdown:
  Nix Store:                15G
  Pixi Cache:               8G
  Home Directory:           45G
  Build Artifacts:          2G
  AI Models:                30G
```

### Before/After Tracking (lines 106-131)
```bash
before=$(du -sb /nix/store | cut -f1)
# ... cleanup ...
after=$(du -sb /nix/store | cut -f1)
freed_mb=$(((before - after) / 1024 / 1024))
log_success "Freed ${freed_mb} MB from Nix store"
```

---

## References

- **Main script:** `scripts/wsl-cleanup.sh` (348 lines)
- **Report function:** lines 57-94
- **Nix cleanup:** lines 96-132
- **Docker cleanup:** lines 134-168
- **Pixi cleanup:** lines 170-199
- **Build cleanup:** lines 201-245

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 46/60 (76.7%)
