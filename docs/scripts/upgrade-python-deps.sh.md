# Script Contract: upgrade-python-deps.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/upgrade-python-deps.sh`

---

## Purpose

Python dependency upgrade advisor that provides version coupling tables and upgrade instructions. Shows PyTorch ecosystem coupling requirements (torchvision/torchaudio), Python version constraints by component (RoboStack 3.11, AIOS 3.10-3.11), and generates pixi.toml update commands. Read-only analysis tool with no automatic upgrades.

---

## Invocation

```bash
./scripts/upgrade-python-deps.sh [command] [options]
```

**Commands:**
- `check` (default) - Check PyTorch upgrades
- `pytorch <version>` - Show upgrade instructions for specific PyTorch version
- `python` - Show Python upgrade paths
- `all` - Run all checks

**Examples:**
```bash
./scripts/upgrade-python-deps.sh check
./scripts/upgrade-python-deps.sh pytorch 2.6
./scripts/upgrade-python-deps.sh python
./scripts/upgrade-python-deps.sh all
```

---

## Side Effects

**None** - Read-only analysis, no file modifications.

---

## Safety Classification

**🟢 SAFE** - Read-only recommendations.

---

## Key Features

### PyTorch Coupling Table (lines 39-46)
```bash
declare -A PYTORCH_COUPLING=(
    ["2.6"]="torchvision:0.21 torchaudio:2.6"
    ["2.5"]="torchvision:0.20 torchaudio:2.5"
    ["2.4"]="torchvision:0.19 torchaudio:2.4"
)
```

### Python Constraints (lines 49-54)
- RoboStack Humble: 3.11 only (conda-forge)
- AIOS: 3.10-3.11 (AGiXT constraint)
- PyTorch: 3.9-3.12
- Transformers: 3.9-3.12

### Upgrade Instructions (lines 143-164)
Generates pixi.toml snippets with version ranges and CUDA build variants.

---

## References

- **Main script:** `scripts/upgrade-python-deps.sh` (263 lines)
- **Coupling table:** lines 39-46
- **Constraints:** lines 49-54
- **Related:** `check-python-deps.sh` (validation companion)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 49/60 (81.7%)
