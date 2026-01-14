# Compatibility Matrix

Version compatibility reference for Python, PyTorch, ROS2, and Nix ecosystem.

**Purpose**: Single source of truth for version compatibility across the stack
**Status**: Phase 8 Complete
**Last Updated**: 2026-01-14

---

## Python Version Compatibility

### Current Python Versions

| Environment | Python Version | Source | Purpose |
|-------------|---------------|--------|---------|
| **Nix Scripts** | 3.13.x | `nixpkgs.python313` | Scripts, tools, utilities |
| **ROS2 + ML** | 3.11.x | Pixi/RoboStack | ROS2, PyTorch, ML stack |

**Evidence**: Dual architecture documented in `docs/CONFLICTS.md` lines 20-43

### Package Compatibility Table

| Package | 3.11 | 3.12 | 3.13 | Notes |
|---------|------|------|------|-------|
| **ROS2 Humble (RoboStack)** | ✅ | ⚠️ | ❌ | RoboStack requires 3.11.x |
| **ROS2 Humble (from source)** | ✅ | ✅ | ⚠️ | Build from source supports 3.12+ |
| **AIOS** | ✅ | ❌ | ❌ | Requires Python 3.10-3.11 exactly |
| **PyTorch 2.5** | ✅ | ✅ | ⚠️ | Official wheels for 3.9-3.12 |
| **NumPy 1.24.x** | ✅ | ✅ | ✅ | All versions supported |
| **NumPy 2.x** | ❌ | ❌ | ❌ | Incompatible with ROS2/ML stack |

**Legend**:
- ✅ Fully Supported
- ⚠️ Partial Support (may require building from source)
- ❌ Not Supported

**Evidence**:
- RoboStack constraint: `docs/CONFLICTS.md` lines 45-50
- AIOS constraint: `docs/CONFLICTS.md` line 120
- NumPy 2.x issue: `docs/CONFLICTS.md` line 464

---

## PyTorch Coupling Matrix

**CRITICAL**: PyTorch, torchvision, and torchaudio versions MUST match to avoid runtime errors.

### Version Coupling Table

| PyTorch | torchvision | torchaudio | Python | Status |
|---------|-------------|------------|--------|--------|
| **2.5.1** | 0.20.1 | 2.5.1 | 3.9-3.12 | ✅ Current |
| **2.5.0** | 0.20.0 | 2.5.0 | 3.9-3.12 | ✅ Supported |
| **2.4.1** | 0.19.1 | 2.4.1 | 3.9-3.12 | ✅ Supported |
| **2.4.0** | 0.19.0 | 2.4.0 | 3.9-3.12 | ✅ Supported |
| **2.3.1** | 0.18.1 | 2.3.1 | 3.8-3.11 | ⚠️ Older |
| **2.3.0** | 0.18.0 | 2.3.0 | 3.8-3.11 | ⚠️ Older |

**Rule**: Always upgrade all three packages together. Use `./scripts/upgrade-python-deps.sh pytorch <version>`.

**Evidence**: `pixi.toml` lines 138-145

### Breaking Combinations (DO NOT USE)

| PyTorch | torchvision | torchaudio | Issue |
|---------|-------------|------------|-------|
| 2.5.x | 0.19.x | * | ❌ Version mismatch runtime error |
| 2.4.x | 0.20.x | * | ❌ Version mismatch runtime error |
| * | * | mismatched | ❌ Audio processing fails |

---

## Nix Channel Compatibility

### Available Channels

| Channel | Version | Stability | Update Frequency | Use Case |
|---------|---------|-----------|------------------|----------|
| **nixpkgs-stable** | 24.11 | High | Monthly | Production deployments |
| **nixpkgs** | unstable | Medium | Daily | Development, latest features |
| **nixpkgs** | 24.05 | High | Security only | Previous LTS (EOL soon) |

**Current Strategy**: Dual-track approach

```nix
# flake.nix
inputs = {
  nixpkgs-stable.url = "github:NixOS/nixpkgs/nixos-24.11";
  nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
}
```

**Evidence**: `docs/CONFLICTS.md` lines 451-455

### Nix Version Requirements

| Nix Version | Features | Status |
|-------------|----------|--------|
| **2.18+** | Flakes stable | ✅ Recommended |
| 2.13-2.17 | Flakes experimental | ⚠️ Works with warnings |
| <2.13 | No flakes | ❌ Unsupported |

---

## ROS2 Compatibility

### Current Distribution

| ROS2 Version | Ubuntu | Python | Status | Support Until |
|--------------|--------|--------|--------|---------------|
| **Humble** | 22.04 | 3.10-3.11 | ✅ Current (LTS) | May 2027 |
| Jazzy | 24.04 | 3.12+ | ⚠️ Testing | May 2029 |
| Iron | 22.04 | 3.10-3.11 | ⚠️ Non-LTS | Nov 2024 |
| Galactic | 20.04 | 3.8-3.9 | ❌ EOL | Nov 2022 |

**Current Choice**: Humble via RoboStack (conda-forge packages)

**Evidence**: ROS2 Humble documented as current in `docs/CONFLICTS.md` lines 7-27

### RoboStack Package Availability

| Package | Python 3.11 | Python 3.12 | Python 3.13 |
|---------|-------------|-------------|-------------|
| ros-humble-rclpy | ✅ | ⚠️ | ❌ |
| ros-humble-std-msgs | ✅ | ⚠️ | ❌ |
| ros-humble-geometry-msgs | ✅ | ⚠️ | ❌ |
| ros-humble-sensor-msgs | ✅ | ⚠️ | ❌ |

**⚠️ Note**: Python 3.12 support requires building from source. Python 3.13 not yet available.

---

## Conda-forge / Pixi Compatibility

### Pixi Version Requirements

| Pixi Version | Features | Status |
|--------------|----------|--------|
| **0.40+** | Multi-environment support | ✅ Current |
| 0.30-0.39 | Basic features | ⚠️ Works |
| <0.30 | Limited | ❌ Unsupported |

**Current Version**: Latest via Nix (`pkgs.pixi`)

**Evidence**: `pixi.toml` uses modern multi-environment syntax

---

## AIOS Strict Dependencies

AIOS requires exact versions that cannot be upgraded freely:

| Package | Required Version | Constraint Reason |
|---------|-----------------|-------------------|
| **pydantic** | ==2.7.0 | Breaking changes in 2.8+ (field validation) |
| **numpy** | ==1.24.3 | NumPy 2.x incompatible |
| **Python** | 3.10-3.11 | AIOS not tested with 3.12+ |

**Evidence**: `docs/CONFLICTS.md` line 120, `pixi.toml` AIOS section

**Impact**: AIOS pins cannot be upgraded without extensive testing.

---

## Known Conflicts and Resolutions

### Python 3.11 (RoboStack) vs 3.13 (Nix Scripts)

**Conflict**: RoboStack requires 3.11, but Python 3.13 has performance improvements

**Resolution**: Dual Python architecture
- Python 3.13 for Nix scripts (`python3.13`)
- Python 3.11 for ROS2/PyTorch (Pixi's `python`)

**Evidence**: `docs/CONFLICTS.md` lines 20-43

### NumPy 2.x Incompatibility

**Conflict**: NumPy 2.x breaks ROS2 and many ML packages

**Resolution**: Pin NumPy <2.0

```toml
numpy = "<2.0"
```

**Evidence**: `docs/CONFLICTS.md` line 464

### AIOS pydantic ==2.7.0

**Conflict**: AIOS breaks with pydantic >=2.8 due to field validation changes

**Resolution**: Exact pin

```toml
pydantic = "==2.7.0"
```

**Evidence**: `docs/CONFLICTS.md` line 120

### PyTorch Version Coupling

**Conflict**: Mismatched PyTorch/torchvision/torchaudio versions cause runtime errors

**Resolution**: Use automated upgrade script

```bash
./scripts/upgrade-python-deps.sh pytorch 2.5.1
```

**Evidence**: `pixi.toml` lines 135-145

---

## Upgrade Paths

### Python 3.11 → 3.12

**Current Blockers**:
- RoboStack packages not available for Python 3.12

**Upgrade Path**:
1. Wait for RoboStack to publish Python 3.12 builds, OR
2. Build ROS2 from source with Python 3.12, OR
3. Use alternative ROS2 distribution (Jazzy)

**Timeline**: Unknown (depends on RoboStack)

**Evidence**: `docs/CONFLICTS.md` lines 45-66

### NumPy 1.x → 2.x

**Current Blockers**:
- ROS2 packages incompatible
- Many ML libraries not updated

**Upgrade Path**:
1. Wait for ROS2 NumPy 2.x compatibility
2. Wait for ML ecosystem (pandas, scikit-learn) to update
3. Review NumPy 2.0 migration guide
4. Test extensively

**Timeline**: 2026+ (ecosystem-wide effort)

**Reference**: https://numpy.org/devdocs/numpy_2_0_migration_guide.html

### ROS2 Humble → Jazzy

**Breaking Changes**:
- Python 3.12+ required (vs 3.10-3.11)
- API changes (consult migration guide)
- RoboStack support unknown

**Upgrade Path**:
1. Review Jazzy release notes
2. Update `pixi.toml` to use robostack-jazzy channel
3. Update all ros-humble-* packages to ros-jazzy-*
4. Rebuild workspace: `colcon build --symlink-install`
5. Run tests: `colcon test`

**Timeline**: When Jazzy reaches maturity (2026+)

**Evidence**: `docs/cookbooks/MAJOR_VERSION_UPGRADE.md`

---

## Testing Compatibility

### Validation Commands

```bash
# Verify Python versions
python --version           # Should show 3.11.x (Pixi)
python3.13 --version       # Should show 3.13.x (Nix)

# Verify PyTorch coupling
pixi list | grep -E "pytorch|torchvision|torchaudio"

# Verify ROS2
pixi run python -c "import rclpy; print('ROS2 OK')"

# Verify NumPy version
pixi run python -c "import numpy; print(numpy.__version__)"  # Should be <2.0

# Run full E2E validation
./scripts/validate-e2e.sh
```

**Evidence**: `scripts/validate-e2e.sh` exists

---

## Related Docs

- [Conflicts and Compatibility](CONFLICTS.md) - Detailed conflict analysis
- [Version Pinning Policy](cookbooks/VERSION_PINNING_POLICY.md) - When to pin versions
- [Dependency Updates](cookbooks/DEPENDENCY_UPDATES.md) - Update procedures
- [Major Version Upgrades](cookbooks/MAJOR_VERSION_UPGRADE.md) - Upgrade paths
- [Python Environments](PYTHON-ENVIRONMENTS.md) - Dual Python architecture
