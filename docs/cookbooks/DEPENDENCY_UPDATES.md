# Dependency Updates

Weekly and monthly routines for updating Nix flake inputs, Pixi packages, and Python dependencies while maintaining PyTorch version coupling.

## What's in the repo (evidence)

- Nix flake inputs: `flake.nix`, `flake.lock`
- Pixi packages: `pixi.toml`, `pixi.lock`
- Python dependency automation: `scripts/upgrade-python-deps.sh`
- Dependency checks: `scripts/check-python-deps.sh`
- PyTorch coupling table: `pixi.toml` lines 138-145
- Conflict documentation: `docs/CONFLICTS.md`

## Goal

1. Keep dependencies up-to-date for security patches
2. Maintain PyTorch version coupling (pytorch/torchvision/torchaudio must match)
3. Test updates before committing
4. Provide easy rollback if updates break functionality

## Quick start

### Weekly Routine (Low-Risk Updates)

Update lockfiles to get latest patches within pinned version ranges:

```bash
# Update both Nix and Pixi lockfiles
nix flake update && pixi update

# Verify no breakage
pixi install --frozen
nix develop --command echo "Nix shell works"

# Test ROS2 + PyTorch
pixi run python -c "import rclpy, torch; print('OK')"

# If all passes, commit
git add flake.lock pixi.lock
git commit -m "chore: update dependency locks (weekly)"
```

**Frequency**: Weekly
**Risk**: Low (only updates within existing version constraints)
**Evidence**: Standard practice for `flake.lock` and `pixi.lock`

### Monthly Routine (Version Upgrades)

Check for new major/minor versions and upgrade if safe:

```bash
# 1. Check for outdated Pixi packages
pixi list --outdated

# 2. Check Python dependency status
./scripts/check-python-deps.sh

# 3. Review PyTorch coupling requirements
cat pixi.toml | grep -A 10 "Version Coupling Table"

# 4. If PyTorch update available, use automated script
./scripts/upgrade-python-deps.sh pytorch 2.5.1

# 5. For other packages, update pixi.toml manually
$EDITOR pixi.toml

# 6. Regenerate locks
pixi update

# 7. Full validation
./scripts/validate-e2e.sh
```

**Frequency**: Monthly
**Risk**: Medium (may introduce breaking changes)
**Evidence**: `scripts/check-python-deps.sh` and `scripts/upgrade-python-deps.sh` exist

### PyTorch Coupling Updates (CRITICAL)

PyTorch versions MUST be upgraded together to avoid runtime errors:

```bash
# CORRECT: Use automated script that handles coupling
./scripts/upgrade-python-deps.sh pytorch 2.5.1

# This script will automatically update:
# - pytorch = "2.5.1"
# - torchvision = "0.20.1"  (matches 2.5.x)
# - torchaudio = "2.5.1"     (matches 2.5.x)
```

**Version Coupling Table** (from `pixi.toml:138-145`):

| PyTorch | torchvision | torchaudio |
|---------|-------------|------------|
| 2.5.x   | 0.20.x      | 2.5.x      |
| 2.4.x   | 0.19.x      | 2.4.x      |
| 2.3.x   | 0.18.x      | 2.3.x      |

**WRONG**: Manual updates without coupling check:
```bash
# DON'T DO THIS - will cause runtime errors!
pixi add pytorch==2.5.1
# (forgets to update torchvision and torchaudio)
```

**Evidence**: `pixi.toml` lines 135-145 document coupling requirement

### Nix Flake Input Updates

Update specific Nix flake inputs (e.g., nixpkgs, home-manager):

```bash
# Update all inputs
nix flake update

# Update specific input only
nix flake lock --update-input nixpkgs
nix flake lock --update-input home-manager

# Test the update
nix flake check
nix develop --command python3 --version

# Commit if successful
git add flake.lock
git commit -m "chore: update nixpkgs to 24.11 stable"
```

**Frequency**: Monthly for stable channel, weekly for unstable
**Evidence**: `flake.nix` defines multiple inputs (nixpkgs, home-manager, etc.)

## Testing After Updates

Always run validation before committing updates:

```bash
# 1. Clean install to catch issues
rm -rf .pixi
pixi install --frozen

# 2. Run ROS2 package tests
colcon test
colcon test-result --verbose

# 3. Run end-to-end validation
./scripts/validate-e2e.sh

# 4. Check resource requirements haven't changed
./scripts/validate-resources.sh

# 5. Test PyTorch CUDA if using GPU
pixi run -e cuda python -c "import torch; print(torch.cuda.is_available())"
```

**Exit codes**:
- `0` = Safe to commit
- `1` = Warnings (review before committing)
- `2` = Critical failures (rollback)

**Evidence**: `scripts/validate-e2e.sh` and `scripts/validate-resources.sh` exist

## Rollback Procedure

If updates break functionality:

```bash
# Rollback lockfiles to previous commit
git restore flake.lock pixi.lock

# Reinstall dependencies
pixi install --frozen
nix develop --command echo "Rollback complete"

# Or rollback to specific commit
git checkout HEAD~1 -- flake.lock pixi.lock
```

**Prevention**: Always test in dev/staging before updating production.

## Special Cases

### AIOS Strict Pinning

AIOS requires exact versions (cannot be updated freely):

```toml
# From pixi.toml - DO NOT change without testing
pydantic = "==2.7.0"
numpy = "==1.24.3"
```

**Reason**: AIOS breaks with pydantic >=2.8 and numpy >=2.0
**Evidence**: `docs/CONFLICTS.md` line 120, `pixi.toml` AIOS section

### RoboStack Python Constraint

ROS2 via RoboStack requires Python 3.11.x:

```bash
# This is locked - don't upgrade to 3.12 yet
python = "3.11.*"
```

**Upgrade path**: Build ROS2 from source OR wait for RoboStack 3.12 support
**Evidence**: `docs/CONFLICTS.md` lines 45-50

### NumPy 2.x Incompatibility

NumPy must stay < 2.0 for ROS2 compatibility:

```toml
numpy = "<2.0"
```

**Evidence**: `docs/CONFLICTS.md` line 464

## Automation Hooks

Consider adding to CI/CD:

```yaml
# .github/workflows/dependency-check.yml (example)
name: Weekly Dependency Check
on:
  schedule:
    - cron: '0 0 * * 1'  # Every Monday
jobs:
  check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: pixi list --outdated
      - run: ./scripts/check-python-deps.sh
```

**Note**: This workflow is an example - verify `.github/workflows/` for actual CI setup.

## Related docs

- [Conflicts and Compatibility](../CONFLICTS.md) - Version coupling rules
- [Version Pinning Policy](VERSION_PINNING_POLICY.md) - When to pin vs use ranges
- [Migration Guides](../MIGRATION_GUIDES.md) - Major version upgrade procedures
- [Python Environments](../PYTHON-ENVIRONMENTS.md) - Dual Python architecture
