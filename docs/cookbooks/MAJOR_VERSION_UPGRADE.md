# Major Version Upgrades

Procedures for upgrading Python, ROS2, Nix, and PyTorch across major versions.

## What's in the repo (evidence)

- Migration guides: `docs/MIGRATION_GUIDES.md`
- Compatibility rules: `docs/CONFLICTS.md`
- PyTorch coupling table: `pixi.toml` lines 138-145
- Automated upgrades: `scripts/upgrade-python-deps.sh`
- Validation: `scripts/validate-e2e.sh`

## Goal

1. Safely upgrade across major versions
2. Minimize downtime and breakage
3. Provide clear rollback path
4. Test upgrades before applying to production

## Quick start

### Pre-Upgrade Checklist

Before any major upgrade:

- [ ] Read CHANGELOG for breaking changes
- [ ] Backup current state (see `docs/cookbooks/BACKUP_RESTORE.md`)
- [ ] Create feature branch: `git checkout -b upgrade-<package>-<version>`
- [ ] Test in dev environment first
- [ ] Document compatibility matrix changes
- [ ] Plan rollback strategy

**Evidence**: Standard upgrade practices

### Python 3.11 → 3.12

**Current State**: Python 3.11.x (RoboStack requirement)

**Compatibility Check**:
```bash
# Check RoboStack support for Python 3.12
pixi search ros-humble-rclpy python=3.12

# Check PyTorch support
pixi search pytorch python=3.12
```

**Upgrade Procedure**:
```toml
# Edit pixi.toml
[dependencies]
python = "3.12.*"  # Changed from 3.11.*
```

```bash
# Regenerate environment
pixi update

# Test ROS2
pixi run python -c "import rclpy; print('ROS2 OK')"

# Test PyTorch
pixi run python -c "import torch; print('PyTorch OK')"

# Full validation
./scripts/validate-e2e.sh
```

**Rollback**:
```bash
git restore pixi.toml pixi.lock
pixi install --frozen
```

**Known Issues**:
- RoboStack may not support Python 3.12 yet (check compatibility)
- Build ROS2 from source as alternative
- Wait for RoboStack to publish 3.12 builds

**Evidence**: `docs/CONFLICTS.md` lines 45-50 document RoboStack constraint

### Python 3.13 Upgrade (Future)

**Current**: Nix scripts use Python 3.13, but Pixi/ROS2 use 3.11

**Strategy**: Dual Python architecture maintained
```bash
# Python 3.13 for Nix scripts (already implemented)
python3.13 --version

# Python 3.11 for ROS2/PyTorch (via Pixi)
pixi run python --version
```

**Evidence**: `docs/CONFLICTS.md` lines 20-43 document dual architecture

### ROS2 Humble → Jazzy

**Current**: ROS2 Humble (LTS until 2027)

**Major Version Upgrade Procedure**:

```bash
# 1. Check Jazzy compatibility
# Review: https://docs.ros.org/en/jazzy/Releases.html

# 2. Update RoboStack channel in pixi.toml
[tool.pixi.feature.ros]
channels = ["robostack-jazzy", "conda-forge"]

# 3. Update all ROS packages
pixi add ros-jazzy-rclpy ros-jazzy-std-msgs  # etc.

# 4. Remove Humble packages
pixi remove ros-humble-rclpy ros-humble-std-msgs  # etc.

# 5. Rebuild all ROS2 packages
colcon build --symlink-install

# 6. Update launch files (check for breaking changes)
# Review Jazzy migration guide

# 7. Run tests
colcon test
./scripts/validate-e2e.sh
```

**Rollback**:
```bash
git restore pixi.toml pixi.lock
colcon clean workspace
pixi install --frozen
colcon build --symlink-install
```

**Breaking Changes**: Review ROS2 Jazzy migration guide for API changes

**Evidence**: ROS2 Humble documented as current version in `docs/CONFLICTS.md`

### Nix 24.11 → 25.05

**Current**: nixpkgs-24.11 (stable)

**Upgrade Procedure**:

```nix
// Edit flake.nix
inputs = {
  nixpkgs-stable.url = "github:NixOS/nixpkgs/nixos-25.05";
}
```

```bash
# Update flake lock
nix flake update

# Test flake check
nix flake check

# Enter development shell
nix develop --command echo "Nix 25.05 works"

# Full rebuild
nix build

# Validate
./scripts/validate-e2e.sh
```

**Rollback**:
```bash
git restore flake.nix flake.lock
nix develop --command echo "Rollback complete"
```

**Breaking Changes**: Check Nix 25.05 release notes for:
- Deprecated options
- Package renames
- Build system changes

**Evidence**: `docs/CONFLICTS.md` lines 451-455 document Nix channel strategy

### PyTorch 2.4 → 2.5

**CRITICAL**: PyTorch upgrades MUST follow coupling rules

**Coupling Table** (from `pixi.toml:138-145`):

| PyTorch | torchvision | torchaudio |
|---------|-------------|------------|
| 2.5.x   | 0.20.x      | 2.5.x      |
| 2.4.x   | 0.19.x      | 2.4.x      |

**Automated Upgrade (RECOMMENDED)**:
```bash
# Use script that handles coupling automatically
./scripts/upgrade-python-deps.sh pytorch 2.5.1

# Verify versions match
pixi list | grep -E "pytorch|torchvision|torchaudio"

# Test
pixi run python -c "import torch; print(torch.__version__)"
./scripts/validate-e2e.sh
```

**Manual Upgrade (NOT RECOMMENDED)**:
```toml
# Edit pixi.toml - MUST update all three together
pytorch = "==2.5.1"
torchvision = "==0.20.1"  # MUST match PyTorch 2.5.x
torchaudio = "==2.5.1"    # MUST match PyTorch 2.5.x
```

**Rollback**:
```bash
./scripts/upgrade-python-deps.sh pytorch 2.4.0
pixi install --frozen
```

**Evidence**: `pixi.toml` lines 135-145, `scripts/upgrade-python-deps.sh` exists

### NumPy 1.x → 2.x

**WARNING**: NumPy 2.x is incompatible with current ROS2/ML stack

```bash
# DO NOT upgrade to NumPy 2.x yet
# Keep: numpy = "<2.0"
```

**Future Upgrade Path**:
1. Wait for ROS2 NumPy 2.x compatibility
2. Wait for ML library updates (scikit-learn, pandas, etc.)
3. Test extensively before upgrading
4. Review: https://numpy.org/devdocs/numpy_2_0_migration_guide.html

**Evidence**: `docs/CONFLICTS.md` line 464

### Kubernetes Migration (Future)

**Current**: Docker Compose deployment

**Migration Strategy** (when ready):

```bash
# 1. Review Helm chart
cat docs/HELM_CHART.md

# 2. Test locally with kind/minikube
kind create cluster
kubectl apply -f k8s/

# 3. Migrate one service at a time
# Start with stateless services (APIs)

# 4. Migrate databases last (requires data migration)

# 5. Run parallel deployments during transition
```

**Evidence**: `docs/HELM_CHART.md` and `docs/deployment/KUBERNETES_MIGRATION.md` exist

## Testing Strategy

### Test in Dev First

```bash
# 1. Create feature branch
git checkout -b upgrade-python-3.12

# 2. Make changes

# 3. Test locally
./scripts/validate-e2e.sh

# 4. Run full test suite
colcon test
colcon test-result --verbose

# 5. If passes, create PR
# 6. Test in staging before production
```

### Staging Environment

```bash
# Deploy to staging
export ENV=staging
docker compose --profile staging up -d

# Run smoke tests
./scripts/validate-e2e.sh

# Monitor for 24 hours before production
```

**Evidence**: Staging environment references in compose files

### Production Deployment

```bash
# 1. Schedule maintenance window
# 2. Notify users of downtime
# 3. Backup production data (see BACKUP_RESTORE.md)
# 4. Deploy upgrade
# 5. Validate
# 6. Monitor logs for 1 hour
# 7. Keep rollback plan ready
```

## Rollback Strategy

### Immediate Rollback (Critical Failures)

```bash
# 1. Restore lockfiles
git restore flake.lock pixi.lock

# 2. Reinstall dependencies
pixi install --frozen
nix develop

# 3. Restart services
docker compose down
docker compose up -d

# 4. Verify
./scripts/validate-e2e.sh
```

### Rollback with Data Migration

If upgrade included database schema changes:

```bash
# 1. Stop services
docker compose down

# 2. Restore database backup
docker compose exec postgres psql -U postgres < backup-pre-upgrade.sql

# 3. Restore code
git revert <upgrade-commit>

# 4. Reinstall
pixi install --frozen

# 5. Restart
docker compose up -d
```

**Evidence**: Database backup procedures in `docs/cookbooks/BACKUP_RESTORE.md`

## Breaking Changes Checklist

After major upgrades, verify:

- [ ] All tests pass: `colcon test && colcon test-result`
- [ ] E2E validation passes: `./scripts/validate-e2e.sh`
- [ ] No deprecation warnings in logs
- [ ] API compatibility maintained (check `docs/API_REFERENCE.md`)
- [ ] Documentation updated (`docs/CONFLICTS.md`, `docs/COMPATIBILITY_MATRIX.md`)
- [ ] CHANGELOG entry added
- [ ] Team notified of changes

## Related docs

- [Migration Guides](../MIGRATION_GUIDES.md) - Detailed upgrade procedures
- [Compatibility Matrix](../COMPATIBILITY_MATRIX.md) - Version compatibility tables
- [Conflicts](../CONFLICTS.md) - Known version conflicts
- [Dependency Updates](DEPENDENCY_UPDATES.md) - Regular update procedures
- [Backup & Restore](BACKUP_RESTORE.md) - Backup before upgrades
