# Version Pinning Policy

Guidelines for when to pin exact versions vs use version ranges in dependencies.

## What's in the repo (evidence)

- Pixi dependencies: `pixi.toml` (100+ package definitions)
- Nix flake inputs: `flake.nix` (nixpkgs, home-manager, etc.)
- PyTorch coupling table: `pixi.toml` lines 138-145
- Conflict documentation: `docs/CONFLICTS.md`
- Example exact pins: `pydantic==2.7.0`, `numpy==1.24.3` (AIOS requirements)

## Goal

1. Balance stability (exact pins) with flexibility (version ranges)
2. Document rationale for all exact pins
3. Enable safe automatic updates where possible
4. Prevent accidental downgrades or incompatible upgrades

## Quick start

### Decision Matrix

Use this table to decide pinning strategy:

| Dependency Type | Strategy | Example | Rationale |
|----------------|----------|---------|-----------|
| Security-critical | Exact pin | `openssl==3.0.13` | CVE tracking |
| Tightly coupled (PyTorch) | Exact pin | `pytorch==2.5.1` | Runtime compatibility |
| Breaking API contracts | Exact pin | `pydantic==2.7.0` | AIOS requires <2.8 |
| Stable APIs | Minor range | `requests >=2.31,<3` | Patch updates OK |
| Dev tools | Minor range | `black >=24.0,<25` | Non-runtime |
| Nix stable channel | Branch pin | `nixpkgs-24.11` | LTS releases |

**Evidence**: `pixi.toml` shows mix of exact pins and ranges

### When to Use Exact Pins (==)

Pin exact versions when:

1. **Security-critical dependencies**
   ```toml
   # Example: Track specific security patches
   openssl = "==3.0.13"
   ```

2. **Known incompatibilities exist**
   ```toml
   # AIOS breaks with pydantic >=2.8
   pydantic = "==2.7.0"
   ```
   **Evidence**: `docs/CONFLICTS.md` line 120, `pixi.toml` AIOS section

3. **Tightly coupled packages (PyTorch)**
   ```toml
   pytorch = "==2.5.1"
   torchvision = "==0.20.1"
   torchaudio = "==2.5.1"
   ```
   **Evidence**: `pixi.toml` lines 138-145 document coupling requirement

4. **NumPy 2.x incompatibility**
   ```toml
   # ROS2/ML stack incompatible with NumPy 2.x
   numpy = "==1.24.3"
   ```
   **Evidence**: `docs/CONFLICTS.md` line 464

### When to Use Version Ranges (>=,<)

Use ranges for:

1. **Development tools (non-runtime)**
   ```toml
   black = ">=24.0,<25"
   pytest = ">=8.0,<9"
   ```
   **Rationale**: Breaking changes rare, patch updates beneficial

2. **Stable library APIs**
   ```toml
   requests = ">=2.31,<3"
   pyyaml = ">=6.0,<7"
   ```
   **Rationale**: Patch/minor versions maintain compatibility

3. **Most general dependencies**
   ```toml
   # Allow automatic security patches
   urllib3 = ">=2.0,<3"
   ```

### Nix Pinning Strategy

Nix uses a dual-track approach:

```nix
# flake.nix
inputs = {
  # Stable channel (production)
  nixpkgs-stable.url = "github:NixOS/nixpkgs/nixos-24.11";

  # Unstable channel (latest features)
  nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
}
```

**Strategy**:
- **Production deployments**: Use `nixpkgs-stable` (24.11 LTS)
- **Development/testing**: Use `nixpkgs` (unstable, daily updates)
- **Update frequency**: Stable = monthly, Unstable = weekly

**Evidence**: `docs/CONFLICTS.md` lines 451-455 document Nix channel strategy

### ROS2 Pinning Strategy

ROS2 is pinned to Humble via RoboStack:

```toml
# Locked to Humble (via robostack-humble channel)
ros-humble-rclpy = "*"
```

**Strategy**:
- **Distribution**: Pin to Humble (LTS until 2027)
- **Patch updates**: Automatic via conda-forge
- **Python compatibility**: Locked to 3.11.x (RoboStack constraint)

**Evidence**: `docs/CONFLICTS.md` lines 17-27, 45-50

## Testing Before Pinning

Before adding a new exact pin:

```bash
# 1. Test with the exact version
pixi add "package==X.Y.Z"
pixi install --frozen

# 2. Run full test suite
colcon test
colcon test-result --verbose

# 3. Run E2E validation
./scripts/validate-e2e.sh

# 4. If all passes, document the pin
$EDITOR pixi.toml
# Add comment explaining why exact pin is needed
```

**Evidence**: `scripts/validate-e2e.sh` exists for validation

## Documenting Pin Rationale

Always document why an exact pin exists:

```toml
# GOOD: Rationale included
# AIOS requires pydantic <2.8 due to breaking changes in field validation
pydantic = "==2.7.0"

# BAD: No explanation
pydantic = "==2.7.0"
```

**Where to document**:
1. Inline comment in `pixi.toml`
2. Entry in `docs/CONFLICTS.md` if it's a known conflict
3. Update `docs/COMPATIBILITY_MATRIX.md` (see related docs)

## Updating Pinned Versions

When a pinned dependency needs updating:

```bash
# 1. Check for breaking changes in changelog
# Visit package's GitHub releases or CHANGELOG

# 2. Update the pin
$EDITOR pixi.toml
# Change pydantic = "==2.7.0" to pydantic = "==2.8.0"

# 3. Test thoroughly
pixi update
./scripts/validate-e2e.sh

# 4. Update documentation
$EDITOR docs/CONFLICTS.md
# Document any changes in compatibility notes
```

## PyTorch Coupling Rules (CRITICAL)

PyTorch versions MUST be upgraded together:

**Rule**: Always match PyTorch major.minor versions across pytorch/torchvision/torchaudio

**Coupling Table** (from `pixi.toml:138-145`):

```toml
# Version Coupling Table:
#   PyTorch 2.5.x requires torchvision 0.20.x, torchaudio 2.5.x
#   PyTorch 2.4.x requires torchvision 0.19.x, torchaudio 2.4.x
#   PyTorch 2.3.x requires torchvision 0.18.x, torchaudio 2.3.x
```

**Upgrade process**:
```bash
# Use the automated script that handles coupling
./scripts/upgrade-python-deps.sh pytorch 2.5.1
```

**Evidence**: `pixi.toml` lines 135-145, `scripts/upgrade-python-deps.sh` exists

## Dependency Conflict Resolution

When version conflicts arise:

1. **Check compatibility matrix**: See `docs/CONFLICTS.md`
2. **Prioritize order**:
   - Security fixes (highest priority)
   - ROS2 compatibility
   - PyTorch coupling
   - Application functionality
3. **Document the conflict**: Add to `docs/CONFLICTS.md`
4. **Consider alternatives**: Can you use a different package?

## Automation Guidelines

For CI/CD, configure automatic updates based on pin type:

```yaml
# Example: Dependabot configuration (if using)
version: 2
updates:
  # Allow automatic minor/patch updates for ranges
  - package-ecosystem: "pip"
    directory: "/"
    schedule:
      interval: "weekly"
    allow:
      - dependency-type: "all"
        update-types: ["patch", "minor"]

  # Ignore exact pins (manual review required)
    ignore:
      - dependency-name: "pydantic"
      - dependency-name: "pytorch"
```

**Note**: Verify `.github/` for actual automation setup

## Related docs

- [Conflicts and Compatibility](../CONFLICTS.md) - Known version conflicts
- [Compatibility Matrix](../COMPATIBILITY_MATRIX.md) - Version compatibility tables
- [Dependency Updates](DEPENDENCY_UPDATES.md) - Update procedures
- [Migration Guides](../MIGRATION_GUIDES.md) - Major version upgrade paths
