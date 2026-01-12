# Pixi Channel Strategy

This document defines the channel priority strategy, compatibility matrix, and conflict detection approach for the ROS2 Humble development environment.

## Overview

Pixi uses conda channels to resolve and install packages. When multiple channels are configured, the order matters for package resolution. This project uses multiple channels to support different use cases (ROS2, ML/AI, CUDA), which requires careful management to avoid conflicts.

## Channel Priority Strategy

### Default Environment

```toml
channels = ["robostack-humble", "conda-forge"]
```

**Priority Order** (highest to lowest):
1. **robostack-humble** - ROS2 Humble packages built for conda
2. **conda-forge** - Community-maintained packages

**Rationale:**
- ROS2 packages from `robostack-humble` take precedence to ensure ROS2 compatibility
- `conda-forge` provides general-purpose packages not in RoboStack
- This order ensures ROS2 dependencies (Python 3.11, specific library versions) are respected

### CUDA Environment

```toml
# feature.cuda
channels = ["pytorch", "nvidia", "conda-forge"]
```

**Priority Order** (highest to lowest):
1. **pytorch** - Official PyTorch channel with CUDA builds
2. **nvidia** - NVIDIA CUDA runtime and cuDNN
3. **conda-forge** - Supporting packages

**Rationale:**
- PyTorch CUDA builds must come from the `pytorch` channel for proper CUDA linking
- NVIDIA channel provides runtime libraries not in conda-forge
- conda-forge fills remaining dependencies

### Combined Environments (e.g., aios-cuda)

When features are combined, channels are merged. The effective priority becomes:
1. Channels from the first listed feature
2. Channels from subsequent features
3. Base workspace channels

**Example:** `aios-cuda` environment with features `["aios", "aios-cuda", "cuda"]`

## Channel Compatibility Matrix

| Channel Combination | Compatible | Notes |
|---------------------|------------|-------|
| `robostack-humble` + `conda-forge` | Yes | Default, well-tested |
| `pytorch` + `nvidia` + `conda-forge` | Yes | CUDA environment, Linux only |
| `robostack-humble` + `pytorch` | Partial | May conflict on numpy, Python versions |
| `robostack-humble` + `nvidia` | No | Different Python/CUDA expectations |
| `pytorch` + `conda-forge` | Yes | CPU PyTorch works fine |

### Package Resolution Conflicts

| Package | robostack-humble | pytorch | conda-forge | Resolution |
|---------|------------------|---------|-------------|------------|
| `python` | 3.11.x | 3.9-3.12 | Latest | Pin to 3.11 in workspace |
| `numpy` | ~1.24 | ~1.24-2.0 | Latest | Pin version range |
| `pytorch` | Not available | 2.x | 2.x (CPU) | Use channel-specific pins |
| `cudatoolkit` | Not available | 12.x | 12.x | Use pytorch channel for CUDA |

## Environment Isolation

To prevent cross-environment conflicts, each feature environment uses a separate `solve-group`:

```toml
[environments]
default = { features = [], solve-group = "default" }
cuda = { features = ["cuda"], solve-group = "cuda" }
aios = { features = ["aios"], solve-group = "aios" }
```

**Benefits:**
- Each solve-group has an independent lock file section
- Prevents one environment's constraints from affecting another
- Allows incompatible package versions across environments

## CI Channel Locking

### Lock File Validation

The `pixi.lock` file contains resolved package versions. CI validates this by:

1. **Lock file freshness check**: Ensure `pixi.lock` matches `pixi.toml`
2. **Channel hash verification**: Verify packages come from expected channels
3. **Cross-platform consistency**: Check lock file has entries for all platforms

### CI Workflow Steps

```yaml
# In .github/workflows/pixi-channel-validation.yml
- name: Verify lock file freshness
  run: pixi install --locked

- name: Check channel sources
  run: |
    python scripts/validate-channels.py
```

## Conflict Detection

### Automated Checks

The `validate-channels.py` script performs:

1. **Channel availability**: Verify all channels are reachable
2. **Package source audit**: Identify which channel provides each package
3. **Version constraint analysis**: Detect conflicting version requirements
4. **Cross-environment leak**: Ensure feature packages don't leak to default

### Manual Verification

```bash
# List package sources
pixi list --json | jq '.[] | {name, channel}'

# Check for duplicate packages from different channels
pixi list --json | jq -r '.[] | "\(.name) \(.channel)"' | sort | uniq -d

# Verify specific package channel
pixi list pytorch --json | jq '.channel'
```

## Troubleshooting Channel Conflicts

### Symptom: Package version mismatch

```
error: package 'numpy' requires version X but Y was found
```

**Solution:**
1. Check which channel provides numpy: `pixi list numpy --json`
2. Add explicit pin in relevant section
3. Consider using `channel = "..."` override

### Symptom: Build string conflicts

```
error: conflicting build strings for pytorch
```

**Solution:**
1. Ensure CUDA packages use explicit `build = "*cuda*"` filter
2. Don't mix CPU and CUDA PyTorch in same environment

### Symptom: Solver fails to find solution

```
error: no solution found for environment 'X'
```

**Solution:**
1. Try removing one channel at a time to identify conflict source
2. Use `pixi install -v` for detailed solver output
3. Check if package exists in prioritized channel

## Best Practices

### 1. Explicit Channel Specification

For packages available in multiple channels, specify the source:

```toml
pytorch = { version = ">=2.5", channel = "pytorch" }
```

### 2. Version Pinning at Boundaries

Pin versions where channels intersect:

```toml
# Shared dependency between ROS2 and ML
numpy = ">=1.24,<2"  # Works for both robostack and pytorch
```

### 3. Separate Solve Groups

Use different solve groups for incompatible feature sets:

```toml
[environments]
ros2 = { features = ["ros2"], solve-group = "ros2" }
ml = { features = ["ml"], solve-group = "ml" }
```

### 4. Document Channel Changes

When adding/modifying channels, update:
- This document (CHANNEL-STRATEGY.md)
- The compatibility matrix
- CI validation scripts

## Channel Health Monitoring

### Periodic Checks (Weekly)

1. **Channel reachability**: `curl -s https://conda.anaconda.org/robostack-humble/`
2. **Package freshness**: Compare latest versions to pinned versions
3. **Security advisories**: Check for CVEs in channel packages

### Metrics to Track

- Solver time per environment
- Lock file size changes
- Failed installs due to channel issues

## References

- [Pixi Channel Documentation](https://pixi.sh/latest/reference/pixi_manifest/#channels)
- [Conda Channel Priority](https://docs.conda.io/projects/conda/en/latest/user-guide/concepts/channels.html)
- [RoboStack Channel](https://robostack.github.io/)
- [PyTorch Conda Channel](https://pytorch.org/get-started/locally/)
