# Known Conflicts and Compatibility Analysis

This document tracks compatibility issues, version constraints, and recommended solutions for the ROS2 Humble development environment.

## Python Version Compatibility

### Official ROS2 Support: Python 3.12.3

Per [ros2/ros2#1684](https://github.com/ros2/ros2/issues/1684) (May 6, 2025), ROS2 maintainer Michael Carroll confirmed:

> "The binaries for Debian/Ubuntu and RHEL are built against the Python version that is found in the underlying distribution (in this case 3.12.3 for Ubuntu Noble)"

| Component | Python Version | Reason |
|-----------|---------------|--------|
| ROS2 Official (Ubuntu Noble) | 3.12.3 | Official binary builds |
| ROS2 (build from source) | 3.12+ | Supported, file issues for incompatibilities |
| RoboStack Humble | 3.11.x | Conda-forge constraint |
| PyTorch 2.5+ | 3.9-3.12 | Official wheel support |

### Dual Python Architecture (Implemented)

This environment uses **two Python versions** for different purposes:

| Python | Source | Purpose | Access |
|--------|--------|---------|--------|
| **3.13.x** | Nix (`python313`) | Scripts, tools, non-ROS apps | `python3.13` |
| **3.11.x** | Pixi/RoboStack | ROS2, PyTorch, ML stack | `python` (via Pixi) |

**Why two Pythons?**
- RoboStack conda packages require Python 3.11.x
- Python 3.13 offers JIT compilation, better performance for scripts
- Isolation prevents version conflicts between ROS2 and other tools

**Usage:**
```bash
# ROS2/PyTorch (via Pixi - default 'python')
python -c "import rclpy; print('ROS2 ready')"
python -c "import torch; print(torch.__version__)"

# Nix Python 3.13 (for scripts/tools)
python3.13 -c "print('Python 3.13 with JIT')"
python3.13 -m venv my_venv  # Create isolated venv
```

### RoboStack Constraint: Python 3.11.x

RoboStack conda packages are built against 3.11. To use Python 3.12+ for ROS2:
1. Build ROS2 from source, OR
2. Wait for RoboStack to update their builds
3. (Investigating) Alternative robotics frameworks

### Python 3.14 Analysis (NOT YET READY)

**Status**: Not compatible with current ROS2 ecosystem

**Key Issues**:
1. **No RoboStack Packages**: The robostack-humble channel has no Python 3.14 builds
2. **PyTorch Incompatibility**: No official Python 3.14 wheels yet
3. **Build from source option**: Per ROS2 maintainers, building from source *should* work

**From ROS2 maintainers** ([ros2/ros2#1684](https://github.com/ros2/ros2/issues/1684)):
> "Building from source should allow for using newer versions of Python. If there are incompatibilities, please open issues/PRs in the corresponding repo."

**Recommendation**:
- **Current**: Stay on Python 3.11 for RoboStack compatibility
- **Upgrade path**: Consider building ROS2 from source with Python 3.12 for performance gains
- **Future**: Python 3.14 support expected with Ubuntu 26.04 LTS

## PyTorch Integration

### Recommended: Pixi/conda-forge (Implemented)

| Approach | Install Time | Performance | ROS2 Compat | Maintenance |
|----------|-------------|-------------|-------------|-------------|
| **Pixi/conda-forge** | 2-5 min | Native | Excellent | Low |
| Nix cudaPackages | 4-8 hours | Native | Complex | High |
| Docker | 5-10 min | <2% overhead | Isolated | Medium |

**Why Pixi wins**:
- Pre-built binaries from conda-forge
- Same Python environment as ROS2
- Native performance (no container overhead)
- Tested coexistence with RoboStack packages

### Configuration

**CPU (default)**:
```bash
# Installed automatically
pixi install
python -c "import torch; print(torch.__version__)"
```

**CUDA (GPU)**:
```bash
# Use CUDA environment
pixi run -e cuda python -c "import torch; print(torch.cuda.is_available())"

# Or use Nix CUDA shell
nix develop .#cuda
```

## CUDA Toolkit

### Current Status: CUDA 12.x Default, 13.x Available

**CUDA Version Selection in nixpkgs**:
- `cudaPackages` defaults to CUDA 12.x (PyTorch/TorchVision compatibility)
- `cudaPackages_13_1` available when core ML libraries support it
- Use CUDA redistributables (`cudaPackages.*`) instead of legacy `cudatoolkit`

Per [nixpkgs CUDA docs](https://github.com/NixOS/nixpkgs/blob/master/doc/languages-frameworks/cuda.section.md):
> "If `cudaPackages_13_1` is the latest release, but core libraries like PyTorch or TorchVision fail to build with it, `cudaPackages` may alias `cudaPackages_12` instead."

### Multi-Shell Approach (Implemented)

| Shell | Command | Use Case |
|-------|---------|----------|
| Default | `nix develop` | CPU-only, all platforms |
| CUDA | `nix develop .#cuda` | GPU workloads, Linux only |
| CI | `nix develop .#ci` | Minimal, fast CI builds |

### CUDA Shell Components

```nix
# Included in .#cuda devshell (using CUDA redistributables):
cudaPackages.cudatoolkit  # CUDA compiler, libraries
cudaPackages.cudnn        # Deep learning primitives
cudaPackages.cutensor     # Tensor operations
cudaPackages.nccl         # Multi-GPU communication
nvtopPackages.full        # GPU monitoring

# For CUDA 13.x specifically (when PyTorch supports it):
# cudaPackages_13_1.cudatoolkit
```

### Binary Cache (IMPORTANT)

The CUDA binary cache moved in November 2025. Add to `/etc/nix/nix.conf`:
```
extra-substituters = https://cache.nixos-cuda.org
extra-trusted-public-keys = cache.nixos-cuda.org-1:AdXGc/SyVzNB8Wf+DrK0EtTbQOKoKkXH1sMgqFpH8ig=
```

**Alternative (legacy, still works)**:
```
extra-substituters = https://cuda-maintainers.cachix.org
extra-trusted-public-keys = cuda-maintainers.cachix.org-1:0dq3bujKpuEPMCX6U4WylrUDZ9JyUG0VpVZa7CNfq5E=
```

### Flox + NVIDIA Partnership (Sept 2025)

Per [Skywork AI](https://skywork.ai/blog/flox-and-cuda-on-nix-a-comprehensive-guide/):
> "NVIDIA is now officially enabling Flox to redistribute pre-compiled CUDA packages. This transforms a process that took hours of compilation into a matter of seconds."

Consider Flox as an alternative for faster CUDA package installation.

### GPU Auto-Detection

The CUDA shell checks for `nvidia-smi` at activation:
- If present: Shows GPU info and CUDA version
- If absent: Warns about missing drivers

**Note**: Nix cannot auto-detect GPUs at build time. Use feature flags:
```bash
# Explicitly choose shell based on hardware
nix develop .#cuda    # For NVIDIA GPUs
nix develop           # For CPU-only
```

## Tool-Specific Conflicts

### From GitHub Resources Analysis

| Tool | Conflict | Resolution |
|------|----------|------------|
| **Unsloth** | PyTorch version pinning (3.11 only) | Use Docker: `docker run -it unsloth/unsloth` |
| **ComfyUI** | PyTorch/CUDA conflicts with ROS2 env | Use dedicated flake: `nix run github:utensils/comfyui-nix` |
| **Agentic Flow** | Node.js ecosystem, not Python | Not recommended for ROS2 integration |
| **AGiXT** | Requires Docker Compose | Docker only, no direct Nix integration |
| **SQLx** | Requires database at compile time | Use `naersk` with prepared DB image |
| **PyO3/maturin** | Complex Nix+Python builds | Use Pixi for builds, `naersk` for packaging |
| **Trivy** | Limited NixOS filesystem scanning | Use `vulnix` for Nix-specific security |

### Nix + Pixi Coexistence

**Potential Issues**:
1. **LD_LIBRARY_PATH conflicts**: Nix and conda can have incompatible library versions
2. **Python path confusion**: Multiple Python installations

**Solutions** (implemented in flake.nix):
- Pixi environment loaded via `shell-hook`
- `DYLD_FALLBACK_LIBRARY_PATH` set correctly on macOS
- Nix provides system tools, Pixi provides ROS2/ML stack

## Version Matrix

### Tested Compatible Versions

| Component | Version | Source | Notes |
|-----------|---------|--------|-------|
| Python (scripts) | 3.13.x | Nix | Latest stable, JIT enabled |
| Python (ROS2) | 3.11.x | Pixi | RoboStack constraint |
| PyTorch | 2.5.x | Pixi | CPU default, CUDA optional |
| CUDA Toolkit | 13.x | Nix | Falls back to 12.x if unavailable |
| cuDNN | 9.x | Nix | Matched to CUDA version |
| GCC | 13.x | Nix | Pinned for CUDA compatibility |
| ROS2 | Humble | Pixi | robostack-humble channel |
| Node.js | 22.x LTS | Nix | For LazyVim plugins |

### Upgrade Paths

**Option 1: Stay on RoboStack (current)**
```
ROS2 Humble + Python 3.11 + CUDA 12.x (RoboStack)
```

**Option 2: Build ROS2 from source**
```
ROS2 Humble + Python 3.12 + CUDA 12.x/13.x
↳ Better performance, official ROS2 support
↳ Requires building ROS2 packages locally
```

**Option 3: Upgrade to ROS2 Jazzy**
```
ROS2 Jazzy + Python 3.12 + CUDA 13.x
↳ Full Python 3.12 support
↳ Wait for RoboStack Jazzy packages
```

## Troubleshooting

### PyTorch CUDA not detected

```bash
# Verify CUDA installation
nvidia-smi
nvcc --version

# Check PyTorch CUDA
python -c "import torch; print(torch.cuda.is_available())"
python -c "import torch; print(torch.version.cuda)"

# Common fix: use CUDA environment
pixi run -e cuda python -c "import torch; print(torch.cuda.is_available())"
```

### Library conflicts

```bash
# Check for conflicting libraries
ldd $(python -c "import torch; print(torch.__file__)")

# Reset environment
pixi clean
pixi install
```

### Nix build failures

```bash
# Check CUDA cache
nix path-info --store https://cuda-maintainers.cachix.org \
  nixpkgs#cudaPackages.cudatoolkit

# Build with verbose output
nix develop .#cuda -L
```
