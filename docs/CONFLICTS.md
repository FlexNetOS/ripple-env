# Known Conflicts and Compatibility Analysis

This document tracks compatibility issues, version constraints, and recommended solutions for the ROS2 Humble development environment.

## Python Version Compatibility

### Current Constraint: Python 3.11.x

| Component | Python Version | Reason |
|-----------|---------------|--------|
| ROS2 Humble (robostack) | 3.11.x | ABI-locked via rclpy C extensions |
| PyTorch 2.5+ | 3.9-3.12 | Official wheel support |
| conda-forge | 3.11.x | Best tested with RoboStack |

### Python 3.14.2 Analysis (NOT RECOMMENDED)

**Status**: Not compatible with ROS2 Humble

**Key Issues**:
1. **ABI Incompatibility**: ROS2's `rclpy` is a C extension compiled against Python 3.10/3.11 ABI. Python 3.14 introduces ABI changes that break binary compatibility.

2. **No RoboStack Packages**: The robostack-humble channel has no Python 3.14 builds. Building from source would require:
   - Rebuilding all 200+ ROS2 packages
   - Potential API changes in Python 3.14 affecting C bindings
   - 6-12 month timeline before community support

3. **PyTorch Incompatibility**: As of January 2025, PyTorch has no official Python 3.14 wheels. Building from source adds 4-8 hours to environment setup.

4. **Performance Claims**: While Python 3.14 includes JIT compilation improvements, the performance gains (10-30% for pure Python) don't apply to ROS2's C++ core or PyTorch's CUDA kernels.

**Recommendation**: Stay on Python 3.11 for ROS2 Humble. Consider ROS2 Jazzy (Python 3.12) for newer Python features.

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

### Current Status: CUDA 12.x (Not 13.1)

**CUDA 13.1 Availability**:
- Not yet in nixpkgs (as of January 2025)
- Expected in nixpkgs-unstable Q2 2025
- conda-forge has CUDA 12.4 as latest stable

### Multi-Shell Approach (Implemented)

| Shell | Command | Use Case |
|-------|---------|----------|
| Default | `nix develop` | CPU-only, all platforms |
| CUDA | `nix develop .#cuda` | GPU workloads, Linux only |
| CI | `nix develop .#ci` | Minimal, fast CI builds |

### CUDA Shell Components

```nix
# Included in .#cuda devshell:
cudaPackages.cudatoolkit  # CUDA compiler, libraries
cudaPackages.cudnn        # Deep learning primitives
cudaPackages.cutensor     # Tensor operations
cudaPackages.nccl         # Multi-GPU communication
nvtopPackages.full        # GPU monitoring
```

### Binary Cache

Add to `/etc/nix/nix.conf` for faster CUDA builds:
```
extra-substituters = https://cuda-maintainers.cachix.org
extra-trusted-public-keys = cuda-maintainers.cachix.org-1:0dq3bujKpuEPMCX6U4WylrUDZ9JyUG0VpVZa7CNfq5E=
```

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

| Component | Version | Notes |
|-----------|---------|-------|
| Python | 3.11.x | Required by RoboStack |
| PyTorch | 2.5.x | CPU default, CUDA optional |
| CUDA Toolkit | 12.4 | Via Pixi or Nix |
| cuDNN | 8.9.x | Matched to CUDA 12.x |
| ROS2 | Humble | robostack-humble channel |
| Node.js | 22.x LTS | For LazyVim plugins |

### Upgrade Path

```
Current: ROS2 Humble + Python 3.11 + CUDA 12.4
    ↓
Future:  ROS2 Jazzy + Python 3.12 + CUDA 13.x (when available)
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
