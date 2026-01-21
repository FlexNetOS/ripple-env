# Performance Tuning

System optimization procedures for WSL, Nix, Docker, and resource-constrained environments.

## What's in the repo (evidence)

- WSL tuning parameters: `scripts/stable-env.sh`
- Resource validation: `scripts/validate-resources.sh`
- Resource profiles: lightweight (4GB) → full (32GB+)
- Observability setup: `docs/OBSERVABILITY-QUICK-START.md`
- Build optimization: `scripts/setup-home-manager.sh`

## Goal

1. Optimize for WSL2 stability (Nix build parallelization limits)
2. Configure appropriate resource profiles
3. Tune Docker memory/CPU limits
4. Optimize Pixi install performance
5. Enable performance monitoring

## Quick start

### WSL2 Tuning (Critical for Stability)

WSL2 requires specific tuning to prevent crashes during Nix builds:

```bash
# Apply stable environment settings
source scripts/stable-env.sh

# Settings applied:
# NIX_BUILD_CORES=2      # Limit parallel builds (prevents crashes)
# NIX_MAX_JOBS=2         # Limit concurrent jobs
# MALLOC_ARENA_MAX=2     # Reduce memory fragmentation
```

**Why these limits?**
- Nix parallelization can crash WSL2 with >2 cores
- Memory pressure causes hangs with default malloc settings
- Pixi install may timeout without limits

**Evidence**: `scripts/stable-env.sh` contains these tuning parameters

**Apply permanently** (add to `~/.bashrc` or `~/.zshrc`):
```bash
echo 'source ~/ripple-env/scripts/stable-env.sh' >> ~/.bashrc
```

### WSL Memory Configuration

Edit `~/.wslconfig` (Windows user directory):

```ini
[wsl2]
# Recommended for development (adjust based on your system)
memory=8GB          # WSL memory limit
processors=4        # CPU cores
swap=2GB            # Swap space
localhostForwarding=true
```

**Resource Profiles**:

| Profile | Memory | CPUs | Disk | Use Case |
|---------|--------|------|------|----------|
| Minimal | 4GB | 2 | 50GB | ROS2 only |
| Standard | 8GB | 4 | 100GB | ROS2 + PyTorch CPU |
| GPU | 16GB | 8 | 200GB | LocalAI + ML models |
| Full Stack | 32GB | 8+ | 500GB | All services |
| High-End Workstation | 384GB | 40 | 2TB+ (WSL VHDX; ≥8TB SSD physical) | Multi-GPU AI training, large-scale builds |

**High-End Workstation Profile** (AMD Threadripper PRO 7965WX, 512GB RAM, Dual RTX 5090):

```ini
[wsl2]
memory=384GB           # 75% of 512GB for WSL
processors=40          # 40 of 48 threads
swap=32GB              # Large swap for AI workloads
nestedVirtualization=true
kernelCommandLine=vm.overcommit_memory=1

[experimental]
autoMemoryReclaim=dropcache
pageReporting=true
```

To use this profile during bootstrap:
```powershell
.\bootstrap.ps1 -HardwareProfile "high-end-workstation"
```

Or apply the pre-configured template:
```bash
cp config/wslconfig/high-end-workstation.wslconfig /mnt/c/Users/$USER/.wslconfig
```

Load the high-performance environment inside WSL:
```bash
source scripts/high-performance-env.sh
```

**Evidence**: `scripts/validate-resources.sh` checks these thresholds

**Apply changes**:
```powershell
# From Windows PowerShell
wsl --shutdown
wsl
```

### Docker Memory Limits

Tune Docker compose service memory limits:

```yaml
# Example: docker-compose.ai.yml
services:
  localai:
    mem_limit: 8g        # Maximum memory
    mem_reservation: 4g  # Guaranteed memory
    cpus: 4              # CPU limit
```

**Recommended limits per service** (from resource validation analysis):

| Service | Memory | CPUs | Notes |
|---------|--------|------|-------|
| Prometheus | 2GB | 1 | Metrics storage |
| PostgreSQL | 4GB | 2 | Database |
| LocalAI | 8-16GB | 4-8 | Model inference |
| Temporal | 4GB | 2 | Workflow engine |
| NATS | 512MB | 1 | Lightweight messaging |

**Evidence**: Thresholds derived from `scripts/validate-resources.sh`

### Pixi Install Optimization

Speed up Pixi package installation:

```bash
# Use frozen lockfile (skip solver, much faster)
pixi install --frozen

# Increase timeout for slow networks
export PIXI_INSTALL_TIMEOUT=300  # 5 minutes

# Use local cache
pixi config set cache-dir /path/to/fast-disk/.pixi-cache
```

**Performance tips**:
- First install: 5-10 minutes (downloads packages)
- Subsequent installs with `--frozen`: 1-2 minutes (uses cache)
- WSL timeout: Extended to 300s in `stable-env.sh`

**Evidence**: Pixi timeouts documented in stability analysis

### Nix Build Optimization

Optimize Nix evaluation and builds:

```bash
# Use eval cache
nix build --eval-cache

# Parallel evaluation (when not in WSL)
# WARNING: Only use outside WSL, causes crashes in WSL
nix build --max-jobs 8 --cores 16

# Use binary cache (avoid rebuilding)
nix build --substituters "https://cache.nixos.org https://nix-community.cachix.org"

# Show build progress
nom build  # 'nix-output-monitor' for better build feedback
```

**WSL WARNING**: Keep NIX_BUILD_CORES=2 and NIX_MAX_JOBS=2 in WSL
**Evidence**: WSL parallelization limits in `scripts/stable-env.sh`

### ROS2 Build Optimization

Tune colcon build parallelization:

```bash
# Auto-detect CPU count (default)
colcon build --symlink-install

# Limit parallel workers (for resource-constrained systems)
colcon build --symlink-install --parallel-workers 2

# Increase if you have resources
colcon build --symlink-install --parallel-workers 8

# Use ccache for faster rebuilds
export CC="ccache gcc"
export CXX="ccache g++"
colcon build --symlink-install
```

**Evidence**: Standard colcon optimization practices

### GPU Optimization

For NVIDIA GPU workloads:

```bash
# Verify GPU access
nvidia-smi

# Set CUDA visible devices (limit GPU usage)
export CUDA_VISIBLE_DEVICES=0  # Use first GPU only

# Configure PyTorch for optimal performance
export OMP_NUM_THREADS=4  # OpenMP threads per process
export MKL_NUM_THREADS=4  # Intel MKL threads

# Check GPU memory allocation
pixi run -e cuda python -c "import torch; print(torch.cuda.memory_summary())"
```

**NVIDIA WSL Setup**:
```bash
# Ensure nvidia-container-toolkit installed
sudo systemctl restart docker

# Verify GPU in container
docker run --gpus all nvidia/cuda:12.0-base nvidia-smi
```

**Evidence**: GPU detection in `scripts/validate-resources.sh`

### Monitoring Performance

Enable performance monitoring:

```bash
# System monitoring
htop  # CPU/memory in real-time

# Docker stats
docker stats

# Nix store size
nix path-info -rS /run/current-system | sort -nk2

# Disk usage
ncdu docker/data  # Interactive disk usage analyzer
```

**Prometheus Metrics** (if deployed):
- Node exporter: System metrics
- cAdvisor: Container metrics
- Custom exporters: Application metrics

**Evidence**: `docs/OBSERVABILITY-QUICK-START.md` documents Prometheus setup

### Disk Space Optimization

Free up disk space:

```bash
# Clean Nix store (removes unused packages)
nix-collect-garbage -d

# Clean Pixi cache
pixi clean

# Clean Docker
docker system prune -a --volumes

# WSL disk cleanup (comprehensive script)
./scripts/wsl-cleanup.sh

# Check savings
df -h
```

**Evidence**: `scripts/wsl-cleanup.sh` exists for cleanup

### Network Optimization

Speed up downloads:

```bash
# Use fastest mirror (Nix)
nix build --option substituters "https://cache.nixos.org"

# Use parallel downloads (conda-forge)
pixi config set max-parallel-downloads 8

# Use local package mirror (advanced)
# Set up local conda mirror or Nix binary cache
```

**Evidence**: Standard practice for package managers

### Build Cache Warming

Pre-populate caches to speed up builds:

```bash
# Download all runtime dependencies
nix develop --command echo "Cache warmed"

# Pre-download Pixi packages
pixi install --frozen

# Build all ROS2 packages once
colcon build --symlink-install
```

**Use case**: CI/CD, shared development machines

## Performance Benchmarks

**UNKNOWN**: Official performance benchmarks not established

**Recommendation**: Establish baselines for:
- Bootstrap time: Fresh install → ready (target: <30 min)
- Build time: Full ROS2 build (target: <10 min)
- Pixi install: With cache (target: <2 min)
- E2E validation: Full stack test (target: <5 min)

See `docs/PRODUCTION_SCALE.md` for resource baselines

## Profile-Specific Tuning

### Lightweight (4GB RAM)

```bash
# Disable GPU services
export DOCKER_PROFILES="core"

# Limit Docker memory
export DOCKER_MEMORY_LIMIT=3g

# Use lightweight tools
pixi add --feature minimal
```

### GPU Workload (16GB+ RAM)

```bash
# Enable GPU services
export DOCKER_PROFILES="core,ai,gpu"

# Allocate more memory to AI services
# Edit docker-compose.ai.yml memory limits

# Use CUDA environment
pixi run -e cuda python
```

### Full Stack (32GB+ RAM)

```bash
# Enable all services
docker compose --profile full up -d

# Remove parallelization limits (not in WSL)
export NIX_BUILD_CORES=8
export NIX_MAX_JOBS=8

# Use aggressive caching
export CCACHE_MAXSIZE=10G
```

### High-End Workstation (512GB+ RAM, Dual GPU)

**Target Hardware:**
- CPU: AMD Ryzen Threadripper PRO 7965WX (24 cores, 48 threads)
- RAM: 512GB DDR5 ECC
- GPU: Dual NVIDIA RTX 5090 (32GB VRAM each, 64GB total)
- Storage: PCIe Gen 5 NVMe SSDs

```bash
# Load high-performance environment
source scripts/high-performance-env.sh

# Settings applied automatically:
# NIX_BUILD_CORES=32, NIX_MAX_JOBS=16, CUDA_VISIBLE_DEVICES=0,1

# Enable all services with maximum resources
docker compose --profile full --profile gpu up -d

# Build with maximum parallelization
colcon build --symlink-install --parallel-workers 24

# Multi-GPU training configuration
export CUDA_VISIBLE_DEVICES=0,1
export NCCL_P2P_DISABLE=0

# Aggressive caching for large builds
export CCACHE_MAXSIZE=50G
```

**Bootstrap for High-End Workstation:**
```powershell
# From Windows PowerShell (as Administrator)
.\bootstrap.ps1 -HardwareProfile "high-end-workstation"
```

**Or apply config manually:**
```bash
# Copy optimized .wslconfig
cp config/wslconfig/high-end-workstation.wslconfig /mnt/c/Users/$USER/.wslconfig

# Restart WSL from Windows PowerShell
wsl --shutdown
wsl

# Load environment inside WSL
source scripts/high-performance-env.sh
```

**Evidence**: `config/wslconfig/high-end-workstation.wslconfig` and `scripts/high-performance-env.sh`

## Troubleshooting Performance Issues

### High Memory Usage

```bash
# Identify memory hogs
docker stats --no-stream | sort -k4 -h

# Reduce Docker memory
docker compose down
# Edit docker-compose.yml memory limits
docker compose up -d
```

### Slow Builds

```bash
# Check disk I/O (slow disk = slow builds)
sudo iotop

# Use faster disk for Nix store (if available)
# Move /nix to SSD via symlink

# Enable ccache
export USE_CCACHE=1
```

### WSL Instability

```bash
# Apply stability settings
source scripts/stable-env.sh

# Check Windows Event Viewer for WSL crashes

# Reduce memory pressure
./scripts/wsl-cleanup.sh
```

**Evidence**: WSL stability issues documented in audit findings

## Related docs

- [Resource Requirements](../ai-ml/AI_RESOURCE_REQUIREMENTS.md) - Service-specific requirements
- [Production Scale](../PRODUCTION_SCALE.md) - Scaling guidelines
- [WSL Build Pipeline](../wsl/WSL2_BUILD_PIPELINE.md) - WSL-specific optimizations
- [Observability](../getting-started/quick-start/OBSERVABILITY-QUICK-START.md) - Performance monitoring
