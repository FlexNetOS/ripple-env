#!/usr/bin/env bash
# High-Performance Environment Setup for Workstation
# Optimized for: AMD Threadripper PRO 7965WX (24C/48T), 512GB RAM, Dual RTX 5090
#
# This script configures maximum performance settings for development and AI/ML workloads.
# Use this script on high-end workstations with adequate cooling and power.
#
# Usage:
#   source scripts/high-performance-env.sh
#
# Note: For WSL2 systems with constrained resources, use scripts/stable-env.sh instead.

set -euo pipefail

echo "🚀 Loading HIGH-PERFORMANCE environment for workstation..."
echo "   Hardware: AMD Threadripper PRO 7965WX, 512GB RAM, Dual RTX 5090"
echo ""

# =============================================================================
# CPU/Build Parallelization (Maximize for 24-core/48-thread CPU)
# =============================================================================

# Nix build parallelization: Use 32 cores for builds, 16 concurrent jobs
# Leave headroom for system processes and GPU compute
export NIX_BUILD_CORES=32
export NIX_MAX_JOBS=16

# CMake parallel build level
export CMAKE_BUILD_PARALLEL_LEVEL=32

# Make parallel jobs
export MAKEFLAGS="-j32"

# Colcon parallel workers for ROS2 builds
export COLCON_PARALLEL_WORKERS=24

# ccache settings for faster rebuilds
export CCACHE_MAXSIZE="50G"
export CCACHE_DIR="${HOME}/.ccache"
export USE_CCACHE=1

# =============================================================================
# Memory Configuration (Optimize for 512GB RAM)
# =============================================================================

# Allow more memory arenas for parallel workloads
export MALLOC_ARENA_MAX=32

# Larger memory thresholds for high-memory systems
export MALLOC_MMAP_THRESHOLD_=1048576    # 1MB threshold
export MALLOC_TRIM_THRESHOLD_=2097152    # 2MB trim threshold
export MALLOC_TOP_PAD_=262144            # 256KB top pad

# Python memory optimization
export PYTHONMALLOC=pymalloc
export PYTHONHASHSEED=random

# =============================================================================
# GPU Configuration (Dual RTX 5090, 64GB total VRAM)
# =============================================================================

# CUDA configuration
export CUDA_VISIBLE_DEVICES=0,1          # Enable both GPUs
export NVIDIA_VISIBLE_DEVICES=all        # For Docker/containers
export NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# CUDA memory management
export PYTORCH_CUDA_ALLOC_CONF="expandable_segments:True"
export TF_GPU_ALLOCATOR=cuda_malloc_async

# OpenMP thread settings for GPU compute
export OMP_NUM_THREADS=16
export MKL_NUM_THREADS=16
export OPENBLAS_NUM_THREADS=16

# NCCL settings for multi-GPU training
export NCCL_DEBUG=WARN
export NCCL_P2P_DISABLE=0
export NCCL_IB_DISABLE=1                 # Disable InfiniBand (not present)
export NCCL_SOCKET_IFNAME=eth0

# WSL2 GPU library path
if [[ -d "/usr/lib/wsl/lib" ]]; then
    export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# =============================================================================
# Disk I/O Optimization (PCIe Gen 5 NVMe SSDs)
# =============================================================================

# Increase file descriptor limits for I/O heavy workloads
ulimit -n 65536 2>/dev/null || true

# Async I/O settings
export URING_SETUP_CQSIZE=32768

# Git performance optimizations
export GIT_PARALLEL_THREADS=32

# =============================================================================
# Docker/Container Configuration
# =============================================================================

# Docker buildx parallelization
export DOCKER_BUILDKIT=1
export BUILDKIT_STEP_LOG_MAX_SIZE=104857600

# Compose parallelization
export COMPOSE_PARALLEL_LIMIT=16

# =============================================================================
# Network Configuration
# =============================================================================

# Increase network buffer sizes
export TCP_NODELAY=1

# Parallel downloads
export PIXI_MAX_PARALLEL_DOWNLOADS=16
export NIX_DOWNLOAD_THREADS=8

# =============================================================================
# PATH Configuration
# =============================================================================

# Ensure WSL paths are correct
export PATH="/usr/lib/wsl/lib:/usr/local/bin:/usr/bin:/bin:/home/${USER:-nixos}/.nix-profile/bin:$PATH"

# =============================================================================
# Load Additional Environment Variables
# =============================================================================

if [[ -f "scripts/env-vars.sh" ]]; then
    source scripts/env-vars.sh
fi

if [[ -f ~/.nix-profile/etc/profile.d/nix.sh ]]; then
    source ~/.nix-profile/etc/profile.d/nix.sh
fi

# =============================================================================
# Convenience Aliases
# =============================================================================

# Fast builds with full parallelization
alias nix-fast='nix build --max-jobs 16 --cores 32'
alias colcon-fast='colcon build --symlink-install --parallel-workers 24'

# Docker with GPU support
alias docker-gpu='docker run --gpus all'
alias compose-gpu='docker compose --profile gpu'

# Load standard env tools
alias load-env='source scripts/env-vars.sh'
alias ripple-save='scripts/session-save.sh'
alias ripple-restore='scripts/session-restore.sh'

# =============================================================================
# Verification
# =============================================================================

echo "✅ HIGH-PERFORMANCE environment loaded!"
echo ""
echo "   CPU Settings:"
echo "     • NIX_BUILD_CORES: $NIX_BUILD_CORES"
echo "     • NIX_MAX_JOBS: $NIX_MAX_JOBS"
echo "     • CMAKE_BUILD_PARALLEL_LEVEL: $CMAKE_BUILD_PARALLEL_LEVEL"
echo "     • COLCON_PARALLEL_WORKERS: $COLCON_PARALLEL_WORKERS"
echo ""
echo "   Memory Settings:"
echo "     • MALLOC_ARENA_MAX: $MALLOC_ARENA_MAX"
echo "     • CCACHE_MAXSIZE: $CCACHE_MAXSIZE"
echo ""
echo "   GPU Settings:"
echo "     • CUDA_VISIBLE_DEVICES: $CUDA_VISIBLE_DEVICES"
echo "     • OMP_NUM_THREADS: $OMP_NUM_THREADS"
echo ""
echo "💡 Commands:"
echo "   • nix-fast <target>     - Build with max parallelization"
echo "   • colcon-fast           - ROS2 build with max workers"
echo "   • docker-gpu            - Docker with GPU access"
echo ""
echo "⚠️  For resource-constrained WSL2, use: source scripts/stable-env.sh"
