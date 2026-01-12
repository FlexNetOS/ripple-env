# Troubleshooting Guide

This guide covers common issues and their solutions when working with the ROS2 Humble development environment.

## Table of Contents

- [Nix Issues](#nix-issues)
- [Pixi Issues](#pixi-issues)
- [ROS2 Issues](#ros2-issues)
- [Docker Issues](#docker-issues)
- [WSL2 Issues](#wsl2-issues)
- [Performance Issues](#performance-issues)
- [GPU/CUDA Issues](#gpucuda-issues)

---

## Nix Issues

### "error: experimental Nix feature 'flakes' is disabled"

**Problem**: Nix flakes are not enabled.

**Solution**:
```bash
# Add to ~/.config/nix/nix.conf (create if needed)
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf

# Restart nix-daemon
sudo systemctl restart nix-daemon
```

### "error: cannot find flake 'flake.nix'"

**Problem**: You're not in the repository root directory.

**Solution**:
```bash
cd /path/to/ripple-env
nix develop
```

### "error: hash mismatch in fixed-output derivation"

**Problem**: Cached derivation hash doesn't match current source.

**Solution**:
```bash
# Clear Nix cache and rebuild
nix store gc
nix develop --rebuild
```

### "nix develop" takes too long

**Problem**: First-time builds download many packages.

**Solution**:
- Use binary cache (already configured in flake.nix)
- Use `nix develop` (default shell) for faster startup
- Check your internet connection
- Be patient on first run (10-15 minutes is normal)

### "error: memory allocation of X bytes failed"

**Problem**: Not enough RAM for Nix evaluation.

**Solution**:
```bash
# Increase swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## Pixi Issues

### "pixi: command not found"

**Problem**: Pixi is not in PATH.

**Solution**:
```bash
# Re-enter nix develop
exit
nix develop

# Or source the profile
source ~/.pixi/bin/env
```

### "pixi install" fails with network error

**Problem**: Network issues or conda-forge is down.

**Solution**:
```bash
# Retry with timeout
pixi install --timeout 300

# Clear cache and retry
pixi clean
pixi install
```

### "Conflicting dependencies" error

**Problem**: Package version conflicts.

**Solution**:
```bash
# Check the conflict
pixi list

# Update lock file
pixi update

# If still failing, check pixi.toml for version constraints
```

### Pixi environment not activating

**Problem**: The Pixi environment isn't sourced.

**Solution**:
```bash
# Inside nix develop, manually activate
eval "$(pixi shell-hook)"

# Or run commands through pixi
pixi run python your_script.py
```

---

## ROS2 Issues

### "ros2: command not found"

**Problem**: ROS2 environment not sourced.

**Solution**:
```bash
# Inside nix develop
source /opt/ros/humble/setup.bash  # If using system ROS2

# Or with Pixi
pixi run ros2 --help
```

### "Package 'X' not found"

**Problem**: Missing ROS2 package.

**Solution**:
```bash
# Add via Pixi
pixi add ros-humble-<package-name>

# Example
pixi add ros-humble-navigation2
```

### "colcon build" fails with CMake errors

**Problem**: Missing build dependencies.

**Solution**:
```bash
# Install rosdep dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Rebuild
colcon build --symlink-install
```

### "ROS_DISTRO is not set"

**Problem**: ROS2 environment variables not configured.

**Solution**:
```bash
# Set manually
export ROS_DISTRO=humble

# Or re-enter the shell
exit
nix develop
```

### Node not receiving messages

**Problem**: ROS2 DDS discovery issues.

**Solution**:
```bash
# Check if nodes are visible
ros2 node list

# Set explicit domain
export ROS_DOMAIN_ID=42

# Use localhost only (for single-machine development)
export ROS_LOCALHOST_ONLY=1
```

---

## Docker Issues

### "Cannot connect to Docker daemon"

**Problem**: Docker daemon not running.

**Solution**:
```bash
# Start Docker
sudo systemctl start docker

# Add user to docker group (requires logout/login)
sudo usermod -aG docker $USER
```

### "docker-compose: command not found"

**Problem**: Docker Compose not installed or wrong version.

**Solution**:
```bash
# Use docker compose (v2 syntax)
docker compose -f docker/docker-compose.yml up -d

# Or install standalone
sudo apt install docker-compose-plugin
```

### Container health check failing

**Problem**: Service isn't ready yet.

**Solution**:
```bash
# Check logs
docker logs <container-name>

# Wait for health check (use the health-check.sh script)
./scripts/health-check.sh http://localhost:8080/health 5 10 "ServiceName"
```

### "network agentic-network not found"

**Problem**: Docker network doesn't exist.

**Solution**:
```bash
# Create the network
docker network create agentic-network

# Then start services
docker compose -f docker/docker-compose.observability.yml up -d
```

### Port already in use

**Problem**: Another service is using the port.

**Solution**:
```bash
# Find what's using the port
sudo lsof -i :8080

# Kill the process or change port in docker-compose
docker compose -f docker/docker-compose.yml down
# Edit the port mapping, then:
docker compose -f docker/docker-compose.yml up -d
```

---

## WSL2 Issues

### Bootstrap script fails on Windows

**Problem**: PowerShell execution policy or missing admin rights.

**Solution**:
```powershell
# Run as Administrator
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
.\bootstrap.ps1
```

### WSL2 Disk Space Management

The WSL2 setup creates a sparse VHDX file that can grow up to 1TB by default. Understanding disk usage patterns helps prevent storage issues.

#### Understanding Disk Usage

**What uses space in WSL2:**

| Component | Typical Size | Location |
|-----------|-------------|----------|
| NixOS base | 5-10 GB | `/nix/store` |
| Nix store (after builds) | 20-50 GB | `/nix/store` |
| Pixi environments | 5-15 GB | `~/.pixi` |
| Docker images/containers | 10-50+ GB | `/var/lib/docker` |
| AI models (if downloaded) | 15-60 GB | `~/models` |
| Build artifacts | 1-10 GB | `~/ros2-humble-env/build` |

**Check current usage inside WSL:**
```bash
# Overall disk usage
df -h /

# Nix store size
du -sh /nix/store

# Pixi cache size
du -sh ~/.pixi

# Docker usage
docker system df

# Find large files
du -h / 2>/dev/null | grep '[0-9]G' | sort -hr | head -20
```

**Check VHD size from Windows (PowerShell):**
```powershell
# Get actual VHD size on disk
$vhdPath = "$env:USERPROFILE\WSL\NixOS-ROS2\ext4.vhdx"
(Get-Item $vhdPath).Length / 1GB
```

#### Automatic Garbage Collection

The WSL2 NixOS image includes automatic garbage collection:

- **Schedule**: Weekly
- **Policy**: Removes generations older than 7 days
- **Store optimization**: Enabled (deduplicates files)

To verify GC is running:
```bash
# Check systemd timer
systemctl status nix-gc.timer

# View GC logs
journalctl -u nix-gc.service
```

#### Manual Cleanup Commands

**Nix store cleanup:**
```bash
# Run garbage collection (removes unused packages)
nix store gc

# Remove all old generations and collect garbage
nix-collect-garbage -d

# Optimize store (deduplicate identical files)
nix store optimise

# Show what would be deleted
nix store gc --dry-run
```

**Pixi cleanup:**
```bash
# Clean Pixi cache
pixi clean

# Remove unused conda packages
pixi global clean
```

**Docker cleanup:**
```bash
# Remove unused containers, networks, images
docker system prune -a

# Remove all unused volumes (careful - data loss!)
docker system prune -a --volumes

# Remove specific unused images
docker image prune -a
```

**Build artifact cleanup:**
```bash
# Clean colcon build artifacts
cd ~/ros2-humble-env
rm -rf build/ install/ log/

# Clean ccache
ccache --clear
```

**AI model cleanup:**
```bash
# If using download-models.sh
./scripts/download-models.sh --clean
```

#### "Not enough disk space" Error

**Problem**: WSL2 VHD is full or near capacity.

**Solution 1: Clean up space inside WSL**
```bash
# Quick cleanup sequence
nix-collect-garbage -d
nix store optimise
docker system prune -a -f
pixi clean
```

**Solution 2: Expand the VHD (from Windows PowerShell as Administrator)**
```powershell
# Shutdown WSL first
wsl --shutdown
Resize-VHD -Path "$env:USERPROFILE\WSL\NixOS-Ripple\ext4.vhdx" -SizeBytes 200GB

# Inside WSL, resize filesystem
wsl -d NixOS-Ripple
sudo resize2fs /dev/sdc
```

**Solution 3: Reclaim unused space (Windows PowerShell as Administrator)**
```powershell
# Shutdown WSL
wsl --shutdown

# Compact the VHD (reclaims freed space)
# Note: sparseVhd=true in .wslconfig handles this automatically
Optimize-VHD -Path "$env:USERPROFILE\WSL\NixOS-ROS2\ext4.vhdx" -Mode Full
```

#### Configuring Disk Size at Install Time

When running `bootstrap.ps1`, you can specify a smaller disk:
```powershell
# Install with 256GB max disk instead of 1TB
.\bootstrap.ps1 -DiskSizeGB 256
```

#### WSL Configuration for Efficient Storage

The default `.wslconfig` includes storage optimizations:
```ini
[wsl2]
memory=8GB
swap=8GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true  # Automatically reclaims unused space
```

To edit:
```powershell
notepad "$env:USERPROFILE\.wslconfig"
# Then restart WSL: wsl --shutdown
```

#### Monitoring Disk Usage

**Create a disk usage report:**
```bash
#!/bin/bash
echo "=== WSL2 Disk Usage Report ==="
echo ""
echo "Filesystem:"
df -h / | tail -1
echo ""
echo "Nix Store: $(du -sh /nix/store 2>/dev/null | cut -f1)"
echo "Pixi:      $(du -sh ~/.pixi 2>/dev/null | cut -f1)"
echo "Docker:    $(docker system df --format 'table {{.Size}}' 2>/dev/null | tail -1)"
echo "Home:      $(du -sh ~ 2>/dev/null | cut -f1)"
```

Save this as `~/disk-usage.sh` and run periodically.

### WSL2 networking issues

**Problem**: Can't access localhost services from Windows.

**Solution**:
```powershell
# Check .wslconfig
notepad "$env:USERPROFILE\.wslconfig"

# Ensure localhostForwarding is enabled:
# [wsl2]
# localhostForwarding=true

wsl --shutdown
wsl
```

### "Nix daemon not running" in WSL2

**Problem**: Nix daemon didn't start automatically.

**Solution**:
```bash
# Start manually
sudo /nix/var/nix/profiles/default/bin/nix-daemon &

# Or restart WSL
wsl --shutdown
wsl -d NixOS-Ripple
```

---

## Performance Issues

### Slow file operations

**Problem**: Nix store or git operations are slow.

**Solution**:
```bash
# Use overlayfs for store
# Add to /etc/nix/nix.conf:
# use-sqlite-wal = true

# Optimize store
nix store optimise
```

### High memory usage

**Problem**: Nix evaluation or builds use too much RAM.

**Solution**:
```bash
# Limit build jobs
export NIX_BUILD_CORES=2

# Use --max-jobs flag
nix develop --max-jobs 2
```

### Slow colcon build

**Problem**: ROS2 builds are slow.

**Solution**:
```bash
# Use parallel builds
colcon build --parallel-workers 4 --symlink-install

# Only build changed packages
colcon build --packages-select my_package

# Use ccache
export CCACHE_DIR=~/.ccache
colcon build --cmake-args -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
```

---

## GPU/CUDA Issues

### "CUDA not available"

**Problem**: CUDA drivers not installed or not detected.

**Solution**:
```bash
# Check NVIDIA driver
nvidia-smi

# Use CUDA shell
nix develop .#cuda

# Verify CUDA in Python
python -c "import torch; print(torch.cuda.is_available())"
```

### "libcuda.so not found"

**Problem**: CUDA libraries not in path.

**Solution**:
```bash
# Set library path
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# Or use nixGL (for non-NixOS)
nix run github:guibou/nixGL -- python your_cuda_script.py
```

### CUDA version mismatch

**Problem**: PyTorch expects different CUDA version.

**Solution**:
```bash
# Check versions
nvidia-smi  # Shows driver CUDA version
nvcc --version  # Shows toolkit version

# Install matching PyTorch
pixi add pytorch-cuda=12.1
```

---

## Still Stuck?

If your issue isn't covered here:

1. **Search existing issues**: [GitHub Issues](https://github.com/FlexNetOS/ripple-env/issues)
2. **Check ROS2 docs**: [ROS2 Troubleshooting](https://docs.ros.org/en/humble/How-To-Guides/Installation-Troubleshooting.html)
3. **Check Nix docs**: [Nix Manual](https://nixos.org/manual/nix/stable/)
4. **Open a new issue**: Include:
   - Operating system and version
   - Output of `nix --version`
   - Complete error message
   - Steps to reproduce

---

## Diagnostic Commands

Use these commands to gather information for bug reports:

```bash
# System info
uname -a
cat /etc/os-release

# Nix info
nix --version
nix show-config

# Environment
echo $PATH
echo $ROS_DISTRO

# Docker
docker version
docker compose version

# GPU (if applicable)
nvidia-smi
```
