# WSL Configuration Templates

This directory contains `.wslconfig` templates optimized for different hardware profiles.

## Available Configurations

### high-end-workstation.wslconfig

**Target Hardware:**
- **CPU**: AMD Ryzen Threadripper PRO 7965WX (24 cores, 48 threads)
- **RAM**: 512GB DDR5 ECC (64GB x 8)
- **GPU**: Dual NVIDIA RTX 5090 (32GB VRAM each)
- **SSD**: 2x Samsung 9100 PRO 4TB PCIe Gen 5
- **HDD**: 24TB + 2x 20TB (NAS storage)

**Resource Allocation:**
- Memory: 384GB (75% of total)
- Processors: 40 threads (83% of available)
- Swap: 32GB

## Installation

1. **Copy configuration to Windows user profile:**

   From PowerShell:
   ```powershell
   Copy-Item "\\wsl.localhost\NixOS-Ripple\home\nixos\ripple-env\config\wslconfig\high-end-workstation.wslconfig" "$env:USERPROFILE\.wslconfig"
   ```

   Or from WSL:
   ```bash
   cp config/wslconfig/high-end-workstation.wslconfig /mnt/c/Users/$USER/.wslconfig
   ```

2. **Restart WSL:**
   ```powershell
   wsl --shutdown
   wsl
   ```

3. **Verify configuration:**
   ```bash
   # Check available memory
   free -h

   # Check CPU count
   nproc

   # Check GPU access
   nvidia-smi
   ```

## Performance Script

For maximum performance, also load the high-performance environment script:

```bash
source scripts/high-performance-env.sh
```

This configures:
- Build parallelization (32 cores, 16 concurrent jobs)
- GPU settings for dual RTX 5090
- Memory optimization for large workloads
- ccache with 50GB cache

## Custom Configurations

To create a custom configuration:

1. Copy the closest template
2. Adjust `memory`, `processors`, and `swap` values
3. Follow the installation steps above

**Guidelines:**
- Leave 20-25% memory for Windows host
- Leave 4-8 threads for Windows tasks
- Swap should be 8-32GB depending on workload

## Troubleshooting

**WSL crashes with high memory allocation:**
- Reduce `memory` value
- Enable `autoMemoryReclaim=gradual` instead of `dropcache`

**GPU not detected:**
- Ensure NVIDIA WSL drivers are installed on Windows
- Run `nvidia-smi` to verify GPU access

**Slow disk I/O:**
- Ensure `sparseVhd=true` is set
- Consider using ext4.vhdx on NVMe SSD

## Related Files

- `scripts/high-performance-env.sh` - Runtime environment for high-end workstations
- `scripts/stable-env.sh` - Conservative settings for resource-constrained systems
- `docs/cookbooks/PERFORMANCE_TUNING.md` - Full tuning guide
