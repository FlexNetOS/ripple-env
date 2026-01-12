# ARM64 Deployment Guide

This guide covers deploying the ROS2 Humble environment on ARM64 platforms.

## Supported Platforms

| Platform | Architecture | Status | Notes |
|----------|--------------|--------|-------|
| Raspberry Pi 4/5 | aarch64-linux | Supported | 4GB+ RAM recommended |
| NVIDIA Jetson Nano/Xavier/Orin | aarch64-linux | Supported | Requires Jetson overlay |
| Apple Silicon (M1/M2/M3) | aarch64-darwin | Supported | Native performance |
| AWS Graviton | aarch64-linux | Supported | EC2 instances |
| Ampere Altra | aarch64-linux | Supported | Cloud instances |

## Quick Start

### Raspberry Pi / Generic ARM64 Linux

```bash
# Install Nix
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# Clone repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# Enter development shell
nix develop
```

### NVIDIA Jetson

For Jetson devices, use the specialized CUDA-enabled shell:

```bash
# After installing Nix
nix develop .#cuda

# Verify CUDA availability
nvidia-smi
python -c "import torch; print(torch.cuda.is_available())"
```

**Note**: Jetson requires the NVIDIA-specific CUDA toolkit. The flake automatically
uses appropriate packages when detecting Jetson hardware.

### Apple Silicon (macOS)

```bash
# Install Nix (Determinate Systems installer recommended)
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# Clone and enter
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env
nix develop
```

## Architecture-Specific Considerations

### Memory Requirements

| Shell | Minimum RAM | Recommended |
|-------|-------------|-------------|
| default | 2GB | 4GB |
| full | 4GB | 8GB |
| cuda | 4GB | 8GB+ |

### Package Availability

Most packages in nixpkgs have ARM64 builds. Known exceptions:

- Some proprietary packages (check with `nix-env -qa --system aarch64-linux`)
- Certain CUDA packages require Jetson-specific builds
- Some pre-built binaries may fall back to source compilation

### Performance Tuning

For robotics workloads on ARM64:

```bash
# Enable performance governor
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Increase file descriptors (for ROS2 DDS)
ulimit -n 65535

# For real-time priority
sudo setcap cap_sys_nice+ep $(which ros2)
```

## Raspberry Pi Specific Setup

### Enable Hardware Interfaces

```bash
# Enable I2C
sudo raspi-config nonint do_i2c 0

# Enable SPI
sudo raspi-config nonint do_spi 0

# Enable Serial (for UART sensors)
sudo raspi-config nonint do_serial_hw 0

# Reboot to apply
sudo reboot
```

### GPIO Access

```bash
# Add user to gpio group
sudo usermod -aG gpio $USER

# Verify access
gpioinfo
```

### Camera (CSI)

```bash
# Enable camera
sudo raspi-config nonint do_camera 0

# Verify with ROS2
ros2 run image_tools cam2image
```

## Jetson Specific Setup

### JetPack SDK

Ensure JetPack SDK is installed for your Jetson model:

- Jetson Nano: JetPack 4.6+
- Jetson Xavier: JetPack 5.1+
- Jetson Orin: JetPack 6.0+

### Power Mode

```bash
# Set maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Verify
sudo nvpmodel -q
```

### CUDA Environment

The flake sets up CUDA automatically. Verify:

```bash
# Check CUDA version
nvcc --version

# Test with PyTorch
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')"
```

## Cross-Compilation

For building ARM64 packages from x86_64:

```bash
# Enable binfmt for cross-architecture support
nix run nixpkgs#qemu -- --version

# Build for ARM64 from x86_64
nix build .#packages.aarch64-linux.default
```

## Troubleshooting

### Slow Nix Evaluation

ARM64 systems may have slower Nix evaluation. Use binary cache:

```bash
# Add binary caches to /etc/nix/nix.conf
extra-substituters = https://cache.nixos.org https://nix-community.cachix.org
extra-trusted-public-keys = cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY= nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs=
```

### Missing ARM64 Package

If a package lacks ARM64 support:

```bash
# Check package availability
nix-env -qa --system aarch64-linux | grep <package>

# Build from source (slower but works)
nix build --system aarch64-linux .#<package>
```

### GPIO Permission Denied

```bash
# Create udev rules
sudo tee /etc/udev/rules.d/99-gpio.rules << EOF
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"
EOF

# Reload and restart
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## CI/CD for ARM64

The repository includes ARM64 CI testing:

```yaml
# .github/workflows/ci.yml
jobs:
  test-arm64:
    runs-on: ubuntu-latest
    steps:
      - uses: docker/setup-qemu-action@v3
        with:
          platforms: arm64
      - uses: docker/setup-buildx-action@v3
      # ... ARM64 tests
```

## Resources

- [NixOS on ARM](https://nixos.wiki/wiki/NixOS_on_ARM)
- [Raspberry Pi Nix](https://nixos.wiki/wiki/NixOS_on_ARM/Raspberry_Pi)
- [Jetson + Nix](https://github.com/anduril/jetpack-nixos)
- [ROS2 on Raspberry Pi](https://docs.ros.org/en/humble/How-To-Guides/Installing-on-Raspberry-Pi.html)
