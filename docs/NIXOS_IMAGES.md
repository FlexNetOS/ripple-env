# NixOS Image Generation

This document describes how to build, test, and release NixOS images for various deployment targets.

## Available Image Formats

| Format | Use Case | Build Command |
|--------|----------|---------------|
| WSL2 tarball | Windows development | `nix build .#nixosConfigurations.wsl-ros2.config.system.build.tarballBuilder` |
| ISO installer | Bare metal installation | `nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage` |
| QEMU VM | Local testing | `nix build .#nixosConfigurations.vm-ros2.config.system.build.vm` |

## Building Images

### WSL2 Tarball

Build a tarball that can be imported into Windows Subsystem for Linux:

```bash
nix build .#nixosConfigurations.wsl-ros2.config.system.build.tarballBuilder
```

Import into WSL2:

```powershell
wsl --import NixOS-ROS2 $env:USERPROFILE\WSL\NixOS-ROS2 result/nixos-wsl.tar.gz
wsl -d NixOS-ROS2
```

### ISO Installer

Build a bootable ISO for bare metal installation:

```bash
nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage
```

Write to USB:

```bash
dd if=result/iso/nixos-*.iso of=/dev/sdX bs=4M status=progress
```

### QEMU VM

Build and run a virtual machine:

```bash
nix build .#nixosConfigurations.vm-ros2.config.system.build.vm
./result/bin/run-nixos-ros2-vm
```

## Security Hardening

All images include a security hardening module with three levels:

| Level | Use Case | Features |
|-------|----------|----------|
| `minimal` | Development | Basic kernel hardening, relaxed firewall |
| `standard` | Production | Full kernel hardening, audit logging, SSH hardening |
| `strict` | High security | All above + file integrity monitoring, strict firewall |

### Configuration

```nix
{
  security.hardening = {
    enable = true;
    level = "standard";  # minimal, standard, or strict
    auditd.enable = true;
  };
}
```

### Hardening Features

- **Kernel Security**: Restricted dmesg, hidden kernel pointers, ptrace restrictions
- **Network Security**: SYN flood protection, ICMP hardening, source route blocking
- **SSH Hardening**: Key-only auth, strong ciphers, rate limiting
- **Systemd Services**:
  - Security audit logging
  - Failed login monitoring
  - File integrity checking (strict mode)
  - Automatic security update checks
- **Firewall**: Configurable with strict mode option

## Testing

### Running Tests Locally

Run all image tests:

```bash
nix flake check
```

Run individual tests:

```bash
# Basic services test
nix build .#checks.x86_64-linux.basic-services

# Docker functionality test
nix build .#checks.x86_64-linux.docker-services

# Development tools test
nix build .#checks.x86_64-linux.dev-tools

# Security hardening test
nix build .#checks.x86_64-linux.security-hardening

# Network connectivity test
nix build .#checks.x86_64-linux.network
```

### Available Tests

| Test | Description |
|------|-------------|
| `basic-services` | SSH, Nix, user management |
| `docker-services` | Docker daemon, container execution |
| `dev-tools` | Git, editors, Python, utilities |
| `security-hardening` | Kernel parameters, security services |
| `network` | Network connectivity, firewall |

## CI/CD Pipeline

The `nixos-images.yml` workflow provides:

### Build Jobs
- Builds all image types in parallel
- Measures build times and image sizes
- Uploads artifacts for 7 days

### Size Monitoring
- Tracks image sizes over time
- Warns when approaching size limits
- Fails if limits exceeded

| Image | Size Limit |
|-------|------------|
| WSL2 | 2048 MB |
| ISO | 3072 MB |
| VM | 4096 MB |

### Testing
- Runs VM tests automatically
- Validates security hardening
- Tests core functionality

### Release Automation
- Triggers on version tags (v*)
- Creates GitHub releases
- Generates checksums
- Includes release notes

## Image Size Optimization

To reduce image sizes:

1. **Remove unused packages**: Review `environment.systemPackages`
2. **Use overlays**: Create minimal package variants
3. **Enable store optimization**: `nix.settings.auto-optimise-store = true`
4. **Clean garbage**: `nix-collect-garbage -d`

## Troubleshooting

### WSL2 Import Fails

```powershell
# Check WSL version
wsl --version

# Ensure WSL2 is default
wsl --set-default-version 2

# Try import with verbose output
wsl --import NixOS-ROS2 $env:USERPROFILE\WSL\NixOS-ROS2 result/nixos-wsl.tar.gz --verbose
```

### VM Won't Start

```bash
# Check KVM is available
ls -la /dev/kvm

# Run with verbose output
nix build .#nixosConfigurations.vm-ros2.config.system.build.vm -L
```

### Test Failures

```bash
# Run test with verbose output
nix build .#checks.x86_64-linux.basic-services -L --print-build-logs
```

## File Structure

```
nix/
├── images/
│   ├── default.nix           # Image builders aggregator
│   ├── wsl.nix               # WSL2 tarball configuration
│   ├── iso.nix               # ISO installer configuration
│   ├── vm.nix                # QEMU VM configuration
│   └── security-hardening.nix # Security hardening module
└── tests/
    ├── default.nix           # Tests aggregator
    └── image-tests.nix       # VM test definitions
```

## Related Documentation

- [Getting Started Guide](GETTING_STARTED.md)
- [Troubleshooting](TROUBLESHOOTING.md)
- [Flake Modularization Plan](NIX_FLAKE_MODULARIZATION.md)
