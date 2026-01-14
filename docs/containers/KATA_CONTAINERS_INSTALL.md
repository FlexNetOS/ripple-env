# Kata Containers Installation Guide

**Status**: P0 (Critical)
**Date**: 2026-01-09
**Priority**: P0 (Blocking)
**Repository**: https://github.com/kata-containers/kata-containers

## Overview

Kata Containers provides lightweight virtual machines (VMs) that feel and perform like containers, but provide stronger workload isolation using hardware virtualization. This is the **default isolation** mechanism per BUILDKIT_STARTER_SPEC.md Layer 3.

## Prerequisites

### Hardware Requirements
- CPU with virtualization extensions (Intel VT-x or AMD-V)
- Nested virtualization enabled (if running in a VM)
- Minimum 4GB RAM recommended

### Software Requirements
- Linux kernel 4.15+
- Docker or containerd installed
- QEMU or Cloud Hypervisor

## Installation

### Quick Status Check

```bash
# Check if Kata is available via our wrapper
kata status

# Or directly check for kata-runtime
which kata-runtime
kata-runtime --version
```

### Ubuntu/Debian Installation

```bash
# Add Kata Containers repository
ARCH=$(dpkg --print-architecture)
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.opensuse.org/repositories/home:/katacontainers:/releases:/x86_64:/master/xUbuntu_$(lsb_release -rs)/Release.key | \
  gpg --dearmor | sudo tee /etc/apt/keyrings/kata-containers.gpg > /dev/null

echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/kata-containers.gpg] \
  https://download.opensuse.org/repositories/home:/katacontainers:/releases:/x86_64:/master/xUbuntu_$(lsb_release -rs)/ /" | \
  sudo tee /etc/apt/sources.list.d/kata-containers.list

# Install Kata Containers
sudo apt-get update
sudo apt-get install -y kata-containers
```

### Fedora Installation

```bash
# Install from Fedora repositories
sudo dnf install -y kata-containers
```

### Static Binary Installation (Any Linux)

```bash
# Get latest version from GitHub releases
# Check: https://github.com/kata-containers/kata-containers/releases
KATA_VERSION=$(curl -s https://api.github.com/repos/kata-containers/kata-containers/releases/latest | grep tag_name | cut -d '"' -f 4)
echo "Installing Kata Containers $KATA_VERSION"

# Download and extract static binaries (amd64 architecture)
curl -fsSL "https://github.com/kata-containers/kata-containers/releases/download/${KATA_VERSION}/kata-static-${KATA_VERSION}-amd64.tar.xz" | \
  sudo tar -C / -xJf -

# For ARM64 systems, use:
# curl -fsSL "https://github.com/kata-containers/kata-containers/releases/download/${KATA_VERSION}/kata-static-${KATA_VERSION}-arm64.tar.xz" | \
#   sudo tar -C / -xJf -

# Verify installation
/opt/kata/bin/kata-runtime --version
```

### NixOS Installation

```nix
# In your NixOS configuration.nix
{
  virtualisation.containerd = {
    enable = true;
    settings = {
      plugins."io.containerd.grpc.v1.cri".containerd.runtimes.kata = {
        runtime_type = "io.containerd.kata.v2";
      };
    };
  };

  environment.systemPackages = with pkgs; [
    kata-containers
  ];
}
```

## Docker Configuration

### Configure Docker to Use Kata

Create or update `/etc/docker/daemon.json`:

```json
{
  "default-runtime": "runc",
  "runtimes": {
    "kata": {
      "path": "/opt/kata/bin/kata-runtime",
      "runtimeArgs": []
    },
    "kata-clh": {
      "path": "/opt/kata/bin/kata-runtime",
      "runtimeArgs": ["--config", "/opt/kata/share/defaults/kata-containers/configuration-clh.toml"]
    },
    "kata-qemu": {
      "path": "/opt/kata/bin/kata-runtime",
      "runtimeArgs": ["--config", "/opt/kata/share/defaults/kata-containers/configuration-qemu.toml"]
    }
  }
}
```

Restart Docker:

```bash
sudo systemctl restart docker
```

### Verify Docker Configuration

```bash
# Check Docker runtimes
docker info | grep -i runtime

# Test Kata with Docker
docker run --rm --runtime=kata alpine uname -a
```

## containerd Configuration

If using containerd directly:

```toml
# /etc/containerd/config.toml
[plugins."io.containerd.grpc.v1.cri".containerd.runtimes.kata]
  runtime_type = "io.containerd.kata.v2"
  privileged_without_host_devices = true
  pod_annotations = ["io.kubernetes.cri.untrusted-workload"]

[plugins."io.containerd.grpc.v1.cri".containerd.runtimes.kata.options]
  ConfigPath = "/opt/kata/share/defaults/kata-containers/configuration.toml"
```

## Kubernetes Configuration

### RuntimeClass for Kata

```yaml
# kata-runtimeclass.yaml
apiVersion: node.k8s.io/v1
kind: RuntimeClass
metadata:
  name: kata
handler: kata
overhead:
  podFixed:
    memory: "160Mi"
    cpu: "250m"
scheduling:
  nodeSelector:
    kata-containers: "true"
```

Apply the RuntimeClass:

```bash
kubectl apply -f kata-runtimeclass.yaml
```

### Use Kata in Pods

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: kata-pod
spec:
  runtimeClassName: kata
  containers:
  - name: nginx
    image: nginx:alpine
```

## Docker Compose Integration

### Using Kata Runtime in docker-compose

```yaml
# docker-compose.kata.yml
version: '3.8'

services:
  secure-agent:
    image: agent-image:latest
    runtime: kata
    container_name: secure-agent
    environment:
      - AGENT_MODE=isolated
    volumes:
      - ./data:/data:ro
    networks:
      - agentic-network

networks:
  agentic-network:
    external: true
```

### Running with Kata Runtime

```bash
# Run specific service with Kata
docker compose -f docker-compose.kata.yml up secure-agent

# Override runtime for any service
docker compose run --runtime=kata agixt
```

## Verification

### Check Kata Installation

```bash
# Use our wrapper
kata check

# Or directly
kata-runtime check

# Expected output should show:
# - CPU capabilities
# - Kernel configuration
# - QEMU/Cloud Hypervisor availability
```

### Performance Test

```bash
# Quick performance comparison
echo "=== runc (standard container) ==="
time docker run --rm alpine echo "Hello from runc"

echo "=== kata (VM-isolated container) ==="
time docker run --rm --runtime=kata alpine echo "Hello from Kata"
```

### Security Verification

```bash
# Verify VM isolation
docker run --rm --runtime=kata alpine cat /proc/1/status | grep -E "^(Name|Pid|Uid):"

# Check hypervisor process
ps aux | grep -E "(qemu|cloud-hypervisor)" | grep -v grep
```

## Configuration

### Kata Configuration File

Default location: `/opt/kata/share/defaults/kata-containers/configuration.toml`

Key settings:

```toml
[hypervisor.qemu]
path = "/opt/kata/bin/qemu-system-x86_64"
kernel = "/opt/kata/share/kata-containers/vmlinux.container"
image = "/opt/kata/share/kata-containers/kata-containers.img"

# Memory settings
default_memory = 2048  # MB
memory_slots = 10
default_maxmemory = 0

# CPU settings
default_vcpus = 1
default_maxvcpus = 0

# Security features
enable_iommu = false
enable_iommu_platform = false
```

### Cloud Hypervisor Configuration

For better performance with Cloud Hypervisor:

```bash
# Use Cloud Hypervisor variant
docker run --rm --runtime=kata-clh alpine echo "Using Cloud Hypervisor"
```

## Troubleshooting

### Common Issues

1. **"kata-runtime: command not found"**
   ```bash
   # Check PATH
   export PATH=$PATH:/opt/kata/bin
   # Or symlink
   sudo ln -sf /opt/kata/bin/kata-runtime /usr/local/bin/
   ```

2. **"Hardware virtualization not available"**
   ```bash
   # Check CPU virtualization
   egrep -c '(vmx|svm)' /proc/cpuinfo
   # Should return > 0

   # Enable in BIOS if 0
   # For VMs, enable nested virtualization
   ```

3. **"QEMU not found"**
   ```bash
   # Install QEMU
   sudo apt-get install -y qemu-system-x86
   # Or use Cloud Hypervisor instead
   ```

4. **"Permission denied"**
   ```bash
   # Add user to kvm group
   sudo usermod -aG kvm $USER
   newgrp kvm
   ```

### Debug Mode

```bash
# Enable debug logging
sudo kata-runtime --log=/tmp/kata.log --log-level=debug run test-container

# Check logs
tail -f /tmp/kata.log
```

## Integration with Agent Runtime

### AIOS Integration

Configure AIOS to use Kata for isolated agent execution:

```python
# In AIOS config
ISOLATION_RUNTIME = "kata"
ISOLATION_MEMORY_LIMIT = "2Gi"
ISOLATION_CPU_LIMIT = 2
```

### AGiXT Integration

Update docker-compose.agixt.yml for Kata isolation:

```yaml
services:
  agixt:
    runtime: kata
    # ... other settings
```

## Security Considerations

### Threat Model

**Mitigated by Kata:**
- Container escape vulnerabilities
- Kernel exploits
- Resource exhaustion across containers
- Information leakage between containers

**Still Required:**
- Network policies
- Image scanning (Trivy)
- Runtime monitoring
- Secret management (Vault)

### Best Practices

1. Use Kata for untrusted workloads
2. Combine with network policies
3. Monitor VM resource usage
4. Regular security updates
5. Audit container runtime selection

## Status

- [x] Installation documentation
- [x] Docker configuration
- [x] containerd configuration
- [x] Kubernetes RuntimeClass
- [x] Troubleshooting guide
- [ ] Actual installation on target systems
- [ ] Performance benchmarks
- [ ] Integration tests

## References

- [Kata Containers Documentation](https://katacontainers.io/docs/)
- [Kata Containers GitHub](https://github.com/kata-containers/kata-containers)
- [Cloud Hypervisor](https://github.com/cloud-hypervisor/cloud-hypervisor)
- [BUILDKIT_STARTER_SPEC.md Layer 3](../../BUILDKIT_STARTER_SPEC.md#14-isolation--sandboxing)
