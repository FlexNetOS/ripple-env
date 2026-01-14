---
title: Deployment
description: Platform-specific deployment guides
tags:
  - deployment
  - wsl2
  - edge
  - kubernetes
---

# Deployment

Deploy ripple-env to various platforms including WSL2, edge devices, and Kubernetes clusters.

## Overview

```mermaid
graph TB
    subgraph "Build"
        SRC[Source Code]
        NIX[Nix Build]
    end

    subgraph "Artifacts"
        ISO[ISO Image]
        TAR[WSL Tarball]
        OCI[Container Image]
        QCOW[QCOW2 VM]
    end

    subgraph "Targets"
        WSL[WSL2]
        BARE[Bare Metal]
        K8S[Kubernetes]
        EDGE[Edge Devices]
    end

    SRC --> NIX

    NIX --> ISO
    NIX --> TAR
    NIX --> OCI
    NIX --> QCOW

    TAR --> WSL
    ISO --> BARE
    OCI --> K8S
    QCOW --> EDGE

    style NIX fill:#4051b5,color:#fff
    style WSL fill:#00bfa5,color:#fff
    style K8S fill:#00bfa5,color:#fff
```

## Quick Start

=== "WSL2"

    ```powershell
    # Build WSL tarball
    nix build .#nixosConfigurations.wsl-ripple.config.system.build.tarballBuilder

    # Import to WSL
    wsl --import NixOS-Ripple $env:USERPROFILE\WSL\NixOS-Ripple result/nixos-wsl.tar.gz
    ```

=== "Container"

    ```bash
    # Build container image
    nix build .#dockerImage

    # Load into Docker
    docker load < result
    ```

=== "ISO"

    ```bash
    # Build ISO installer
    nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage
    ```

## Documentation

<div class="grid cards" markdown>

-   :material-chip:{ .lg .middle } __Edge Deployment__

    ---

    Deploy to Raspberry Pi, Jetson, and embedded devices

    [:octicons-arrow-right-24: Edge Guide](edge.md)

-   :material-microsoft-windows:{ .lg .middle } __WSL2 Pipeline__

    ---

    Windows Subsystem for Linux deployment

    [:octicons-arrow-right-24: WSL2 Guide](wsl2.md)

-   :simple-nixos:{ .lg .middle } __NixOS Images__

    ---

    Generate NixOS images for various targets

    [:octicons-arrow-right-24: NixOS Images](nixos-images.md)

-   :material-cube-outline:{ .lg .middle } __Kata Containers__

    ---

    Lightweight VM-based container runtime

    [:octicons-arrow-right-24: Kata Guide](kata.md)

</div>

## Supported Platforms

| Platform | Architecture | Status |
|----------|-------------|--------|
| Linux (NixOS) | x86_64, aarch64 | Supported |
| macOS | x86_64, arm64 | Supported |
| Windows (WSL2) | x86_64 | Supported |
| Raspberry Pi | aarch64 | Supported |
| Jetson | aarch64 | Supported |
| Kubernetes | any | Supported |
