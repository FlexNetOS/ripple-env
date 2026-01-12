---
title: Architecture
description: System design and architecture decisions
tags:
  - architecture
  - design
  - adr
---

# Architecture

This section documents the system architecture, design decisions, and technical rationale behind ripple-env.

## System Overview

```mermaid
graph TB
    subgraph "Entry Points"
        CLI[CLI Tools]
        IDE[IDE Integration]
        CI[CI/CD Pipelines]
    end

    subgraph "Core Platform"
        NIX[Nix Flake]
        PIXI[Pixi Environment]
        DIRENV[direnv]
    end

    subgraph "Development"
        ROS[ROS2 Humble]
        PY[Python 3.11+]
        RUST[Rust Toolchain]
    end

    subgraph "Services"
        AI[AI Services]
        OBS[Observability]
        SEC[Security]
    end

    subgraph "Deployment"
        WSL[WSL2]
        K8S[Kubernetes]
        EDGE[Edge Devices]
    end

    CLI --> NIX
    IDE --> DIRENV
    CI --> NIX

    NIX --> ROS
    NIX --> PY
    NIX --> RUST
    PIXI --> PY
    DIRENV --> NIX

    ROS --> AI
    PY --> AI
    AI --> OBS
    OBS --> SEC

    NIX --> WSL
    NIX --> K8S
    NIX --> EDGE

    style NIX fill:#4051b5,color:#fff
    style ROS fill:#00bfa5,color:#fff
    style AI fill:#7c4dff,color:#fff
```

## Documentation

<div class="grid cards" markdown>

-   :material-sitemap:{ .lg .middle } __System Overview__

    ---

    High-level architecture and component relationships

    [:octicons-arrow-right-24: Overview](overview.md)

-   :simple-nixos:{ .lg .middle } __Nix Flake Design__

    ---

    Flake structure and modularization

    [:octicons-arrow-right-24: Nix Flake](nix-flake.md)

-   :material-robot:{ .lg .middle } __ARIA Orchestrator__

    ---

    Multi-agent orchestration architecture

    [:octicons-arrow-right-24: ARIA](aria.md)

-   :material-file-document:{ .lg .middle } __Decision Records__

    ---

    Architecture Decision Records (ADRs)

    [:octicons-arrow-right-24: ADRs](decisions/index.md)

</div>

## Design Principles

1. **Reproducibility**: Every build produces identical results
2. **Cross-platform**: Works on Linux, macOS, and Windows (WSL2)
3. **Modularity**: Components can be used independently
4. **Security-first**: mTLS, secrets management, SBOM by default
5. **Observable**: Built-in metrics, logging, and tracing

## Component Layers

```mermaid
graph LR
    subgraph "Layer 1: Foundation"
        L1[Nix + Pixi]
    end

    subgraph "Layer 2: Core"
        L2[ROS2 + Python]
    end

    subgraph "Layer 3: Services"
        L3[AI + Observability]
    end

    subgraph "Layer 4: Applications"
        L4[Agents + Robots]
    end

    L1 --> L2 --> L3 --> L4
```

See the [BUILDKIT_STARTER_SPEC.md](https://github.com/FlexNetOS/ripple-env/blob/main/BUILDKIT_STARTER_SPEC.md) for the complete layer specification.
