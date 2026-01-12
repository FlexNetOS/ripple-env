---
title: System Overview
description: High-level architecture and component relationships
tags:
  - architecture
  - overview
---

# System Overview

ripple-env is a cross-platform ROS2 development environment built on three pillars:

1. **Nix Flake**: Reproducible system packages
2. **Pixi**: Python and Conda ecosystem
3. **Docker Compose**: Containerized services

## Architecture Diagram

```mermaid
graph TB
    subgraph "User Interface"
        CLI[CLI Tools]
        IDE[IDE Integration]
        WEB[Web UI]
    end

    subgraph "Entry Layer"
        DIRENV[direnv]
        SHELL[Shell Scripts]
        HOOKS[Git Hooks]
    end

    subgraph "Core Platform"
        NIX[Nix Flake]
        PIXI[Pixi Environment]
    end

    subgraph "Development Tools"
        ROS[ROS2 Humble]
        PY[Python 3.11+]
        RUST[Rust Toolchain]
        NODE[Node.js]
    end

    subgraph "AI Services"
        LOCALAI[LocalAI]
        AGIXT[AGiXT]
        MINDSDB[MindsDB]
    end

    subgraph "Infrastructure"
        DOCKER[Docker Compose]
        K8S[Kubernetes]
        TEMPORAL[Temporal]
    end

    subgraph "Observability"
        PROM[Prometheus]
        GRAF[Grafana]
        OTEL[OpenTelemetry]
    end

    CLI --> DIRENV
    IDE --> DIRENV

    DIRENV --> NIX
    SHELL --> NIX

    NIX --> ROS
    NIX --> PY
    NIX --> RUST
    PIXI --> PY
    PIXI --> NODE

    ROS --> LOCALAI
    PY --> AGIXT

    LOCALAI --> DOCKER
    AGIXT --> DOCKER
    MINDSDB --> DOCKER
    TEMPORAL --> DOCKER

    DOCKER --> PROM
    K8S --> PROM
    PROM --> GRAF
    OTEL --> GRAF

    style NIX fill:#4051b5,color:#fff
    style PIXI fill:#4051b5,color:#fff
    style ROS fill:#00bfa5,color:#fff
    style GRAF fill:#f46800,color:#fff
```

## Layer Architecture

The system follows a layered architecture as defined in BUILDKIT_STARTER_SPEC.md:

| Layer | Components | Purpose |
|-------|------------|---------|
| 1 | Nix, Pixi, direnv | Foundation |
| 2 | Git, Pre-commit | Version Control |
| 3 | Python, Rust, Node | Languages |
| 4 | ROS2, Colcon | Robotics |
| 5 | Testing, Linting | Quality |
| 6 | Docker, K8s | Containers |
| 7 | Temporal, NATS | Orchestration |
| 8 | AI Services | Intelligence |
| 9 | Security, mTLS | Protection |
| 10 | Observability | Monitoring |

## Data Flow

```mermaid
sequenceDiagram
    participant Dev as Developer
    participant NIX as Nix
    participant ENV as Environment
    participant SVC as Services
    participant OBS as Observability

    Dev->>NIX: nix develop
    NIX->>ENV: Load packages
    ENV->>Dev: Shell ready

    Dev->>SVC: Start services
    SVC->>OBS: Send metrics
    OBS->>Dev: Dashboard
```

## Key Design Decisions

1. **Nix for reproducibility**: Ensures consistent environments across machines
2. **Pixi for Python**: Faster dependency resolution than conda
3. **direnv for activation**: Automatic environment loading
4. **Docker for services**: Isolated service deployment
5. **Flake-parts for modularity**: Composable Nix configuration
