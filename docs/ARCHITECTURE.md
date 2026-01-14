# Architecture Overview

**Status:** Phase 3 Complete (Environment Taxonomy + Layering)
**Last Updated:** 2026-01-14
**Evidence:** flake.nix, pixi.toml, docker-compose.*.yml, ARIA_MANIFEST.yaml

---

## Environment Layer Diagram

```mermaid
graph TB
    subgraph "Layer 0: Host Operating System"
        L0_WIN[Windows 11/10]
        L0_LINUX[Linux]
        L0_MAC[macOS]
    end

    subgraph "Layer 1: Virtualization/Container Runtime"
        L1_WSL[WSL2 + NixOS-WSL]
        L1_DOCKER[Docker Engine]
        L1_PODMAN[Podman]
        L1_KATA[Kata Containers]
    end

    subgraph "Layer 2: Package Management"
        L2_NIX[Nix Flake<br/>System packages]
        L2_PIXI[Pixi<br/>Python/Conda packages]
        L2_HELM[Helm<br/>K8s packages]
    end

    subgraph "Layer 3: Development Shells"
        L3_DEFAULT[default<br/>Base ROS2 + tools]
        L3_FULL[full<br/>All optional tools]
        L3_CUDA[cuda<br/>GPU acceleration]
        L3_IDENTITY[identity<br/>Auth development]
    end

    subgraph "Layer 4: Pixi Environments"
        L4_DEFAULT[default<br/>ROS2 Humble]
        L4_CUDA[cuda<br/>PyTorch GPU]
        L4_AIOS[aios<br/>Agent OS]
        L4_LLMOPS[llmops<br/>LLM evaluation]
        L4_FINETUNE[finetuning<br/>Model training]
        L4_CACHE[caching<br/>LLM caching]
        L4_QUDAG[qudag<br/>Quantum-resistant]
    end

    subgraph "Layer 5: Docker Compose Stacks"
        L5_OBS[observability<br/>Prometheus, Grafana, Loki]
        L5_MSG[messaging<br/>NATS, Temporal]
        L5_ID[identity<br/>Keycloak, Vault, Step-CA]
        L5_AI[ai<br/>LocalAI, AGiXT, Refact]
        L5_DATA[data<br/>Neo4j, MindsDB, PostgreSQL]
        L5_EDGE[edge<br/>Kong, AgentGateway]
        L5_AUTO[automation<br/>n8n, OPA]
        L5_UI[ui<br/>Lobe Chat, Open-Lovable]
    end

    subgraph "Layer 6: Kubernetes (Optional)"
        L6_ARGO[ArgoCD<br/>GitOps]
        L6_ROLLOUTS[Argo Rollouts<br/>Deployments]
        L6_WORKFLOWS[Argo Workflows<br/>DAG execution]
    end

    %% Connections
    L0_WIN --> L1_WSL
    L0_LINUX --> L1_DOCKER
    L0_MAC --> L1_DOCKER

    L1_WSL --> L2_NIX
    L1_DOCKER --> L2_NIX
    L1_DOCKER --> L2_HELM

    L2_NIX --> L3_DEFAULT
    L2_NIX --> L3_FULL
    L2_NIX --> L3_CUDA

    L3_DEFAULT --> L4_DEFAULT
    L3_CUDA --> L4_CUDA
    L3_FULL --> L4_AIOS
    L3_FULL --> L4_LLMOPS

    L4_DEFAULT --> L5_OBS
    L4_AIOS --> L5_AI
    L4_DEFAULT --> L5_MSG
    L4_DEFAULT --> L5_ID

    L5_OBS --> L6_ARGO
    L5_MSG --> L6_WORKFLOWS

    %% Styling
    classDef host fill:#e3f2fd,stroke:#1565c0,stroke-width:2px
    classDef virt fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef pkg fill:#e8f5e9,stroke:#2e7d32,stroke-width:2px
    classDef shell fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef env fill:#fff8e1,stroke:#ff8f00,stroke-width:2px
    classDef docker fill:#ffebee,stroke:#c62828,stroke-width:2px
    classDef k8s fill:#e1f5fe,stroke:#0288d1,stroke-width:2px

    class L0_WIN,L0_LINUX,L0_MAC host
    class L1_WSL,L1_DOCKER,L1_PODMAN,L1_KATA virt
    class L2_NIX,L2_PIXI,L2_HELM pkg
    class L3_DEFAULT,L3_FULL,L3_CUDA,L3_IDENTITY shell
    class L4_DEFAULT,L4_CUDA,L4_AIOS,L4_LLMOPS,L4_FINETUNE,L4_CACHE,L4_QUDAG env
    class L5_OBS,L5_MSG,L5_ID,L5_AI,L5_DATA,L5_EDGE,L5_AUTO,L5_UI docker
    class L6_ARGO,L6_ROLLOUTS,L6_WORKFLOWS k8s
```

---

## Layer Descriptions

### Layer 0: Host Operating System

The foundation layer where the user runs their development environment.

| Platform | Bootstrap Script | Requirements |
|----------|-----------------|--------------|
| Windows 10/11 | `bootstrap.ps1` | Build 19041+, Admin rights, WSL2 capable |
| Linux (Ubuntu, Fedora, NixOS) | `bootstrap.sh` | systemd or equivalent, curl |
| macOS (Intel/ARM) | `bootstrap.sh` | Xcode CLI tools |

**Evidence:** `bootstrap.sh` lines 471-547 (detect_system), `bootstrap.ps1` lines 260-275

### Layer 1: Virtualization/Container Runtime

Container and VM runtimes that host services.

| Runtime | Purpose | Configuration |
|---------|---------|---------------|
| WSL2 | Windows Linux subsystem | `.wslconfig` for memory/swap |
| Docker Engine | Container runtime | `daemon.json` for storage/cgroups |
| Podman | Rootless containers | Alternative to Docker |
| Kata Containers | Sandboxed execution | For agent isolation |

**Evidence:** `bootstrap.ps1` lines 311-488, `docker/` directory

### Layer 2: Package Management

Three complementary package managers with different scopes.

| Manager | Scope | Lock File | Configuration |
|---------|-------|-----------|---------------|
| Nix | System packages, shells | `flake.lock` | `flake.nix` |
| Pixi | Python/Conda packages | `pixi.lock` | `pixi.toml` |
| Helm | Kubernetes packages | `Chart.lock` | `charts/*/Chart.yaml` |

**Evidence:** `flake.nix` (2000+ lines), `pixi.toml` (1200+ lines), `charts/flexstack/`

### Layer 3: Nix Development Shells

Pre-configured development environments with different tool sets.

| Shell | Command | Tools Included | Use Case |
|-------|---------|----------------|----------|
| default | `nix develop` | Nix, Pixi, direnv, git, gh, nom | Daily development |
| full | `nix develop .#full` | + bat, eza, fd, rg, jq, yq, trivy, kubectl, helm | Full toolchain |
| cuda | `nix develop .#cuda` | + CUDA toolkit, cuDNN | GPU development |
| identity | `nix develop .#identity` | + Vault, Keycloak tools | Auth development |

**Evidence:** `flake.nix` devShells section, `ARIA_MANIFEST.yaml` devshells section

### Layer 4: Pixi Environments

Python/Conda environments for specific workloads.

| Environment | Solve Group | Key Packages | Purpose |
|-------------|-------------|--------------|---------|
| default | default | ROS2 Humble, Python 3.11, NumPy, Pandas | Base robotics |
| cuda | cuda | PyTorch CUDA, CuPy | GPU compute |
| aios | aios | LiteLLM, ChromaDB, LangChain | Agent runtime |
| aios-cuda | aios-cuda | + CUDA support | GPU agents |
| llmops | llmops | TruLens, promptfoo | LLM evaluation |
| finetuning | finetuning | PEFT, transformers, datasets | Model training |
| caching | caching | vCache, Redis | Prompt caching |
| docs | docs | MkDocs, Material | Documentation |
| qudag | qudag | Qiskit, quantum-resistant crypto | Quantum research |

**Evidence:** `pixi.toml` environments section, `ARIA_MANIFEST.yaml` pixi_components

### Layer 5: Docker Compose Stacks

Service stacks deployed via Docker Compose.

| Stack | Services | Ports | Configuration |
|-------|----------|-------|---------------|
| observability | Prometheus, Grafana, Loki, Tempo, Alertmanager | 9090, 3000, 3100, 3200, 9093 | `docker-compose.observability.yml` |
| messaging | NATS, Temporal | 4222, 8222, 7233 | `docker-compose.messaging.yml` |
| identity | Keycloak, Vault, Step-CA, Vaultwarden | 8080, 8200, 9000, 8090 | `docker-compose.identity.yml` |
| ai | LocalAI, AGiXT, Refact | 8080, 7437, 8008 | `docker-compose.*.yml` |
| data | Neo4j, MindsDB, PostgreSQL, ClickHouse | 7474, 47334, 5432 | `docker-compose.data.yml` |
| edge | Kong, AgentGateway | 8000, 8001 | `docker-compose.edge.yml` |
| automation | n8n, OPA | 5678, 8181 | `docker-compose.automation.yml` |
| ui | Lobe Chat, Open-Lovable | 3210, 3211 | `docker-compose.ui.yml` |

**Evidence:** `docker/docker-compose.*.yml` files, `ARIA_MANIFEST.yaml` docker_services

### Layer 6: Kubernetes (Optional)

GitOps-based Kubernetes deployment for production.

| Component | Purpose | Manifests |
|-----------|---------|-----------|
| ArgoCD | GitOps controller | `manifests/argocd/` |
| Argo Rollouts | Progressive delivery | `manifests/argo-rollouts/` |
| Argo Workflows | DAG execution | `manifests/argo-workflows/` |

**Evidence:** `manifests/` directory, `install-argocd.sh`, `install-argo-rollouts.sh`

---

## Package Distribution Strategy

```mermaid
graph LR
    subgraph "Nix (System Layer)"
        NIX[flake.nix] --> SYS[System Tools<br/>git, curl, jq]
        NIX --> CLI[CLI Tools<br/>kubectl, helm, trivy]
        NIX --> LANG[Language Runtimes<br/>Node.js, Rust]
    end

    subgraph "Pixi (Python Layer)"
        PIXI[pixi.toml] --> ROS[ROS2 Packages<br/>ros-humble-*]
        PIXI --> PY[Python Packages<br/>numpy, pandas, torch]
        PIXI --> ML[ML Packages<br/>transformers, peft]
    end

    subgraph "Docker (Service Layer)"
        DOCKER[docker-compose.*.yml] --> DB[Databases<br/>PostgreSQL, Neo4j]
        DOCKER --> MQ[Message Queues<br/>NATS, Redis]
        DOCKER --> AI[AI Services<br/>LocalAI, AGiXT]
    end

    SYS --> PIXI
    CLI --> DOCKER
    ROS --> DOCKER
```

### Distribution Rules

1. **System tools** (curl, git, jq, yq) → **Nix** (stable, reproducible)
2. **CLI tools** (kubectl, helm, trivy) → **Nix** (pinned versions)
3. **ROS2 packages** → **Pixi** (RoboStack channel)
4. **Python packages** → **Pixi** (conda-forge channel)
5. **Services** → **Docker** (isolated, configurable)
6. **Production** → **Kubernetes** (scalable, observable)

**Evidence:** `ARIA_MANIFEST.yaml` validation rules, `docs/CHANNEL-STRATEGY.md`

---

## Network Topology

```mermaid
graph TB
    subgraph "External"
        USER[User/Client]
        GITHUB[GitHub Actions]
    end

    subgraph "Edge Layer"
        KONG[Kong Gateway<br/>:8000, :8001]
        AGENT_GW[AgentGateway<br/>:9090]
    end

    subgraph "Service Mesh"
        NATS[NATS<br/>:4222, :8222]
        TEMPORAL[Temporal<br/>:7233]
    end

    subgraph "Auth Layer"
        KC[Keycloak<br/>:8080]
        VAULT[Vault<br/>:8200]
        STEP_CA[Step-CA<br/>:9000]
    end

    subgraph "AI Layer"
        LOCALAI[LocalAI<br/>:8080]
        AGIXT[AGiXT<br/>:7437]
        REFACT[Refact<br/>:8008]
    end

    subgraph "Data Layer"
        PG[PostgreSQL<br/>:5432]
        NEO4J[Neo4j<br/>:7474]
        REDIS[Redis<br/>:6379]
        MINIO[MinIO<br/>:9000]
    end

    subgraph "Observability"
        PROM[Prometheus<br/>:9090]
        GRAF[Grafana<br/>:3000]
        LOKI[Loki<br/>:3100]
    end

    USER --> KONG
    GITHUB --> KONG
    KONG --> KC
    KONG --> LOCALAI
    KONG --> AGIXT

    AGENT_GW --> NATS
    AGENT_GW --> TEMPORAL

    LOCALAI --> REDIS
    AGIXT --> PG
    AGIXT --> NEO4J

    KC --> PG
    VAULT --> PG

    PROM --> KONG
    PROM --> NATS
    PROM --> LOCALAI
    GRAF --> PROM
    GRAF --> LOKI
```

---

## Security Boundaries

```mermaid
graph TB
    subgraph "Trust Zone: External"
        EXT[External Clients]
    end

    subgraph "Trust Zone: Edge"
        EDGE[Kong + mTLS]
    end

    subgraph "Trust Zone: Service"
        SVC[Internal Services]
    end

    subgraph "Trust Zone: Data"
        DATA[Databases + Secrets]
    end

    EXT -->|HTTPS + OAuth2| EDGE
    EDGE -->|mTLS| SVC
    SVC -->|Vault Token| DATA

    style EXT fill:#ffebee,stroke:#c62828
    style EDGE fill:#fff3e0,stroke:#e65100
    style SVC fill:#e8f5e9,stroke:#2e7d32
    style DATA fill:#e3f2fd,stroke:#1565c0
```

### Security Components

| Layer | Component | Authentication | Configuration |
|-------|-----------|----------------|---------------|
| Edge | Kong Gateway | OAuth2/OIDC, API Keys | `config/kong/` |
| Edge | Step-CA | X.509 Certificates | `config/step-ca/` |
| Service | Keycloak | OIDC Tokens | `config/keycloak/` |
| Service | OPA | Policy Decisions | `config/opa/policies/` |
| Data | Vault | Token/Certificate | `config/vault/` |
| Data | PostgreSQL | Username/Password | Docker Compose env |

**Evidence:** `docker-compose.identity.yml`, `scripts/generate-service-certs.sh`, `scripts/init-step-ca.sh`

---

## Build Artifacts

| Artifact Type | Build Command | Output Location | Purpose |
|---------------|---------------|-----------------|---------|
| NixOS WSL Tarball | `nix build .#wsl-ripple` | `result/nixos-wsl.tar.gz` | WSL distribution |
| NixOS ISO | `nix build .#iso-ros2` | `result/nixos.iso` | Bootable installer |
| NixOS VM | `nix build .#vm-ros2` | `result/*.qcow2` | QEMU VM image |
| Docker Images | `docker compose build` | Local registry | Service containers |
| Helm Chart | `helm package charts/flexstack` | `*.tgz` | K8s deployment |
| ROS2 Packages | `colcon build` | `install/` | ROS2 nodes |

**Evidence:** `docs/NIXOS_IMAGES.md`, `.github/workflows/nixos-images.yml`

---

## References

- `flake.nix` — Nix flake configuration
- `pixi.toml` — Pixi package configuration
- `ARIA_MANIFEST.yaml` — Component verification manifest
- `docker/docker-compose.*.yml` — Docker Compose stacks
- `docs/BUILDKIT_STARTER_SPEC.md` — Infrastructure specification
- `docs/MTLS_SETUP.md` — mTLS configuration guide
