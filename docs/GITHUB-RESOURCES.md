# GitHub Resources Research & Analysis

> **Status**: Research Complete
> **Last Updated**: January 2026

This document catalogs external GitHub resources evaluated for integration with the ros2-humble-env project, including Nix/Pixi compatibility analysis and conflict assessment.

---

## Quick Reference - Integration Status

| Category | Top Recommendations | Status | Priority |
|----------|--------------------| ------------|----------|
| **AI Agents** | LocalAI, AIOS | ⚠️ Docker/Pixi | High |
| **Monitoring** | Prometheus, Trivy, Trippy | ✅ **Integrated** | High |
| **Messaging** | NATS Server + CLI | ✅ **Integrated** | High |
| **Security** | OPA, Trivy, Vault, gVisor | ✅ **Integrated** | High |
| **Identity** | Keycloak, Vaultwarden | ✅ **Integrated** (identity shell) | High |
| **Evaluation** | Promptfoo | ✅ **Integrated** | High |
| **Memory** | Memori, Memobase | ⚠️ Pixi | Medium |
| **Rust** | maturin, sqlx-cli | ✅ **Integrated** | High |

---

## ⚠️ Critical Conflicts Warning

Before integrating any tools, be aware of these conflicts:

| Tool | Conflict | Resolution |
|------|----------|------------|
| **Unsloth** | PyTorch version specificity (3.11 only) | Use Docker: `docker run -it unsloth/unsloth` |
| **ComfyUI** | PyTorch/CUDA conflicts with ROS2 env | Use dedicated flake: `nix run github:utensils/comfyui-nix` |
| **Agentic Flow** | Node.js ecosystem, not Python | ❌ **Not recommended** for ROS2 |
| **AGiXT** | Requires Docker Compose | Docker only, no Nix integration |
| **SQLx** | Requires database at compile time | Use `naersk` with prepared DB image |
| **PyO3/maturin** | Complex Nix+Python builds | Use Pixi for builds, `naersk` for packaging |
| **Trivy** | Limited NixOS filesystem scanning | Use `vulnix` for Nix packages instead |

### Python Version Matrix

| Tool | Required Python | ROS2 Humble (3.11) |
|------|-----------------|-------------------|
| AIOS/Cerebrum | 3.10/3.11 only | ✅ Compatible |
| TruLens | 3.9+ | ✅ Compatible |
| Memori | 3.10+ | ✅ Compatible |
| ComfyUI | 3.12-3.13 | ❌ **Separate env required** |
| Unsloth | 3.11-3.13 | ⚠️ Separate env recommended |

---

## Table of Contents

1. [AI Agent Platforms](#ai-agent-platforms)
2. [Infrastructure & Monitoring](#infrastructure--monitoring)
3. [Networking & P2P](#networking--p2p)
4. [Memory & Database Systems](#memory--database-systems)
5. [API Gateway & Service Mesh](#api-gateway--service-mesh)
6. [Rust Ecosystem Tools](#rust-ecosystem-tools)
7. [DevOps & Orchestration](#devops--orchestration)
8. [Security & Identity](#security--identity)
9. [AI/ML Evaluation & Workflows](#aiml-evaluation--workflows)
10. [Storage & Distributed Systems](#storage--distributed-systems)
11. [Pixi Package Additions](#pixi-package-additions)
12. [Conflict Analysis](#conflict-analysis)
13. [Integration Recommendations](#integration-recommendations)

---

## AI Agent Platforms

### LocalAI - Distributed P2P Inference ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [mudler/LocalAI](https://github.com/mudler/LocalAI) |
| **Stars** | 41.2k |
| **Description** | Distributed, P2P and decentralized inference - OpenAI-compatible drop-in |
| **Nix Package** | ✅ `pkgs.local-ai` (v2.24.2) |
| **Installation** | `nix shell nixpkgs#local-ai` or Docker |
| **Status** | ✅ Ready for integration |
| **Relevance** | **Critical** - Edge AI inference for robots |

**Key Features**: Multi-backend (llama.cpp, vLLM, Ollama), ARM64/Jetson support, P2P swarms

**ROS2 Integration**:
```nix
# Add to flake.nix devshell
commonPackages = with pkgs; [ local-ai ];
```

### AIOS - AI Agent Operating System ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [agiresearch/AIOS](https://github.com/agiresearch/AIOS) |
| **Stars** | ~2k |
| **Agent SDK** | [agiresearch/Cerebrum](https://github.com/agiresearch/Cerebrum) |
| **Nix Package** | ❌ Not packaged (pure Python) |
| **Installation** | `pixi add python=3.10` then `pip install aios-agent-sdk` |
| **Status** | ⚠️ Requires Pixi setup |
| **Relevance** | **High** - Best fit for robotics agent orchestration |

**Key Features**: LLM kernel scheduling, memory management, tool coordination, ReAct/MetaGPT support

### AGiXT - Dynamic AI Agent Automation Platform
| Attribute | Value |
|-----------|-------|
| **Repository** | [Josh-XT/AGiXT](https://github.com/Josh-XT/AGiXT) |
| **Stars** | 2.7k+ |
| **Rust SDK** | [AGiXT/rust-sdk](https://github.com/AGiXT/rust-sdk) |
| **Nix Package** | ❌ Not packaged |
| **Installation** | Docker Compose only |
| **Status** | ⚠️ Requires Docker |
| **Relevance** | Medium - Enterprise features, requires Docker |

### Agentic Flow - AI Agent Orchestration
| Attribute | Value |
|-----------|-------|
| **Repository** | [ruvnet/agentic-flow](https://github.com/ruvnet/agentic-flow) |
| **Stars** | 325 |
| **Language** | TypeScript/Node.js |
| **Nix Package** | ❌ Not packaged (Node.js) |
| **Status** | ❌ Not recommended for ROS2 |
| **Relevance** | Low - Node.js ecosystem conflicts |

### Agent Gateway
| Attribute | Value |
|-----------|-------|
| **Repository** | [agentgateway/agentgateway](https://github.com/agentgateway/agentgateway) |
| **Stars** | 1,384 |
| **Language** | Rust (74.6%) |
| **Status** | ⚠️ Infrastructure layer |
| **Relevance** | Medium - Multi-robot routing (advanced use) |

---

## Infrastructure & Monitoring

### Prometheus - Metrics ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [prometheus/prometheus](https://github.com/prometheus/prometheus) |
| **Stars** | 62.1k |
| **Version** | v3.9.1 (Jan 2026) |
| **Nix Package** | ✅ `pkgs.prometheus` |
| **NixOS Module** | ✅ `services.prometheus` |
| **Status** | ✅ Ready for integration |
| **Relevance** | **Critical** - ROS2 DDS metrics via Fast-DDS statistics |

**ROS2 Integration**: Official Vulcanexus/Fast-DDS support, `ros2_monitor_grafana` community project

### Netdata - Real-Time Monitoring ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [netdata/netdata](https://github.com/netdata/netdata) |
| **Stars** | 77.3k |
| **Nix Package** | ✅ `pkgs.netdata` |
| **NixOS Module** | ✅ `services.netdata` |
| **Status** | ✅ Ready for integration |
| **Relevance** | **High** - Zero-config edge monitoring with ML anomaly detection |

### Trivy - Security Scanner
| Attribute | Value |
|-----------|-------|
| **Repository** | [aquasecurity/trivy](https://github.com/aquasecurity/trivy) |
| **Stars** | 30.9k |
| **Version** | v0.68.2 |
| **Nix Package** | ✅ `pkgs.trivy` |
| **Status** | ⚠️ Limited NixOS scanning |
| **Relevance** | High - Container/SBOM scanning (use `vulnix` for Nix packages) |

### Trippy - Network Diagnostics
| Attribute | Value |
|-----------|-------|
| **Repository** | [fujiapple852/trippy](https://github.com/fujiapple852/trippy) |
| **Stars** | 6.5k |
| **Nix Package** | ✅ `pkgs.trippy` |
| **NixOS Module** | ✅ `programs.trippy` |
| **Status** | ✅ Ready for integration |
| **Relevance** | Medium - Network path analysis for DDS traffic |

### OpenTelemetry
| Attribute | Value |
|-----------|-------|
| **Repository** | [open-telemetry](https://github.com/open-telemetry) |
| **Nix Package** | ⚠️ Python packages via pixi |
| **Installation** | `pixi add opentelemetry-api opentelemetry-sdk` |
| **Status** | ⚠️ Supplementary to ros2_tracing |
| **Relevance** | Medium - Distributed tracing (ROS2 uses LTTng natively) |

### Unsloth - Fast LLM Fine-tuning
| Attribute | Value |
|-----------|-------|
| **Repository** | [unslothai/unsloth](https://github.com/unslothai/unsloth) |
| **Nix Package** | ❌ Not packaged |
| **Installation** | Docker: `docker run -it unsloth/unsloth` |
| **Status** | ⚠️ Requires isolated GPU environment |
| **Relevance** | Medium - Model training (use Docker/separate env) |

---

## Networking & P2P

### NATS Server - Messaging ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [nats-io/nats-server](https://github.com/nats-io/nats-server) |
| **Stars** | 18.9k |
| **Version** | v2.12.3 |
| **Nix Package** | ✅ `pkgs.nats-server`, `pkgs.natscli` |
| **Devenv Support** | ✅ `services.nats` |
| **Status** | ✅ **Integrated in devshell** |
| **Relevance** | **High** - WAN/multi-site robot communication (complementary to DDS) |

**Integration Note**: Both `nats-server` and `natscli` are included in the devshell for local development and testing.

**Use Case**: DDS for in-robot, NATS for cross-network. Bridge via `nats-ros-connector`.

### Rust libp2p - P2P Networking
| Attribute | Value |
|-----------|-------|
| **Repository** | [libp2p/rust-libp2p](https://github.com/libp2p/rust-libp2p) |
| **Version** | 0.56.0 |
| **Nix Package** | ❌ Build via `rustPlatform.buildRustPackage` |
| **Installation** | `cargo add libp2p` |
| **Status** | ⚠️ Requires Rust toolchain |
| **Relevance** | High - Robot mesh networking (Robonomics uses this) |

### NOA Dynamo - Distributed Inference
| Attribute | Value |
|-----------|-------|
| **Repository** | [FlexNetOS/dynamo/tree/noa-dynamo](https://github.com/FlexNetOS/dynamo/tree/noa-dynamo) |
| **Status** | 🔬 Internal project |
| **Relevance** | High - Multi-robot inference at scale |

### ChirpStack - LoRaWAN
| Attribute | Value |
|-----------|-------|
| **Repository** | [chirpstack/chirpstack](https://github.com/chirpstack/chirpstack) |
| **Status** | 🔬 IoT-specific |
| **Relevance** | Medium - Long-range robot/IoT networks |

---

## Memory & Database Systems

### Memori - SQL Native Memory ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [MemoriLabs/Memori](https://github.com/MemoriLabs/Memori) |
| **Version** | 3.1.0+ |
| **Installation** | `pixi add memori` or `pip install memori` |
| **Databases** | SQLite (built-in), PostgreSQL, MySQL |
| **Status** | ✅ Ready via Pixi |
| **Relevance** | **High** - Robot knowledge graph, skill registry |

### Memobase - Long-Term Memory
| Attribute | Value |
|-----------|-------|
| **Repository** | [memodb-io/memobase](https://github.com/memodb-io/memobase) |
| **Version** | 0.0.36 |
| **Installation** | `pixi add memobase` |
| **Status** | ✅ Ready via Pixi |
| **Relevance** | High - User profile-based robot memory |

### SQLx - Rust SQL Toolkit ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [launchbadge/sqlx](https://github.com/launchbadge/sqlx) |
| **Downloads** | 54.9M |
| **Nix Package** | ✅ `pkgs.sqlx-cli` |
| **Installation** | `cargo add sqlx` (sqlx-cli available in devshell) |
| **Status** | ✅ **CLI integrated in devshell** |
| **Relevance** | High - Rust database access |

**Integration Note**: `sqlx-cli` is included in the devshell for database migrations and schema management. For library builds requiring compile-time DB verification, use `naersk` with a prepared DB image.

### Neon - Serverless Postgres
| Attribute | Value |
|-----------|-------|
| **Repository** | [neondatabase/neon](https://github.com/neondatabase/neon) |
| **Status** | 🔬 Cloud service |
| **Relevance** | Low - Development databases (prefer local PostgreSQL) |

---

## API Gateway & Service Mesh

### Kong Gateway
| Attribute | Value |
|-----------|-------|
| **Repository** | [Kong/kong](https://github.com/Kong/kong) |
| **Nix Package** | ✅ `pkgs.kong` |
| **Status** | ✅ Available |
| **Relevance** | Medium - API management for robot services |

---

## Rust Ecosystem Tools

### PyO3 - Rust-Python Bindings ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [PyO3/pyo3](https://github.com/PyO3/pyo3) |
| **Version** | 0.21 |
| **Requires** | Rust 1.83+, maturin |
| **Installation** | `cargo add pyo3` (maturin available in devshell) |
| **Status** | ✅ **maturin integrated in devshell** |
| **Relevance** | **High** - ROS2 Python-Rust interop |

**Integration Note**: `maturin` build tool is included in the devshell for building PyO3-based Python packages from Rust.

### RustCoder MCP - AI-Assisted Rust Dev
| Attribute | Value |
|-----------|-------|
| **Repository** | [cardea-mcp/RustCoder](https://github.com/cardea-mcp/RustCoder) |
| **Status** | ⚠️ MCP server (Python-based) |
| **Relevance** | High - AI-assisted ROS2 Rust package generation |

### uutils coreutils
| Attribute | Value |
|-----------|-------|
| **Repository** | [uutils/coreutils](https://github.com/uutils/coreutils) |
| **Nix Package** | ✅ `pkgs.uutils-coreutils` |
| **Status** | ✅ Ready |
| **Relevance** | Low - System utilities |

### syn - Rust Parser
| Attribute | Value |
|-----------|-------|
| **Repository** | [dtolnay/syn](https://github.com/dtolnay/syn) |
| **Downloads** | 1.27B |
| **Status** | ✅ Standard crate |
| **Relevance** | Medium - Proc-macro development |

### Kellnr - Private Rust Registry
| Attribute | Value |
|-----------|-------|
| **Repository** | [kellnr/kellnr](https://github.com/kellnr/kellnr) |
| **Installation** | Docker: `docker run -p 8000:8000 ghcr.io/kellnr/kellnr:5` |
| **Status** | ⚠️ Nix pure builds need workarounds |
| **Relevance** | Medium - Private robot driver crates |

---

## DevOps & Orchestration

### Temporal - Workflow Engine ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [temporalio/temporal](https://github.com/temporalio/temporal) |
| **Nix Package** | ❌ Not packaged |
| **Installation** | Docker Compose (recommended) |
| **Status** | ⚠️ Use Docker |
| **Relevance** | **High** - Durable robot mission workflows |

### Argo CD - GitOps
| Attribute | Value |
|-----------|-------|
| **Repository** | [argoproj/argo-cd](https://github.com/argoproj/argo-cd) |
| **Nix Package** | ✅ `pkgs.argocd` |
| **Status** | ✅ Ready (Kubernetes required) |
| **Relevance** | High - GitOps robot deployments |

**Nix Enhancement**: Use [nixidy](https://nixidy.dev/) for Nix-native Kubernetes configs

### gVisor Sandbox ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [google/gvisor](https://github.com/google/gvisor) |
| **Nix Package** | ✅ `pkgs.gvisor` |
| **Status** | ✅ **Integrated in devshell** (Linux only) |
| **Relevance** | High - Sandbox untrusted ROS2 packages |

**Integration Note**: gVisor (`runsc`) is included in the Linux devshell for container sandboxing. Usage: `docker run --runtime=runsc ...`. Performance overhead is 5-15% for network workloads. Not suitable for hard real-time control loops.

---

## Security & Identity

### HashiCorp Vault ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [hashicorp/vault](https://github.com/hashicorp/vault) |
| **Nix Package** | ✅ `pkgs.vault` (BSL license) |
| **NixOS Module** | ✅ `services.vault` |
| **Status** | ✅ **Integrated in devshell** |
| **Relevance** | **High** - DDS-Security PKI, API key management |

**Integration Note**: Vault CLI is included in the full devshell. Use `vault-dev` helper to start dev server (auto-unsealed, root token: `root`). Requires `NIXPKGS_ALLOW_UNFREE=1` due to BSL license. For production, use NixOS `services.vault` module.

### Keycloak - Identity Management ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [keycloak/keycloak](https://github.com/keycloak/keycloak) |
| **Nix Package** | ✅ `pkgs.keycloak` |
| **NixOS Module** | ✅ `services.keycloak` (NixOS 25.05+) |
| **Status** | ✅ **Integrated in identity devshell** |
| **Relevance** | **High** - Robot fleet OAuth2/OIDC authentication |

**Integration Note**: Keycloak is included in the `devShells.identity` shell (Linux only). Usage: `nix develop .#identity`. Requires Java 21 and PostgreSQL (both included). Start with: `keycloak start-dev --http-port=8080`.

### Vaultwarden ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [dani-garcia/vaultwarden](https://github.com/dani-garcia/vaultwarden) |
| **Nix Package** | ✅ `pkgs.vaultwarden` |
| **NixOS Module** | ✅ `services.vaultwarden` |
| **Status** | ✅ **Integrated in identity devshell** |
| **Relevance** | Medium - Team password management |

**Integration Note**: Vaultwarden is included in the `devShells.identity` shell (Linux only). Usage: `nix develop .#identity`. Supports SQLite (default) or PostgreSQL. Start with: `vaultwarden`.

### Open Policy Agent (OPA) ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [open-policy-agent/opa](https://github.com/open-policy-agent/opa) |
| **Nix Package** | ✅ `pkgs.opa` |
| **Status** | ✅ **Integrated in devshell** |
| **Relevance** | High - ROS2 topic access policies |

**Integration Note**: OPA CLI is included in the full devshell. Use Rego policies for ROS2 topic/service access control. Run `opa run --server` for policy server mode. See OPA docs for DDS-Security integration patterns.

---

## AI/ML Evaluation & Workflows

### TruLens - LLM Evaluation ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [truera/trulens](https://github.com/truera/trulens) |
| **Version** | 2.5.2 |
| **Installation** | `pixi add trulens` |
| **Status** | ✅ Ready via Pixi |
| **Relevance** | **High** - Robot LLM decision quality tracking |

### OpenAI Evals ⭐ RECOMMENDED
| Attribute | Value |
|-----------|-------|
| **Repository** | [openai/evals](https://github.com/openai/evals) |
| **Installation** | `pip install evals` |
| **Status** | ✅ Ready via Pixi |
| **Relevance** | **High** - LLM benchmark evaluation |

### Promptfoo ⭐ INTEGRATED
| Attribute | Value |
|-----------|-------|
| **Repository** | [promptfoo/promptfoo](https://github.com/promptfoo/promptfoo) |
| **Language** | TypeScript/Node.js |
| **Installation** | `promptfoo` (wrapper in devshell) or `npx promptfoo@latest` |
| **Status** | ✅ **Integrated via wrapper** |
| **Relevance** | High - LLM testing & robot command parsing evaluation |

**Integration Note**: A `promptfoo` wrapper is included in the devshell that runs `npx promptfoo@latest` automatically. Node.js 22 is also available for direct npm usage.

### ComfyUI
| Attribute | Value |
|-----------|-------|
| **Repository** | [comfyanonymous/ComfyUI](https://github.com/comfyanonymous/ComfyUI) |
| **Nix Flake** | ✅ `github:utensils/comfyui-nix` |
| **Status** | ⚠️ Use dedicated Nix flake |
| **Relevance** | Medium - Vision workflows (isolate from main env) |

---

## Pixi Package Additions

### PyO3
| Attribute | Value |
|-----------|-------|
| **Repository** | [PyO3/pyo3](https://github.com/PyO3/pyo3) |
| **Pixi** | `pixi add maturin` (build tool) |
| **Status** | ✅ Ready |
| **Relevance** | **High** - Python-Rust ROS2 bindings |

### RustPython
| Attribute | Value |
|-----------|-------|
| **Repository** | [RustPython/RustPython](https://github.com/RustPython/RustPython) |
| **Status** | 🔬 Experimental |
| **Relevance** | Low - Embedded Python alternative |

---

## Conflict Analysis

### No Conflicts - Safe to Integrate

| Tool | Reason |
|------|--------|
| LocalAI | Nix package, Docker isolation available |
| Prometheus | Standard NixOS module |
| Netdata | Standard NixOS module |
| NATS Server | Standard Nix package, devenv support |
| Vault | ✅ Integrated - Go binary, BSL license (unfree) |
| Keycloak | NixOS module - Java 17+, PostgreSQL backend |
| Vaultwarden | NixOS module - Rust binary, SQLite/PostgreSQL |
| OPA | ✅ Integrated - Go binary, no conflicts |
| gVisor | ✅ Integrated - Linux only, Docker runtime |
| Trivy | ✅ Integrated - Go binary, container scanning |
| TruLens/Evals | Pure Python, Pixi managed |
| Memori/Memobase | Pure Python, Pixi managed |

### Potential Conflicts - Manage Carefully

| Tool | Conflict | Resolution |
|------|----------|------------|
| **Unsloth** | PyTorch version specificity | Use Docker container |
| **ComfyUI** | PyTorch/CUDA conflicts | Use dedicated Nix flake |
| **Promptfoo** | Node.js ecosystem | ✅ Resolved: wrapper in devshell |
| **SQLx** | Requires DB at compile time | ✅ CLI integrated; use naersk for builds |
| **PyO3/maturin** | Complex Nix + Python build | ✅ maturin integrated in devshell |
| **AGiXT** | Requires Docker | Docker Compose only |

### Python Version Requirements

| Tool | Python Version | ROS2 Humble Compatible |
|------|---------------|------------------------|
| AIOS | 3.10/3.11 only | ✅ Yes (ROS2 uses 3.11) |
| TruLens | 3.9+ | ✅ Yes |
| Memori | 3.10+ | ✅ Yes |
| ComfyUI | 3.12-3.13 | ⚠️ Separate env needed |

---

## Integration Recommendations

### Priority 1: Critical for Robotics (Immediate)

| Tool | Action | Installation |
|------|--------|--------------|
| **LocalAI** | Add to flake.nix | `pkgs.local-ai` |
| **Prometheus** | Enable NixOS module | `services.prometheus.enable = true` |
| **NATS Server** | Add for multi-site | `pkgs.nats-server` |
| **Keycloak** | Robot auth | `services.keycloak.enable = true` |

```nix
# flake.nix additions
commonPackages = with pkgs; [
  local-ai
  nats-server
  prometheus
];
```

### Priority 2: Infrastructure Enhancement (This Month)

| Tool | Action | Installation |
|------|--------|--------------|
| **Netdata** | Edge monitoring | `services.netdata.enable = true` |
| **Trippy** | Network diagnostics | `pkgs.trippy` |
| **TruLens** | LLM evaluation | `pixi add trulens` |
| **Memori** | Robot memory | `pixi add memori` |
| **OPA** | Policy enforcement | `pkgs.opa` |

### Priority 3: Developer Experience (Next Quarter)

| Tool | Action | Installation |
|------|--------|--------------|
| **AIOS/Cerebrum** | Agent orchestration | Custom Pixi env |
| **PyO3/maturin** | Rust-Python interop | `pixi add maturin` |
| **Vault** | Secrets management | `services.vault.enable = true` |
| **RustCoder MCP** | AI Rust dev | MCP server config |

### Priority 4: Optional/Experimental

| Tool | Use Case | Notes |
|------|----------|-------|
| **ComfyUI** | Vision workflows | Use dedicated Nix flake |
| **Unsloth** | Model fine-tuning | Docker only |
| **Temporal** | Complex workflows | Docker Compose |
| **gVisor** | Sandboxing | 5-15% overhead |

---

## Installation Quick Reference

### Nix Flake Additions
```nix
# In flake.nix perSystem block
devshells.default.devshell.packages = with pkgs; [
  # AI & Inference
  local-ai

  # Monitoring
  prometheus
  netdata
  trippy

  # Messaging
  nats-server

  # Security
  vault
  opa
  trivy

  # DevOps
  argocd
];
```

### Pixi Additions
```toml
# In pixi.toml
[dependencies]
python = ">=3.11,<3.12"

[pypi-dependencies]
trulens = "*"
memori = "*"
memobase = "*"
openai = "*"
aios-agent-sdk = ">=0.0.3"
```

### Docker Services
```yaml
# docker-compose.yml for services without Nix packages
services:
  temporal:
    image: temporalio/server:latest
    ports:
      - "7233:7233"

  unsloth:
    image: unsloth/unsloth:latest
    runtime: nvidia
```

---

## Sources

### AI Platforms
- [LocalAI Documentation](https://localai.io/)
- [AIOS Paper (COLM 2025)](https://github.com/agiresearch/AIOS)
- [AGiXT Documentation](https://agixt.com/)

### Infrastructure
- [Prometheus NixOS Wiki](https://wiki.nixos.org/wiki/Prometheus)
- [Netdata NixOS Wiki](https://wiki.nixos.org/wiki/Netdata)
- [Vulcanexus ROS2 Prometheus](https://docs.vulcanexus.org/)

### Security
- [Vault NixOS Module](https://github.com/NixOS/nixpkgs/blob/master/nixos/modules/services/security/vault.nix)
- [Keycloak NixOS Wiki](https://wiki.nixos.org/wiki/Keycloak)
- [OPA Documentation](https://www.openpolicyagent.org/)

### Rust Ecosystem
- [rust-libp2p](https://github.com/libp2p/rust-libp2p)
- [PyO3 User Guide](https://pyo3.rs/)
- [Building with SQLx on Nix](https://ipetkov.dev/blog/building-with-sqlx-on-nix/)

### ROS2 Integration
- [ros2_monitor_grafana](https://github.com/iwatake2222/ros2_monitor_grafana)
- [ROS-LLM Framework](https://github.com/Auromix/ROS-LLM)
- [NATS-ROS Connector](https://github.com/aljanabim/nats-ros-connector)

---

*Research completed: January 2026*
