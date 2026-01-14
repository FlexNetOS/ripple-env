# Glossary

**Status:** Complete
**Last Updated:** 2026-01-14

This glossary defines terminology used throughout the ripple-env documentation.

---

## A

### AGiXT
**AI agent orchestration platform.** Provides multi-agent coordination, tool management, and LLM integration. Runs as a Docker service on port 7437.

**Evidence:** `docker/docker-compose.agixt.yml`

### AIOS (Agent Operating System)
**Pixi environment for AI agents.** Includes LiteLLM, ChromaDB, and LangChain for building and running AI agents.

**Evidence:** `pixi.toml` feature.aios section

### ArgoCD
**GitOps continuous delivery tool for Kubernetes.** Syncs Kubernetes state with Git repository definitions.

**Evidence:** `manifests/argocd/`, `scripts/install-argocd.sh`

### ARIA Manifest
**Single source of truth for component verification.** YAML file (`ARIA_MANIFEST.yaml`) that defines all components, their verification commands, and health checks.

**Evidence:** `ARIA_MANIFEST.yaml` (1200+ lines)

---

## B

### Bootstrap Script
**Automated setup script for fresh machines.** `bootstrap.sh` (Linux/macOS) or `bootstrap.ps1` (Windows) that installs all dependencies and configures the environment.

**Evidence:** `bootstrap.sh`, `bootstrap.ps1`

---

## C

### Colcon
**Build tool for ROS2 packages.** Handles building, testing, and installing ROS2 workspaces.

**Aliases:**
- `cb` = `colcon build --symlink-install`
- `ct` = `colcon test`
- `ctr` = `colcon test-result --verbose`

**Evidence:** `flake.nix` coreCommandWrappers

### Conda-Forge
**Community-driven conda package channel.** Primary source for Python/scientific packages in Pixi.

**Evidence:** `pixi.toml` channels section

---

## D

### DevShell
**Nix development environment.** Provides a reproducible shell with specific packages installed. Entered via `nix develop` or `nom develop`.

**Variants:**
- `default` — Base tools
- `full` — All tools
- `cuda` — GPU support
- `identity` — Auth tools

**Evidence:** `flake.nix` devShells section

### direnv
**Automatic environment activation.** Loads `.envrc` when entering directories, enabling seamless devshell activation.

**Evidence:** `.envrc`, `bootstrap.sh` lines 669-695

### Docker Compose Stack
**Multi-container application definition.** YAML files defining services, networks, and volumes for deployment.

**Available Stacks:** 22 (observability, messaging, identity, ai, data, edge, automation, ui, etc.)

**Evidence:** `docker/docker-compose.*.yml`

---

## E

### E2E Validation
**End-to-end validation script.** `scripts/validate-e2e.sh` runs comprehensive tests across all components.

**Evidence:** `scripts/validate-e2e.sh`

---

## F

### Flake
**Nix package definition with locked dependencies.** `flake.nix` defines packages, shells, and modules; `flake.lock` pins exact versions.

**Evidence:** `flake.nix`, `flake.lock`

### FlakeHub
**Nix flake registry.** Used for publishing and discovering Nix flakes.

**Evidence:** `.github/workflows/flakehub-publish-tagged.yml`

### FlexStack
**Helm chart for Kubernetes deployment.** Bundles all ripple-env services for K8s.

**Evidence:** `charts/flexstack/`

---

## G

### Golden Path
**Recommended workflow for common tasks.** Documented sequences of commands that achieve specific goals reliably.

**Defined Paths:**
1. GP-1: Fresh Machine → Full Stack
2. GP-2: Update Toolchain
3. GP-3: Build & Verify
4. GP-4: Deploy & Smoke Test

**Evidence:** `docs/modules/bootstrap.md`, `docs/graphs/bootstrap_flow.mmd`

### Grafana
**Visualization and dashboarding platform.** Displays metrics from Prometheus, logs from Loki, traces from Tempo.

**Port:** 3000

**Evidence:** `docker/docker-compose.observability.yml`

---

## H

### Home-Manager
**Nix-based user configuration management.** Manages dotfiles, user packages, and settings declaratively.

**Evidence:** `modules/home-manager/`, `flake.nix` homeManagerModules

### Holochain
**Distributed application framework.** P2P coordination for agent-centric applications.

**Evidence:** `docker/docker-compose.holochain.yml`, `flake.nix` holochainPackages

---

## I

### Idempotent
**Safe to run multiple times.** A script or command that produces the same result regardless of how many times it's executed.

**Example:** `bootstrap.sh` stages are idempotent — they check state before acting.

---

## J

### JetStream
**NATS persistent messaging layer.** Provides durable message storage, replay, and consumer groups.

**Evidence:** `scripts/init-jetstream.sh`, `scripts/verify-jetstream.sh`

---

## K

### Keycloak
**Identity and access management platform.** Provides OAuth2/OIDC authentication and authorization.

**Ports:** 8080 (HTTP), 8443 (HTTPS)

**Evidence:** `docker/docker-compose.identity.yml`

### Kong
**API gateway and service mesh.** Routes traffic, enforces authentication, rate limiting, and observability.

**Ports:** 8000 (proxy), 8001 (admin)

**Evidence:** `docker/docker-compose.edge.yml`

### Kubernetes
**Container orchestration platform.** Optional production deployment target via ArgoCD.

**Evidence:** `manifests/`, `k8s/`

---

## L

### LocalAI
**Local LLM inference server.** OpenAI-compatible API for running models locally.

**Port:** 8080

**Evidence:** `docker/docker-compose.localai.yml`

### Loki
**Log aggregation system.** Collects and indexes logs for querying in Grafana.

**Port:** 3100

**Evidence:** `docker/docker-compose.observability.yml`

---

## M

### MindsDB
**AI-powered database query engine.** SQL interface for ML models and predictions.

**Ports:** 47334 (MySQL), 47335 (HTTP), 47336 (MongoDB)

**Evidence:** `docker/docker-compose.data.yml`

### MinIO
**S3-compatible object storage.** Used for artifacts, models, and backups.

**Ports:** 9000 (API), 9001 (Console)

**Evidence:** `docker/docker-compose.state.yml`

### mTLS (Mutual TLS)
**Two-way certificate authentication.** Both client and server verify each other's certificates.

**Evidence:** `docs/MTLS_SETUP.md`, `scripts/generate-service-certs.sh`

---

## N

### NATS
**High-performance messaging system.** Pub/sub and request/reply patterns for service communication.

**Ports:** 4222 (clients), 8222 (monitoring)

**Evidence:** `docker/docker-compose.messaging.yml`

### Nix
**Reproducible package manager.** Builds packages in isolation with exact dependency tracking.

**Evidence:** `flake.nix`, `flake.lock`

### NixOS
**Linux distribution built on Nix.** Entire system configured declaratively.

**Evidence:** `nix/images/`, `docs/NIXOS_IMAGES.md`

### NixOS-WSL
**NixOS for Windows Subsystem for Linux.** Custom NixOS distribution for WSL2.

**Evidence:** `bootstrap.ps1` lines 354-417

### nom (Nix Output Monitor)
**Pretty output for Nix builds.** Replaces `nix build` with progress bars and status.

**Evidence:** `bootstrap.sh` lines 698-724

---

## O

### OPA (Open Policy Agent)
**Policy-as-code engine.** Evaluates authorization decisions using Rego policies.

**Port:** 8181

**Evidence:** `docker/docker-compose.automation.yml`, `config/opa/policies/`

### Open-Lovable
**AI-powered UI generation tool.** Web-based interface for building applications with AI.

**Port:** 3211

**Evidence:** `docker/docker-compose.ui.yml`

---

## P

### Pixi
**Fast conda package manager.** Multi-platform package management with lock files.

**Evidence:** `pixi.toml`, `pixi.lock`

### Prometheus
**Time-series metrics database.** Scrapes and stores metrics for monitoring.

**Port:** 9090

**Evidence:** `docker/docker-compose.observability.yml`

---

## Q

### QuDAG
**Quantum-resistant cryptography environment.** Pixi environment with quantum-safe algorithms.

**Evidence:** `pixi.toml` feature.qudag, `docker-compose.qudag.yml`

---

## R

### RoboStack
**ROS packages for conda.** Conda channel providing ROS2 packages.

**Channel:** `robostack-humble`

**Evidence:** `pixi.toml` channels section

### ROS2 Humble
**Robot Operating System version 2 (Humble Hawksbill).** Primary robotics framework.

**Evidence:** `pixi.toml` dependencies

### RuVector
**Rust-based vector database.** High-performance similarity search.

**Evidence:** `docker-compose.ruvector.yml`, `scripts/ruvector.sh`

---

## S

### SBOM (Software Bill of Materials)
**List of software components.** Generated by Syft for supply chain security.

**Evidence:** `.github/workflows/sbom.yml`

### Step-CA
**Smallstep Certificate Authority.** Issues and manages X.509 certificates.

**Port:** 9000

**Evidence:** `docker/docker-compose.identity.yml`, `scripts/init-step-ca.sh`

---

## T

### Temporal
**Durable workflow execution engine.** Runs long-running, fault-tolerant workflows.

**Port:** 7233

**Evidence:** `docker/docker-compose.temporal.yml`

### TensorZero
**LLM inference gateway.** Routes requests across LLM providers.

**Port:** 3030

**Evidence:** `docker/docker-compose.llmops.yml`

### Trivy
**Security vulnerability scanner.** Scans containers, filesystems, and dependencies.

**Evidence:** `.github/workflows/security.yml`, `scripts/security-audit.sh`

---

## V

### Vault
**HashiCorp secrets management.** Stores and retrieves secrets securely.

**Port:** 8200

**Evidence:** `docker/docker-compose.identity.yml`

### Vaultwarden
**Bitwarden-compatible password manager.** Self-hosted credential storage.

**Port:** 8090

**Evidence:** `docker/docker-compose.identity.yml`

### vCache
**LLM prompt caching layer.** Reduces inference costs by caching responses.

**Evidence:** `docker/docker-compose.caching.yml`

---

## W

### WSL2 (Windows Subsystem for Linux 2)
**Linux compatibility layer for Windows.** Runs NixOS-WSL for development.

**Evidence:** `bootstrap.ps1`, `.wslconfig`

---

## Abbreviations

| Abbreviation | Meaning |
|--------------|---------|
| API | Application Programming Interface |
| CA | Certificate Authority |
| CI/CD | Continuous Integration / Continuous Delivery |
| CLI | Command Line Interface |
| DAG | Directed Acyclic Graph |
| GPU | Graphics Processing Unit |
| HTTPS | HTTP Secure |
| K8s | Kubernetes |
| LLM | Large Language Model |
| ML | Machine Learning |
| mTLS | Mutual Transport Layer Security |
| OIDC | OpenID Connect |
| RBAC | Role-Based Access Control |
| ROS2 | Robot Operating System 2 |
| TLS | Transport Layer Security |
| UI | User Interface |
| VM | Virtual Machine |
| WSL | Windows Subsystem for Linux |
| YAML | YAML Ain't Markup Language |
