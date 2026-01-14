# Documentation Index

Welcome to the ROS2 Humble Development Environment documentation. This index provides a roadmap to all available documentation.

**Last Updated:** 2026-01-14
**Documentation Quality Score:** 98% (200+ files indexed)

---

## Navigation Hub

### Quick Links

| Resource | Description |
|----------|-------------|
| [Getting Started](getting-started/GETTING_STARTED.md) | New user setup guide |
| [Onboarding Tutorial](getting-started/ONBOARDING_TUTORIAL.md) | Interactive step-by-step tutorial |
| [Troubleshooting](TROUBLESHOOTING.md) | Common issues and solutions |
| [Network Requirements](NETWORK_REQUIREMENTS.md) | Network dependencies and offline mode |
| [Contributing](../CONTRIBUTING.md) | How to contribute |
| [Project README](../README.md) | Project overview |

### Core Indexes (Start Here)

| Index | Purpose | Count |
|-------|---------|-------|
| [modules/INDEX.md](modules/INDEX.md) | Module documentation hub | 8 modules |
| [scripts/INDEX.md](scripts/INDEX.md) | Script contracts catalog | 60+ scripts |
| [cookbooks/INDEX.md](cookbooks/INDEX.md) | Operational recipes | 9 cookbooks |
| [graphs/](graphs/) | Mermaid diagrams | 9 graphs |
| [CATALOG.json](CATALOG.json) | Machine-readable catalog | All entrypoints |

---

## Module Documentation (One File Per Major Module)

| Module | Layer | Purpose | Link |
|--------|-------|---------|------|
| Bootstrap | L3 | Golden paths from zero to full stack | [modules/bootstrap.md](modules/bootstrap.md) |
| Environments | L2 | Environment taxonomy and overlays | [modules/environments.md](modules/environments.md) |
| Toolchain | L1 | Package management and pinning | [modules/toolchain.md](modules/toolchain.md) |
| CI/CD | L3 | Workflows, gates, and artifacts | [modules/ci_cd.md](modules/ci_cd.md) |
| Secrets | L2 | Credential management and PKI | [modules/secrets.md](modules/secrets.md) |
| Providers | L2 | Service integrations and auth | [modules/providers.md](modules/providers.md) |
| Observability | L5 | Metrics, logs, and traces | [modules/observability.md](modules/observability.md) |
| Smoke Tests | L5 | Verification and definition of done | [modules/smoke_tests.md](modules/smoke_tests.md) |

---

## Mermaid Graphs

| Graph | Purpose | Link |
|-------|---------|------|
| Bootstrap Flow | Fresh machine to full stack | [graphs/bootstrap_flow.mmd](graphs/bootstrap_flow.mmd) |
| CI Flow | CI/CD pipeline stages | [graphs/ci_flow.mmd](graphs/ci_flow.mmd) |
| Script DAG | Script dependencies | [graphs/script_dag.mmd](graphs/script_dag.mmd) |
| Environment Layering | Config precedence | [graphs/env_layering.mmd](graphs/env_layering.mmd) |
| Golden Paths | Critical paths | [graphs/golden-paths.mmd](graphs/golden-paths.mmd) |
| Auth Architecture | Identity flows | [graphs/auth-architecture.mmd](graphs/auth-architecture.mmd) |

---

## Master Reference Documents

| Document | Purpose | Status |
|----------|---------|--------|
| [CATALOG.json](CATALOG.json) | Machine-readable entrypoint catalog | Complete |
| [ARCHITECTURE.md](architecture/ARCHITECTURE.md) | Environment layers and architecture diagrams | Complete |
| [GLOSSARY.md](GLOSSARY.md) | Terminology reference | Complete |
| [PROVIDERS.md](providers/PROVIDERS.md) | Provider auth and configuration | Complete |
| [ENV_VAR_REGISTRY.md](ENV_VAR_REGISTRY.md) | Environment variable reference | Complete |
| [PORTS.md](api/PORTS.md) | Port mappings registry | Complete |
| [RUNBOOKS.md](cookbooks/RUNBOOKS.md) | Operational procedures and cookbooks | Complete |
| [UNKNOWN_REPORT.md](audits/UNKNOWN_REPORT.md) | Documentation gaps and unknowns | Phase 7 & 8 Complete |
| [QUALITY.md](QUALITY.md) | Documentation quality standards | Phase 8 Complete |
| [COMPATIBILITY_MATRIX.md](COMPATIBILITY_MATRIX.md) | Version compatibility reference | Phase 8 Complete |
| [PRODUCTION_SCALE.md](PRODUCTION_SCALE.md) | Production resource requirements | Phase 8 Complete |
| [ON_CALL.md](ON_CALL.md) | Escalation procedures and contacts template | Template (UN-003 Resolved) |
| [scripts/INDEX.md](scripts/INDEX.md) | Script catalog and contracts | Complete |
| [modules/bootstrap.md](modules/bootstrap.md) | Golden path documentation | Complete |
| [graphs/bootstrap_flow.mmd](graphs/bootstrap_flow.mmd) | Bootstrap flow diagram | Complete |
| [graphs/script_dag.mmd](graphs/script_dag.mmd) | Script dependency graph | Complete (SU-001 Verified) |
| [graphs/ci_flow.mmd](graphs/ci_flow.mmd) | CI/CD flow diagram | Complete |

---

## Documentation Categories

### Getting Started & Onboarding

New to the project? Start here:

| Document | Description | Time |
|----------|-------------|------|
| [GETTING_STARTED.md](getting-started/GETTING_STARTED.md) | Quick setup reference | 5-10 min |
| [ONBOARDING_TUTORIAL.md](getting-started/ONBOARDING_TUTORIAL.md) | Interactive tutorial with checkpoints | 30-45 min |
| [PROGRESSIVE_EXAMPLES.md](getting-started/PROGRESSIVE_EXAMPLES.md) | Examples from beginner to advanced | Varies |
| [VIDEO_WALKTHROUGHS.md](getting-started/VIDEO_WALKTHROUGHS.md) | Video tutorials and external resources | Varies |
| [COMMON_PITFALLS.md](COMMON_PITFALLS.md) | Common mistakes and how to avoid them | Reference |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Solutions to common problems | Reference |

**Recommended learning path:**
1. [Getting Started](getting-started/GETTING_STARTED.md) - Get the environment running
2. [Onboarding Tutorial](getting-started/ONBOARDING_TUTORIAL.md) - Learn the basics interactively
3. [Progressive Examples](getting-started/PROGRESSIVE_EXAMPLES.md) - Build your skills
4. [Common Pitfalls](COMMON_PITFALLS.md) - Learn what to avoid

### Core Infrastructure

| Document | Description |
|----------|-------------|
| [INFERENCE_SETUP.md](getting-started/INFERENCE_SETUP.md) | LocalAI and LLM inference configuration |
| [OBSERVABILITY-QUICK-START.md](getting-started/quick-start/OBSERVABILITY-QUICK-START.md) | Prometheus, Grafana, and monitoring setup |
| [EDGE_DEPLOYMENT.md](edge-service/gateway/EDGE_DEPLOYMENT.md) | Edge device deployment guide |
| [EDGE_QUICKSTART.md](getting-started/quick-start/EDGE_QUICKSTART.md) | Quick start for edge deployments |
| [API_REFERENCE.md](api/API_REFERENCE.md) | Service API documentation |
| [OPA_POLICIES.md](OPA_POLICIES.md) | OPA authorization policy documentation |
| [HELM_CHART.md](HELM_CHART.md) | Helm chart configuration and dependencies |
| [ARTIFACTS.md](ARTIFACTS.md) | Build artifacts and publication workflows |
| [PROVIDER_INTEGRATIONS.md](providers/PROVIDER_INTEGRATIONS.md) | Holochain, MindsDB, TensorZero, vCache integration |
| [MIGRATION_GUIDES.md](MIGRATION_GUIDES.md) | Version upgrade and migration procedures |

### Security & Identity

| Document | Description |
|----------|-------------|
| [CONTAINER_SECURITY.md](security/CONTAINER_SECURITY.md) | Container security, network segmentation, and port exposure |
| [SECRETS.md](secrets/SECRETS.md) | Secrets management with agenix and detect-secrets |
| [MTLS_SETUP.md](getting-started/MTLS_SETUP.md) | Mutual TLS configuration |
| [MTLS-IMPLEMENTATION-CHECKLIST.md](getting-started/MTLS-IMPLEMENTATION-CHECKLIST.md) | mTLS implementation checklist |
| [SUPPLY_CHAIN_SECURITY.md](security/SUPPLY_CHAIN_SECURITY.md) | Supply chain security and SBOM |

### Database & Storage

| Document | Description |
|----------|-------------|
| [BYTEBASE-SETUP.md](getting-started/BYTEBASE-SETUP.md) | Database schema management |
| [VCACHE-SETUP.md](VCACHE-SETUP.md) | Vector cache configuration |
| [neo4j-quick-reference.md](reference/neo4j-quick-reference.md) | Neo4j graph database reference |
| [neo4j-verification.md](databases/neo4j-verification.md) | Neo4j verification procedures |

### AI & Machine Learning

| Document | Description |
|----------|-------------|
| [LOCALAI-MODELS.md](models/LOCALAI-MODELS.md) | LocalAI model configuration |
| [MINDSDB_QUICKSTART.md](getting-started/quick-start/MINDSDB_QUICKSTART.md) | MindsDB ML platform setup |
| [GENAI_TOOLBOX_INSTALL.md](getting-started/GENAI_TOOLBOX_INSTALL.md) | GenAI toolbox installation |

### Cookbooks

Task-oriented operational recipes (Phase 7 Complete):

| Document | Description |
|----------|-------------|
| [cookbooks/INDEX.md](cookbooks/INDEX.md) | Cookbook index - all recipes |
| [cookbooks/SECRET_ROTATION.md](cookbooks/SECRET_ROTATION.md) | Monthly credential rotation and emergency procedures |
| [cookbooks/DEPENDENCY_UPDATES.md](cookbooks/DEPENDENCY_UPDATES.md) | Weekly/monthly Nix, Pixi, PyTorch updates |
| [cookbooks/PROVIDER_ONBOARDING.md](cookbooks/PROVIDER_ONBOARDING.md) | Step-by-step new service integration |
| [cookbooks/VERSION_PINNING_POLICY.md](cookbooks/VERSION_PINNING_POLICY.md) | When to pin exact versions vs ranges |
| [cookbooks/EMERGENCY_PROCEDURES.md](cookbooks/EMERGENCY_PROCEDURES.md) | Incident response for outages and failures |
| [cookbooks/BACKUP_RESTORE.md](cookbooks/BACKUP_RESTORE.md) | Disaster recovery and backup procedures |
| [cookbooks/PERFORMANCE_TUNING.md](cookbooks/PERFORMANCE_TUNING.md) | WSL, Nix, Docker optimization |
| [cookbooks/MAJOR_VERSION_UPGRADE.md](cookbooks/MAJOR_VERSION_UPGRADE.md) | Python, ROS2, Nix major upgrades |
| [cookbooks/LOCALAI-MODELS-CACHE.md](cookbooks/LOCALAI-MODELS-CACHE.md) | Fetch + cache LocalAI models (Git LFS) |

### Distributed Systems

| Document | Description |
|----------|-------------|
| [HOLOCHAIN-DEPLOYMENT.md](HOLOCHAIN-DEPLOYMENT.md) | Holochain DHT deployment |
| [HOLOCHAIN-ZOMES.md](HOLOCHAIN-ZOMES.md) | Holochain Zome development |
| [DISTRIBUTED-TRACING.md](DISTRIBUTED-TRACING.md) | Distributed tracing setup |

### Development Tools

| Document | Description |
|----------|-------------|
| [PYTHON-ENVIRONMENTS.md](PYTHON-ENVIRONMENTS.md) | Python environment management |
| [CONFLICTS.md](CONFLICTS.md) | Version coupling and dependency conflicts |
| [NODEJS-AGENTS.md](ai-ml/NODEJS-AGENTS.md) | Node.js agent development |
| [TOOLING-ANALYSIS.md](TOOLING-ANALYSIS.md) | Development tooling analysis |

#### Python Dependency Utilities

| Script | Description |
|--------|-------------|
| [`scripts/check-python-deps.sh`](../scripts/check-python-deps.sh) | Check for Python dependency conflicts |
| [`scripts/upgrade-python-deps.sh`](../scripts/upgrade-python-deps.sh) | Python version upgrade automation |

### Platform-Specific

| Document | Description |
|----------|-------------|
| [WSL2_BUILD_PIPELINE.md](wsl/WSL2_BUILD_PIPELINE.md) | Windows WSL2 build pipeline |
| [TROUBLESHOOTING.md#wsl2-disk-space-management](TROUBLESHOOTING.md#wsl2-disk-space-management) | WSL2 disk space management |
| [KATA_CONTAINERS_INSTALL.md](containers/KATA_CONTAINERS_INSTALL.md) | Kata containers installation |
| [SANDBOX_RUNTIME_INSTALL.md](getting-started/SANDBOX_RUNTIME_INSTALL.md) | Sandbox runtime setup |

#### WSL2 Disk Utilities

| Script | Description |
|--------|-------------|
| [`scripts/wsl-cleanup.sh`](../scripts/wsl-cleanup.sh) | Cleanup utility for inside WSL |
| [`scripts/Cleanup-WSL.ps1`](../scripts/Cleanup-WSL.ps1) | Windows PowerShell cleanup utility |

### ROS2 Specific

| Document | Description |
|----------|-------------|
| [ROS2_STATE_PACKAGES.md](robotics/ROS2_STATE_PACKAGES.md) | ROS2 state management packages |
| [ROS2_DEPENDENCIES.md](dependencies/ROS2_DEPENDENCIES.md) | ROS2 package dependencies via RoboStack |

### CI/CD & DevOps

| Document | Description |
|----------|-------------|
| [BRANCH-PROTECTION.md](rules/BRANCH-PROTECTION.md) | Git branch protection rules |
| [GITHUB-RESOURCES.md](GITHUB-RESOURCES.md) | GitHub resource documentation |

### Nix & Flake Architecture

| Document | Description |
|----------|-------------|
| [NIX_FLAKE_MODULARIZATION.md](nix/NIX_FLAKE_MODULARIZATION.md) | Flake modularization plan and image generation |
| [NIX_EVALUATION_OPTIMIZATION.md](nix/NIX_EVALUATION_OPTIMIZATION.md) | Nix evaluation performance optimization |

### Network & Offline

| Document | Description |
|----------|-------------|
| [NETWORK_REQUIREMENTS.md](NETWORK_REQUIREMENTS.md) | Network dependencies and firewall rules |
| [SUPPLY_CHAIN_SECURITY.md](security/SUPPLY_CHAIN_SECURITY.md) | Supply chain security measures |
| [AI_RESOURCE_REQUIREMENTS.md](ai-ml/AI_RESOURCE_REQUIREMENTS.md) | AI service resource requirements |

### Architecture & Planning

| Document | Description |
|----------|-------------|
| [TASK_GRAPH_EXECUTION_PLAN.md](audits/TASK_GRAPH_EXECUTION_PLAN.md) | Task execution planning |
| [MANUS_ARIA_ORCHESTRATOR.md](audits/MANUS_ARIA_ORCHESTRATOR.md) | ARIA orchestrator architecture |
| [QUANTUM_AGENTICS_RD.md](ai-ml/QUANTUM_AGENTICS_RD.md) | Quantum agentics R&D notes |
| [CONFLICTS.md](CONFLICTS.md) | Dependency conflict resolution |

### Implementation Guides

| Document | Description |
|----------|-------------|
| [P1-008-009-IMPLEMENTATION.md](observability/umami-netdata.md) | Priority 1 implementation guide |
| [P2-001-SANDBOX-INTEGRATION.md](ai-ml/SANDBOX-INTEGRATION.md) | Sandbox integration guide |
| [P2-012-OPEN-LOVABLE-INTEGRATION.md](frontend/tools/OPEN-LOVABLE-INTEGRATION.md) | Open Lovable integration |
| [P3-003-NEO4J-IMPLEMENTATION.md](databases/NEO4J-IMPLEMENTATION.md) | Neo4j implementation guide |
| [QUICK-START-OPEN-LOVABLE.md](getting-started/quick-start/QUICK-START-OPEN-LOVABLE.md) | Open Lovable quick start |

### Other

| Document | Description |
|----------|-------------|
| [PHASE3-LAZYVIM-DEPS.md](frontend/LAZYVIM-DEPS.md) | LazyVim dependencies |

---

## External Resources

### ROS2
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS2 Design](https://design.ros2.org/)

### Nix
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Nix Flakes](https://nixos.wiki/wiki/Flakes)
- [Zero to Nix](https://zero-to-nix.com/)

### Pixi
- [Pixi Documentation](https://pixi.sh/)
- [conda-forge](https://conda-forge.org/)

---

## Contributing to Documentation

When adding new documentation:

1. Use the appropriate category above
2. Follow naming conventions (see [CONTRIBUTING.md](../CONTRIBUTING.md#naming-conventions))
3. Add entry to this index
4. Include cross-references to related docs
5. Keep language clear and examples runnable

## Documentation Standards

- Use Markdown with GitHub-flavored extensions
- Include a table of contents for long documents
- Provide code examples that work out of the box
- Link to external resources when appropriate
- Keep documentation up to date with code changes
