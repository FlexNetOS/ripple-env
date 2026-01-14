# Documentation Index

Welcome to the ROS2 Humble Development Environment documentation. This index provides a roadmap to all available documentation.

## Quick Links

| Resource | Description |
|----------|-------------|
| [Getting Started](GETTING_STARTED.md) | New user setup guide |
| [Onboarding Tutorial](ONBOARDING_TUTORIAL.md) | Interactive step-by-step tutorial |
| [Troubleshooting](TROUBLESHOOTING.md) | Common issues and solutions |
| [Network Requirements](NETWORK_REQUIREMENTS.md) | Network dependencies and offline mode |
| [Contributing](../CONTRIBUTING.md) | How to contribute |
| [Project README](../README.md) | Project overview |

## Master Reference Documents

| Document | Purpose | Status |
|----------|---------|--------|
| [CATALOG.json](CATALOG.json) | Machine-readable entrypoint catalog | Complete |
| [ARCHITECTURE.md](ARCHITECTURE.md) | Environment layers and architecture diagrams | Complete |
| [GLOSSARY.md](GLOSSARY.md) | Terminology reference | Complete |
| [PROVIDERS.md](PROVIDERS.md) | Provider auth and configuration | Complete |
| [ENV_VAR_REGISTRY.md](ENV_VAR_REGISTRY.md) | Environment variable reference | Complete |
| [PORTS.md](PORTS.md) | Port mappings registry | Complete |
| [RUNBOOKS.md](RUNBOOKS.md) | Operational procedures and cookbooks | Complete |
| [UNKNOWN_REPORT.md](UNKNOWN_REPORT.md) | Documentation gaps and unknowns | Complete |
| [scripts/INDEX.md](scripts/INDEX.md) | Script catalog and contracts | Complete |
| [modules/bootstrap.md](modules/bootstrap.md) | Golden path documentation | Complete |
| [graphs/bootstrap_flow.mmd](graphs/bootstrap_flow.mmd) | Bootstrap flow diagram | Complete |
| [graphs/script_dag.mmd](graphs/script_dag.mmd) | Script dependency graph | Draft |
| [graphs/ci_flow.mmd](graphs/ci_flow.mmd) | CI/CD flow diagram | Complete |

---

## Documentation Categories

### Getting Started & Onboarding

New to the project? Start here:

| Document | Description | Time |
|----------|-------------|------|
| [GETTING_STARTED.md](GETTING_STARTED.md) | Quick setup reference | 5-10 min |
| [ONBOARDING_TUTORIAL.md](ONBOARDING_TUTORIAL.md) | Interactive tutorial with checkpoints | 30-45 min |
| [PROGRESSIVE_EXAMPLES.md](PROGRESSIVE_EXAMPLES.md) | Examples from beginner to advanced | Varies |
| [VIDEO_WALKTHROUGHS.md](VIDEO_WALKTHROUGHS.md) | Video tutorials and external resources | Varies |
| [COMMON_PITFALLS.md](COMMON_PITFALLS.md) | Common mistakes and how to avoid them | Reference |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Solutions to common problems | Reference |

**Recommended learning path:**
1. [Getting Started](GETTING_STARTED.md) - Get the environment running
2. [Onboarding Tutorial](ONBOARDING_TUTORIAL.md) - Learn the basics interactively
3. [Progressive Examples](PROGRESSIVE_EXAMPLES.md) - Build your skills
4. [Common Pitfalls](COMMON_PITFALLS.md) - Learn what to avoid

### Core Infrastructure

| Document | Description |
|----------|-------------|
| [INFERENCE_SETUP.md](INFERENCE_SETUP.md) | LocalAI and LLM inference configuration |
| [OBSERVABILITY-QUICK-START.md](OBSERVABILITY-QUICK-START.md) | Prometheus, Grafana, and monitoring setup |
| [EDGE_DEPLOYMENT.md](EDGE_DEPLOYMENT.md) | Edge device deployment guide |
| [EDGE_QUICKSTART.md](EDGE_QUICKSTART.md) | Quick start for edge deployments |
| [API_REFERENCE.md](API_REFERENCE.md) | Service API documentation |
| [OPA_POLICIES.md](OPA_POLICIES.md) | OPA authorization policy documentation |
| [HELM_CHART.md](HELM_CHART.md) | Helm chart configuration and dependencies |
| [ARTIFACTS.md](ARTIFACTS.md) | Build artifacts and publication workflows |
| [PROVIDER_INTEGRATIONS.md](PROVIDER_INTEGRATIONS.md) | Holochain, MindsDB, TensorZero, vCache integration |
| [MIGRATION_GUIDES.md](MIGRATION_GUIDES.md) | Version upgrade and migration procedures |

### Security & Identity

| Document | Description |
|----------|-------------|
| [CONTAINER_SECURITY.md](CONTAINER_SECURITY.md) | Container security, network segmentation, and port exposure |
| [SECRETS.md](SECRETS.md) | Secrets management with agenix and detect-secrets |
| [MTLS_SETUP.md](MTLS_SETUP.md) | Mutual TLS configuration |
| [MTLS-IMPLEMENTATION-CHECKLIST.md](MTLS-IMPLEMENTATION-CHECKLIST.md) | mTLS implementation checklist |
| [SUPPLY_CHAIN_SECURITY.md](SUPPLY_CHAIN_SECURITY.md) | Supply chain security and SBOM |

### Database & Storage

| Document | Description |
|----------|-------------|
| [BYTEBASE-SETUP.md](BYTEBASE-SETUP.md) | Database schema management |
| [VCACHE-SETUP.md](VCACHE-SETUP.md) | Vector cache configuration |
| [neo4j-quick-reference.md](neo4j-quick-reference.md) | Neo4j graph database reference |
| [neo4j-verification.md](neo4j-verification.md) | Neo4j verification procedures |

### AI & Machine Learning

| Document | Description |
|----------|-------------|
| [LOCALAI-MODELS.md](LOCALAI-MODELS.md) | LocalAI model configuration |
| [MINDSDB_QUICKSTART.md](MINDSDB_QUICKSTART.md) | MindsDB ML platform setup |
| [GENAI_TOOLBOX_INSTALL.md](GENAI_TOOLBOX_INSTALL.md) | GenAI toolbox installation |

### Cookbooks

| Document | Description |
|----------|-------------|
| [cookbooks/INDEX.md](cookbooks/INDEX.md) | Task-oriented recipes |
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
| [NODEJS-AGENTS.md](NODEJS-AGENTS.md) | Node.js agent development |
| [TOOLING-ANALYSIS.md](TOOLING-ANALYSIS.md) | Development tooling analysis |

#### Python Dependency Utilities

| Script | Description |
|--------|-------------|
| [`scripts/check-python-deps.sh`](../scripts/check-python-deps.sh) | Check for Python dependency conflicts |
| [`scripts/upgrade-python-deps.sh`](../scripts/upgrade-python-deps.sh) | Python version upgrade automation |

### Platform-Specific

| Document | Description |
|----------|-------------|
| [WSL2_BUILD_PIPELINE.md](WSL2_BUILD_PIPELINE.md) | Windows WSL2 build pipeline |
| [TROUBLESHOOTING.md#wsl2-disk-space-management](TROUBLESHOOTING.md#wsl2-disk-space-management) | WSL2 disk space management |
| [KATA_CONTAINERS_INSTALL.md](KATA_CONTAINERS_INSTALL.md) | Kata containers installation |
| [SANDBOX_RUNTIME_INSTALL.md](SANDBOX_RUNTIME_INSTALL.md) | Sandbox runtime setup |

#### WSL2 Disk Utilities

| Script | Description |
|--------|-------------|
| [`scripts/wsl-cleanup.sh`](../scripts/wsl-cleanup.sh) | Cleanup utility for inside WSL |
| [`scripts/Cleanup-WSL.ps1`](../scripts/Cleanup-WSL.ps1) | Windows PowerShell cleanup utility |

### ROS2 Specific

| Document | Description |
|----------|-------------|
| [ROS2_STATE_PACKAGES.md](ROS2_STATE_PACKAGES.md) | ROS2 state management packages |
| [ROS2_DEPENDENCIES.md](ROS2_DEPENDENCIES.md) | ROS2 package dependencies via RoboStack |

### CI/CD & DevOps

| Document | Description |
|----------|-------------|
| [BRANCH-PROTECTION.md](BRANCH-PROTECTION.md) | Git branch protection rules |
| [GITHUB-RESOURCES.md](GITHUB-RESOURCES.md) | GitHub resource documentation |

### Nix & Flake Architecture

| Document | Description |
|----------|-------------|
| [NIX_FLAKE_MODULARIZATION.md](NIX_FLAKE_MODULARIZATION.md) | Flake modularization plan and image generation |
| [NIX_EVALUATION_OPTIMIZATION.md](NIX_EVALUATION_OPTIMIZATION.md) | Nix evaluation performance optimization |

### Network & Offline

| Document | Description |
|----------|-------------|
| [NETWORK_REQUIREMENTS.md](NETWORK_REQUIREMENTS.md) | Network dependencies and firewall rules |
| [SUPPLY_CHAIN_SECURITY.md](SUPPLY_CHAIN_SECURITY.md) | Supply chain security measures |
| [AI_RESOURCE_REQUIREMENTS.md](AI_RESOURCE_REQUIREMENTS.md) | AI service resource requirements |

### Architecture & Planning

| Document | Description |
|----------|-------------|
| [TASK_GRAPH_EXECUTION_PLAN.md](TASK_GRAPH_EXECUTION_PLAN.md) | Task execution planning |
| [MANUS_ARIA_ORCHESTRATOR.md](MANUS_ARIA_ORCHESTRATOR.md) | ARIA orchestrator architecture |
| [QUANTUM_AGENTICS_RD.md](QUANTUM_AGENTICS_RD.md) | Quantum agentics R&D notes |
| [CONFLICTS.md](CONFLICTS.md) | Dependency conflict resolution |

### Implementation Guides

| Document | Description |
|----------|-------------|
| [P1-008-009-IMPLEMENTATION.md](P1-008-009-IMPLEMENTATION.md) | Priority 1 implementation guide |
| [P2-001-SANDBOX-INTEGRATION.md](P2-001-SANDBOX-INTEGRATION.md) | Sandbox integration guide |
| [P2-012-OPEN-LOVABLE-INTEGRATION.md](P2-012-OPEN-LOVABLE-INTEGRATION.md) | Open Lovable integration |
| [P3-003-NEO4J-IMPLEMENTATION.md](P3-003-NEO4J-IMPLEMENTATION.md) | Neo4j implementation guide |
| [QUICK-START-OPEN-LOVABLE.md](QUICK-START-OPEN-LOVABLE.md) | Open Lovable quick start |

### Other

| Document | Description |
|----------|-------------|
| [PHASE3-LAZYVIM-DEPS.md](PHASE3-LAZYVIM-DEPS.md) | LazyVim dependencies |

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
