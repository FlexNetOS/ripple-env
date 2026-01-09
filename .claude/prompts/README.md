# Prompts Directory

This directory contains reusable prompt templates designed for Claude Opus 4.5 and other advanced models.

## Available Prompts

| Prompt | Version | Model | Description |
|--------|---------|-------|-------------|
| `aria-orchestrator.md` | 2.0.0 | Opus 4.5 | ARIA: Agentic Research & Integration Architect |

## ARIA Orchestrator v2.0

ARIA is a comprehensive codebase audit and orchestration system that:

1. **Discovers** ALL files, subjects, and reference links from README.md and docs/BUILDKIT_STARTER_SPEC.md
2. **Deploys** 14 parallel research teams (1 per domain)
3. **Maps** every repository to installation method (Nix/Pixi/Docker/Cargo/NPM)
4. **Resolves** conflicts with A/B feature flags (never removes features)
5. **Generates** prioritized task backlogs with verification commands

### Model Assignment

| Role | Model | Purpose |
|------|-------|---------|
| **Orchestrator (ARIA)** | `opus` | Complex reasoning, synthesis, conflict resolution |
| **Lead Agents** | `sonnet` | Domain expertise, deep analysis |
| **Specialized Agents** | `sonnet` | Focused research, configuration validation |
| **Sub-Agents** | `haiku` | Fast validation, link checking, counting |

### Domain Teams (14 Total)

| Domain | Lead Focus | Specialists |
|--------|------------|-------------|
| Host OS | Nix/NixOS | flake.nix, modules/, pixi.toml |
| Isolation | Containers | Kata, Firecracker, sandbox-runtime |
| Cluster | Kubernetes | Argo CD, Rollouts, Workflows |
| Edge | Gateways | Kong, AgentGateway, MCP routing |
| Identity | Auth/Policy | Keycloak, OPA, Vault |
| Messaging | Event Bus | NATS, Temporal, n8n |
| Agent Runtime | Agent OS | AIOS, AGiXT, claude-flow |
| Tool Execution | MCP Tools | genai-toolbox, midstream, solvers |
| Inference | Models | LocalAI, MOE policy, GGUF models |
| State | Databases | Postgres, Redis, MinIO/IPFS |
| Coordination | P2P | Holochain, DNAs, lair-keystore |
| LLMOps | Evaluation | promptfoo, TruLens, TensorZero |
| UI | Interfaces | Lobe Chat, JupyterLab, PixiJS |
| Security | Scanning | Trivy, Syft/Grype, Cosign |

### Quick Commands

```bash
# Full comprehensive audit (Opus orchestrator + 14 domain teams)
/aria-audit

# Quick scan (discovery only)
/aria-scan

# Domain-specific audits
/aria-audit-nix      # Nix/Flake configuration
/aria-audit-ci       # CI/CD workflows
/aria-audit-agents   # Agents and skills

# Generate tasks from audit
/aria-tasks
```

### Key Features

#### Installation Mapping
Every repository is mapped to its installation method:
- **Nix packages** → `flake.nix` (`pkgs.<name>`)
- **Python packages** → `pixi.toml` (`[dependencies]`)
- **Docker services** → `docker-compose.<service>.yml`
- **Rust crates** → `rust/Cargo.toml`
- **NPM wrappers** → `flake.nix` (`writeShellScriptBin npx`)
- **Binary downloads** → `flake.nix` (`fetchurl`/`buildRustPackage`)

#### Feature Flags for Conflicts
When components compete for the same responsibility:
- **Never remove** — always create A/B feature flag
- Default option defined
- Switching mechanism documented

Example:
```toml
# pixi.toml
[feature.vectordb-ruvector]
[feature.vectordb-chromadb]

[environments]
default = { features = ["vectordb-ruvector"] }
chromadb = { features = ["vectordb-chromadb"] }
```

#### Deliverables

1. **Repository Census** — 100% coverage
2. **Reference Link Audit** — All URLs validated
3. **Installation Mapping** — Every repo → method
4. **Feature Flag Matrix** — All conflicts resolved
5. **Workflow Verification** — CI checklist
6. **Task Backlog** — P0/P1/P2/P3 prioritized

## Creating New Prompts

Use ARIA v2.0 as a template. Include these sections:

```markdown
<system>       # Role, identity, model assignment
<context>      # Environment, resources, target stack
<objective>    # Mission, success criteria
<method>       # Phases, team structure, steps
<output_format># Required deliverables
<constraints>  # Operating boundaries
<thinking_guidance># Reasoning approach
<execution>    # Trigger, immediate actions
```

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 2.0.0 | 2026-01 | Major rewrite: Model specs, 14 domains, installation mapping, feature flags, docs/BUILDKIT_STARTER_SPEC.md integration |
| 1.0.0 | 2026-01 | Initial ARIA orchestrator |
