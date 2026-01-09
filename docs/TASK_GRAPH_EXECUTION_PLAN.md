# ARIA Task Graph & Execution Plan for Manus 1.6

> **Generated**: 2026-01-09
> **Repository**: FlexNetOS/ros2-humble-env
> **Total Repositories**: 131 unique (168 with duplicates)
> **Total Domains**: 20

---

## Executive Summary

This document outlines the complete task graph and execution plan for the ARIA orchestrator running on Manus 1.6. The plan involves:

1. **Discovery Phase**: File census and repository extraction (COMPLETED)
2. **Parallel Analysis Phase**: 20 domain teams analyzing repositories concurrently
3. **Aggregation Phase**: Consolidate findings, detect conflicts, map installations
4. **Task Generation Phase**: Create prioritized backlog (P0/P1/P2/P3)
5. **Verification Phase**: Generate evidence ledger and final report

---

## Phase 1: Discovery & Census (COMPLETED ✅)

### Completed Actions

1. ✅ Cloned repository: `FlexNetOS/ros2-humble-env` (main branch)
2. ✅ Read ARIA orchestrator prompt: `.claude/prompts/aria-orchestrator.md`
3. ✅ Adapted prompt for Manus 1.6 architecture
4. ✅ Extracted repositories from BUILDKIT_STARTER_SPEC.md: 102 repos
5. ✅ Extracted repositories from README.md: 56 repos
6. ✅ Combined and deduplicated: 131 unique repos
7. ✅ Analyzed by domain: 168 repo entries across 20 domains
8. ✅ Generated `/tmp/repos_by_domain.json` with structured data

### Discovery Results

**File Census:**
```
Configuration files:
- flake.nix (Nix configuration)
- pixi.toml (Pixi/Conda packages)
- rust/Cargo.toml (Rust workspace)
- docker-compose.agixt.yml (Docker services)
- .github/workflows/*.yml (CI/CD workflows)

Documentation files:
- README.md
- BUILDKIT_STARTER_SPEC.md (SSoT)
- CONTRIBUTING.md
- SECURITY.md
- docs/*.md (multiple)
```

**Repository Census:**
```
Total unique repositories: 131
From BUILDKIT_STARTER_SPEC.md: 102
From README.md: 56
Overlap: 27
Organized into: 20 domains
```

**Domain Distribution:**
| Domain | Repo Count |
|--------|------------|
| Host OS & Environment | 3 |
| Isolation & Runtime | 3 |
| Cluster & Delivery | 3 |
| Edge & Agent Traffic | 2 |
| Identity & Policy | 5 |
| Messaging & Orchestration | 3 |
| Agent Runtime | 10 |
| Tool Execution | 5 |
| Inference | 1+ |
| State & Storage | Multiple |
| Data & Query | 3 |
| LLMOps & Evaluation | 3 |
| Training | 2 |
| UI | 3 |
| Build Tools | 2 |
| Notebooks | 1 |
| Observability | 5 |
| Security | 4 |
| DevOps & Autonomy | 2 |
| Additional Tools | Many |

---

## Phase 2: Parallel Domain Analysis (READY TO EXECUTE)

### Execution Strategy

Use Manus `map` tool to deploy 20 parallel domain analysis agents. Each agent will:

1. **Analyze domain repositories**: Extract all repos for the domain
2. **Determine installation methods**: Nix/Pixi/Docker/Cargo/NPM/Binary/Submodule
3. **Check current status**: Installed/Missing/Partial
4. **Detect conflicts**: Identify overlapping components
5. **List missing dependencies**: Find gaps in installation
6. **Generate verification commands**: Commands to test installation
7. **Prioritize tasks**: Assign P0/P1/P2/P3 priorities

### Map Tool Configuration

**Inputs (20 domains):**
```
1. Host OS & Environment
2. Isolation & Runtime
3. Cluster & Delivery
4. Edge & Agent Traffic
5. Identity & Policy
6. Messaging & Orchestration
7. Agent Runtime
8. Tool Execution
9. Inference
10. State & Storage
11. Data & Query
12. LLMOps & Evaluation
13. Training
14. UI
15. Build Tools
16. Notebooks
17. Observability
18. Security
19. DevOps & Autonomy
20. Additional Tools
```

**Output Schema:**
```json
{
  "domain": "string - Domain name",
  "repos_found": "number - Count of repositories",
  "repos_list": "string - Comma-separated repo names",
  "installation_method": "string - Primary method (Nix|Pixi|Docker|Cargo|NPM|Binary|Submodule)",
  "current_status": "string - Installation status summary",
  "conflicts_detected": "string - Conflicting components or 'None'",
  "missing_deps": "string - Missing dependencies or 'None'",
  "verification_commands": "string - Shell commands to verify",
  "priority_tasks": "string - Top 3 tasks with priorities"
}
```

**Prompt Template:**
```
You are a domain specialist analyzing the {{input}} domain for the ros2-humble-env repository.

Context:
- Repository: /home/ubuntu/ros2-humble-env
- Domain data: /tmp/repos_by_domain.json
- SSoT: BUILDKIT_STARTER_SPEC.md

Your tasks:
1. Read the repos_by_domain.json file and extract all repositories for the {{input}} domain
2. For each repository, determine the appropriate installation method (Nix/Pixi/Docker/Cargo/NPM/Binary/Submodule)
3. Check the current repository files (flake.nix, pixi.toml, docker-compose.*.yml, rust/Cargo.toml) to determine installation status
4. Identify any conflicting components that serve the same purpose
5. List any missing dependencies or tools
6. Generate verification commands to test installation
7. Create a prioritized list of the top 3 tasks for this domain

Output all findings in the structured format provided.
```

---

## Phase 3: Aggregation & Conflict Resolution (AFTER MAP COMPLETES)

### Actions

1. **Aggregate domain reports**: Collect all 20 domain analysis results
2. **Build installation status matrix**: Create tables for Nix/Pixi/Docker/Cargo/NPM
3. **Detect conflicts**: Identify overlapping components across domains
4. **Design feature flags**: Create A/B flags for conflicting components
5. **Map dependencies**: Build dependency graph between components
6. **Generate verification checklist**: Create comprehensive test commands

### Expected Outputs

**Installation Status Matrix:**
```markdown
## Nix Packages (flake.nix)
| Package | Current | Required | Status | Priority |
|---------|---------|----------|--------|----------|
| nats-server | ✅ | ✅ | Installed | - |
| holochain | ❌ | ✅ | MISSING | P0 |
| vault | ✅ | ✅ | Installed | - |
...

## Pixi Packages (pixi.toml)
| Package | Current | Required | Status | Priority |
|---------|---------|----------|--------|----------|
| pytorch | ✅ | ✅ | Installed | - |
| mlflow | ❌ | ✅ | MISSING | P1 |
...

## Docker Services (docker-compose.*.yml)
| Service | File | Status | Priority |
|---------|------|--------|----------|
| agixt | docker-compose.agixt.yml | ✅ Configured | - |
| keycloak | MISSING | ❌ Not configured | P0 |
...

## Rust Crates (rust/Cargo.toml)
| Crate | Current | Required | Status | Priority |
|-------|---------|----------|--------|----------|
| sqlx | ✅ | ✅ | Installed | - |
| holochain | ❌ | ✅ | MISSING | P0 |
...
```

**Feature Flag Matrix:**
```markdown
| Conflict Area | Option A | Option B | Default | Config Location | Rationale |
|---------------|----------|----------|---------|-----------------|-----------|
| Inference | LocalAI | vLLM | A | pixi.toml | BUILDKIT_STARTER_SPEC.md specifies LocalAI as primary |
| Vector DB | ruvector | chromadb | A | pixi.toml | ruvector is Rust-native, aligns with stack |
| Container Runtime | Kata | Firecracker | A | flake.nix | Kata is default, Firecracker for high-risk |
| Object Store | MinIO | rustfs | A | docker-compose | MinIO is mature, rustfs is experimental |
```

---

## Phase 4: Task Backlog Generation (AFTER AGGREGATION)

### Task Prioritization Rules

**P0 — Immediate (Blocking):**
- Missing critical dependencies that block core functionality
- Broken workflows that prevent CI/CD
- Configuration errors that prevent startup
- Security vulnerabilities (Critical/High)

**P1 — High Priority (Core Stack):**
- Core component installations (13 layers from BUILDKIT_STARTER_SPEC.md)
- Primary feature configurations
- Integration tests for core components
- Documentation for critical features

**P2 — Standard Priority (Secondary):**
- Secondary component installations
- Optional features
- Enhancement of existing features
- Non-critical documentation updates

**P3 — Backlog (Optional/R&D):**
- Experimental features
- Research components
- Future enhancements
- Nice-to-have optimizations

### Task Template

Each task includes:
```markdown
### [Priority] Task Title

**Description**: Clear description of what needs to be done

**Domain**: [Domain name from 20 domains]

**Affected Files**:
- `file1.nix`
- `file2.toml`

**Installation Method**: Nix | Pixi | Docker | Cargo | NPM | Binary | Submodule

**Dependencies**:
- Task ID: [Blocking task if any]
- External: [External dependencies]

**Verification Commands**:
```bash
# Commands to verify completion
command1
command2
```

**Acceptance Criteria**:
- [ ] Criterion 1
- [ ] Criterion 2
- [ ] Smoke test passes

**Estimated Effort**: X hours

**Evidence Required**:
- File hashes (SHA-256)
- Test output logs
- Configuration diffs
```

---

## Phase 5: Verification & Evidence (FINAL PHASE)

### Evidence Ledger

**Required Artifacts:**

1. **FINAL_REPORT.md**: Comprehensive report with:
   - Claims table
   - Evidence ledger
   - Truth Gate checklist
   - Gap scan results
   - Task backlog

2. **HASHES.txt**: SHA-256 hashes of all key files
   ```bash
   find . -type f \( -name "*.nix" -o -name "*.toml" -o -name "*.yml" -o -name "Cargo.toml" \) \
     ! -path "./.git/*" ! -path "./.pixi/*" \
     -print0 | sort -z | xargs -0 sha256sum > HASHES.txt
   ```

3. **INSTALLATION_STATUS.md**: Current vs required status for all components

4. **FEATURE_FLAGS.md**: All A/B feature flags with switching instructions

5. **VERIFICATION_CHECKLIST.md**: Commands to verify each component

6. **TASK_BACKLOG.md**: Prioritized task list with estimates

### Truth Gate Checklist

```markdown
## Truth Gate Checklist

- [ ] All 131 repositories cataloged with source file and line number
- [ ] All 20 domains analyzed with findings documented
- [ ] Installation methods determined for all repositories
- [ ] Current status verified for all components (Installed/Missing/Partial)
- [ ] All conflicts identified and feature flags designed
- [ ] All missing dependencies listed
- [ ] Verification commands provided for all components
- [ ] Task backlog generated with P0/P1/P2/P3 priorities
- [ ] SHA-256 hashes generated for all key configuration files
- [ ] Evidence ledger complete with file paths and timestamps
- [ ] Triple-verification protocol completed (Pass A/B/C)
- [ ] No fabricated data, all findings cite file:line or URL
- [ ] Cross-platform considerations documented (Linux/macOS/WSL2)
- [ ] No features omitted, all conflicts resolved with A/B flags
```

### Triple-Verification Protocol

**Pass A — Self-check:**
- Internal consistency of findings
- Spec ↔ artifacts ↔ tests mapping
- Unit smoke tests for key components

**Pass B — Independent re-derivation:**
- Re-extract repository lists from source files
- Re-count repositories and verify against initial census
- Re-analyze sample domains to confirm methodology

**Pass C — Adversarial check:**
- Negative tests (check for repos that should NOT be included)
- Boundary cases (repos with ambiguous installation methods)
- Cross-tool verification (compare grep results with Python analysis)

---

## Execution Timeline

### Estimated Duration

| Phase | Duration | Parallelization |
|-------|----------|-----------------|
| Phase 1: Discovery | ✅ COMPLETED | Sequential |
| Phase 2: Domain Analysis | 10-15 min | 20 parallel agents |
| Phase 3: Aggregation | 5-10 min | Sequential |
| Phase 4: Task Generation | 5-10 min | Sequential |
| Phase 5: Verification | 5-10 min | Sequential |
| **Total** | **25-45 min** | Mixed |

### Execution Order

```
Phase 1 (DONE) → Phase 2 (20 parallel) → Phase 3 → Phase 4 → Phase 5
                      ↓
            [Domain 1] [Domain 2] ... [Domain 20]
                      ↓
            Aggregate all results
                      ↓
            Detect conflicts & map installations
                      ↓
            Generate prioritized task backlog
                      ↓
            Create evidence ledger & final report
```

---

## Success Criteria

### Quantitative Metrics

- [ ] 131 unique repositories cataloged
- [ ] 20 domain reports generated
- [ ] 100% of repositories mapped to installation method
- [ ] All conflicts identified and feature-flagged
- [ ] Task backlog with ≥50 tasks across all priorities
- [ ] ≥95% of components have verification commands
- [ ] SHA-256 hashes for ≥100 key files

### Qualitative Metrics

- [ ] No omitted features or repositories
- [ ] All findings cite file:line or URL
- [ ] Clear A/B switching mechanism for all conflicts
- [ ] Cross-platform considerations documented
- [ ] Evidence-based recommendations only
- [ ] Triple-verification protocol completed

---

## Risk Mitigation

### Potential Issues

1. **Large repository count (131)**: Mitigated by parallel processing (20 domains)
2. **Complex dependencies**: Mitigated by dependency graph generation
3. **Conflicting components**: Mitigated by A/B feature flags
4. **Missing documentation**: Mitigated by evidence-based analysis
5. **Cross-platform differences**: Mitigated by explicit platform checks

### Contingency Plans

- If domain analysis fails: Retry with more specific prompts
- If conflicts unresolvable: Escalate to user for decision
- If verification commands fail: Document as P0 task
- If installation method unclear: Mark as "Requires Investigation"

---

## Next Steps

1. **Execute Phase 2**: Launch 20 parallel domain analysis agents via `map` tool
2. **Monitor progress**: Track completion of all 20 subtasks
3. **Aggregate results**: Consolidate findings into structured reports
4. **Generate artifacts**: Create all required evidence files
5. **Deliver to user**: Present FINAL_REPORT.md with attachments

---

## Appendix: Domain Details

### Domain 1: Host OS & Environment
- **Repositories**: NixOS/nixpkgs, prefix-dev/pixi, nushell/nushell
- **Installation**: Nix (system-level)
- **Priority**: P0 (foundational)

### Domain 2: Isolation & Runtime
- **Repositories**: sandbox-runtime, kata-containers, firecracker
- **Installation**: Nix + Binary
- **Priority**: P0 (security-critical)

### Domain 3: Cluster & Delivery
- **Repositories**: argo-cd, argo-rollouts, argo-workflows
- **Installation**: Nix + Helm
- **Priority**: P1 (deployment automation)

### Domain 4: Edge & Agent Traffic
- **Repositories**: Kong/kong, agentgateway/agentgateway
- **Installation**: Docker + Binary
- **Priority**: P0 (traffic routing)

### Domain 5: Identity & Policy
- **Repositories**: keycloak, opa, vault, smallstep/cli, vaultwarden
- **Installation**: Docker + Nix
- **Priority**: P0 (security gates)

### Domain 6: Messaging & Orchestration
- **Repositories**: nats-server, temporal, n8n
- **Installation**: Nix + Docker
- **Priority**: P0 (autonomy backbone)

### Domain 7: Agent Runtime
- **Repositories**: AIOS, Cerebrum, AGiXT, agno, agentic-flow, claude-flow, Synaptic-Mesh, daa, refact, arkflow
- **Installation**: Mixed (Pixi + Docker + Cargo)
- **Priority**: P0 (core agent substrate)

### Domain 8: Tool Execution
- **Repositories**: genai-toolbox, remote-agentic-coding-system, midstream, ruv-FANN, sublinear-time-solver
- **Installation**: Pixi + Cargo
- **Priority**: P1 (agent tools)

### Domain 9: Inference
- **Repositories**: LocalAI
- **Installation**: Docker + Pixi
- **Priority**: P0 (model serving)

### Domain 10: State & Storage
- **Repositories**: postgres, redis, minio, kubo, holochain, lair
- **Installation**: Docker + Nix + Cargo
- **Priority**: P0 (data persistence)

### Domain 11: Data & Query
- **Repositories**: mindsdb, datafusion, neo4j-labs
- **Installation**: Pixi + Cargo
- **Priority**: P1 (analytics)

### Domain 12: LLMOps & Evaluation
- **Repositories**: promptfoo, trulens, tensorzero
- **Installation**: NPM + Pixi
- **Priority**: P1 (quality gates)

### Domain 13: Training
- **Repositories**: mlflow, unsloth
- **Installation**: Pixi
- **Priority**: P2 (model training)

### Domain 14: UI
- **Repositories**: lobe-chat, open-lovable, pixijs
- **Installation**: Docker + NPM
- **Priority**: P1 (operator interface)

### Domain 15: Build Tools
- **Repositories**: swc, esbuild
- **Installation**: Nix + NPM
- **Priority**: P1 (build optimization)

### Domain 16: Notebooks
- **Repositories**: jupyterlab
- **Installation**: Pixi
- **Priority**: P2 (development tools)

### Domain 17: Observability
- **Repositories**: prometheus, grafana, loki, netdata, umami
- **Installation**: Docker + Nix
- **Priority**: P1 (monitoring)

### Domain 18: Security
- **Repositories**: trivy, syft, grype, cosign
- **Installation**: Nix
- **Priority**: P0 (vulnerability scanning)

### Domain 19: DevOps & Autonomy
- **Repositories**: agenticsorg/devops, quantum-agentics
- **Installation**: Pixi + Cargo
- **Priority**: P1 (agentic ops)

### Domain 20: Additional Tools
- **Repositories**: sqlite, neon, bytebase, vCache, prompt-cache, ruvector, opentelemetry-collector, containerd, runc, helm, kubectl, kustomize, neovim, git, jj, curl, jq, yq
- **Installation**: Mixed (Nix primary)
- **Priority**: P1-P2 (supporting tools)

---

**END OF TASK GRAPH & EXECUTION PLAN**
