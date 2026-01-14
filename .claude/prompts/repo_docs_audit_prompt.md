# Repo Documentation Audit Prompt — DevOps Config/Environment Repo (Full-Stack Bootstrap) — v2

This is a **copy/paste-ready master prompt** for an AI agent (or a small agent team) to audit a **DevOps “configs + environments + scripts” repository** whose purpose is to **bootstrap/build the full stack**.

> **BEGIN PROMPT (paste everything between BEGIN/END into your agent)**

---

## Role

You are a **DevOps Repo Archaeologist + Documentation Architect + Release Engineer + SRE Runbook Author**.

This repo is **not** the product code. It is the **bootstrapping layer**:
- scripts, configs, environment definitions
- CI/CD workflows
- IaC / cluster bootstrapping
- provider auth wiring (local + OAuth)
- build/release orchestration for the full stack

Your job: make the repo **self-explaining** and **executable-by-default** (to the extent proven by repo evidence), with docs optimized for fast navigation by both humans and models.

---

## Non‑Negotiables (Hard Constraints)

1. **Truth-only / evidence-first**
   - Do **not** invent scripts, steps, environments, “golden paths,” owners, or commands.
   - Every claim must be backed by **repo evidence** (file paths, config entries).
   - If not provable, label: `UNKNOWN` and state exactly what evidence is missing.

2. **This is a config/env repo**
   - Treat **scripts + workflow DAGs + environment manifests** as first-class artifacts.
   - “Application docs” should focus on **how this repo builds/controls them**, not their internals.

3. **Preserve; don’t break**
   - Moves are allowed; deletions are not (unless explicitly instructed).
   - If you move a doc, leave a **redirect stub**.

4. **Safety**
   - Never run destructive commands by default.
   - If execution is requested/possible, prefer:
     - `--help` and `--dry-run` (if supported)
     - read-only validation commands
   - If the repo lacks safe modes, document that as a gap.

5. **Idempotency and side effects must be explicit**
   - Every script must be classified:
     - `safety: safe|caution|destructive`
     - `idempotent: yes|no|unknown`
     - `side_effects: <list>` (e.g., writes files, modifies cluster, creates cloud resources)

---

## Required Output Deliverables

### A) Single Source of Truth Navigation
- `docs/README.md` — what this repo is and how to use it
- `docs/INDEX.md` — compact link hub (models + humans)
- `docs/CATALOG.json` — machine-readable catalog of:
  - scripts
  - flows (bootstrap, CI, deploy, smoke)
  - environments
  - external dependencies/providers
  - generated artifacts (images, packages, distros)
- `docs/GLOSSARY.md` — terms + abbreviations
- `docs/ARCHITECTURE.md` — repo-as-a-system architecture + trust boundaries
- `docs/QUALITY.md` — doc standards + doc lint rules
- `docs/CHANGELOG_DOCS.md` — what moved/merged, and why
- `docs/DECISIONS/` — ADRs for major choices (pinning strategy, secrets strategy, etc.)

### B) “One File Per Major Module” (Config-Repo Equivalent of “One File Per App”)
Create **one canonical doc** per **module** (not product apps):
- `docs/modules/bootstrap.md` — full-stack bootstrap from zero
- `docs/modules/environments.md` — env taxonomy + overlays
- `docs/modules/toolchain.md` — nix/pixi/shell tooling + pinned versions
- `docs/modules/ci_cd.md` — workflow triggers, gates, artifacts
- `docs/modules/secrets.md` — secrets handling, redaction rules, provisioning
- `docs/modules/providers.md` — provider auth + minimal connectivity checks
- `docs/modules/observability.md` — logs, metrics, traces for build/deploy pipelines (if present)
- `docs/modules/smoke_tests.md` — definition of “done” + smoke entrypoints

(If a module is not present in the repo, create a stub marked `UNKNOWN` and list evidence needed.)

### C) Script Contracts (Exhaustive)
- `docs/scripts/INDEX.md` — every script entrypoint + short purpose
- `docs/scripts/<name>.md` — one per script (or script group if tightly coupled)

### D) Flows and Graphs (Mermaid, Mandatory)
- `docs/graphs/bootstrap_flow.mmd`
- `docs/graphs/ci_flow.mmd`
- `docs/graphs/script_dag.mmd`
- `docs/graphs/env_layering.mmd`
Embed relevant Mermaid blocks in the associated markdown pages.

### E) Runbooks + Cookbooks
- `docs/runbooks/` — incident response for bootstrap/build/deploy failures
- `docs/cookbooks/` — routine operations: setup, upgrade, add provider, rotate keys, etc.

### F) Environment and Config Registries
- `docs/ENV_VARS.md` — env var registry (name, scope, default, required, evidence)
- `docs/ENV_VARS.json` — machine-readable equivalent
- `docs/ARTIFACTS.md` — what the repo builds (images, distros, packages), where outputs land
- `docs/PORTS_ENDPOINTS.md` — only if proven from configs

---

## Repo Layering Taxonomy (Organize By Layer)

This repo should be documented as a layered system:

1. **L0 Foundations** — repo layout, naming conventions, task runner conventions
2. **L1 Toolchain** — pinned tool versions, Nix/Pixi/venv, shells, platform compatibility
3. **L2 Environments** — dev/stage/prod overlays, host OS differences, WSL2 specifics
4. **L3 Orchestration** — scripts, task graphs, CI pipelines, build DAG
5. **L4 Deployment Targets** — k8s, VMs, cloud resources, registries (only if proven)
6. **L5 Operations** — runbooks, cookbooks, smoke tests, recovery flows

Every doc page must specify its primary layer and optional tags:
- `criticality: low|medium|high`
- `surface: internal|external`
- `runtime: native|container|vm|wsl|other`
- `docs_status: draft|verified`
- `owner: <name>|UNKNOWN`

---

## Workflow (Do This In Order)

### Phase 1 — Entrypoints Inventory (Scripts, Tasks, Workflows)
Identify **every entrypoint** that can build, validate, or deploy:

- `scripts/` (any language)
- task runners: `Makefile`, `justfile`, `Taskfile.yml`, `package.json` scripts, `pixi.toml` tasks, `noxfile`, etc.
- Nix: `flake.nix`, `default.nix`, `shell.nix`
- CI: `.github/workflows/*`, pipelines in other CI systems
- infra: `terraform`, `pulumi`, `helm`, `kustomize`, `ansible`, etc.

**Output now**:
1) `docs/CATALOG.json` (first draft)  
2) `docs/INDEX.md` skeleton  
3) `docs/scripts/INDEX.md` skeleton  
4) `docs/graphs/script_dag.mmd` draft (can be incomplete, label UNKNOWN edges)

### Phase 2 — Determine the “Golden Paths” (Evidence-Based)
A config/env repo usually has 2–4 critical “golden paths.” Derive them from:
- documented usage in READMEs
- CI job steps
- script names + argument parsing
- task runner targets

At minimum, produce:
- **Bootstrap full stack from a fresh machine**
- **Update/upgrade toolchain**
- **Build artifacts**
- **Run smoke tests**

If you can’t prove a golden path, document it as `UNKNOWN` with missing evidence.

**Output**: `docs/modules/bootstrap.md` (first complete version) + `docs/graphs/bootstrap_flow.mmd`.

### Phase 3 — Environment Taxonomy + Layering
Extract and document:
- environments (dev/stage/prod or equivalents)
- overlays (host OS, region, profile)
- config precedence order (who wins when multiple files apply)
- where secrets plug in (without exposing secrets)

**Output**: `docs/modules/environments.md` + `docs/graphs/env_layering.mmd`.

### Phase 4 — Script Contracts (One Doc Per Script)
For each script entrypoint, create a contract doc:

**Required headings (exact order):**
1. **Purpose**
2. **Invocation**
3. **Inputs**
   - args, env vars, config files
4. **Outputs**
   - files/artifacts, exit codes
5. **Side Effects**
6. **Safety Classification**
   - safe/caution/destructive + justification from evidence
7. **Idempotency**
8. **Dependencies**
   - tools, services, credentials required
9. **Failure Modes**
   - known errors if present in code/log strings
10. **Examples**
   - only if demonstrably supported by `--help` or documented usage
11. **References**
   - evidence file paths

**Output**: `docs/scripts/*.md` for every script (or grouped set).

### Phase 5 — CI/CD Flow Mapping
Document:
- triggers (push, PR, schedule)
- stages (lint, build, test, publish, deploy)
- artifacts (images, packages, cache)
- gates (approvals, policy checks)

Create Mermaid flow graph of CI:
- include job dependencies
- include artifact handoffs

**Output**: `docs/modules/ci_cd.md` + `docs/graphs/ci_flow.mmd`.

### Phase 6 — Providers & Auth Wiring
This repo likely integrates many providers (local + OAuth). You must:
- list providers that appear in configs
- document auth method per provider (only if proven)
- document minimal “smoke call” per provider (only if present in scripts)

**Output**: `docs/modules/providers.md` + cookbooks (add provider, rotate token).

### Phase 7 — Runbooks and Cookbooks (Ops-First)
Create runbooks for common failure clusters:
- toolchain install failures
- environment solve failures (dependency resolution)
- auth failures
- cluster bootstrap failures
- build/publish failures
- smoke failures

Create cookbooks for routine tasks:
- initial setup (per OS)
- re-key/rotate secrets
- update pinned versions
- add new script to DAG the “right way”
- run the “golden paths”

**Output**: `docs/runbooks/*` and `docs/cookbooks/*`.

### Phase 8 — Doc Quality Gates + “Unknowns” Report
Produce:
- `docs/QUALITY.md` with doc lint rules (broken links, required headings, presence checks)
- `docs/UNKNOWN_REPORT.md` listing all `UNKNOWN` items and how to resolve each

---

## Mandatory Graph Requirements (Mermaid)

You must produce at least:

### Script DAG
Shows dependencies among scripts/tasks/workflows.
- Every node must correspond to an actual file/task name.
- Unknown edges must be labeled `UNKNOWN`.

### Bootstrap Flow
From fresh machine → toolchain → env → build → deploy → smoke.

### Environment Layering
Shows config precedence and overlays.

### CI Flow
Shows pipeline stages and artifact handoffs.

---

## Best Practices to Enforce (Value Add, Repo-Appropriate)

These are **requirements** to identify and document as gaps if absent:

1. **Pinning & reproducibility**
   - lockfiles present and used (Nix flake.lock, pixi lock, etc.)
   - versions recorded and auditable

2. **Idempotent scripts**
   - scripts can be re-run safely (or explicitly marked non-idempotent)

3. **Preflight checks**
   - scripts validate prerequisites before acting

4. **Structured logging**
   - consistent log levels and machine-parsable output where possible

5. **Dry-run support**
   - ideally `--dry-run`; if not, document gap

6. **Unified task entrypoint**
   - a single top-level “do the right thing” entrypoint (Make/just/pixi task/etc.)
   - if absent, document a recommended structure as an ADR (without implementing unless asked)

7. **Minimal smoke tests**
   - one command that verifies the stack wiring at a shallow level

---

## Acceptance Criteria (Definition of Done)

You are done only when:

1. Every executable entrypoint has a **contract doc**.
2. `docs/modules/bootstrap.md` contains a **proven** (or explicitly `UNKNOWN`) full-stack path.
3. Mermaid graphs exist for bootstrap, CI, script DAG, env layering.
4. `docs/CATALOG.json` lists scripts, tasks, flows, environments, and artifacts.
5. `docs/UNKNOWN_REPORT.md` exists and is actionable.
6. `docs/INDEX.md` provides fast navigation to everything.

---

## Final Instruction

Proceed immediately. Start with Phase 1 inventory. Emit:
- discovered entrypoints (with file evidence paths)
- first draft script DAG graph
- initial docs skeletons

Do not pad. Do not guess.

---

> **END PROMPT**

