# Phase 5: CI/CD Flow Documentation

**Status:** Phase 5 Complete
**Last Updated:** 2026-01-13
**Purpose:** Comprehensive mapping of CI/CD workflows, script dependencies, and operational paths

---

## Overview

This document provides a complete mapping of the CI/CD infrastructure in ripple-env, including:
- 27 GitHub Actions workflows
- 57 automation scripts
- Makefile integration
- Tool ecosystem (Nix, Pixi, Docker)
- Golden paths for common operations

---

## Table of Contents

1. [CI/CD Architecture](#cicd-architecture)
2. [Workflow Catalog](#workflow-catalog)
3. [Script Execution Flows](#script-execution-flows)
4. [Golden Paths](#golden-paths)
5. [Integration Points](#integration-points)
6. [Dependency Graph](#dependency-graph)

---

## CI/CD Architecture

### Trigger Events

**Push Events:**
- `push` to `main`/`master` → Triggers CI, Security, Release workflows
- Validates code quality before merge

**Pull Requests:**
- `pull_request` → Triggers CI, E2E Validation, Component Verification
- Comprehensive validation before merge

**Scheduled:**
- Weekly Monday 3 AM UTC → Security scanning
- Periodic dependency updates

**Manual Dispatch:**
- `workflow_dispatch` → All workflows support manual triggering
- Useful for testing and troubleshooting

### Three-Layer Model

```
┌─────────────────────────────────────────────────────────┐
│ Layer 1: GitHub Actions Workflows (27 workflows)       │
│ - Orchestration and coordination                        │
│ - Matrix builds, parallel execution                     │
│ - Artifact management                                    │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 2: Automation Scripts (57 scripts)               │
│ - Reusable business logic                               │
│ - Called by workflows and Makefile                      │
│ - Platform-specific implementations                      │
└─────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────┐
│ Layer 3: Tool Ecosystem                                 │
│ - Nix (flakes, devShells)                              │
│ - Pixi (conda environments)                             │
│ - Docker (compose, services)                            │
│ - npm/pnpm (JS packages)                                │
└─────────────────────────────────────────────────────────┘
```

---

## Workflow Catalog

### Core Workflows (5)

#### 1. **ci.yml** - Main CI Pipeline
**Triggers:** push, pull_request
**Jobs:**
- `flake-check`: Validate Nix flake (`nix flake check`, `nix flake show`)
- `ros2-build`: Build & test ROS2 packages (`colcon build`, `colcon test`)
- `macos-build`: Verify macOS compatibility
- `security`: Trivy filesystem scan
- `pre-commit`: Code style checks
- `summary`: Aggregate results, generate report

**Required for merge:** flake-check, ros2-build, security

**Key Features:**
- Concurrency control (cancel in-progress)
- Disk space management (frees 15-20GB before build)
- Matrix builds (Ubuntu, macOS)
- SARIF upload for security findings

**Scripts Called:**
- None directly (inline commands)

---

#### 2. **e2e-validation.yml** - End-to-End Validation
**Triggers:** push, pull_request, workflow_dispatch
**Jobs (6 phases):**

**Phase 1: Configuration Validation**
- YAML/TOML syntax validation
- Docker Compose file validation
- Python-based parsers

**Phase 2: Nix Flake Validation**
- `nix flake check --no-build`
- Metadata verification

**Phase 3: Pixi Environment Validation**
- `pixi install`
- Package import tests (numpy, pandas, torch, etc.)

**Phase 4: Docker Service Integration**
- Observability stack (Prometheus, Grafana, Loki)
- Messaging stack (NATS, Temporal)
- Automation stack (OPA, n8n)
- Edge stack (Kong Gateway)
- AI stack (LocalAI, AGiXT)

**Phase 5: Integration Tests**
- Python test suite (`pytest`)
- ROS2 environment checks

**Phase 6: Report Generation**
- Validation summary
- Artifact upload

**Scripts Called:**
- `scripts/health-check.sh` (service readiness)

**Key Features:**
- Optional Docker tests (`skip_docker` input)
- Comprehensive service coverage
- Health check retry logic
- Log collection on failure

---

#### 3. **component-verification.yml** - Component Validation
**Triggers:** push (on manifest/config changes), pull_request, workflow_dispatch
**Jobs (4 phases):**

**Phase 1: Manifest Validation**
- Validate `ARIA_MANIFEST.yaml` against JSON schema
- Generate verification scripts from manifest

**Phase 2: Nix Component Verification**
- Infrastructure tools (nix, direnv, git, gh)
- Package managers (pixi)
- Dev tools (nom, nixfmt)
- Holochain toolchain

**Phase 3: Pixi Component Verification**
- Python packages (numpy, pandas, torch)
- ROS2 tools (ros2, colcon)
- Code quality (ruff, black, mypy)
- ML packages (transformers, mlflow)

**Phase 4: Configuration Consistency**
- Required files check (flake.nix, pixi.toml, etc.)
- YAML/TOML validation
- Cross-reference checks

**Scripts Called:**
- `scripts/validate-manifest.py`
- `scripts/generate-verification.py`

**Key Features:**
- Profile-based verification (minimal, ci, default, full)
- Manifest-driven automation
- Generated verification scripts

---

#### 4. **security.yml** - Security Scanning
**Triggers:** push, pull_request, weekly schedule, workflow_dispatch
**Jobs (4 parallel scans):**

**CodeQL Analysis:**
- Language: Python
- Queries: security-and-quality
- SARIF upload to GitHub Security

**Trivy Filesystem Scan:**
- Severity: CRITICAL, HIGH
- Ignore unfixed vulnerabilities
- SARIF format

**Trivy Config Scan:**
- IaC misconfigurations
- Best practices checks
- Table format (non-blocking)

**Grype SBOM Scan:**
- Generate SBOM with Syft (CycloneDX format)
- Scan with Grype
- Comprehensive vulnerability database

**Scripts Called:**
- None (uses GitHub Actions)

**Key Features:**
- Multiple scanning tools (defense in depth)
- SARIF integration with GitHub Security
- Weekly automated scans
- Non-blocking config checks

---

#### 5. **release.yml** - Automated Releases
**Triggers:** push to main, workflow_dispatch
**Jobs:**

**check-release:**
- Parse conventional commits
- Determine version bump (major/minor/patch)
- Breaking changes detection

**generate-changelog:**
- Git log parsing
- Categorize changes (features, fixes, breaking)
- Markdown format

**create-release:**
- Create git tag
- GitHub release creation
- Changelog attachment

**publish-artifacts:**
- Nix flake publication
- Docker image push
- Binary artifacts

**Scripts Called:**
- None (git operations)

**Key Features:**
- Conventional commits enforcement
- Automated versioning (semver)
- Changelog generation
- Artifact publishing

---

### Specialized Workflows (22)

#### Testing & Validation (6)

**bootstrap-test.yml**
- Test bootstrap scripts on multiple platforms
- Linux, macOS, Windows/WSL2

**test-bootstrap.yml**
- Verify bootstrap prerequisites
- Dependency checks

**benchmarks.yml**
- Performance benchmarking
- Nix flake evaluation timing
- `scripts/benchmark-eval.sh`

**python-matrix.yml**
- Python version matrix (3.10, 3.11, 3.12)
- Dependency compatibility

**realtime-latency.yml**
- Real-time performance tests
- CPU isolation verification
- `scripts/isolate-cpu.sh`

**eval-gate.yml**
- Nix flake evaluation performance gate
- Fail if evaluation >15s

---

#### Build & Artifacts (3)

**nixos-images.yml**
- Build NixOS images (ISO, WSL2, VM)
- `nixos-generate` integration

**wsl2-build.yml**
- Build WSL2 tarball
- Windows-specific testing

**flakehub-publish-tagged.yml**
- Publish to FlakeHub on tagged releases

---

#### Security & Compliance (4)

**container-security.yml**
- Scan all Docker images with Trivy
- `scripts/scan-containers.sh`

**sbom.yml**
- Generate Software Bill of Materials
- Syft + CycloneDX

**attestation.yml**
- SLSA provenance attestation
- Build artifact signing

**opa-policy-gate.yml**
- Open Policy Agent policy enforcement
- Authorization checks

---

#### Code Quality (3)

**shellcheck.yml**
- Lint all shell scripts
- POSIX compliance

**editorconfig.yml**
- EditorConfig compliance
- Consistent formatting

**config-validation.yml**
- YAML/TOML validation
- `scripts/validate-configs.sh`

---

#### AI/ML Workflows (4)

**localai-test.yml**
- LocalAI model loading
- Inference API testing

**agixt-test.yml**
- AGiXT agent framework
- Agent execution tests

**verify-ai-tools.yml**
- AI toolchain verification
- Model availability

**k8s-validation.yml**
- Kubernetes manifest validation
- Helm chart linting

---

#### Documentation (2)

**docs.yml**
- Build documentation
- Deploy to GitHub Pages

**performance.yml**
- Generate performance reports
- Benchmark visualization

---

## Script Execution Flows

See [`script-execution-flow.mmd`](graphs/script-execution-flow.mmd) for visual diagram.

### Bootstrap Phase

**Linux/macOS:**
```bash
bootstrap.sh
├── Check prerequisites
│   ├── curl/wget
│   ├── git
│   └── tar
├── Install Nix (if missing)
│   └── Deterministic Systems installer
├── Enable Nix flakes
├── Setup direnv
└── Install Pixi
```

**Windows:**
```powershell
bootstrap.ps1
├── Check prerequisites
│   ├── PowerShell 7+
│   ├── WSL2
│   └── Git for Windows
├── Install Nix in WSL2
├── Setup Windows integration
└── Configure WSL2 optimization
    ├── fix-wsl-stability.sh
    └── stable-env.sh
```

---

### Validation Phase

**validate-configs.sh** → Entry point for all validation
```bash
validate-configs.sh
├── Docker Compose validation
│   ├── Syntax check (docker compose config)
│   └── Service dependencies
├── YAML/TOML parsing
│   ├── Python parsers
│   └── Schema validation
└── Nix flake check
    ├── nix flake check
    └── nix flake show
```

**validate-e2e.sh** → Comprehensive 6-phase validation
```bash
validate-e2e.sh
├── Phase 1: Config validation (calls validate-configs.sh)
├── Phase 2: Network initialization
│   └── init-docker-networks.sh
├── Phase 3: Service deployment
│   ├── deploy-observability.sh
│   ├── deploy-edge.sh
│   └── deploy.sh
├── Phase 4: Health checks
│   └── health-check.sh (per service)
├── Phase 5: Verification scripts
│   ├── verify-observability.sh
│   ├── verify-edge.sh
│   ├── verify-state-storage.sh
│   └── verify-mtls-setup.sh
└── Phase 6: Integration tests
    └── pytest
```

---

### Initialization Phase

**Prerequisite: Docker must be running**

```bash
# 1. Networks
init-docker-networks.sh
├── agentic-network (internal)
├── observability (external)
└── edge-network (external)

# 2. Databases
init-multi-db.sh
├── PostgreSQL (port 5432)
│   ├── Create databases (aios, temporal, keycloak)
│   └── Create users
├── Redis (port 6379)
│   └── Basic auth
└── Neo4j (port 7687)
    └── Graph database

# 3. Messaging
init-jetstream.sh
├── Create streams (6 total)
│   ├── AGENTS (agent lifecycle)
│   ├── TASKS (task queue)
│   ├── EVENTS (system events)
│   ├── LOGS (structured logs)
│   ├── METRICS (telemetry)
│   └── STATE (state snapshots)
└── Create consumers (7 total)

# 4. Certificate Authority
init-step-ca.sh
├── Initialize root CA
├── Create intermediate CA
├── Generate passwords
└── Start step-ca container
```

---

### Certificate Management

**Zero-Trust mTLS Infrastructure:**

```bash
# Generate service certificates
generate-service-certs.sh
├── Service identity certs
│   ├── Common Name: service-name
│   ├── SAN: DNS, IP
│   └── Usage: TLS Server Auth, TLS Client Auth
├── Certificate chain validation
└── Store in secrets/

# Rotate certificates
rotate-certs.sh
├── Backup existing certs (timestamped)
├── Generate new certs (same identity)
├── Verify chain
└── Update services

# Automated rotation
setup-cert-rotation-cron.sh
├── Cron job (weekly)
└── Pre-expiry rotation (7 days before)
```

---

### Deployment Phase

**install-all.sh** - Complete system installation (7 phases):

```bash
install-all.sh
├── Phase 1: Validate prerequisites
│   ├── validate-configs.sh
│   └── validate-resources.sh
├── Phase 2: Initialize infrastructure
│   ├── init-docker-networks.sh
│   ├── init-multi-db.sh
│   └── init-jetstream.sh
├── Phase 3: Setup certificates
│   ├── init-step-ca.sh
│   └── generate-service-certs.sh
├── Phase 4: Deploy core services
│   ├── PostgreSQL, Redis, NATS
│   └── Health check validation
├── Phase 5: Deploy service groups
│   ├── Observability (Prometheus, Grafana, Loki)
│   ├── Messaging (NATS, Temporal)
│   ├── Automation (OPA, n8n)
│   ├── Edge (Kong, Nginx)
│   ├── Inference (LocalAI)
│   └── UI (Dashboards)
├── Phase 6: Install Argo CD/Workflows/Rollouts
│   ├── install-argocd.sh
│   ├── install-argo-rollouts.sh
│   └── setup-argo-workflows.sh
└── Phase 7: Verification
    ├── verify-mtls-setup.sh
    ├── verify-observability.sh
    ├── verify-state-storage.sh
    └── verify-argo-workflows.sh
```

**Deployment Variants:**

- **deploy.sh** - Main stack only (core services)
- **deploy-observability.sh** - Monitoring stack
- **deploy-edge.sh** - API gateway and ingress
- **deploy-airgapped.sh** - Offline/air-gapped deployment

---

### Verification Phase

**Purpose:** Verify services are healthy and correctly configured

```bash
# mTLS Infrastructure
verify-mtls-setup.sh
├── Check Step-CA health
├── Validate certificate chain
├── Verify service certs
└── Test mutual TLS handshake

# Observability Stack
verify-observability.sh
├── Prometheus
│   ├── Health endpoint
│   ├── Scrape targets
│   └── Alert rules
├── Grafana
│   ├── Health endpoint
│   ├── Data sources
│   └── Dashboards
└── Loki
    ├── Health endpoint
    └── Log ingestion

# Edge Services
verify-edge.sh
├── Kong Gateway
│   ├── Admin API
│   ├── Routes
│   └── Plugins
└── Nginx
    └── Configuration test

# Messaging
verify-jetstream.sh
├── NATS server health
├── Stream status (6 streams)
└── Consumer status (7 consumers)

# State Storage
verify-state-storage.sh
├── PostgreSQL
│   ├── Connection test
│   └── Database queries
├── Redis
│   ├── PING command
│   └── Key operations
└── Temporal
    ├── Frontend gRPC
    └── Workflow registration

# AI Services
verify-mindsdb.sh
├── API health
├── Database connections
└── ML model availability

# Automation
verify-argo-workflows.sh
├── Argo server
├── Workflow CRDs
└── RBAC permissions

# Specialized Components
verify-open-lovable.sh  # Open-source Lovable UI
verify-qudag.sh         # Quantum-resistant DAG
verify-ruvector.sh      # Vector database (npm)
```

---

## Golden Paths

See [`golden-paths.mmd`](graphs/golden-paths.mmd) for visual diagram.

### Path 1: Fresh Install (First-Time Setup)

**Objective:** Get a working development environment from scratch

**Prerequisites:**
- Linux/macOS/Windows with WSL2
- 16GB+ RAM, 50GB+ disk space
- Internet connection

**Steps:**

1. **Run Bootstrap**
   ```bash
   # Linux/macOS
   ./bootstrap.sh

   # Windows (PowerShell as Admin)
   .\bootstrap.ps1
   ```

2. **Validate Setup**
   ```bash
   ./scripts/validate-configs.sh
   ```

3. **Install Dependencies (Minimal)**
   ```bash
   ./scripts/install-all.sh --minimal
   ```

4. **Verify Installation**
   ```bash
   ./scripts/validate-e2e.sh
   ```

5. **Start Development**
   ```bash
   nix develop
   ```

**Expected Time:** 30-60 minutes

---

### Path 2: Development Workflow

**Objective:** Daily development workflow

**Steps:**

1. **Enter Development Shell**
   ```bash
   nix develop
   # or with direnv
   cd ripple-env  # auto-loads .envrc
   ```

2. **Start Development Services**
   ```bash
   # Minimal (core only)
   ./scripts/flexstack.sh up minimal

   # Development (core + tools)
   ./scripts/flexstack.sh up dev

   # AI (core + inference)
   ./scripts/flexstack.sh up ai
   ```

3. **Make Changes**
   - Edit code
   - Add dependencies to `pixi.toml` or `flake.nix`
   - Update documentation

4. **Run Tests**
   ```bash
   # Quick local tests
   make test

   # Full validation
   make test-e2e
   ```

5. **Commit Changes**
   ```bash
   git add .
   git commit -m "feat: add new feature"
   # Pre-commit hooks run automatically
   ```

6. **Push & CI**
   ```bash
   git push
   # Triggers GitHub Actions workflows
   ```

**Daily Cycle:** 5-15 minutes per iteration

---

### Path 3: Service Deployment

**Objective:** Deploy full stack for production-like environment

**Prerequisites:**
- Docker running
- 32GB+ RAM for full stack
- Nix and Pixi installed

**Steps:**

1. **Validate Resources**
   ```bash
   ./scripts/validate-resources.sh
   ```

2. **Initialize Networks**
   ```bash
   ./scripts/init-docker-networks.sh
   ```

3. **Setup Certificates (mTLS)**
   ```bash
   ./scripts/init-step-ca.sh
   ./scripts/generate-service-certs.sh
   ```

4. **Deploy Stack**

   **Option A: Profile-based (Recommended)**
   ```bash
   # Minimal (core services only): 1GB RAM
   ./scripts/flexstack.sh up minimal

   # AI stack (core + inference): 6.2GB RAM
   ./scripts/flexstack.sh up ai

   # Observability: 1.2GB RAM
   ./scripts/flexstack.sh up observability

   # Full stack: 10GB+ RAM
   ./scripts/flexstack.sh up full
   ```

   **Option B: Complete installation**
   ```bash
   ./scripts/install-all.sh
   ```

5. **Verify Deployment**
   ```bash
   # Check all services
   ./scripts/flexstack.sh health

   # Verify specific stacks
   ./scripts/verify-mtls-setup.sh
   ./scripts/verify-observability.sh
   ./scripts/verify-state-storage.sh
   ```

6. **Monitor Services**
   - Grafana: http://localhost:3000 (admin/admin)
   - Prometheus: http://localhost:9090
   - NATS Monitoring: http://localhost:8222
   - Kong Admin: http://localhost:8001
   - LocalAI API: http://localhost:8080/v1

**Deployment Time:**
- Minimal: 5-10 minutes
- Full: 30-45 minutes (includes model downloads)

---

### Path 4: Troubleshooting

**Objective:** Diagnose and fix issues

**Common Issues:**

**Issue: Service won't start**
```bash
# 1. Check service status
./scripts/flexstack.sh status

# 2. View logs
./scripts/flexstack.sh logs [service-name]
# or
docker compose logs -f [service-name]

# 3. Verify health
./scripts/flexstack.sh health

# 4. Manual health check
./scripts/health-check.sh http://localhost:PORT/health 5 10 "Service Name"
```

**Issue: Configuration errors**
```bash
# Validate all configs
./scripts/validate-configs.sh

# Check specific file
docker compose -f docker/docker-compose.yml config
```

**Issue: Disk space (WSL2)**
```bash
# Inside WSL
./scripts/wsl-cleanup.sh --all

# From Windows (PowerShell)
.\scripts\Cleanup-WSL.ps1 -Cleanup -Compact
```

**Issue: Port conflicts**
```bash
# Check port usage
netstat -tulpn | grep :PORT  # Linux
lsof -i :PORT                # macOS
netstat -ano | findstr :PORT # Windows

# Or use flexstack status
./scripts/flexstack.sh status
```

**Issue: Certificate errors**
```bash
# Verify mTLS setup
./scripts/verify-mtls-setup.sh

# Regenerate certificates
./scripts/rotate-certs.sh
```

**Issue: Python dependency conflicts**
```bash
# Check dependencies
./scripts/check-python-deps.sh

# Upgrade if needed
./scripts/upgrade-python-deps.sh
```

---

### Path 5: Security Audit

**Objective:** Comprehensive security review

**Prerequisites:**
- Docker running
- Services deployed

**Steps:**

1. **Scan Containers**
   ```bash
   ./scripts/scan-containers.sh
   # Generates: reports/*.{html,json,sarif}
   ```

2. **Run Full Security Audit**
   ```bash
   ./scripts/security-audit.sh
   # Runs: Trivy + Gitleaks
   ```

3. **Check Certificates**
   ```bash
   ./scripts/verify-mtls-setup.sh
   ```

4. **Rotate Certificates (if needed)**
   ```bash
   # Check expiry
   openssl x509 -in secrets/service.crt -noout -dates

   # Rotate
   ./scripts/rotate-certs.sh
   ```

5. **Review Reports**
   ```bash
   # HTML reports
   open reports/trivy-report.html

   # JSON for automation
   cat reports/trivy-results.json | jq '.Results[].Vulnerabilities'

   # SARIF for GitHub
   # Upload to GitHub Security tab
   ```

**Frequency:** Weekly (automated via GitHub Actions)

---

## Integration Points

### Makefile Integration

**Purpose:** Convenience wrapper for common commands

**Targets:**

| Target | Script/Command | Purpose |
|--------|---------------|---------|
| `make help` | - | Show available targets |
| `make doctor` | `node --version`, `docker --version`, etc. | Show toolchain versions |
| `make install` | `npm install` | Install JS dependencies |
| `make build` | `npm run swc:build` | Build JS artifacts |
| `make test` | `validate-configs.sh`, `nix flake check`, etc. | Local validation |
| `make test-e2e` | `validate-e2e.sh` | Full e2e tests |
| `make deploy` | `deploy.sh` | Deploy stack |
| `make security` | `security-audit.sh` | Security scan |
| `make monitor` | `docker compose up observability` | Start monitoring |
| `make dev` | `docker compose up dev` | Start dev stack |
| `make clean` | `rm -rf dist .tmp` | Clean artifacts |

**Usage:**
```bash
# Quick validation before commit
make test

# Full test suite
make test-e2e

# Start observability
make monitor
```

---

### Pre-commit Hook Integration

**Configuration:** `.pre-commit-config.yaml`

**Hooks:**
- `trailing-whitespace` - Remove trailing whitespace
- `end-of-file-fixer` - Ensure newline at EOF
- `check-yaml` - YAML syntax
- `check-json` - JSON syntax
- `check-toml` - TOML syntax
- `check-added-large-files` - Prevent large commits
- `nixfmt` - Nix code formatting
- `shellcheck` - Shell script linting
- `black` - Python code formatting

**Workflow Integration:**
- CI workflow runs `pre-commit run --all-files`
- Enforces code quality gates

---

### GitHub Actions Integration

**Script Reuse Pattern:**

```yaml
# In workflow: e2e-validation.yml
- name: Validate configurations
  run: |
    ./scripts/validate-configs.sh

- name: Health check service
  run: |
    ./scripts/health-check.sh http://localhost:9090/-/healthy 5 5 "Prometheus"
```

**Benefits:**
- Scripts tested locally before CI
- Consistent behavior (local == CI)
- Easier debugging

**Artifact Flow:**
```
Local Development
    ↓
Makefile (make test)
    ↓
Scripts (validate-configs.sh)
    ↓
Git Commit
    ↓
Pre-commit Hooks
    ↓
Git Push
    ↓
GitHub Actions (ci.yml)
    ↓
Same Scripts (validate-configs.sh)
    ↓
SARIF/JSON Reports
    ↓
GitHub Security Tab
```

---

## Dependency Graph

### Tool Dependencies

```
Nix (Package Manager)
├── Provides: pixi binary
├── Provides: docker binary
├── Provides: Development shells
└── Manages: System dependencies

Pixi (Environment Manager)
├── Requires: Nix (for installation)
├── Provides: Python environments
├── Provides: ROS2 packages
└── Manages: Conda packages

Docker (Container Runtime)
├── Requires: Nix (for installation) or native
├── Provides: Service orchestration
├── Provides: Isolation
└── Manages: Containers

npm/pnpm (JS Package Manager)
├── Requires: Node.js (from Nix or native)
├── Provides: JS dependencies
├── Provides: Build tools (SWC)
└── Manages: node_modules
```

---

### Script Dependencies

**Tier 1: Foundation Scripts**
- `bootstrap.sh` / `bootstrap.ps1` - No dependencies
- `fix-wsl-stability.sh` - Bash only
- `stable-env.sh` - Bash only

**Tier 2: Validation Scripts**
- `validate-configs.sh`
  - Requires: docker, python3, yq
- `health-check.sh`
  - Requires: curl
- `check-python-deps.sh`
  - Requires: pixi, python

**Tier 3: Initialization Scripts**
- `init-docker-networks.sh`
  - Requires: docker
- `init-multi-db.sh`
  - Requires: docker, docker compose
  - Depends on: init-docker-networks.sh
- `init-jetstream.sh`
  - Requires: docker, nats CLI
  - Depends on: init-docker-networks.sh
- `init-step-ca.sh`
  - Requires: docker
  - Depends on: init-docker-networks.sh

**Tier 4: Certificate Scripts**
- `generate-service-certs.sh`
  - Requires: step CLI, openssl
  - Depends on: init-step-ca.sh
- `rotate-certs.sh`
  - Requires: step CLI
  - Depends on: generate-service-certs.sh

**Tier 5: Deployment Scripts**
- `deploy.sh`
  - Requires: docker compose, validate-configs.sh
  - Depends on: init-* scripts, generate-service-certs.sh
- `deploy-observability.sh`
  - Requires: docker compose
  - Depends on: init-docker-networks.sh
- `install-all.sh` (Orchestrator)
  - Requires: All of the above
  - Calls: 15+ scripts in sequence

**Tier 6: Verification Scripts**
- All `verify-*.sh` scripts
  - Require: curl, docker
  - Depend on: Corresponding deploy scripts

---

### Workflow Dependencies

**Sequential Dependencies:**
```
validate-configs
    ↓
validate-nix
    ↓
validate-pixi
    ↓
integration-tests
    ↓
generate-report
```

**Parallel with Merge:**
```
flake-check ─┐
ros2-build ──┤
macos-build ─┼─→ summary
security ────┤
pre-commit ──┘
```

**Cross-Workflow:**
```
ci.yml (Pass) ─┐
e2e.yml (Pass)─┼─→ release.yml
security.yml ──┘
```

---

## Visual Diagrams

All diagrams are in Mermaid format and can be rendered in GitHub, VS Code, or via Mermaid Live Editor.

### Available Diagrams

1. **cicd-main-flow.mmd** - High-level CI/CD pipeline overview
   - Trigger events
   - Main workflows
   - Security scanning
   - Release pipeline

2. **script-execution-flow.mmd** - Detailed script dependencies
   - Bootstrap phase
   - Validation phase
   - Initialization phase
   - Deployment phase
   - Verification phase

3. **golden-paths.mmd** - Operational workflows
   - Fresh install path
   - Development workflow
   - Service deployment
   - Troubleshooting guide
   - Security audit

4. **makefile-workflow-integration.mmd** - Integration layers
   - Makefile targets
   - Script layer
   - GitHub Actions
   - Tool ecosystem

---

## Metrics and Quality Gates

### CI Performance Targets

| Workflow | Target Time | Actual (Avg) | Gate |
|----------|-------------|--------------|------|
| flake-check | <10s | 8s | Pass |
| ros2-build | <30min | 25min | Pass |
| macos-build | <20min | 15min | Pass |
| e2e-validation | <45min | 40min | Pass |
| security-scan | <15min | 12min | Pass |

### Script Performance

| Script | Target | Purpose |
|--------|--------|---------|
| bootstrap.sh | <5min | First-time setup |
| validate-configs.sh | <30s | Config validation |
| validate-e2e.sh | <10min | Full validation |
| deploy.sh | <5min | Core stack |
| install-all.sh | <30min | Complete install |

### Quality Gates

**Required for PR Merge:**
- ✅ Nix flake check passes
- ✅ ROS2 build succeeds
- ✅ Security scan: No CRITICAL
- ✅ Pre-commit hooks pass

**Recommended:**
- ⚠️ E2E validation passes
- ⚠️ Component verification passes
- ⚠️ Security scan: No HIGH

**Optional:**
- ℹ️ Documentation updated
- ℹ️ Changelog entry
- ℹ️ Test coverage maintained

---

## Operational Runbooks

### Daily Operations

**Morning Standup Checks:**
```bash
# 1. Check service health
make monitor
# Open Grafana: http://localhost:3000

# 2. Review overnight logs
docker compose logs --since 12h | grep ERROR

# 3. Check disk usage
df -h /
du -sh /nix/store
```

**Before Deployment:**
```bash
# 1. Validate configuration
make test

# 2. Run security scan
make security

# 3. Check resource requirements
./scripts/validate-resources.sh

# 4. Review pending changes
git log --oneline origin/main..HEAD
```

**After Deployment:**
```bash
# 1. Verify all services
./scripts/flexstack.sh health

# 2. Check metrics
# Open Prometheus: http://localhost:9090

# 3. Review logs
./scripts/flexstack.sh logs | grep ERROR

# 4. Run smoke tests
make test-e2e
```

---

### Weekly Maintenance

**Monday Morning:**
```bash
# 1. Security scan (automated)
# GitHub Actions runs security.yml

# 2. Check for updates
nix flake update
pixi update

# 3. Review vulnerabilities
cat reports/trivy-results.json | jq '.Results[].Vulnerabilities[] | select(.Severity == "CRITICAL")'

# 4. Rotate certificates (if needed)
./scripts/rotate-certs.sh
```

---

### Incident Response

**Service Down:**
```bash
# 1. Identify failed service
./scripts/flexstack.sh status

# 2. Check logs
./scripts/flexstack.sh logs [service]

# 3. Check health endpoint
./scripts/health-check.sh http://localhost:PORT/health 3 5 "Service"

# 4. Restart service
docker compose restart [service]

# 5. Verify recovery
./scripts/verify-[component].sh
```

**Certificate Expiry:**
```bash
# 1. Check expiry dates
./scripts/verify-mtls-setup.sh

# 2. Backup existing certs
cp -r secrets/ secrets.backup.$(date +%Y%m%d)

# 3. Rotate certificates
./scripts/rotate-certs.sh

# 4. Verify new certs
./scripts/verify-mtls-setup.sh

# 5. Restart services
docker compose restart
```

---

## Best Practices

### Script Development

1. **Idempotency:** Scripts should be safe to run multiple times
2. **Error Handling:** Use `set -e`, check exit codes
3. **Logging:** Color-coded output (INFO, WARN, ERROR)
4. **Health Checks:** Always verify after changes
5. **Cleanup:** Remove temporary files on exit

### Workflow Development

1. **Concurrency:** Use `concurrency` to cancel outdated runs
2. **Caching:** Use `magic-nix-cache-action` for Nix
3. **Artifacts:** Upload reports and logs
4. **Summary:** Generate `$GITHUB_STEP_SUMMARY`
5. **Security:** Use SARIF for vulnerability reports

### Deployment

1. **Validation First:** Always run `validate-configs.sh`
2. **Resource Check:** Ensure adequate RAM/disk with `validate-resources.sh`
3. **Incremental:** Deploy in phases (core → observability → edge → AI)
4. **Health Checks:** Verify each phase before proceeding
5. **Rollback Plan:** Keep previous deployment artifacts

---

## Troubleshooting Guide

### Common CI Failures

**Nix Flake Check Failed:**
```
Error: error: flake output attribute 'x' is not a derivation
```
**Solution:**
- Check `flake.nix` syntax
- Verify all outputs are valid
- Run locally: `nix flake check`

**ROS2 Build Failed:**
```
Error: colcon: command not found
```
**Solution:**
- Verify Pixi environment
- Check `pixi.toml` for `colcon` package
- Regenerate lock file: `pixi install`

**Security Scan Failed:**
```
Error: CRITICAL vulnerabilities found
```
**Solution:**
- Review `trivy-results.sarif`
- Update vulnerable packages
- Add exceptions if false positive (`.trivyignore`)

### Common Script Failures

**Docker Compose Not Found:**
```
Error: docker compose: command not found
```
**Solution:**
- Install Docker Compose v2
- Check PATH includes Docker binaries
- Verify with: `docker compose version`

**Permission Denied:**
```
Error: permission denied while trying to connect to Docker daemon
```
**Solution:**
- Add user to docker group: `sudo usermod -aG docker $USER`
- Restart session
- Or use sudo for Docker commands

**Port Already in Use:**
```
Error: bind: address already in use
```
**Solution:**
- Find process: `lsof -i :PORT` or `netstat -tulpn | grep PORT`
- Stop conflicting service
- Or change port in compose file

---

## Future Enhancements

### Planned Improvements

1. **GitOps:** Full Argo CD integration for declarative deployments
2. **Canary Deployments:** Progressive rollouts with Argo Rollouts
3. **Multi-Cluster:** Support for distributed deployments
4. **Observability:** Enhanced tracing with Jaeger/Tempo
5. **Self-Healing:** Automated recovery from common failures

### Roadmap

**Q1 2026:**
- [ ] Complete Argo CD migration
- [ ] Implement canary deployments
- [ ] Enhanced monitoring dashboards

**Q2 2026:**
- [ ] Multi-cluster support
- [ ] Disaster recovery automation
- [ ] Performance optimization

---

## References

### Documentation
- **Phase 4:** [PHASE4_STATUS.md](PHASE4_STATUS.md) - Script contract docs
- **Troubleshooting:** [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
- **Getting Started:** [GETTING_STARTED.md](GETTING_STARTED.md)

### Workflows
- `.github/workflows/` - All 27 workflows
- `Makefile` - Convenience targets
- `scripts/` - 57 automation scripts

### Diagrams
- `docs/graphs/cicd-main-flow.mmd` - CI/CD overview
- `docs/graphs/script-execution-flow.mmd` - Script dependencies
- `docs/graphs/golden-paths.mmd` - Operational workflows
- `docs/graphs/makefile-workflow-integration.mmd` - Integration layers

---

**Document Version:** 1.0
**Phase 5:** COMPLETE ✅
**Coverage:** 27 workflows, 57 scripts, 5 golden paths documented
