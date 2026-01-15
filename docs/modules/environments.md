# Environment Taxonomy & Layering

**Layer:** L2 (Environments)
**Criticality:** HIGH
**Surface:** internal
**Runtime:** native, container, wsl
**Status:** verified
**Owner:** ripple-env maintainers

---

## Purpose

This module documents the **environment taxonomy**, **overlay strategy**, and **configuration precedence** for the ripple-env platform. It defines how different environments (dev, CUDA, AIOS, LLMOps, etc.) layer on top of each other and where secrets/config values are sourced.

---

## Environment Taxonomy

### Pixi Environments (12 total)

**Evidence:** `pixi.toml` lines 557-571

| Environment | Solve Group | Features | Platform Support | Purpose |
|-------------|-------------|----------|------------------|---------|
| `default` | default | [] | linux-64, osx-64, osx-arm64, linux-aarch64 | Base ROS2 Humble environment |
| `cuda` | cuda | [cuda] | linux-64 only | CUDA-enabled PyTorch (requires NVIDIA GPU) |
| `aios` | aios | [aios] | linux-64, osx-64, osx-arm64, linux-aarch64 | AIOS Agent OS (Python 3.10-3.11 strict) |
| `aios-cuda` | aios-cuda | [aios, aios-cuda, cuda] | linux-64 only | AIOS with GPU acceleration (vLLM) |
| `llmops` | llmops | [llmops] | linux-64, osx-64, osx-arm64, linux-aarch64 | LLMOps evaluation (TruLens, MLflow) |
| `finetuning` | finetuning | [finetuning] | linux-64, osx-64, osx-arm64, linux-aarch64 | LLM finetuning (Unsloth) |
| `finetuning-cuda` | finetuning-cuda | [finetuning, cuda] | linux-64 only | LLM finetuning with GPU |
| `caching` | caching | [caching] | linux-64, osx-64, osx-arm64, linux-aarch64 | LLM prompt/response caching (vCache) |
| `docs` | docs | [docs] | linux-64, osx-64, osx-arm64, linux-aarch64 | Documentation generation (MkDocs) |
| `vectordb-chromadb` | vectordb-chromadb | [vectordb-chromadb] | linux-64, osx-64, osx-arm64, linux-aarch64 | ChromaDB vector database |
| `vectordb-ruvector` | vectordb-ruvector | [vectordb-ruvector] | linux-64, osx-64, osx-arm64, linux-aarch64 | RuVector vector database (Rust-native) |
| `qudag` | qudag | [qudag] | linux-64, osx-64, osx-arm64, linux-aarch64 | QuDAG quantum-resistant communication |

**Activation Examples:**

```bash
# Default environment
pixi run python --version

# CUDA environment
pixi run -e cuda python train.py

# AIOS environment
pixi run -e aios aios start

# LLMOps environment
pixi run -e llmops mlflow ui

# Finetuning with CUDA
pixi run -e finetuning-cuda python finetune.py

# Vector database A/B testing
pixi run -e vectordb-chromadb python test_chromadb.py
pixi run -e vectordb-ruvector python test_ruvector.py
```

---

## Channel Strategy

**Evidence:** `pixi.toml` lines 1-15

### Base Channels (Priority Order)

```toml
channels = ["robostack-humble", "conda-forge"]
```

1. **robostack-humble** (highest priority)
   - ROS2 Humble packages
   - Takes precedence for ROS compatibility

2. **conda-forge** (lowest priority)
   - Community packages for general dependencies

### CUDA Channels (Priority Order)

**Evidence:** `pixi.toml` lines 149-161

```toml
[feature.cuda]
channels = ["pytorch", "nvidia", "conda-forge"]
```

1. **pytorch** (highest priority)
   - Official CUDA-linked PyTorch builds

2. **nvidia** (medium priority)
   - CUDA runtime and cuDNN libraries

3. **conda-forge** (lowest priority)
   - Supporting packages

**Critical Constraint:** Do not combine `robostack-humble` and CUDA in the same solve-group due to incompatible Python/CUDA expectations.

---

## Package Version Coupling

**Evidence:** `pixi.toml` lines 133-148

### PyTorch Stack Coupling

```toml
pytorch = ">=2.5,<2.6"
torchvision = ">=0.20,<0.21"
torchaudio = ">=2.5,<2.6"
```

**Version Coupling Table:**
| PyTorch | torchvision | torchaudio | CUDA |
|---------|-------------|------------|------|
| 2.5.x | 0.20.x | 2.5.x | 12.4 |
| 2.4.x | 0.19.x | 2.4.x | 12.1 |
| 2.3.x | 0.18.x | 2.3.x | 11.8 |

**Upgrade Command:**

```bash
./scripts/upgrade-python-deps.sh pytorch <version>
./scripts/check-python-deps.sh
```

### AIOS Strict Pinning

**Evidence:** `pixi.toml` lines 175-233

**Critical Constraints:**
- Python **3.10-3.11 ONLY** (no 3.12+ support - APIs removed)
- `pydantic==2.7.0` (exact version)
- `numpy==1.24.3` (exact version)
- `click==8.1.7` (exact version)

**Rationale:** AIOS Agent OS requires strict dependency pinning for compatibility with agent kernel and Cerebrum SDK.

---

## Environment Variable Registry

### Precedence Order

**Evidence:** `.env.example` lines 224-227

1. **Exported environment variables** (highest priority)
2. **`.env` file** in current directory
3. **Default values** in docker-compose files (lowest priority)

### Environment Files

**Evidence:** Glob pattern `**/.env*`

| File | Purpose | Commit Status | Evidence |
|------|---------|---------------|----------|
| `.env.example` | Template with all variables | Committed | 237 lines |
| `.env.state.example` | State/storage layer template | Committed | 166 lines |
| `.env.agixt.example` | AGiXT specific config | Committed | N/A |
| `.env.qudag.example` | QuDAG specific config | Committed | N/A |
| `.env` | Active configuration | **Never commit** | User-created |
| `.envrc` | Direnv auto-activation | Committed | 142 lines |
| `.envrc.local` | Local direnv overrides | **Never commit** | Optional |

### Core Environment Variables

**Evidence:** `scripts/env-vars.sh`, `scripts/stable-env.sh`, `.env.example`

#### Nix/Build Configuration

```bash
# From scripts/stable-env.sh (lines 7-16)
NIX_BUILD_CORES=2                      # WSL-safe parallel jobs
NIX_MAX_JOBS=2                         # Max concurrent Nix builds
CMAKE_BUILD_PARALLEL_LEVEL=2           # CMake parallelism

# Memory optimization (WSL stability)
MALLOC_ARENA_MAX=2
MALLOC_MMAP_THRESHOLD_=131072
MALLOC_TRIM_THRESHOLD_=131072
MALLOC_TOP_PAD_=131072
```

#### Editor Configuration

```bash
# From scripts/env-vars.sh (lines 5-6)
EDITOR=${EDITOR:-hx}                   # Default: Helix
VISUAL=${VISUAL:-hx}
```

#### LocalAI Configuration

```bash
# From scripts/env-vars.sh (line 10)
LOCALAI_MODELS_PATH=${LOCALAI_MODELS_PATH:-$HOME/.local/share/localai/models}
```

#### AIOS Configuration

```bash
# From scripts/env-vars.sh (lines 13-14)
AIOS_DIR=${AIOS_DIR:-$HOME/.local/share/aios}
AIOS_PORT=${AIOS_PORT:-8000}
```

#### PromptCache Configuration

```bash
# From scripts/env-vars.sh (lines 17-18)
PROMPTCACHE_DIR=${PROMPTCACHE_DIR:-$HOME/.local/share/prompt-cache}
PROMPTCACHE_PORT=${PROMPTCACHE_PORT:-8080}
```

#### Direnv Configuration

```bash
# From .envrc (lines 52-59)
RIPPLE_ENV_ROOT=$(pwd)
RIPPLE_ENV_CACHE=$HOME/.cache/ripple-env
RIPPLE_ENV_CONFIG=$HOME/.config/ripple-env
HOME_MANAGER_CONFIG=$HOME/.config/home-manager/home.nix
```

---

## Service Credentials

**Evidence:** `.env.example` lines 50-236

### Identity & Access Management

```bash
# Keycloak (lines 54-62)
KEYCLOAK_ADMIN=${KEYCLOAK_ADMIN:-admin}
KEYCLOAK_ADMIN_PASSWORD=${KEYCLOAK_ADMIN_PASSWORD:-changeme}
KEYCLOAK_DB_USER=${KEYCLOAK_DB_USER:-keycloak}
KEYCLOAK_DB_PASSWORD=${KEYCLOAK_DB_PASSWORD:-changeme}

# Vaultwarden (lines 64-66)
VAULTWARDEN_ADMIN_TOKEN=${VAULTWARDEN_ADMIN_TOKEN:-changeme}
```

### LLMOps Stack

```bash
# TensorZero ClickHouse (lines 73-75)
CLICKHOUSE_USER=${CLICKHOUSE_USER:-tensorzero}
CLICKHOUSE_PASSWORD=${CLICKHOUSE_PASSWORD:-changeme}

# MLflow PostgreSQL (lines 77-80)
MLFLOW_DB_USER=${MLFLOW_DB_USER:-mlflow}
MLFLOW_DB_PASSWORD=${MLFLOW_DB_PASSWORD:-changeme}
```

### Automation Services

```bash
# n8n (lines 87-93)
N8N_BASIC_AUTH_USER=${N8N_BASIC_AUTH_USER:-admin}
N8N_BASIC_AUTH_PASSWORD=${N8N_BASIC_AUTH_PASSWORD:-changeme}
N8N_DB_USER=${N8N_DB_USER:-n8n}
N8N_DB_PASSWORD=${N8N_DB_PASSWORD:-changeme}
```

### Observability Stack

```bash
# Grafana (lines 100-101)
GRAFANA_ADMIN_USER=${GRAFANA_ADMIN_USER:-admin}
GRAFANA_ADMIN_PASSWORD=${GRAFANA_ADMIN_PASSWORD:-changeme}

# Umami Analytics (lines 104-110)
UMAMI_DB_USER=${UMAMI_DB_USER:-umami}
UMAMI_DB_PASSWORD=${UMAMI_DB_PASSWORD:-changeme}
UMAMI_APP_SECRET=${UMAMI_APP_SECRET:-changeme}
```

### Messaging & Workflow

```bash
# Temporal (lines 120-122)
TEMPORAL_DB_USER=${TEMPORAL_DB_USER:-temporal}
TEMPORAL_DB_PASSWORD=${TEMPORAL_DB_PASSWORD:-changeme}
```

### Edge Gateway

```bash
# Kong (lines 129-134)
KONG_DB_USER=${KONG_DB_USER:-kong}
KONG_DB_PASSWORD=${KONG_DB_PASSWORD:-changeme}
```

### UI Services

```bash
# Lobe Chat (lines 141-151)
LOBE_DB_USER=${LOBE_DB_USER:-lobe}
LOBE_DB_PASSWORD=${LOBE_DB_PASSWORD:-changeme}
LOBE_MINIO_ROOT_USER=${LOBE_MINIO_ROOT_USER:-changeme}
LOBE_NEXT_AUTH_SECRET=${LOBE_NEXT_AUTH_SECRET:-changeme}

# OpenAI-compatible API (lines 159-160)
OPENAI_API_KEY=${OPENAI_API_KEY:-sk-dummy-key-for-localai}
OPENAI_BASE_URL=${OPENAI_BASE_URL:-http://localai:8080/v1}
```

### AGiXT Stack

```bash
# AGiXT (lines 179-186)
AGIXT_API_KEY=${AGIXT_API_KEY:-changeme}
AGIXT_POSTGRES_PASSWORD=${AGIXT_POSTGRES_PASSWORD:-changeme}
AGIXT_MINIO_ROOT_USER=${AGIXT_MINIO_ROOT_USER:-changeme}
```

### Data Services

```bash
# MindsDB (lines 201-209)
MINDSDB_API_KEY=${MINDSDB_API_KEY:-changeme}
MINDSDB_DB_USER=${MINDSDB_DB_USER:-mindsdb}
MINDSDB_DB_PASSWORD=${MINDSDB_DB_PASSWORD:-changeme}
```

---

## State & Storage Configuration

**Evidence:** `.env.state.example` lines 1-166

### Redis Configuration

```bash
# Lines 18-32
REDIS_URL=redis://localhost:6379
REDIS_HOST=localhost
REDIS_PORT=6379
REDIS_DB=0
REDIS_PASSWORD=

# Cache TTL settings (in seconds)
REDIS_CACHE_TTL=3600                    # 1 hour - default cache
REDIS_SESSION_TTL=86400                 # 24 hours - session data
REDIS_PROMPT_CACHE_TTL=7200             # 2 hours - LLM prompt cache
REDIS_RATE_LIMIT_TTL=60                 # 1 minute - rate limit counters

# Memory limits
REDIS_MAX_MEMORY=2gb
REDIS_MAX_MEMORY_POLICY=allkeys-lru
```

### MinIO Configuration

```bash
# Lines 39-55
MINIO_ROOT_USER=minioadmin
MINIO_ROOT_PASSWORD=minioadmin          # CHANGE IN PRODUCTION!
MINIO_ENDPOINT=http://localhost:9000
MINIO_CONSOLE_URL=http://localhost:9001

# Bucket names
MINIO_BUCKET_ARTIFACTS=aria-artifacts
MINIO_BUCKET_MODELS=aria-models
MINIO_BUCKET_LOGS=aria-logs
MINIO_BUCKET_VECTORS=aria-vectors

# AWS SDK compatibility
AWS_ACCESS_KEY_ID=${MINIO_ROOT_USER}
AWS_SECRET_ACCESS_KEY=${MINIO_ROOT_PASSWORD}
AWS_ENDPOINT_URL=${MINIO_ENDPOINT}
```

### Vector Database Selection

```bash
# Lines 60-67
# Feature flag for vector store selection
# Options: pgvector | ruvector | chromadb | both
VECTOR_STORE=ruvector
```

### RuVector Configuration

```bash
# Lines 80-108
# Server endpoints (forward-looking placeholders)
RUVECTOR_ENDPOINT=http://localhost:8000
RUVECTOR_GRPC_ENDPOINT=localhost:8001
RUVECTOR_METRICS_ENDPOINT=http://localhost:8002

# Index configuration
RUVECTOR_EMBEDDING_DIM=384
RUVECTOR_INDEX_TYPE=hnsw-gnn
RUVECTOR_HNSW_M=16
RUVECTOR_HNSW_EF_CONSTRUCT=200
RUVECTOR_HNSW_EF_SEARCH=100

# GNN self-learning
RUVECTOR_GNN_ENABLED=true
RUVECTOR_GNN_LAYERS=2
RUVECTOR_GNN_HIDDEN_DIM=128

# Compression
RUVECTOR_COMPRESSION_ENABLED=true
RUVECTOR_COMPRESSION_RATIO=8

# Cypher query support
RUVECTOR_CYPHER_ENABLED=true
RUVECTOR_MAX_QUERY_DEPTH=10
```

---

## Overlay Strategy

### Layer Diagram

```
┌─────────────────────────────────────────────────┐
│ L5: User Overrides (.env, exported vars)       │ ← Highest Priority
├─────────────────────────────────────────────────┤
│ L4: Stack-Specific (.env.state, .env.agixt)   │
├─────────────────────────────────────────────────┤
│ L3: Platform-Specific (WSL, native, container) │
├─────────────────────────────────────────────────┤
│ L2: Environment Features (cuda, aios, llmops)  │
├─────────────────────────────────────────────────┤
│ L1: Base Environment (pixi.toml default)       │ ← Lowest Priority
└─────────────────────────────────────────────────┘
```

### Overlay Resolution Example

**Scenario:** User wants AIOS with CUDA on WSL2

**Resolution Order:**

1. **Base (L1):** `pixi.toml` default environment
   - Python 3.11, ROS2 Humble, base packages

2. **Features (L2):** AIOS + CUDA features activated
   - `pixi run -e aios-cuda ...`
   - Overrides Python to 3.10-3.11
   - Adds pydantic==2.7.0, numpy==1.24.3 (strict pins)
   - Adds CUDA toolkit from pytorch/nvidia channels

3. **Platform (L3):** WSL2-specific adjustments
   - Source `scripts/stable-env.sh`
   - `NIX_BUILD_CORES=2`, `NIX_MAX_JOBS=2`
   - Memory optimization settings

4. **Stack Config (L4):** Load `.env.state` for vector DB config
   - `VECTOR_STORE=ruvector`
   - RuVector GNN settings

5. **User Overrides (L5):** Export custom values
   - `export AIOS_PORT=9000`
   - `export RUVECTOR_GNN_ENABLED=false`

**Final Environment:**
- Python 3.10 (AIOS constraint)
- CUDA 12.4 (from pytorch channel)
- pydantic==2.7.0 (AIOS strict pin)
- NIX_BUILD_CORES=2 (WSL stability)
- AIOS_PORT=9000 (user override)
- RUVECTOR_GNN_ENABLED=false (user override)

---

## Secrets Management Strategy

**Evidence:** `.env.example` lines 213-236

### Password Generation

```bash
# General passwords
openssl rand -base64 32

# Tokens/keys
openssl rand -hex 32

# Alternative (requires pwgen)
pwgen -s 32 1
```

### Vault Integration (Recommended for Production)

**Evidence:** `.env.example` lines 220-222

```bash
# Store secrets in Vault
vault kv put secret/aria/identity keycloak_admin_password=<secure-value>
vault kv put secret/aria/databases postgres_passwords=<secure-value>
```

**Policy File:** `config/vault/policy-secrets.hcl` (referenced but not yet read)

### Production Security Checklist

**Evidence:** `.env.example` lines 229-236

- [ ] All passwords changed from defaults
- [ ] Secrets stored in Vault or equivalent
- [ ] `.env` file has restricted permissions (`chmod 600 .env`)
- [ ] `.env` file is in `.gitignore`
- [ ] Secrets are rotated regularly (90 days recommended)
- [ ] Audit logs enabled for all services

---

## Platform-Specific Overlays

### Linux (Native)

**Environment:**
- Full ROS2 support
- All 12 pixi environments available
- CUDA support (with NVIDIA GPU)

**Activation:**
```bash
nix develop
pixi run <command>
```

### macOS

**Environment:**
- Limited ROS2 runtime support
- Development tools available
- No CUDA support
- ARM64 (Apple Silicon) support

**Activation:**
```bash
nix develop
pixi run <command>
```

**Platform Detection:** `pixi.toml` line 15

```toml
platforms = ["osx-64", "osx-arm64"]
```

### Windows (WSL2)

**Environment:**
- Full Linux compatibility via NixOS-WSL
- Requires WSL stability fixes
- Memory/build parallelism limited

**Activation:**
```bash
wsl -d NixOS-Ripple
cd ~/ripple-env
source scripts/stable-env.sh  # WSL stability
pixi-safe install              # Timeout-protected
```

**WSL-Specific Settings:** `scripts/stable-env.sh` lines 7-19

```bash
NIX_BUILD_CORES=2
NIX_MAX_JOBS=2
CMAKE_BUILD_PARALLEL_LEVEL=2
MALLOC_ARENA_MAX=2
WSLENV="NIX_BUILD_CORES/w:NIX_MAX_JOBS/w"
```

---

## Unknown Configuration Paths

### Phase 3 Gaps (For Future Investigation)

1. **Vault policy files:** Referenced but not analyzed
   - `config/vault/policy-secrets.hcl`
   - Policy structure and access rules

2. **Docker-compose environment injection:**
   - How `.env` values are passed to containers
   - Override precedence in compose files

3. **Service-specific config files:**
   - Keycloak realm configuration
   - Prometheus targets
   - Grafana datasources

4. **Secret rotation procedures:**
   - Automated rotation scripts
   - Rotation schedule
   - Service restart requirements

5. **Environment-specific overrides:**
   - dev/stage/prod differentiation
   - Region-specific configs
   - Feature flags per environment

These will be addressed in:
- **Phase 4:** Script contract analysis (rotation scripts)
- **Phase 6:** Provider auth wiring (Vault, Keycloak configs)
- **Phase 7:** Runbooks (secret rotation procedures)

---

## References

**Evidence Files:**
- `pixi.toml` (572 lines)
- `.env.example` (237 lines)
- `.env.state.example` (166 lines)
- `.envrc` (142 lines)
- `scripts/env-vars.sh` (33 lines)
- `scripts/stable-env.sh` (62 lines)
- `.env.agixt.example` (exists)
- `.env.qudag.example` (exists)

**Phase 1 Deliverables:**
- `docs/CATALOG.json` — Environment catalog
- `docs/AUDIT_PROGRESS_REPORT.md` — Phase 1 report

**Phase 2 Deliverables:**
- `docs/modules/bootstrap.md` — Golden paths
- `docs/graphs/bootstrap_flow.mmd` — Bootstrap flow diagram

---

**Last Updated:** 2026-01-13 (Phase 3)
**Next Update:** Phase 4 (Script Contracts)
