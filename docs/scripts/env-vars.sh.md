# Script Contract: env-vars.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/env-vars.sh`

---

## Purpose

Set AI/ML environment variables for ripple-env tools. Configures default editor, LocalAI models directory, AIOS Agent OS settings, and PromptCache configuration. Creates necessary directories. Previously contained in `.envrc`, extracted for explicit sourcing.

---

## Invocation

```bash
source scripts/env-vars.sh
```

**Note:** Must be sourced (not executed) to export variables to current shell.

---

## Outputs

**Standard Output:**
```
🎯 AI/ML environment variables loaded:
   EDITOR: hx
   LOCALAI_MODELS_PATH: /home/user/.local/share/localai/models
   AIOS_DIR: /home/user/.local/share/aios (port: 8000)
   PROMPTCACHE_DIR: /home/user/.local/share/prompt-cache (port: 8080)
```

---

## Side Effects

### Environment Variables Set

**Editor configuration (lines 6-7):**
```bash
EDITOR="${EDITOR:-hx}"      # Default: Helix editor
VISUAL="${VISUAL:-hx}"      # Visual editor (same)
```

**LocalAI configuration (line 10):**
```bash
LOCALAI_MODELS_PATH="${LOCALAI_MODELS_PATH:-$HOME/.local/share/localai/models}"
```

**AIOS Agent OS (lines 13-14):**
```bash
AIOS_DIR="${AIOS_DIR:-$HOME/.local/share/aios}"
AIOS_PORT="${AIOS_PORT:-8000}"
```

**PromptCache (lines 17-18):**
```bash
PROMPTCACHE_DIR="${PROMPTCACHE_DIR:-$HOME/.local/share/prompt-cache}"
PROMPTCACHE_PORT="${PROMPTCACHE_PORT:-8080}"
```

### Directories Created

**Evidence:** Lines 21-23

```bash
mkdir -p "$LOCALAI_MODELS_PATH" 2>/dev/null || true
mkdir -p "$AIOS_DIR" 2>/dev/null || true
mkdir -p "$PROMPTCACHE_DIR" 2>/dev/null || true
```

**Silent failure:** Redirects errors to `/dev/null`, uses `|| true` to prevent script exit.

---

## Safety Classification

**🟢 SAFE** - Environment variables only, creates user directories.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be sourced repeatedly. `mkdir -p` safe for existing directories.

---

## Environment Variable Details

### 1. Editor Configuration

**Evidence:** Lines 6-7

**EDITOR:** Used by git, crontab, visudo, and many CLI tools.
**VISUAL:** Used by programs expecting full-screen editor.

**Default:** Helix (`hx`) - Modern modal editor with LSP support.

**Override:**
```bash
export EDITOR=vim
source scripts/env-vars.sh  # Keeps vim
```

### 2. LocalAI Models Path

**Evidence:** Line 10

**Purpose:** Storage location for downloaded AI models (GGUF, GGML formats).

**Default:** `$HOME/.local/share/localai/models`

**Used by:**
- LocalAI service (docker-compose.localai.yml mounts this path)
- Model download scripts (fetch-localai-models.sh)

**Typical models:**
- mistral-7b-instruct-v0.1.Q4_K_M.gguf
- llama-2-7b-chat.Q4_K_M.gguf
- all-MiniLM-L6-v2.gguf (embeddings)

### 3. AIOS Agent OS

**Evidence:** Lines 13-14

**Purpose:** AIOS (Agent Operating System) data directory and service port.

**Components:**
- Agent definitions
- Task queues
- Execution logs
- State persistence

**Port:** 8000 (default) - AIOS API server.

**Integration:** Used by agentic workflows in ARIA platform.

### 4. PromptCache

**Evidence:** Lines 17-18

**Purpose:** Caching layer for LLM prompts and responses.

**Benefits:**
- Reduces API calls
- Improves response time
- Enables offline operation

**Port:** 8080 (default) - PromptCache service.

---

## ROS2 Integration (Commented)

**Evidence:** Lines 32-33

```bash
# ROS2 workspace configuration (uncomment if needed)
# layout ros2
# layout colcon
```

**Purpose:** Reserved for ROS2 workspace activation if using direnv with custom layouts.

**Note:** Currently disabled to keep script focused on AI/ML tools.

---

## Usage Patterns

### Standard Workflow

```bash
# 1. Source environment
source scripts/env-vars.sh

# 2. Download models (if needed)
./scripts/fetch-localai-models.sh mistral-7b-instruct

# 3. Start services
docker compose -f docker-compose.localai.yml up -d

# 4. Verify model path
ls -lh $LOCALAI_MODELS_PATH
```

### Custom Configuration

```bash
# Override defaults before sourcing
export LOCALAI_MODELS_PATH=/mnt/models/llm
export AIOS_PORT=9000

source scripts/env-vars.sh
```

---

## Integration

**Loaded by:**
- `scripts/stable-env.sh` (lines 31-33)
- `.envrc` (optionally)
- User shell profiles (manually)

**Used by:**
- `scripts/fetch-localai-models.sh`
- `docker-compose.localai.yml` (volume mounts)
- AIOS agent launcher
- PromptCache service

---

## Directory Structure

**After sourcing:**
```
$HOME/.local/share/
├── localai/
│   └── models/               # LLM model files (.gguf)
├── aios/                     # Agent OS data
│   ├── agents/               # Agent definitions
│   ├── tasks/                # Task queue
│   └── logs/                 # Execution logs
└── prompt-cache/             # Cached prompts
    ├── prompts/              # Prompt storage
    └── responses/            # Response cache
```

---

## References

### Source Code
- **Main script:** `scripts/env-vars.sh` (33 lines)
- **Editor config:** lines 6-7
- **LocalAI config:** line 10
- **AIOS config:** lines 13-14
- **PromptCache config:** lines 17-18
- **Directory creation:** lines 21-23
- **Output display:** lines 25-29

### Related Files
- **Loader:** `scripts/stable-env.sh` (sources this file)
- **LocalAI compose:** `docker-compose.localai.yml`
- **Model fetcher:** `scripts/fetch-localai-models.sh`
- **Alternative:** `.envrc` (direnv)

### External Resources
- [Helix Editor](https://helix-editor.com/)
- [LocalAI](https://localai.io/)
- [XDG Base Directory Specification](https://specifications.freedesktop.org/basedir-spec/basedir-spec-latest.html)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 18/60 contracts complete
