# Agenix secrets configuration
# This file defines which SSH keys can decrypt which secrets
#
# Usage:
#   1. Add your SSH public key to the appropriate list below
#   2. Create encrypted secrets with: agenix -e secrets/<name>.age
#   3. Re-key existing secrets after adding keys: agenix -r
#
# For more info: https://github.com/ryantm/agenix

let
  # =============================================================================
  # SSH Public Keys
  # =============================================================================
  # Add authorized SSH public keys here
  # These keys can decrypt the secrets they are assigned to

  # Developer keys (add your SSH public key here)
  # Example: developer1 = "ssh-ed25519 AAAA... user@host";
  developers = [
    # Add developer SSH public keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... developer@workstation"
  ];

  # CI/CD system keys
  # These keys are used by automated systems
  ci = [
    # Add CI system SSH public keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... github-actions"
  ];

  # Server/host keys (for NixOS deployments)
  # Get host key with: ssh-keyscan -t ed25519 hostname
  servers = [
    # Add server SSH host keys here
    # "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... root@server"
  ];

  # Combined key lists for different access levels
  allDevelopers = developers;
  allSystems = ci ++ servers;
  everyone = allDevelopers ++ allSystems;

in
{
  # =============================================================================
  # Secret Definitions
  # =============================================================================
  # Each entry maps a secret file to the keys that can decrypt it
  # Format: "secrets/<name>.age".publicKeys = [ list of authorized keys ];

  # Example secrets (uncomment and customize as needed):

  # API keys for external services
  # "openai-api-key.age".publicKeys = allDevelopers;
  # "anthropic-api-key.age".publicKeys = allDevelopers;

  # Database credentials
  # "postgres-password.age".publicKeys = everyone;

  # AGiXT configuration
  # "agixt-api-key.age".publicKeys = everyone;

  # LocalAI configuration
  # "localai-api-key.age".publicKeys = allDevelopers;

  # OAuth/OIDC secrets
  # "oauth-client-secret.age".publicKeys = everyone;
}
# =============================================================================
# Additional Environment Variables and Configuration
# =============================================================================
RUST_LOG=info
DATABASE_URL=postgres://user:pass@localhost:5432/noa_agentigos
OPENAI_API_KEY=sk-...
QDRANT_URL=http://localhost:6333
CUDA_VISIBLE_DEVICES=0
INFERENCE_URL=http://127.0.0.1:8082
RETRIEVAL_URL=http://127.0.0.1:8083

# Multi-Runtime Inference Configuration
# Select engine: gpt2 | llama_gguf
CANDLE_MODEL=llama_gguf

# Device: cpu | cuda | cuda:N | auto
CANDLE_DEVICE=cpu

# Generation parameters
MAX_NEW_TOKENS=256
TEMPERATURE=0.7
TOP_P=0.95

# GPT-2 Configuration (when CANDLE_MODEL=gpt2)
CANDLE_TOKENIZER_PATH=./models/gpt2/tokenizer.json
CANDLE_CONFIG_PATH=./models/gpt2/config.json
CANDLE_WEIGHTS_PATH=./models/gpt2/model.safetensors
CANDLE_MAX_TOKENS=64

# LLaMA-GGUF Configuration (when CANDLE_MODEL=llama_gguf)
GGUF_PATH=./models/llama/Meta-Llama-3.1-8B-Instruct-Q4_K_M.gguf
TOKENIZER_JSON_PATH=./models/llama/tokenizer.json

# Advanced Crawler Settings with Concurrency Control
CRAWL_USER_AGENT=NOA-AgenticOS/1.0 (+https://localhost)
CRAWL_MAX_PAGES=80
CRAWL_SITEMAP_FALLBACK=true
CRAWL_SAME_HOST_ONLY=true
CRAWL_DEFAULT_DELAY_MS=1000
CRAWL_MAX_SITEMAP_DEPTH=3
CRAWL_GLOBAL_CONCURRENCY=8
CRAWL_HOST_CONCURRENCY=2

# Persistent Queue Configuration
AGENT_QUEUE_PATH=.data/agent-queue
AGENT_QUEUE_WORKERS=2
AGENT_QUEUE_RETRY_LIMIT=3
AGENT_QUEUE_RETRY_BACKOFF_MS=5000

# Autonomous Agent Configuration
AGENT_AUTONOMY_ENABLED=false
AGENT_INGEST_SOURCES=https://www.rust-lang.org/sitemap.xml,https://qdrant.tech/documentation/
AGENT_INGEST_INTERVAL_SECS=1800
AGENT_NAMESPACE=auto
AGENT_MAX_CHUNK_CHARS=2000

# ===== FASTEMBED-RS NATIVE EMBEDDINGS WITH ZERO-DOWNTIME MIGRATION =====

# Embeddings Configuration (Retrieval defaults to fastembed-rs; Agent can embed if preferred)
EMBED_SOURCE=fastembed_rs            # fastembed_rs | localai (Retrieval migration uses this)
EMBED_MODEL=AllMiniLML6V2            # fastembed-rs model id (see list_supported_models)
EMBED_BATCH=64                       # Batch size for embedding generation

# Qdrant Configuration with Alias Strategy for Zero-Downtime Migrations
QDRANT_URL=http://127.0.0.1:6333
QDRANT_COLLECTION_ALIAS=agent_docs   # Stable alias your app queries (never changes)
QDRANT_DISTANCE=Cosine               # Cosine | Dot | Euclid | Manhattan

# LocalAI Fallback Configuration (if using EMBED_SOURCE=localai)
LOCALAI_BASE=http://127.0.0.1:8089
LOCALAI_MODEL=text-embedding-3-small

# Migration Settings
MIGRATION_BATCH_SIZE=512             # Points per migration batch
MIGRATION_CLEANUP_OLD=false          # Keep old collections for rollback

# ===== ENHANCED BACKEND SELECTION =====

# Analytics backend with object store support
ANALYTICS_DATA_ROOT=./data/analytics

# ===== ADVANCED FEATURE FLAGS =====

# Agent Cache Configuration
AGENT_CACHE_DIR=.data/agent-cache
AGENT_CACHE_ENABLED=true

# MCP Tools Security (when mcp feature enabled)
MCP_TOOLS_ENABLED=true
MCP_ALLOW_FS_WRITE=true
MCP_MAX_FILE_SIZE=10485760
MCP_ALLOWED_HOSTS=localhost,127.0.0.1

# Database Configuration (for A/B logging and traces)
PGHOST=localhost
PGPORT=5432
PGUSER=postgres
PGPASSWORD=password
PGDATABASE=noa_agentigos

# Object Store Configuration (for analytics)
AWS_ACCESS_KEY_ID=your_access_key
AWS_SECRET_ACCESS_KEY=your_secret_key
AWS_REGION=us-east-1
AWS_S3_BUCKET=noa-analytics

# Google Cloud Storage (optional)
GOOGLE_APPLICATION_CREDENTIALS=/path/to/service-account-key.json
GCS_BUCKET=noa-analytics

# Azure Storage (optional)
AZURE_STORAGE_ACCOUNT=noaanalytics
AZURE_STORAGE_KEY=your_storage_key
AZURE_STORAGE_CONTAINER=analytics

# Arkflow Stream Configuration
ARKFLOW_ENABLED=false
ARKFLOW_POLL_INTERVAL_SECS=10
ARKFLOW_BATCH_SIZE=100

# Performance and Build Optimization
RUSTC_WRAPPER=sccache
SCCACHE_DIR=.cache/sccache
CARGO_BUILD_JOBS=8

# Development and Testing
DEV_HOT_RELOAD=true
DEV_DEBUG_LOGGING=false
DEV_MOCK_INFERENCE=false
TEST_DATABASE_URL=postgres://postgres:password@localhost/noa_test

# Security Configuration
CORS_ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
API_RATE_LIMIT=1000
SESSION_SECRET=your_session_secret_change_in_production

# Monitoring and Observability
METRICS_ENABLED=true
TRACING_ENDPOINT=http://localhost:14268/api/traces
PROMETHEUS_ENDPOINT=http://localhost:9090
GITHUB_FINE_GRAINED_PAT=11BSPRLEY0jPP0RZoXcMUU_Th0aWANGia9r3DCE5hYB9SHBZFhFv4kBKVf3eKKdveYUCL2Q2KKcwR87Urv
