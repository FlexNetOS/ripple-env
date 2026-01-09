# Inference Setup Guide

**Status**: P1 (High Priority)
**Date**: 2026-01-09
**Spec Reference**: BUILDKIT_STARTER_SPEC.md §1.12 (Inference Plane)

## Overview

This guide covers setting up the inference plane for the FlexStack agentic OS, including:
- LocalAI for local model serving
- MOE (Multi-model fan-out) policy configuration
- Cloud provider integration
- Model downloads and verification

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Inference Plane                          │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │  LocalAI    │  │  MOE Policy │  │   Cloud Providers   │ │
│  │  (Primary)  │  │  (Reducer)  │  │   (Fallback)        │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
│         │                │                    │             │
│         ▼                ▼                    ▼             │
│  ┌─────────────────────────────────────────────────────────┐│
│  │              TensorZero Gateway                         ││
│  │         (Routing, Metrics, Evaluation)                  ││
│  └─────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────┘
```

## Quick Start

### 1. Start LocalAI Service

```bash
# Using Docker Compose (recommended)
docker compose -f docker-compose.inference.yml up -d

# Or using the Nix wrapper
nix develop .#full
localai start
```

### 2. Download Required Models

```bash
# Download all required GGUF models (minimum 5 for MOE)
./scripts/download-models.sh

# Verify downloads
./scripts/download-models.sh --verify

# Download optional models too
./scripts/download-models.sh --all
```

### 3. Verify Setup

```bash
# Check LocalAI status
curl http://localhost:8080/readyz

# List available models
curl http://localhost:8080/v1/models

# Test completion
curl http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "model": "gemma-3n-E2B-it",
    "messages": [{"role": "user", "content": "Hello!"}]
  }'
```

## Local Tier Configuration

### Required Models (Minimum 5 for MOE)

| Model | File | Size | Specialization |
|-------|------|------|----------------|
| gemma-3n-E2B-it | gemma-3n-E2B-it-UD-Q4_K_XL.gguf | ~2GB | General |
| Phi-4-mini-reasoning | Phi-4-mini-reasoning-UD-Q4_K_XL.gguf | ~2GB | Reasoning |
| DeepSeek-R1-Distill-Qwen-1.5B | DeepSeek-R1-Distill-Qwen-1.5B-Q8_0.gguf | ~1.5GB | Code |
| gemma-3-4b-it-qat | gemma-3-4b-it-qat-UD-Q4_K_XL.gguf | ~2.5GB | General |
| Qwen3-4B | Qwen3-4B-Q4_K_M.gguf | ~2.5GB | Code |

### Optional Models

| Model | File | Size | Specialization |
|-------|------|------|----------------|
| gemma-3-1b-it | gemma-3-1b-it-BF16.gguf | ~2GB | Fast general |
| Qwen3-0.6B | Qwen3-0.6B-BF16.gguf | ~1.2GB | Ultra-fast |

### Model Directory

Models are stored in: `./data/localai/models/`

```bash
# Check model directory
ls -la ./data/localai/models/

# Total disk space required
# - Required models: ~15GB
# - All models: ~20GB
```

## Cloud Tier Configuration

### Supported Providers

| Provider | Priority | Type | Configuration |
|----------|----------|------|---------------|
| Claude Code | 1 (highest) | CLI | API key required |
| Codex | 2 | CLI | OpenAI API key |
| GitHub Copilot | 3 | CLI | GitHub auth |
| OpenRouter | 4 (fallback) | API | API key required |

### API Key Configuration

Create or update `.env`:

```bash
# Claude Code / Anthropic
export ANTHROPIC_API_KEY=sk-ant-...

# OpenAI / Codex
export OPENAI_API_KEY=sk-...

# OpenRouter (fallback)
export OPENROUTER_API_KEY=sk-or-...

# GitHub Copilot
# Authenticate via: gh auth login
```

### CLI Tool Installation

```bash
# Claude Code CLI
# Already available in Claude Code sessions

# Codex CLI
npm install -g @openai/codex

# GitHub Copilot CLI
gh extension install github/gh-copilot
```

## MOE Policy Configuration

The MOE (Multi-model fan-out) policy is configured in:
`manifests/distributed/inference_policy.yaml`

### Key Settings

```yaml
inference_policy:
  name: flexstack-moe
  version: "1.0"

  local_tier:
    enabled: true
    backend: localai
    timeout_ms: 30000
    min_models: 5  # Minimum for MOE consensus

  cloud_tier:
    enabled: true
    timeout_ms: 60000
    fallback: true
    min_providers: 2

  reducer:
    strategy: weighted_consensus
    min_agreement: 0.6
    policy_gate: opa://inference/merge
```

### Reducer Strategies

| Strategy | Description | Use Case |
|----------|-------------|----------|
| `weighted_consensus` | Weight votes by model confidence | Default, most reliable |
| `majority_vote` | Simple majority wins | Fast, less nuanced |
| `highest_confidence` | Pick highest confidence response | Single best answer |
| `ensemble` | Combine all responses | Complex analysis |

## TensorZero Integration

TensorZero provides the LLMOps gateway for routing and metrics.

### Start TensorZero

```bash
docker compose -f docker-compose.llmops.yml up -d tensorzero tensorzero-clickhouse
```

### Configuration

TensorZero config: `manifests/llmops/tensorzero.toml`

```toml
[gateway]
host = "0.0.0.0"
port = 3000

[observability]
clickhouse_url = "http://tensorzero-clickhouse:8123"

[models.localai]
provider = "openai_compatible"
base_url = "http://localai:8080/v1"
```

### Endpoints

- Gateway: http://localhost:3030
- UI: http://localhost:3031
- Metrics: ClickHouse on port 8123

## Verification Checklist

```bash
# 1. LocalAI running
curl -s http://localhost:8080/readyz | jq

# 2. Models loaded (minimum 5)
curl -s http://localhost:8080/v1/models | jq '.data | length'

# 3. Test local inference
curl -s http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model":"gemma-3n-E2B-it","messages":[{"role":"user","content":"Say hello"}]}' | jq

# 4. TensorZero gateway (if enabled)
curl -s http://localhost:3030/health

# 5. Cloud tier (verify API keys set)
echo "ANTHROPIC_API_KEY: ${ANTHROPIC_API_KEY:+SET}"
echo "OPENAI_API_KEY: ${OPENAI_API_KEY:+SET}"
```

## Troubleshooting

### LocalAI Won't Start

```bash
# Check logs
docker compose -f docker-compose.inference.yml logs localai

# Verify port not in use
lsof -i :8080

# Check Docker resources
docker system df
```

### Models Not Loading

```bash
# Verify model files exist
ls -la ./data/localai/models/*.gguf

# Check model file integrity
./scripts/download-models.sh --verify

# Re-download corrupted models
./scripts/download-models.sh --clean
./scripts/download-models.sh
```

### Cloud Provider Errors

```bash
# Test Anthropic API
curl https://api.anthropic.com/v1/messages \
  -H "x-api-key: $ANTHROPIC_API_KEY" \
  -H "anthropic-version: 2023-06-01" \
  -H "content-type: application/json" \
  -d '{"model":"claude-3-haiku-20240307","max_tokens":10,"messages":[{"role":"user","content":"Hi"}]}'

# Test OpenAI API
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### MOE Consensus Failures

```bash
# Check inference policy
cat manifests/distributed/inference_policy.yaml

# Verify minimum models
curl -s http://localhost:8080/v1/models | jq '.data | length'
# Should be >= 5

# Check TensorZero logs
docker compose -f docker-compose.llmops.yml logs tensorzero
```

## Performance Tuning

### LocalAI Settings

```yaml
# docker-compose.inference.yml
environment:
  - THREADS=4              # CPU threads per request
  - CONTEXT_SIZE=2048      # Max context window
  - PARALLEL=2             # Concurrent requests
  - F16=true               # Use FP16 for faster inference
```

### GPU Acceleration

```yaml
# Uncomment GPU section in docker-compose.localai.yml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

### Memory Optimization

```bash
# For systems with limited RAM, use smaller models:
./scripts/download-models.sh --list

# Download only required minimum
./scripts/download-models.sh  # Skips optional models
```

## Security Considerations

1. **API Keys**: Never commit API keys to git
2. **Network**: LocalAI should only be accessible internally
3. **Rate Limiting**: Configure TensorZero rate limits for cloud providers
4. **Audit Logging**: Enable TensorZero ClickHouse logging

## Related Documentation

- [BUILDKIT_STARTER_SPEC.md §1.12](../BUILDKIT_STARTER_SPEC.md) - Inference Plane specification
- [docker-compose.inference.yml](../docker-compose.inference.yml) - LocalAI Docker config
- [manifests/distributed/inference_policy.yaml](../manifests/distributed/inference_policy.yaml) - MOE policy
- [manifests/llmops/tensorzero.toml](../manifests/llmops/tensorzero.toml) - TensorZero config
- [scripts/download-models.sh](../scripts/download-models.sh) - Model download script
