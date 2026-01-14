# LLMOps Documentation

**Spec Reference**: BUILDKIT_STARTER_SPEC.md §1.16 (LLMOps & Evaluation)

## Overview

This directory contains documentation for the LLMOps stack, which provides:
- **TensorZero**: LLMOps gateway for routing, metrics, and experiments
- **MLflow**: Experiment tracking and model registry
- **promptfoo**: Prompt regression testing
- **TruLens**: Runtime LLM evaluation

## Quick Start

```bash
# Start LLMOps stack
docker compose -f docker-compose.llmops.yml up -d

# Verify services
curl http://localhost:3030/health    # TensorZero Gateway
curl http://localhost:3031           # TensorZero UI
curl http://localhost:5000           # MLflow
```

## Components

| Component | Port | Purpose |
|-----------|------|---------|
| TensorZero | 3030 | LLMOps gateway |
| TensorZero UI | 3031 | Web dashboard |
| ClickHouse | 8123 | Metrics storage |
| MLflow | 5000 | Experiment tracking |

## Documentation

- [TensorZero Setup](./tensorzero.md) — Gateway configuration and usage
- [MLflow Guide](./mlflow.md) — Experiment tracking workflows
- [Evaluation Workflows](./evaluation.md) — promptfoo and TruLens usage

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    LLMOps Layer                         │
├─────────────────────────────────────────────────────────┤
│  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐ │
│  │  TensorZero   │  │    MLflow     │  │  promptfoo  │ │
│  │   (Gateway)   │  │  (Tracking)   │  │   (Tests)   │ │
│  └───────┬───────┘  └───────┬───────┘  └──────┬──────┘ │
│          │                  │                  │        │
│          ▼                  ▼                  ▼        │
│  ┌─────────────────────────────────────────────────────┐│
│  │              ClickHouse (Metrics DB)                ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
```

## Configuration Files

| File | Purpose |
|------|---------|
| `docker-compose.llmops.yml` | Docker Compose services |
| `manifests/llmops/tensorzero.toml` | TensorZero configuration |

## Integration with Inference Plane

TensorZero acts as the LLMOps gateway for the inference plane:

1. **Routing**: Routes requests to LocalAI or cloud providers
2. **Metrics**: Collects latency, token usage, satisfaction scores
3. **Experiments**: A/B testing for prompt variants
4. **Evaluation**: Integrates with promptfoo for regression tests

## Related Documentation

- [Inference Setup](../getting-started/INFERENCE_SETUP.md)
- [BUILDKIT_STARTER_SPEC.md](../../BUILDKIT_STARTER_SPEC.md)
