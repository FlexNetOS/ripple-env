# TensorZero Setup Guide

**Component**: LLMOps Gateway
**Port**: 3030 (Gateway), 3031 (UI)

## Overview

TensorZero provides a unified gateway for LLM operations:
- Request routing to multiple model providers
- Metrics collection and storage
- A/B testing and experimentation
- Function-based prompt management

## Quick Start

```bash
# Start TensorZero stack
docker compose -f docker-compose.llmops.yml up -d tensorzero tensorzero-clickhouse tensorzero-ui

# Health check
curl http://localhost:3030/health

# Access UI
open http://localhost:3031
```

## Configuration

Configuration file: `manifests/llmops/tensorzero.toml`

### Gateway Settings

```toml
[gateway]
host = "0.0.0.0"
port = 3030

[observability]
clickhouse_url = "http://tensorzero-clickhouse:8123"
```

### Model Providers

```toml
[models.localai]
provider = "openai_compatible"
base_url = "http://localai:8080/v1"
default_model = "gemma-3n-E2B-it"

[models.localai-reasoning]
provider = "openai_compatible"
base_url = "http://localai:8080/v1"
default_model = "Phi-4-mini-reasoning"

[models.localai-code]
provider = "openai_compatible"
base_url = "http://localai:8080/v1"
default_model = "DeepSeek-R1-Distill-Qwen-1.5B"

[models.agixt]
provider = "openai_compatible"
base_url = "http://agixt:7437/v1"
api_key_env = "AGIXT_API_KEY"
```

### Functions

Functions define reusable prompt templates:

```toml
[functions.chat]
description = "General chat completion"
default_model = "localai"
variants = ["default", "reasoning"]

[functions.code]
description = "Code generation and completion"
default_model = "localai-code"
variants = ["default", "agixt"]

[functions.json_generation]
description = "Structured JSON output"
default_model = "localai"
variants = ["default"]
```

### Metrics

```toml
[metrics.latency]
type = "histogram"
description = "Request latency in milliseconds"

[metrics.tokens]
type = "counter"
description = "Total tokens processed"

[metrics.satisfaction]
type = "gauge"
description = "User satisfaction score"

[metrics.completion]
type = "counter"
description = "Completion success rate"
```

## API Usage

### Chat Completion

```bash
curl -X POST http://localhost:3030/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "function": "chat",
    "variant": "default",
    "messages": [
      {"role": "user", "content": "Hello, how are you?"}
    ]
  }'
```

### Code Generation

```bash
curl -X POST http://localhost:3030/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "function": "code",
    "messages": [
      {"role": "user", "content": "Write a Python function to calculate factorial"}
    ]
  }'
```

### JSON Generation

```bash
curl -X POST http://localhost:3030/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{
    "function": "json_generation",
    "messages": [
      {"role": "user", "content": "Extract entities from: John works at Acme Inc"}
    ],
    "response_format": {"type": "json_object"}
  }'
```

## Experimentation

### A/B Testing

TensorZero supports variant-based A/B testing:

```toml
[functions.chat.variants.default]
model = "localai"
weight = 0.8

[functions.chat.variants.reasoning]
model = "localai-reasoning"
weight = 0.2
```

### Tracking Experiments

```bash
# Query experiment results
curl http://localhost:8123 -d "
SELECT
  variant,
  avg(latency_ms) as avg_latency,
  count(*) as requests
FROM tensorzero.requests
WHERE function = 'chat'
GROUP BY variant
"
```

## Metrics & Monitoring

### ClickHouse Queries

```sql
-- Request latency by model
SELECT
  model,
  quantile(0.5)(latency_ms) as p50,
  quantile(0.95)(latency_ms) as p95,
  quantile(0.99)(latency_ms) as p99
FROM tensorzero.requests
WHERE created_at > now() - interval 1 hour
GROUP BY model;

-- Token usage by function
SELECT
  function,
  sum(input_tokens) as total_input,
  sum(output_tokens) as total_output
FROM tensorzero.requests
WHERE created_at > now() - interval 24 hour
GROUP BY function;

-- Error rate
SELECT
  model,
  countIf(status = 'error') / count(*) as error_rate
FROM tensorzero.requests
WHERE created_at > now() - interval 1 hour
GROUP BY model;
```

### Grafana Integration

TensorZero metrics can be visualized in Grafana:

1. Add ClickHouse data source
2. Import TensorZero dashboard
3. Configure alerts for latency/error thresholds

## Troubleshooting

### Gateway Not Starting

```bash
# Check logs
docker compose -f docker-compose.llmops.yml logs tensorzero

# Verify ClickHouse connection
docker compose -f docker-compose.llmops.yml exec tensorzero curl http://tensorzero-clickhouse:8123/ping
```

### Model Connection Errors

```bash
# Test LocalAI connectivity from TensorZero container
docker compose -f docker-compose.llmops.yml exec tensorzero curl http://localai:8080/readyz

# Verify network
docker network ls | grep agentic
```

### ClickHouse Issues

```bash
# Check ClickHouse health
curl http://localhost:8123/ping

# View tables
curl http://localhost:8123 -d "SHOW TABLES FROM tensorzero"
```

## Best Practices

1. **Use functions**: Define reusable prompt templates instead of raw completions
2. **Track variants**: Use A/B testing to optimize prompts
3. **Monitor metrics**: Set up alerts for latency and error rate thresholds
4. **Log requests**: Enable full request/response logging for debugging
5. **Rate limit**: Configure rate limits per model provider

## Related Documentation

- [TensorZero Official Docs](https://tensorzero.com/docs)
- [ClickHouse Documentation](https://clickhouse.com/docs)
- [Inference Setup](../getting-started/INFERENCE_SETUP.md)
