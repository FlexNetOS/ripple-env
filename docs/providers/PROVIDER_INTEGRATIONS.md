# Provider Integrations

**Status:** Complete
**Last Updated:** 2026-01-14
**Purpose:** Document integration status for specialized providers

---

## Overview

This document details the integration status and configuration for specialized service providers in the FlexStack environment.

---

## Holochain Integration

### Status: Available (Optional)

**Compose File:** `docker/docker-compose.holochain.yml`

### Services

| Service | Image | Port | Purpose |
|---------|-------|------|---------|
| holochain-bootstrap | `holochain/bootstrap-server:0.6.0` | 8888 | DHT bootstrap server |
| holochain-conductor | `holochain/holochain:0.6.0` | 8889 | Holochain conductor |

### Configuration

```yaml
environment:
  HOLOCHAIN_BOOTSTRAP_URL: "http://holochain-bootstrap:8888"
  HOLOCHAIN_SIGNAL_URL: "ws://holochain-bootstrap:8888"
  RIPPLE_DATA: /data
```

### Usage

```bash
# Start Holochain services
docker network create agentic-network 2>/dev/null || true
docker compose -f docker/docker-compose.holochain.yml up -d

# Verify health
curl http://localhost:8888/health
curl http://localhost:8889/health
```

### hApp Development

hApps (Holochain applications) are stored in `manifests/holochain/`:

```
manifests/holochain/
├── dna/           # DNA definitions
├── zomes/         # Zome code (Rust)
└── workdir/       # Working directory
```

### Integration Points

- **Agent Coordination:** Distributed agent state via DHT
- **Data Persistence:** Immutable, content-addressed storage
- **P2P Networking:** Direct peer-to-peer communication

---

## MindsDB Integration

### Status: Available (Optional)

**Compose File:** `docker/docker-compose.data.yml`

### Services

| Service | Image | Ports | Purpose |
|---------|-------|-------|---------|
| mindsdb | `mindsdb/mindsdb:25.14.0` | 47334, 47335, 47336 | ML database |
| mindsdb-db | `postgres:17.2-alpine` | - | Metadata store |

### Endpoints

| Protocol | Port | Usage |
|----------|------|-------|
| HTTP API | 47334 | REST API, Web UI |
| MySQL | 47335 | MySQL protocol |
| MongoDB | 47336 | MongoDB protocol |

### Configuration

```yaml
environment:
  MINDSDB_STORAGE_DIR: /mindsdb/storage
  MINDSDB_API_KEY: ${MINDSDB_API_KEY:-changeme}
```

### Usage

```bash
# Start MindsDB
docker compose -f docker/docker-compose.data.yml up -d mindsdb mindsdb-db

# Access Web UI
open http://localhost:47334

# Connect via MySQL
mysql -h localhost -P 47335 -u mindsdb -p
```

### Example Queries

```sql
-- Create a data source connection
CREATE DATABASE postgres_data
WITH ENGINE = "postgres",
PARAMETERS = {
  "host": "postgres",
  "port": "5432",
  "user": "postgres",
  "password": "changeme",
  "database": "flexstack"
};

-- Create a predictor
CREATE PREDICTOR sales_forecast
FROM postgres_data.sales
PREDICT revenue
USING engine = 'lightwood';

-- Query predictions
SELECT * FROM sales_forecast
WHERE month = 'next';
```

### Integration Points

- **LocalAI:** Use LocalAI models for NLP tasks
- **PostgreSQL:** Connect to FlexStack database
- **Temporal:** Trigger predictions from workflows

---

## TensorZero Integration

### Status: Available (Optional)

**Compose File:** `docker/docker-compose.llmops.yml`

### Services

| Service | Image | Port | Purpose |
|---------|-------|------|---------|
| tensorzero | `tensorzero/tensorzero:2025.10.6` | 3030 | LLMOps gateway |
| tensorzero-ui | `tensorzero/tensorzero-ui:2025.10.6` | 3031 | Dashboard |
| tensorzero-clickhouse | `clickhouse/clickhouse-server:24.11-alpine` | 8123, 9000 | Analytics DB |

### Configuration

```yaml
environment:
  CLICKHOUSE_URL: http://tensorzero-clickhouse:8123
  TENSORZERO_CONFIG: /etc/tensorzero/tensorzero.toml
```

### Configuration File

Location: `manifests/llmops/tensorzero.toml`

```toml
# Example configuration
[gateway]
host = "0.0.0.0"
port = 3000

[providers.localai]
type = "openai-compatible"
base_url = "http://localai:8080/v1"

[providers.openai]
type = "openai"
api_key = "${OPENAI_API_KEY}"

[routing.default]
strategy = "fallback"
providers = ["localai", "openai"]
```

### Usage

```bash
# Start TensorZero stack
docker compose -f docker/docker-compose.llmops.yml up -d

# Access gateway
curl http://localhost:3030/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "default", "messages": [{"role": "user", "content": "Hello"}]}'

# Access UI
open http://localhost:3031
```

### Features

| Feature | Description |
|---------|-------------|
| Provider Routing | Fallback, load balancing, A/B testing |
| Observability | Request logging to ClickHouse |
| Experiments | A/B test different prompts/models |
| Evaluation | Built-in eval framework |

### Integration Points

- **LocalAI:** Primary local provider
- **External APIs:** OpenAI, Anthropic fallback
- **MLflow:** Experiment tracking

---

## vCache Integration

### Status: Available (Optional)

**Compose File:** `docker/docker-compose.caching.yml`

### Services

| Service | Image | Port | Purpose |
|---------|-------|------|---------|
| vcache | `vcache:latest` (local build) | 8080 | Semantic cache |
| vcache-redis | `redis:7-alpine` | 6379 | Cache backend |
| vcache-embedding | `ghcr.io/xenova/transformers.js:latest` | 3000 | Embedding service |

### Configuration

```yaml
environment:
  REDIS_HOST: redis
  REDIS_PORT: 6379
  EMBEDDING_SERVICE_URL: http://embedding-service:3000
  SIMILARITY_THRESHOLD: 0.92
  CACHE_TTL: 3600
  MAX_CACHE_SIZE: 1GB
```

### Configuration File

Location: `config/vcache/config.yaml`

```yaml
# Cache configuration
cache:
  backend: redis
  eviction_policy: lru
  max_size: 1GB
  ttl: 3600

# Embedding configuration
embedding:
  model: Xenova/all-MiniLM-L6-v2
  dimensions: 384

# Similarity threshold
similarity:
  threshold: 0.92
  error_bound: 0.01
```

### Usage

```bash
# Start vCache stack
docker compose -f docker/docker-compose.caching.yml up -d

# Query with caching
curl http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "default", "messages": [{"role": "user", "content": "What is 2+2?"}]}'

# Check cache stats
curl http://localhost:8080/stats
```

### Features

| Feature | Description |
|---------|-------------|
| Semantic Matching | Find similar prompts via embeddings |
| Error Bounds | Guaranteed similarity threshold |
| Eviction Policies | LRU, FIFO, MRU, custom |
| Cache Stats | Hit rate, latency reduction |

### Integration Points

- **LocalAI:** Cache LocalAI responses
- **TensorZero:** Use as caching layer
- **Redis:** Shared cache with other services

---

## Integration Patterns

### Full LLMOps Stack

```bash
# Start all LLMOps services
docker compose -f docker/docker-compose.yml up -d localai
docker compose -f docker/docker-compose.llmops.yml up -d
docker compose -f docker/docker-compose.caching.yml up -d

# Traffic flow:
# Client -> vCache -> TensorZero -> LocalAI/OpenAI
```

### Data Analysis Stack

```bash
# Start data services
docker compose -f docker/docker-compose.data.yml up -d

# Traffic flow:
# SQL Query -> MindsDB -> PostgreSQL/Neo4j
# ML Prediction -> MindsDB -> LocalAI
```

### Distributed Agent Stack

```bash
# Start Holochain
docker compose -f docker/docker-compose.holochain.yml up -d

# Agent coordination via:
# - Holochain DHT for state
# - NATS for messaging
# - Temporal for workflows
```

---

## Environment Variables

| Variable | Service | Default | Purpose |
|----------|---------|---------|---------|
| `MINDSDB_API_KEY` | MindsDB | changeme | API authentication |
| `CLICKHOUSE_PASSWORD` | TensorZero | changeme | ClickHouse auth |
| `SIMILARITY_THRESHOLD` | vCache | 0.92 | Cache match threshold |
| `HOLOCHAIN_BOOTSTRAP_URL` | Holochain | - | Bootstrap server URL |

---

## Health Checks

```bash
# MindsDB
curl http://localhost:47334/api/status

# TensorZero
curl http://localhost:3030/health

# vCache
curl http://localhost:8080/health

# Holochain
curl http://localhost:8888/health
curl http://localhost:8889/health
```

---

## References

- [MindsDB Documentation](https://docs.mindsdb.com/)
- [TensorZero Documentation](https://tensorzero.com/docs)
- [vCache Project](https://github.com/vcache-project/vCache)
- [Holochain Documentation](https://developer.holochain.org/)

