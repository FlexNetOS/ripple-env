# vCache Semantic Prompt Caching Setup

## Overview

vCache (Vector Cache) is a semantic prompt caching layer that uses Redis for distributed cache storage and semantic embeddings for intelligent cache retrieval. This reduces latency and costs for LLM applications by caching and reusing similar prompts.

## Architecture

```
┌─────────────────────┐
│   Application       │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│   vCache Server     │
│  (Caching Logic)    │
└──────────┬──────────┘
           │
      ┌────┴───────┬──────────────┐
      ▼            ▼              ▼
┌─────────┐  ┌──────────────┐  ┌────────────┐
│  Redis  │  │  Embedding   │  │  Semantic  │
│ (Store) │  │  Service     │  │ Similarity │
└─────────┘  └──────────────┘  └────────────┘
```

## Components

### 1. Redis Cache
- **Service**: `redis`
- **Port**: 6379
- **Database**: 1 (isolated from other Redis instances)
- **Features**:
  - Persistent storage with AOF (Append-Only File)
  - LRU eviction policy (maxmemory 1GB)
  - Automatic cleanup

### 2. Embedding Service
- **Service**: `embedding-service`
- **Image**: ghcr.io/xenova/transformers.js
- **Port**: 3000
- **Model**: Xenova/all-MiniLM-L6-v2
- **Purpose**: Generate semantic embeddings for prompt similarity

### 3. vCache Server
- **Service**: `vcache`
- **Port**: 8080
- **Configuration File**: `/etc/vcache/config.yaml`
- **Purpose**: Main caching orchestrator

## Configuration

### vCache Config File

Located at `config/vcache/config.yaml`:

```yaml
vcache:
  enabled: true
  backend: redis
  redis:
    host: redis
    port: 6379
    db: 1
  semantic:
    enabled: true
    similarity_threshold: 0.92
    embedding_model: all-MiniLM-L6-v2
  ttl: 3600
  max_cache_size: 1GB
```

**Configuration Options**:

| Option | Default | Description |
|--------|---------|-------------|
| `enabled` | `true` | Enable/disable vCache |
| `backend` | `redis` | Cache backend (redis) |
| `redis.host` | `redis` | Redis hostname |
| `redis.port` | `6379` | Redis port |
| `redis.db` | `1` | Redis database number |
| `semantic.enabled` | `true` | Enable semantic caching |
| `semantic.similarity_threshold` | `0.92` | Similarity score (0-1) for cache hits |
| `semantic.embedding_model` | `all-MiniLM-L6-v2` | HuggingFace embedding model |
| `ttl` | `3600` | Cache entry time-to-live (seconds) |
| `max_cache_size` | `1GB` | Maximum cache size |

## Quick Start

### 1. Start Services

```bash
docker compose -f docker/docker-compose.caching.yml up -d
```

### 2. Verify Services

```bash
# Check service status
docker compose -f docker/docker-compose.caching.yml ps

# Check Redis connectivity
docker exec vcache-redis redis-cli ping

# Check Embedding service
curl http://localhost:3000/health

# Check vCache server
curl http://localhost:8080/health
```

### 3. View Logs

```bash
# vCache logs
docker logs -f vcache-server

# Redis logs
docker logs -f vcache-redis

# Embedding service logs
docker logs -f vcache-embedding
```

## Usage Examples

### Basic Cache Storage

```python
import requests

# Store a prompt in cache
cache_request = {
    "prompt": "Explain quantum computing",
    "response": "Quantum computers use quantum bits...",
    "model": "gpt-4",
    "metadata": {
        "tokens": 150,
        "cost": 0.45
    }
}

response = requests.post(
    "http://localhost:8080/cache/store",
    json=cache_request
)
```

### Semantic Cache Retrieval

```python
# Retrieve from cache with semantic matching
query = {
    "prompt": "What is quantum computing?",
    "threshold": 0.92
}

response = requests.get(
    "http://localhost:8080/cache/retrieve",
    json=query
)

if response.status_code == 200:
    cached = response.json()
    print(f"Cache hit! Similar to: {cached['original_prompt']}")
    print(f"Similarity score: {cached['similarity']}")
else:
    print("No matching cache entry found")
```

### Cache Statistics

```python
# Get cache statistics
response = requests.get("http://localhost:8080/cache/stats")
stats = response.json()

print(f"Total entries: {stats['total_entries']}")
print(f"Cache hit rate: {stats['hit_rate']:.2%}")
print(f"Memory used: {stats['memory_used']}")
```

## Performance Tuning

### Similarity Threshold Adjustment

**Higher threshold (0.95-1.0)**: Stricter matching, fewer false positives
```yaml
semantic:
  similarity_threshold: 0.98
```

**Lower threshold (0.85-0.90)**: Broader matching, higher cache hit rate
```yaml
semantic:
  similarity_threshold: 0.90
```

### Cache TTL Configuration

**Shorter TTL (600s)**: Fresh data, lower memory usage
```yaml
vcache:
  ttl: 600
```

**Longer TTL (7200s)**: More cache reuse, higher memory usage
```yaml
vcache:
  ttl: 7200
```

### Redis Memory Management

Adjust Redis memory limits in docker-compose:

```yaml
command: redis-server --maxmemory 2gb --maxmemory-policy allkeys-lru
```

## Integration with LLM Applications

### Python Integration

```python
from vcache_client import VCacheClient

# Initialize client
vcache = VCacheClient(url="http://localhost:8080")

# Check cache before API call
cached_response = vcache.get(prompt)

if cached_response:
    print(f"Using cached response (saved ${cached_response['cost']} tokens)")
    response = cached_response
else:
    # Call LLM API
    response = llm_api.complete(prompt)

    # Store in cache
    vcache.set(
        prompt=prompt,
        response=response,
        metadata={
            "model": "gpt-4",
            "tokens": response.usage.total_tokens
        }
    )

return response
```

## Monitoring and Debugging

### Redis CLI Access

```bash
docker exec -it vcache-redis redis-cli

# Inside redis-cli:
> SELECT 1
> KEYS *
> INFO stats
> MEMORY STATS
```

### vCache Metrics

```bash
# Get detailed cache metrics
curl http://localhost:8080/metrics
```

### Common Issues

#### Redis Connection Failed
```bash
# Check Redis is running and accessible
docker exec vcache-redis redis-cli ping
# Expected output: PONG
```

#### Embedding Service Timeout
```bash
# Check embedding service logs
docker logs vcache-embedding

# Verify it's responding
curl http://localhost:3000/health
```

#### High Memory Usage
```bash
# Monitor Redis memory
docker exec vcache-redis redis-cli INFO memory

# Clear cache if needed
docker exec vcache-redis redis-cli FLUSHDB
```

## Production Deployment

### Environment Variables

Set these in production:

```bash
VCACHE_ENABLED=true
REDIS_HOST=redis.prod.internal
REDIS_PORT=6379
REDIS_PASSWORD=your-secure-password
EMBEDDING_SERVICE_URL=http://embedding-prod:3000
SIMILARITY_THRESHOLD=0.92
CACHE_TTL=3600
MAX_CACHE_SIZE=5GB
```

### Redis Clustering

For high-availability:

```yaml
redis:
  image: redis:7-alpine
  command: redis-server --cluster-enabled yes
  volumes:
    - redis_cluster_data:/data
```

### Monitoring Stack

Integrate with Prometheus/Grafana:

```yaml
vcache:
  environment:
    - ENABLE_METRICS=true
    - METRICS_PORT=9090
  ports:
    - "9090:9090"
```

## Cleanup

### Stop All Services

```bash
docker compose -f docker/docker-compose.caching.yml down
```

### Remove Data Volumes

```bash
docker compose -f docker/docker-compose.caching.yml down -v
```

## References

- [Redis Documentation](https://redis.io/docs/)
- [HuggingFace Models](https://huggingface.co/models)
- [vCache GitHub Repository](https://github.com/your-repo/vcache)
- [Semantic Caching Concepts](https://arxiv.org/abs/2307.00993)

## Support

For issues and questions:
- Check the logs: `docker logs vcache-server`
- Verify service health: `docker ps -a`
- Review Redis metrics: `docker exec vcache-redis redis-cli INFO`
