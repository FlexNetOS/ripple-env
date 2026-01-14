# AI Resource Requirements Guide

**Status**: Documentation
**Date**: 2026-01-12
**Purpose**: Comprehensive resource requirements for AI services in FlexStack

## Overview

This document provides detailed resource requirements for all AI services in the FlexStack environment. Use this guide to:
- Plan hardware requirements before deployment
- Select appropriate models for your hardware
- Optimize resource usage for development vs production
- Troubleshoot performance issues

## Quick Reference: Minimum Requirements

| Profile | RAM | Storage | GPU | Use Case |
|---------|-----|---------|-----|----------|
| **Lightweight** | 4GB | 10GB | None | Development, testing |
| **Standard** | 16GB | 50GB | None | Production (CPU inference) |
| **GPU-Accelerated** | 16GB + 8GB VRAM | 50GB | NVIDIA RTX 3060+ | Fast inference |
| **Full Stack** | 32GB + 16GB VRAM | 100GB | NVIDIA RTX 4080+ | All services, large models |

---

## GPU Requirements Matrix

### NVIDIA GPU Compatibility

| GPU | VRAM | CUDA | Supported Models | Performance Tier |
|-----|------|------|------------------|------------------|
| RTX 4090 | 24GB | 8.9 | All models, 70B quantized | Excellent |
| RTX 4080 | 16GB | 8.9 | Up to 34B quantized | Excellent |
| RTX 4070 Ti | 12GB | 8.9 | Up to 13B quantized | Very Good |
| RTX 4060 Ti | 8GB | 8.9 | Up to 7B quantized | Good |
| RTX 3090 | 24GB | 8.6 | All models, 70B quantized | Excellent |
| RTX 3080 | 10GB | 8.6 | Up to 13B quantized | Good |
| RTX 3070 | 8GB | 8.6 | Up to 7B quantized | Good |
| RTX 3060 | 12GB | 8.6 | Up to 13B quantized | Good |
| Tesla T4 | 16GB | 7.5 | Up to 13B quantized | Good (Cloud) |
| Tesla V100 | 16/32GB | 7.0 | Up to 34B quantized | Very Good (Cloud) |
| A100 | 40/80GB | 8.0 | All models | Excellent (Cloud) |

### AMD GPU Support (ROCm)

| GPU | VRAM | ROCm | Support Status |
|-----|------|------|----------------|
| RX 7900 XTX | 24GB | 5.6+ | Experimental (LocalAI) |
| RX 7900 XT | 20GB | 5.6+ | Experimental (LocalAI) |
| RX 6900 XT | 16GB | 5.6+ | Limited |

> **Note**: AMD GPU support requires ROCm-compatible LocalAI builds. NVIDIA is recommended for production.

### Apple Silicon Support

| Chip | Unified Memory | Metal | Supported Models | Notes |
|------|----------------|-------|------------------|-------|
| M1 | 8-16GB | 3 | Up to 7B | Basic inference |
| M1 Pro/Max | 16-64GB | 3 | Up to 34B | Good performance |
| M2 | 8-24GB | 3 | Up to 13B | Improved inference |
| M2 Pro/Max/Ultra | 16-192GB | 3 | Up to 70B | Excellent |
| M3 | 8-24GB | 3 | Up to 13B | Best efficiency |
| M3 Pro/Max | 18-128GB | 3 | Up to 70B | Excellent |

> **Note**: Apple Silicon uses unified memory. Model size should be < 75% of available unified memory.

---

## Model Size and VRAM Requirements

### GGUF Quantization Guide

| Quantization | Quality | Size Reduction | VRAM vs FP16 |
|--------------|---------|----------------|--------------|
| Q8_0 | Excellent | 50% | 50% |
| Q6_K | Very Good | 40% | 40% |
| Q5_K_M | Good | 35% | 35% |
| Q4_K_M | Good | 25% | 25% |
| Q4_K_S | Acceptable | 25% | 25% |
| Q3_K_M | Acceptable | 20% | 20% |
| Q2_K | Poor | 15% | 15% |

### Model VRAM Requirements (GPU Mode)

| Model Size | FP16 | Q8_0 | Q4_K_M | Q3_K_M |
|------------|------|------|--------|--------|
| 0.5B | 1GB | 0.5GB | 0.4GB | 0.3GB |
| 1.5B | 3GB | 1.5GB | 1GB | 0.8GB |
| 3B | 6GB | 3GB | 2GB | 1.5GB |
| 7B | 14GB | 7GB | 4GB | 3GB |
| 13B | 26GB | 13GB | 8GB | 6GB |
| 34B | 68GB | 34GB | 20GB | 15GB |
| 70B | 140GB | 70GB | 40GB | 30GB |

### FlexStack Default Models

| Model | Size | Quantization | Disk | VRAM (GPU) | RAM (CPU) |
|-------|------|--------------|------|------------|-----------|
| gemma-3n-E2B-it | 2B | Q4_K_XL | ~2GB | 2GB | 4GB |
| Phi-4-mini-reasoning | 3.8B | Q4_K_XL | ~2GB | 3GB | 6GB |
| DeepSeek-R1-Distill-Qwen-1.5B | 1.5B | Q8_0 | ~1.5GB | 1.5GB | 3GB |
| gemma-3-4b-it-qat | 4B | Q4_K_XL | ~2.5GB | 3GB | 6GB |
| Qwen3-4B | 4B | Q4_K_M | ~2.5GB | 3GB | 6GB |
| gemma-3-1b-it (optional) | 1B | BF16 | ~2GB | 2GB | 4GB |
| Qwen3-0.6B (optional) | 0.6B | BF16 | ~1.2GB | 1GB | 2GB |

**Total Disk Space**:
- Required models only: ~15GB
- All models: ~20GB

---

## Service Resource Requirements

### LocalAI

LocalAI provides OpenAI-compatible inference for local models.

#### CPU Mode (Default)

| Configuration | RAM | CPU Cores | Threads | Context | Throughput |
|---------------|-----|-----------|---------|---------|------------|
| Minimal | 4GB | 2 | 2 | 512 | ~5 tok/s |
| Standard | 8GB | 4 | 4 | 2048 | ~15 tok/s |
| Performance | 16GB | 8 | 8 | 4096 | ~30 tok/s |
| High | 32GB | 16 | 16 | 8192 | ~50 tok/s |

#### GPU Mode

| Configuration | RAM | VRAM | GPU Layers | Context | Throughput |
|---------------|-----|------|------------|---------|------------|
| Entry | 8GB | 4GB | 20 | 2048 | ~50 tok/s |
| Standard | 16GB | 8GB | 35 | 4096 | ~100 tok/s |
| Performance | 16GB | 12GB | All | 8192 | ~150 tok/s |
| High | 32GB | 24GB | All | 16384 | ~200 tok/s |

#### Docker Compose Resource Limits

```yaml
# CPU Mode
services:
  localai:
    deploy:
      resources:
        limits:
          memory: 8G
          cpus: '4.0'
        reservations:
          memory: 4G
          cpus: '2.0'

# GPU Mode
services:
  localai-gpu:
    deploy:
      resources:
        limits:
          memory: 16G
        reservations:
          memory: 8G
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

### AGiXT

AGiXT requires multiple containers for full functionality.

| Service | RAM Limit | RAM Reserved | CPU Limit | Storage |
|---------|-----------|--------------|-----------|---------|
| AGiXT API | 4GB | 1GB | 4 cores | 1GB |
| AGiXT Interactive | 1GB | 256MB | 1 core | 100MB |
| PostgreSQL | 2GB | 512MB | 2 cores | 10GB |
| MinIO | 1GB | 256MB | 1 core | Variable |
| **Total** | **8GB** | **2GB** | **8 cores** | **12GB+** |

#### Docker Compose Resource Limits

```yaml
services:
  agixt:
    deploy:
      resources:
        limits:
          memory: 4G
          cpus: '4.0'
        reservations:
          memory: 1G
          cpus: '1.0'

  postgres:
    deploy:
      resources:
        limits:
          memory: 2G
          cpus: '2.0'
        reservations:
          memory: 512M
```

### AIOS (Agent Operating System)

AIOS is primarily CPU-bound but can leverage GPU for inference.

| Mode | RAM | CPU | GPU | Throughput |
|------|-----|-----|-----|------------|
| CPU Only | 4GB | 4 cores | None | ~10 tok/s |
| Hybrid | 8GB | 4 cores | 4GB VRAM | ~50 tok/s |
| GPU Accelerated | 16GB | 4 cores | 8GB+ VRAM | ~100 tok/s |

#### Vector Database Requirements

| Backend | RAM | Storage | Notes |
|---------|-----|---------|-------|
| ChromaDB | 1-4GB | 1GB+ | Default, embedded |
| Qdrant | 2-8GB | 5GB+ | Production, standalone |

### vCache (Semantic Caching)

| Component | RAM | CPU | Storage |
|-----------|-----|-----|---------|
| Redis | 1-2GB | 1 core | 1GB (configured limit) |
| Embedding Service | 1GB | 2 cores | 500MB (model) |
| vCache Server | 512MB | 1 core | 100MB |
| **Total** | **3-4GB** | **4 cores** | **2GB** |

### TensorZero (LLMOps Gateway)

| Component | RAM | CPU | Storage |
|-----------|-----|-----|---------|
| TensorZero Gateway | 512MB | 1 core | 100MB |
| ClickHouse | 2-4GB | 2 cores | 10GB+ (logs) |
| **Total** | **3-5GB** | **3 cores** | **10GB+** |

### Refact AI (Self-Hosted Coding Assistant)

| Mode | RAM | VRAM | CPU | Storage |
|------|-----|------|-----|---------|
| CPU Only | 8GB | - | 4 cores | 5GB |
| GPU | 4GB | 6GB+ | 2 cores | 5GB |

---

## Deployment Profiles

### Lightweight Profile (Development/Testing)

Minimum resources for basic AI functionality.

**Requirements**:
- RAM: 4GB minimum, 8GB recommended
- CPU: 2-4 cores
- Storage: 20GB
- GPU: Not required

**Services**:
- LocalAI (CPU mode, single model)
- Redis (minimal cache)

**Docker Compose**: `docker-compose.lightweight.yml`

```bash
# Start lightweight mode
docker compose -f docker/docker-compose.lightweight.yml up -d
```

### Standard Profile (CPU Production)

Full functionality without GPU acceleration.

**Requirements**:
- RAM: 16GB minimum, 32GB recommended
- CPU: 4-8 cores
- Storage: 50GB
- GPU: Not required

**Services**:
- LocalAI (CPU mode, multiple models)
- AGiXT (full stack)
- vCache
- TensorZero

**Docker Compose**: Use default compose files

```bash
# Start standard mode
docker compose -f docker/docker-compose.localai.yml up -d
docker compose -f docker/docker-compose.agixt.yml up -d
docker compose -f docker/docker-compose.caching.yml up -d
```

### GPU-Accelerated Profile

Enhanced performance with GPU inference.

**Requirements**:
- RAM: 16GB minimum
- CPU: 4-8 cores
- VRAM: 8GB minimum, 12GB+ recommended
- Storage: 50GB
- GPU: NVIDIA RTX 3060+ or better

**Services**:
- LocalAI (GPU mode)
- AGiXT (full stack)
- vCache
- TensorZero
- Refact AI (optional)

```bash
# Enable GPU in LocalAI
# Edit docker/docker-compose.localai.yml to uncomment GPU section

docker compose -f docker/docker-compose.localai.yml up -d
```

### Full Stack Profile

All services with maximum resources.

**Requirements**:
- RAM: 32GB minimum, 64GB recommended
- CPU: 8-16 cores
- VRAM: 16GB+ recommended
- Storage: 100GB
- GPU: NVIDIA RTX 4080+ or better

**Services**:
- All inference services (LocalAI GPU, Refact)
- All orchestration (AGiXT, AIOS)
- All caching (vCache, Redis)
- All monitoring (TensorZero, ClickHouse)

---

## Performance Benchmarks

### Token Generation Speed (7B Model)

| Hardware | Quantization | Tokens/sec | First Token Latency |
|----------|--------------|------------|---------------------|
| i7-12700K (CPU) | Q4_K_M | 15-20 | 500ms |
| RTX 3060 12GB | Q4_K_M | 60-80 | 100ms |
| RTX 3080 10GB | Q4_K_M | 80-100 | 80ms |
| RTX 4080 16GB | Q4_K_M | 100-130 | 60ms |
| RTX 4090 24GB | Q4_K_M | 140-180 | 40ms |
| M2 Max (64GB) | Q4_K_M | 40-60 | 150ms |

### Memory Bandwidth Impact

| Memory Type | Bandwidth | Impact on CPU Inference |
|-------------|-----------|-------------------------|
| DDR4-2400 | 38 GB/s | Baseline |
| DDR4-3200 | 51 GB/s | +15% throughput |
| DDR5-4800 | 77 GB/s | +40% throughput |
| DDR5-6400 | 102 GB/s | +60% throughput |

### Context Size Impact

| Context Size | VRAM Overhead | Latency Impact |
|--------------|---------------|----------------|
| 512 | +0.1GB | Baseline |
| 2048 | +0.5GB | +10% |
| 4096 | +1GB | +25% |
| 8192 | +2GB | +50% |
| 16384 | +4GB | +100% |

---

## Resource Validation

Before starting services, validate your system resources:

```bash
# Run resource validation script
./scripts/validate-resources.sh

# Or check manually
./scripts/validate-resources.sh --check-only
```

The script checks:
- Available RAM and swap
- CPU cores and architecture
- GPU availability and VRAM
- Disk space in data directories
- Docker resource limits

---

## Optimization Recommendations

### CPU-Only Systems

1. **Use smaller models**: Prefer 1.5B-4B models
2. **Lower context size**: Use 512-2048 tokens
3. **Enable swap**: Configure 8-16GB swap for headroom
4. **Use Q4 quantization**: Best performance/quality ratio
5. **Disable unused services**: Run only essential containers

### GPU-Accelerated Systems

1. **Maximize GPU layers**: Load entire model to VRAM if possible
2. **Use batch inference**: Process multiple requests together
3. **Enable KV cache**: Reduces recomputation
4. **Use FP16**: Better than FP32 for inference
5. **Monitor VRAM**: Avoid OOM with proper limits

### Memory-Constrained Systems

1. **Use mmap**: LocalAI supports memory-mapped model loading
2. **Single model**: Load only one model at a time
3. **Aggressive cache TTL**: Short TTL in vCache
4. **Disable ClickHouse**: Skip LLMOps logging
5. **Minimal AGiXT**: Skip MinIO if not needed

---

## Troubleshooting

### Out of Memory (OOM)

```bash
# Check container memory usage
docker stats

# Reduce model context
CONTEXT_SIZE=1024 docker compose -f docker/docker-compose.localai.yml up -d

# Use smaller quantization
# Replace Q8_0 models with Q4_K_M variants
```

### Slow Inference

```bash
# Check CPU usage
htop

# Increase threads (CPU mode)
THREADS=8 docker compose -f docker/docker-compose.localai.yml up -d

# Verify GPU is being used
nvidia-smi
```

### GPU Not Detected

```bash
# Check NVIDIA driver
nvidia-smi

# Check Docker GPU support
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi

# Verify container has GPU access
docker exec localai-gpu nvidia-smi
```

---

## Related Documentation

- [INFERENCE_SETUP.md](../getting-started/INFERENCE_SETUP.md) - Inference plane configuration
- [VCACHE-SETUP.md](../VCACHE-SETUP.md) - Semantic caching setup
- [docker-compose.md](../containers/docker-compose.md) - Docker service architecture
- [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - General troubleshooting

## References

- [LocalAI Documentation](https://localai.io/)
- [GGUF Format Specification](https://github.com/ggerganov/ggml/blob/master/docs/gguf.md)
- [NVIDIA CUDA Compatibility](https://developer.nvidia.com/cuda-gpus)
- [Docker Resource Constraints](https://docs.docker.com/config/containers/resource_constraints/)
