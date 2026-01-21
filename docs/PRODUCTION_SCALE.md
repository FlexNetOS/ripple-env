# Production Scale Requirements

Resource baselines and scaling guidelines for production deployments.

**Purpose**: Define hardware/resource requirements for different deployment profiles
**Status**: Phase 8 Complete
**Last Updated**: 2026-01-14
**Evidence**: Thresholds derived from `scripts/validate-resources.sh`

---

## Overview

This document provides resource baselines for scaling ripple-env from lightweight development (4GB RAM) to full production stack (64GB+ RAM). All values are based on observed resource usage and validation thresholds.

**Resource Validation**: `./scripts/validate-resources.sh`

---

## Resource Profiles

### Profile: Lightweight (1-3 Services)

**Use Case**: ROS2 development only, minimal AI services

| Resource | Minimum | Recommended | Maximum |
|----------|---------|-------------|---------|
| **CPU** | 2 cores | 4 cores | 8 cores |
| **RAM** | 4GB | 8GB | 16GB |
| **Disk** | 20GB | 50GB | 100GB |
| **Network** | 10 Mbps | 100 Mbps | 1 Gbps |

**Services Included**:
- ROS2 Humble (via RoboStack)
- Core development tools (Nix, Pixi)
- Git, editors, shell

**Not Included**:
- Docker services
- AI/ML inference
- Databases

**Example Setup**:
```bash
# Pixi minimal install
pixi install --frozen

# ROS2 only
pixi run python -c "import rclpy"
```

**Evidence**: Minimum thresholds from `scripts/validate-resources.sh`

---

### Profile: Standard (4-7 Services)

**Use Case**: Full development stack with Docker services

| Resource | Minimum | Recommended | Maximum |
|----------|---------|-------------|---------|
| **CPU** | 4 cores | 8 cores | 16 cores |
| **RAM** | 16GB | 32GB | 64GB |
| **Disk** | 50GB | 100GB | 200GB |
| **Network** | 100 Mbps | 1 Gbps | 10 Gbps |

**Services Included**:
- ROS2 + PyTorch (CPU)
- PostgreSQL
- Keycloak
- Vault
- NATS
- Prometheus
- Neo4j

**Docker Compose Profiles**: `core,data,identity,messaging`

**Example Setup**:
```bash
# Start core services
docker compose --profile core --profile data up -d

# Verify resource usage
docker stats
```

**Measured Usage** (from validation):
- PostgreSQL: ~2GB RAM
- Keycloak: ~1-2GB RAM
- Prometheus: ~1-2GB RAM
- NATS: ~512MB RAM
- Vault: ~256MB RAM

**Evidence**: Service memory limits from docker-compose files

---

### Profile: GPU (AI/ML Workloads)

**Use Case**: LocalAI, model inference, GPU-accelerated training

| Resource | Minimum | Recommended | Maximum |
|----------|---------|-------------|---------|
| **CPU** | 8 cores | 16 cores | 32+ cores |
| **RAM** | 32GB | 64GB | 128GB+ |
| **GPU VRAM** | 8GB | 16GB | 24GB+ |
| **Disk** | 100GB | 200GB | 500GB+ |
| **Network** | 1 Gbps | 10 Gbps | 40 Gbps |

**Services Included**:
- All Standard services
- LocalAI (GPU inference)
- MindsDB (ML platform)
- Large language models (7B-70B parameters)

**GPU Requirements**:
- NVIDIA GPU with CUDA 12.0+ support
- nvidia-container-toolkit installed
- Adequate VRAM for model size

**Model Size vs VRAM**:

| Model Size | VRAM Required | Examples |
|------------|---------------|----------|
| 7B params | 8GB | Mistral-7B, Llama-3-8B |
| 13B params | 16GB | Llama-2-13B |
| 34B params | 24GB | CodeLlama-34B |
| 70B params | 48GB+ | Llama-2-70B (quantized) |

**Example Setup**:
```bash
# Enable GPU profile
docker compose --profile core --profile ai --profile gpu up -d

# Verify GPU access
nvidia-smi
docker run --gpus all nvidia/cuda:12.0-base nvidia-smi
```

**Evidence**: GPU detection in `scripts/validate-resources.sh`

---

### Profile: Full Stack (10+ Services)

**Use Case**: Complete production environment, all services enabled

| Resource | Minimum | Recommended | Maximum |
|----------|---------|-------------|---------|
| **CPU** | 8 cores | 16+ cores | 64+ cores |
| **RAM** | 32GB | 64GB+ | 256GB+ |
| **Disk** | 100GB | 500GB+ | 2TB+ |
| **Network** | 1 Gbps | 10 Gbps | 100 Gbps |

**Services Included**:
- All GPU profile services
- Temporal (workflow engine)
- Grafana (visualization)
- Distributed tracing
- Multiple databases
- Full observability stack

**Docker Compose Profiles**: `full`

**Example Setup**:
```bash
# Start all services
docker compose --profile full up -d

# Verify all healthy
docker compose ps
./scripts/validate-e2e.sh
```

**Scaling Considerations**:
- Use Kubernetes for >10 service instances
- Consider multi-node deployment
- External load balancer required

**Evidence**: Full profile validation in `scripts/validate-e2e.sh`

---

## Service-Specific Requirements

### Critical Services

| Service | CPU | Memory | Disk | Notes |
|---------|-----|--------|------|-------|
| **Prometheus** | 1 core | 2GB | 10GB | Metrics retention |
| **PostgreSQL** | 2 cores | 4GB | 20GB | Database size dependent |
| **LocalAI** | 4-8 cores | 8-16GB | 50GB | Model storage |
| **Temporal** | 2 cores | 4GB | 10GB | Workflow history |
| **Neo4j** | 2 cores | 2-4GB | 10GB | Graph database |
| **Keycloak** | 1 core | 1-2GB | 1GB | Authentication |
| **Vault** | 1 core | 256MB | 1GB | Secrets storage |
| **NATS** | 1 core | 512MB | 5GB | Message persistence |
| **Grafana** | 1 core | 512MB | 1GB | Dashboard storage |

**Evidence**: Derived from resource validation analysis and docker-compose memory limits

---

## Scaling Strategies

### Vertical Scaling (Single Node)

**When to Use**:
- <10 services
- Development/staging environments
- Budget constraints

**Approach**:
1. Increase RAM first (biggest impact)
2. Add CPU cores as needed
3. Use SSD for disk I/O
4. Monitor with Prometheus

**Limits**:
- WSL2: ~32GB RAM practical limit
- Docker: Host resource constraints
- Single point of failure

---

### Horizontal Scaling (Multi-Node)

**When to Use**:
- >10 service instances
- High availability required
- Production environments

**Approach**:
1. Migrate to Kubernetes (see `docs/HELM_CHART.md`)
2. Use external load balancer
3. Deploy databases with replication
4. Implement service mesh (Istio/Linkerd)

**Services to Scale Horizontally**:
- API services (stateless)
- Workers/processors
- LocalAI inference (multiple replicas)

**Services to Keep Centralized**:
- Databases (use replication, not sharding initially)
- Vault (HA mode with Raft)

**Evidence**: `docs/HELM_CHART.md` documents Kubernetes migration

---

## Performance Benchmarks

**UNKNOWN**: Formal performance benchmarks not yet established

**Recommended Benchmarks to Establish**:

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| Bootstrap time | <30 min | Time from fresh install to ready |
| Build time | <10 min | Full ROS2 workspace build |
| E2E validation | <5 min | `./scripts/validate-e2e.sh` |
| API response time | <100ms | P95 latency |
| Inference latency | <2s | LocalAI 7B model |

**Future Work**: Conduct load testing to establish baselines

**Priority**: MEDIUM (helpful for capacity planning)

---

## Monitoring Thresholds

### Resource Alerts (from Observability Setup)

| Metric | Warning | Critical | Action |
|--------|---------|----------|--------|
| **CPU Usage** | >70% | >90% | Scale up or optimize |
| **RAM Usage** | >80% | >95% | Add RAM or reduce services |
| **Disk Usage** | >75% | >90% | Clean up or add storage |
| **Disk I/O** | >80% | >95% | Use faster disk (SSD) |
| **Network Bandwidth** | >70% | >90% | Upgrade network or optimize |

**Evidence**: Thresholds documented in `docs/OBSERVABILITY-QUICK-START.md`

### Service Health Checks

| Service | Check Interval | Timeout | Retries | Failure Action |
|---------|---------------|---------|---------|----------------|
| PostgreSQL | 30s | 10s | 3 | Alert + restart |
| Vault | 30s | 10s | 3 | Alert (manual unseal) |
| Keycloak | 30s | 10s | 3 | Alert + restart |
| NATS | 30s | 10s | 3 | Alert + restart |
| LocalAI | 60s | 30s | 3 | Alert (slow model load) |

**Exit Codes** (from `scripts/validate-e2e.sh`):
- `0` = All services healthy
- `1` = Warnings (investigate but not critical)
- `2` = Critical failures (immediate action required)

**Evidence**: Health check configuration in docker-compose files

---

## Capacity Planning

### Growth Projections

| Services | Current | 6 Months | 12 Months | 24 Months |
|----------|---------|----------|-----------|-----------|
| **Count** | 9 | 15 | 25 | 50+ |
| **RAM Required** | 16GB | 32GB | 64GB | 128GB+ |
| **Disk Required** | 100GB | 200GB | 500GB | 1TB+ |
| **Strategy** | Single node | Single node | Multi-node | Kubernetes |

**Recommendation**: Plan migrations early
- 6 months: Upgrade to 32GB RAM, 200GB disk
- 12 months: Kubernetes migration planning
- 24 months: Full Kubernetes deployment

---

## Cost Optimization

### Resource Optimization Tips

1. **Use Docker resource limits**:
   ```yaml
   services:
     myservice:
       mem_limit: 2g
       cpus: 1.0
   ```

2. **Stop unused services**:
   ```bash
   docker compose stop <unused-service>
   ```

3. **Use lightweight alternatives**:
   - Alpine Linux base images
   - MinIO instead of full S3
   - SQLite for development

4. **Clean up regularly**:
   ```bash
   nix-collect-garbage -d
   docker system prune -a
   ./scripts/wsl-cleanup.sh  # WSL only
   ```

**Evidence**: `scripts/wsl-cleanup.sh` exists

---

## WSL2-Specific Considerations

### WSL Memory Configuration

Edit `~/.wslconfig` (Windows user directory):

```ini
[wsl2]
memory=32GB       # Allocate 32GB (for Standard profile)
processors=8      # Allocate 8 cores
swap=4GB          # Swap space
```

**Recommendations by Profile**:

| Profile | Memory | Processors | Swap |
|---------|--------|------------|------|
| Lightweight | 4-8GB | 2-4 | 1GB |
| Standard | 16-32GB | 4-8 | 2-4GB |
| GPU | 32-64GB | 8-16 | 4-8GB |
| Full Stack | 64GB+ | 16+ | 8GB+ |
| High-End Workstation | 384GB | 40 | 32GB |

**High-End Workstation Profile** (AMD Threadripper PRO 7965WX, 512GB RAM, Dual RTX 5090):

```powershell
# Bootstrap with high-end profile
.\bootstrap.ps1 -HardwareProfile "high-end-workstation"
```

Or use the pre-configured template from WSL:
```bash
cp config/wslconfig/high-end-workstation.wslconfig /mnt/c/Users/$USER/.wslconfig
```

**Apply changes**:
```powershell
wsl --shutdown
wsl
```

**Evidence**: WSL configuration documented in `docs/WSL2_BUILD_PIPELINE.md`

---

## Related Docs

- [Performance Tuning](cookbooks/PERFORMANCE_TUNING.md) - Optimization procedures
- [Resource Validation](scripts/validate-resources.sh) - Automated resource checks
- [AI Resource Requirements](ai-ml/AI_RESOURCE_REQUIREMENTS.md) - AI service requirements
- [Observability Setup](getting-started/quick-start/OBSERVABILITY-QUICK-START.md) - Monitoring configuration
- [WSL Build Pipeline](wsl/WSL2_BUILD_PIPELINE.md) - WSL-specific optimizations
- [Helm Chart](HELM_CHART.md) - Kubernetes scaling path
