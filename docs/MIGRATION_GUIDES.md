# Migration Guides

**Status:** Complete
**Last Updated:** 2026-01-14
**Purpose:** Guides for upgrading and migrating between versions

---

## Overview

This document provides migration guides for common upgrade scenarios in the ripple-env development environment.

---

## Table of Contents

1. [ROS2 Version Upgrades](#ros2-version-upgrades)
2. [Standalone to Kubernetes Migration](#standalone-to-kubernetes-migration)
3. [Nix Flake Input Updates](#nix-flake-input-updates)
4. [Python Version Upgrades](#python-version-upgrades)
5. [Docker Compose to Kubernetes](#docker-compose-to-kubernetes)

---

## ROS2 Version Upgrades

### Humble to Future LTS (When Available)

**When:** ROS2 Jazzy or later becomes available in RoboStack

**Pre-Migration Checklist:**
- [ ] Review [ROS2 Migration Guide](https://docs.ros.org/en/rolling/Releases.html)
- [ ] Check RoboStack channel availability
- [ ] Back up current environment: `pixi export > pixi-backup.yml`
- [ ] Test in isolated environment first

**Step 1: Update Channel**

```toml
# pixi.toml
[feature.ros]
channels = ["robostack-jazzy", "conda-forge"]  # Update channel

[feature.ros.dependencies]
ros-jazzy-desktop = ">=0.10.0"  # Update package prefix
```

**Step 2: Update Package References**

```bash
# Find all ros-humble references
grep -r "ros-humble" --include="*.toml" --include="*.nix"

# Update each reference from ros-humble-* to ros-jazzy-*
```

**Step 3: Rebuild Environment**

```bash
pixi clean
pixi install
nix develop
```

**Step 4: Test ROS2 Functionality**

```bash
ros2 --version
ros2 doctor
colcon build
colcon test
```

**Breaking Changes to Watch:**
- API changes in rclcpp/rclpy
- Message type changes
- Launch file syntax updates
- Parameter handling changes

---

## Standalone to Kubernetes Migration

### Phase 1: Prepare Docker Compose Services

**Pre-Migration Checklist:**
- [ ] All services running in Docker Compose
- [ ] Persistent volumes documented
- [ ] Environment variables cataloged
- [ ] Network topology mapped

**Step 1: Export Docker Compose Configuration**

```bash
# Generate Kubernetes manifests from compose
kompose convert -f docker-compose.yml -o k8s/

# Or use Helm chart
helm template flexstack charts/flexstack > k8s/all-in-one.yaml
```

**Step 2: Configure Persistent Storage**

```yaml
# k8s/pvc.yaml
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: postgres-pvc
spec:
  accessModes: ["ReadWriteOnce"]
  resources:
    requests:
      storage: 10Gi
```

**Step 3: Create ConfigMaps and Secrets**

```bash
# From .env files
kubectl create configmap flexstack-config --from-env-file=.env

# For secrets
kubectl create secret generic flexstack-secrets \
  --from-literal=POSTGRES_PASSWORD=changeme \
  --from-literal=REDIS_PASSWORD=changeme
```

### Phase 2: Deploy to Kubernetes

**Step 1: Create Namespace**

```bash
kubectl create namespace flexstack
kubectl config set-context --current --namespace=flexstack
```

**Step 2: Deploy with Helm**

```bash
helm install flexstack charts/flexstack \
  --namespace flexstack \
  --values values-production.yaml
```

**Step 3: Verify Deployment**

```bash
kubectl get pods -n flexstack
kubectl get services -n flexstack
kubectl logs -f deployment/localai -n flexstack
```

### Phase 3: Data Migration

**Step 1: Backup Docker Volumes**

```bash
# PostgreSQL
docker exec postgres pg_dump -U postgres > backup.sql

# Redis
docker exec redis redis-cli BGSAVE
docker cp redis:/data/dump.rdb ./redis-backup.rdb
```

**Step 2: Restore to Kubernetes**

```bash
# Copy to pod
kubectl cp backup.sql postgres-0:/tmp/backup.sql

# Restore
kubectl exec -it postgres-0 -- psql -U postgres < /tmp/backup.sql
```

---

## Nix Flake Input Updates

### Updating All Inputs

```bash
# Update all flake inputs
nix flake update

# Update specific input
nix flake update nixpkgs

# Check for changes
git diff flake.lock
```

### Updating nixpkgs

**Pre-Update:**

```bash
# Check current nixpkgs version
nix flake metadata --json | jq '.locks.nodes.nixpkgs.locked.rev'
```

**Step 1: Update**

```bash
nix flake update nixpkgs
```

**Step 2: Test Build**

```bash
nix flake check
nix build
```

**Step 3: Resolve Breaking Changes**

Common issues:
- Package renamed: Check [NixOS package search](https://search.nixos.org/packages)
- Package removed: Find alternative or pin old nixpkgs
- Option changed: Check [NixOS options](https://search.nixos.org/options)

### Pinning Specific Versions

```nix
# flake.nix
{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";  # Pin to release
    nixpkgs-unstable.url = "github:NixOS/nixpkgs/nixos-unstable";
  };
}
```

---

## Python Version Upgrades

### 3.11 to 3.12

**Pre-Migration Checklist:**
- [ ] Check AIOS compatibility (requires 3.11)
- [ ] Review deprecated APIs
- [ ] Test all Python packages

**Step 1: Update pixi.toml**

```toml
[dependencies]
python = ">=3.12,<3.13"
```

**Step 2: Check for Incompatibilities**

```bash
# Run compatibility check
./scripts/check-python-deps.sh

# Common issues:
# - distutils removed (use setuptools)
# - asyncio changes
# - typing module changes
```

**Step 3: Update Code**

```bash
# Use pyupgrade for automatic fixes
pyupgrade --py312-plus **/*.py

# Use ruff for linting
ruff check --fix .
```

**Known Incompatibilities:**

| Package | Issue | Solution |
|---------|-------|----------|
| AIOS | Requires 3.11 | Use separate environment |
| Some ML libs | May lag behind | Check compatibility |

---

## Docker Compose to Kubernetes

### Service-by-Service Migration

#### LocalAI

**Docker Compose:**
```yaml
localai:
  image: localai/localai:v2.24.2-aio-cpu
  ports: ["8080:8080"]
  volumes:
    - localai_models:/models
```

**Kubernetes:**
```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: localai
spec:
  replicas: 1
  template:
    spec:
      containers:
      - name: localai
        image: localai/localai:v2.24.2-aio-cpu
        ports:
        - containerPort: 8080
        volumeMounts:
        - name: models
          mountPath: /models
      volumes:
      - name: models
        persistentVolumeClaim:
          claimName: localai-models
```

#### PostgreSQL

**Docker Compose:**
```yaml
postgres:
  image: postgres:17.2-alpine
  environment:
    POSTGRES_PASSWORD: changeme
  volumes:
    - postgres_data:/var/lib/postgresql/data
```

**Kubernetes (using Helm):**
```bash
helm install postgresql bitnami/postgresql \
  --set auth.postgresPassword=changeme \
  --set persistence.size=10Gi
```

### Network Migration

| Docker Compose | Kubernetes |
|----------------|------------|
| `networks: agentic-network` | `Service` + `NetworkPolicy` |
| `depends_on` | `initContainers` or readiness probes |
| `ports: "8080:8080"` | `Service` + optional `Ingress` |

---

## Rollback Procedures

### Nix Rollback

```bash
# List generations
nix profile history

# Rollback to previous
nix profile rollback

# Rollback to specific generation
nix profile rollback --to <generation>
```

### Pixi Rollback

```bash
# Restore from backup
pixi clean
git checkout pixi.lock
pixi install
```

### Kubernetes Rollback

```bash
# Check rollout history
kubectl rollout history deployment/localai

# Rollback to previous
kubectl rollout undo deployment/localai

# Rollback to specific revision
kubectl rollout undo deployment/localai --to-revision=2
```

---

## References

- [ROS2 Migration Guides](https://docs.ros.org/en/rolling/Releases.html)
- [NixOS Upgrade Notes](https://nixos.org/manual/nixos/stable/release-notes.html)
- [Kompose](https://kompose.io/) - Docker Compose to Kubernetes
- [Python 3.12 What's New](https://docs.python.org/3/whatsnew/3.12.html)

