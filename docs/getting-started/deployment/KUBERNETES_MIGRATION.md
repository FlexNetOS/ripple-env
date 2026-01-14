# Kubernetes Migration Guide

This guide provides a comprehensive path for migrating FlexStack from Docker Compose to Kubernetes. The migration supports both raw Kubernetes manifests (with Kustomize) and Helm charts.

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Migration Approaches](#migration-approaches)
4. [Step-by-Step Migration](#step-by-step-migration)
5. [Service-by-Service Migration](#service-by-service-migration)
6. [Configuration Migration](#configuration-migration)
7. [Data Migration](#data-migration)
8. [Networking Differences](#networking-differences)
9. [Validation](#validation)
10. [Rollback Strategy](#rollback-strategy)
11. [Troubleshooting](#troubleshooting)

## Overview

FlexStack's Kubernetes deployment provides:

- **Scalability**: Horizontal pod autoscaling and multi-replica deployments
- **High Availability**: Pod disruption budgets and rolling updates
- **Security**: Network policies, RBAC, and secrets management
- **Observability**: Native Prometheus metrics and Kubernetes events
- **GitOps Ready**: ArgoCD integration for continuous deployment

### Architecture Comparison

| Aspect | Docker Compose | Kubernetes |
|--------|---------------|------------|
| Orchestration | Single host | Multi-node cluster |
| Networking | Docker network | Kubernetes Services + DNS |
| Storage | Docker volumes | PersistentVolumeClaims |
| Secrets | .env files | Kubernetes Secrets |
| Configuration | docker-compose.yml | Manifests/Helm charts |
| Health checks | Container healthcheck | liveness/readiness probes |
| Scaling | Manual | HPA/VPA |

## Prerequisites

### Required Tools

```bash
# Kubernetes CLI
kubectl version --client

# Helm (for Helm-based deployment)
helm version

# Kustomize (for manifest-based deployment)
kustomize version

# Optional: kubeval for validation
kubeval --version
```

### Cluster Requirements

- Kubernetes 1.25+
- StorageClass with dynamic provisioning
- At least 16GB RAM, 8 CPU cores across nodes
- Ingress controller (nginx, traefik, or similar)

### Recommended Cluster Sizing

| Environment | Nodes | Per-Node Resources |
|-------------|-------|-------------------|
| Development | 1 | 8 CPU, 16GB RAM |
| Staging | 3 | 4 CPU, 16GB RAM |
| Production | 5+ | 8 CPU, 32GB RAM |

## Migration Approaches

### Approach 1: Kustomize Manifests (Recommended for GitOps)

Best for teams using GitOps with ArgoCD or Flux.

```bash
# Deploy development environment
kubectl apply -k manifests/kubernetes/overlays/development

# Deploy production environment
kubectl apply -k manifests/kubernetes/overlays/production
```

### Approach 2: Helm Charts (Recommended for Enterprise)

Best for teams needing templating and release management.

```bash
# Install with Helm
helm install flexstack charts/flexstack \
  --namespace flexstack \
  --create-namespace \
  -f custom-values.yaml
```

### Approach 3: Hybrid (ArgoCD + Helm)

Best for production with full GitOps workflow.

```bash
# Apply ArgoCD application
kubectl apply -f manifests/argocd/applications.yaml
```

## Step-by-Step Migration

### Phase 1: Preparation (Week 1)

1. **Audit Current State**
   ```bash
   # List running containers
   docker compose ps

   # Export current volumes
   docker compose exec postgres pg_dumpall > backup.sql

   # Document environment variables
   docker compose config > compose-resolved.yml
   ```

2. **Set Up Kubernetes Cluster**
   ```bash
   # For local development with kind
   kind create cluster --config kind-config.yaml

   # For production, use managed Kubernetes
   # (EKS, GKE, AKS, or self-managed)
   ```

3. **Install Prerequisites**
   ```bash
   # Install metrics-server
   kubectl apply -f https://github.com/kubernetes-sigs/metrics-server/releases/latest/download/components.yaml

   # Install ingress controller
   helm install nginx-ingress ingress-nginx/ingress-nginx
   ```

### Phase 2: Deploy Core Infrastructure (Week 2)

1. **Deploy Namespaces**
   ```bash
   kubectl apply -f manifests/kubernetes/base/namespace.yaml
   ```

2. **Deploy Core Services**
   ```bash
   # PostgreSQL, Redis, MinIO
   kubectl apply -k manifests/kubernetes/base/core

   # Wait for services to be ready
   kubectl wait --for=condition=ready pod -l app.kubernetes.io/component=database -n flexstack --timeout=300s
   ```

3. **Migrate Data**
   ```bash
   # Restore PostgreSQL backup
   kubectl exec -i $(kubectl get pod -l app.kubernetes.io/name=postgres -o jsonpath='{.items[0].metadata.name}' -n flexstack) -n flexstack -- psql -U postgres < backup.sql
   ```

### Phase 3: Deploy Application Services (Week 3)

1. **Deploy by Profile**
   ```bash
   # Observability
   kubectl apply -k manifests/kubernetes/base/observability

   # AI Services
   kubectl apply -k manifests/kubernetes/base/ai

   # Identity
   kubectl apply -k manifests/kubernetes/base/identity

   # Messaging
   kubectl apply -k manifests/kubernetes/base/messaging

   # Edge
   kubectl apply -k manifests/kubernetes/base/edge

   # Automation
   kubectl apply -k manifests/kubernetes/base/automation
   ```

2. **Verify Deployments**
   ```bash
   kubectl get pods --all-namespaces -l app.kubernetes.io/part-of=flexstack
   ```

### Phase 4: Validate and Cutover (Week 4)

1. **Run Validation Tests**
   ```bash
   # Health checks
   kubectl get pods --all-namespaces | grep -v Running

   # Service connectivity
   kubectl run debug --rm -it --image=busybox -- wget -qO- http://localai.flexstack-ai:8080/readyz
   ```

2. **Update DNS/Load Balancer**
   - Point DNS to Kubernetes ingress
   - Update firewall rules

3. **Decommission Docker Compose**
   ```bash
   docker compose down
   ```

## Service-by-Service Migration

### PostgreSQL

| Docker Compose | Kubernetes |
|---------------|------------|
| `postgres:5432` | `postgres.flexstack.svc.cluster.local:5432` |
| Docker volume | PersistentVolumeClaim |
| Environment vars | Secret + ConfigMap |

**Migration Steps:**
```bash
# 1. Backup from Docker
docker compose exec postgres pg_dumpall -U postgres > postgres_backup.sql

# 2. Deploy Kubernetes PostgreSQL
kubectl apply -f manifests/kubernetes/base/core/postgres.yaml

# 3. Restore to Kubernetes
kubectl cp postgres_backup.sql flexstack/postgres-0:/tmp/
kubectl exec -it postgres-0 -n flexstack -- psql -U postgres -f /tmp/postgres_backup.sql
```

### LocalAI

| Docker Compose | Kubernetes |
|---------------|------------|
| `localai:8080` | `localai.flexstack-ai.svc.cluster.local:8080` |
| Bind mount models | PVC for models |
| CPU allocation | Resource limits |

**Migration Steps:**
```bash
# 1. Copy models from Docker volume
docker cp flexstack-localai:/models ./models_backup

# 2. Deploy Kubernetes LocalAI
kubectl apply -f manifests/kubernetes/base/ai/localai.yaml

# 3. Copy models to PVC
kubectl cp ./models_backup flexstack-ai/localai-xxx:/models/
```

### Temporal

| Docker Compose | Kubernetes |
|---------------|------------|
| `temporal:7233` | `temporal.flexstack-messaging.svc.cluster.local:7233` |
| Single instance | Multi-replica capable |
| Auto-setup image | Same image with init |

**Migration Steps:**
```bash
# 1. Export Temporal data (if needed)
# Note: Temporal data is in PostgreSQL

# 2. Deploy Kubernetes Temporal
kubectl apply -f manifests/kubernetes/base/messaging/temporal.yaml

# 3. Verify workflow history
kubectl port-forward -n flexstack-messaging svc/temporal-ui 8088:8080
# Visit http://localhost:8088
```

## Configuration Migration

### Environment Variables to ConfigMaps/Secrets

```yaml
# Docker Compose .env
POSTGRES_PASSWORD=changeme
LOCALAI_THREADS=4
```

Becomes:

```yaml
# Kubernetes Secret
apiVersion: v1
kind: Secret
metadata:
  name: postgres-credentials
type: Opaque
stringData:
  POSTGRES_PASSWORD: changeme

# Kubernetes ConfigMap
apiVersion: v1
kind: ConfigMap
metadata:
  name: localai-config
data:
  THREADS: "4"
```

### Docker Compose Profiles to Kubernetes Namespaces

| Docker Profile | Kubernetes Namespace |
|---------------|---------------------|
| core | flexstack |
| observability | flexstack-observability |
| ai | flexstack-ai |
| identity | flexstack-identity |
| edge | flexstack-edge |
| messaging | flexstack-messaging |

## Data Migration

### Strategy 1: Direct Copy (Small datasets)

```bash
# Export from Docker
docker compose exec -T postgres pg_dump -U postgres mydb > mydb.sql

# Import to Kubernetes
kubectl exec -i postgres-0 -n flexstack -- psql -U postgres mydb < mydb.sql
```

### Strategy 2: Streaming Replication (Large datasets)

```bash
# Set up PostgreSQL streaming replication between Docker and Kubernetes
# Then promote Kubernetes as primary after cutover
```

### Strategy 3: Application-Level Migration

```bash
# Use application APIs to export/import data
curl http://docker-host:7437/api/export > agixt_data.json
curl -X POST http://k8s-host:7437/api/import -d @agixt_data.json
```

## Networking Differences

### Service Discovery

| Docker Compose | Kubernetes |
|---------------|------------|
| Container name | `<service>.<namespace>.svc.cluster.local` |
| `postgres:5432` | `postgres.flexstack.svc.cluster.local:5432` |
| Network aliases | Service with multiple ports |

### Example Connection Strings

```yaml
# Docker Compose
DATABASE_URL: postgresql://postgres:5432/mydb

# Kubernetes
DATABASE_URL: postgresql://postgres.flexstack.svc.cluster.local:5432/mydb
```

### Network Policies

Kubernetes adds network segmentation:

```yaml
apiVersion: networking.k8s.io/v1
kind: NetworkPolicy
metadata:
  name: allow-postgres
spec:
  podSelector:
    matchLabels:
      app.kubernetes.io/name: postgres
  ingress:
    - from:
        - namespaceSelector:
            matchLabels:
              app.kubernetes.io/part-of: flexstack
      ports:
        - port: 5432
```

## Validation

### Pre-Migration Checklist

- [ ] All Docker volumes backed up
- [ ] Environment variables documented
- [ ] Network connectivity requirements mapped
- [ ] Storage requirements calculated
- [ ] Resource limits defined

### Post-Migration Validation

```bash
# Check all pods are running
kubectl get pods --all-namespaces -l app.kubernetes.io/part-of=flexstack

# Check services are accessible
kubectl run test --rm -it --image=curlimages/curl -- sh
> curl http://localai.flexstack-ai:8080/readyz
> curl http://prometheus.flexstack-observability:9090/-/healthy

# Run integration tests
kubectl apply -f tests/integration/k8s-tests.yaml
```

### Health Check Commands

```bash
# PostgreSQL
kubectl exec -it postgres-0 -n flexstack -- pg_isready

# Redis
kubectl exec -it redis-0 -n flexstack -- redis-cli ping

# LocalAI
kubectl exec -it deployment/localai -n flexstack-ai -- curl localhost:8080/readyz

# Prometheus
kubectl exec -it deployment/prometheus -n flexstack-observability -- wget -qO- localhost:9090/-/healthy
```

## Rollback Strategy

### Quick Rollback

```bash
# Scale down Kubernetes
kubectl scale --replicas=0 deployment --all -n flexstack
kubectl scale --replicas=0 deployment --all -n flexstack-ai
# ... for all namespaces

# Restart Docker Compose
cd /path/to/ripple-env
docker compose --profile full up -d
```

### Gradual Rollback

```bash
# Route traffic back to Docker
# Update DNS/load balancer to point to Docker host

# Scale down Kubernetes services one by one
kubectl scale --replicas=0 deployment/localai -n flexstack-ai
# Verify Docker service is handling traffic
# Repeat for each service
```

## Troubleshooting

### Common Issues

#### Pods Not Starting

```bash
# Check pod events
kubectl describe pod <pod-name> -n <namespace>

# Check logs
kubectl logs <pod-name> -n <namespace>

# Common fixes:
# - PVC not bound: Check storageClass
# - Image pull errors: Check imagePullSecrets
# - Resource limits: Increase limits
```

#### Service Not Accessible

```bash
# Check service exists
kubectl get svc -n <namespace>

# Test DNS resolution
kubectl run debug --rm -it --image=busybox -- nslookup <service-name>.<namespace>.svc.cluster.local

# Test connectivity
kubectl run debug --rm -it --image=curlimages/curl -- curl <service-url>
```

#### Database Connection Failed

```bash
# Check secret mounted
kubectl exec -it <pod> -- env | grep DATABASE

# Test database connectivity
kubectl run pg-debug --rm -it --image=postgres:17 -- psql -h postgres.flexstack -U postgres

# Check network policies
kubectl get networkpolicies -n <namespace>
```

### Logging and Monitoring

```bash
# View pod logs
kubectl logs -f deployment/<name> -n <namespace>

# View events
kubectl get events -n <namespace> --sort-by='.lastTimestamp'

# Access Grafana dashboards
kubectl port-forward -n flexstack-observability svc/grafana 3000:3000
```

## Related Documentation

- [Kubernetes Manifests README](../../manifests/kubernetes/README.md)
- [Helm Charts README](../../charts/README.md)
- [ArgoCD Applications](../../manifests/argocd/applications.yaml)
- [Docker Compose Reference](../../docker/docker-compose.yml)
