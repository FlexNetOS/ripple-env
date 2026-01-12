# FlexStack Kubernetes Manifests

This directory contains Kubernetes manifests generated from the Docker Compose configurations, organized using Kustomize for environment-specific deployments.

## Directory Structure

```
kubernetes/
├── base/                           # Base manifests (shared across environments)
│   ├── namespace.yaml              # Namespace definitions
│   ├── kustomization.yaml          # Base kustomization
│   ├── core/                       # Core infrastructure
│   │   ├── postgres.yaml           # PostgreSQL StatefulSet
│   │   ├── redis.yaml              # Redis Deployment
│   │   ├── minio.yaml              # MinIO object storage
│   │   └── kustomization.yaml
│   ├── observability/              # Monitoring stack
│   │   ├── prometheus.yaml         # Metrics collection
│   │   ├── grafana.yaml            # Visualization
│   │   ├── loki.yaml               # Log aggregation
│   │   ├── tempo.yaml              # Distributed tracing
│   │   ├── alertmanager.yaml       # Alert routing
│   │   └── kustomization.yaml
│   ├── ai/                         # AI/ML services
│   │   ├── localai.yaml            # Local LLM inference
│   │   ├── agixt.yaml              # Agent orchestration
│   │   └── kustomization.yaml
│   ├── identity/                   # Identity & security
│   │   ├── keycloak.yaml           # OIDC provider
│   │   ├── vault.yaml              # Secrets management
│   │   └── kustomization.yaml
│   ├── data/                       # Data services
│   │   ├── neo4j.yaml              # Graph database
│   │   └── kustomization.yaml
│   ├── edge/                       # API gateway
│   │   ├── kong.yaml               # Kong gateway
│   │   └── kustomization.yaml
│   ├── messaging/                  # Messaging & workflows
│   │   ├── nats.yaml               # Event bus
│   │   ├── temporal.yaml           # Workflow orchestration
│   │   └── kustomization.yaml
│   └── automation/                 # Automation
│       ├── n8n.yaml                # Workflow automation
│       ├── opa.yaml                # Policy agent
│       └── kustomization.yaml
└── overlays/                       # Environment-specific overrides
    ├── development/                # Development settings
    │   ├── kustomization.yaml
    │   └── development-storage.yaml
    └── production/                 # Production settings
        ├── kustomization.yaml
        ├── network-policies.yaml
        └── pod-disruption-budgets.yaml
```

## Service Profiles Mapping

| Docker Compose Profile | Kubernetes Directory | Namespace |
|------------------------|---------------------|-----------|
| core | base/core | flexstack |
| observability | base/observability | flexstack-observability |
| ai | base/ai | flexstack-ai |
| identity | base/identity | flexstack-identity |
| data | base/data | flexstack |
| edge | base/edge | flexstack-edge |
| messaging | base/messaging | flexstack-messaging |
| automation | base/automation | flexstack |

## Quick Start

### Prerequisites

- kubectl configured with cluster access
- kustomize (or kubectl with kustomize support)
- A running Kubernetes cluster (minikube, kind, k3s, or cloud provider)

### Deploy Development Environment

```bash
# Preview what will be deployed
kubectl kustomize overlays/development

# Apply to cluster
kubectl apply -k overlays/development
```

### Deploy Production Environment

```bash
# Preview what will be deployed
kubectl kustomize overlays/production

# Apply to cluster
kubectl apply -k overlays/production
```

### Deploy Specific Profiles

```bash
# Deploy only core services
kubectl apply -k base/core

# Deploy observability stack
kubectl apply -k base/observability

# Deploy AI services
kubectl apply -k base/ai
```

## Validation

Validate manifests before deployment:

```bash
# Dry-run validation
kubectl apply -k overlays/development --dry-run=client

# Server-side validation
kubectl apply -k overlays/development --dry-run=server

# Using kubeval for static validation
kubeval --strict base/**/*.yaml
```

## Migration from Docker Compose

### Service Dependencies

When migrating from Docker Compose, consider the following dependency order:

1. **Core services** (postgres, redis, minio) - Deploy first
2. **Identity services** (keycloak, vault) - Depends on postgres
3. **Messaging services** (nats, temporal) - Depends on postgres
4. **AI services** (localai, agixt) - Depends on postgres, minio
5. **Edge services** (kong) - Depends on its own database
6. **Observability** (prometheus, grafana, loki, tempo) - Can be deployed independently
7. **Automation** (n8n, opa) - Depends on postgres

### Key Differences

| Aspect | Docker Compose | Kubernetes |
|--------|---------------|------------|
| Networking | Docker network | Kubernetes Services + DNS |
| Storage | Docker volumes | PersistentVolumeClaims |
| Configuration | .env files | ConfigMaps + Secrets |
| Health checks | healthcheck directive | livenessProbe + readinessProbe |
| Resource limits | deploy.resources | resources.limits/requests |
| Service discovery | Container names | Service DNS names |

### Environment Variables

Environment variables from Docker Compose are migrated to:
- **ConfigMaps**: Non-sensitive configuration
- **Secrets**: Sensitive data (passwords, API keys)

Example DNS name mapping:
- Docker: `postgres:5432`
- Kubernetes: `postgres.flexstack.svc.cluster.local:5432`

## Customization

### Adding New Services

1. Create a new YAML file in the appropriate profile directory
2. Add the resource to the profile's `kustomization.yaml`
3. Update overlays if environment-specific changes are needed

### Modifying Resources

Use Kustomize patches in overlays:

```yaml
# overlays/production/kustomization.yaml
patches:
  - patch: |
      - op: replace
        path: /spec/replicas
        value: 3
    target:
      kind: Deployment
      name: my-service
```

## Troubleshooting

### Common Issues

1. **PVC not binding**: Check storage class availability
2. **Service not reachable**: Verify network policies and service selectors
3. **Pod CrashLoopBackOff**: Check logs with `kubectl logs -f <pod>`
4. **Resources pending**: Check node resources and quotas

### Debugging Commands

```bash
# Check pod status
kubectl get pods -n flexstack

# View pod logs
kubectl logs -f deployment/postgres -n flexstack

# Describe resource for events
kubectl describe pod <pod-name> -n flexstack

# Check network policies
kubectl get networkpolicies -A
```

## Related Documentation

- [Kubernetes Migration Guide](../../docs/deployment/KUBERNETES_MIGRATION.md)
- [Helm Charts](../../charts/README.md)
- [ArgoCD Applications](../argocd/applications.yaml)
