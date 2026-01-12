# Kubernetes Overlays

Environment-specific configurations using Kustomize overlays.

## Structure

```
overlays/
├── README.md          # This file
├── qa/                # QA environment
│   └── kustomization.yaml
└── staging/           # Staging environment
    └── kustomization.yaml
```

## Environments

### QA Environment

Purpose: Quality assurance and testing

**Characteristics:**
- Reduced resource allocation
- Debug logging enabled
- Single replica for most services
- Smaller storage volumes (5Gi)
- Faster deployment cycles

**Deploy:**
```bash
kubectl apply -k manifests/kubernetes/overlays/qa
```

### Staging Environment

Purpose: Production-like validation before release

**Characteristics:**
- Production-like resource allocation
- High availability (2+ replicas)
- Production-sized storage (20Gi)
- Node affinity for staging nodes
- Metrics and tracing enabled

**Deploy:**
```bash
kubectl apply -k manifests/kubernetes/overlays/staging
```

## Usage

### Apply an overlay

```bash
# QA
kubectl apply -k manifests/kubernetes/overlays/qa

# Staging
kubectl apply -k manifests/kubernetes/overlays/staging
```

### View generated manifests

```bash
# QA
kubectl kustomize manifests/kubernetes/overlays/qa

# Staging
kubectl kustomize manifests/kubernetes/overlays/staging
```

### Diff before apply

```bash
# QA
kubectl diff -k manifests/kubernetes/overlays/qa

# Staging
kubectl diff -k manifests/kubernetes/overlays/staging
```

### Delete an overlay

```bash
# QA
kubectl delete -k manifests/kubernetes/overlays/qa

# Staging
kubectl delete -k manifests/kubernetes/overlays/staging
```

## Customization

### Add environment-specific patches

Edit the `kustomization.yaml` in the overlay directory:

```yaml
patches:
  - patch: |-
      - op: replace
        path: /spec/replicas
        value: 3
    target:
      kind: Deployment
      name: my-service
```

### Add environment-specific ConfigMaps

```yaml
configMapGenerator:
  - name: my-config
    literals:
      - KEY=value
```

### Override images

```yaml
images:
  - name: my-image
    newTag: v2.0.0
```

## CI/CD Integration

### GitOps with ArgoCD

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: flexstack-staging
spec:
  project: default
  source:
    repoURL: https://github.com/FlexNetOS/ripple-env
    targetRevision: main
    path: manifests/kubernetes/overlays/staging
  destination:
    server: https://kubernetes.default.svc
    namespace: flexstack-staging
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
```

### GitHub Actions

```yaml
- name: Deploy to QA
  run: |
    kubectl apply -k manifests/kubernetes/overlays/qa

- name: Deploy to Staging
  run: |
    kubectl apply -k manifests/kubernetes/overlays/staging
```

## Best Practices

1. **Progressive deployment**: QA → Staging → Production
2. **Keep overlays minimal**: Only override what's necessary
3. **Use labels**: Tag resources with environment labels
4. **Test overlays**: Use `kubectl kustomize` to validate
5. **Version control**: Commit overlay changes with base changes

## Validation

Validate overlays before deploying:

```bash
# Validate YAML syntax
kubectl kustomize manifests/kubernetes/overlays/qa > /dev/null && echo "QA overlay valid"
kubectl kustomize manifests/kubernetes/overlays/staging > /dev/null && echo "Staging overlay valid"

# Check for resource conflicts
kubectl apply -k manifests/kubernetes/overlays/qa --dry-run=client
kubectl apply -k manifests/kubernetes/overlays/staging --dry-run=client
```

## Troubleshooting

### Overlay not applying

Check the base paths are correct:

```yaml
bases:
  - ../../base/messaging  # Must be relative path
```

### Resource conflicts

Ensure unique namespaces:

```yaml
namespace: flexstack-qa  # Different per overlay
```

### Patches not working

Verify the target selector matches:

```yaml
target:
  kind: Deployment
  name: my-service  # Must match actual resource name
```

## References

- [Kustomize Documentation](https://kustomize.io/)
- [Kubernetes Namespaces](https://kubernetes.io/docs/concepts/overview/working-with-objects/namespaces/)
- [Base Manifests](../base/)
