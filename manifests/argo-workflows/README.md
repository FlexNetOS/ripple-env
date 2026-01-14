# Argo Workflows - ARIA Infrastructure

This directory contains Argo Workflows configurations, templates, and examples for the ARIA platform.

## Directory Structure

```
argo-workflows/
├── config/                          # Configuration files
│   ├── workflow-controller-configmap.yaml
│   ├── rbac.yaml
│   └── argo-server-service.yaml
├── templates/                       # Workflow templates
│   ├── hello-world.yaml            # Basic test workflow
│   ├── ci-pipeline.yaml            # CI/CD pipeline example
│   ├── ml-pipeline.yaml            # ML training pipeline
│   ├── parallel-processing.yaml    # Parallel task execution
│   └── conditional-workflow.yaml   # Conditional logic example
└── README.md                        # This file
```

## Quick Start

### 1. Install Argo Workflows
Argo Workflows is automatically installed when you run:
```bash
docker compose -f docker/docker-compose.argo.yml up -d
```

### 2. Verify Installation
```bash
export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml
kubectl get pods -n argo
```

### 3. Apply Configuration
```bash
# Apply RBAC and service account
kubectl apply -f manifests/argo-workflows/config/rbac.yaml

# Apply workflow controller configuration
kubectl apply -f manifests/argo-workflows/config/workflow-controller-configmap.yaml

# Restart controller to pick up new config
kubectl rollout restart deployment/workflow-controller -n argo

# Expose Argo Server UI (if not already exposed)
kubectl apply -f manifests/argo-workflows/config/argo-server-service.yaml
```

### 4. Access Argo Workflows UI
```bash
# Port forward method
kubectl -n argo port-forward svc/argo-server 2746:2746

# Or use NodePort (already configured in docker-compose)
# Access at: http://localhost:2746
```

## Workflow Templates

### Hello World
Simple test workflow to verify Argo Workflows is functioning:
```bash
kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo
```

### CI/CD Pipeline
Demonstrates a complete CI/CD workflow:
```bash
# Install the template
kubectl apply -f manifests/argo-workflows/templates/ci-pipeline.yaml

# Submit workflow from template
kubectl create -n argo -f - <<EOF
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata:
  generateName: ci-pipeline-run-
spec:
  workflowTemplateRef:
    name: ci-pipeline
  arguments:
    parameters:
    - name: repo-url
      value: "https://github.com/your-org/your-repo"
    - name: branch
      value: "main"
    - name: image-tag
      value: "v1.0.0"
EOF
```

### ML Pipeline
Machine learning training and deployment workflow:
```bash
# Install the template
kubectl apply -f manifests/argo-workflows/templates/ml-pipeline.yaml

# Submit workflow
kubectl create -n argo -f - <<EOF
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata:
  generateName: ml-training-
spec:
  workflowTemplateRef:
    name: ml-pipeline
  arguments:
    parameters:
    - name: dataset-path
      value: "s3://aria-datasets/training-data"
    - name: model-name
      value: "aria-model-v1"
    - name: epochs
      value: "20"
EOF
```

### Parallel Processing
Demonstrates parallel task execution:
```bash
kubectl apply -f manifests/argo-workflows/templates/parallel-processing.yaml

kubectl create -n argo -f - <<EOF
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata:
  generateName: parallel-run-
spec:
  workflowTemplateRef:
    name: parallel-processing
  arguments:
    parameters:
    - name: batch-size
      value: "10"
EOF
```

### Conditional Workflow
Shows conditional execution based on parameters:
```bash
kubectl apply -f manifests/argo-workflows/templates/conditional-workflow.yaml

# Deploy to staging
kubectl create -n argo -f - <<EOF
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata:
  generateName: deploy-staging-
spec:
  workflowTemplateRef:
    name: conditional-workflow
  arguments:
    parameters:
    - name: environment
      value: "staging"
EOF

# Deploy to production (with approval gate)
kubectl create -n argo -f - <<EOF
apiVersion: argoproj.io/v1alpha1
kind: Workflow
metadata:
  generateName: deploy-production-
spec:
  workflowTemplateRef:
    name: conditional-workflow
  arguments:
    parameters:
    - name: environment
      value: "production"
EOF
```

## Argo CLI Commands

### Install Argo CLI
```bash
# Linux
curl -sLO https://github.com/argoproj/argo-workflows/releases/latest/download/argo-linux-amd64.gz
gunzip argo-linux-amd64.gz
chmod +x argo-linux-amd64
sudo mv argo-linux-amd64 /usr/local/bin/argo

# Verify installation
argo version
```

### Common CLI Operations
```bash
# List workflows
argo list -n argo

# Get workflow details
argo get <workflow-name> -n argo

# Watch workflow execution
argo watch <workflow-name> -n argo

# Get workflow logs
argo logs <workflow-name> -n argo

# Delete workflow
argo delete <workflow-name> -n argo

# Submit workflow from file
argo submit manifests/argo-workflows/templates/hello-world.yaml -n argo

# Submit with parameters
argo submit manifests/argo-workflows/templates/ci-pipeline.yaml \
  -p repo-url=https://github.com/example/repo \
  -p branch=develop \
  -p image-tag=v2.0.0 \
  -n argo

# Resubmit a workflow
argo resubmit <workflow-name> -n argo

# Stop a running workflow
argo stop <workflow-name> -n argo

# Resume a suspended workflow
argo resume <workflow-name> -n argo
```

## Monitoring and Observability

### View Workflow Status
```bash
# List all workflows with status
argo list -n argo

# Get detailed workflow information
kubectl get workflow -n argo

# Watch workflow events
kubectl get events -n argo --watch
```

### Access Logs
```bash
# Get workflow logs via CLI
argo logs <workflow-name> -n argo

# Follow logs in real-time
argo logs <workflow-name> -n argo -f

# Get logs from specific container
argo logs <workflow-name> -n argo -c <container-name>

# Get logs via kubectl
kubectl logs -n argo <pod-name>
```

### Workflow Metrics
```bash
# Access Prometheus metrics endpoint
kubectl port-forward -n argo svc/workflow-controller-metrics 9090:9090

# Metrics available at: http://localhost:9090/metrics
```

## Troubleshooting

### Check Argo Workflows Installation
```bash
# Check all Argo components
kubectl get all -n argo

# Check workflow controller logs
kubectl logs -n argo deployment/workflow-controller

# Check argo server logs
kubectl logs -n argo deployment/argo-server
```

### Common Issues

#### Workflow pods not starting
```bash
# Check service account permissions
kubectl get serviceaccount argo-workflow -n argo
kubectl describe rolebinding argo-workflow-binding -n argo

# Check pod events
kubectl describe pod <pod-name> -n argo
```

#### UI not accessible
```bash
# Verify argo-server is running
kubectl get pods -n argo -l app=argo-server

# Check service
kubectl get svc argo-server -n argo

# Port forward manually
kubectl port-forward -n argo svc/argo-server 2746:2746
```

#### Workflow stuck in pending
```bash
# Check workflow status
argo get <workflow-name> -n argo

# Check for resource constraints
kubectl top nodes
kubectl describe nodes

# Check pod status
kubectl get pods -n argo
```

## Integration with ARIA

### With Argo CD
Use Argo Workflows for CI/CD pipelines that deploy via Argo CD:
```yaml
- name: trigger-argocd-sync
  container:
    image: argoproj/argocd:latest
    command: [argocd, app, sync, myapp]
```

### With Argo Rollouts
Trigger progressive delivery with Argo Rollouts:
```yaml
- name: promote-rollout
  container:
    image: argoproj/kubectl-argo-rollouts:latest
    command: [kubectl-argo-rollouts, promote, myapp]
```

### With Temporal
Trigger Temporal workflows from Argo:
```yaml
- name: start-temporal-workflow
  container:
    image: temporalio/tctl:latest
    command: [tctl, workflow, start, --type, MyWorkflow]
```

## Best Practices

1. **Use WorkflowTemplates** for reusable workflows
2. **Set resource limits** on all containers
3. **Use TTL strategies** to clean up old workflows
4. **Implement proper RBAC** for security
5. **Use artifacts** for data passing between steps
6. **Add retries** for transient failures
7. **Use parallelism limits** to avoid resource exhaustion
8. **Monitor workflow metrics** via Prometheus
9. **Version your workflows** in Git
10. **Test workflows** in staging before production

## Additional Resources

- [Argo Workflows Documentation](https://argoproj.github.io/argo-workflows/)
- [Workflow Examples](https://github.com/argoproj/argo-workflows/tree/master/examples)
- [Best Practices](https://argoproj.github.io/argo-workflows/best-practices/)
- [API Reference](https://argoproj.github.io/argo-workflows/swagger/)

## Support

For issues specific to ARIA's Argo Workflows configuration:
1. Check workflow logs: `argo logs <workflow-name> -n argo`
2. Review controller logs: `kubectl logs -n argo deployment/workflow-controller`
3. Consult the troubleshooting section above
4. Contact the L5 Cluster & Delivery Domain Team
