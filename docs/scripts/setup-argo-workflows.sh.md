# Script Contract: setup-argo-workflows.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/setup-argo-workflows.sh`

---

## Purpose

Setup and configure Argo Workflows after installation (P1-011). Applies RBAC configuration, workflow controller ConfigMap, patches argo-server service, installs workflow templates, restarts controllers, and optionally runs test workflow. Post-installation configuration script.

---

## Invocation

```bash
./scripts/setup-argo-workflows.sh
```

**Interactive:** Prompts to run test workflow at end (lines 126-149).

**Requirements:**
- k3s cluster running
- Argo Workflows installed in `argo` namespace
- kubectl configured with access

---

## Outputs

**Standard Output:**
```
========================================
Argo Workflows Setup Script
========================================

Checking kubectl connectivity...
✓ kubectl connectivity verified

Checking Argo Workflows installation...
✓ Argo namespace exists

Applying Argo Workflows configurations...
  - Applying RBAC configuration...
  - Applying workflow controller configuration...
  - Configuring Argo Server service...
✓ Configurations applied

Installing workflow templates...
  - Installing ci-pipeline...
  - Installing ml-pipeline...
  - Installing parallel-processing...
  - Installing conditional-workflow...
✓ Workflow templates installed

Restarting Argo controllers to apply changes...
Waiting for rollout to complete...
✓ Controllers restarted

Verifying Argo Workflows installation...
  - Checking pods...
  - Checking services...
  - Checking workflow templates...
✓ Verification complete

Do you want to run a test workflow? (y/n) y
Submitting hello-world test workflow...
Workflow status:
...

========================================
Argo Workflows Setup Complete!
========================================

Access Information:
...
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success |
| `1` | Error (kubectl connectivity failed, Argo not installed) |

---

## Side Effects

### Kubernetes Resources Applied

**RBAC (line 54):**
- ServiceAccount: `argo-workflow`
- Role: `argo-workflow-role`
- RoleBinding: `argo-workflow-binding`

**ConfigMap (line 58):**
- `workflow-controller-configmap` in argo namespace

**Service Patch (line 62):**
- Changes `argo-server` service type to NodePort
- Exposes port 2746 on NodePort 32746

**Workflow Templates (lines 72-83):**
- `ci-pipeline` - CI/CD pipeline template
- `ml-pipeline` - Machine learning pipeline
- `parallel-processing` - Parallel task execution
- `conditional-workflow` - Conditional branching

### Controller Restarts (lines 92-97)

**Evidence:**
```bash
kubectl rollout restart deployment/workflow-controller -n argo
kubectl rollout restart deployment/argo-server -n argo
kubectl rollout status deployment/workflow-controller -n argo --timeout=120s
kubectl rollout status deployment/argo-server -n argo --timeout=120s
```

**Purpose:** Apply configuration changes (requires restart).

**Timeout:** 120 seconds per deployment.

---

## Safety Classification

**🟡 CAUTION** - Modifies Kubernetes cluster resources, restarts controllers.

---

## Idempotency

**✅ MOSTLY IDEMPOTENT** - `kubectl apply` is idempotent, rollout restart is safe to repeat.

**Non-idempotent:** Test workflow creation (line 132) creates duplicate workflows if run multiple times.

---

## Setup Steps

### 1. Check kubectl Connectivity (lines 25-33)

**Evidence:**
```bash
check_kubectl() {
    echo "Checking kubectl connectivity..."
    if ! kubectl get nodes > /dev/null 2>&1; then
        echo -e "${RED}Error: Cannot connect to Kubernetes cluster${NC}"
        echo "Make sure k3s is running: docker compose -f docker/docker-compose.argo.yml up -d"
        exit 1
    fi
    echo -e "${GREEN}✓ kubectl connectivity verified${NC}"
}
```

**Exit on failure:** Cannot proceed without cluster access.

### 2. Check Argo Installation (lines 36-45)

**Evidence:**
```bash
check_argo_installed() {
    echo "Checking Argo Workflows installation..."
    if ! kubectl get namespace argo > /dev/null 2>&1; then
        echo -e "${RED}Error: Argo namespace not found${NC}"
        echo "Run the installation first: docker compose -f docker/docker-compose.argo.yml up -d"
        exit 1
    fi
    echo -e "${GREEN}✓ Argo namespace exists${NC}"
}
```

**Prerequisite check:** Ensures base installation complete.

### 3. Apply Configurations (lines 48-65)

**RBAC (line 54):**
```bash
kubectl apply -f manifests/argo-workflows/config/rbac.yaml
```

**Controller ConfigMap (line 58):**
```bash
kubectl apply -f manifests/argo-workflows/config/workflow-controller-configmap.yaml
```

**Service Patch (line 62):**
```bash
kubectl patch svc argo-server -n argo -p '{"spec": {"type": "NodePort", "ports": [{"name": "web", "port": 2746, "targetPort": 2746, "nodePort": 32746, "protocol": "TCP"}]}}'
```

**Service type change:**
- **Before:** ClusterIP (internal only)
- **After:** NodePort (exposed on node IP:32746)

**|| true:** Line 62 has `|| true` to prevent failure if patch already applied.

### 4. Install Templates (lines 68-86)

**Evidence:**
```bash
templates=(
    "manifests/argo-workflows/templates/ci-pipeline.yaml"
    "manifests/argo-workflows/templates/ml-pipeline.yaml"
    "manifests/argo-workflows/templates/parallel-processing.yaml"
    "manifests/argo-workflows/templates/conditional-workflow.yaml"
)

for template in "${templates[@]}"; do
    template_name=$(basename "$template" .yaml)
    echo "  - Installing $template_name..."
    kubectl apply -f "$template"
done
```

**Template types:**
- **ci-pipeline:** Build, test, deploy stages
- **ml-pipeline:** Data prep, training, evaluation
- **parallel-processing:** Fan-out/fan-in patterns
- **conditional-workflow:** If/else logic

### 5. Restart Controllers (lines 89-100)

**Evidence:** Lines 92-97

**Rollout commands:**
- `rollout restart` - Triggers new deployment
- `rollout status` - Waits for completion (120s timeout)

**Both controllers restarted:**
- workflow-controller (executes workflows)
- argo-server (UI and API)

### 6. Verify Installation (lines 103-123)

**Checks:**
1. **Pods:** `kubectl get pods -n argo` (line 109)
2. **Services:** `kubectl get svc -n argo` (line 114)
3. **Templates:** `kubectl get workflowtemplate -n argo` (line 119)

**Purpose:** Visual confirmation of successful setup.

### 7. Optional Test Workflow (lines 126-149)

**Evidence:**
```bash
read -p "Do you want to run a test workflow? (y/n) " -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]; then
    kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo
    sleep 5
    kubectl get workflows -n argo
fi
```

**Interactive:** User decides whether to test.

**Note:** Uses `create` not `apply`, so running multiple times creates duplicate workflows.

### 8. Display Access Info (lines 152-182)

**Evidence:** Lines 158-181

**Displays:**
- UI URLs (localhost:2746, localhost:32746)
- Port forward command
- Common kubectl commands
- Argo CLI commands (if installed)
- Documentation links

---

## KUBECONFIG Handling

**Evidence:** Lines 19-22

```bash
if [ -z "$KUBECONFIG" ]; then
    export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml
    echo -e "${YELLOW}KUBECONFIG not set, using: $KUBECONFIG${NC}"
fi
```

**Default path:** Matches k3s cluster created by docker-compose.argo.yml.

---

## RBAC Configuration

**Purpose:** Allow workflows to create pods, access secrets, use persistent volumes.

**Resources created:**
- **ServiceAccount:** argo-workflow (identity for workflows)
- **Role:** argo-workflow-role (permissions within argo namespace)
- **RoleBinding:** argo-workflow-binding (links SA to role)

**Typical permissions:**
- pods: create, get, list, watch, delete
- configmaps: get, list, watch
- secrets: get, list
- persistentvolumeclaims: get, list, watch, create

---

## Workflow Templates

**Template types and purposes:**

### ci-pipeline.yaml
- Build application from source
- Run unit tests
- Run integration tests
- Push container image
- Deploy to staging

### ml-pipeline.yaml
- Load training data
- Preprocess features
- Train model
- Evaluate metrics
- Register model to MLflow

### parallel-processing.yaml
- Fan-out: Split data into chunks
- Parallel: Process each chunk independently
- Fan-in: Aggregate results

### conditional-workflow.yaml
- Check condition
- If true: Execute branch A
- If false: Execute branch B
- Merge: Continue with results

---

## Access Information

**UI Access (lines 160-162):**
```
Direct access: http://localhost:2746
NodePort access: http://localhost:32746
```

**Port forwarding (line 165):**
```bash
kubectl -n argo port-forward svc/argo-server 2746:2746
```

**Common commands (lines 167-171):**
```bash
kubectl get workflows -n argo
kubectl get workflow <name> -n argo -o yaml
kubectl logs -n argo <pod-name>
kubectl get workflowtemplate -n argo
```

**Argo CLI (lines 173-176):**
```bash
argo list -n argo
argo watch <workflow-name> -n argo
argo logs <workflow-name> -n argo
```

---

## Integration

**Part of:** P1-011 ARIA Infrastructure (Workflow Orchestration)

**Depends on:**
- k3s cluster running
- Argo Workflows base installation
- kubectl configured
- Manifest files in `manifests/argo-workflows/`

**Used with:**
- `verify-argo-workflows.sh` - Verify setup
- `deploy.sh` - May call this script
- CI/CD pipelines - Workflow execution

---

## References

### Source Code
- **Main script:** `scripts/setup-argo-workflows.sh` (198 lines)
- **kubectl check:** lines 25-33
- **Argo check:** lines 36-45
- **Apply configs:** lines 48-65
- **Install templates:** lines 68-86
- **Restart controllers:** lines 89-100
- **Verify:** lines 103-123
- **Test workflow:** lines 126-149

### Related Files
- **RBAC config:** `manifests/argo-workflows/config/rbac.yaml`
- **Controller config:** `manifests/argo-workflows/config/workflow-controller-configmap.yaml`
- **Templates:** `manifests/argo-workflows/templates/*.yaml`
- **Verification:** `scripts/verify-argo-workflows.sh`
- **Compose file:** `docker-compose.argo.yml` (or `docker/docker-compose.argo.yml`)

### External Resources
- [Argo Workflows](https://argoproj.github.io/workflows/)
- [WorkflowTemplate CRD](https://argo-workflows.readthedocs.io/en/latest/workflow-templates/)
- [Argo CLI](https://github.com/argoproj/argo-workflows/releases)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 28/60 contracts complete
