# Script Contract: verify-argo-workflows.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-argo-workflows.sh`

---

## Purpose

Verify Argo Workflows installation (P1-011, ARIA Infrastructure). Checks Kubernetes connectivity, namespace configuration, deployments, pods, services, RBAC, ConfigMaps, workflow templates, and network accessibility. Requires k3s cluster running.

---

## Invocation

```bash
./scripts/verify-argo-workflows.sh
```

**Environment Variables:**
- `ARGO_REQUIRE_CLUSTER=1` - Fail if cluster unreachable (default: 0, warnings only)
- `KUBECONFIG` - Path to kubeconfig (default: `./data/k3s/kubeconfig/kubeconfig.yaml`)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (0 failures) |
| `1` | Failed (1+ failures, or cluster unreachable if ARGO_REQUIRE_CLUSTER=1) |

### Counters
- Passed (green)
- Failed (red)
- Warnings (yellow)

---

## Side Effects

**Read-only:** Kubernetes inspection only, no modifications.

**KUBECONFIG default:** Sets `KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml` if not set (lines 42-45).

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no cluster modifications.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Verification Sections (12)

### 1. Kubernetes Connectivity (lines 74-89)
- `kubectl get nodes` test
- Early exit if cluster unavailable and `ARGO_REQUIRE_CLUSTER=0` (default)
- Fail if cluster required but unavailable

**Evidence:** Lines 75-89.

**Behavior:**
- **Cluster down + ARGO_REQUIRE_CLUSTER=0:** Warning, exit 0 (lines 85-89)
- **Cluster down + ARGO_REQUIRE_CLUSTER=1:** Fail, exit 1 (lines 79-82)

### 2. Namespace Verification (lines 93-110)
Checks for:
- **argo** namespace (required)
- **argocd** namespace (optional warning)
- **argo-rollouts** namespace (optional warning)

**Evidence:** Lines 94-109.

### 3. Argo Workflows Deployments (lines 113-134)
- **workflow-controller** deployment - Available status
- **argo-server** deployment - Available status

**Evidence:** Lines 114-133.

**Status check:**
```bash
status=$(kubectl get deployment workflow-controller -n argo \
  -o jsonpath='{.status.conditions[?(@.type=="Available")].status}')
```

### 4. Pod Status (lines 137-147)
- Lists all pods in argo namespace
- Counts running pods (expects >= 2)

**Evidence:** Lines 139-146.

### 5. Service Configuration (lines 150-161)
- **argo-server** service exists
- Service type (NodePort/LoadBalancer preferred for external access)

**Evidence:** Lines 151-160.

### 6. RBAC Configuration (lines 164-181)
Checks for:
- **argo-workflow** service account
- **argo-workflow-role** (role)
- **argo-workflow-binding** (rolebinding)

**Warning if missing:** Run `setup-argo-workflows.sh` (lines 168, 174, 180).

**Evidence:** Lines 165-180.

### 7. Configuration (lines 184-189)
- **workflow-controller-configmap** exists
- Warning if missing (uses defaults)

**Evidence:** Lines 185-188.

### 8. Workflow Templates (lines 192-200)
- Counts WorkflowTemplate resources in argo namespace
- Lists templates if found
- Warning if none (run setup-argo-workflows.sh)

**Evidence:** Lines 193-199.

### 9. Workflow Execution Test (lines 203-209)
- Checks `kubectl auth can-i create workflows.argoproj.io -n argo`
- Verifies RBAC permissions for workflow creation

**Evidence:** Lines 205-208.

### 10. Network Access (lines 212-217)
- Checks if port 2746 is listening (Argo Workflows UI)
- Uses `netstat` or `ss` for cross-platform support

**Evidence:** Lines 213-216.

### 11. Workflow History (lines 220-228)
- Counts existing workflows in argo namespace
- Lists recent workflows
- Info message if none found (normal for new installation)

**Evidence:** Lines 221-227.

### 12. Component Versions (lines 231-239)
Displays container images for:
- **workflow-controller** deployment
- **argo-server** deployment

**Evidence:** Lines 233-238.

---

## KUBECONFIG Handling

**Evidence:** Lines 42-45

**Default path:**
```bash
if [ -z "${KUBECONFIG:-}" ]; then
    export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml
    echo -e "${YELLOW}KUBECONFIG not set, using: $KUBECONFIG${NC}"
fi
```

**Location:** k3s creates kubeconfig at `/etc/rancher/k3s/k3s.yaml` and copies to data directory for external access.

---

## Compose File Resolution

**Evidence:** Lines 34-37

**Pattern:** Check subdirectory first, fallback to root
```bash
ARGO_COMPOSE_FILE="docker-compose.argo.yml"
if [ -f "docker/docker-compose.argo.yml" ]; then
    ARGO_COMPOSE_FILE="docker/docker-compose.argo.yml"
fi
```

---

## Docker Compose Command Resolution

**Evidence:** Lines 25-32

**Prefers v2 plugin:**
```bash
COMPOSE=("docker" "compose")
if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    if command -v docker-compose >/dev/null 2>&1; then
        COMPOSE=("docker-compose")
    else
        COMPOSE=()
    fi
fi
```

---

## Key Features

### 1. Graceful Cluster Unavailability

**Evidence:** Lines 79-89

**Default behavior (ARGO_REQUIRE_CLUSTER=0):**
- Warns if cluster down
- Skips live checks
- Exits 0 (success)
- Shows setup instructions

**Strict mode (ARGO_REQUIRE_CLUSTER=1):**
- Fails if cluster down
- Exits 1 (failure)
- Shows docker compose command

### 2. RBAC Verification

**Evidence:** Lines 165-180

**Checks three resources:**
1. Service account: `argo-workflow`
2. Role: `argo-workflow-role`
3. RoleBinding: `argo-workflow-binding`

**Purpose:** Enables workflows to create pods, access secrets, use PVCs.

### 3. WorkflowTemplate Discovery

**Evidence:** Lines 193-199

**Lists installed templates:**
```bash
template_count=$(kubectl get workflowtemplate -n argo --no-headers 2>/dev/null | wc -l)
kubectl get workflowtemplate -n argo
```

**Templates provide:** Reusable workflow patterns (hello-world, data-processing, ML training).

### 4. Port Accessibility Check

**Evidence:** Lines 213-216

**Cross-platform port check:**
```bash
if netstat -tuln 2>/dev/null | grep -q ":2746 " || \
   ss -tuln 2>/dev/null | grep -q ":2746 "; then
    # Port listening
fi
```

**Uses:** `netstat` (Linux/macOS) or `ss` (newer Linux).

---

## Next Steps

**Displayed on success (lines 252-268):**

**If warnings present:**
```bash
./scripts/setup-argo-workflows.sh
kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo
kubectl get workflows -n argo
```

**If clean:**
```bash
kubectl create -f manifests/argo-workflows/templates/hello-world.yaml -n argo
kubectl get workflows -n argo
http://localhost:2746
```

**Access UI:**
```bash
# Direct
http://localhost:2746

# Port forward
kubectl -n argo port-forward svc/argo-server 2746:2746
```

---

## Troubleshooting Steps

**Displayed on failure (lines 274-279):**
```bash
# 1. Check k3s
docker ps | grep k3s

# 2. View installer logs
docker compose -f docker-compose.argo.yml logs argocd-installer

# 3. Check controller logs
kubectl logs -n argo -l app=workflow-controller

# 4. Check server logs
kubectl logs -n argo -l app=argo-server

# 5. Restart installation
docker compose -f docker-compose.argo.yml down
docker compose -f docker-compose.argo.yml up -d
```

---

## Argo Workflows Context

**P1-011: ARIA Infrastructure - Workflow Orchestration**
- **Controller:** Manages workflow execution, CRD operator
- **Server:** REST API and UI (port 2746)
- **Namespace:** argo (dedicated)
- **Storage:** k3s local-path-provisioner for artifacts

**Integration Points:**
- NATS JetStream for event-driven workflows
- MindsDB for AI/ML task execution
- Vault for secrets management
- ArgoCD for GitOps deployment

---

## References

### Source Code
- **Main script:** `scripts/verify-argo-workflows.sh` (283 lines)
- **Connectivity check:** lines 74-89
- **Namespace check:** lines 93-110
- **Deployment check:** lines 113-134
- **RBAC check:** lines 164-181
- **Template check:** lines 192-200

### Related Files
- **Compose file:** `docker-compose.argo.yml` (or `docker/docker-compose.argo.yml`)
- **Kubeconfig:** `./data/k3s/kubeconfig/kubeconfig.yaml`
- **Templates:** `manifests/argo-workflows/templates/`
- **Setup script:** `scripts/setup-argo-workflows.sh`
- **Documentation:** `manifests/argo-workflows/README.md`

### External Resources
- [Argo Workflows](https://argoproj.github.io/workflows/)
- [k3s](https://k3s.io/) - Lightweight Kubernetes
- [WorkflowTemplate CRD](https://argo-workflows.readthedocs.io/en/latest/workflow-templates/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 17/60 contracts complete
