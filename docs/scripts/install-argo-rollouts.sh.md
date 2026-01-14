# Script Contract: install-argo-rollouts.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/install-argo-rollouts.sh`

---

## Purpose

Install, uninstall, or check status of Argo Rollouts for progressive delivery in Kubernetes. Applies Kustomize manifests, waits for deployment readiness, and provides dashboard access instructions. Supports blue/green, canary, and A/B testing deployments.

---

## Invocation

```bash
./scripts/install-argo-rollouts.sh [OPTIONS]
```

**Options:**
- `--install` - Install Argo Rollouts (default action)
- `--uninstall` - Remove Argo Rollouts from cluster
- `--status` - Check installation status and list rollouts
- `--help`, `-h` - Show usage information

**Examples:**
```bash
./scripts/install-argo-rollouts.sh             # Install
./scripts/install-argo-rollouts.sh --status    # Check status
./scripts/install-argo-rollouts.sh --uninstall # Remove
```

**Requirements:**
- kubectl installed and in PATH
- Active Kubernetes cluster connection
- Kustomize manifests at `manifests/kubernetes/base/delivery`

---

## Outputs

**Standard Output (install success):**
```
[INFO] Checking prerequisites...
[OK] Prerequisites satisfied
[INFO] Installing Argo Rollouts...
[OK] Argo Rollouts manifests applied
[INFO] Waiting for Argo Rollouts to be ready...
[OK] Argo Rollouts is ready

╔════════════════════════════════════════════════════════════════╗
║           Argo Rollouts Installed Successfully                 ║
╚════════════════════════════════════════════════════════════════╝

Access the Argo Rollouts dashboard:
  kubectl port-forward -n argo-rollouts svc/argo-rollouts-dashboard 3100:3100
  Open: http://localhost:3100

Install kubectl plugin:
  curl -LO https://github.com/argoproj/argo-rollouts/releases/latest/download/kubectl-argo-rollouts-linux-amd64
  chmod +x kubectl-argo-rollouts-linux-amd64
  sudo mv kubectl-argo-rollouts-linux-amd64 /usr/local/bin/kubectl-argo-rollouts

Example usage:
  kubectl argo rollouts list rollouts -A
  kubectl argo rollouts get rollout <rollout-name> --watch
```

**Standard Output (status):**
```
[INFO] Checking Argo Rollouts status...

[OK] Namespace: argo-rollouts exists

[INFO] Deployments:
NAME            READY   UP-TO-DATE   AVAILABLE   AGE
argo-rollouts   1/1     1            1           5m

[INFO] Pods:
NAME                             READY   STATUS    RESTARTS   AGE
argo-rollouts-5d8b8c7f8d-abc12   1/1     Running   0          5m

[INFO] Services:
NAME                        TYPE        CLUSTER-IP       EXTERNAL-IP   PORT(S)
argo-rollouts-dashboard     ClusterIP   10.43.100.50     <none>        3100/TCP

[OK] Rollouts CRD is installed

[INFO] Active Rollouts across all namespaces:
NAMESPACE   NAME            REPLICAS   DESIRED   READY   STATUS
production  app-rollout     5          5         5       Healthy
```

**Standard Output (uninstall):**
```
[WARN] Uninstalling Argo Rollouts...
[OK] Argo Rollouts uninstalled
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (action completed) |
| `1` | Error (kubectl missing, cluster unreachable, timeout, namespace not found) |

---

## Side Effects

### Install Action (lines 56-93)

**Kubernetes Resources Created (line 60):**
- Namespace: `argo-rollouts`
- Deployment: `argo-rollouts` controller
- Service: `argo-rollouts-dashboard`
- ServiceAccount, Role, RoleBinding (RBAC)
- CustomResourceDefinition: `rollouts.argoproj.io`

**Applied via Kustomize:** `kubectl apply -k manifests/kubernetes/base/delivery`

**Waiting:** 300-second timeout for deployment availability (lines 66-70)

### Uninstall Action (lines 96-104)

**Resources Deleted:** All resources from Kustomize manifest (line 99)

**Graceful failure:** Continues if resources don't exist (line 100)

### Status Action (lines 107-140)

**No side effects** - Read-only queries

---

## Safety Classification

**🟡 CAUTION** - Installs/removes cluster-wide resources, requires cluster access.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**
- Install: `kubectl apply` is idempotent, safe to re-run
- Uninstall: Safe if resources don't exist (error suppressed)
- Status: Fully idempotent (read-only)

---

## Command Structure

### Prerequisites Check (lines 39-53)

**Evidence:**
```bash
check_prereqs() {
    log_info "Checking prerequisites..."

    if ! command -v kubectl &>/dev/null; then
        log_error "kubectl not found. Please install kubectl."
        exit 1
    fi

    if ! kubectl cluster-info &>/dev/null; then
        log_error "Cannot connect to Kubernetes cluster. Please check your kubeconfig."
        exit 1
    fi

    log_success "Prerequisites satisfied"
}
```

**Checks:**
1. kubectl available in PATH (line 42)
2. Cluster accessible (line 47)

### Install Function (lines 56-93)

**Evidence:**
```bash
install() {
    log_info "Installing Argo Rollouts..."

    # Apply manifests
    kubectl apply -k "${MANIFESTS_DIR}"

    log_success "Argo Rollouts manifests applied"

    # Wait for deployment
    log_info "Waiting for Argo Rollouts to be ready..."
    kubectl wait --for=condition=available --timeout=300s \
        deployment/argo-rollouts -n "${NAMESPACE}" || {
        log_error "Timeout waiting for Argo Rollouts deployment"
        exit 1
    }

    log_success "Argo Rollouts is ready"
```

**Kustomize Application:** line 60 - applies all manifests in `manifests/kubernetes/base/delivery`

**Timeout:** 300 seconds (5 minutes) for deployment to become available (line 66)

**Success Box:** lines 76-92 - dashboard access instructions and kubectl plugin installation

### Uninstall Function (lines 96-104)

**Evidence:**
```bash
uninstall() {
    log_warn "Uninstalling Argo Rollouts..."

    kubectl delete -k "${MANIFESTS_DIR}" || {
        log_warn "Some resources may not exist"
    }

    log_success "Argo Rollouts uninstalled"
}
```

**Graceful failure:** line 100 - continues even if resources don't exist

### Status Function (lines 107-140)

**Evidence:**
```bash
status() {
    log_info "Checking Argo Rollouts status..."

    if kubectl get namespace "${NAMESPACE}" &>/dev/null; then
        log_success "Namespace: ${NAMESPACE} exists"
    else
        log_error "Namespace: ${NAMESPACE} not found"
        exit 1
    fi

    log_info "Deployments:"
    kubectl get deployments -n "${NAMESPACE}" -o wide

    log_info "Pods:"
    kubectl get pods -n "${NAMESPACE}" -o wide

    log_info "Services:"
    kubectl get services -n "${NAMESPACE}" -o wide

    # Check if rollouts CRD exists
    if kubectl get crd rollouts.argoproj.io &>/dev/null; then
        log_success "Rollouts CRD is installed"
        log_info "Active Rollouts across all namespaces:"
        kubectl get rollouts -A || log_warn "No rollouts found"
    else
        log_warn "Rollouts CRD not found..."
    fi
}
```

**Checks performed:**
1. Namespace exists (line 111)
2. Deployments status (line 120)
3. Pods status (line 124)
4. Services status (line 128)
5. Rollouts CRD presence (line 132)
6. Active rollouts in all namespaces (line 136)

---

## Configuration

**Evidence:** Lines 19-23

```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MANIFESTS_DIR="${PROJECT_ROOT}/manifests/kubernetes/base/delivery"
NAMESPACE="argo-rollouts"
```

**Manifests location:** `manifests/kubernetes/base/delivery` (Kustomize format)

**Target namespace:** `argo-rollouts` (line 23)

---

## Progressive Delivery Strategies

**Argo Rollouts supports:**

### Blue/Green Deployment
```yaml
strategy:
  blueGreen:
    activeService: app-active
    previewService: app-preview
    autoPromotionEnabled: true
```

### Canary Deployment
```yaml
strategy:
  canary:
    steps:
    - setWeight: 20
    - pause: {duration: 1m}
    - setWeight: 50
    - pause: {duration: 1m}
```

### A/B Testing
```yaml
strategy:
  canary:
    canaryService: app-canary
    trafficRouting:
      istio:
        virtualService:
          name: app-vsvc
```

---

## Dashboard Access

**Evidence:** Lines 80-82

```bash
echo "Access the Argo Rollouts dashboard:"
echo "  kubectl port-forward -n ${NAMESPACE} svc/argo-rollouts-dashboard 3100:3100"
echo "  Open: http://localhost:3100"
```

**Port forward command:**
```bash
kubectl port-forward -n argo-rollouts svc/argo-rollouts-dashboard 3100:3100
```

**Access URL:** http://localhost:3100

---

## kubectl Plugin Installation

**Evidence:** Lines 84-87

```bash
echo "Install kubectl plugin:"
echo "  curl -LO https://github.com/argoproj/argo-rollouts/releases/latest/download/kubectl-argo-rollouts-linux-amd64"
echo "  chmod +x kubectl-argo-rollouts-linux-amd64"
echo "  sudo mv kubectl-argo-rollouts-linux-amd64 /usr/local/bin/kubectl-argo-rollouts"
```

**Plugin commands:**
```bash
kubectl argo rollouts list rollouts -A
kubectl argo rollouts get rollout <rollout-name> --watch
kubectl argo rollouts promote <rollout-name>
kubectl argo rollouts abort <rollout-name>
```

---

## Integration with P1-006

**Evidence:** Lines 195-196

```bash
echo "║           Argo Rollouts Installation Script                    ║"
echo "║           P1-006: Progressive Delivery                          ║"
```

**Part of:** P1-006 ARIA Progressive Delivery

**Provides:**
- Automated canary releases
- Blue/green deployments
- A/B testing capabilities
- Gradual traffic shifting
- Automated rollbacks

---

## Troubleshooting

### kubectl Not Found

**Symptoms:** "kubectl not found" error

**Fix:**
```bash
# Install kubectl
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
chmod +x kubectl
sudo mv kubectl /usr/local/bin/
```

### Cluster Connection Failed

**Symptoms:** "Cannot connect to Kubernetes cluster"

**Fix:**
```bash
# Check KUBECONFIG
echo $KUBECONFIG

# Or set KUBECONFIG
export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml

# Verify connection
kubectl cluster-info
```

### Deployment Timeout

**Symptoms:** "Timeout waiting for Argo Rollouts deployment"

**Causes:**
- Image pull delays
- Insufficient cluster resources
- Network issues

**Debug:**
```bash
# Check pod status
kubectl get pods -n argo-rollouts

# Describe pod for events
kubectl describe pod -n argo-rollouts <pod-name>

# Check logs
kubectl logs -n argo-rollouts deployment/argo-rollouts
```

### CRD Not Found

**Symptoms:** "Rollouts CRD not found" warning

**Fix:**
```bash
# Reinstall with upstream manifests
kubectl create -f https://github.com/argoproj/argo-rollouts/releases/latest/download/install.yaml
```

---

## References

### Source Code
- **Main script:** `scripts/install-argo-rollouts.sh` (216 lines)
- **Prerequisites:** lines 39-53
- **Install function:** lines 56-93
- **Uninstall function:** lines 96-104
- **Status function:** lines 107-140
- **Main function:** lines 163-213

### Related Files
- **Kustomize manifests:** `manifests/kubernetes/base/delivery/`
- **Example rollouts:** `manifests/kubernetes/overlays/*/rollouts/`
- **Verification:** `scripts/verify-argo-rollouts.sh` (if exists)

### External Resources
- [Argo Rollouts Documentation](https://argo-rollouts.readthedocs.io/)
- [Progressive Delivery](https://www.weave.works/blog/progressive-delivery)
- [Kustomize](https://kubectl.docs.kubernetes.io/guides/introduction/kustomize/)
- [P1-006 ARIA Progressive Delivery](https://github.com/FlexNetOS/aria)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 35/60 contracts complete
