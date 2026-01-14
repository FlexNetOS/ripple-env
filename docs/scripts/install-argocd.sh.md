# Script Contract: install-argocd.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/install-argocd.sh`

---

## Purpose

Install Argo CD, Argo Rollouts, and Argo Workflows into k3s cluster. Waits for k3s availability, creates namespaces, applies latest stable manifests, patches ArgoCD service to NodePort, and displays initial admin password. Automated GitOps stack setup.

---

## Invocation

```bash
./scripts/install-argocd.sh
```

**Requirements:**
- k3s cluster running
- kubectl configured with cluster access
- Internet connectivity (downloads manifests from GitHub)

---

## Outputs

**Standard Output:**
```
Waiting for k3s to be ready...
  Still waiting...
NAME       STATUS   ROLES                  AGE   VERSION
k3s-node   Ready    control-plane,master   5m    v1.27.3+k3s1

Installing Argo CD...
namespace/argocd created
[Manifest installation output...]

Installing Argo Rollouts...
namespace/argo-rollouts created
[Manifest installation output...]

Installing Argo Workflows...
namespace/argo created
[Manifest installation output...]

Waiting for Argo CD to be ready...
deployment.apps/argocd-server condition met

Patching Argo CD service to NodePort...
service/argocd-server patched

=== Installation complete! ===

Initial admin password:
AbCdEfGh123456

Access Argo CD UI at: https://localhost:30443
Username: admin
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all components installed) |
| `1` | Error (k3s unavailable, kubectl failed, download failed, timeout) |

---

## Side Effects

### Kubernetes Resources Created

**Namespaces (lines 11, 15, 19):**
- `argocd` - Argo CD GitOps controller
- `argo-rollouts` - Progressive delivery controller
- `argo` - Argo Workflows orchestration

**Argo CD Components (line 12):**
- Deployments: argocd-server, argocd-repo-server, argocd-applicationset-controller, argocd-dex-server
- StatefulSet: argocd-application-controller
- Services: argocd-server, argocd-repo-server, argocd-dex-server
- ConfigMaps, Secrets, RBAC

**Argo Rollouts Components (line 16):**
- Deployment: argo-rollouts
- Service: argo-rollouts-metrics
- RBAC

**Argo Workflows Components (line 20):**
- Deployments: workflow-controller, argo-server
- Services: argo-server
- ConfigMaps, RBAC

### Service Modifications (line 26)

**ArgoCD Server Patch:**
```bash
kubectl patch svc argocd-server -n argocd -p '{"spec": {"type": "NodePort", "ports": [{"port": 443, "nodePort": 30443, "targetPort": 8080}]}}'
```

**Changes:**
- Type: ClusterIP → NodePort
- External port: 30443
- Target port: 8080 (ArgoCD serves on 8080, not 443)

---

## Safety Classification

**🟡 CAUTION** - Installs cluster-wide components, downloads manifests from internet.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**
- Namespaces: `|| true` prevents failure if exists (lines 11, 15, 19)
- Manifests: `kubectl apply` is idempotent
- Patch: Safe to run repeatedly

**Non-idempotent:** Creates duplicate resources if manifests change structure.

---

## Installation Steps

### 1. Wait for k3s Cluster (lines 4-8)

**Evidence:**
```bash
echo "Waiting for k3s to be ready..."
until kubectl get nodes 2>/dev/null; do
  echo "  Still waiting..."
  sleep 5
done
```

**Retry logic:** Infinite loop with 5-second intervals.

**Success condition:** `kubectl get nodes` returns successfully.

**Purpose:** Ensures cluster API is accessible before proceeding.

### 2. Install Argo CD (lines 10-12)

**Evidence:**
```bash
echo "Installing Argo CD..."
kubectl create namespace argocd || true
kubectl apply -n argocd -f https://raw.githubusercontent.com/argoproj/argo-cd/stable/manifests/install.yaml
```

**Manifest source:** GitHub stable release

**Components installed:**
- Application controller (manages applications)
- Repo server (connects to Git repositories)
- Server (REST API and UI)
- ApplicationSet controller (app templating)
- Dex (SSO/OAuth2)
- Redis (caching)

### 3. Install Argo Rollouts (lines 14-16)

**Evidence:**
```bash
echo "Installing Argo Rollouts..."
kubectl create namespace argo-rollouts || true
kubectl apply -n argo-rollouts -f https://github.com/argoproj/argo-rollouts/releases/latest/download/install.yaml
```

**Manifest source:** GitHub latest release

**Purpose:** Progressive delivery (blue/green, canary, A/B testing)

### 4. Install Argo Workflows (lines 18-20)

**Evidence:**
```bash
echo "Installing Argo Workflows..."
kubectl create namespace argo || true
kubectl apply -n argo -f https://github.com/argoproj/argo-workflows/releases/latest/download/install.yaml
```

**Manifest source:** GitHub latest release

**Purpose:** Container-native workflow orchestration

### 5. Wait for Argo CD Availability (lines 22-23)

**Evidence:**
```bash
echo "Waiting for Argo CD to be ready..."
kubectl wait --for=condition=available --timeout=300s deployment/argocd-server -n argocd
```

**Timeout:** 300 seconds (5 minutes)

**Condition:** Deployment Available (pods running and ready)

**Purpose:** Ensures Argo CD API is ready before patching service.

### 6. Patch Service to NodePort (lines 25-26)

**Evidence:**
```bash
echo "Patching Argo CD service to NodePort..."
kubectl patch svc argocd-server -n argocd -p '{"spec": {"type": "NodePort", "ports": [{"port": 443, "nodePort": 30443, "targetPort": 8080}]}}'
```

**Purpose:** Expose Argo CD UI externally (accessible at https://localhost:30443)

**Note:** targetPort 8080 (ArgoCD server port, not 443)

### 7. Display Access Information (lines 28-37)

**Evidence:**
```bash
echo ""
echo "=== Installation complete! ==="
echo ""
echo "Initial admin password:"
kubectl -n argocd get secret argocd-initial-admin-secret -o jsonpath="{.data.password}" | base64 -d
echo ""
echo ""
echo "Access Argo CD UI at: https://localhost:30443"
echo "Username: admin"
echo ""
```

**Password extraction:** Decodes base64-encoded password from secret.

---

## Argo CD Initial Setup

### Access UI

**URL:** https://localhost:30443
**Username:** admin
**Password:** From script output (or extract manually)

**First login:**
1. Accept self-signed certificate warning
2. Login with admin credentials
3. Change admin password (Settings → Account → Update Password)

### Create First Application

```bash
# Via UI: New App button
# Via CLI:
argocd app create guestbook \
  --repo https://github.com/argoproj/argocd-example-apps.git \
  --path guestbook \
  --dest-server https://kubernetes.default.svc \
  --dest-namespace default
```

---

## Argo Rollouts Setup

### Install kubectl Plugin

```bash
# Download plugin
curl -LO https://github.com/argoproj/argo-rollouts/releases/latest/download/kubectl-argo-rollouts-linux-amd64

# Install
chmod +x kubectl-argo-rollouts-linux-amd64
sudo mv kubectl-argo-rollouts-linux-amd64 /usr/local/bin/kubectl-argo-rollouts

# Verify
kubectl argo rollouts version
```

### Dashboard Access

```bash
# Port forward
kubectl argo rollouts dashboard

# Access at http://localhost:3100
```

---

## Argo Workflows Setup

### Access UI

```bash
# Port forward
kubectl -n argo port-forward svc/argo-server 2746:2746

# Access at https://localhost:2746
```

### Submit Test Workflow

```bash
# Install argo CLI
curl -sLO https://github.com/argoproj/argo-workflows/releases/latest/download/argo-linux-amd64.gz
gunzip argo-linux-amd64.gz
chmod +x argo-linux-amd64
sudo mv argo-linux-amd64 /usr/local/bin/argo

# Submit workflow
argo submit -n argo --watch https://raw.githubusercontent.com/argoproj/argo-workflows/master/examples/hello-world.yaml
```

---

## Version Pinning (Future Enhancement)

**Current:** Uses `stable` and `latest` releases (may change over time)

**Recommended for production:**
```bash
# Pin to specific versions
ARGOCD_VERSION="v2.8.4"
ROLLOUTS_VERSION="v1.6.0"
WORKFLOWS_VERSION="v3.5.0"

kubectl apply -n argocd -f "https://raw.githubusercontent.com/argoproj/argo-cd/${ARGOCD_VERSION}/manifests/install.yaml"
kubectl apply -n argo-rollouts -f "https://github.com/argoproj/argo-rollouts/releases/download/${ROLLOUTS_VERSION}/install.yaml"
kubectl apply -n argo -f "https://github.com/argoproj/argo-workflows/releases/download/${WORKFLOWS_VERSION}/install.yaml"
```

---

## Troubleshooting

### k3s Not Ready

**Symptoms:** "Still waiting..." loops indefinitely

**Causes:**
- k3s not started
- KUBECONFIG not set
- kubectl not installed

**Fix:**
```bash
# Check k3s status
docker ps | grep k3s

# Set KUBECONFIG
export KUBECONFIG=./data/k3s/kubeconfig/kubeconfig.yaml

# Start k3s
docker compose -f docker-compose.argo.yml up -d
```

### Argo CD Timeout

**Symptoms:** "error: timed out waiting for the condition"

**Causes:**
- Image pull delays
- Insufficient resources
- Network issues

**Fix:**
```bash
# Check pod status
kubectl get pods -n argocd

# View pod events
kubectl describe pod -n argocd <pod-name>

# Increase timeout
kubectl wait --timeout=600s ...
```

### Service Patch Fails

**Symptoms:** "Error from server (NotFound): services "argocd-server" not found"

**Causes:**
- ArgoCD not fully installed
- Namespace mismatch

**Fix:**
```bash
# Verify service exists
kubectl get svc -n argocd

# Wait longer for installation
kubectl wait --timeout=600s deployment/argocd-server -n argocd
```

---

## Integration with ARIA

**Part of:** P1-011 ARIA Infrastructure (GitOps & Progressive Delivery)

**Provides:**
- GitOps continuous delivery (Argo CD)
- Progressive rollouts (Argo Rollouts)
- Workflow orchestration (Argo Workflows)

**Used with:**
- `setup-argo-workflows.sh` - Configure workflows
- `verify-argo-workflows.sh` - Verify installation
- Git repositories for application manifests

---

## References

### Source Code
- **Main script:** `scripts/install-argocd.sh` (38 lines)
- **k3s wait:** lines 4-8
- **Argo CD install:** lines 10-12
- **Argo Rollouts install:** lines 14-16
- **Argo Workflows install:** lines 18-20
- **Service patch:** lines 25-26

### Related Files
- **Argo Workflows setup:** `scripts/setup-argo-workflows.sh`
- **Verification:** `scripts/verify-argo-workflows.sh`
- **Compose file:** `docker-compose.argo.yml`

### External Resources
- [Argo CD](https://argo-cd.readthedocs.io/)
- [Argo Rollouts](https://argo-rollouts.readthedocs.io/)
- [Argo Workflows](https://argo-workflows.readthedocs.io/)
- [k3s](https://k3s.io/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 33/60 contracts complete
