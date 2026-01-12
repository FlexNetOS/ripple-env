# P0-P1 Task Completion Prompt

> **Target Model**: Claude Sonnet 4.5
> **Estimated Duration**: ~40 hours of work
> **Priority**: Complete P0 tasks first, then P1 in order
> **Branch**: Create new branch from `claude/kubernetes-helm-setup-GRZEL`

---

## Context

You are working on the `FlexNetOS/ripple-env` repository, a ROS2 Humble development environment built with Nix flakes and Pixi. An ARIA audit identified 20 P0-P1 tasks needed to reach 95%+ compliance with BUILDKIT_STARTER_SPEC.md.

**Key Files:**
- `flake.nix` - Main Nix flake (K8s tools behind feature flag)
- `pixi.toml` - Python/Conda packages
- `manifests/kubernetes/` - K8s manifests (Kustomize structure)
- `charts/flexstack/` - Helm charts
- `docker/docker-compose.yml` - Docker services
- `BUILDKIT_STARTER_SPEC.md` - Single Source of Truth

**Current Branch**: `claude/kubernetes-helm-setup-GRZEL`

---

## Instructions

Complete ALL 20 tasks below in order. For each task:
1. Read relevant files before making changes
2. Make minimal, focused changes
3. Test changes work (syntax validation, dry-run where possible)
4. Commit with descriptive message after each task or logical group

---

## P0 Tasks (5 tasks) — Do These First

### Task 1: Add kubectl/helm/kustomize to default devShell
**File**: `flake.nix`
**Action**: Move K8s tools from feature flag to default devShell packages

Find the section where `kubectl`, `helm`, and `kustomize` are behind a feature flag (likely in `k8sPackages` or similar). Add them to `commonPackages` or the default devShell's `packages` list.

```nix
# Add to commonPackages or default devShell:
kubectl
kubernetes-helm
kustomize
argocd  # Also add ArgoCD CLI
```

Verify with: `nix flake check --no-build`

---

### Task 2: Create data/localai/models directory
**Action**: Create directory structure for LocalAI models

```bash
mkdir -p data/localai/models
```

Create a `.gitkeep` file and a `README.md`:

**File**: `data/localai/models/README.md`
```markdown
# LocalAI Models Directory

Place GGUF model files here for LocalAI inference.

## Recommended Models

| Model | Size | Use Case |
|-------|------|----------|
| `phi-2.Q4_K_M.gguf` | 1.6GB | Fast general-purpose |
| `mistral-7b-instruct-v0.2.Q4_K_M.gguf` | 4.4GB | High-quality instruction following |
| `llama-2-7b-chat.Q4_K_M.gguf` | 4.1GB | Chat applications |

## Download Commands

```bash
# Using huggingface-cli (install via: pip install huggingface-hub)
huggingface-cli download TheBloke/phi-2-GGUF phi-2.Q4_K_M.gguf --local-dir .
huggingface-cli download TheBloke/Mistral-7B-Instruct-v0.2-GGUF mistral-7b-instruct-v0.2.Q4_K_M.gguf --local-dir .
```

## Volume Mount

This directory is mounted to `/models` in the LocalAI container.
```

Add to `.gitignore`:
```
data/localai/models/*.gguf
data/localai/models/*.bin
```

---

### Task 3: Create model download script
**File**: `scripts/download-localai-models.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# LocalAI Model Downloader
# Downloads recommended GGUF models for local inference

MODELS_DIR="${1:-data/localai/models}"
mkdir -p "$MODELS_DIR"

echo "📦 Downloading LocalAI models to $MODELS_DIR..."

# Check for huggingface-cli
if ! command -v huggingface-cli &> /dev/null; then
    echo "Installing huggingface-hub..."
    pip install --quiet huggingface-hub
fi

# Download phi-2 (small, fast)
if [ ! -f "$MODELS_DIR/phi-2.Q4_K_M.gguf" ]; then
    echo "⬇️  Downloading phi-2..."
    huggingface-cli download TheBloke/phi-2-GGUF phi-2.Q4_K_M.gguf --local-dir "$MODELS_DIR"
fi

echo "✅ Models downloaded successfully"
echo ""
echo "Models available:"
ls -lh "$MODELS_DIR"/*.gguf 2>/dev/null || echo "  (none yet)"
```

Make executable: `chmod +x scripts/download-localai-models.sh`

---

### Task 4: Add NATS authentication to K8s manifest
**File**: `manifests/kubernetes/base/messaging/nats.yaml`

Add Secret and update NATS deployment to use authentication:

```yaml
---
apiVersion: v1
kind: Secret
metadata:
  name: nats-credentials
  namespace: flexstack-messaging
type: Opaque
stringData:
  # System account for internal services
  NATS_USER: "flexstack"
  NATS_PASSWORD: "changeme-in-production"
  # Operator signing key (generate with: nsc generate nkey --operator)
  OPERATOR_SIGNING_KEY: ""
---
apiVersion: v1
kind: ConfigMap
metadata:
  name: nats-config
  namespace: flexstack-messaging
data:
  nats.conf: |
    port: 4222
    http_port: 8222

    jetstream {
      store_dir: /data/jetstream
      max_mem: 1Gi
      max_file: 10Gi
    }

    authorization {
      users = [
        { user: "flexstack", password: "$NATS_PASSWORD" }
      ]
    }

    # Cluster configuration (for HA)
    cluster {
      name: flexstack-nats
      port: 6222
    }
```

Update the NATS Deployment to mount the config and secret.

---

### Task 5: Move BUILDKIT_STARTER_SPEC.md to docs/
**Action**: Move file and update references

```bash
git mv BUILDKIT_STARTER_SPEC.md docs/BUILDKIT_STARTER_SPEC.md
```

Update references in:
- `README.md` - Update any links
- `.claude/CLAUDE.md` - Update file path references
- Any other files referencing the spec

---

## P1 Tasks (15 tasks)

### Task 6: Create Argo Rollouts manifests
**Directory**: `manifests/kubernetes/base/delivery/`

Create progressive delivery manifests:

**File**: `manifests/kubernetes/base/delivery/kustomization.yaml`
```yaml
apiVersion: kustomize.config.k8s.io/v1beta1
kind: Kustomization
namespace: flexstack-delivery
resources:
  - namespace.yaml
  - argo-rollouts.yaml
  - analysis-templates.yaml
```

**File**: `manifests/kubernetes/base/delivery/namespace.yaml`
```yaml
apiVersion: v1
kind: Namespace
metadata:
  name: flexstack-delivery
  labels:
    app.kubernetes.io/part-of: flexstack
```

**File**: `manifests/kubernetes/base/delivery/argo-rollouts.yaml`
```yaml
# Argo Rollouts controller deployment
# Install CRDs separately: kubectl apply -f https://github.com/argoproj/argo-rollouts/releases/latest/download/install.yaml
---
apiVersion: v1
kind: ServiceAccount
metadata:
  name: argo-rollouts
  namespace: flexstack-delivery
---
apiVersion: rbac.authorization.k8s.io/v1
kind: ClusterRole
metadata:
  name: argo-rollouts
rules:
  - apiGroups: ["argoproj.io"]
    resources: ["rollouts", "rollouts/status", "rollouts/finalizers"]
    verbs: ["*"]
  - apiGroups: ["argoproj.io"]
    resources: ["analysisruns", "analysistemplates", "experiments"]
    verbs: ["*"]
  - apiGroups: [""]
    resources: ["services", "endpoints", "pods", "secrets", "configmaps"]
    verbs: ["get", "list", "watch", "patch", "update"]
  - apiGroups: ["apps"]
    resources: ["deployments", "replicasets"]
    verbs: ["*"]
---
apiVersion: rbac.authorization.k8s.io/v1
kind: ClusterRoleBinding
metadata:
  name: argo-rollouts
subjects:
  - kind: ServiceAccount
    name: argo-rollouts
    namespace: flexstack-delivery
roleRef:
  kind: ClusterRole
  name: argo-rollouts
  apiGroup: rbac.authorization.k8s.io
```

**File**: `manifests/kubernetes/base/delivery/analysis-templates.yaml`
```yaml
# Analysis templates for canary deployments
apiVersion: argoproj.io/v1alpha1
kind: AnalysisTemplate
metadata:
  name: success-rate
  namespace: flexstack-delivery
spec:
  metrics:
    - name: success-rate
      interval: 1m
      successCondition: result[0] >= 0.95
      failureLimit: 3
      provider:
        prometheus:
          address: http://prometheus.flexstack-observability:9090
          query: |
            sum(rate(http_requests_total{status=~"2.*"}[5m])) /
            sum(rate(http_requests_total[5m]))
---
apiVersion: argoproj.io/v1alpha1
kind: AnalysisTemplate
metadata:
  name: latency-check
  namespace: flexstack-delivery
spec:
  metrics:
    - name: p99-latency
      interval: 1m
      successCondition: result[0] < 500
      failureLimit: 3
      provider:
        prometheus:
          address: http://prometheus.flexstack-observability:9090
          query: |
            histogram_quantile(0.99, sum(rate(http_request_duration_seconds_bucket[5m])) by (le))
```

---

### Task 7: Add Argo Rollouts to install script
**File**: `scripts/install-argo-rollouts.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# Install Argo Rollouts controller and CRDs

NAMESPACE="${1:-argo-rollouts}"
VERSION="${2:-v1.7.2}"

echo "📦 Installing Argo Rollouts $VERSION..."

# Install CRDs and controller
kubectl create namespace "$NAMESPACE" --dry-run=client -o yaml | kubectl apply -f -
kubectl apply -n "$NAMESPACE" -f "https://github.com/argoproj/argo-rollouts/releases/download/$VERSION/install.yaml"

# Install kubectl plugin
if command -v kubectl-argo-rollouts &> /dev/null; then
    echo "✅ kubectl-argo-rollouts already installed"
else
    echo "📥 Installing kubectl-argo-rollouts plugin..."
    curl -LO "https://github.com/argoproj/argo-rollouts/releases/download/$VERSION/kubectl-argo-rollouts-linux-amd64"
    chmod +x kubectl-argo-rollouts-linux-amd64
    sudo mv kubectl-argo-rollouts-linux-amd64 /usr/local/bin/kubectl-argo-rollouts
fi

echo "✅ Argo Rollouts installed successfully"
kubectl argo rollouts version
```

---

### Task 8: Create Step-CA K8s manifest
**File**: `manifests/kubernetes/base/identity/step-ca.yaml`

```yaml
---
apiVersion: v1
kind: Secret
metadata:
  name: step-ca-credentials
  namespace: flexstack-identity
type: Opaque
stringData:
  PASSWORD: "changeme-step-ca-password"
---
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: step-ca-data
  namespace: flexstack-identity
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 1Gi
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: step-ca
  namespace: flexstack-identity
  labels:
    app.kubernetes.io/name: step-ca
    app.kubernetes.io/component: pki
spec:
  replicas: 1
  selector:
    matchLabels:
      app.kubernetes.io/name: step-ca
  template:
    metadata:
      labels:
        app.kubernetes.io/name: step-ca
    spec:
      containers:
        - name: step-ca
          image: smallstep/step-ca:0.27.4
          ports:
            - containerPort: 9000
              name: https
          env:
            - name: DOCKER_STEPCA_INIT_NAME
              value: "FlexStack CA"
            - name: DOCKER_STEPCA_INIT_DNS_NAMES
              value: "step-ca,step-ca.flexstack-identity.svc.cluster.local"
            - name: DOCKER_STEPCA_INIT_PASSWORD
              valueFrom:
                secretKeyRef:
                  name: step-ca-credentials
                  key: PASSWORD
          volumeMounts:
            - name: step-ca-data
              mountPath: /home/step
          resources:
            limits:
              memory: 256Mi
              cpu: "500m"
            requests:
              memory: 128Mi
              cpu: "100m"
          livenessProbe:
            httpGet:
              path: /health
              port: 9000
              scheme: HTTPS
            initialDelaySeconds: 30
          readinessProbe:
            httpGet:
              path: /health
              port: 9000
              scheme: HTTPS
      volumes:
        - name: step-ca-data
          persistentVolumeClaim:
            claimName: step-ca-data
---
apiVersion: v1
kind: Service
metadata:
  name: step-ca
  namespace: flexstack-identity
spec:
  type: ClusterIP
  ports:
    - port: 9000
      targetPort: 9000
      name: https
  selector:
    app.kubernetes.io/name: step-ca
```

Update `manifests/kubernetes/base/identity/kustomization.yaml` to include `step-ca.yaml`.

---

### Task 9: Create Vaultwarden K8s manifest
**File**: `manifests/kubernetes/base/identity/vaultwarden.yaml`

```yaml
---
apiVersion: v1
kind: Secret
metadata:
  name: vaultwarden-credentials
  namespace: flexstack-identity
type: Opaque
stringData:
  ADMIN_TOKEN: "changeme-admin-token"
---
apiVersion: v1
kind: PersistentVolumeClaim
metadata:
  name: vaultwarden-data
  namespace: flexstack-identity
spec:
  accessModes:
    - ReadWriteOnce
  resources:
    requests:
      storage: 5Gi
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: vaultwarden
  namespace: flexstack-identity
  labels:
    app.kubernetes.io/name: vaultwarden
    app.kubernetes.io/component: secrets
spec:
  replicas: 1
  selector:
    matchLabels:
      app.kubernetes.io/name: vaultwarden
  template:
    metadata:
      labels:
        app.kubernetes.io/name: vaultwarden
    spec:
      containers:
        - name: vaultwarden
          image: vaultwarden/server:1.32.4
          ports:
            - containerPort: 80
              name: http
          env:
            - name: ADMIN_TOKEN
              valueFrom:
                secretKeyRef:
                  name: vaultwarden-credentials
                  key: ADMIN_TOKEN
            - name: WEBSOCKET_ENABLED
              value: "true"
            - name: SIGNUPS_ALLOWED
              value: "false"
          volumeMounts:
            - name: vaultwarden-data
              mountPath: /data
          resources:
            limits:
              memory: 512Mi
              cpu: "1"
            requests:
              memory: 128Mi
              cpu: "100m"
          livenessProbe:
            httpGet:
              path: /alive
              port: 80
          readinessProbe:
            httpGet:
              path: /alive
              port: 80
      volumes:
        - name: vaultwarden-data
          persistentVolumeClaim:
            claimName: vaultwarden-data
---
apiVersion: v1
kind: Service
metadata:
  name: vaultwarden
  namespace: flexstack-identity
spec:
  type: ClusterIP
  ports:
    - port: 80
      targetPort: 80
      name: http
  selector:
    app.kubernetes.io/name: vaultwarden
```

Update `manifests/kubernetes/base/identity/kustomization.yaml` to include `vaultwarden.yaml`.

---

### Task 10: Add Netdata K8s manifest
**File**: `manifests/kubernetes/base/observability/netdata.yaml`

```yaml
---
apiVersion: apps/v1
kind: DaemonSet
metadata:
  name: netdata
  namespace: flexstack-observability
  labels:
    app.kubernetes.io/name: netdata
    app.kubernetes.io/component: monitoring
spec:
  selector:
    matchLabels:
      app.kubernetes.io/name: netdata
  template:
    metadata:
      labels:
        app.kubernetes.io/name: netdata
      annotations:
        prometheus.io/scrape: "true"
        prometheus.io/port: "19999"
    spec:
      hostNetwork: true
      hostPID: true
      containers:
        - name: netdata
          image: netdata/netdata:v1.47.5
          ports:
            - containerPort: 19999
              name: http
          securityContext:
            capabilities:
              add:
                - SYS_PTRACE
                - SYS_ADMIN
          volumeMounts:
            - name: proc
              mountPath: /host/proc
              readOnly: true
            - name: sys
              mountPath: /host/sys
              readOnly: true
            - name: varrun
              mountPath: /var/run/docker.sock
              readOnly: true
          resources:
            limits:
              memory: 512Mi
              cpu: "1"
            requests:
              memory: 256Mi
              cpu: "250m"
      volumes:
        - name: proc
          hostPath:
            path: /proc
        - name: sys
          hostPath:
            path: /sys
        - name: varrun
          hostPath:
            path: /var/run/docker.sock
---
apiVersion: v1
kind: Service
metadata:
  name: netdata
  namespace: flexstack-observability
spec:
  type: ClusterIP
  ports:
    - port: 19999
      targetPort: 19999
      name: http
  selector:
    app.kubernetes.io/name: netdata
```

Update `manifests/kubernetes/base/observability/kustomization.yaml`.

---

### Task 11: Add Umami K8s manifest
**File**: `manifests/kubernetes/base/observability/umami.yaml`

```yaml
---
apiVersion: v1
kind: Secret
metadata:
  name: umami-credentials
  namespace: flexstack-observability
type: Opaque
stringData:
  DATABASE_URL: "postgresql://postgres:changeme@postgres.flexstack.svc.cluster.local:5432/umami"
  HASH_SALT: "changeme-random-salt"
---
apiVersion: apps/v1
kind: Deployment
metadata:
  name: umami
  namespace: flexstack-observability
  labels:
    app.kubernetes.io/name: umami
    app.kubernetes.io/component: analytics
spec:
  replicas: 1
  selector:
    matchLabels:
      app.kubernetes.io/name: umami
  template:
    metadata:
      labels:
        app.kubernetes.io/name: umami
    spec:
      containers:
        - name: umami
          image: ghcr.io/umami-software/umami:postgresql-v2.14.0
          ports:
            - containerPort: 3000
              name: http
          envFrom:
            - secretRef:
                name: umami-credentials
          resources:
            limits:
              memory: 512Mi
              cpu: "1"
            requests:
              memory: 256Mi
              cpu: "250m"
          livenessProbe:
            httpGet:
              path: /api/heartbeat
              port: 3000
          readinessProbe:
            httpGet:
              path: /api/heartbeat
              port: 3000
---
apiVersion: v1
kind: Service
metadata:
  name: umami
  namespace: flexstack-observability
spec:
  type: ClusterIP
  ports:
    - port: 3000
      targetPort: 3000
      name: http
  selector:
    app.kubernetes.io/name: umami
```

Update `manifests/kubernetes/base/observability/kustomization.yaml`.

---

### Task 12: Add tempo.yaml to kustomization
**File**: `manifests/kubernetes/base/observability/kustomization.yaml`

Ensure `tempo.yaml` is listed in resources if it exists but isn't included.

---

### Task 13: Add agentic-flow to package.json
**File**: `package.json` (create if doesn't exist)

```json
{
  "name": "ripple-env",
  "version": "1.0.0",
  "private": true,
  "description": "FlexStack agent frameworks",
  "scripts": {
    "agent:flow": "npx agentic-flow",
    "agent:claude": "npx @anthropics/claude-flow"
  },
  "dependencies": {
    "agentic-flow": "^0.1.0"
  },
  "devDependencies": {},
  "engines": {
    "node": ">=20.0.0"
  }
}
```

---

### Task 14: Add claude-flow to package.json
Update the `package.json` from Task 13:

```json
{
  "dependencies": {
    "agentic-flow": "^0.1.0",
    "@anthropics/claude-flow": "^0.1.0"
  }
}
```

---

### Task 15: Create Node.js wrapper scripts
**File**: `scripts/agent-flow.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# Wrapper for agentic-flow
cd "$(dirname "$0")/.."

if [ ! -d "node_modules" ]; then
    echo "Installing Node.js dependencies..."
    pnpm install
fi

exec pnpm exec agentic-flow "$@"
```

**File**: `scripts/claude-flow.sh`

```bash
#!/usr/bin/env bash
set -euo pipefail

# Wrapper for claude-flow
cd "$(dirname "$0")/.."

if [ ! -d "node_modules" ]; then
    echo "Installing Node.js dependencies..."
    pnpm install
fi

exec pnpm exec claude-flow "$@"
```

Make executable: `chmod +x scripts/agent-flow.sh scripts/claude-flow.sh`

---

### Task 16: Implement Holochain DNA placeholder
**File**: `manifests/holochain/dnas/flexstack-agent/dna.yaml`

```yaml
# FlexStack Agent DNA Configuration
# For agent coordination via Holochain DHT

manifest_version: "1"
name: flexstack-agent
uid: ~
properties: ~
zomes:
  - name: agent_registry
    bundled: ../../zomes/agent_registry.wasm
    hash: ~
  - name: task_coordination
    bundled: ../../zomes/task_coordination.wasm
    hash: ~
```

**File**: `manifests/holochain/dnas/flexstack-agent/README.md`

```markdown
# FlexStack Agent DNA

Holochain DNA for decentralized agent coordination.

## Zomes

- `agent_registry`: Register and discover agents
- `task_coordination`: Distributed task assignment

## Building

```bash
# Requires Holochain development environment
nix develop
hc dna pack .
```

## Status

🚧 **Placeholder** - Zome implementations pending
```

---

### Task 17: Enable mTLS documentation
**File**: `docs/deployment/MTLS_SETUP.md`

```markdown
# mTLS Configuration Guide

Enable mutual TLS between FlexStack services.

## Prerequisites

- Step-CA deployed (see K8s identity manifests)
- Certificates generated for each service

## Docker Compose

Add to service environment:

```yaml
services:
  your-service:
    environment:
      - TLS_CERT_FILE=/certs/tls.crt
      - TLS_KEY_FILE=/certs/tls.key
      - TLS_CA_FILE=/certs/ca.crt
    volumes:
      - ./certs:/certs:ro
```

## Kubernetes

Use cert-manager with Step-CA issuer:

```yaml
apiVersion: cert-manager.io/v1
kind: Certificate
metadata:
  name: service-cert
spec:
  secretName: service-tls
  issuerRef:
    name: step-ca-issuer
    kind: ClusterIssuer
  dnsNames:
    - service.namespace.svc.cluster.local
```

## Status

⚠️ **Not enabled by default** - Enable per environment needs
```

---

### Task 18: Create QA/Staging K8s overlays
**Directory**: `manifests/kubernetes/overlays/staging/`

**File**: `manifests/kubernetes/overlays/staging/kustomization.yaml`

```yaml
apiVersion: kustomize.config.k8s.io/v1beta1
kind: Kustomization

namespace: flexstack-staging

resources:
  - ../../base

namePrefix: staging-

commonLabels:
  environment: staging

patches:
  - patch: |-
      - op: replace
        path: /spec/replicas
        value: 2
    target:
      kind: Deployment
      labelSelector: "app.kubernetes.io/part-of=flexstack"
```

**File**: `manifests/kubernetes/overlays/qa/kustomization.yaml`

```yaml
apiVersion: kustomize.config.k8s.io/v1beta1
kind: Kustomization

namespace: flexstack-qa

resources:
  - ../../base

namePrefix: qa-

commonLabels:
  environment: qa

patches:
  - patch: |-
      - op: replace
        path: /spec/replicas
        value: 1
    target:
      kind: Deployment
      labelSelector: "app.kubernetes.io/part-of=flexstack"
```

---

### Task 19: Add TruLens runtime integration
**File**: `config/trulens/config.yaml`

```yaml
# TruLens Configuration for LLM Evaluation
# See: https://www.trulens.org/

database:
  type: postgresql
  host: postgres.flexstack.svc.cluster.local
  port: 5432
  database: trulens

feedback:
  providers:
    - name: openai
      api_base: http://localai.flexstack-ai.svc.cluster.local:8080/v1
    - name: bedrock
      enabled: false

app:
  default_record_ingest: true
  default_feedback_mode: deferred
```

**File**: `pixi.toml` addition

Add to dependencies:
```toml
trulens-eval = ">=1.0.0"
```

---

### Task 20: Create finetuning example script
**File**: `scripts/finetune-example.py`

```python
#!/usr/bin/env python3
"""
Example finetuning script for FlexStack.

Demonstrates how to finetune a model using the local infrastructure.
"""

import os
from pathlib import Path

# MLflow for experiment tracking
import mlflow

# Transformers for model training
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    TrainingArguments,
    Trainer,
    DataCollatorForLanguageModeling,
)
from datasets import load_dataset

# Configuration
MODEL_NAME = os.getenv("MODEL_NAME", "microsoft/phi-2")
OUTPUT_DIR = Path(os.getenv("OUTPUT_DIR", "data/finetuned-models"))
MLFLOW_TRACKING_URI = os.getenv("MLFLOW_TRACKING_URI", "http://localhost:5000")

def main():
    """Run finetuning example."""
    # Setup MLflow
    mlflow.set_tracking_uri(MLFLOW_TRACKING_URI)
    mlflow.set_experiment("flexstack-finetune")

    with mlflow.start_run():
        # Log parameters
        mlflow.log_param("model_name", MODEL_NAME)
        mlflow.log_param("output_dir", str(OUTPUT_DIR))

        # Load tokenizer and model
        print(f"Loading model: {MODEL_NAME}")
        tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
        tokenizer.pad_token = tokenizer.eos_token

        model = AutoModelForCausalLM.from_pretrained(
            MODEL_NAME,
            device_map="auto",
            trust_remote_code=True,
        )

        # Load example dataset
        dataset = load_dataset("wikitext", "wikitext-2-raw-v1", split="train[:1000]")

        def tokenize_function(examples):
            return tokenizer(
                examples["text"],
                truncation=True,
                max_length=512,
                padding="max_length",
            )

        tokenized_dataset = dataset.map(
            tokenize_function,
            batched=True,
            remove_columns=dataset.column_names,
        )

        # Training arguments
        training_args = TrainingArguments(
            output_dir=str(OUTPUT_DIR),
            num_train_epochs=1,
            per_device_train_batch_size=4,
            gradient_accumulation_steps=4,
            logging_steps=10,
            save_steps=100,
            learning_rate=2e-5,
            fp16=True,
            report_to="mlflow",
        )

        # Data collator
        data_collator = DataCollatorForLanguageModeling(
            tokenizer=tokenizer,
            mlm=False,
        )

        # Trainer
        trainer = Trainer(
            model=model,
            args=training_args,
            train_dataset=tokenized_dataset,
            data_collator=data_collator,
        )

        # Train
        print("Starting training...")
        trainer.train()

        # Save model
        trainer.save_model()
        print(f"Model saved to {OUTPUT_DIR}")

        # Log artifact
        mlflow.log_artifacts(str(OUTPUT_DIR))

if __name__ == "__main__":
    main()
```

Make executable: `chmod +x scripts/finetune-example.py`

---

## Verification Checklist

After completing all tasks, verify:

```bash
# Nix flake check
nix flake check --no-build

# Kustomize build (all overlays)
kustomize build manifests/kubernetes/overlays/development
kustomize build manifests/kubernetes/overlays/staging
kustomize build manifests/kubernetes/overlays/qa
kustomize build manifests/kubernetes/overlays/production

# Helm lint
helm lint charts/flexstack

# Shell scripts are executable
find scripts -name "*.sh" -type f -exec test -x {} \; -print

# Python scripts have valid syntax
python3 -m py_compile scripts/finetune-example.py
```

---

## Commit Strategy

Group commits logically:

1. **P0 Commit**: "feat: add K8s CLI tools to default devShell and create model infrastructure"
   - Tasks 1-5

2. **P1 Delivery Commit**: "feat: add Argo Rollouts manifests and install script"
   - Tasks 6-7

3. **P1 Identity Commit**: "feat: add Step-CA and Vaultwarden K8s manifests"
   - Tasks 8-9

4. **P1 Observability Commit**: "feat: add Netdata and Umami K8s manifests"
   - Tasks 10-12

5. **P1 Agent Commit**: "feat: add Node.js agent frameworks and wrapper scripts"
   - Tasks 13-15

6. **P1 Infrastructure Commit**: "feat: add Holochain DNA placeholder and mTLS docs"
   - Tasks 16-17

7. **P1 Environment Commit**: "feat: add QA/Staging overlays and LLMOps tooling"
   - Tasks 18-20

---

## Expected Outcome

After completion:
- Overall compliance: **95%+**
- All K8s CLI tools available in devShell
- Progressive delivery with Argo Rollouts
- Complete identity stack (Keycloak, Vault, Step-CA, Vaultwarden)
- Enhanced observability (Netdata, Umami)
- Node.js agent frameworks ready
- QA and Staging environments defined

---

*Generated from ARIA Audit Report v2.2.0*
