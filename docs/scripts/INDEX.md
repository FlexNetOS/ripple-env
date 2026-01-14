# Script Index

**Status:** Phase 1 Complete (Inventory) | Phase 4 In Progress (Contract Docs)
**Last Updated:** 2026-01-14

This index catalogs all 60 script entrypoints discovered in Phase 1. Each script will receive a detailed contract document in Phase 4.

---

## Critical Bootstrap Scripts

| Script | Platform | Purpose | Contract Doc | Evidence |
|--------|----------|---------|--------------|----------|
| `bootstrap.sh` | Linux/macOS | Complete environment setup (Nix, direnv, pixi, shells) | [bootstrap.sh.md](./bootstrap.sh.md) ✅ | 1160 lines, 13 stages, retry logic, state persistence |
| `bootstrap.ps1` | Windows (Admin) | WSL2 + NixOS + full stack setup | [bootstrap.ps1.md](./bootstrap.ps1.md) ✅ | 806 lines, 8 stages, retry logic, state persistence |

---

## Deployment Scripts (5)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `deploy.sh` | Main deployment orchestrator | [deploy.sh.md](./deploy.sh.md) ⏳ | deployment |
| `deploy-edge.sh` | Deploy edge services | [deploy-edge.sh.md](./deploy-edge.sh.md) ⏳ | deployment |
| `deploy-observability.sh` | Deploy observability stack | [deploy-observability.sh.md](./deploy-observability.sh.md) ⏳ | deployment |
| `deploy-airgapped.sh` | Airgapped deployment preparation | [deploy-airgapped.sh.md](./deploy-airgapped.sh.md) ⏳ | deployment |
| `install-all.sh` | Install all services | [install-all.sh.md](./install-all.sh.md) ⏳ | deployment |

---

## Verification Scripts (5)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `validate-e2e.sh` | End-to-end validation | [validate-e2e.sh.md](./validate-e2e.sh.md) ⏳ | verification |
| `validate-configs.sh` | Configuration validation | [validate-configs.sh.md](./validate-configs.sh.md) ⏳ | verification |
| `validate-manifest.py` | ARIA manifest validation | [validate-manifest.py.md](./validate-manifest.py.md) ⏳ | verification |
| `validate-resources.sh` | Resource validation | [validate-resources.sh.md](./validate-resources.sh.md) ⏳ | verification |
| `validate-channels.py` | Pixi channel validation | [validate-channels.py.md](./validate-channels.py.md) ⏳ | verification |

---

## Component Verification Scripts (10)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `verify-argo-workflows.sh` | Verify Argo Workflows | [verify-argo-workflows.sh.md](./verify-argo-workflows.sh.md) ⏳ | component_verification |
| `verify-edge.sh` | Verify edge services | [verify-edge.sh.md](./verify-edge.sh.md) ⏳ | component_verification |
| `verify-jetstream.sh` | Verify NATS JetStream | [verify-jetstream.sh.md](./verify-jetstream.sh.md) ⏳ | component_verification |
| `verify-mindsdb.sh` | Verify MindsDB | [verify-mindsdb.sh.md](./verify-mindsdb.sh.md) ⏳ | component_verification |
| `verify-mtls-setup.sh` | Verify mTLS setup | [verify-mtls-setup.sh.md](./verify-mtls-setup.sh.md) ⏳ | component_verification |
| `verify-observability.sh` | Verify observability stack | [verify-observability.sh.md](./verify-observability.sh.md) ⏳ | component_verification |
| `verify-open-lovable.sh` | Verify Open Lovable | [verify-open-lovable.sh.md](./verify-open-lovable.sh.md) ⏳ | component_verification |
| `verify-qudag.sh` | Verify QuDAG | [verify-qudag.sh.md](./verify-qudag.sh.md) ⏳ | component_verification |
| `verify-ruvector.sh` | Verify RuVector | [verify-ruvector.sh.md](./verify-ruvector.sh.md) ⏳ | component_verification |
| `verify-state-storage.sh` | Verify state storage | [verify-state-storage.sh.md](./verify-state-storage.sh.md) ⏳ | component_verification |

---

## Initialization Scripts (4)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `init-docker-networks.sh` | Initialize Docker networks | [init-docker-networks.sh.md](./init-docker-networks.sh.md) ⏳ | initialization |
| `init-jetstream.sh` | Initialize NATS JetStream | [init-jetstream.sh.md](./init-jetstream.sh.md) ⏳ | initialization |
| `init-step-ca.sh` | Initialize Step CA | [init-step-ca.sh.md](./init-step-ca.sh.md) ⏳ | initialization |
| `init-multi-db.sh` | Initialize multi-database | [init-multi-db.sh.md](./init-multi-db.sh.md) ⏳ | initialization |

---

## Installation Scripts (2)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `install-argocd.sh` | Install ArgoCD | [install-argocd.sh.md](./install-argocd.sh.md) ⏳ | installation |
| `install-argo-rollouts.sh` | Install Argo Rollouts | [install-argo-rollouts.sh.md](./install-argo-rollouts.sh.md) ⏳ | installation |

---

## Security Scripts (5)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `security-audit.sh` | Run security audits | [security-audit.sh.md](./security-audit.sh.md) ⏳ | security |
| `scan-containers.sh` | Scan containers with Trivy | [scan-containers.sh.md](./scan-containers.sh.md) ⏳ | security |
| `generate-service-certs.sh` | Generate service certificates | [generate-service-certs.sh.md](./generate-service-certs.sh.md) ⏳ | security |
| `rotate-certs.sh` | Rotate certificates | [rotate-certs.sh.md](./rotate-certs.sh.md) ⏳ | security |
| `setup-cert-rotation-cron.sh` | Setup certificate rotation cron | [setup-cert-rotation-cron.sh.md](./setup-cert-rotation-cron.sh.md) ⏳ | security |

---

## Database Scripts (4)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `populate-config-db.sh` | Populate configuration database | [populate-config-db.sh.md](./populate-config-db.sh.md) ⏳ | database |
| `populate-config-db.py` | Populate configuration database (Python) | [populate-config-db.py.md](./populate-config-db.py.md) ⏳ | database |
| `query-config-db.sh` | Query configuration database | [query-config-db.sh.md](./query-config-db.sh.md) ⏳ | database |
| `query-config-db.py` | Query configuration database (Python) | [query-config-db.py.md](./query-config-db.py.md) ⏳ | database |

---

## Setup Scripts (3)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `setup-argo-workflows.sh` | Setup Argo Workflows | [setup-argo-workflows.sh.md](./setup-argo-workflows.sh.md) ⏳ | setup |
| `setup-home-manager.sh` | Setup Home Manager | [setup-home-manager.sh.md](./setup-home-manager.sh.md) ⏳ | setup |
| `setup-cert-rotation-cron.sh` | Setup certificate rotation | [setup-cert-rotation-cron.sh.md](./setup-cert-rotation-cron.sh.md) ⏳ | setup |

---

## Build Scripts (2)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `build-frontend.sh` | Build frontend assets | [build-frontend.sh.md](./build-frontend.sh.md) ⏳ | build |
| `download-models.sh` | Download AI models | [download-models.sh.md](./download-models.sh.md) ⏳ | build |

### Model Fetch/Cache (LocalAI)

| Script | Platform | Purpose | Contract Doc | Category |
|--------|----------|---------|--------------|----------|
| `fetch-localai-models.sh` | Linux/WSL | Fetch LocalAI models via Git LFS; cache/restore models | ⏳ | build |
| `fetch-localai-models.ps1` | Windows | Fetch LocalAI models via Git LFS; cache/restore models | ⏳ | build |

---

## Maintenance Scripts (5)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `upgrade-python-deps.sh` | Upgrade Python dependencies | [upgrade-python-deps.sh.md](./upgrade-python-deps.sh.md) ⏳ | maintenance |
| `check-python-deps.sh` | Check Python dependency versions | [check-python-deps.sh.md](./check-python-deps.sh.md) ⏳ | maintenance |
| `Cleanup-WSL.ps1` | Clean up WSL artifacts | [Cleanup-WSL.ps1.md](./Cleanup-WSL.ps1.md) ⏳ | maintenance |
| `wsl-cleanup.sh` | Clean up WSL (bash) | [wsl-cleanup.sh.md](./wsl-cleanup.sh.md) ⏳ | maintenance |
| `fix-wsl-stability.sh` | Fix WSL stability issues | [fix-wsl-stability.sh.md](./fix-wsl-stability.sh.md) ⏳ | maintenance |

---

## Performance Scripts (2)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `isolate-cpu.sh` | Isolate CPU for performance | [isolate-cpu.sh.md](./isolate-cpu.sh.md) ⏳ | performance |
| `benchmark-eval.sh` | Run benchmarks | [benchmark-eval.sh.md](./benchmark-eval.sh.md) ⏳ | performance |

---

## Utility Scripts (8)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `health-check.sh` | Health check services | [health-check.sh.md](./health-check.sh.md) ⏳ | utilities |
| `env-vars.sh` | Environment variable helpers | [env-vars.sh.md](./env-vars.sh.md) ⏳ | utilities |
| `stable-env.sh` | Stable environment configuration | [stable-env.sh.md](./stable-env.sh.md) ⏳ | utilities |
| `ruvector.sh` | RuVector CLI wrapper | [ruvector.sh.md](./ruvector.sh.md) ⏳ | utilities |
| `ruvector.ps1` | RuVector CLI wrapper (PowerShell) | [ruvector.ps1.md](./ruvector.ps1.md) ⏳ | utilities |
| `cleanup-ruvector.ps1` | Cleanup RuVector artifacts | [cleanup-ruvector.ps1.md](./cleanup-ruvector.ps1.md) ⏳ | utilities |
| `flexstack.sh` | FlexStack orchestrator | [flexstack.sh.md](./flexstack.sh.md) ⏳ | utilities |
| `ros2-version` | ROS2 version info | [ros2-version.md](./ros2-version.md) ⏳ | utilities |

---

## Sandboxing Scripts (2)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `sandbox-agent.sh` | Sandbox agent execution | [sandbox-agent.sh.md](./sandbox-agent.sh.md) ⏳ | sandboxing |
| `sandbox-wrapper.sh` | Sandbox wrapper | [sandbox-wrapper.sh.md](./sandbox-wrapper.sh.md) ⏳ | sandboxing |

---

## Analysis Scripts (3)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `analyze-dependencies.py` | Analyze dependencies | [analyze-dependencies.py.md](./analyze-dependencies.py.md) ⏳ | analysis |
| `analyze-workflows.py` | Analyze workflows | [analyze-workflows.py.md](./analyze-workflows.py.md) ⏳ | analysis |
| `agent-config.py` | Agent configuration management | [agent-config.py.md](./agent-config.py.md) ⏳ | analysis |

---

## Generation Scripts (1)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `generate-verification.py` | Generate verification scripts from ARIA manifest | [generate-verification.py.md](./generate-verification.py.md) ⏳ | generation |

---

## ML Ops Scripts (1)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `finetune-example.py` | LLM finetuning example | [finetune-example.py.md](./finetune-example.py.md) ⏳ | ml_ops |

---

## Offline Deployment Scripts (1)

| Script | Purpose | Contract Doc | Category |
|--------|---------|--------------|----------|
| `prepare-offline.sh` | Prepare for offline/airgapped deployment | [prepare-offline.sh.md](./prepare-offline.sh.md) ⏳ | offline |

---

## Script Categories Summary

| Category | Count | Status |
|----------|-------|--------|
| Bootstrap | 2 | Phase 1 ✅ |
| Deployment | 5 | Phase 1 ✅ |
| Verification | 5 | Phase 1 ✅ |
| Component Verification | 10 | Phase 1 ✅ |
| Initialization | 4 | Phase 1 ✅ |
| Installation | 2 | Phase 1 ✅ |
| Security | 5 | Phase 1 ✅ |
| Database | 4 | Phase 1 ✅ |
| Setup | 3 | Phase 1 ✅ |
| Build | 2 | Phase 1 ✅ |
| Maintenance | 5 | Phase 1 ✅ |
| Performance | 2 | Phase 1 ✅ |
| Utilities | 8 | Phase 1 ✅ |
| Sandboxing | 2 | Phase 1 ✅ |
| Analysis | 3 | Phase 1 ✅ |
| Generation | 1 | Phase 1 ✅ |
| ML Ops | 1 | Phase 1 ✅ |
| Offline | 1 | Phase 1 ✅ |
| **Total** | **60** | **Inventory Complete** |

---

## Next Steps

### Phase 4: Script Contract Generation

Each script will receive a contract document with the following sections:

1. **Purpose**
2. **Invocation**
3. **Inputs** (args, env vars, config files)
4. **Outputs** (files/artifacts, exit codes)
5. **Side Effects**
6. **Safety Classification** (safe/caution/destructive + justification)
7. **Idempotency**
8. **Dependencies** (tools, services, credentials)
9. **Failure Modes** (known errors)
10. **Examples** (only if demonstrably supported)
11. **References** (evidence file paths)

---

**Last Updated:** 2026-01-13 (Phase 1)
**Next Update:** Phase 4 (Script Contract Generation)
