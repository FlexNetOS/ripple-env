# Modules Index

**Purpose:** Navigate to canonical documentation for each major system module.
**Layer:** All layers (L0-L5)
**Status:** Complete
**Last Updated:** 2026-01-14

---

## Quick Reference

| Module | Layer | Purpose | Status |
|--------|-------|---------|--------|
| [Bootstrap](bootstrap.md) | L3 | Golden paths from zero to full stack | verified |
| [Environments](environments.md) | L2 | Environment taxonomy and overlays | verified |
| [Toolchain](toolchain.md) | L1 | Package management and pinning | verified |
| [CI/CD](ci_cd.md) | L3 | Workflows, gates, and artifacts | verified |
| [Secrets](secrets.md) | L2 | Credential management and PKI | verified |
| [Providers](providers.md) | L2 | Service integrations and auth | verified |
| [Observability](observability.md) | L5 | Metrics, logs, and traces | verified |
| [Smoke Tests](smoke_tests.md) | L5 | Verification and definition of done | verified |

---

## Layered View

### L0 - Foundation
- Repository layout, naming conventions
- **See:** [ARCHITECTURE.md](../architecture/ARCHITECTURE.md)

### L1 - Toolchain
- [Toolchain Module](toolchain.md) - Nix, Pixi, npm, Cargo

### L2 - Environments
- [Environments Module](environments.md) - Dev/prod overlays
- [Secrets Module](secrets.md) - Credential management
- [Providers Module](providers.md) - Service integrations

### L3 - Orchestration
- [Bootstrap Module](bootstrap.md) - Setup flows
- [CI/CD Module](ci_cd.md) - Pipeline automation

### L4 - Deployment Targets
- **See:** [getting-started/deployment/](../getting-started/deployment/)
- Kubernetes, VMs, WSL2, Edge devices

### L5 - Operations
- [Observability Module](observability.md) - Monitoring stack
- [Smoke Tests Module](smoke_tests.md) - Verification procedures
- **See:** [cookbooks/](../cookbooks/INDEX.md) for operational recipes

---

## Module Metadata

All module docs include:
- **Layer:** L0-L5 classification
- **Criticality:** LOW/MEDIUM/HIGH
- **Surface:** internal/external
- **Runtime:** native/container/VM/WSL
- **Status:** draft/verified
- **Owner:** Responsible team

---

## Related Navigation

| Resource | Purpose |
|----------|---------|
| [docs/INDEX.md](../README.md) | Main documentation index |
| [docs/CATALOG.json](../CATALOG.json) | Machine-readable catalog |
| [docs/GLOSSARY.md](../GLOSSARY.md) | Terminology reference |
| [docs/graphs/](../graphs/) | Mermaid diagrams |
| [docs/scripts/INDEX.md](../scripts/INDEX.md) | Script contracts |
| [docs/cookbooks/INDEX.md](../cookbooks/INDEX.md) | Operational recipes |

---

**Next:** Start with [Bootstrap Module](bootstrap.md) for golden path documentation.
