# ARIA Task Reference - Quick Lookup Guide

**Document**: Task ID to Details Mapping
**Generated**: 2026-01-10
**Source**: ARIA v2.2.0 Audit Findings

---

## P2 Priority Tasks (Medium) - 30 Hours Total

### P2-001: Sandbox Runtime for Untrusted Agents
- **Domain**: Domain 2 - Agent Sandboxing & Isolation
- **Effort**: 8 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: gVisor OCI runtime
- **Main Files**:
  - `/home/user/ripple-env/flake.nix`
  - `/home/user/ripple-env/modules/linux/docker.nix`
  - `/home/user/ripple-env/manifests/agent-isolation/` (create)
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~30

---

### P2-002: Configure NATS JetStream
- **Domain**: Domain 6 - Distributed Messaging & Event Bus
- **Effort**: 4 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: NATS JetStream, persistent message queue
- **Main Files**:
  - `/home/user/ripple-env/manifests/distributed/nats/` (create)
  - `/home/user/ripple-env/config/nats/jetstream-config.yaml` (create)
  - `/home/user/ripple-env/pixi.toml`
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~110

---

### P2-003: Implement Holochain Zomes (WASM)
- **Domain**: Domain 11 - Coordination (Holochain)
- **Effort**: 16 hours
- **Complexity**: Large
- **Status**: Pending (scaffolding complete)
- **Blocker**: Blocks P3-003
- **Key Technology**: Holochain, Rust, WASM, HDK/HDI
- **Main Files**:
  - `/home/user/ripple-env/manifests/holochain/dnas/*/zomes/*/src/lib.rs` (create/implement)
  - `/home/user/ripple-env/rust/Cargo.toml`
  - `/home/user/ripple-env/manifests/holochain/conductor.yaml`
- **Dependencies**: None (but 5 sub-phases)
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~190
- **Related**:
  - `/home/user/ripple-env/manifests/holochain/IMPLEMENTATION_SUMMARY.md`
  - `/home/user/ripple-env/manifests/holochain/HOLOCHAIN.md`

---

### P2-004: Install Node.js Agent Frameworks
- **Domain**: Domain 7 - Agent Frameworks & Runtimes
- **Effort**: 2 hours
- **Complexity**: Small
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: Node.js 22, pnpm, agentic-flow, claude-flow
- **Main Files**:
  - `/home/user/ripple-env/package.json` (create)
  - `/home/user/ripple-env/pnpm-lock.yaml` (create)
  - `/home/user/ripple-env/flake.nix`
  - `/home/user/ripple-env/modules/common/packages.nix`
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~290

---

### P2-005: Enable mTLS on Services
- **Domain**: Domain 5 - Service Security & mTLS
- **Effort**: 4 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: Step CA, mutual TLS, certificates
- **Main Files**:
  - `/home/user/ripple-env/config/step-ca/` (modify)
  - `/home/user/ripple-env/manifests/security/` (create)
  - Service config files (Temporal, Holochain, NATS, etc.)
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~360

---

### P2-006: Create Grafana Dashboards for ROS2
- **Domain**: Domain 14 - Observability & Monitoring
- **Effort**: 4 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: Grafana, Prometheus, ROS2 metrics
- **Main Files**:
  - `/home/user/ripple-env/config/grafana/provisioning/dashboards/ros2-*.json` (create)
  - `/home/user/ripple-env/manifests/observability/prometheus.yml`
  - `/home/user/ripple-env/config/grafana/provisioning/dashboards/default.yml`
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~440

---

## P3 Priority Tasks (Low) - 12 Hours Total

### P3-001: Add vCache Semantic Prompt Caching
- **Domain**: Domain 10 - Caching & State
- **Effort**: 4 hours
- **Complexity**: Small
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: vCache, semantic caching, Redis
- **Main Files**:
  - `/home/user/ripple-env/pixi.toml` (caching feature)
  - `/home/user/ripple-env/docs/VCACHE_SETUP.md` (create)
  - `/home/user/ripple-env/examples/llm-caching-example.py` (create)
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~495
- **Note**: Requires building from source; not on PyPI

---

### P3-002: Configure Bytebase for Database CI/CD
- **Domain**: Domain 9 - Data Management & Migration
- **Effort**: 4 hours
- **Complexity**: Small
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: Bytebase, database schema CI/CD, GitOps
- **Main Files**:
  - `/home/user/ripple-env/manifests/database/bytebase-deployment.yaml` (create)
  - `/home/user/ripple-env/docker/docker-compose.yml`
  - `/home/user/ripple-env/docs/BYTEBASE_WORKFLOW.md` (create)
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~560

---

### P3-003: Deploy Holochain Bootstrap Servers
- **Domain**: Domain 11 - Coordination (Holochain)
- **Effort**: 4 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None (but depends on P2-003)
- **Key Technology**: Holochain bootstrap, signal servers, networking
- **Main Files**:
  - `/home/user/ripple-env/manifests/holochain/bootstrap/` (create)
  - `/home/user/ripple-env/manifests/holochain/networks.json`
  - `/home/user/ripple-env/infrastructure/holochain/terraform/` (create)
- **Dependencies**: P2-003 (Holochain zomes)
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~620

---

### P3-004: Add Distributed Tracing (Jaeger/Tempo)
- **Domain**: Domain 14 - Observability & Monitoring
- **Effort**: 4 hours
- **Complexity**: Medium
- **Status**: Pending
- **Blocker**: None
- **Key Technology**: Jaeger, Tempo, OpenTelemetry, OTEL
- **Main Files**:
  - `/home/user/ripple-env/manifests/observability/jaeger-deployment.yaml` (create)
  - `/home/user/ripple-env/config/otel-collector/jaeger-config.yaml` (create)
  - `/home/user/ripple-env/pixi.toml` (add OTEL packages)
- **Dependencies**: None
- **Specification**: See `ARIA_TASK_SPECS.yaml` line ~680

---

## Quick Statistics

| Metric | Value |
|--------|-------|
| Total Tasks | 10 |
| P2 Tasks | 6 |
| P3 Tasks | 4 |
| Total Effort | 42 hours |
| P2 Effort | 30 hours |
| P3 Effort | 12 hours |
| Tasks with Blockers | 0 |
| Tasks with Dependencies | 1 (P3-003) |

---

## Implementation Recommendation Order

### Quick Wins (Phase 1) - 10 hours
1. **P2-002** - NATS JetStream (4h)
2. **P2-004** - Node.js Agents (2h)
3. **P2-006** - Grafana Dashboards (4h)

### Security (Phase 2) - 12 hours
1. **P2-001** - Sandbox Runtime (8h)
2. **P2-005** - mTLS (4h)

### Complex (Phase 3) - 16 hours
1. **P2-003** - Holochain Zomes (16h)

### Advanced (Phase 4) - 12 hours
1. **P3-001** - vCache Caching (4h)
2. **P3-002** - Bytebase Database (4h)
3. **P3-004** - Distributed Tracing (4h)

### Deployment (Phase 5) - 4 hours
1. **P3-003** - Holochain Bootstrap (4h) - after P2-003

---

## Task File Locations

### Main Specification Files (in repository root)
```
/home/user/ripple-env/
├── ARIA_TASK_SPECS.yaml              # Complete YAML specifications (547 lines)
├── ARIA_TASK_EXTRACTION_SUMMARY.md   # Implementation guide (367 lines)
└── ARIA_TASK_REFERENCE.md            # This file
```

### Current Implementation Status
- **P2-003**: 30% complete (scaffolding done, zomes pending)
- **All Others**: 0% complete (ready for implementation)

---

## Files to Review Before Starting

### For Understanding Architecture
- `/home/user/ripple-env/BUILDKIT_STARTER_SPEC.md` - Overall architecture
- `/home/user/ripple-env/.claude/CLAUDE.md` - Project guidelines

### For Specific Domains
- **Holochain**: `/home/user/ripple-env/manifests/holochain/HOLOCHAIN.md`
- **Observability**: `/home/user/ripple-env/manifests/observability/`
- **Security**: `/home/user/ripple-env/config/step-ca/README.md`
- **Agent Gateway**: `/home/user/ripple-env/config/agentgateway/config.yaml`

---

## Commands to Run Tasks

### For P2-002 (NATS JetStream)
```bash
cd /home/user/ripple-env
git checkout -b feat/p2-002-nats-jetstream
# See ARIA_TASK_SPECS.yaml for detailed steps
```

### For P2-003 (Holochain Zomes)
```bash
cd /home/user/ripple-env
git checkout -b feat/p2-003-holochain-zomes
# Refer to manifests/holochain/IMPLEMENTATION_SUMMARY.md for phases
```

### For Any Task
```bash
# 1. Review specification
grep -A 50 'id: "PXXX-YYY"' ARIA_TASK_SPECS.yaml

# 2. Create branch
git checkout -b feat/pxxx-yyy-short-name

# 3. Follow files_to_modify list in YAML
# 4. Run acceptance_tests from YAML
# 5. Verify with quality_checklist from YAML
```

---

## Contact & Escalation

- **Questions on Task Specs**: Review `ARIA_TASK_SPECS.yaml` (detailed YAML)
- **Questions on Process**: Review `ARIA_TASK_EXTRACTION_SUMMARY.md` (implementation guide)
- **Questions on Priority**: Check `implementation_order_recommendation` section
- **Domain Experts**: See related_issues in each task specification

---

## Document Version

- **Created**: 2026-01-10
- **Format**: Markdown (this file) + YAML (ARIA_TASK_SPECS.yaml)
- **Source**: ARIA v2.2.0 Audit Findings
- **Status**: Ready for Implementation
- **Next Review**: After completing Phase 1

---

**TIP**: For detailed implementation steps, open `/home/user/ripple-env/ARIA_TASK_SPECS.yaml` and search for your task ID (e.g., "P2-001").
