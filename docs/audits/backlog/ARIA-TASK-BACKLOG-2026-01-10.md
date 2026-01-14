# ARIA Task Backlog

**Generated**: 2026-01-10
**Based on**: ARIA Audit v2.2.0 (Multi-Agent Parallel Execution)
**Repository**: FlexNetOS/ripple-env
**Branch**: claude/fetch-main-branch-OKeqW

---

## Execution Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 23 |
| Total Effort (Sequential) | 82 hours |
| Total Effort (Parallelized) | 31 hours |
| Efficiency Gain | 62% |
| Critical Path | 20 hours |
| Agents Deployed | 22 (Wave-based) |

---

## P0 — Immediate Action Required

### P0-001: Add HashiCorp Vault to Docker Compose

**Issue**: HashiCorp Vault is configured in flake.nix but MISSING from docker-compose.identity.yml. No secrets management capability.

**Location**: `docker/docker-compose.identity.yml`

**Impact**: Blocks 4 downstream tasks (P0-002, P1-003, P1-004, P2-005). Critical security gap - all services use plaintext credentials.

**Acceptance Criteria**:
- [ ] Vault service running on port 8200
- [ ] Dev mode enabled for initial deployment
- [ ] Health check endpoint responding
- [ ] Vault CLI can connect and authenticate
- [ ] Root token documented securely

**Files to Modify**:
- `docker/docker-compose.identity.yml`
- `config/vault/vault.hcl` (create)

**Implementation**:
```yaml
vault:
  image: hashicorp/vault:1.18
  container_name: vault
  ports:
    - "8200:8200"
  environment:
    VAULT_DEV_ROOT_TOKEN_ID: "dev-root-token"
    VAULT_DEV_LISTEN_ADDRESS: "0.0.0.0:8200"
  cap_add:
    - IPC_LOCK
  networks:
    - identity-network
  healthcheck:
    test: ["CMD", "vault", "status"]
    interval: 10s
    timeout: 5s
    retries: 3
```

**Complexity**: Small (1 hour)
**Dependencies**: None
**Priority Adjustment**: P0++ (IMMEDIATE) - Blocks 4 downstream tasks

---

### P0-002: Enable NATS Authentication

**Issue**: NATS server running without authentication. Unauthenticated message bus is a critical security vulnerability (SEC-005 violation).

**Location**: `docker/docker-compose.messaging.yml`, `config/nats/`

**Impact**: Blocks P0-003 (service mesh). Data leakage risk. No audit trail of NATS clients.

**Acceptance Criteria**:
- [ ] NATS auth enabled with password authentication
- [ ] Credentials stored in Vault (after P0-001)
- [ ] All existing clients updated with credentials
- [ ] Auth failure logged and monitored
- [ ] Verification: `nats account info --user admin` works

**Files to Modify**:
- `docker/docker-compose.messaging.yml`
- `config/nats/nats.conf` (create)

**Implementation**:
```hcl
# config/nats/nats.conf
authorization {
  users = [
    {user: "admin", password: "$NATS_ADMIN_PASS"}
    {user: "temporal", password: "$NATS_TEMPORAL_PASS"}
    {user: "n8n", password: "$NATS_N8N_PASS"}
  ]
}
```

**Complexity**: Small (2 hours)
**Dependencies**: P0-001 (Vault for credential storage)
**Priority Adjustment**: P0+ (IMMEDIATE) - Critical security gap

---

### P0-003: Configure Service Mesh Routing (NATS-Temporal-n8n)

**Issue**: Services are isolated on separate Docker networks. No inter-service event bus configured. NATS, Temporal, and n8n cannot communicate.

**Location**: `docker/docker-compose.messaging.yml`, `docker/docker-compose.temporal.yml`, `docker/docker-compose.automation.yml`

**Impact**: Event-driven architecture non-functional. Services cannot orchestrate workflows.

**Acceptance Criteria**:
- [ ] All messaging services on common `agentic-network`
- [ ] NATS subjects configured for service communication
- [ ] Temporal workers subscribed to NATS events
- [ ] n8n webhooks can trigger NATS messages
- [ ] End-to-end event flow tested

**Files to Modify**:
- `docker/docker-compose.messaging.yml`
- `docker/docker-compose.temporal.yml`
- `docker/docker-compose.automation.yml`
- `config/nats/nats.conf`

**Complexity**: Medium (4 hours)
**Dependencies**: P0-002 (NATS auth must be enabled)

---

## P1 — High Priority (This Week)

### P1-001: Create promptfooconfig.yaml

**Issue**: promptfoo installed via npx wrapper but no configuration file exists. LLM evaluation pipeline non-functional.

**Location**: `promptfooconfig.yaml` (root directory)

**Impact**: Cannot evaluate LLM responses for quality, safety, or accuracy. LLMOps domain at 75% ready.

**Acceptance Criteria**:
- [ ] promptfooconfig.yaml created with baseline test cases
- [ ] At least 3 eval metrics defined (accuracy, safety, relevance)
- [ ] Integration with LocalAI models configured
- [ ] Sample prompts with expected outputs
- [ ] `promptfoo eval` runs successfully

**Files to Modify**:
- `promptfooconfig.yaml` (create)
- `docs/llmops/evaluation-guide.md` (update)

**Complexity**: Small (2 hours)
**Dependencies**: P1-008 (LocalAI models must be downloaded first)

---

### P1-002: Add Agent Gateway to Docker Compose

**Issue**: ADR-002 defines AgentGateway architecture but no Docker service exists. Gateway referenced in edge stack but not deployed.

**Location**: `docker/docker-compose.edge.yml`

**Impact**: Cannot route MCP tool calls to agents. API gateway incomplete.

**Acceptance Criteria**:
- [ ] AgentGateway service in docker-compose.edge.yml
- [ ] Port 8090 (API) and 8092 (metrics) exposed
- [ ] Health check at /health endpoint
- [ ] Integration with Kong gateway
- [ ] MCP routing configuration loaded

**Files to Modify**:
- `docker/docker-compose.edge.yml`
- `config/agentgateway/config.yaml` (verify exists)

**Implementation**:
```yaml
agentgateway:
  image: agentgateway/agentgateway:latest
  container_name: agentgateway
  ports:
    - "8090:8090"
    - "8092:8092"
  volumes:
    - ./config/agentgateway:/etc/agentgateway:ro
  networks:
    - edge-network
  healthcheck:
    test: ["CMD", "curl", "-f", "http://localhost:8090/health"]
    interval: 30s
    timeout: 10s
    retries: 3
```

**Complexity**: Small (2 hours)
**Dependencies**: None

---

### P1-003: Configure Gateway Authentication (Kong + AgentGateway)

**Issue**: Both Kong and AgentGateway running without authentication. Unauthenticated APIs are attack vectors.

**Location**: `docker/docker-compose.edge.yml`, `config/kong/`

**Impact**: API access control missing. Cannot enforce authorization policies.

**Acceptance Criteria**:
- [ ] Kong OAuth2 plugin enabled
- [ ] Keycloak OIDC provider configured
- [ ] AgentGateway JWT validation enabled
- [ ] Rate limiting policies applied
- [ ] Authentication flow tested end-to-end

**Files to Modify**:
- `docker/docker-compose.edge.yml`
- `config/kong/kong.yml`
- `config/agentgateway/config.yaml`

**Complexity**: Medium (4 hours)
**Dependencies**: P1-002, P1-004, P1-005
**Priority Adjustment**: P1++ (HIGH) - Security critical

---

### P1-004: Set up Vault-Keycloak OIDC Integration

**Issue**: Vault and Keycloak configured separately with no OIDC integration. Cannot use Keycloak identities for Vault access.

**Location**: `config/vault/`, `config/keycloak/`

**Impact**: Duplicate identity management. Cannot enforce unified access policies.

**Acceptance Criteria**:
- [ ] Keycloak OIDC provider configured in Vault
- [ ] Vault policies mapped to Keycloak roles
- [ ] JWT auth method enabled in Vault
- [ ] Service accounts can authenticate via OIDC
- [ ] Integration tested with sample workflow

**Files to Modify**:
- `config/vault/auth-oidc.hcl` (create)
- `config/keycloak/vault-client.json` (create)

**Complexity**: Medium (4 hours)
**Dependencies**: P0-001 (Vault must be deployed)

---

### P1-005: Deploy OPA Server and Load Policies

**Issue**: OPA installed in flake.nix but server not deployed. Policies in `.claude/policies/` not loaded.

**Location**: `docker/docker-compose.identity.yml`, `config/opa/`

**Impact**: Authorization policies not enforced at runtime. Configuration enforcement only at lint time.

**Acceptance Criteria**:
- [ ] OPA server running on port 8181
- [ ] All policies from `config/opa/policies/` loaded
- [ ] Policy data updated from Vault (secrets)
- [ ] Health check and decision logging enabled
- [ ] `opa eval` returns expected decisions

**Files to Modify**:
- `docker/docker-compose.identity.yml`
- `config/opa/opa.yaml` (create)

**Complexity**: Small (3 hours)
**Dependencies**: None

---

### P1-006: Fix sandbox-runtime (Build from Source)

**Issue**: sandbox-runtime referenced for untrusted agent isolation but only exists as wrapper script. Actual binary not built.

**Location**: `flake.nix`, `scripts/`

**Impact**: Cannot isolate untrusted code execution. Agents run in unrestricted environment.

**Acceptance Criteria**:
- [ ] sandbox-runtime built from source or configured
- [ ] Integration with gVisor/Firecracker
- [ ] Resource limits configurable
- [ ] Sandbox profiles for different risk levels
- [ ] Test execution in sandbox verified

**Files to Modify**:
- `flake.nix`
- `scripts/sandbox-wrapper.sh`

**Complexity**: Small (2 hours)
**Dependencies**: None

---

### P1-007: Fix genai-toolbox Nix Hashes

**Issue**: genai-toolbox Nix package has incorrect or missing hashes. Build fails.

**Location**: `flake.nix`

**Impact**: Cannot use genai-toolbox for MCP tool execution. Tool layer incomplete.

**Acceptance Criteria**:
- [ ] genai-toolbox builds successfully
- [ ] Hash matches upstream release
- [ ] Binary available in devShell
- [ ] Basic functionality verified

**Files to Modify**:
- `flake.nix`

**Complexity**: Trivial (1 hour)
**Dependencies**: None

---

### P1-008: Download LocalAI Models

**Issue**: LocalAI configured but no models downloaded. Inference plane non-functional.

**Location**: `data/localai/models/`

**Impact**: Cannot run local inference. LLM evaluation pipeline blocked (P1-001).

**Acceptance Criteria**:
- [ ] At least 2 GGUF models downloaded
- [ ] Recommended: mistral-7b-instruct, llama2-7b-chat
- [ ] Models referenced in LocalAI config
- [ ] Inference endpoint returns responses
- [ ] `/v1/models` lists available models

**Files to Modify**:
- `data/localai/models/` (download)
- `config/localai/models.yaml` (update)

**Complexity**: Trivial (1 hour)
**Dependencies**: None

---

### P1-009: Standardize PostgreSQL to 17.2

**Issue**: PostgreSQL version inconsistency. Temporal uses 16-alpine while others use 17.2-alpine.

**Location**: `docker/docker-compose.temporal.yml`

**Impact**: Version drift creates compatibility risk. Complicates backup/restore.

**Acceptance Criteria**:
- [ ] All PostgreSQL services use 17.2-alpine
- [ ] Temporal compatibility verified
- [ ] Data migration documented (if needed)
- [ ] Backup/restore procedure tested

**Files to Modify**:
- `docker/docker-compose.temporal.yml`

**Complexity**: Trivial (1 hour)
**Dependencies**: None

---

### P1-010: Document Python Dual-Environment Design

**Issue**: Python 3.13 (Nix) and Python 3.11 (Pixi) coexist by design but not documented. Causes user confusion.

**Location**: `docs/PYTHON-ENVIRONMENTS.md` (create)

**Impact**: Onboarding friction. Users confused about which Python to use.

**Acceptance Criteria**:
- [ ] PYTHON-ENVIRONMENTS.md created
- [ ] Explains Nix 3.13 vs Pixi 3.11 rationale
- [ ] Documents which Python for which use case
- [ ] README updated with reference
- [ ] Troubleshooting section included

**Files to Modify**:
- `docs/PYTHON-ENVIRONMENTS.md` (create)
- `README.md` (update)

**Complexity**: Trivial (1 hour)
**Dependencies**: None

---

## P2 — Standard Priority (Next Sprint)

### P2-001: Integrate sandbox-runtime for Untrusted Agents

**Issue**: gVisor/Firecracker installed but not integrated for agent sandboxing. Untrusted code runs unrestricted.

**Location**: `scripts/`, `config/sandbox/`

**Impact**: Security risk from untrusted agent code. Cannot isolate agent execution.

**Acceptance Criteria**:
- [ ] Sandbox profiles for risk levels (low, medium, high)
- [ ] gVisor integration for container isolation
- [ ] Resource limits enforced (CPU, memory, network)
- [ ] AGiXT configured to use sandbox for untrusted agents
- [ ] Escape scenarios documented

**Files to Modify**:
- `scripts/sandbox-agent.sh` (create)
- `config/sandbox/profiles/` (create)
- `docker/docker-compose.agixt.yml` (update)

**Complexity**: Large (8 hours)
**Dependencies**: P1-006
**Risk Level**: MEDIUM - Security-critical integration

---

### P2-002: Configure NATS JetStream for Message Persistence

**Issue**: NATS configured for pub/sub but JetStream not enabled. Messages lost on restart.

**Location**: `docker/docker-compose.messaging.yml`, `config/nats/`

**Impact**: No message durability. Cannot replay events for debugging.

**Acceptance Criteria**:
- [ ] JetStream enabled in NATS config
- [ ] Streams created for key subjects
- [ ] Retention policies configured
- [ ] Persistence storage mounted
- [ ] Stream metrics exposed to Prometheus

**Files to Modify**:
- `docker/docker-compose.messaging.yml`
- `config/nats/jetstream.conf` (create)

**Complexity**: Medium (4 hours)
**Dependencies**: P0-002, P0-003

---

### P2-003: Implement Holochain Zomes (WASM)

**Issue**: Holochain installed and conductor configured, but DNA zomes not implemented. Only scaffolded templates exist.

**Location**: `manifests/holochain/dnas/`

**Impact**: Distributed coordination layer non-functional. Cannot use Holochain for consensus.

**Acceptance Criteria**:
- [ ] 5 DNA zomes implemented (identity, permissions, logging, consensus, storage)
- [ ] Entry types defined with validation
- [ ] CRUD functions implemented
- [ ] WASM compilation successful
- [ ] Conductor loads and runs DNAs
- [ ] Integration tests pass

**Files to Modify**:
- `manifests/holochain/dnas/*/zomes/` (implement)
- `manifests/holochain/conductor-config.yaml` (update)

**Phases**:
1. Define entry types (2-3h)
2. CRUD functions (3-4h)
3. WASM compilation (2h)
4. Conductor integration (2-3h)
5. Service bridge (3-4h)

**Complexity**: Large (16 hours)
**Dependencies**: None
**Priority Adjustment**: P1+ - On critical path (20h)
**Risk Level**: HIGH - Largest task, Rust/WASM expertise required

---

### P2-004: Install Node.js Agent Frameworks

**Issue**: Node.js agent frameworks (claude-flow, agentic-flow) referenced but not installed.

**Location**: `package.json` (create), `flake.nix`

**Impact**: Cannot use Node.js-based agent orchestration patterns.

**Acceptance Criteria**:
- [ ] claude-flow installed and configured
- [ ] agentic-flow installed and configured
- [ ] Integration with existing AGiXT agents
- [ ] Sample workflows documented

**Files to Modify**:
- `package.json` (create or update)
- `flake.nix`

**Complexity**: Small (2 hours)
**Dependencies**: None

---

### P2-005: Enable mTLS on Services

**Issue**: Services communicate over unencrypted channels. No mutual TLS between components.

**Location**: `docker/docker-compose.*.yml`, `config/step-ca/`

**Impact**: Service-to-service traffic unencrypted. Cannot verify service identity.

**Acceptance Criteria**:
- [ ] step-ca issuing service certificates
- [ ] All Docker services configured for mTLS
- [ ] Certificate rotation automated
- [ ] Non-mTLS connections rejected
- [ ] Certificate monitoring in Grafana

**Files to Modify**:
- `config/step-ca/ca.json`
- `docker/docker-compose.*.yml` (all stacks)
- `scripts/rotate-certs.sh` (create)

**Complexity**: Medium (4 hours)
**Dependencies**: P0-001, P1-004

---

### P2-006: Create Grafana Dashboards for ROS2

**Issue**: Prometheus scraping ROS2 metrics but no Grafana dashboards provisioned.

**Location**: `manifests/observability/grafana/dashboards/`

**Impact**: Cannot visualize ROS2 system health. Debugging blind spots.

**Acceptance Criteria**:
- [ ] ROS2 node status dashboard
- [ ] Topic throughput dashboard
- [ ] Service latency dashboard
- [ ] Resource utilization (CPU, memory per node)
- [ ] Alert rules for critical thresholds

**Files to Modify**:
- `manifests/observability/grafana/dashboards/ros2-*.json` (create)
- `manifests/observability/grafana/provisioning/dashboards.yaml`

**Complexity**: Medium (4 hours)
**Dependencies**: None

---

## P3 — Backlog

### P3-001: Add vCache Semantic Prompt Caching

**Issue**: vCache semantic caching referenced in spec but not integrated. Prompt caching relies on exact matching only.

**Location**: `pixi.toml`, `config/vcache/`

**Impact**: Redundant LLM calls for semantically similar prompts. Increased cost.

**Acceptance Criteria**:
- [ ] vCache integrated with LocalAI
- [ ] Semantic similarity threshold configured
- [ ] Cache hit rate metrics exposed
- [ ] Cache invalidation policies defined

**Files to Modify**:
- `pixi.toml`
- `config/vcache/config.yaml` (create)
- `docker/docker-compose.inference.yml`

**Complexity**: Medium (4 hours)
**Dependencies**: None

---

### P3-002: Configure Bytebase for DB CI/CD

**Issue**: Bytebase referenced for database CI/CD but not configured.

**Location**: `docker/docker-compose.data.yml`

**Impact**: Database schema changes not version controlled. No migration pipeline.

**Acceptance Criteria**:
- [ ] Bytebase deployed and configured
- [ ] PostgreSQL databases registered
- [ ] Migration workflows defined
- [ ] CI integration for schema changes

**Files to Modify**:
- `docker/docker-compose.data.yml`
- `config/bytebase/` (create)

**Complexity**: Medium (4 hours)
**Dependencies**: None

---

### P3-003: Deploy Holochain Bootstrap Servers

**Issue**: Using public Holochain bootstrap servers. Cannot operate independently.

**Location**: `manifests/holochain/`, `docker/`

**Impact**: Dependency on external infrastructure. Network discovery fails if public servers down.

**Acceptance Criteria**:
- [ ] Bootstrap server deployed
- [ ] Conductor configured to use local bootstrap
- [ ] Peer discovery verified
- [ ] Failover to public servers documented

**Files to Modify**:
- `docker/docker-compose.holochain.yml` (create)
- `manifests/holochain/conductor-config.yaml`

**Complexity**: Medium (4 hours)
**Dependencies**: P2-003 (Holochain zomes must be implemented)

---

### P3-004: Add Distributed Tracing (Jaeger/Tempo)

**Issue**: OpenTelemetry collector configured but no trace backend. Cannot visualize distributed traces.

**Location**: `docker/docker-compose.observability.yml`

**Impact**: Cannot debug cross-service request flows. Latency source identification difficult.

**Acceptance Criteria**:
- [ ] Jaeger or Tempo deployed
- [ ] OTLP traces flowing to backend
- [ ] Service dependency graph visible
- [ ] Trace sampling configured
- [ ] Grafana datasource configured

**Files to Modify**:
- `docker/docker-compose.observability.yml`
- `config/otel/otel-collector.yaml`

**Complexity**: Medium (4 hours)
**Dependencies**: None

---

## Summary

| Priority | Count | Effort (Sequential) | Effort (Parallel) | Notes |
|----------|-------|---------------------|-------------------|-------|
| P0 | 3 | 7 hours | 7 hours | Sequential chain required |
| P1 | 10 | 21 hours | 8 hours | Wave 1+2 parallelization |
| P2 | 6 | 38 hours | 20 hours | P2-003 bottleneck (16h) |
| P3 | 4 | 16 hours | 4 hours | Full parallelization |
| **Total** | **23** | **82 hours** | **31 hours** | **62% efficiency gain** |

---

## Implementation Waves

### Wave 1: Foundation (Day 1) — 3 hours with 4 agents
- P0-001: Add Vault ⭐ CRITICAL
- P1-002: AgentGateway
- P1-005: OPA Server
- P1-006: Sandbox-runtime
- P1-007: genai-toolbox hashes
- P1-008: LocalAI models
- P1-009: PostgreSQL standardization
- P1-010: Python documentation

### Wave 2: Identity & Models (Day 1-2) — 4 hours with 3 agents
- P0-002: NATS auth (needs Vault)
- P1-004: Vault-Keycloak OIDC
- P1-001: promptfoo config (needs models)

### Wave 3: Service Mesh (Day 2-3) — 4 hours with 5 agents
- P0-003: Service mesh routing
- P1-003: Gateway auth
- P2-002: JetStream
- P2-004: Node.js agents
- P2-006: Grafana dashboards

### Wave 4: Security & Complex (Week 1-2) — 16 hours with 3 agents
- P2-001: Sandbox integration
- P2-005: mTLS
- P2-003: Holochain zomes ⭐ CRITICAL PATH

### Wave 5: Advanced (Week 2-3) — 4 hours with 4 agents
- P3-001: vCache
- P3-002: Bytebase
- P3-003: Holochain bootstrap (needs P2-003)
- P3-004: Distributed tracing

---

## Critical Path

```
P2-003 (Holochain Zomes, 16h) → P3-003 (Bootstrap, 4h)
Total: 20 hours (24% of total effort)
```

**Mitigation**: Allocate 2 Rust developers to P2-003 → reduce to 9-10h

---

## Risk Adjustments

| Task | Original | Adjusted | Reason |
|------|----------|----------|--------|
| P0-001 | P0 | P0++ | Blocks 4 tasks, critical security |
| P0-002 | P0 | P0+ | Security gap, blocks mesh |
| P2-003 | P2 | P1+ | On critical path (16h), complex |
| P1-003 | P1 | P1++ | Unauthenticated APIs |

---

## Execution Timeline

| Phase | Duration | Outcome |
|-------|----------|---------|
| P0 Resolution | 1 day | Production Viable (85%) |
| P1 Completion | +1 week | Production Ready (95%) |
| P2 Hardening | +2-3 weeks | Production Optimized (98%) |
| P3 Enhancements | +1 week | Production Complete (100%) |

---

*Report generated by ARIA Orchestrator v2.2.0*
*Multi-agent parallel execution with Kimi K2 cross-analysis*
*2026-01-10*
