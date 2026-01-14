# Script Contract: verify-qudag.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-qudag.sh`

---

## Purpose

Verify QuDAG integration (P3-011, Layer 11 P2P Coordination). Checks Rust dependencies, Docker Compose configuration, environment variables, Pixi feature flags, MCP server configuration, network setup, documentation, and optional service status.

---

## Invocation

```bash
chmod +x scripts/verify-qudag.sh
./scripts/verify-qudag.sh
```

**Environment Variables:**
- `QUDAG_REQUIRE_RUST=1` - Enforce Rust crate checks (default: 0, warnings only)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Passed (0 failures) |
| `1` | Failed (1+ failures) |

### Counters
- Passed (green)
- Failed (red)
- Warnings (yellow)

---

## Side Effects

**Minimal:** Read-only verification. Creates no persistent state.

**Service Checks (optional):** If Docker and QuDAG are running, tests health and MCP endpoints (lines 437-466).

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly without side effects.

---

## Verification Stages (8)

### Stage 1: Rust Dependencies (lines 90-152)
Checks `rust/Cargo.toml` for QuDAG crates:
- **qudag-core** - Core DAG functionality
- **qudag-crypto** - Cryptographic primitives
- **qudag-network** - P2P networking
- **qudag-mcp** - MCP protocol support
- **blake3** - Hash library (1.5)
- **chacha20poly1305** - Cipher (0.10)

**Evidence:** Lines 98-149 for crate checks.

**Enforcement:** Warnings unless `QUDAG_REQUIRE_RUST=1` (lines 95, 101-105).

### Stage 2: Docker Compose Configuration (lines 160-190)
- **docker-compose.qudag.yml** exists
- Syntax validation via `docker compose config`
- **qudag-node** service defined (required)
- **qudag-mcp** service defined (optional, MCP profile)

**Evidence:** Lines 163-189.

### Stage 3: Environment Configuration (lines 198-229)
Checks `.env.qudag.example` for:
- `QUDAG_CRYPTO_SUITE` - Cryptographic suite selection
- `QUDAG_CONSENSUS_ALGORITHM` - Consensus mechanism
- `QUDAG_MCP_ENABLED` - MCP protocol toggle
- `QUDAG_ONION_ENABLED` - Onion routing toggle

**Evidence:** Lines 201-226.

### Stage 4: Pixi Feature Flags (lines 237-266)
Checks `pixi.toml` for:
- `[feature.qudag]` - QuDAG feature definition
- `websockets >= 11.0` - WebSocket support for MCP
- `mcp >= 1.0` - MCP SDK
- `qudag` environment with features

**Evidence:** Lines 241-262.

### Stage 5: MCP Server Configuration (lines 274-335)
Validates `manifests/mcp/qudag-server.json`:
- JSON syntax validation (Python or jq)
- Tools count (expects >= 10 tools)
- Resources count (expects >= 5 resources)

**Evidence:** Lines 277-332.

**JSON Validation:** Uses Python fallback if jq unavailable (lines 75-84).

### Stage 6: Network Configuration (lines 343-405)
Validates `manifests/qudag/networks.json`:
- JSON syntax validation
- `local` network defined
- `testnet` network defined

**Evidence:** Lines 346-402.

### Stage 7: Documentation (lines 413-427)
- **QUDAG_IMPLEMENTATION.md** (required)
- **RUVECTOR_QUDAG_AUDIT_REPORT.md** (optional)

**Evidence:** Lines 416-425.

### Stage 8: Service Status (lines 434-466)
**Optional checks if Docker running:**
- QuDAG node container status
- Health endpoint: `http://localhost:9000/health`
- MCP endpoint: `http://localhost:9000/mcp`
- agentic-network Docker network

**Evidence:** Lines 437-463.

---

## JSON Validation Strategy

**Evidence:** Lines 59-84

**Cross-platform approach:**
1. **Python fallback** (lines 59-67)
   ```bash
   json_validate_with_python() {
       python - "$file" <<'PY'
   import json, sys
   with open(sys.argv[1], 'r', encoding='utf-8') as f:
       json.load(f)
   PY
   }
   ```

2. **jq preferred** (lines 69-71)
   - Faster, more common in shell scripts
   - Fallback if not available

3. **Detection logic** (lines 73-84)
   - Try Python first
   - Then jq
   - Return code 2 if neither available

---

## Key Features

### 1. Rust Crate Checks

**Evidence:** Lines 98-149

Searches `rust/Cargo.toml` for GitHub repository references:
```bash
grep -q 'qudag-core.*=.*{.*git.*=.*"https://github.com/ruvnet/qudag"'
```

**Crates checked:**
- qudag-core (core DAG)
- qudag-crypto (cryptographic primitives)
- qudag-network (libp2p-based P2P)
- qudag-mcp (Model Context Protocol)
- blake3 (hash function)
- chacha20poly1305 (AEAD cipher)

### 2. MCP Server Validation

**Evidence:** Lines 280-324

Counts tools and resources in MCP manifest:
```python
data = json.load(f)
tools = len(data.get('tools', []) or [])
resources = len(data.get('resources', []) or [])
```

**Expectations:**
- Tools: >= 10 (graph operations, node management, consensus)
- Resources: >= 5 (DAG state, network info, metrics)

### 3. Optional Live Checks

**Evidence:** Lines 437-466

Only runs if Docker available and QuDAG running:
- Container health status
- HTTP health endpoint (port 9000)
- MCP endpoint accessibility
- Network existence (agentic-network)

---

## Next Steps

**Displayed on success (lines 486-491):**
```bash
docker network create agentic-network
docker compose -f docker-compose.qudag.yml up -d
curl http://localhost:9000/health
curl -X POST http://localhost:9000/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","method":"tools/list","id":1}'
docker compose -f docker-compose.qudag.yml --profile testnet up -d
```

---

## QuDAG Architecture Context

**P3-011: P2P Coordination Layer**
- **Core:** Quantum-resistant DAG consensus
- **Crypto:** Post-quantum cryptography (NIST standards)
- **Network:** libp2p-based P2P with onion routing support
- **MCP:** Model Context Protocol for AI agent coordination
- **Consensus:** Byzantine fault-tolerant DAG ordering

**Integration Points:**
- NATS JetStream for event sourcing
- RuVector for vector embeddings
- Argo Workflows for orchestration
- Open-Lovable for UI

---

## References

### Source Code
- **Main script:** `scripts/verify-qudag.sh` (501 lines)
- **Rust checks:** lines 90-152
- **Docker checks:** lines 160-190
- **Environment checks:** lines 198-229
- **Pixi checks:** lines 237-266
- **MCP checks:** lines 274-335
- **Network checks:** lines 343-405
- **Service checks:** lines 434-466

### Related Files
- **Rust manifest:** `rust/Cargo.toml`
- **Docker Compose:** `docker-compose.qudag.yml`
- **Environment:** `.env.qudag.example`
- **Pixi config:** `pixi.toml`
- **MCP manifest:** `manifests/mcp/qudag-server.json`
- **Networks:** `manifests/qudag/networks.json`
- **Documentation:** `docs/implementation/QUDAG_IMPLEMENTATION.md`
- **Audit report:** `docs/audits/RUVECTOR_QUDAG_AUDIT_REPORT.md`

### External Resources
- [QuDAG Repository](https://github.com/ruvnet/qudag)
- [libp2p](https://libp2p.io/) - P2P networking
- [Model Context Protocol](https://modelcontextprotocol.io/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 15/60 contracts complete
