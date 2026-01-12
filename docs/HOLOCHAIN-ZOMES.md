# Holochain Zomes Implementation Guide

**Document Version**: 1.0.0
**Status**: Scaffolding Phase Complete (P2-003)
**Last Updated**: 2026-01-10

## Overview

This document provides a comprehensive implementation plan for Holochain zomes (WebAssembly modules) in the ARIA/FlexStack ecosystem. It covers the scaffolding, development, testing, and deployment of zomes across all DNAs.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [ARIA DNA - Identity Management](#aria-dna---identity-management)
3. [Build and Compilation](#build-and-compilation)
4. [Testing Strategy](#testing-strategy)
5. [Deployment Procedures](#deployment-procedures)
6. [Integration Patterns](#integration-patterns)
7. [Development Roadmap](#development-roadmap)
8. [Troubleshooting](#troubleshooting)

## Architecture Overview

### Zome Structure

Each DNA consists of two types of zomes:

1. **Integrity Zomes**: Define entry types, link types, and validation rules
   - Pure data definitions
   - Deterministic validation
   - Uses HDI (Holochain Deterministic Integrity) crate

2. **Coordinator Zomes**: Implement business logic and CRUD operations
   - External function calls
   - Network operations
   - Uses HDK (Holochain Development Kit) crate

### DNA Catalog

| DNA | Purpose | Status | Zomes |
|-----|---------|--------|-------|
| agent_registry | Node capability advertisement | Scaffolding | capabilities, heartbeat, discovery |
| resource_mesh | Resource allocation | Scaffolding | resources, scheduling, accounting |
| policy_store | OPA policy distribution | Scaffolding | policies, decisions, audit |
| artifact_index | IPFS CID registry | Scaffolding | artifacts, provenance, replication |
| memory_shards | Vector DB coordination | Scaffolding | shards, routing, sync |
| **aria** | **Agent identity management** | **Scaffolding Complete** | **identity** |

## ARIA DNA - Identity Management

### Current Implementation Status

**Phase**: Scaffolding Complete (P2-003)
**Duration**: 16 hours allocated
**Completion Date**: 2026-01-10

#### Delivered Components

```
manifests/holochain/dnas/aria/
├── dna.yaml                              # DNA manifest
├── README.md                             # DNA documentation
├── workdir/                              # Build directory
└── zomes/
    ├── identity_integrity/               # Integrity zome
    │   ├── Cargo.toml
    │   └── src/
    │       └── lib.rs                    # Entry types and validation
    └── identity/                         # Coordinator zome
        ├── Cargo.toml
        └── src/
            └── lib.rs                    # CRUD operations
```

### Entry Types

#### 1. AgentProfile

Stores identity information for ARIA agents in the DHT.

```rust
pub struct AgentProfile {
    pub name: String,                     // Display name (1-100 chars)
    pub capabilities: Vec<String>,        // Agent capabilities (min 1)
    pub created_at: Timestamp,            // Creation timestamp
    pub metadata: Option<String>,         // Optional JSON metadata
}
```

**Validation Rules**:
- Name: 1-100 characters, non-empty
- Capabilities: At least one, all non-empty
- Metadata: Optional, valid JSON if provided

**Use Cases**:
- Agent registration and discovery
- Capability matching for task assignment
- Agent metadata storage

#### 2. AgentCredential

Stores cryptographic credentials and authentication data.

```rust
pub struct AgentCredential {
    pub public_key: String,               // Public key identifier
    pub credential_type: String,          // ed25519, secp256k1, rsa2048, rsa4096
    pub issued_at: Timestamp,             // Issuance timestamp
    pub expires_at: Option<Timestamp>,    // Optional expiration
    pub issuer: AgentPubKey,              // Issuer's public key
}
```

**Validation Rules**:
- Public key: Non-empty
- Credential type: Must be one of the recognized types
- Expiration: Must be after issuance if set

**Use Cases**:
- Cryptographic identity verification
- Multi-key agent support
- Credential lifecycle management

#### 3. RoleAssignment

Maps agents to roles for RBAC (Role-Based Access Control).

```rust
pub struct RoleAssignment {
    pub role: String,                     // Role identifier (alphanumeric + - _)
    pub agent: AgentPubKey,               // Agent assigned to role
    pub assigned_at: Timestamp,           // Assignment timestamp
    pub expires_at: Option<Timestamp>,    // Optional expiration
    pub assigned_by: AgentPubKey,         // Assigner's public key
}
```

**Validation Rules**:
- Role: Non-empty, alphanumeric + underscore + hyphen
- Expiration: Must be after assignment if set
- Audit trail: Immutable record of assigner

**Use Cases**:
- Role-based access control
- Permission delegation
- Time-limited access grants

### Link Types

The identity zome uses the following link types to create relationships in the DHT:

1. **AgentToProfile**: Links agent public key to their profile
2. **AgentToCredential**: Links agent to their credentials
3. **AgentToRole**: Links agent to their role assignments
4. **RoleToAgent**: Links role anchor to assigned agents
5. **AllProfiles**: Links global anchor to all profiles (for discovery)

### Public Functions

#### Profile Management

| Function | Input | Output | Description |
|----------|-------|--------|-------------|
| `create_profile` | `AgentProfile` | `ActionHash` | Create new profile |
| `get_my_profile` | `()` | `Option<Record>` | Get current agent's profile |
| `get_profile_for_agent` | `AgentPubKey` | `Option<Record>` | Get specific agent's profile |
| `update_profile` | `AgentProfile` | `ActionHash` | Update current profile |
| `delete_profile` | `()` | `ActionHash` | Delete current profile |
| `get_all_profiles` | `()` | `Vec<Record>` | Get all profiles in network |
| `search_profiles_by_capability` | `String` | `Vec<Record>` | Search by capability |

#### Credential Management

| Function | Input | Output | Description |
|----------|-------|--------|-------------|
| `create_credential` | `AgentCredential` | `ActionHash` | Issue new credential |
| `get_my_credentials` | `()` | `Vec<Record>` | Get current agent's credentials |
| `get_credentials_for_agent` | `AgentPubKey` | `Vec<Record>` | Get specific agent's credentials |
| `revoke_credential` | `ActionHash` | `ActionHash` | Revoke a credential |

#### Role Management

| Function | Input | Output | Description |
|----------|-------|--------|-------------|
| `assign_role` | `RoleAssignment` | `ActionHash` | Assign role to agent |
| `get_my_roles` | `()` | `Vec<Record>` | Get current agent's roles |
| `get_roles_for_agent` | `AgentPubKey` | `Vec<Record>` | Get specific agent's roles |
| `get_agents_with_role` | `String` | `Vec<Record>` | Get all agents with role |
| `revoke_role` | `ActionHash` | `ActionHash` | Revoke role assignment |

#### Utility

| Function | Input | Output | Description |
|----------|-------|--------|-------------|
| `whoami` | `()` | `AgentInfo` | Get current agent info |

## Build and Compilation

### Prerequisites

Ensure the following are installed and available:

```bash
# Verify Rust toolchain with WASM target
rustc --version
rustup target list | grep wasm32-unknown-unknown

# If not installed:
rustup target add wasm32-unknown-unknown

# Verify Holochain tools
holochain --version  # Should be 0.4.0+
hc --version         # Should be 0.4.0+
```

### Building ARIA Identity Zomes

#### Step 1: Build Integrity Zome

```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria/zomes/identity_integrity

# Build for WASM
cargo build --release --target wasm32-unknown-unknown

# Verify output
ls -lh target/wasm32-unknown-unknown/release/identity_integrity.wasm
```

Expected output: `identity_integrity.wasm` (~200-500 KB)

#### Step 2: Build Coordinator Zome

```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria/zomes/identity

# Build for WASM
cargo build --release --target wasm32-unknown-unknown

# Verify output
ls -lh target/wasm32-unknown-unknown/release/identity.wasm
```

Expected output: `identity.wasm` (~300-800 KB)

#### Step 3: Copy WASM to DNA Directory

```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria

# Create zomes directory if it doesn't exist
mkdir -p zomes

# Copy compiled WASM files
cp zomes/identity_integrity/target/wasm32-unknown-unknown/release/identity_integrity.wasm zomes/
cp zomes/identity/target/wasm32-unknown-unknown/release/identity.wasm zomes/

# Verify
ls -lh zomes/*.wasm
```

#### Step 4: Package the DNA

```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria

# Package the DNA
hc dna pack .

# This creates aria.dna in the current directory
ls -lh aria.dna
```

Expected output: `aria.dna` (~1-2 MB)

### Automated Build Script

Create a build script for convenience:

```bash
#!/bin/bash
# build-aria-dna.sh

set -e

ARIA_DIR="/home/user/ripple-env/manifests/holochain/dnas/aria"

echo "Building ARIA DNA..."

# Build integrity zome
echo "Building identity_integrity zome..."
cd "$ARIA_DIR/zomes/identity_integrity"
cargo build --release --target wasm32-unknown-unknown

# Build coordinator zome
echo "Building identity coordinator zome..."
cd "$ARIA_DIR/zomes/identity"
cargo build --release --target wasm32-unknown-unknown

# Copy WASM files
echo "Copying WASM files..."
cd "$ARIA_DIR"
mkdir -p zomes
cp zomes/identity_integrity/target/wasm32-unknown-unknown/release/identity_integrity.wasm zomes/
cp zomes/identity/target/wasm32-unknown-unknown/release/identity.wasm zomes/

# Package DNA
echo "Packaging DNA..."
hc dna pack .

echo "Build complete: $ARIA_DIR/aria.dna"
```

## Testing Strategy

### Unit Tests

Unit tests validate individual functions and validation rules.

#### Integrity Zome Tests

Create `identity_integrity/src/lib.rs` tests:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_agent_profile_valid() {
        let profile = AgentProfile {
            name: "test-agent".to_string(),
            capabilities: vec!["inference".to_string()],
            created_at: Timestamp::now(),
            metadata: None,
        };

        let result = validate_agent_profile(&profile).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Valid));
    }

    #[test]
    fn test_validate_agent_profile_empty_name() {
        let profile = AgentProfile {
            name: "".to_string(),
            capabilities: vec!["inference".to_string()],
            created_at: Timestamp::now(),
            metadata: None,
        };

        let result = validate_agent_profile(&profile).unwrap();
        assert!(matches!(result, ValidateCallbackResult::Invalid(_)));
    }

    // Add more tests...
}
```

Run tests:

```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria/zomes/identity_integrity
cargo test
```

#### Coordinator Zome Tests

Coordinator zome tests require a Holochain test harness:

```rust
// In identity/tests/integration_test.rs
use hdk::prelude::*;
use holochain::test_utils::*;

#[tokio::test]
async fn test_create_and_get_profile() {
    // Test setup
    let (conductor, _agent, _cell_id) = setup_conductor().await;

    // Create profile
    let profile = AgentProfile {
        name: "test-agent".to_string(),
        capabilities: vec!["inference".to_string()],
        created_at: Timestamp::now(),
        metadata: None,
    };

    let action_hash = create_profile(profile.clone()).unwrap();

    // Retrieve profile
    let retrieved = get_my_profile(()).unwrap();
    assert!(retrieved.is_some());

    // Verify content
    let record = retrieved.unwrap();
    let retrieved_profile: AgentProfile = record.entry().to_app_option().unwrap().unwrap();
    assert_eq!(retrieved_profile.name, profile.name);
}
```

### Integration Tests

Integration tests validate zome interactions across the network.

Create test scenarios:

1. **Profile Lifecycle**: Create, update, delete profile
2. **Credential Issuance**: Issue and revoke credentials
3. **Role Management**: Assign and revoke roles
4. **Search and Discovery**: Find agents by capability
5. **Multi-Agent Scenarios**: Multiple agents interacting

### Test Coverage Goals

- **Unit Tests**: 80%+ coverage of validation logic
- **Integration Tests**: 100% coverage of public functions
- **End-to-End Tests**: Critical user journeys

## Deployment Procedures

### Local Development Deployment

#### Step 1: Start Lair Keystore

```bash
# Create lair directory
mkdir -p ~/.local/share/ripple/lair

# Initialize lair keystore
lair-keystore init --lair-root ~/.local/share/ripple/lair

# Start lair server (in background)
lair-keystore server --lair-root ~/.local/share/ripple/lair &

# Save the connection URL
export LAIR_CONNECTION_URL="unix://$HOME/.local/share/ripple/lair/socket"
```

#### Step 2: Start Holochain Conductor

```bash
# Start conductor with config
holochain --config-path /home/user/ripple-env/manifests/holochain/conductor.yaml

# Conductor will start on:
# - Admin interface: ws://localhost:8888
# - App interface: ws://localhost:8889
```

#### Step 3: Install ARIA DNA

Using `hc` CLI:

```bash
# Generate agent key
hc keygen --path ~/.local/share/ripple/holochain/agent-key

# Install DNA
hc app install \
  /home/user/ripple-env/manifests/holochain/dnas/aria/aria.dna \
  --agent-key $(cat ~/.local/share/ripple/holochain/agent-key) \
  --installed-app-id aria-identity \
  --membrane-proof "{}"

# Enable the app
hc app enable aria-identity
```

Using Admin API (JavaScript):

```javascript
import { AdminWebsocket } from '@holochain/client';

const adminWs = await AdminWebsocket.connect('ws://localhost:8888');

// Install DNA
const installedApp = await adminWs.installApp({
  installed_app_id: 'aria-identity',
  agent_key: agentPubKey,
  dnas: [
    {
      path: '/home/user/ripple-env/manifests/holochain/dnas/aria/aria.dna',
      role_id: 'aria',
      membrane_proof: null
    }
  ]
});

// Enable the app
await adminWs.enableApp({ installed_app_id: 'aria-identity' });
```

### Production Deployment

#### Docker Deployment

Create a Dockerfile for the conductor:

```dockerfile
FROM nixos/nix:latest

# Copy Nix flake
COPY flake.nix /app/
COPY flake.lock /app/
WORKDIR /app

# Build environment with Holochain
RUN nix develop --command bash -c "holochain --version"

# Copy conductor config and DNAs
COPY manifests/holochain /app/manifests/holochain

# Expose ports
EXPOSE 8888 8889

# Start conductor
CMD ["nix", "develop", "--command", "holochain", "--config-path", "/app/manifests/holochain/conductor.yaml"]
```

Build and run:

```bash
docker build -t aria-conductor .
docker run -p 8888:8888 -p 8889:8889 aria-conductor
```

#### Kubernetes Deployment

See `/home/user/ripple-env/manifests/distributed/holochain/` for Kubernetes manifests.

## Integration Patterns

### AGiXT Integration

AGiXT agents can interact with Holochain zomes via the app interface.

#### Python Client Example

```python
from holochain_client import AgentClient

# Connect to conductor
client = AgentClient("ws://localhost:8889", "aria-identity")

# Create profile
profile = {
    "name": "agixt-agent-01",
    "capabilities": ["llm-inference", "text-generation"],
    "created_at": int(time.time() * 1e6),  # Microseconds
    "metadata": '{"model": "claude-3-opus"}'
}

action_hash = await client.call_zome(
    "aria",
    "identity",
    "create_profile",
    profile
)

# Search for inference agents
inference_agents = await client.call_zome(
    "aria",
    "identity",
    "search_profiles_by_capability",
    "llm-inference"
)

for agent in inference_agents:
    print(f"Found: {agent['name']} - {agent['capabilities']}")
```

### ROS2 Integration

ROS2 nodes can publish/subscribe to Holochain signals.

```python
import rclpy
from rclpy.node import Node
from holochain_client import AgentClient

class HolochainBridge(Node):
    def __init__(self):
        super().__init__('holochain_bridge')
        self.hc_client = AgentClient("ws://localhost:8889", "aria-identity")

    async def on_agent_signal(self, signal):
        # Received signal from Holochain
        if signal['type'] == 'NewProfile':
            self.get_logger().info(f"New agent registered: {signal['data']}")

    async def publish_capability(self, capability):
        # Update profile with new capability
        profile = await self.hc_client.call_zome(
            "aria", "identity", "get_my_profile", None
        )
        profile['capabilities'].append(capability)
        await self.hc_client.call_zome(
            "aria", "identity", "update_profile", profile
        )
```

### Policy Store Integration

Combine ARIA identity with OPA policy evaluation:

```python
# Get agent roles
roles = await client.call_zome("aria", "identity", "get_my_roles", None)

# Query policy store
policy_decision = await client.call_zome(
    "policy_store",
    "decisions",
    "evaluate_policy",
    {
        "policy": "resource_access",
        "input": {
            "agent": agent_pubkey,
            "roles": [r['role'] for r in roles],
            "action": "inference.run",
            "resource": "gpu-a100"
        }
    }
)

if policy_decision['allow']:
    # Proceed with action
    pass
```

## Development Roadmap

### Phase 1: Scaffolding (COMPLETE)
**Duration**: 16 hours
**Status**: Complete (P2-003)

- [x] Create directory structure
- [x] Define entry types (AgentProfile, AgentCredential, RoleAssignment)
- [x] Implement validation rules
- [x] Create CRUD operations
- [x] Write Cargo.toml configurations
- [x] Create DNA manifest (dna.yaml)
- [x] Document implementation

### Phase 2: Testing and Refinement (NEXT)
**Duration**: 24 hours
**Status**: Not started

- [ ] Write unit tests for validation logic
- [ ] Implement integration tests
- [ ] Add test coverage tooling
- [ ] Performance benchmarking
- [ ] Refine validation rules based on testing
- [ ] Add comprehensive error handling

### Phase 3: Additional Zomes
**Duration**: 40 hours
**Status**: Not started

- [ ] Capabilities zome (skill matching)
  - Entry types: SkillDefinition, SkillEndorsement
  - Functions: Skill search, endorsement verification
- [ ] Reputation zome (trust scoring)
  - Entry types: ReputationScore, ReputationEvent
  - Functions: Calculate scores, track history
- [ ] Delegation zome (permission delegation)
  - Entry types: Delegation, DelegationChain
  - Functions: Delegate permissions, verify chains

### Phase 4: Integration
**Duration**: 32 hours
**Status**: Not started

- [ ] AGiXT client library
- [ ] ROS2 bridge node
- [ ] Policy Store integration
- [ ] Resource Mesh integration
- [ ] End-to-end integration tests

### Phase 5: Production Hardening
**Duration**: 40 hours
**Status**: Not started

- [ ] Security audit
- [ ] Performance optimization
- [ ] Monitoring and observability
- [ ] Deployment automation
- [ ] Documentation completion
- [ ] User guides and tutorials

### Total Estimated Time

- **Phase 1**: 16 hours (COMPLETE)
- **Phase 2**: 24 hours
- **Phase 3**: 40 hours
- **Phase 4**: 32 hours
- **Phase 5**: 40 hours
- **Total**: 152 hours (~19 days @ 8 hours/day)

## Troubleshooting

### Common Build Issues

#### Issue: WASM target not found

```
error: can't find crate for `std`
```

**Solution**:
```bash
rustup target add wasm32-unknown-unknown
```

#### Issue: HDK version mismatch

```
error: failed to resolve: use of undeclared crate or module `hdk`
```

**Solution**: Verify HDK version in Cargo.toml matches installed version:
```bash
cargo tree | grep hdk
# Update Cargo.toml if needed
```

#### Issue: DNA packaging fails

```
error: WASM file not found at zomes/identity.wasm
```

**Solution**: Ensure WASM files are copied to the zomes directory:
```bash
cd /home/user/ripple-env/manifests/holochain/dnas/aria
mkdir -p zomes
cp zomes/*/target/wasm32-unknown-unknown/release/*.wasm zomes/
```

### Runtime Issues

#### Issue: Conductor won't start

```
error: failed to connect to lair keystore
```

**Solution**: Start lair keystore first:
```bash
lair-keystore server --lair-root ~/.local/share/ripple/lair &
```

#### Issue: DNA installation fails

```
error: membrane proof required
```

**Solution**: Provide empty membrane proof if none required:
```bash
hc app install ... --membrane-proof "{}"
```

#### Issue: Validation failures

```
error: Entry validation failed
```

**Solution**: Check validation logic and ensure entry data meets all constraints. Enable debug logging:
```bash
RUST_LOG=debug holochain --config-path conductor.yaml
```

### Performance Issues

#### Issue: Slow DHT operations

**Possible Causes**:
- Network latency
- Insufficient peers
- Arc settings

**Solutions**:
1. Adjust gossip parameters in conductor.yaml
2. Increase peer connections
3. Use arc_clamping: full for development

#### Issue: Large WASM files

**Solutions**:
1. Enable release optimizations in Cargo.toml
2. Use wasm-opt for further compression:
```bash
wasm-opt -Oz input.wasm -o output.wasm
```

## References

### Documentation

- [Holochain Core Concepts](https://developer.holochain.org/concepts/)
- [HDK API Reference](https://docs.rs/hdk)
- [HDI API Reference](https://docs.rs/hdi)
- [Holochain Gym Exercises](https://holochain-gym.github.io/)

### Source Code

- ARIA DNA: `/home/user/ripple-env/manifests/holochain/dnas/aria/`
- Holochain manifests: `/home/user/ripple-env/manifests/holochain/`
- BUILDKIT_STARTER_SPEC: `/home/user/ripple-env/BUILDKIT_STARTER_SPEC.md`

### Community

- [Holochain Forum](https://forum.holochain.org)
- [Holochain Discord](https://discord.gg/holochain)
- [GitHub Issues](https://github.com/holochain/holochain/issues)

## Appendix A: File Structure

Complete file structure for ARIA DNA:

```
/home/user/ripple-env/manifests/holochain/dnas/aria/
├── dna.yaml                                    # DNA manifest
├── README.md                                   # DNA-specific documentation
├── aria.dna                                    # Packaged DNA (after build)
├── workdir/                                    # Build artifacts
└── zomes/
    ├── identity_integrity.wasm                 # Compiled integrity zome
    ├── identity.wasm                           # Compiled coordinator zome
    ├── identity_integrity/
    │   ├── Cargo.toml                          # Integrity zome dependencies
    │   └── src/
    │       └── lib.rs                          # Entry types and validation
    └── identity/
        ├── Cargo.toml                          # Coordinator zome dependencies
        └── src/
            └── lib.rs                          # CRUD operations
```

## Appendix B: Build Checklist

Use this checklist when building and deploying ARIA DNA:

- [ ] Verify Rust toolchain: `rustc --version`
- [ ] Verify WASM target: `rustup target list | grep wasm32`
- [ ] Verify Holochain tools: `holochain --version && hc --version`
- [ ] Build integrity zome: `cargo build --release --target wasm32-unknown-unknown`
- [ ] Build coordinator zome: `cargo build --release --target wasm32-unknown-unknown`
- [ ] Copy WASM files to zomes directory
- [ ] Package DNA: `hc dna pack .`
- [ ] Verify DNA package: `ls -lh aria.dna`
- [ ] Start lair keystore
- [ ] Start conductor
- [ ] Install DNA via admin interface
- [ ] Enable installed app
- [ ] Test basic operations (create profile, get profile)
- [ ] Verify in logs and monitoring

## Appendix C: Validation Test Cases

Complete set of test cases for validation logic:

### AgentProfile Validation

1. **Valid Cases**
   - Standard profile with single capability
   - Profile with multiple capabilities
   - Profile with metadata JSON
   - Profile with 100-character name (boundary)

2. **Invalid Cases**
   - Empty name
   - Name exceeding 100 characters
   - Empty capabilities list
   - Capability with empty string
   - Invalid JSON in metadata (optional check)

### AgentCredential Validation

1. **Valid Cases**
   - ed25519 credential
   - secp256k1 credential
   - rsa2048 credential
   - rsa4096 credential
   - Credential with expiration
   - Credential without expiration

2. **Invalid Cases**
   - Empty public key
   - Invalid credential type
   - Expiration before issuance
   - Expiration equal to issuance

### RoleAssignment Validation

1. **Valid Cases**
   - Role with alphanumeric name
   - Role with underscores
   - Role with hyphens
   - Assignment with expiration
   - Assignment without expiration

2. **Invalid Cases**
   - Empty role name
   - Role with special characters (!, @, #, etc.)
   - Expiration before assignment
   - Expiration equal to assignment

---

**Document Maintenance**: Update this document as new zomes are implemented and integration patterns evolve.
