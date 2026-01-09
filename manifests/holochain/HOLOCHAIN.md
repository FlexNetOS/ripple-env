# Holochain Integration for ARIA/FlexStack

**Implementation Status**: P0-006 (Initial scaffolding complete)

## Overview

Holochain provides the distributed coordination layer for ARIA, enabling agent-centric P2P networking without centralized servers. This integration implements 5 core DNAs that replace centralized services with distributed alternatives.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    ARIA FlexStack                       │
├─────────────────────────────────────────────────────────┤
│  AGiXT Agents  │  ROS2 Nodes  │  Rust Services         │
├─────────────────────────────────────────────────────────┤
│              Holochain Conductor                        │
│  ┌───────────┬───────────┬───────────┬───────────┐    │
│  │  Agent    │ Resource  │  Policy   │ Artifact  │    │
│  │ Registry  │   Mesh    │   Store   │   Index   │    │
│  └───────────┴───────────┴───────────┴───────────┘    │
│  ┌───────────────────────────────────────────────┐    │
│  │           Memory Shards                        │    │
│  └───────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────┤
│              P2P Networking (Holochain)                 │
│  WebRTC Transport │ Gossip │ DHT │ Lair Keystore       │
└─────────────────────────────────────────────────────────┘
```

## Core DNAs

### 1. Agent Registry (`agent_registry`)
- **Purpose**: Node capability advertisement and discovery
- **Zomes**: capabilities, heartbeat, discovery
- **Replaces**: Centralized service registry
- **Status**: Scaffolding complete

### 2. Resource Mesh (`resource_mesh`)
- **Purpose**: Compute/storage/inference allocation and scheduling
- **Zomes**: resources, scheduling, accounting
- **Replaces**: Centralized resource manager
- **Status**: Scaffolding complete

### 3. Policy Store (`policy_store`)
- **Purpose**: Distributed OPA policy cache and decisions
- **Zomes**: policies, decisions, audit
- **Replaces**: Centralized policy server
- **Status**: Scaffolding complete

### 4. Artifact Index (`artifact_index`)
- **Purpose**: IPFS CID registry and provenance tracking
- **Zomes**: artifacts, provenance, replication
- **Replaces**: Centralized artifact registry
- **Status**: Scaffolding complete

### 5. Memory Shards (`memory_shards`)
- **Purpose**: Vector DB partition coordination
- **Zomes**: shards, routing, sync
- **Replaces**: Centralized vector DB coordinator
- **Status**: Scaffolding complete

## Binary Components (flake.nix)

The following Holochain binaries are provided via Nix:

- **holochain** (v0.4.0) - Conductor runtime
- **hc** (v0.4.0) - Development CLI
- **lair-keystore** (v0.4.5) - Secure keystore

## Rust Integration (Cargo.toml)

Development dependencies:

```toml
hdk = "0.4.0"              # Holochain Development Kit
hdi = "0.5.0"              # Holochain Deterministic Integrity
holochain = "0.4.0"        # Core conductor library
holochain_types = "0.4.0"  # Type definitions
holochain_zome_types = "0.4.0"  # Zome type definitions
```

## Directory Structure

```
manifests/holochain/
├── HOLOCHAIN.md           # This file
├── conductor.yaml         # Conductor configuration
├── networks.json          # Network configuration
├── versions.json          # Version pinning
└── dnas/
    ├── index.json         # DNA registry
    ├── agent_registry/    # Agent discovery DNA
    │   ├── dna.yaml       # DNA manifest
    │   ├── README.md      # Documentation
    │   ├── zomes/         # Compiled WASM zomes
    │   └── workdir/       # Build directory
    ├── resource_mesh/     # Resource allocation DNA
    ├── policy_store/      # Policy distribution DNA
    ├── artifact_index/    # Artifact registry DNA
    └── memory_shards/     # Vector DB coordination DNA
```

## Verification Commands

### 1. Check Holochain Installation

```bash
# Verify binaries are available
holochain --version
hc --version
lair-keystore --version
```

Expected output:
```
holochain 0.4.0
hc 0.4.0
lair-keystore 0.4.5
```

### 2. Generate Test Sandbox

```bash
# Create a test sandbox with a sample DNA
hc sandbox generate ./test-sandbox
cd ./test-sandbox
hc sandbox run
```

### 3. Validate DNA Manifests

```bash
# Validate each DNA manifest
cd /home/user/ros2-humble-env/manifests/holochain/dnas/agent_registry
hc dna init

cd /home/user/ros2-humble-env/manifests/holochain/dnas/resource_mesh
hc dna init

cd /home/user/ros2-humble-env/manifests/holochain/dnas/policy_store
hc dna init

cd /home/user/ros2-humble-env/manifests/holochain/dnas/artifact_index
hc dna init

cd /home/user/ros2-humble-env/manifests/holochain/dnas/memory_shards
hc dna init
```

### 4. Test Conductor Configuration

```bash
# Validate conductor.yaml
holochain --config-path /home/user/ros2-humble-env/manifests/holochain/conductor.yaml --check
```

### 5. Verify Rust Dependencies

```bash
# Check that Holochain crates are available
cd /home/user/ros2-humble-env/rust
cargo tree | grep holochain
```

## Next Steps (Implementation Roadmap)

### Phase 1: Integrity Zomes (P1-014-1)
- [ ] Define entry types for each DNA
- [ ] Implement validation rules
- [ ] Create link types for relationships
- [ ] Write unit tests

### Phase 2: Coordinator Zomes (P1-014-2)
- [ ] Implement CRUD operations
- [ ] Add query functions
- [ ] Implement signal handlers
- [ ] Write integration tests

### Phase 3: Build & Package (P1-014-3)
- [ ] Compile zomes to WASM
- [ ] Package DNAs with `hc dna pack`
- [ ] Generate DNA hashes
- [ ] Update index.json with real hashes

### Phase 4: Conductor Integration (P1-014-4)
- [ ] Configure hApp bundles
- [ ] Set up admin interfaces
- [ ] Configure app interfaces
- [ ] Test conductor startup

### Phase 5: Service Integration (P1-014-5)
- [ ] Integrate with AGiXT agents
- [ ] Connect to ROS2 nodes
- [ ] Wire up Rust services
- [ ] End-to-end testing

## References

- [Holochain Documentation](https://developer.holochain.org)
- [Holochain GitHub](https://github.com/holochain/holochain)
- [HDK Documentation](https://docs.rs/hdk)
- [HDI Documentation](https://docs.rs/hdi)
- [Holochain Gym](https://holochain-gym.github.io/)
- [BUILDKIT_STARTER_SPEC.md](../../BUILDKIT_STARTER_SPEC.md) - Section 9.7

## Support

For Holochain-specific issues:
- Forum: https://forum.holochain.org
- Discord: https://discord.gg/holochain
- GitHub Issues: https://github.com/holochain/holochain/issues

For ARIA integration issues:
- See project documentation
- Contact L11 Coordination team lead
