# Holochain DNA Definitions

This directory contains Holochain DNA definitions for P2P coordination.

## Structure

```
dnas/
├── README.md              # This file
├── coordination/          # Coordination DNA
│   ├── dna.yaml          # DNA manifest
│   └── zomes/            # Zome implementations
├── messaging/             # P2P messaging DNA
│   ├── dna.yaml
│   └── zomes/
└── storage/               # Distributed storage DNA
    ├── dna.yaml
    └── zomes/
```

## DNA Types

### Coordination DNA
- **Purpose**: Agent coordination and task distribution
- **Zomes**:
  - `tasks`: Task queue and assignment
  - `agents`: Agent registry and capabilities
  - `consensus`: Consensus protocol for decisions

### Messaging DNA
- **Purpose**: Decentralized messaging between agents
- **Zomes**:
  - `channels`: Message channels and subscriptions
  - `relay`: Message relay and routing
  - `encryption`: End-to-end encryption

### Storage DNA
- **Purpose**: Content-addressed distributed storage
- **Zomes**:
  - `files`: File storage and retrieval
  - `metadata`: File metadata and indexing
  - `replication`: Replication strategy

## Development

### Prerequisites

```bash
# Install Holochain CLI tools
nix develop .#full

# Holochain should be available
holochain --version
hc --version
```

### Building DNAs

```bash
# Build a DNA
cd dnas/coordination
hc dna pack .

# Build all DNAs
./scripts/build-holochain-dnas.sh
```

### Testing

```bash
# Run DNA tests
cd dnas/coordination
hc test

# Run all tests
./scripts/test-holochain-dnas.sh
```

## Deployment

DNAs are deployed via the Holochain conductor:

```bash
# Deploy to local conductor
hc sandbox generate dnas/coordination --run

# Deploy to production (via manifests)
kubectl apply -f manifests/holochain/conductor.yaml
```

## Resources

- [Holochain Documentation](https://developer.holochain.org/)
- [DNA Specification](https://developer.holochain.org/concepts/dna/)
- [Zome Development Guide](https://developer.holochain.org/concepts/zomes/)
- [BUILDKIT_STARTER_SPEC.md](../../docs/BUILDKIT_STARTER_SPEC.md) - Layer 11: Holochain P2P
