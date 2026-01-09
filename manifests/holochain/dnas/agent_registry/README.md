# Agent Registry DNA

**Status**: Initial scaffolding (P0-006)

## Purpose

Tracks all FlexStack nodes, their capabilities (compute, storage, inference), and availability status. This DNA serves as the discovery layer for the distributed agent mesh.

## Zomes

### Integrity Zomes (Data Structures & Validation)

1. **capabilities_integrity** - Defines node capability structures
2. **heartbeat_integrity** - Defines heartbeat/liveness structures
3. **discovery_integrity** - Defines peer discovery structures

### Coordinator Zomes (Business Logic)

1. **capabilities** - Node capability advertisement and querying
2. **heartbeat** - Heartbeat management and availability tracking
3. **discovery** - Peer discovery and network topology

## Entry Types

- **NodeCapability**: Hardware specs, service endpoints, resource limits
- **Heartbeat**: Periodic liveness signals with metrics
- **PeerInfo**: Agent addresses, connection info, reputation

## Building

```bash
# From this directory
hc dna pack ./workdir
```

## Testing

```bash
# Run tests (once zomes are implemented)
cargo test
```

## Next Steps

1. Implement integrity zome entry types
2. Implement coordinator zome functions
3. Add validation rules
4. Write unit tests
5. Integration testing with conductor

## References

- [Holochain Documentation](https://developer.holochain.org)
- [HDK Reference](https://docs.rs/hdk)
- [HDI Reference](https://docs.rs/hdi)
