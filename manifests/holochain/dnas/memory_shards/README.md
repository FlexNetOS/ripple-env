# Memory Shards DNA

**Status**: Initial scaffolding (P0-006)

## Purpose

Coordinates distributed vector memory across nodes. Handles partition routing, shard allocation, and synchronization for the distributed vector database.

## Zomes

### Integrity Zomes (Data Structures & Validation)

1. **shards_integrity** - Defines shard structures
2. **routing_integrity** - Defines routing structures
3. **sync_integrity** - Defines sync structures

### Coordinator Zomes (Business Logic)

1. **shards** - Shard allocation and management
2. **routing** - Query routing and load balancing
3. **sync** - Shard synchronization

## Entry Types

- **Shard**: Vector partition with metadata
- **RoutingTable**: Shard-to-node mapping
- **SyncState**: Cross-shard synchronization state

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
