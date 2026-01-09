# Artifact Index DNA

**Status**: Initial scaffolding (P0-006)

## Purpose

Indexes content-addressed artifacts and tracks their provenance. Serves as the registry for IPFS CIDs with metadata and replication coordination.

## Zomes

### Integrity Zomes (Data Structures & Validation)

1. **artifacts_integrity** - Defines artifact structures
2. **provenance_integrity** - Defines provenance structures
3. **replication_integrity** - Defines replication structures

### Coordinator Zomes (Business Logic)

1. **artifacts** - Artifact indexing and metadata
2. **provenance** - Provenance tracking
3. **replication** - Replication coordination

## Entry Types

- **Artifact**: IPFS CID with metadata (size, type, hash)
- **Provenance**: Build/creation provenance chain
- **ReplicationStatus**: Per-node replication state

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
