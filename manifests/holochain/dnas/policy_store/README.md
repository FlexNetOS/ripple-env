# Policy Store DNA

**Status**: Initial scaffolding (P0-006)

## Purpose

Stores and distributes OPA policies for decentralized enforcement. Provides distributed policy cache and decision logging.

## Zomes

### Integrity Zomes (Data Structures & Validation)

1. **policies_integrity** - Defines policy structures
2. **decisions_integrity** - Defines decision structures
3. **audit_integrity** - Defines audit log structures

### Coordinator Zomes (Business Logic)

1. **policies** - Policy CRUD and distribution
2. **decisions** - Policy decision caching
3. **audit** - Decision audit logging

## Entry Types

- **Policy**: OPA policy bundles with versioning
- **Decision**: Cached policy decisions with TTL
- **AuditLog**: Decision audit trail

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
