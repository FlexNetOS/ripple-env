# Resource Mesh DNA

**Status**: Initial scaffolding (P0-006)

## Purpose

Manages distributed resource allocation across the agent mesh. Handles compute/storage/inference scheduling, allocation, and accounting.

## Zomes

### Integrity Zomes (Data Structures & Validation)

1. **resources_integrity** - Defines resource pool structures
2. **scheduling_integrity** - Defines task scheduling structures
3. **accounting_integrity** - Defines usage tracking structures

### Coordinator Zomes (Business Logic)

1. **resources** - Resource pool management and discovery
2. **scheduling** - Task allocation and scheduling
3. **accounting** - Usage tracking and metering

## Entry Types

- **ResourcePool**: Available compute/storage/inference capacity
- **TaskRequest**: Resource allocation requests
- **Allocation**: Active resource allocations
- **UsageRecord**: Resource consumption metrics

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
