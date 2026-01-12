# ARIA DNA

**Status**: Scaffolding Phase (P2-003)
**DNA UID**: `00000000-0000-0000-0000-000000000006`

## Overview

The ARIA DNA provides decentralized identity and access management for the ARIA agent ecosystem. It enables agents to:

- Create and manage identity profiles
- Issue and verify cryptographic credentials
- Assign and manage role-based access control (RBAC)
- Discover agents by capabilities

## Architecture

```
┌─────────────────────────────────────────┐
│            ARIA DNA                     │
├─────────────────────────────────────────┤
│  Identity Zome                          │
│  ┌───────────────────────────────────┐ │
│  │  Entry Types:                     │ │
│  │  - AgentProfile                   │ │
│  │  - AgentCredential                │ │
│  │  - RoleAssignment                 │ │
│  └───────────────────────────────────┘ │
│  ┌───────────────────────────────────┐ │
│  │  Operations:                      │ │
│  │  - Create/Update/Delete Profile   │ │
│  │  - Issue/Revoke Credentials       │ │
│  │  - Assign/Revoke Roles            │ │
│  │  - Search by Capability           │ │
│  └───────────────────────────────────┘ │
└─────────────────────────────────────────┘
```

## Zomes

### Identity Integrity Zome

Defines the data structures and validation rules for identity management.

**Entry Types:**

1. **AgentProfile**
   - `name: String` - Display name for the agent
   - `capabilities: Vec<String>` - List of agent capabilities
   - `created_at: Timestamp` - Profile creation timestamp
   - `metadata: Option<String>` - Optional JSON metadata

2. **AgentCredential**
   - `public_key: String` - Public key identifier
   - `credential_type: String` - Type (ed25519, secp256k1, rsa2048, rsa4096)
   - `issued_at: Timestamp` - Issuance timestamp
   - `expires_at: Option<Timestamp>` - Optional expiration
   - `issuer: AgentPubKey` - Issuer's public key

3. **RoleAssignment**
   - `role: String` - Role identifier (admin, operator, viewer, etc.)
   - `agent: AgentPubKey` - Agent assigned to role
   - `assigned_at: Timestamp` - Assignment timestamp
   - `expires_at: Option<Timestamp>` - Optional expiration
   - `assigned_by: AgentPubKey` - Assigner's public key

**Link Types:**

- `AgentToProfile` - Links agent to their profile
- `AgentToCredential` - Links agent to their credentials
- `AgentToRole` - Links agent to their role assignments
- `RoleToAgent` - Links role to assigned agents
- `AllProfiles` - Anchor for all profiles in the network

**Validation Rules:**

- Agent names must be 1-100 characters
- Agents must have at least one capability
- Credential types must be recognized
- Expiration times must be after issuance/assignment times
- Role names can only contain alphanumeric characters, underscores, and hyphens

### Identity Coordinator Zome

Implements the business logic for identity management operations.

**Functions:**

Profile Management:
- `create_profile(profile: AgentProfile) -> ActionHash`
- `get_my_profile() -> Option<Record>`
- `get_profile_for_agent(agent: AgentPubKey) -> Option<Record>`
- `update_profile(profile: AgentProfile) -> ActionHash`
- `delete_profile() -> ActionHash`
- `get_all_profiles() -> Vec<Record>`
- `search_profiles_by_capability(capability: String) -> Vec<Record>`

Credential Management:
- `create_credential(credential: AgentCredential) -> ActionHash`
- `get_my_credentials() -> Vec<Record>`
- `get_credentials_for_agent(agent: AgentPubKey) -> Vec<Record>`
- `revoke_credential(action_hash: ActionHash) -> ActionHash`

Role Management:
- `assign_role(assignment: RoleAssignment) -> ActionHash`
- `get_my_roles() -> Vec<Record>`
- `get_roles_for_agent(agent: AgentPubKey) -> Vec<Record>`
- `get_agents_with_role(role: String) -> Vec<Record>`
- `revoke_role(action_hash: ActionHash) -> ActionHash`

Utility:
- `whoami() -> AgentInfo`

## Use Cases

### Agent Registration

```rust
// Create a new agent profile
let profile = AgentProfile {
    name: "inference-node-01".to_string(),
    capabilities: vec![
        "llm-inference".to_string(),
        "text-generation".to_string(),
        "embedding".to_string(),
    ],
    created_at: sys_time()?,
    metadata: Some(r#"{"model": "claude-3-opus", "gpu": "A100"}"#.to_string()),
};

let action_hash = create_profile(profile)?;
```

### Capability Discovery

```rust
// Find all agents that can perform inference
let inference_agents = search_profiles_by_capability("llm-inference".to_string())?;

for record in inference_agents {
    let profile: AgentProfile = record.entry().to_app_option()?.unwrap();
    println!("Found agent: {} with capabilities: {:?}", profile.name, profile.capabilities);
}
```

### Credential Issuance

```rust
// Issue a credential to an agent
let credential = AgentCredential {
    public_key: "ed25519:abc123...".to_string(),
    credential_type: "ed25519".to_string(),
    issued_at: sys_time()?,
    expires_at: Some(sys_time()? + Duration::from_secs(86400 * 365)), // 1 year
    issuer: agent_info()?.agent_latest_pubkey,
};

let action_hash = create_credential(credential)?;
```

### Role-Based Access Control

```rust
// Assign admin role to an agent
let assignment = RoleAssignment {
    role: "admin".to_string(),
    agent: target_agent_pubkey,
    assigned_at: sys_time()?,
    expires_at: None, // No expiration
    assigned_by: agent_info()?.agent_latest_pubkey,
};

let action_hash = assign_role(assignment)?;

// Later, check who has admin access
let admins = get_agents_with_role("admin".to_string())?;
```

## Building

### Build the Zomes

```bash
# Build the integrity zome
cd /home/user/ripple-env/manifests/holochain/dnas/aria/zomes/identity_integrity
cargo build --release --target wasm32-unknown-unknown

# Build the coordinator zome
cd /home/user/ripple-env/manifests/holochain/dnas/aria/zomes/identity
cargo build --release --target wasm32-unknown-unknown
```

### Package the DNA

```bash
# From the ARIA DNA directory
cd /home/user/ripple-env/manifests/holochain/dnas/aria

# Copy the compiled WASM files
cp zomes/identity_integrity/target/wasm32-unknown-unknown/release/identity_integrity.wasm zomes/
cp zomes/identity/target/wasm32-unknown-unknown/release/identity.wasm zomes/

# Package the DNA
hc dna pack .
```

## Testing

### Unit Tests

```bash
# Run integrity zome tests
cd zomes/identity_integrity
cargo test

# Run coordinator zome tests
cd zomes/identity
cargo test
```

### Integration Tests

Integration tests will be added in Phase 2 of the implementation.

## Integration with ARIA

The ARIA DNA integrates with the broader ARIA ecosystem:

1. **AGiXT Agents**: Query agent capabilities and credentials before task assignment
2. **Policy Store DNA**: Use role assignments for policy evaluation
3. **Resource Mesh DNA**: Match agent capabilities with resource requirements
4. **Agent Registry DNA**: Cross-reference with capability advertisements

## Security Considerations

1. **Profile Integrity**: Validation rules ensure profile data meets quality standards
2. **Credential Verification**: Credential types are restricted to known secure formats
3. **Role Expiration**: Optional expiration times for time-limited access
4. **Audit Trail**: All operations create immutable records in the DHT
5. **Decentralized Trust**: No central authority; trust distributed across the network

## Next Steps

### Phase 1: Complete Entry Types (CURRENT)
- [x] Define AgentProfile, AgentCredential, RoleAssignment
- [x] Implement validation rules
- [x] Create CRUD operations
- [ ] Add unit tests

### Phase 2: Additional Zomes
- [ ] Implement capabilities zome (skill matching)
- [ ] Implement reputation zome (trust scoring)
- [ ] Implement delegation zome (permission delegation)

### Phase 3: Integration
- [ ] Connect to AGiXT agent runtime
- [ ] Integrate with Policy Store DNA
- [ ] Integrate with Resource Mesh DNA
- [ ] Add comprehensive integration tests

### Phase 4: Production Readiness
- [ ] Performance optimization
- [ ] Security audit
- [ ] Documentation completion
- [ ] Deployment procedures

## References

- [Holochain HDK Documentation](https://docs.rs/hdk)
- [Holochain HDI Documentation](https://docs.rs/hdi)
- [ARIA Architecture](/docs/ARIA_ARCHITECTURE.md)
- [BUILDKIT_STARTER_SPEC.md](../../../BUILDKIT_STARTER_SPEC.md)

## Support

For issues specific to the ARIA DNA:
- Create an issue in the project repository
- Contact the L11 Coordination team
