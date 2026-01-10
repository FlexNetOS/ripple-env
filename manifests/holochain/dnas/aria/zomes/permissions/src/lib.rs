use hdk::prelude::*;
use permissions_integrity::*;

/// Anchor for all permissions
const ALL_PERMISSIONS_ANCHOR: &str = "all_permissions";
/// Anchor for all roles
const ALL_ROLES_ANCHOR: &str = "all_roles";

// ============================================================================
// Permission CRUD Operations
// ============================================================================

/// Create a new permission
#[hdk_extern]
pub fn create_permission(permission: Permission) -> ExternResult<ActionHash> {
    // Create the permission entry
    let action_hash = create_entry(EntryTypes::Permission(permission.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&permission)?;

    // Link from the all_permissions anchor
    let anchor_hash = hash_entry(&ALL_PERMISSIONS_ANCHOR)?;
    create_link(
        anchor_hash,
        entry_hash.clone(),
        LinkTypes::AllPermissions,
        permission.id.as_bytes().to_vec(),
    )?;

    Ok(action_hash)
}

/// Get a permission by its action hash
#[hdk_extern]
pub fn get_permission(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// Get a permission by its ID
#[hdk_extern]
pub fn get_permission_by_id(permission_id: String) -> ExternResult<Option<Record>> {
    let all_permissions = get_all_permissions(())?;

    for record in all_permissions {
        if let Some(entry) = record.entry().as_option() {
            if let Ok(permission) = Permission::try_from(entry.clone()) {
                if permission.id == permission_id {
                    return Ok(Some(record));
                }
            }
        }
    }

    Ok(None)
}

/// Update a permission
#[hdk_extern]
pub fn update_permission(input: UpdatePermissionInput) -> ExternResult<ActionHash> {
    update_entry(input.original_action_hash, &input.updated_permission)
}

/// Input for updating a permission
#[derive(Serialize, Deserialize, Debug)]
pub struct UpdatePermissionInput {
    pub original_action_hash: ActionHash,
    pub updated_permission: Permission,
}

/// Delete a permission
#[hdk_extern]
pub fn delete_permission(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get all permissions in the network
#[hdk_extern]
pub fn get_all_permissions(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = hash_entry(&ALL_PERMISSIONS_ANCHOR)?;

    // Get all links from the anchor
    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllPermissions)?.build(),
    )?;

    // Fetch all the records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

// ============================================================================
// Role Management
// ============================================================================

/// Create a new role
#[hdk_extern]
pub fn create_role(role: Role) -> ExternResult<ActionHash> {
    // Create the role entry
    let action_hash = create_entry(EntryTypes::Role(role.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&role)?;

    // Link from the all_roles anchor
    let anchor_hash = hash_entry(&ALL_ROLES_ANCHOR)?;
    create_link(
        anchor_hash,
        entry_hash.clone(),
        LinkTypes::AllRoles,
        role.id.as_bytes().to_vec(),
    )?;

    // Create links from role to each permission
    for perm_id in &role.permissions {
        if let Ok(Some(perm_record)) = get_permission_by_id(perm_id.clone()) {
            let perm_entry_hash = perm_record.action_address();
            create_link(
                entry_hash.clone(),
                perm_entry_hash.clone(),
                LinkTypes::RoleToPermission,
                perm_id.as_bytes().to_vec(),
            )?;
            // Bidirectional link
            if let Some(perm_entry) = perm_record.entry().as_option() {
                let perm_hash = hash_entry(perm_entry)?;
                create_link(
                    perm_hash,
                    entry_hash.clone(),
                    LinkTypes::PermissionToRole,
                    role.id.as_bytes().to_vec(),
                )?;
            }
        }
    }

    Ok(action_hash)
}

/// Get a role by its action hash
#[hdk_extern]
pub fn get_role(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// Get a role by its ID
#[hdk_extern]
pub fn get_role_by_id(role_id: String) -> ExternResult<Option<Record>> {
    let all_roles = get_all_roles(())?;

    for record in all_roles {
        if let Some(entry) = record.entry().as_option() {
            if let Ok(role) = Role::try_from(entry.clone()) {
                if role.id == role_id {
                    return Ok(Some(record));
                }
            }
        }
    }

    Ok(None)
}

/// Update a role
#[hdk_extern]
pub fn update_role(input: UpdateRoleInput) -> ExternResult<ActionHash> {
    update_entry(input.original_action_hash, &input.updated_role)
}

/// Input for updating a role
#[derive(Serialize, Deserialize, Debug)]
pub struct UpdateRoleInput {
    pub original_action_hash: ActionHash,
    pub updated_role: Role,
}

/// Delete a role
#[hdk_extern]
pub fn delete_role(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get all roles in the network
#[hdk_extern]
pub fn get_all_roles(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = hash_entry(&ALL_ROLES_ANCHOR)?;

    // Get all links from the anchor
    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllRoles)?.build(),
    )?;

    // Fetch all the records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Assign a permission to an existing role
#[hdk_extern]
pub fn assign_permission_to_role(input: AssignPermissionInput) -> ExternResult<ActionHash> {
    // Get the existing role
    let role_record = get_role(input.role_action_hash.clone())?;

    match role_record {
        Some(record) => {
            if let Some(entry) = record.entry().as_option() {
                let mut role = Role::try_from(entry.clone()).map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "Failed to deserialize role: {:?}",
                        e
                    )))
                })?;

                // Add permission if not already present
                if !role.permissions.contains(&input.permission_id) {
                    role.permissions.push(input.permission_id.clone());

                    // Update the role entry
                    let new_action_hash = update_entry(input.role_action_hash, &role)?;

                    // Create link from role to permission
                    let role_entry_hash = hash_entry(&role)?;
                    if let Ok(Some(perm_record)) = get_permission_by_id(input.permission_id.clone())
                    {
                        let perm_entry_hash = perm_record.action_address();
                        create_link(
                            role_entry_hash.clone(),
                            perm_entry_hash.clone(),
                            LinkTypes::RoleToPermission,
                            input.permission_id.as_bytes().to_vec(),
                        )?;
                    }

                    return Ok(new_action_hash);
                }

                // Permission already assigned, return existing action hash
                Ok(record.action_address().clone())
            } else {
                Err(wasm_error!(WasmErrorInner::Guest(
                    "Role entry not found".to_string()
                )))
            }
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(
            "Role not found".to_string()
        ))),
    }
}

/// Input for assigning a permission to a role
#[derive(Serialize, Deserialize, Debug)]
pub struct AssignPermissionInput {
    pub role_action_hash: ActionHash,
    pub permission_id: String,
}

/// Remove a permission from a role
#[hdk_extern]
pub fn remove_permission_from_role(input: AssignPermissionInput) -> ExternResult<ActionHash> {
    // Get the existing role
    let role_record = get_role(input.role_action_hash.clone())?;

    match role_record {
        Some(record) => {
            if let Some(entry) = record.entry().as_option() {
                let mut role = Role::try_from(entry.clone()).map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "Failed to deserialize role: {:?}",
                        e
                    )))
                })?;

                // Remove permission if present
                role.permissions.retain(|p| p != &input.permission_id);

                // Update the role entry
                let new_action_hash = update_entry(input.role_action_hash, &role)?;

                Ok(new_action_hash)
            } else {
                Err(wasm_error!(WasmErrorInner::Guest(
                    "Role entry not found".to_string()
                )))
            }
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(
            "Role not found".to_string()
        ))),
    }
}

/// Get all permissions for a role
#[hdk_extern]
pub fn get_permissions_for_role(role_action_hash: ActionHash) -> ExternResult<Vec<Record>> {
    let role_record = get_role(role_action_hash)?;

    match role_record {
        Some(record) => {
            if let Some(entry) = record.entry().as_option() {
                let role = Role::try_from(entry.clone()).map_err(|e| {
                    wasm_error!(WasmErrorInner::Guest(format!(
                        "Failed to deserialize role: {:?}",
                        e
                    )))
                })?;

                let mut permissions = Vec::new();
                for perm_id in &role.permissions {
                    if let Ok(Some(perm_record)) = get_permission_by_id(perm_id.clone()) {
                        permissions.push(perm_record);
                    }
                }

                Ok(permissions)
            } else {
                Ok(Vec::new())
            }
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(
            "Role not found".to_string()
        ))),
    }
}

// ============================================================================
// Role Binding Operations
// ============================================================================

/// Bind a role to an agent
#[hdk_extern]
pub fn bind_role_to_agent(binding: RoleBinding) -> ExternResult<ActionHash> {
    // Create the binding entry
    let action_hash = create_entry(EntryTypes::RoleBinding(binding.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&binding)?;

    // Link from role to agent
    let role_anchor = hash_entry(&binding.role_id)?;
    create_link(
        role_anchor,
        entry_hash.clone(),
        LinkTypes::RoleToAgent,
        binding.agent_pubkey.get_raw_39().to_vec(),
    )?;

    // Link from agent to their binding
    create_link(
        binding.agent_pubkey.clone(),
        entry_hash,
        LinkTypes::AgentToRoleBinding,
        binding.role_id.as_bytes().to_vec(),
    )?;

    Ok(action_hash)
}

/// Unbind a role from an agent (delete the binding)
#[hdk_extern]
pub fn unbind_role_from_agent(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get all role bindings for the current agent
#[hdk_extern]
pub fn get_my_role_bindings(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    get_role_bindings_for_agent(agent_pubkey)
}

/// Get all role bindings for a specific agent
#[hdk_extern]
pub fn get_role_bindings_for_agent(agent: AgentPubKey) -> ExternResult<Vec<Record>> {
    // Get links from agent to role bindings
    let links = get_links(
        GetLinksInputBuilder::try_new(agent, LinkTypes::AgentToRoleBinding)?.build(),
    )?;

    // Fetch all binding records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get all agents bound to a specific role
#[hdk_extern]
pub fn get_agents_with_role(role_id: String) -> ExternResult<Vec<Record>> {
    let role_anchor = hash_entry(&role_id)?;

    // Get links from role to bindings
    let links = get_links(
        GetLinksInputBuilder::try_new(role_anchor, LinkTypes::RoleToAgent)?.build(),
    )?;

    // Fetch all binding records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get all permissions for an agent (aggregated from all their roles)
#[hdk_extern]
pub fn get_agent_permissions(agent: AgentPubKey) -> ExternResult<Vec<Record>> {
    // Get all role bindings for the agent
    let bindings = get_role_bindings_for_agent(agent)?;

    let mut all_permissions = Vec::new();
    let mut seen_permission_ids = std::collections::HashSet::new();

    for binding_record in bindings {
        if let Some(entry) = binding_record.entry().as_option() {
            if let Ok(binding) = RoleBinding::try_from(entry.clone()) {
                // Check if binding has expired
                if let Some(expires_at) = binding.expires_at {
                    let now = sys_time()?;
                    if now > expires_at {
                        continue; // Skip expired bindings
                    }
                }

                // Get the role for this binding
                if let Ok(Some(role_record)) = get_role_by_id(binding.role_id.clone()) {
                    if let Some(role_entry) = role_record.entry().as_option() {
                        if let Ok(role) = Role::try_from(role_entry.clone()) {
                            // Get all permissions for this role
                            for perm_id in &role.permissions {
                                if !seen_permission_ids.contains(perm_id) {
                                    if let Ok(Some(perm_record)) =
                                        get_permission_by_id(perm_id.clone())
                                    {
                                        all_permissions.push(perm_record);
                                        seen_permission_ids.insert(perm_id.clone());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Ok(all_permissions)
}

/// Get all permissions for the current agent
#[hdk_extern]
pub fn get_my_permissions(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    get_agent_permissions(agent_pubkey)
}

/// Check if an agent has a specific permission
#[hdk_extern]
pub fn check_agent_permission(input: CheckPermissionInput) -> ExternResult<bool> {
    let permissions = get_agent_permissions(input.agent)?;

    for record in permissions {
        if let Some(entry) = record.entry().as_option() {
            if let Ok(permission) = Permission::try_from(entry.clone()) {
                // Check exact match or wildcard
                let action_matches =
                    permission.action == input.action || permission.action == "*";
                let resource_matches =
                    permission.resource == input.resource || permission.resource == "*";

                if action_matches && resource_matches {
                    return Ok(true);
                }

                // Check resource prefix match (e.g., "agent:*" matches "agent:123")
                if permission.resource.ends_with(":*") {
                    let prefix = &permission.resource[..permission.resource.len() - 1];
                    if input.resource.starts_with(prefix) && action_matches {
                        return Ok(true);
                    }
                }
            }
        }
    }

    Ok(false)
}

/// Input for checking a permission
#[derive(Serialize, Deserialize, Debug)]
pub struct CheckPermissionInput {
    pub agent: AgentPubKey,
    pub action: String,
    pub resource: String,
}

/// Check if the current agent has a specific permission
#[hdk_extern]
pub fn check_my_permission(input: CheckMyPermissionInput) -> ExternResult<bool> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    check_agent_permission(CheckPermissionInput {
        agent: agent_pubkey,
        action: input.action,
        resource: input.resource,
    })
}

/// Input for checking own permission
#[derive(Serialize, Deserialize, Debug)]
pub struct CheckMyPermissionInput {
    pub action: String,
    pub resource: String,
}

/// Get agent info (wrapper for convenience)
#[hdk_extern]
pub fn whoami(_: ()) -> ExternResult<AgentInfo> {
    agent_info()
}
