use hdi::prelude::*;

/// Permission entry type
/// Defines a specific permission that can be granted to roles
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Permission {
    /// Unique identifier for this permission
    pub id: String,
    /// Action this permission allows (e.g., "read", "write", "execute", "admin")
    pub action: String,
    /// Resource this permission applies to (e.g., "agent:*", "workflow:deployment")
    pub resource: String,
    /// Optional conditions as JSON string (e.g., time-based, attribute-based)
    pub conditions: Option<String>,
    /// Timestamp when this permission was created
    pub created_at: Timestamp,
    /// Creator's agent public key
    pub created_by: AgentPubKey,
}

/// Role entry type
/// Groups permissions together under a named role
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Role {
    /// Unique identifier for this role
    pub id: String,
    /// Human-readable name for the role (e.g., "Admin", "Operator", "Viewer")
    pub name: String,
    /// Description of the role's purpose
    pub description: Option<String>,
    /// List of permission IDs associated with this role
    pub permissions: Vec<String>,
    /// Timestamp when this role was created
    pub created_at: Timestamp,
    /// Creator's agent public key
    pub created_by: AgentPubKey,
}

/// RoleBinding entry type
/// Binds a role to an agent, granting them all permissions in that role
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RoleBinding {
    /// Role ID being bound
    pub role_id: String,
    /// Agent receiving this role
    pub agent_pubkey: AgentPubKey,
    /// Timestamp of binding
    pub bound_at: Timestamp,
    /// Optional expiration timestamp
    pub expires_at: Option<Timestamp>,
    /// Agent who created this binding
    pub bound_by: AgentPubKey,
    /// Optional metadata as JSON string
    pub metadata: Option<String>,
}

/// All entry types for the permissions zome
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_def(required_validations = 5)]
    Permission(Permission),
    #[entry_def(required_validations = 5)]
    Role(Role),
    #[entry_def(required_validations = 5)]
    RoleBinding(RoleBinding),
}

/// Link types for the permissions zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from role to permissions
    RoleToPermission,
    /// Links from permission to roles that include it
    PermissionToRole,
    /// Links from role to bound agents
    RoleToAgent,
    /// Links from agent to their role bindings
    AgentToRoleBinding,
    /// All permissions anchor
    AllPermissions,
    /// All roles anchor
    AllRoles,
}

/// Valid action types for permissions
pub const VALID_ACTIONS: &[&str] = &[
    "read",
    "write",
    "create",
    "delete",
    "update",
    "execute",
    "admin",
    "grant",
    "revoke",
    "*",
];

/// Validation rules for Permission
pub fn validate_permission(permission: &Permission) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if permission.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission ID cannot be empty".to_string(),
        ));
    }

    // ID length constraints
    if permission.id.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission ID cannot exceed 64 characters".to_string(),
        ));
    }

    // ID format validation (alphanumeric + underscore + hyphen + colon)
    if !permission
        .id
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-' || c == ':')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission ID can only contain alphanumeric characters, underscores, hyphens, and colons"
                .to_string(),
        ));
    }

    // Action must not be empty
    if permission.action.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission action cannot be empty".to_string(),
        ));
    }

    // Action must be recognized
    if !VALID_ACTIONS.contains(&permission.action.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid action: {}. Must be one of: {}",
            permission.action,
            VALID_ACTIONS.join(", ")
        )));
    }

    // Resource must not be empty
    if permission.resource.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission resource cannot be empty".to_string(),
        ));
    }

    // Resource length constraints
    if permission.resource.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Permission resource cannot exceed 256 characters".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for Role
pub fn validate_role(role: &Role) -> ExternResult<ValidateCallbackResult> {
    // ID must not be empty
    if role.id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID cannot be empty".to_string(),
        ));
    }

    // ID length constraints
    if role.id.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID cannot exceed 64 characters".to_string(),
        ));
    }

    // ID format validation (alphanumeric + underscore + hyphen)
    if !role
        .id
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID can only contain alphanumeric characters, underscores, and hyphens"
                .to_string(),
        ));
    }

    // Name must not be empty
    if role.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Role name cannot be empty".to_string(),
        ));
    }

    // Name length constraints
    if role.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Role name cannot exceed 100 characters".to_string(),
        ));
    }

    // Permissions list should not have duplicates
    let mut seen = std::collections::HashSet::new();
    for perm_id in &role.permissions {
        if !seen.insert(perm_id) {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Duplicate permission ID in role: {}",
                perm_id
            )));
        }
    }

    // Each permission ID should be valid format
    for perm_id in &role.permissions {
        if perm_id.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Permission ID in role cannot be empty".to_string(),
            ));
        }
        if perm_id.len() > 64 {
            return Ok(ValidateCallbackResult::Invalid(format!(
                "Permission ID '{}' exceeds 64 characters",
                perm_id
            )));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for RoleBinding
pub fn validate_role_binding(binding: &RoleBinding) -> ExternResult<ValidateCallbackResult> {
    // Role ID must not be empty
    if binding.role_id.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID cannot be empty".to_string(),
        ));
    }

    // Role ID length constraints
    if binding.role_id.len() > 64 {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID cannot exceed 64 characters".to_string(),
        ));
    }

    // Role ID format validation
    if !binding
        .role_id
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Role ID can only contain alphanumeric characters, underscores, and hyphens"
                .to_string(),
        ));
    }

    // If expiration is set, it must be after binding time
    if let Some(expires_at) = binding.expires_at {
        if expires_at <= binding.bound_at {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration time must be after binding time".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::Permission(permission) => validate_permission(&permission),
            EntryTypes::Role(role) => validate_role(&role),
            EntryTypes::RoleBinding(binding) => validate_role_binding(&binding),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            ..
        }) => {
            // For updates, ensure the original entry exists and validate the new entry
            match app_entry {
                EntryTypes::Permission(permission) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_permission(&permission)
                }
                EntryTypes::Role(role) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_role(&role)
                }
                EntryTypes::RoleBinding(binding) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_role_binding(&binding)
                }
            }
        }
        FlatOp::StoreEntry(OpEntry::DeleteEntry {
            original_action_hash,
            ..
        }) => {
            // Verify the entry to be deleted exists
            let _ = must_get_action(original_action_hash)?;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => {
            // Validate link creation based on link type
            match link_type {
                LinkTypes::RoleToPermission
                | LinkTypes::PermissionToRole
                | LinkTypes::RoleToAgent
                | LinkTypes::AgentToRoleBinding
                | LinkTypes::AllPermissions
                | LinkTypes::AllRoles => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            original_action: _,
            action: _,
        } => {
            // Allow link deletion
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}
