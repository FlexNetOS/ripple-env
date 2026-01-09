# OPA Authorization Policy for ROS2 Humble Agentic Environment
# =============================================================================
# This policy controls access to agents, tools, and resources
# =============================================================================

package ros2.authz

import future.keywords.if
import future.keywords.in

# Default deny
default allow := false

# Allow admin users full access
allow if {
    input.user.role == "admin"
}

# Allow agents to access their own resources
allow if {
    input.user.type == "agent"
    input.resource.owner == input.user.id
}

# Allow read access to public resources
allow if {
    input.action == "read"
    input.resource.visibility == "public"
}

# Allow agents to invoke tools they have permission for
allow if {
    input.action == "invoke"
    input.resource.type == "tool"
    input.user.permissions[_] == input.resource.name
}

# Allow LocalAI model inference for authenticated users
allow if {
    input.action == "inference"
    input.resource.type == "model"
    input.user.authenticated == true
}

# Rate limiting check (returns remaining quota)
rate_limit_ok if {
    input.user.requests_remaining > 0
}

# Audit logging decision
audit := {
    "allowed": allow,
    "user": input.user.id,
    "action": input.action,
    "resource": input.resource.name,
    "timestamp": time.now_ns()
}
