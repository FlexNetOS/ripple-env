package ros2.authz

# Default deny - secure by default
default allow := false

# Audit logging for policy decisions
audit := {
    "timestamp": time.now_ns(),
    "decision": allow,
    "user": input.user,
    "action": input.action,
    "resource": input.resource
}

# Allow if user has admin role (full access)
allow if {
    input.user.role == "admin"
}

# Legacy support for roles array
allow if {
    input.user.roles[_] == "admin"
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

# Allow read access via GET only to public or user-owned resources
allow if {
    input.method == "GET"
    input.user.authenticated == true
    input.action == "read"
    some owner
    owner := input.resource.owner
    owner == input.user.id or input.resource.visibility == "public"
}

# Allow ROS2 topic access based on namespace
allow if {
    input.resource.type == "ros2_topic"
    input.user.namespace == input.resource.namespace
}

# Allow agents to invoke tools they have permission for
allow if {
    input.user.type == "agent"
    input.action == "invoke"
    input.resource.type == "tool"
    input.resource.name == input.user.permissions[_]
}

# Allow authenticated users to perform model inference
allow if {
    input.user.authenticated == true
    input.action == "inference"
    input.resource.type == "model"
}
