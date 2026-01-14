# OPA Policy Documentation

**Status:** Complete
**Last Updated:** 2026-01-14
**Location:** `config/opa/policies/`

---

## Overview

Open Policy Agent (OPA) is deployed in FlexStack for authorization and service mesh policy enforcement. OPA evaluates policy decisions at runtime using the Rego policy language.

---

## Policy Files

| File | Package | Purpose |
|------|---------|---------|
| `authz.rego` | `ros2.authz` | User/agent authorization decisions |
| `service_mesh.rego` | `service_mesh` | Service-to-service communication policies |

---

## Authorization Policy (`authz.rego`)

### Package

```rego
package ros2.authz
```

### Default Behavior

**Deny by default** - All access is denied unless explicitly allowed.

```rego
default allow := false
```

### Policy Rules

#### 1. Admin Access

Full access for users with admin role:

```rego
allow if {
    input.user.role == "admin"
}

# Legacy support for roles array
allow if {
    input.user.roles[_] == "admin"
}
```

**Input Example:**
```json
{
  "user": {"role": "admin"},
  "action": "any",
  "resource": {"type": "any"}
}
```

#### 2. Agent Resource Ownership

Agents can access resources they own:

```rego
allow if {
    input.user.type == "agent"
    input.resource.owner == input.user.id
}
```

**Input Example:**
```json
{
  "user": {"type": "agent", "id": "agent-123"},
  "action": "write",
  "resource": {"owner": "agent-123", "type": "workspace"}
}
```

#### 3. Public Resource Access

Read access to public resources:

```rego
allow if {
    input.action == "read"
    input.resource.visibility == "public"
}
```

#### 4. Authenticated User Read Access

GET requests for authenticated users to public or owned resources:

```rego
allow if {
    input.method == "GET"
    input.user.authenticated == true
    input.action == "read"
    owner := input.resource.owner
    owner == input.user.id or input.resource.visibility == "public"
}
```

#### 5. ROS2 Topic Namespace Access

Topic access based on namespace matching:

```rego
allow if {
    input.resource.type == "ros2_topic"
    input.user.namespace == input.resource.namespace
}
```

**Input Example:**
```json
{
  "user": {"namespace": "/robot1"},
  "resource": {"type": "ros2_topic", "namespace": "/robot1"}
}
```

#### 6. Agent Tool Invocation

Agents can invoke permitted tools:

```rego
allow if {
    input.user.type == "agent"
    input.action == "invoke"
    input.resource.type == "tool"
    input.resource.name == input.user.permissions[_]
}
```

**Input Example:**
```json
{
  "user": {"type": "agent", "permissions": ["bash", "read", "write"]},
  "action": "invoke",
  "resource": {"type": "tool", "name": "bash"}
}
```

#### 7. Model Inference Access

Authenticated users can perform model inference:

```rego
allow if {
    input.user.authenticated == true
    input.action == "inference"
    input.resource.type == "model"
}
```

### Audit Logging

All policy decisions are logged:

```rego
audit := {
    "timestamp": time.now_ns(),
    "decision": allow,
    "user": input.user,
    "action": input.action,
    "resource": input.resource
}
```

---

## Service Mesh Policy (`service_mesh.rego`)

### Package

```rego
package service_mesh
```

### Default Behavior

**Deny connections by default:**

```rego
default allow_connection = false
```

### Allowed Service Connections

#### NATS Messaging

The following services are allowed to connect to NATS:

| Source Service | Destination | Purpose |
|----------------|-------------|---------|
| `temporal` | `nats` | Workflow event publishing |
| `n8n` | `nats` | Automation triggers |
| `agixt` | `nats` | Agent event streaming |

```rego
allow_connection {
    input.source.service == "temporal"
    input.destination.service == "nats"
}

allow_connection {
    input.source.service == "n8n"
    input.destination.service == "nats"
}

allow_connection {
    input.source.service == "agixt"
    input.destination.service == "nats"
}
```

---

## Integration Points

### Services Using OPA

| Service | Integration | Port | Purpose |
|---------|-------------|------|---------|
| Kong Gateway | HTTP Plugin | 8181 | API authorization |
| AGiXT | Policy check | 8181 | Tool invocation authorization |
| n8n | Webhook auth | 8181 | Workflow trigger authorization |

### Docker Compose Configuration

```yaml
# docker-compose.yml
opa:
  image: openpolicyagent/opa:0.71.0-rootless
  container_name: flexstack-opa
  command:
    - "run"
    - "--server"
    - "--addr=0.0.0.0:8181"
    - "--log-level=info"
    - "--log-format=json"
    - "/policies"
  ports:
    - "8181:8181"
  volumes:
    - ../config/opa/policies:/policies:ro
```

### Kubernetes Deployment

```yaml
# charts/flexstack/values.yaml
automation:
  opa:
    enabled: true
    replicas: 2  # High availability
    resources:
      limits:
        memory: 256Mi
        cpu: "500m"
```

---

## API Usage

### Query Authorization Decision

```bash
# Check if user can read a resource
curl -X POST http://localhost:8181/v1/data/ros2/authz/allow \
  -H "Content-Type: application/json" \
  -d '{
    "input": {
      "user": {"id": "user-123", "authenticated": true},
      "action": "read",
      "resource": {"visibility": "public"}
    }
  }'

# Response: {"result": true}
```

### Query Service Mesh Policy

```bash
# Check if service connection is allowed
curl -X POST http://localhost:8181/v1/data/service_mesh/allow_connection \
  -H "Content-Type: application/json" \
  -d '{
    "input": {
      "source": {"service": "temporal"},
      "destination": {"service": "nats"}
    }
  }'

# Response: {"result": true}
```

### Get Audit Log

```bash
curl -X POST http://localhost:8181/v1/data/ros2/authz/audit \
  -H "Content-Type: application/json" \
  -d '{
    "input": {
      "user": {"id": "user-123", "role": "admin"},
      "action": "write",
      "resource": {"type": "config"}
    }
  }'
```

---

## Extending Policies

### Adding New Authorization Rules

1. Edit `config/opa/policies/authz.rego`
2. Add new rule with proper documentation:

```rego
# Allow developers to read logs
allow if {
    input.user.role == "developer"
    input.action == "read"
    input.resource.type == "logs"
}
```

3. Test the policy:

```bash
opa test config/opa/policies/
```

4. Reload OPA:

```bash
docker-compose restart opa
```

### Adding Service Mesh Rules

1. Edit `config/opa/policies/service_mesh.rego`
2. Add connection rule:

```rego
# Allow new-service to connect to redis
allow_connection {
    input.source.service == "new-service"
    input.destination.service == "redis"
}
```

---

## Policy Coverage Matrix

| Resource Type | Actions | Roles/Types | Policy Rule |
|---------------|---------|-------------|-------------|
| Any | Any | admin | Full access |
| Owned resource | Any | agent | Owner access |
| Public resource | read | Any | Public read |
| ros2_topic | Any | Same namespace | Namespace match |
| tool | invoke | agent | Permission check |
| model | inference | Authenticated | Auth check |

---

## Troubleshooting

### Policy Not Taking Effect

```bash
# Check OPA is running
curl http://localhost:8181/health

# Check policies are loaded
curl http://localhost:8181/v1/policies

# Test policy with input
curl -X POST http://localhost:8181/v1/data/ros2/authz/allow \
  -d '{"input": {...}}'
```

### Debug Policy Decisions

```bash
# Enable decision logging
docker-compose exec opa sh -c "opa run --server --set decision_logs.console=true /policies"

# Check OPA logs
docker-compose logs opa | grep decision
```

---

## References

- [OPA Documentation](https://www.openpolicyagent.org/docs/)
- [Rego Language Reference](https://www.openpolicyagent.org/docs/latest/policy-language/)
- [PROVIDERS.md](providers/PROVIDERS.md) - Provider authentication
- [ARCHITECTURE.md](architecture/ARCHITECTURE.md) - System architecture
