# API Reference

**Status:** Complete
**Last Updated:** 2026-01-14
**Purpose:** Document internal service APIs

---

## Overview

This document provides API reference for the primary services in FlexStack. All APIs follow REST conventions unless otherwise noted.

---

## LocalAI API

**Base URL:** `http://localhost:8080`
**Documentation:** OpenAI-compatible API

### Health Check

```http
GET /readyz
```

**Response:**
```json
{"status": "ok"}
```

### List Models

```http
GET /v1/models
```

**Response:**
```json
{
  "object": "list",
  "data": [
    {
      "id": "gpt-4",
      "object": "model",
      "created": 1686935002,
      "owned_by": "localai"
    }
  ]
}
```

### Chat Completions

```http
POST /v1/chat/completions
Content-Type: application/json
```

**Request:**
```json
{
  "model": "gpt-4",
  "messages": [
    {"role": "system", "content": "You are a helpful assistant."},
    {"role": "user", "content": "Hello!"}
  ],
  "temperature": 0.7,
  "max_tokens": 1000
}
```

**Response:**
```json
{
  "id": "chatcmpl-123",
  "object": "chat.completion",
  "created": 1677652288,
  "model": "gpt-4",
  "choices": [{
    "index": 0,
    "message": {
      "role": "assistant",
      "content": "Hello! How can I help you today?"
    },
    "finish_reason": "stop"
  }],
  "usage": {
    "prompt_tokens": 12,
    "completion_tokens": 9,
    "total_tokens": 21
  }
}
```

### Embeddings

```http
POST /v1/embeddings
Content-Type: application/json
```

**Request:**
```json
{
  "model": "text-embedding-ada-002",
  "input": "The food was delicious"
}
```

**Response:**
```json
{
  "object": "list",
  "data": [{
    "object": "embedding",
    "embedding": [0.0023064255, -0.009327292, ...],
    "index": 0
  }],
  "model": "text-embedding-ada-002",
  "usage": {
    "prompt_tokens": 5,
    "total_tokens": 5
  }
}
```

### Image Generation

```http
POST /v1/images/generations
Content-Type: application/json
```

**Request:**
```json
{
  "prompt": "a white siamese cat",
  "n": 1,
  "size": "512x512"
}
```

### Text-to-Speech

```http
POST /v1/audio/speech
Content-Type: application/json
```

**Request:**
```json
{
  "model": "tts-1",
  "input": "Hello, how are you?",
  "voice": "alloy"
}
```

---

## AGiXT API

**Base URL:** `http://localhost:7437`
**Authentication:** API key in header `Authorization: Bearer <key>`

### Health Check

```http
GET /health
```

**Response:**
```json
{"status": "healthy"}
```

### List Agents

```http
GET /api/agents
Authorization: Bearer <AGIXT_API_KEY>
```

**Response:**
```json
{
  "agents": [
    {
      "name": "default",
      "status": "active",
      "provider": "openai"
    }
  ]
}
```

### Create Agent

```http
POST /api/agents
Authorization: Bearer <AGIXT_API_KEY>
Content-Type: application/json
```

**Request:**
```json
{
  "name": "my-agent",
  "settings": {
    "provider": "openai",
    "AI_MODEL": "gpt-4",
    "AI_TEMPERATURE": 0.7
  }
}
```

### Run Agent Prompt

```http
POST /api/agents/{agent_name}/prompt
Authorization: Bearer <AGIXT_API_KEY>
Content-Type: application/json
```

**Request:**
```json
{
  "prompt": "Write a hello world program in Python",
  "prompt_category": "default",
  "prompt_name": "default"
}
```

**Response:**
```json
{
  "response": "Here's a simple hello world program in Python:\n\n```python\nprint('Hello, World!')\n```"
}
```

### Execute Command

```http
POST /api/agents/{agent_name}/command
Authorization: Bearer <AGIXT_API_KEY>
Content-Type: application/json
```

**Request:**
```json
{
  "command_name": "Write to File",
  "command_args": {
    "filename": "hello.py",
    "content": "print('Hello, World!')"
  }
}
```

### List Chains

```http
GET /api/chains
Authorization: Bearer <AGIXT_API_KEY>
```

### Run Chain

```http
POST /api/chains/{chain_name}/run
Authorization: Bearer <AGIXT_API_KEY>
Content-Type: application/json
```

**Request:**
```json
{
  "user_input": "Research and summarize AI trends",
  "agent_override": "research-agent"
}
```

---

## NATS JetStream API

**Client Port:** `4222`
**Monitoring:** `http://localhost:8222`

### Connection (Go Example)

```go
nc, err := nats.Connect("nats://localhost:4222")
js, err := nc.JetStream()
```

### Publish Message

```go
js.Publish("orders.new", []byte(`{"order_id": 123}`))
```

### Subscribe to Stream

```go
sub, err := js.Subscribe("orders.*", func(m *nats.Msg) {
    fmt.Printf("Received: %s\n", string(m.Data))
    m.Ack()
})
```

### Monitoring Endpoints

```http
# Server info
GET http://localhost:8222/varz

# Connection info
GET http://localhost:8222/connz

# JetStream info
GET http://localhost:8222/jsz
```

---

## Temporal API

**gRPC Port:** `7233`
**UI:** `http://localhost:8088`

### Workflow Example (Python)

```python
from temporalio import workflow, activity
from temporalio.client import Client

@activity.defn
async def say_hello(name: str) -> str:
    return f"Hello, {name}!"

@workflow.defn
class GreetingWorkflow:
    @workflow.run
    async def run(self, name: str) -> str:
        return await workflow.execute_activity(
            say_hello,
            name,
            start_to_close_timeout=timedelta(seconds=10),
        )

# Start workflow
client = await Client.connect("localhost:7233")
result = await client.execute_workflow(
    GreetingWorkflow.run,
    "World",
    id="greeting-workflow",
    task_queue="greeting-task-queue",
)
```

### REST API (via UI)

```http
# List workflows
GET http://localhost:8088/api/v1/namespaces/default/workflows

# Get workflow
GET http://localhost:8088/api/v1/namespaces/default/workflows/{workflow_id}
```

---

## OPA Policy API

**Base URL:** `http://localhost:8181`

### Health Check

```http
GET /health
```

### Query Policy

```http
POST /v1/data/{package}/{rule}
Content-Type: application/json
```

**Example - Authorization Check:**
```http
POST /v1/data/ros2/authz/allow
Content-Type: application/json

{
  "input": {
    "user": {"id": "user-123", "role": "admin"},
    "action": "write",
    "resource": {"type": "config"}
  }
}
```

**Response:**
```json
{"result": true}
```

### List Policies

```http
GET /v1/policies
```

### Upload Policy

```http
PUT /v1/policies/{policy_id}
Content-Type: text/plain

package example.authz
default allow = false
allow { input.user.role == "admin" }
```

---

## Kong Admin API

**Base URL:** `http://localhost:8001`

### List Services

```http
GET /services
```

### Create Service

```http
POST /services
Content-Type: application/json

{
  "name": "my-api",
  "url": "http://upstream:8080"
}
```

### Create Route

```http
POST /services/{service}/routes
Content-Type: application/json

{
  "paths": ["/api/v1"],
  "methods": ["GET", "POST"]
}
```

### List Plugins

```http
GET /plugins
```

### Add Rate Limiting

```http
POST /services/{service}/plugins
Content-Type: application/json

{
  "name": "rate-limiting",
  "config": {
    "minute": 100
  }
}
```

---

## Keycloak API

**Base URL:** `http://localhost:8082`
**Admin Console:** `http://localhost:8082/admin`

### Get Token

```http
POST /realms/{realm}/protocol/openid-connect/token
Content-Type: application/x-www-form-urlencoded

grant_type=password&client_id=my-client&username=user&password=pass
```

**Response:**
```json
{
  "access_token": "eyJhbGciOiJSUzI1NiIsInR5cCI...",
  "expires_in": 300,
  "refresh_token": "eyJhbGciOiJIUzI1NiIsInR5cCI...",
  "token_type": "Bearer"
}
```

### User Info

```http
GET /realms/{realm}/protocol/openid-connect/userinfo
Authorization: Bearer <access_token>
```

### Admin API

```http
# List users
GET /admin/realms/{realm}/users
Authorization: Bearer <admin_token>

# Create user
POST /admin/realms/{realm}/users
Authorization: Bearer <admin_token>
Content-Type: application/json

{
  "username": "newuser",
  "email": "user@example.com",
  "enabled": true
}
```

---

## Vault API

**Base URL:** `http://localhost:8200`
**Authentication:** Token in header `X-Vault-Token: <token>`

### Health Check

```http
GET /v1/sys/health
```

### Read Secret

```http
GET /v1/secret/data/{path}
X-Vault-Token: <token>
```

**Response:**
```json
{
  "data": {
    "data": {
      "api_key": "secret-value"
    },
    "metadata": {
      "created_time": "2024-01-01T00:00:00Z",
      "version": 1
    }
  }
}
```

### Write Secret

```http
POST /v1/secret/data/{path}
X-Vault-Token: <token>
Content-Type: application/json

{
  "data": {
    "api_key": "new-secret-value"
  }
}
```

---

## Prometheus API

**Base URL:** `http://localhost:9090`

### Query

```http
GET /api/v1/query?query=up
```

### Range Query

```http
GET /api/v1/query_range?query=rate(http_requests_total[5m])&start=2024-01-01T00:00:00Z&end=2024-01-01T01:00:00Z&step=60s
```

### Targets

```http
GET /api/v1/targets
```

### Alerts

```http
GET /api/v1/alerts
```

---

## MinIO API

**Base URL:** `http://localhost:9000`
**Console:** `http://localhost:9001`

MinIO implements the S3-compatible API. Use any S3 client:

```bash
# Using AWS CLI
aws --endpoint-url http://localhost:9000 s3 ls

# Using mc (MinIO Client)
mc alias set local http://localhost:9000 minioadmin minioadmin
mc ls local/
```

---

## Error Codes

### Standard HTTP Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 201 | Created |
| 400 | Bad Request |
| 401 | Unauthorized |
| 403 | Forbidden |
| 404 | Not Found |
| 429 | Rate Limited |
| 500 | Server Error |
| 503 | Service Unavailable |

### Service-Specific Errors

| Service | Error Format |
|---------|--------------|
| LocalAI | OpenAI-compatible error objects |
| AGiXT | `{"error": "message", "code": "ERROR_CODE"}` |
| Kong | `{"message": "error details"}` |
| OPA | `{"code": "code", "message": "details"}` |

---

## Rate Limits

| Service | Default Limit | Configurable |
|---------|---------------|--------------|
| LocalAI | None | Via Kong plugin |
| AGiXT | None | Via Kong plugin |
| Kong Admin | None | Via plugin |
| Keycloak | Built-in | Yes |

---

## References

- [LocalAI Documentation](https://localai.io/api/)
- [AGiXT Documentation](https://github.com/Josh-XT/AGiXT)
- [NATS Documentation](https://docs.nats.io/)
- [Temporal Documentation](https://docs.temporal.io/)
- [OPA Documentation](https://www.openpolicyagent.org/docs/)
- [Kong Documentation](https://docs.konghq.com/)
- [Keycloak Documentation](https://www.keycloak.org/docs/)
- [Vault Documentation](https://developer.hashicorp.com/vault/docs)
