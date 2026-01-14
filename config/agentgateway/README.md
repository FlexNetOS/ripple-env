# AgentGateway Configuration

This directory contains configuration files for the AgentGateway service (P0-004).

## Files

- `config.yaml`: Main configuration file for AgentGateway
- `routes.yaml`: Custom routing configuration (optional)

## Configuration Overview

### Server Settings
- Main API: Port 8090
- Admin API: Port 8091
- Metrics: Port 8092

### MCP Protocol Support
AgentGateway supports the Model Context Protocol (MCP) for agent communication:
- Transports: stdio, http
- Max message size: 10MB
- Timeout: 60s

### Upstream Services
- **LocalAI**: http://localai:8080 - AI inference engine
- **Kong**: http://kong:8001 (admin), http://kong:8000 (proxy)

### Features
- Rate limiting: 100 req/s with burst of 200
- Circuit breaker protection
- Connection pooling
- Health checks
- CORS support
- Prometheus metrics

## Usage

The configuration is automatically mounted as read-only in the AgentGateway container:
```yaml
volumes:
  - ../config/agentgateway:/config:ro
```

## Customization

To customize the configuration:
1. Edit `config.yaml` with your desired settings
2. Restart the AgentGateway service:
   ```bash
  docker compose -f docker/docker-compose.edge.yml restart agentgateway
   ```

## Health Checks

AgentGateway health endpoint: http://localhost:8090/health

Returns:
```json
{
  "status": "healthy",
  "upstreams": {
    "localai": "healthy",
    "kong": "healthy"
  }
}
```

## Metrics

Prometheus metrics are available at: http://localhost:8092/metrics

Key metrics:
- `agentgateway_requests_total`
- `agentgateway_request_duration_seconds`
- `agentgateway_upstream_requests_total`
- `agentgateway_circuit_breaker_state`
