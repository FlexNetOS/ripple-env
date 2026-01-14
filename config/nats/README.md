# NATS JetStream Configuration

This directory contains NATS server configuration files for the agentic system.

## Files

- **nats.conf** - Main NATS server configuration with authentication and basic JetStream settings
- **jetstream.conf** - JetStream stream and consumer definitions (documentation/reference)

## JetStream Streams

The following JetStream streams are configured for message persistence:

| Stream | Subjects | Retention | Max Age | Purpose |
|--------|----------|-----------|---------|---------|
| WORKFLOWS | `workflows.>` | WorkQueue | 7 days | Workflow execution events and state transitions |
| EVENTS | `events.>` | Limits | 30 days | System and application events for observability |
| LOGS | `logs.>` | Limits | 14 days | Application and system logs |
| COMMANDS | `commands.>` | WorkQueue | 1 hour | Agent commands and control messages |
| TELEMETRY | `telemetry.>` | Limits | 7 days | Metrics, traces, and monitoring data |
| NOTIFICATIONS | `notifications.>` | Interest | 24 hours | User and system notifications |

## Setup Instructions

### 1. Start NATS Server

Using Docker Compose:

```bash
docker compose -f docker/docker-compose.messaging.yml up -d nats
```

### 2. Initialize JetStream Streams

Run the initialization script to create all streams and consumers:

```bash
# Set the admin password (from your .env file or environment)
export NATS_PASSWORD="${NATS_ADMIN_PASS}"

# Run the initialization script
./scripts/init-jetstream.sh

# Or with explicit parameters
./scripts/init-jetstream.sh \
  --nats-url nats://localhost:4222 \
  --user admin \
  --password "${NATS_PASSWORD}"
```

### 3. Verify Configuration

Verify that all streams and consumers are properly configured:

```bash
export NATS_PASSWORD="${NATS_ADMIN_PASS}"
./scripts/verify-jetstream.sh
```

## Stream Details

### WORKFLOWS Stream

- **Purpose**: Distribute workflow tasks across worker agents
- **Retention**: WorkQueue (messages removed after acknowledgment)
- **Use Case**: Task queues, job distribution, workflow orchestration
- **Consumers**:
  - `workflow-processor`: Pull-based consumer for processing workflow events

### EVENTS Stream

- **Purpose**: Event sourcing and audit logging
- **Retention**: Limits (keep based on size/age/count)
- **Use Case**: Event-driven architecture, system observability, debugging
- **Consumers**:
  - `event-logger`: Push-based consumer for real-time event processing
  - `event-analytics`: Pull-based consumer for batch analytics (metrics only)

### LOGS Stream

- **Purpose**: Centralized log aggregation
- **Retention**: Limits with automatic cleanup of old logs
- **Use Case**: Application logs, system logs, debugging
- **Consumers**:
  - `log-aggregator`: Durable consumer for log collection and forwarding

### COMMANDS Stream

- **Purpose**: Command and control for agents
- **Retention**: WorkQueue with short retention (1 hour)
- **Use Case**: Agent control, remote procedure calls, imperative actions
- **Consumers**:
  - `command-executor`: Exactly-once delivery for idempotent command execution

### TELEMETRY Stream

- **Purpose**: Observability data collection
- **Retention**: Limits optimized for high-volume time-series data
- **Use Case**: Prometheus metrics, OpenTelemetry traces, performance monitoring
- **Consumers**:
  - `metrics-collector`: Pull-based batch consumer for metrics systems

### NOTIFICATIONS Stream

- **Purpose**: User and system notifications
- **Retention**: Interest (keep while consumers are active)
- **Use Case**: Alerts, notifications, ephemeral messages
- **Consumers**:
  - `notification-dispatcher`: Push-based consumer for immediate delivery

## Authentication

NATS uses password-based authentication with the following users:

- **admin**: Administrative access (use `NATS_ADMIN_PASS` environment variable)
- **temporal**: Temporal workflow system (use `NATS_TEMPORAL_PASS`)
- **n8n**: n8n workflow automation (use `NATS_N8N_PASS`)
- **agixt**: AGiXT agent framework (use `NATS_AGIXT_PASS`)

## Monitoring

Access the NATS monitoring endpoint:

```bash
# Server health check
curl http://localhost:8222/healthz

# Server statistics
curl http://localhost:8222/varz

# JetStream information
curl http://localhost:8222/jsz

# Stream information
nats --user admin --password "$NATS_PASSWORD" stream list
nats --user admin --password "$NATS_PASSWORD" stream info WORKFLOWS
```

## Usage Examples

### Publishing Messages

```bash
# Publish a workflow event
nats --user admin --password "$NATS_PASSWORD" \
  pub workflows.execution.start '{"workflow_id": "wf-123", "status": "started"}'

# Publish an event
nats --user admin --password "$NATS_PASSWORD" \
  pub events.metrics.cpu '{"value": 45.2, "timestamp": "2026-01-10T00:00:00Z"}'

# Publish a log
nats --user admin --password "$NATS_PASSWORD" \
  pub logs.app.error '{"level": "error", "message": "Connection failed"}'
```

### Consuming Messages

```bash
# Pull messages from workflow processor
nats --user admin --password "$NATS_PASSWORD" \
  consumer next WORKFLOWS workflow-processor

# Subscribe to events
nats --user admin --password "$NATS_PASSWORD" \
  sub events.>

# View stream contents
nats --user admin --password "$NATS_PASSWORD" \
  stream view WORKFLOWS
```

## Configuration Management

### Updating Stream Configuration

To update stream retention policies or limits:

1. Edit the `jetstream.conf` file with new settings
2. Update the `init-jetstream.sh` script with new parameters
3. Run the script to apply changes:

```bash
NATS_PASSWORD="$NATS_ADMIN_PASS" ./scripts/init-jetstream.sh
```

Note: Some changes may require deleting and recreating streams, which will lose data.

### Adding New Streams

1. Add stream definition to `jetstream.conf`
2. Add creation command to `init-jetstream.sh`
3. Add verification checks to `verify-jetstream.sh`
4. Run initialization script

## Troubleshooting

### Stream Not Found

```bash
# List all streams
nats --user admin --password "$NATS_PASSWORD" stream list

# Recreate streams
NATS_PASSWORD="$NATS_ADMIN_PASS" ./scripts/init-jetstream.sh
```

### Consumer Not Receiving Messages

```bash
# Check consumer status
nats --user admin --password "$NATS_PASSWORD" \
  consumer info WORKFLOWS workflow-processor

# Check for pending messages
nats --user admin --password "$NATS_PASSWORD" \
  stream info WORKFLOWS
```

### Authentication Errors

Ensure environment variables are set correctly:

```bash
# Check NATS configuration
docker exec nats-server cat /etc/nats/nats.conf

# Test authentication
nats --server nats://localhost:4222 \
  --user admin \
  --password "$NATS_ADMIN_PASS" \
  server ping
```

## References

- [NATS Documentation](https://docs.nats.io/)
- [JetStream Guide](https://docs.nats.io/nats-concepts/jetstream)
- [NATS CLI](https://github.com/nats-io/natscli)
- Task: P2-002 - Configure NATS JetStream for Message Persistence

## Related Files

- Docker Compose: `/home/user/ripple-env/docker/docker-compose.messaging.yml`
- Init Script: `/home/user/ripple-env/scripts/init-jetstream.sh`
- Verify Script: `/home/user/ripple-env/scripts/verify-jetstream.sh`
