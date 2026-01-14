# Script Contract: init-jetstream.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/init-jetstream.sh`

---

## Purpose

Initialize NATS JetStream streams and consumers for the agentic system (P2-002). Creates 6 streams with different retention policies and 7 consumers for message processing, event sourcing, logging, and telemetry.

---

## Invocation

```bash
./init-jetstream.sh [--nats-url URL] [--user USER] [--password PASS]
```

**Options:**
- `--nats-url` - NATS server URL (default: nats://localhost:4222)
- `--user` - NATS username (default: admin)
- `--password` - NATS password (required)
- `--help` - Show help message

**Environment Variables:**
- `NATS_URL` - Server URL (default: nats://localhost:4222)
- `NATS_USER` - Username (default: admin)
- `NATS_PASSWORD` - Password (required, no default)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all streams and consumers created |
| `1` | Failure - NATS CLI missing, server unavailable, or password not provided |

---

## Side Effects

**Creates JetStream streams** (6 total):
1. **WORKFLOWS** - WorkQueue, 7d retention, 10G max, workflow tasks
2. **EVENTS** - Limits, 30d retention, 50G max, event sourcing
3. **LOGS** - Limits, 14d retention, 100G max, application logs
4. **COMMANDS** - WorkQueue, 1h retention, 1G max, command processing
5. **TELEMETRY** - Limits, 7d retention, 200G max, metrics/traces
6. **NOTIFICATIONS** - Interest, 24h retention, 5G max, pub/sub notifications

**Evidence:** lines 199-215

**Creates consumers** (7 total):
- workflow-processor (pull)
- event-logger (push)
- event-analytics (pull)
- log-aggregator (pull)
- command-executor (pull)
- metrics-collector (pull)
- notification-dispatcher (push)

**Evidence:** lines 222-239

---

## Safety Classification

**🟡 CAUTION** - Creates persistent message streams, may overwrite existing configuration.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**
- Streams: Updates if exists (may fail on incompatible changes, line 123)
- Consumers: Skips if exists (lines 157-161)

---

## Stream Configurations

| Stream | Subjects | Retention | Max Age | Max Msgs | Max Bytes | Replicas |
|--------|----------|-----------|---------|----------|-----------|----------|
| WORKFLOWS | workflows.> | workqueue | 7d | 1M | 10G | 1 |
| EVENTS | events.> | limits | 30d | 10M | 50G | 1 |
| LOGS | logs.> | limits | 14d | 50M | 100G | 1 |
| COMMANDS | commands.> | workqueue | 1h | 100K | 1G | 1 |
| TELEMETRY | telemetry.> | limits | 7d | 100M | 200G | 1 |
| NOTIFICATIONS | notifications.> | interest | 24h | 1M | 5G | 1 |

**Evidence:** lines 199-215

---

## Consumer Configurations

| Consumer | Stream | Filter | Ack Wait | Max Deliver | Mode |
|----------|--------|--------|----------|-------------|------|
| workflow-processor | WORKFLOWS | workflows.* | 30s | 3 | pull |
| event-logger | EVENTS | (all) | 20s | 5 | push |
| event-analytics | EVENTS | events.metrics.> | 60s | 2 | pull |
| log-aggregator | LOGS | (all) | 10s | 2 | pull |
| command-executor | COMMANDS | commands.* | 60s | 1 | pull |
| metrics-collector | TELEMETRY | telemetry.metrics.> | 30s | 3 | pull |
| notification-dispatcher | NOTIFICATIONS | (all) | 15s | 5 | push |

**Evidence:** lines 222-239

---

## Key Functions

### wait_for_nats()
**Evidence:** lines 88-104
- Pings NATS server up to 30 attempts (60 seconds)
- Uses nats CLI `server ping` command

### create_stream(name, subjects, retention, max_age, max_msgs, max_bytes, replicas, discard)
**Evidence:** lines 107-143
- Creates or updates stream
- File storage
- Configurable retention (workqueue, limits, interest)

### create_consumer(stream, name, filter, ack_wait, max_deliver, mode)
**Evidence:** lines 146-184
- Creates consumer (skips if exists)
- Supports pull/push modes
- Explicit ACK with configurable wait time

---

## Requirements

**NATS CLI:** Must be installed
- Check: line 79
- Install: https://github.com/nats-io/natscli#installation

**NATS Server:** Must be running and accessible
- Verified with server ping (lines 88-104)

**Password:** Required, no default
- Validation: lines 56-60

---

## References

### Source Code
- **Main script:** `scripts/init-jetstream.sh` (254 lines)
- **Stream creation:** lines 199-215
- **Consumer creation:** lines 222-239
- **Wait logic:** lines 88-104

### Related Files
- **Documentation:** `config/nats/jetstream.conf` (referenced at line 249)

### External Resources
- [NATS JetStream Documentation](https://docs.nats.io/nats-concepts/jetstream)
- [NATS CLI](https://github.com/nats-io/natscli)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 11/60 contracts complete
