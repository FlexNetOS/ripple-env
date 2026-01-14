# Script Contract: verify-jetstream.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-jetstream.sh`

---

## Purpose

Verify NATS JetStream streams and consumers (P2-002, Messaging Layer). Checks CLI installation, server connectivity, JetStream enablement, 6 streams configuration, 7 consumers status, and performs message publish/subscribe test.

---

## Invocation

```bash
./verify-jetstream.sh [--nats-url URL] [--user USER] [--password PASS]
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
- `JETSTREAM_REQUIRE=1` - Fail if CLI missing or cluster unavailable (default: 0, warnings only)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (0 failures) |
| `1` | Failed (1+ failures) |

### Counters
- PASS_COUNT (green)
- FAIL_COUNT (red)

---

## Side Effects

**Minimal:** Read-only verification.

**Test Message (Stage 6):** Publishes single test message to `workflows.test` subject (lines 211-235).

---

## Safety Classification

**🟢 SAFE** - Read-only verification, single test message only.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly. Test message is ephemeral.

---

## Verification Stages (6)

### Stage 1: NATS CLI (lines 116-131)
- Checks `nats` command availability
- Shows version
- Early exit if missing and `JETSTREAM_REQUIRE=0` (default)
- Fails if `JETSTREAM_REQUIRE=1`

**Evidence:** Lines 119-130.

### Stage 2: NATS Server Connection (lines 133-148)
- Pings NATS server via `nats server ping`
- Shows server info (first 5 lines)
- Fails if unreachable

**Evidence:** Lines 136-147.

### Stage 3: JetStream Enablement (lines 150-163)
- Checks `nats server info` for JetStream mentions
- Displays JetStream configuration
- Fails if not enabled

**Evidence:** Lines 153-162.

### Stage 4: Verify Streams (lines 165-183)
Checks 6 expected streams:

| Stream | Subjects | Purpose |
|--------|----------|---------|
| WORKFLOWS | workflows.> | Workflow tasks |
| EVENTS | events.> | Event sourcing |
| LOGS | logs.> | Application logs |
| COMMANDS | commands.> | Command processing |
| TELEMETRY | telemetry.> | Metrics/traces |
| NOTIFICATIONS | notifications.> | Pub/sub notifications |

**Evidence:** Lines 27-34 (expected), lines 168-182 (verification).

**Displays:** Message count, byte count, consumer count for each stream.

### Stage 5: Verify Consumers (lines 185-209)
Checks 7 expected consumers:

| Consumer | Stream | Purpose |
|----------|--------|---------|
| workflow-processor | WORKFLOWS | Process workflows |
| event-logger | EVENTS | Log all events |
| event-analytics | EVENTS | Analytics on events.metrics.> |
| log-aggregator | LOGS | Aggregate logs |
| command-executor | COMMANDS | Execute commands |
| metrics-collector | TELEMETRY | Collect telemetry.metrics.> |
| notification-dispatcher | NOTIFICATIONS | Dispatch notifications |

**Evidence:** Lines 37-44 (expected), lines 188-208 (verification).

**Displays:** Ack Policy, Ack Wait, Max Deliver for each consumer.

### Stage 6: Test Message Publish/Subscribe (lines 211-235)
- Publishes test message to `workflows.test` subject
- Message format: `JetStream test message - {timestamp}`
- Verifies message stored in WORKFLOWS stream
- Checks message count > 0

**Evidence:** Lines 214-234.

---

## Expected Configuration

### Streams (lines 27-34)

**Evidence:** Array defines expected configuration
```bash
declare -A EXPECTED_STREAMS=(
    ["WORKFLOWS"]="workflows.>"
    ["EVENTS"]="events.>"
    ["LOGS"]="logs.>"
    ["COMMANDS"]="commands.>"
    ["TELEMETRY"]="telemetry.>"
    ["NOTIFICATIONS"]="notifications.>"
)
```

### Consumers (lines 37-44)

**Evidence:** Maps stream to consumer names
```bash
declare -A EXPECTED_CONSUMERS=(
    ["WORKFLOWS"]="workflow-processor"
    ["EVENTS"]="event-logger event-analytics"
    ["LOGS"]="log-aggregator"
    ["COMMANDS"]="command-executor"
    ["TELEMETRY"]="metrics-collector"
    ["NOTIFICATIONS"]="notification-dispatcher"
)
```

---

## Password Handling

**Evidence:** Lines 73-83

**Behavior:**
- **Password missing + JETSTREAM_REQUIRE=0:** Warning, exit 0 (lines 80-82)
- **Password missing + JETSTREAM_REQUIRE=1:** Fail, exit 1 (lines 74-77)

**Security:** Password passed via command line flag `--password` (not secure for production, consider credential file).

---

## NATS CLI Requirements

**Evidence:** Lines 119-130

**Installation:**
```bash
# macOS
brew install nats-io/nats-tools/nats

# Linux
curl -sf https://binaries.nats.dev/nats-io/natscli/nats@latest | sh

# Or build from source
go install github.com/nats-io/natscli/nats@latest
```

**Reference:** https://github.com/nats-io/natscli#installation

---

## Key Features

### 1. Stream Information Display

**Evidence:** Lines 175-178

**Shows:**
- Messages count
- Bytes stored
- Number of consumers

```bash
local details=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                  stream info "$stream" 2>/dev/null | grep -E "Messages|Bytes|Consumers" | head -n 3)
```

### 2. Consumer Details Display

**Evidence:** Lines 199-203

**Shows:**
- Ack Policy (explicit, none, all)
- Ack Wait (timeout for acknowledgment)
- Max Deliver (redelivery attempts)

```bash
local details=$(nats ... consumer info "$stream" "$consumer" 2>/dev/null | \
                grep -E "Ack Policy|Ack Wait|Max Deliver" | head -n 3)
```

### 3. Message Count Verification

**Evidence:** Lines 227-234

**Extracts message count:**
```bash
local msg_count=$(nats ... stream info WORKFLOWS 2>/dev/null | \
                  grep "Messages:" | awk '{print $2}')
```

**Validates:** Count > 0 indicates test message stored successfully.

### 4. Continue-on-Error Pattern

**Evidence:** Lines 271-276

**All verifications use `|| true`:**
```bash
verify_nats_cli || true
verify_nats_connection || true
verify_jetstream_enabled || true
verify_streams || true
verify_consumers || true
test_message_publish || true
```

**Rationale:** Collects all failures before showing summary (lines 237-260).

---

## Next Steps

**Displayed on success (lines 246-250):**
```bash
# JetStream is properly configured and operational
# Configuration file: config/nats/jetstream.conf
# Initialization script: scripts/init-jetstream.sh
```

**Displayed on failure (lines 254-258):**
```bash
# Run the initialization script to set up JetStream:
NATS_PASSWORD=$NATS_ADMIN_PASS ./scripts/init-jetstream.sh
```

---

## NATS JetStream Context

**P2-002: Messaging Layer**
- **Streams:** Persistent message storage with replay capability
- **Consumers:** Pull or push-based message consumption
- **Subjects:** Hierarchical routing (e.g., `workflows.task.created`)
- **Retention:** Configurable (limits, workqueue, interest-based)

**Integration Points:**
- Argo Workflows for event-driven orchestration
- MindsDB for ML predictions on stream data
- Open-Lovable for real-time UI updates
- AIOS for agent communication

---

## References

### Source Code
- **Main script:** `scripts/verify-jetstream.sh` (284 lines)
- **Expected config:** lines 27-44
- **CLI check:** lines 116-131
- **Connection check:** lines 133-148
- **Stream verification:** lines 165-183
- **Consumer verification:** lines 185-209
- **Message test:** lines 211-235
- **Summary:** lines 237-260

### Related Files
- **Initialization:** `scripts/init-jetstream.sh`
- **Configuration:** `config/nats/jetstream.conf`
- **Compose file:** `docker-compose.messaging.yml` (inferred)

### External Resources
- [NATS JetStream](https://docs.nats.io/nats-concepts/jetstream)
- [NATS CLI](https://github.com/nats-io/natscli)
- [Stream Configuration](https://docs.nats.io/running-a-nats-service/nats_admin/jetstream_admin/streams)
- [Consumer Types](https://docs.nats.io/running-a-nats-service/nats_admin/jetstream_admin/consumers)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 19/60 contracts complete
