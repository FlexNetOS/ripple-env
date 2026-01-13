#!/bin/bash
# =============================================================================
# NATS JetStream Verification Script
# P2-002: Verify NATS JetStream Configuration
# =============================================================================
# This script verifies that JetStream streams and consumers are properly
# configured and operational.
#
# Usage:
#   ./verify-jetstream.sh [--nats-url NATS_URL] [--user USER] [--password PASS]
# =============================================================================

set -e
set -u
set -o pipefail

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
NATS_URL="${NATS_URL:-nats://localhost:4222}"
NATS_USER="${NATS_USER:-admin}"
NATS_PASSWORD="${NATS_PASSWORD:-}"

JETSTREAM_REQUIRE="${JETSTREAM_REQUIRE:-0}"

# Expected streams and their subjects
declare -A EXPECTED_STREAMS=(
    ["WORKFLOWS"]="workflows.>"
    ["EVENTS"]="events.>"
    ["LOGS"]="logs.>"
    ["COMMANDS"]="commands.>"
    ["TELEMETRY"]="telemetry.>"
    ["NOTIFICATIONS"]="notifications.>"
)

# Expected consumers per stream
declare -A EXPECTED_CONSUMERS=(
    ["WORKFLOWS"]="workflow-processor"
    ["EVENTS"]="event-logger event-analytics"
    ["LOGS"]="log-aggregator"
    ["COMMANDS"]="command-executor"
    ["TELEMETRY"]="metrics-collector"
    ["NOTIFICATIONS"]="notification-dispatcher"
)

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --nats-url)
            NATS_URL="$2"
            shift 2
            ;;
        --user)
            NATS_USER="$2"
            shift 2
            ;;
        --password)
            NATS_PASSWORD="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [--nats-url URL] [--user USER] [--password PASS]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate required parameters
if [ -z "$NATS_PASSWORD" ]; then
    if [ "$JETSTREAM_REQUIRE" = "1" ]; then
        echo "Error: NATS_PASSWORD is required"
        echo "Set via environment variable or --password flag"
        exit 1
    fi

    echo "[WARN] NATS_PASSWORD is not set; skipping live JetStream checks."
    echo "       Set JETSTREAM_REQUIRE=1 to require a live NATS/JetStream environment."
    exit 0
fi

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
PASS_COUNT=0
FAIL_COUNT=0

log_info() {
    echo -e "\n[INFO] $*"
}

log_pass() {
    echo "[PASS] $*"
    PASS_COUNT=$((PASS_COUNT + 1))
}

log_fail() {
    echo "[FAIL] $*" >&2
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

log_section() {
    echo ""
    echo "============================================================================="
    echo "$*"
    echo "============================================================================="
}

# -----------------------------------------------------------------------------
# Verification Functions
# -----------------------------------------------------------------------------

verify_nats_cli() {
    log_section "1. Verifying NATS CLI"

    if command -v nats &> /dev/null; then
        local version=$(nats --version 2>&1 | head -n1)
        log_pass "nats CLI is installed: $version"
    else
        if [ "$JETSTREAM_REQUIRE" = "1" ]; then
            log_fail "nats CLI is not installed"
            echo "Install from: https://github.com/nats-io/natscli#installation"
            return 1
        fi
        echo "[WARN] nats CLI not installed; skipping JetStream checks. Set JETSTREAM_REQUIRE=1 to enforce."
        exit 0
    fi
}

verify_nats_connection() {
    log_section "2. Verifying NATS Server Connection"

    if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
           server ping &> /dev/null; then
        log_pass "Successfully connected to NATS server at $NATS_URL"

        # Get server info
        local info=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                     server info 2>/dev/null | head -n 5)
        echo "$info"
    else
        log_fail "Cannot connect to NATS server at $NATS_URL"
        return 1
    fi
}

verify_jetstream_enabled() {
    log_section "3. Verifying JetStream is Enabled"

    local js_info=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                    server info 2>/dev/null | grep -i jetstream || echo "")

    if [ -n "$js_info" ]; then
        log_pass "JetStream is enabled"
        echo "$js_info"
    else
        log_fail "JetStream is not enabled"
        return 1
    fi
}

verify_streams() {
    log_section "4. Verifying JetStream Streams"

    for stream in "${!EXPECTED_STREAMS[@]}"; do
        local subject="${EXPECTED_STREAMS[$stream]}"

        if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
               stream info "$stream" &> /dev/null; then
            log_pass "Stream '$stream' exists with subjects: $subject"

            # Get stream details
            local details=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                          stream info "$stream" 2>/dev/null | grep -E "Messages|Bytes|Consumers" | head -n 3)
            echo "$details" | sed 's/^/       /'
        else
            log_fail "Stream '$stream' does not exist"
        fi
    done
}

verify_consumers() {
    log_section "5. Verifying JetStream Consumers"

    for stream in "${!EXPECTED_CONSUMERS[@]}"; do
        local consumers="${EXPECTED_CONSUMERS[$stream]}"

        echo ""
        echo "Stream: $stream"

        for consumer in $consumers; do
            if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                   consumer info "$stream" "$consumer" &> /dev/null; then
                log_pass "Consumer '$consumer' exists on stream '$stream'"

                # Get consumer details
                local details=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                              consumer info "$stream" "$consumer" 2>/dev/null | \
                              grep -E "Ack Policy|Ack Wait|Max Deliver" | head -n 3)
                echo "$details" | sed 's/^/       /'
            else
                log_fail "Consumer '$consumer' does not exist on stream '$stream'"
            fi
        done
    done
}

test_message_publish() {
    log_section "6. Testing Message Publish/Subscribe"

    local test_subject="workflows.test"
    local test_message="JetStream test message - $(date +%s)"

    # Publish test message
    if echo "$test_message" | nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
         pub "$test_subject" &> /dev/null; then
        log_pass "Successfully published test message to subject '$test_subject'"
    else
        log_fail "Failed to publish test message"
        return 1
    fi

    # Check if message was stored in WORKFLOWS stream
    local msg_count=$(nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
                      stream info WORKFLOWS 2>/dev/null | grep "Messages:" | awk '{print $2}')

    if [ -n "$msg_count" ] && [ "$msg_count" -gt 0 ]; then
        log_pass "WORKFLOWS stream contains $msg_count message(s)"
    else
        log_fail "WORKFLOWS stream is empty or inaccessible"
    fi
}

show_summary() {
    log_section "Verification Summary"

    echo "Total Checks: $((PASS_COUNT + FAIL_COUNT))"
    echo "Passed: $PASS_COUNT"
    echo "Failed: $FAIL_COUNT"

    if [ $FAIL_COUNT -eq 0 ]; then
        echo ""
        echo "✓ All JetStream verifications passed!"
        echo ""
        echo "JetStream is properly configured and operational."
        echo "Configuration file: config/nats/jetstream.conf"
        echo "Initialization script: scripts/init-jetstream.sh"
        return 0
    else
        echo ""
        echo "✗ Some verifications failed."
        echo ""
        echo "Run the initialization script to set up JetStream:"
        echo "  NATS_PASSWORD=\$NATS_ADMIN_PASS ./scripts/init-jetstream.sh"
        return 1
    fi
}

# -----------------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------------
main() {
    echo "NATS JetStream Verification"
    echo "Connecting to: $NATS_URL"
    echo ""

    # Run all verifications
    verify_nats_cli || true
    verify_nats_connection || true
    verify_jetstream_enabled || true
    verify_streams || true
    verify_consumers || true
    test_message_publish || true

    # Show summary
    show_summary
}

# Run main function
main "$@"
