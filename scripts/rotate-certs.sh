#!/usr/bin/env bash
# Automated certificate rotation for ARIA services
# This script renews certificates before expiration and reloads services

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CERT_DIR="$PROJECT_ROOT/data/certs"
CA_URL="${STEP_CA_URL:-https://localhost:9000}"
ROOT_CA="$PROJECT_ROOT/config/step-ca/pki/root_ca.crt"
LOG_FILE="${LOG_FILE:-$PROJECT_ROOT/logs/cert-rotation.log}"

# Create log directory if it doesn't exist
mkdir -p "$(dirname "$LOG_FILE")"

# Logging function
log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

# Error handler
error() {
    log "❌ ERROR: $*"
    exit 1
}

log "🔄 Starting Certificate Rotation for ARIA"
log "=========================================="
log "CA URL: $CA_URL"
log "Certificate Directory: $CERT_DIR"
log ""

# Check if step-cli is available
if ! command -v step &> /dev/null; then
    error "step-cli not found. Please run: nix develop"
fi

# Check if CA is running
if ! step ca health --ca-url "$CA_URL" --root "$ROOT_CA" &> /dev/null; then
    error "Step-CA is not running or not healthy. Start with: docker-compose -f docker-compose.identity.yml up -d step-ca"
fi

log "✓ Step-CA is healthy"
log ""

# Define services and their docker-compose files
declare -A SERVICES=(
    ["keycloak"]="docker/docker-compose.identity.yml"
    ["vaultwarden"]="docker/docker-compose.identity.yml"
    ["vault"]="docker/docker-compose.identity.yml"
    ["temporal"]="docker/docker-compose.temporal.yml"
    ["postgres"]="docker/docker-compose.state.yml"
    ["grafana"]="docker/docker-compose.observability.yml"
    ["prometheus"]="docker/docker-compose.observability.yml"
    ["nats"]="docker/docker-compose.messaging.yml"
)

# Renewal threshold (renew if cert expires within this many days)
RENEWAL_THRESHOLD_DAYS="${RENEWAL_THRESHOLD_DAYS:-30}"

# Function to check if certificate needs renewal
needs_renewal() {
    local cert_file=$1
    local threshold_days=$2

    if [ ! -f "$cert_file" ]; then
        log "   Certificate not found: $cert_file"
        return 0  # Needs renewal (creation)
    fi

    # Get certificate expiration date
    local expiry_date
    expiry_date=$(step certificate inspect "$cert_file" --format json | jq -r '.validity.end')

    # Convert to epoch seconds
    local expiry_epoch
    expiry_epoch=$(date -d "$expiry_date" +%s 2>/dev/null || date -j -f "%Y-%m-%dT%H:%M:%SZ" "$expiry_date" +%s 2>/dev/null)

    local current_epoch
    current_epoch=$(date +%s)

    local threshold_epoch
    threshold_epoch=$((current_epoch + (threshold_days * 86400)))

    if [ "$expiry_epoch" -le "$threshold_epoch" ]; then
        local days_until_expiry=$(( (expiry_epoch - current_epoch) / 86400 ))
        log "   Certificate expires in $days_until_expiry days (threshold: $threshold_days days)"
        return 0  # Needs renewal
    else
        local days_until_expiry=$(( (expiry_epoch - current_epoch) / 86400 ))
        log "   Certificate valid for $days_until_expiry days - no renewal needed"
        return 1  # Does not need renewal
    fi
}

# Function to renew a certificate
renew_certificate() {
    local service=$1
    local cert_file=$2
    local key_file=$3

    log "🔑 Renewing certificate for: $service"

    # Create backup of existing certificate
    if [ -f "$cert_file" ]; then
        local backup_file
        backup_file="${cert_file}.backup.$(date +%Y%m%d_%H%M%S)"
        cp "$cert_file" "$backup_file"
        log "   Backup created: $backup_file"
    fi

    # Renew the certificate
    if step ca renew "$cert_file" "$key_file" --force &>> "$LOG_FILE"; then
        log "   ✓ Certificate renewed successfully"

        # Set appropriate permissions
        chmod 600 "$key_file"
        chmod 644 "$cert_file"

        return 0
    else
        log "   ❌ Failed to renew certificate"

        # Restore from backup if renewal failed
        if [ -f "$backup_file" ]; then
            cp "$backup_file" "$cert_file"
            log "   Restored from backup"
        fi

        return 1
    fi
}

# Function to reload service after certificate renewal
reload_service() {
    local service=$1
    local compose_file=$2

    log "🔄 Reloading service: $service"

    # Check if docker-compose file exists
    if [ ! -f "$PROJECT_ROOT/$compose_file" ]; then
        log "   ⚠️  Docker compose file not found: $compose_file"
        return 1
    fi

    # Check if service is running
    if ! docker-compose -f "$PROJECT_ROOT/$compose_file" ps "$service" | grep -q "Up"; then
        log "   ⚠️  Service not running: $service"
        return 0
    fi

    # Gracefully restart the service
    if docker-compose -f "$PROJECT_ROOT/$compose_file" restart "$service" &>> "$LOG_FILE"; then
        log "   ✓ Service reloaded successfully"

        # Wait for service to be healthy
        local max_wait=60
        local waited=0
        while [ $waited -lt $max_wait ]; do
            if docker-compose -f "$PROJECT_ROOT/$compose_file" ps "$service" | grep -q "healthy\|Up"; then
                log "   ✓ Service is healthy"
                return 0
            fi
            sleep 2
            waited=$((waited + 2))
        done

        log "   ⚠️  Service started but health check timed out"
        return 1
    else
        log "   ❌ Failed to reload service"
        return 1
    fi
}

# Track renewal statistics
RENEWED_COUNT=0
FAILED_COUNT=0
SKIPPED_COUNT=0

# Process each service
for SERVICE in "${!SERVICES[@]}"; do
    SERVICE_CERT_DIR="$CERT_DIR/$SERVICE"
    CERT_FILE="$SERVICE_CERT_DIR/$SERVICE.crt"
    KEY_FILE="$SERVICE_CERT_DIR/$SERVICE.key"
    COMPOSE_FILE="${SERVICES[$SERVICE]}"

    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    log "Processing: $SERVICE"
    log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    # Check if certificate needs renewal
    if needs_renewal "$CERT_FILE" "$RENEWAL_THRESHOLD_DAYS"; then
        # Attempt to renew
        if renew_certificate "$SERVICE" "$CERT_FILE" "$KEY_FILE"; then
            RENEWED_COUNT=$((RENEWED_COUNT + 1))

            # Reload the service with new certificate
            reload_service "$SERVICE" "$COMPOSE_FILE" || log "   ⚠️  Service reload had issues but continuing"
        else
            FAILED_COUNT=$((FAILED_COUNT + 1))
            log "   ⚠️  Continuing to next service"
        fi
    else
        SKIPPED_COUNT=$((SKIPPED_COUNT + 1))
    fi

    log ""
done

# Print summary
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "📊 Certificate Rotation Summary"
log "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "✅ Renewed:  $RENEWED_COUNT"
log "⏭️  Skipped:  $SKIPPED_COUNT (still valid)"
log "❌ Failed:   $FAILED_COUNT"
log ""

if [ "$FAILED_COUNT" -gt 0 ]; then
    log "⚠️  Some certificates failed to renew. Check logs for details."
    exit 1
fi

log "✅ Certificate rotation complete!"
log ""
log "📝 Next steps:"
log "   1. Verify services are running: docker-compose ps"
log "   2. Check service health: docker-compose -f docker-compose.identity.yml ps"
log "   3. Review logs: tail -f $LOG_FILE"
log ""

# If running in cron mode, send notification (optional)
if [ "${NOTIFY_ON_COMPLETION:-false}" = "true" ]; then
    log "📧 Sending notification..."
    # Add notification logic here (email, Slack, etc.)
    # Example: curl -X POST https://hooks.slack.com/... -d "Cert rotation complete: $RENEWED_COUNT renewed"
fi

exit 0
