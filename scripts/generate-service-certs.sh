#!/usr/bin/env bash
# Generate mTLS certificates for all ARIA services
# This script uses step-ca to issue certificates for inter-service communication

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CERT_DIR="$PROJECT_ROOT/data/certs"
CA_URL="${STEP_CA_URL:-https://localhost:9000}"
ROOT_CA="$PROJECT_ROOT/config/step-ca/pki/root_ca.crt"

echo "🔐 Generating Service Certificates for ARIA"
echo "==========================================="
echo "CA URL: $CA_URL"
echo "Certificates will be saved to: $CERT_DIR"
echo ""

# Check if step-cli is available
if ! command -v step &> /dev/null; then
    echo "❌ Error: step-cli not found"
    echo "   Please run: nix develop"
    exit 1
fi

# Check if CA is running
if ! step ca health --ca-url "$CA_URL" --root "$ROOT_CA" &> /dev/null; then
    echo "❌ Error: Step-CA is not running or not healthy"
    echo "   Please start it with: docker-compose -f docker-compose.identity.yml up -d step-ca"
    exit 1
fi

echo "✓ Step-CA is healthy"
echo ""

# Define services and their DNS names
declare -A SERVICES=(
    ["keycloak"]="keycloak.identity.svc.cluster.local keycloak localhost"
    ["vaultwarden"]="vaultwarden.identity.svc.cluster.local vaultwarden localhost"
    ["vault"]="vault.identity.svc.cluster.local vault localhost"
    ["temporal"]="temporal.workflow.svc.cluster.local temporal localhost"
    ["postgres"]="postgres.state.svc.cluster.local postgres localhost"
    ["grafana"]="grafana.observability.svc.cluster.local grafana localhost"
    ["prometheus"]="prometheus.observability.svc.cluster.local prometheus localhost"
    ["nats"]="nats.messaging.svc.cluster.local nats localhost"
)

# Bootstrap step-cli if not already done
if [ ! -f "$HOME/.step/config/defaults.json" ]; then
    echo "📝 Bootstrapping step-cli..."
    FINGERPRINT=$(step certificate fingerprint "$ROOT_CA")
    step ca bootstrap \
        --ca-url "$CA_URL" \
        --fingerprint "$FINGERPRINT" \
        --install
    echo "   ✓ Bootstrap complete"
    echo ""
fi

# Generate certificates for each service
for SERVICE in "${!SERVICES[@]}"; do
    SERVICE_CERT_DIR="$CERT_DIR/$SERVICE"
    mkdir -p "$SERVICE_CERT_DIR"

    # Parse SANs
    read -ra SANS <<< "${SERVICES[$SERVICE]}"
    PRIMARY_NAME="${SANS[0]}"

    echo "🔑 Generating certificate for: $SERVICE"
    echo "   Primary name: $PRIMARY_NAME"
    echo "   SANs: ${SERVICES[$SERVICE]}"

    # Build SAN arguments
    SAN_ARGS=()
    for SAN in "${SANS[@]}"; do
        SAN_ARGS+=("--san" "$SAN")
    done

    # Request certificate from CA
    step ca certificate "$PRIMARY_NAME" \
        "$SERVICE_CERT_DIR/$SERVICE.crt" \
        "$SERVICE_CERT_DIR/$SERVICE.key" \
        --provisioner acme \
        "${SAN_ARGS[@]}" \
        --force \
        --not-after=8760h

    # Set appropriate permissions
    chmod 600 "$SERVICE_CERT_DIR/$SERVICE.key"
    chmod 644 "$SERVICE_CERT_DIR/$SERVICE.crt"

    echo "   ✓ Certificate saved to: $SERVICE_CERT_DIR/"
    echo ""
done

# Copy root CA to cert directory for convenience
cp "$ROOT_CA" "$CERT_DIR/aria-root-ca.crt"
chmod 644 "$CERT_DIR/aria-root-ca.crt"

echo "✅ All service certificates generated!"
echo ""
echo "📋 Summary:"
echo "   Certificates: $CERT_DIR/"
for SERVICE in "${!SERVICES[@]}"; do
    echo "     - $SERVICE: $CERT_DIR/$SERVICE/$SERVICE.{crt,key}"
done
echo ""
echo "🚀 Next steps:"
echo "   1. Enable mTLS in docker-compose files (uncomment volume mounts)"
echo "   2. Restart services: docker-compose -f docker-compose.identity.yml restart"
echo "   3. Verify mTLS: step certificate verify $CERT_DIR/keycloak/keycloak.crt --roots $ROOT_CA"
echo ""
echo "📖 For more information, see: docs/MTLS_SETUP.md"
echo ""
