#!/usr/bin/env bash
# Initialize Step-CA for ARIA development
# This script generates the root and intermediate CA certificates

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STEP_CA_DIR="$PROJECT_ROOT/config/step-ca"
PKI_DIR="$STEP_CA_DIR/pki"
DB_DIR="$STEP_CA_DIR/db"
SECRETS_DIR="$STEP_CA_DIR/secrets"

echo "🔐 Initializing Step-CA for ARIA"
echo "================================="
echo "PKI Directory: $PKI_DIR"
echo ""

# Check if step-cli is available
if ! command -v step &> /dev/null; then
    echo "❌ Error: step-cli not found"
    echo "   Please run: nix develop"
    exit 1
fi

# Create directories
mkdir -p "$PKI_DIR" "$DB_DIR" "$SECRETS_DIR"

# Generate a secure random password for the CA
if [ ! -f "$SECRETS_DIR/password.txt" ]; then
    echo "📝 Generating CA password..."
    openssl rand -base64 32 > "$SECRETS_DIR/password.txt"
    chmod 600 "$SECRETS_DIR/password.txt"
    echo "   ✓ Password saved to: $SECRETS_DIR/password.txt"
fi

PASSWORD_FILE="$SECRETS_DIR/password.txt"

# Check if CA is already initialized
if [ -f "$PKI_DIR/root_ca.crt" ]; then
    echo "⚠️  CA already initialized"
    echo "   Root CA: $PKI_DIR/root_ca.crt"
    echo ""
    read -p "Do you want to re-initialize? This will DELETE existing certificates [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi
    echo "🗑️  Removing existing PKI..."
    rm -rf "$PKI_DIR"/* "$DB_DIR"/*
fi

echo "🔑 Generating Root CA..."
step certificate create "ARIA Root CA" \
    "$PKI_DIR/root_ca.crt" "$PKI_DIR/root_ca_key" \
    --profile root-ca \
    --no-password \
    --insecure \
    --not-after=87600h

echo "   ✓ Root CA generated"

echo "🔑 Generating Intermediate CA..."
step certificate create "ARIA Intermediate CA" \
    "$PKI_DIR/intermediate_ca.crt" "$PKI_DIR/intermediate_ca_key" \
    --profile intermediate-ca \
    --ca "$PKI_DIR/root_ca.crt" \
    --ca-key "$PKI_DIR/root_ca_key" \
    --password-file "$PASSWORD_FILE" \
    --not-after=43800h

echo "   ✓ Intermediate CA generated"

echo "🔑 Generating JWK Provisioner..."
step crypto jwk create \
    "$SECRETS_DIR/provisioner.pub.json" \
    "$SECRETS_DIR/provisioner.key.json" \
    --kty EC \
    --crv P-256 \
    --password-file "$PASSWORD_FILE"

echo "   ✓ JWK Provisioner generated"

# Extract values for ca.json
echo "📝 Extracting provisioner values..."
PROV_X=$(jq -r .x "$SECRETS_DIR/provisioner.pub.json")
PROV_Y=$(jq -r .y "$SECRETS_DIR/provisioner.pub.json")
PROV_KEY=$(jq -r .key "$SECRETS_DIR/provisioner.key.json")

# Update ca.json with real values
echo "📝 Updating ca.json..."
CA_CONFIG="$STEP_CA_DIR/ca.json"
jq \
    --arg x "$PROV_X" \
    --arg y "$PROV_Y" \
    --arg key "$PROV_KEY" \
    '.authority.provisioners[0].key.x = $x |
     .authority.provisioners[0].key.y = $y |
     .authority.provisioners[0].encryptedKey = $key' \
    "$CA_CONFIG" > "$CA_CONFIG.tmp" && mv "$CA_CONFIG.tmp" "$CA_CONFIG"

echo "   ✓ ca.json updated"

# Calculate and update fingerprint in defaults.json
echo "📝 Updating defaults.json with root CA fingerprint..."
FINGERPRINT=$(step certificate fingerprint "$PKI_DIR/root_ca.crt")
DEFAULTS_CONFIG="$STEP_CA_DIR/defaults.json"
jq --arg fp "$FINGERPRINT" '.fingerprint = $fp' "$DEFAULTS_CONFIG" > "$DEFAULTS_CONFIG.tmp" && \
    mv "$DEFAULTS_CONFIG.tmp" "$DEFAULTS_CONFIG"

echo "   ✓ defaults.json updated"

# Set appropriate permissions
chmod 600 "$PKI_DIR"/*_key "$SECRETS_DIR"/*.json
chmod 644 "$PKI_DIR"/*.crt

echo ""
echo "✅ Step-CA initialization complete!"
echo ""
echo "📋 Summary:"
echo "   Root CA:          $PKI_DIR/root_ca.crt"
echo "   Intermediate CA:  $PKI_DIR/intermediate_ca.crt"
echo "   Fingerprint:      $FINGERPRINT"
echo "   Password file:    $PASSWORD_FILE"
echo ""
echo "🚀 Next steps:"
echo "   1. Start the CA: docker-compose -f docker-compose.identity.yml up step-ca"
echo "   2. Test health:   step ca health --ca-url https://localhost:9000 --root $PKI_DIR/root_ca.crt"
echo "   3. Bootstrap:     step ca bootstrap --ca-url https://localhost:9000 --fingerprint $FINGERPRINT"
echo ""
echo "⚠️  IMPORTANT: Keep $PASSWORD_FILE secure!"
echo "   In production, store it in HashiCorp Vault or a Kubernetes Secret"
echo ""
