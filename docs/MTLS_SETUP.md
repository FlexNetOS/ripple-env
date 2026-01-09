# mTLS Setup for ARIA Inter-Service Communication

This document describes the mutual TLS (mTLS) setup for secure service-to-service communication in the ARIA platform using smallstep/step-ca.

## Overview

ARIA uses **mutual TLS (mTLS)** to secure communication between services. This provides:

- **Authentication**: Both client and server verify each other's identity
- **Encryption**: All traffic is encrypted in transit
- **Authorization**: Services can implement fine-grained access control based on certificate properties
- **Zero Trust**: No implicit trust - every connection is verified

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Step-CA                              │
│  (Certificate Authority - issues & manages certificates)    │
│                     Port: 9000                              │
└──────────────────┬──────────────────────────────────────────┘
                   │ Issues certificates
                   ├─────────────┬─────────────┬──────────────┐
                   ▼             ▼             ▼              ▼
            ┌─────────┐   ┌───────────┐  ┌─────────┐  ┌──────────┐
            │Keycloak │   │Vaultwarden│  │Temporal │  │Postgres  │
            │ (OIDC)  │   │  (Vault)  │  │(Workflow)│  │  (DB)    │
            └─────────┘   └───────────┘  └─────────┘  └──────────┘
                   ▲             ▲             ▲              ▲
                   └─────────────┴─────────────┴──────────────┘
                          mTLS connections with
                          client & server certificates
```

## Components

### 1. Step-CA (Certificate Authority)

- **Purpose**: Issues and manages X.509 certificates
- **Configuration**: `/home/user/ros2-humble-env/config/step-ca/ca.json`
- **Provisioners**:
  - **JWK**: For manual certificate requests (admin use)
  - **ACME**: Automated certificate management (Let's Encrypt protocol)
  - **SSHPOP**: SSH certificate authentication

### 2. Step-CLI

- **Purpose**: Client tool for interacting with Step-CA
- **Installation**: Already included in `flake.nix` (line 209)
- **Usage**: Certificate generation, inspection, and management

## Quick Start

### 1. Initialize the CA

```bash
# Generate root and intermediate CA certificates
./scripts/init-step-ca.sh
```

This creates:
- Root CA certificate (valid for 10 years)
- Intermediate CA certificate (valid for 5 years)
- JWK provisioner for admin access
- ACME provisioner for automated cert management

### 2. Start the Certificate Authority

```bash
# Start step-ca via docker-compose
docker-compose -f docker-compose.identity.yml up -d step-ca

# Verify it's running
docker-compose -f docker-compose.identity.yml ps step-ca

# Check CA health
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt
```

### 3. Bootstrap a Client

```bash
# Bootstrap step-cli to trust your CA
step ca bootstrap \
  --ca-url https://localhost:9000 \
  --fingerprint $(step certificate fingerprint config/step-ca/pki/root_ca.crt)

# This creates ~/.step/ with CA configuration
```

### 4. Request a Certificate

```bash
# Option A: Using ACME (automated)
step ca certificate keycloak.identity.svc.cluster.local \
  keycloak.crt keycloak.key \
  --provisioner acme

# Option B: Using JWK (manual, requires password)
step ca certificate keycloak.identity.svc.cluster.local \
  keycloak.crt keycloak.key \
  --provisioner admin
```

## Service Integration

### Docker Compose Configuration

Services need:
1. Volume mounts for certificates
2. Environment variables for certificate paths
3. Network configuration for step-ca access

Example (Keycloak):

```yaml
services:
  keycloak:
    image: quay.io/keycloak/keycloak:23.0
    volumes:
      # Mount CA root certificate
      - ./config/step-ca/pki/root_ca.crt:/etc/ssl/certs/aria-root-ca.crt:ro
      # Mount service certificate and key
      - ./data/certs/keycloak:/etc/certs:ro
    environment:
      # Enable HTTPS with mTLS
      KC_HTTPS_CERTIFICATE_FILE: /etc/certs/keycloak.crt
      KC_HTTPS_CERTIFICATE_KEY_FILE: /etc/certs/keycloak.key
      KC_HTTPS_CLIENT_AUTH: request
      KC_HTTPS_TRUST_STORE_FILE: /etc/ssl/certs/aria-root-ca.crt
    networks:
      - identity
```

### Certificate Renewal

Step-CA supports automatic certificate renewal:

```bash
# Renew a certificate before expiration
step ca renew keycloak.crt keycloak.key \
  --force \
  --out keycloak-renewed.crt

# Or use step-ca's built-in renewal service
step ca renew --daemon keycloak.crt keycloak.key
```

### Automated Renewal with step-agent

For production, use `step-agent` to automatically renew certificates:

```yaml
services:
  keycloak:
    # ... (service config)

  keycloak-cert-renewer:
    image: smallstep/step-cli:latest
    command: >
      step ca renew --daemon
      /etc/certs/keycloak.crt
      /etc/certs/keycloak.key
    volumes:
      - ./data/certs/keycloak:/etc/certs
      - ./config/step-ca/pki/root_ca.crt:/etc/ssl/certs/aria-root-ca.crt:ro
    networks:
      - identity
    depends_on:
      - step-ca
```

## Certificate Management

### Inspect a Certificate

```bash
# View certificate details
step certificate inspect keycloak.crt

# Check expiration
step certificate inspect --format json keycloak.crt | jq -r .validity.end

# Verify certificate chain
step certificate verify keycloak.crt \
  --roots config/step-ca/pki/root_ca.crt
```

### Revoke a Certificate

```bash
# Revoke a certificate (requires provisioner)
step ca revoke --cert keycloak.crt --key keycloak.key
```

### List Active Certificates

Step-CA stores certificates in its database:

```bash
# Check CA database (BadgerDB)
ls -lh config/step-ca/db/

# For production with PostgreSQL:
# SELECT * FROM certificates WHERE status = 'valid';
```

## Development Workflow

### 1. Generate Certificates for All Services

Create a script to generate certificates for all ARIA services:

```bash
#!/usr/bin/env bash
# scripts/generate-service-certs.sh

set -euo pipefail

SERVICES=(
  "keycloak.identity.svc.cluster.local"
  "vaultwarden.identity.svc.cluster.local"
  "vault.identity.svc.cluster.local"
  "temporal.workflow.svc.cluster.local"
  "postgres.state.svc.cluster.local"
  "grafana.observability.svc.cluster.local"
  "prometheus.observability.svc.cluster.local"
)

CERT_DIR="data/certs"
mkdir -p "$CERT_DIR"

for SERVICE in "${SERVICES[@]}"; do
  SERVICE_NAME="${SERVICE%%.*}"
  SERVICE_CERT_DIR="$CERT_DIR/$SERVICE_NAME"
  mkdir -p "$SERVICE_CERT_DIR"

  echo "Generating certificate for $SERVICE..."
  step ca certificate "$SERVICE" \
    "$SERVICE_CERT_DIR/$SERVICE_NAME.crt" \
    "$SERVICE_CERT_DIR/$SERVICE_NAME.key" \
    --provisioner acme \
    --san "$SERVICE_NAME" \
    --san "localhost"

  echo "  ✓ $SERVICE_CERT_DIR/$SERVICE_NAME.crt"
done

echo "All service certificates generated!"
```

### 2. Update docker-compose Files

Ensure each service mounts:
- Root CA certificate
- Service certificate
- Service private key

### 3. Test mTLS Connection

```bash
# Test with curl
curl --cacert config/step-ca/pki/root_ca.crt \
     --cert data/certs/client/client.crt \
     --key data/certs/client/client.key \
     https://keycloak:8443/health

# Test with step
step certificate verify keycloak.crt --roots config/step-ca/pki/root_ca.crt
```

## Security Best Practices

### 1. Key Management

- **Never commit private keys** to git (`.gitignore` is configured)
- **Encrypt intermediate CA key** with a strong password
- **Rotate keys regularly** (at least annually)
- **Use HSM** for root CA key in production

### 2. Certificate Lifecycle

- **Short-lived certificates**: Default 1 year, consider 90 days for production
- **Automated renewal**: Use ACME or step-agent
- **Monitor expiration**: Alert 30 days before expiry
- **Revocation**: Have a process to revoke compromised certificates

### 3. Network Security

- **Isolate step-ca**: Run on internal network only
- **Rate limiting**: Prevent DoS on certificate issuance
- **Audit logging**: Log all certificate requests and revocations
- **Access control**: Restrict provisioner access

### 4. Production Considerations

```yaml
# Use PostgreSQL instead of BadgerDB for HA
db:
  type: postgresql
  dataSource: "postgresql://stepca:password@postgres:5432/stepca?sslmode=verify-full"

# Enable ACME with DNS-01 challenge for wildcard certs
provisioners:
  - type: ACME
    name: acme-dns
    challenges:
      - dns-01
    options:
      x509:
        templateFile: /etc/step-ca/templates/x509/leaf.tpl
```

## Troubleshooting

### CA Not Starting

```bash
# Check logs
docker-compose -f docker-compose.identity.yml logs step-ca

# Common issues:
# 1. Port 9000 already in use
# 2. Password file not found
# 3. Certificate files missing
```

### Certificate Validation Fails

```bash
# Verify CA trust chain
step certificate verify keycloak.crt \
  --roots config/step-ca/pki/root_ca.crt \
  --verbose

# Check certificate expiration
step certificate inspect keycloak.crt --format json | jq .validity

# Verify SAN (Subject Alternative Name)
step certificate inspect keycloak.crt | grep DNS
```

### Connection Refused

```bash
# Check if step-ca is listening
netstat -tulpn | grep 9000

# Test with health check
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt

# Check firewall rules
iptables -L -n | grep 9000
```

### Certificate Not Trusted

```bash
# Ensure root CA is in system trust store
# Linux:
sudo cp config/step-ca/pki/root_ca.crt /usr/local/share/ca-certificates/aria-root-ca.crt
sudo update-ca-certificates

# macOS:
sudo security add-trusted-cert -d -r trustRoot \
  -k /Library/Keychains/System.keychain \
  config/step-ca/pki/root_ca.crt
```

## Advanced Usage

### Custom Certificate Templates

Create custom certificate templates for specific use cases:

```bash
# templates/x509/leaf.tpl
{
  "subject": {
    "commonName": "{{ .Subject.CommonName }}",
    "organization": "ARIA Platform"
  },
  "sans": {{ toJson .SANs }},
  "keyUsage": ["keyEncipherment", "digitalSignature"],
  "extKeyUsage": ["serverAuth", "clientAuth"],
  "extensions": [
    {
      "id": "1.3.6.1.4.1.99999.1",
      "critical": false,
      "value": "{{ .Token.environment }}"
    }
  ]
}
```

### Webhook Validation

Configure step-ca to validate certificate requests via webhook:

```json
{
  "authority": {
    "provisioners": [
      {
        "type": "ACME",
        "name": "acme-webhook",
        "options": {
          "webhook": {
            "url": "https://webhook.example.com/validate",
            "certType": "X509"
          }
        }
      }
    ]
  }
}
```

### Certificate Monitoring

Monitor certificate expiration with Prometheus:

```yaml
# prometheus.yml
scrape_configs:
  - job_name: 'cert-exporter'
    static_configs:
      - targets: ['cert-exporter:9117']
    relabel_configs:
      - source_labels: [__address__]
        target_label: __param_target
      - source_labels: [__param_target]
        target_label: instance
```

## References

- [smallstep Documentation](https://smallstep.com/docs/)
- [Step-CA Configuration](https://smallstep.com/docs/step-ca/configuration)
- [Step-CA ACME](https://smallstep.com/docs/step-ca/acme-basics)
- [mTLS Best Practices](https://smallstep.com/hello-mtls)
- [ARIA Security Audit](../SECURITY-AUDIT-REPORT.md)

## Support

For issues or questions:
1. Check the [troubleshooting section](#troubleshooting)
2. Review step-ca logs: `docker-compose logs step-ca`
3. Consult the [smallstep community](https://github.com/smallstep/certificates/discussions)
