# ARIA Service Certificates Directory

This directory stores TLS certificates and private keys for ARIA services.

## Directory Structure

Each service has its own subdirectory containing:
- `<service>.crt` - Public certificate
- `<service>.key` - Private key (NEVER commit to git)

```
certs/
├── aria-root-ca.crt          # Root CA certificate (public)
├── keycloak/
│   ├── keycloak.crt
│   └── keycloak.key
├── vault/
│   ├── vault.crt
│   └── vault.key
├── vaultwarden/
│   ├── vaultwarden.crt
│   └── vaultwarden.key
├── temporal/
│   ├── temporal.crt
│   └── temporal.key
├── postgres/
│   ├── postgres.crt
│   └── postgres.key
├── grafana/
│   ├── grafana.crt
│   └── grafana.key
├── prometheus/
│   ├── prometheus.crt
│   └── prometheus.key
└── nats/
    ├── nats.crt
    └── nats.key
```

## Generating Certificates

Use the provided script to generate certificates for all services:

```bash
# Ensure Step-CA is running
docker-compose -f docker/docker-compose.identity.yml up -d step-ca

# Generate certificates
./scripts/generate-service-certs.sh
```

## Certificate Rotation

Certificates should be rotated regularly:

```bash
# Manual rotation
./scripts/rotate-certs.sh

# Setup automated rotation (cron)
./scripts/setup-cert-rotation-cron.sh
```

## Security

### CRITICAL: Private Key Security

- **NEVER** commit `.key` files to version control
- **NEVER** share private keys via email or chat
- **NEVER** store private keys in cloud storage
- Restrict file permissions: `chmod 600 *.key`

### File Permissions

```bash
# Set correct permissions
find . -name "*.key" -exec chmod 600 {} \;
find . -name "*.crt" -exec chmod 644 {} \;
```

### In Production

1. Use Hardware Security Modules (HSM) for CA keys
2. Store service keys in HashiCorp Vault or Kubernetes Secrets
3. Enable audit logging for all key access
4. Implement key rotation policies (90-day certificates)
5. Use automated certificate management (ACME)

## Verification

### Verify Certificate

```bash
# Check certificate details
step certificate inspect keycloak/keycloak.crt

# Verify against root CA
step certificate verify keycloak/keycloak.crt \
  --roots aria-root-ca.crt

# Check expiration
openssl x509 -in keycloak/keycloak.crt -noout -dates
```

### Test mTLS Connection

```bash
# Test HTTPS with client certificate
curl --cacert aria-root-ca.crt \
     --cert keycloak/keycloak.crt \
     --key keycloak/keycloak.key \
     https://keycloak:8443/health
```

## Troubleshooting

### Permission Denied

```bash
# Fix ownership
chown -R $USER:$USER .

# Fix permissions
chmod 755 .
chmod 644 *.crt
chmod 600 *.key
```

### Certificate Expired

```bash
# Check expiration
./scripts/rotate-certs.sh

# Or renew manually
step ca renew keycloak/keycloak.crt keycloak/keycloak.key --force
```

### Certificate Not Trusted

```bash
# Verify root CA
step certificate fingerprint aria-root-ca.crt

# Compare with CA fingerprint
step ca health --ca-url https://localhost:9000 --root aria-root-ca.crt
```

## Backup

### Backup Certificates

```bash
# Create encrypted backup
tar czf certs-backup-$(date +%Y%m%d).tar.gz .
gpg --symmetric certs-backup-$(date +%Y%m%d).tar.gz
rm certs-backup-$(date +%Y%m%d).tar.gz

# Store in secure location (Vault, encrypted S3, etc.)
```

### Restore Certificates

```bash
# Decrypt and extract
gpg --decrypt certs-backup-YYYYMMDD.tar.gz.gpg | tar xz
```

## References

- [ARIA mTLS Setup Guide](../../docs/getting-started/MTLS_SETUP.md)
- [Step-CA Certificate Management](https://smallstep.com/docs/step-ca)
- [Certificate Rotation Script](../../scripts/rotate-certs.sh)
- [Certificate Generation Script](../../scripts/generate-service-certs.sh)
