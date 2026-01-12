# mTLS Implementation Checklist for ARIA Platform

This checklist guides you through enabling mutual TLS (mTLS) on all ARIA services as part of task P2-005.

## Prerequisites

- [ ] Step-CA initialized (`./scripts/init-step-ca.sh`)
- [ ] Docker and docker-compose installed
- [ ] Step-CLI available in nix development environment
- [ ] All services defined in docker-compose files

## Phase 1: Certificate Authority Setup

### 1.1 Initialize Step-CA

```bash
# Initialize the CA (one-time setup)
./scripts/init-step-ca.sh

# Expected output:
# - Root CA certificate created
# - Intermediate CA certificate created
# - JWK provisioner configured
# - Password file generated
```

**Verification:**
- [ ] File exists: `/home/user/ripple-env/config/step-ca/pki/root_ca.crt`
- [ ] File exists: `/home/user/ripple-env/config/step-ca/pki/intermediate_ca.crt`
- [ ] File exists: `/home/user/ripple-env/config/step-ca/secrets/password.txt`

### 1.2 Start Step-CA

```bash
# Start the certificate authority
docker-compose -f docker/docker-compose.identity.yml up -d step-ca

# Wait for healthy status
docker-compose -f docker/docker-compose.identity.yml ps step-ca

# Test CA health
step ca health \
  --ca-url https://localhost:9000 \
  --root config/step-ca/pki/root_ca.crt
```

**Verification:**
- [ ] Step-CA container is running
- [ ] Health check returns: "step-ca is running"
- [ ] Port 9000 is accessible

## Phase 2: Service Certificate Generation

### 2.1 Generate Certificates

```bash
# Generate certificates for all services
./scripts/generate-service-certs.sh

# Expected: Certificates for 8 services
# - keycloak
# - vaultwarden
# - vault
# - temporal
# - postgres
# - grafana
# - prometheus
# - nats
```

**Verification:**
- [ ] Certificates created in `/home/user/ripple-env/data/certs/<service>/`
- [ ] Each service has `.crt` and `.key` files
- [ ] Root CA copied to `/home/user/ripple-env/data/certs/aria-root-ca.crt`
- [ ] Key files have 600 permissions
- [ ] Certificate files have 644 permissions

### 2.2 Verify Certificates

```bash
# Inspect a certificate
step certificate inspect data/certs/keycloak/keycloak.crt

# Verify certificate chain
step certificate verify \
  data/certs/keycloak/keycloak.crt \
  --roots config/step-ca/pki/root_ca.crt

# Check all certificates
for service in keycloak vault vaultwarden temporal postgres grafana prometheus nats; do
  echo "Checking $service..."
  step certificate verify \
    "data/certs/$service/$service.crt" \
    --roots config/step-ca/pki/root_ca.crt
done
```

**Verification:**
- [ ] All certificates verify successfully
- [ ] Validity period is 8760h (1 year)
- [ ] Subject organization is "ARIA Platform"
- [ ] SANs include service name and localhost

## Phase 3: Service Configuration

### 3.1 Keycloak mTLS Configuration

Edit `docker/docker-compose.identity.yml`:

```yaml
keycloak:
  environment:
    # Uncomment these lines:
    KC_HTTPS_CERTIFICATE_FILE: /etc/certs/keycloak.crt
    KC_HTTPS_CERTIFICATE_KEY_FILE: /etc/certs/keycloak.key
    KC_HTTPS_CLIENT_AUTH: request
    KC_HTTPS_TRUST_STORE_FILE: /etc/ssl/certs/aria-root-ca.crt
  volumes:
    # Uncomment these lines:
    - ./config/step-ca/pki/root_ca.crt:/etc/ssl/certs/aria-root-ca.crt:ro
    - ./data/certs/keycloak:/etc/certs:ro
```

**Verification:**
- [ ] Configuration uncommented
- [ ] Certificates mounted correctly
- [ ] Service restarts without errors

### 3.2 Vault mTLS Configuration

Edit `docker/docker-compose.identity.yml`:

```yaml
vault:
  volumes:
    # Uncomment these lines:
    - ./config/step-ca/pki/root_ca.crt:/etc/ssl/certs/aria-root-ca.crt:ro
    - ./data/certs/vault:/vault/certs:ro
```

Create `config/vault/vault.hcl`:

```hcl
listener "tcp" {
  address       = "0.0.0.0:8200"
  tls_cert_file = "/vault/certs/vault.crt"
  tls_key_file  = "/vault/certs/vault.key"
  tls_client_ca_file = "/etc/ssl/certs/aria-root-ca.crt"
  tls_require_and_verify_client_cert = true
}
```

**Verification:**
- [ ] Vault configuration updated
- [ ] Certificates mounted
- [ ] Vault restarts on HTTPS port

### 3.3 Vaultwarden mTLS Configuration

Edit `docker/docker-compose.identity.yml`:

```yaml
vaultwarden:
  environment:
    # Uncomment:
    ROCKET_TLS: '{certs="/etc/certs/vaultwarden.crt",key="/etc/certs/vaultwarden.key"}'
  volumes:
    # Uncomment:
    - ./config/step-ca/pki/root_ca.crt:/etc/ssl/certs/aria-root-ca.crt:ro
    - ./data/certs/vaultwarden:/etc/certs:ro
```

**Verification:**
- [ ] Configuration updated
- [ ] Service accessible via HTTPS

### 3.4 Additional Services

Repeat similar configuration for:

- [ ] Temporal (workflow)
- [ ] PostgreSQL (state)
- [ ] Grafana (observability)
- [ ] Prometheus (observability)
- [ ] NATS (messaging)

## Phase 4: Testing

### 4.1 Service Health Checks

```bash
# Test Keycloak HTTPS
curl --cacert data/certs/aria-root-ca.crt \
     https://localhost:8443/health

# Test Vault HTTPS
curl --cacert data/certs/aria-root-ca.crt \
     https://localhost:8200/v1/sys/health

# Test with client certificate (mTLS)
curl --cacert data/certs/aria-root-ca.crt \
     --cert data/certs/keycloak/keycloak.crt \
     --key data/certs/keycloak/keycloak.key \
     https://keycloak:8443/health
```

**Verification:**
- [ ] All services respond to HTTPS requests
- [ ] mTLS connections succeed
- [ ] HTTP connections are rejected (if HTTP disabled)

### 4.2 Certificate Validation

```bash
# Run verification script
./scripts/verify-mtls-setup.sh

# Expected output:
# - All services have valid certificates
# - All certificates verify against root CA
# - mTLS connections succeed
```

**Verification:**
- [ ] Verification script passes all checks
- [ ] No expired certificates
- [ ] No certificate errors in logs

## Phase 5: Certificate Rotation Setup

### 5.1 Manual Rotation Test

```bash
# Test certificate rotation
./scripts/rotate-certs.sh

# Review logs
tail -f logs/cert-rotation.log
```

**Verification:**
- [ ] Rotation script completes successfully
- [ ] Services reload with new certificates
- [ ] No downtime during rotation

### 5.2 Automated Rotation

```bash
# Setup cron job for daily rotation checks
./scripts/setup-cert-rotation-cron.sh

# Verify cron job
crontab -l | grep rotate-certs
```

**Verification:**
- [ ] Cron job installed (runs daily at 2 AM)
- [ ] Logs directory exists: `/home/user/ripple-env/logs/`
- [ ] Rotation threshold set to 30 days

### 5.3 Alternative: Systemd Timer

```bash
# Install systemd timer
sudo cp /tmp/cert-rotation.service /etc/systemd/system/
sudo cp /tmp/cert-rotation.timer /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable cert-rotation.timer
sudo systemctl start cert-rotation.timer

# Verify timer
sudo systemctl status cert-rotation.timer
```

**Verification:**
- [ ] Timer installed and enabled
- [ ] Timer scheduled correctly
- [ ] Test run completes successfully

## Phase 6: Monitoring and Alerts

### 6.1 Certificate Expiration Monitoring

Add to `config/prometheus/prometheus.yml`:

```yaml
scrape_configs:
  - job_name: 'cert-exporter'
    static_configs:
      - targets: ['localhost:9117']
```

**Verification:**
- [ ] Certificate metrics exported
- [ ] Prometheus scraping cert-exporter
- [ ] Alerts configured for expiration

### 6.2 Audit Logging

Enable audit logging in `config/step-ca/ca-config.json`:

```json
{
  "security": {
    "auditLog": {
      "enabled": true,
      "logPath": "/var/log/step-ca/audit.log"
    }
  }
}
```

**Verification:**
- [ ] Audit log enabled
- [ ] Certificate issuance logged
- [ ] Revocations logged

## Phase 7: Documentation

### 7.1 Update Project Documentation

- [ ] README.md updated with mTLS information
- [ ] MTLS_SETUP.md reviewed and accurate
- [ ] Service-specific documentation updated

### 7.2 Create Runbooks

- [ ] Certificate rotation runbook
- [ ] Certificate revocation procedure
- [ ] Emergency certificate replacement process
- [ ] Disaster recovery plan

## Phase 8: Security Hardening

### 8.1 Access Control

```bash
# Restrict certificate directory permissions
chmod 700 data/certs/
chmod 600 data/certs/*/*.key
chmod 644 data/certs/*/*.crt

# Restrict CA secrets
chmod 600 config/step-ca/secrets/*
```

**Verification:**
- [ ] Only root/owner can access private keys
- [ ] CA password file protected
- [ ] No world-readable secrets

### 8.2 Backup and Recovery

```bash
# Create encrypted backup
tar czf certs-backup-$(date +%Y%m%d).tar.gz \
  config/step-ca/pki \
  config/step-ca/secrets \
  data/certs

gpg --symmetric certs-backup-$(date +%Y%m%d).tar.gz

# Store in secure location
# - HashiCorp Vault
# - Encrypted S3 bucket
# - Offline storage
```

**Verification:**
- [ ] Backup procedure documented
- [ ] Backup tested (can restore)
- [ ] Backup stored securely off-site
- [ ] Recovery tested

### 8.3 Production Checklist

- [ ] Replace BadgerDB with PostgreSQL for HA
- [ ] Store CA password in Vault
- [ ] Enable ACME DNS-01 for wildcard certificates
- [ ] Implement certificate revocation list (CRL)
- [ ] Enable OCSP for real-time validation
- [ ] Use HSM for root CA key storage
- [ ] Set up certificate transparency logging
- [ ] Configure rate limiting on CA
- [ ] Implement webhook validation
- [ ] Set up geo-redundant CA instances

## Phase 9: Validation

### 9.1 Final Checks

```bash
# Run comprehensive verification
./scripts/verify-mtls-setup.sh

# Check all service logs
docker-compose -f docker/docker-compose.identity.yml logs | grep -i "tls\|cert\|ssl"

# Test service-to-service mTLS
# (Implementation specific to your architecture)
```

**Verification:**
- [ ] All services using mTLS
- [ ] No certificate errors in logs
- [ ] Inter-service communication working
- [ ] External access properly secured

### 9.2 Performance Testing

- [ ] Measure TLS handshake overhead
- [ ] Test connection pooling
- [ ] Verify session resumption
- [ ] Check certificate cache performance

### 9.3 Security Audit

- [ ] Run security scan on certificates
- [ ] Verify cipher suites (TLS 1.2/1.3 only)
- [ ] Check for weak keys
- [ ] Validate certificate chains
- [ ] Review access logs

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| Certificate verification failed | Check root CA trust, verify SANs match hostname |
| Service won't start with mTLS | Check certificate/key permissions, verify paths |
| Certificate expired | Run `./scripts/rotate-certs.sh` to renew |
| CA health check fails | Ensure step-ca container is running, check port 9000 |
| Permission denied on key file | `chmod 600 *.key` and check file ownership |

### Getting Help

1. Review logs: `docker-compose logs <service>`
2. Check mTLS setup guide: `docs/MTLS_SETUP.md`
3. Verify step-ca status: `step ca health`
4. Review certificate details: `step certificate inspect <file>`

## Success Criteria

- [ ] All services communicating via mTLS
- [ ] Certificate rotation automated
- [ ] Monitoring and alerting configured
- [ ] Documentation complete and accurate
- [ ] Security hardening implemented
- [ ] Backup and recovery tested
- [ ] Team trained on operations

## Next Steps

After completing this checklist:

1. Schedule first certificate rotation test
2. Set up monitoring dashboards
3. Document lessons learned
4. Plan for production deployment
5. Train operations team
6. Schedule security audit

## References

- [MTLS Setup Guide](MTLS_SETUP.md)
- [Step-CA Documentation](https://smallstep.com/docs/)
- [Certificate Templates](../config/step-ca/templates/README.md)
- [Rotation Script](../scripts/rotate-certs.sh)
- [Generation Script](../scripts/generate-service-certs.sh)
