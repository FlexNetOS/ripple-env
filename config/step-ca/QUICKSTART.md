# Step-CA Quick Start Guide

This is a quick reference for getting started with mTLS in ARIA.

## Prerequisites

```bash
# Enter Nix development environment
nix develop
```

## 3-Step Setup

### Step 1: Initialize the Certificate Authority

```bash
./scripts/init-step-ca.sh
```

**What it does**:
- Generates root CA (10-year validity)
- Generates intermediate CA (5-year validity)
- Creates secure password for CA
- Configures ACME provisioner

**Output**: Certificates in `config/step-ca/pki/`

### Step 2: Start Step-CA

```bash
docker compose -f docker/docker-compose.identity.yml up -d step-ca
```

**What it does**:
- Starts Step-CA on port 9000
- Loads PKI from `config/step-ca/`
- Enables ACME and JWK provisioners

**Verify**:
```bash
# Check container
docker compose -f docker/docker-compose.identity.yml ps step-ca

# Check health
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt
```

### Step 3: Generate Service Certificates

```bash
./scripts/generate-service-certs.sh
```

**What it does**:
- Generates certificates for all ARIA services
- Uses ACME provisioner (automated)
- Configures proper SANs

**Output**: Certificates in `data/certs/<service>/`

## Enable mTLS

### Option 1: Quick Test (Keycloak)

```bash
# 1. Edit docker-compose.identity.yml
# 2. Uncomment these lines in keycloak service:
#    - volumes for certificates
#    - environment variables for mTLS
# 3. Restart
docker compose -f docker/docker-compose.identity.yml restart keycloak
```

### Option 2: Test with curl

```bash
# Test mTLS connection
curl --cacert config/step-ca/pki/root_ca.crt \
     --cert data/certs/keycloak/keycloak.crt \
     --key data/certs/keycloak/keycloak.key \
     https://localhost:8443/health
```

## Common Commands

### Certificate Management

```bash
# Inspect certificate
step certificate inspect data/certs/keycloak/keycloak.crt

# Check expiration
step certificate inspect data/certs/keycloak/keycloak.crt --format json | jq .validity.end

# Verify certificate
step certificate verify data/certs/keycloak/keycloak.crt --roots config/step-ca/pki/root_ca.crt

# Renew certificate
step ca renew data/certs/keycloak/keycloak.crt data/certs/keycloak/keycloak.key --force
```

### CA Management

```bash
# Check CA health
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt

# View CA configuration
cat config/step-ca/ca.json | jq .

# Check logs
docker compose -f docker/docker-compose.identity.yml logs step-ca
```

### Verification

```bash
# Run full verification
./scripts/verify-mtls-setup.sh
```

## Troubleshooting

### CA won't start

```bash
# Check password file exists
ls -l config/step-ca/secrets/password.txt

# Check PKI files exist
ls -l config/step-ca/pki/

# Re-initialize if needed
rm -rf config/step-ca/pki config/step-ca/db config/step-ca/secrets
./scripts/init-step-ca.sh
```

### Certificate request fails

```bash
# Check CA is healthy
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt

# Bootstrap step-cli
step ca bootstrap \
  --ca-url https://localhost:9000 \
  --fingerprint $(step certificate fingerprint config/step-ca/pki/root_ca.crt)
```

### Service won't connect with mTLS

```bash
# Verify certificate
step certificate verify data/certs/keycloak/keycloak.crt --roots config/step-ca/pki/root_ca.crt

# Check certificate SANs
step certificate inspect data/certs/keycloak/keycloak.crt | grep DNS

# Check expiration
step certificate inspect data/certs/keycloak/keycloak.crt | grep "Not After"
```

## File Locations

| File | Location | Purpose |
|------|----------|---------|
| Root CA | `config/step-ca/pki/root_ca.crt` | Trust anchor |
| Intermediate CA | `config/step-ca/pki/intermediate_ca.crt` | Issues certs |
| CA Password | `config/step-ca/secrets/password.txt` | CA key password |
| CA Config | `config/step-ca/ca.json` | CA settings |
| Service Certs | `data/certs/<service>/` | Service certificates |

## Security Notes

⚠️ **IMPORTANT**:
- Never commit files in `config/step-ca/pki/` or `config/step-ca/secrets/`
- Keep `password.txt` secure (600 permissions)
- In production, use HashiCorp Vault or Kubernetes Secrets for password storage
- Rotate certificates regularly (1-year default, consider 90 days for production)

## Next Steps

1. ✅ CA initialized
2. ✅ Step-CA running
3. ✅ Service certificates generated
4. 🎯 Enable mTLS in docker-compose files
5. 🎯 Test service-to-service mTLS
6. 🎯 Set up automated renewal
7. 🎯 Add certificate monitoring

## Full Documentation

For complete documentation, see:
- [docs/MTLS_SETUP.md](../../docs/MTLS_SETUP.md) - Complete mTLS guide
- [config/step-ca/README.md](./README.md) - Configuration details
- [P1-005-IMPLEMENTATION-SUMMARY.md](../../P1-005-IMPLEMENTATION-SUMMARY.md) - Implementation details
