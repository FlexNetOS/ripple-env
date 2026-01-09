# Step-CA Configuration

This directory contains the configuration for the smallstep Certificate Authority (CA) used for mTLS between ARIA services.

## Directory Structure

```
config/step-ca/
├── ca.json              # Main CA configuration
├── defaults.json        # Default client configuration
├── .gitignore          # Ignore sensitive files
├── README.md           # This file
├── pki/                # Generated certificates (not in git)
│   ├── root_ca.crt
│   ├── intermediate_ca.crt
│   └── intermediate_ca_key
└── db/                 # BadgerDB database (not in git)
```

## Initial Setup

### 1. Initialize the CA

Run the initialization script to generate certificates and keys:

```bash
# From the project root
./scripts/init-step-ca.sh
```

This will:
- Generate a root CA certificate
- Generate an intermediate CA certificate
- Create the provisioner key
- Update ca.json with the correct values
- Create a password file for the CA

### 2. Verify Configuration

```bash
# Check the CA configuration
step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt

# Inspect the root certificate
step certificate inspect config/step-ca/pki/root_ca.crt
```

## Development Usage

### Start the CA Server

```bash
# Using docker-compose
docker-compose -f docker-compose.identity.yml up step-ca

# Or standalone
step-ca config/step-ca/ca.json
```

### Generate Service Certificates

```bash
# Bootstrap a service with the CA
step ca bootstrap --ca-url https://localhost:9000 \
  --fingerprint $(step certificate fingerprint config/step-ca/pki/root_ca.crt)

# Request a certificate for a service
step ca certificate keycloak.identity.svc.cluster.local \
  keycloak.crt keycloak.key \
  --provisioner acme
```

## Production Considerations

### Security

1. **Password Protection**: The intermediate CA key is encrypted with a password
   - Store password in HashiCorp Vault or Kubernetes Secret
   - Use `--password-file` flag to provide password to step-ca

2. **Key Storage**: Consider using a KMS for root key storage
   - AWS KMS, Google Cloud KMS, Azure Key Vault
   - Or use a Hardware Security Module (HSM)

3. **Network Security**:
   - Run step-ca on an internal network only
   - Use mTLS for CA API access
   - Implement rate limiting for certificate requests

### High Availability

For production, consider:
- Multiple step-ca replicas with shared database
- Database replication (PostgreSQL instead of BadgerDB)
- Load balancer for CA endpoints
- Certificate renewal automation

### Monitoring

Monitor these metrics:
- Certificate issuance rate
- Certificate expiration dates
- Failed authentication attempts
- CA API health checks

## References

- [smallstep Documentation](https://smallstep.com/docs/)
- [Step-CA Configuration Reference](https://smallstep.com/docs/step-ca/configuration)
- [mTLS Best Practices](https://smallstep.com/hello-mtls)
