# Certificate Templates for Step-CA

This directory contains X.509 certificate templates used by Step-CA to issue certificates for ARIA services.

## Directory Structure

```
templates/
├── x509/
│   ├── leaf.tpl           # Service certificate template (mTLS)
│   ├── intermediate.tpl   # Intermediate CA template
│   └── root.tpl          # Root CA template
└── README.md
```

## Template Overview

### Leaf Certificate Template (`x509/leaf.tpl`)

Used for service certificates that enable mTLS between ARIA components.

**Features:**
- Subject organization: ARIA Platform
- Key usage: Digital signature, key encipherment
- Extended key usage: Server auth, client auth (for mTLS)
- Custom extensions for environment and service metadata
- Subject Alternative Names (SANs) for service discovery

**Example certificate:**
```
Subject: CN=keycloak.identity.svc.cluster.local, O=ARIA Platform, OU=Infrastructure
SANs: keycloak.identity.svc.cluster.local, keycloak, localhost
Key Usage: Digital Signature, Key Encipherment
Extended Key Usage: TLS Web Server Authentication, TLS Web Client Authentication
```

### Intermediate CA Template (`x509/intermediate.tpl`)

Used for intermediate CA certificates that sign service certificates.

**Features:**
- CA flag: true
- Path length: 0 (can only sign end-entity certificates)
- Key usage: Certificate signing, CRL signing

### Root CA Template (`x509/root.tpl`)

Used for root CA certificates (top of trust chain).

**Features:**
- CA flag: true
- Path length: 1 (can sign intermediates and end-entity certs)
- Key usage: Certificate signing, CRL signing

## Using Templates

### With Step-CA Configuration

Templates are referenced in `ca.json`:

```json
{
  "authority": {
    "provisioners": [
      {
        "type": "ACME",
        "name": "acme",
        "options": {
          "x509": {
            "templateFile": "/home/step/templates/x509/leaf.tpl"
          }
        }
      }
    ]
  }
}
```

### With Token Claims

Templates can use token claims to customize certificates:

```bash
# Request certificate with custom claims
step ca certificate service.example.com \
  service.crt service.key \
  --provisioner acme \
  --set environment=production \
  --set service=keycloak
```

This populates template variables:
- `{{ .Token.environment }}` → `production`
- `{{ .Token.service }}` → `keycloak`

## Template Variables

### Available Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `.Subject.CommonName` | Certificate CN | `keycloak.identity.svc.cluster.local` |
| `.SANs` | Subject Alternative Names | `["keycloak", "localhost"]` |
| `.Token.environment` | Deployment environment | `development`, `staging`, `production` |
| `.Token.service` | Service name | `keycloak`, `vault`, `temporal` |

### Template Functions

| Function | Description | Example |
|----------|-------------|---------|
| `toJson` | Convert to JSON | `{{ toJson .SANs }}` |
| `default` | Provide default value | `{{ .Token.env \| default "dev" }}` |

## Custom Extensions

Templates define custom X.509 extensions for ARIA metadata:

### Extension 1.3.6.1.4.1.99999.1 - Environment

Stores the deployment environment (development, staging, production).

**Usage:**
```bash
openssl x509 -in service.crt -text -noout | grep -A1 "1.3.6.1.4.1.99999.1"
```

### Extension 1.3.6.1.4.1.99999.2 - Service

Stores the service name for identification.

**Usage:**
```bash
openssl x509 -in service.crt -text -noout | grep -A1 "1.3.6.1.4.1.99999.2"
```

## Modifying Templates

### Adding New Fields

To add new subject fields:

```json
{
  "subject": {
    "commonName": {{ toJson .Subject.CommonName }},
    "organization": "ARIA Platform",
    "locality": "{{ .Token.datacenter | default \"us-east-1\" }}"  # New field
  }
}
```

### Adding New Extensions

To add custom extensions:

```json
{
  "extensions": [
    {
      "id": "1.3.6.1.4.1.99999.3",
      "critical": false,
      "value": "{{ .Token.cluster | default \"default\" }}"
    }
  ]
}
```

## Validation

### Test Template Syntax

```bash
# Validate JSON syntax
jq empty config/step-ca/templates/x509/leaf.tpl

# Test with step-ca
step ca certificate test.local test.crt test.key \
  --template config/step-ca/templates/x509/leaf.tpl
```

### Inspect Generated Certificate

```bash
# View certificate details
step certificate inspect test.crt

# Check specific fields
openssl x509 -in test.crt -noout -text | grep -E "(Subject:|DNS:|X509v3)"
```

## Best Practices

1. **Keep templates simple**: Only add fields that are necessary
2. **Use defaults**: Provide sensible defaults for optional fields
3. **Validate before deploying**: Test templates with step-ca before production
4. **Version control**: Track template changes in git
5. **Document custom extensions**: Explain the purpose of custom OIDs

## Security Considerations

### Key Usage

- **Digital Signature**: Always include for authentication
- **Key Encipherment**: Required for TLS key exchange
- **Certificate Signing**: Only for CA certificates

### Extended Key Usage

- **Server Auth**: For services accepting connections
- **Client Auth**: For services making mTLS connections
- Both are required for bidirectional mTLS

### Certificate Constraints

- Set `isCA: false` for service certificates
- Set `maxPathLen: 0` to prevent delegation
- Use short validity periods (90 days recommended)

## Troubleshooting

### Template Not Found

```bash
# Ensure templates are mounted in docker-compose
docker compose -f docker/docker-compose.identity.yml exec step-ca ls -la /home/step/templates/x509/
```

### Invalid Template Syntax

```bash
# Check step-ca logs for template errors
docker compose -f docker/docker-compose.identity.yml logs step-ca | grep template
```

### Missing Variables

If a template variable is missing, the certificate request will fail:

```
Error: template: leaf.tpl:5:15: executing "leaf.tpl" at <.Token.missing>: can't evaluate field missing
```

**Solution**: Either provide the variable or use a default:
```
{{ .Token.missing | default "default-value" }}
```

## References

- [Step-CA Templates Documentation](https://smallstep.com/docs/step-ca/templates)
- [X.509 Certificate Extensions](https://datatracker.ietf.org/doc/html/rfc5280#section-4.2)
- [ARIA mTLS Setup Guide](../../docs/MTLS_SETUP.md)
- [Step-CA Configuration](../ca.json)

## Examples

See `../../docs/MTLS_SETUP.md` for complete examples of certificate generation using these templates.
