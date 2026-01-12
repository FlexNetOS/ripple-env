# mTLS Setup Guide

Mutual TLS (mTLS) configuration for FlexStack services using Step-CA.

## Overview

This guide covers setting up mTLS authentication for:
- Service-to-service communication
- API endpoints
- Database connections
- Message queues (NATS)

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Step-CA                          │
│            Certificate Authority                    │
│  - Issues certificates                              │
│  - Validates certificates                           │
│  - Manages certificate lifecycle                    │
└─────────────────────────────────────────────────────┘
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
   ┌────────┐      ┌────────┐      ┌────────┐
   │Service │      │Service │      │Service │
   │   A    │◄────►│   B    │◄────►│   C    │
   └────────┘      └────────┘      └────────┘
   mTLS enabled    mTLS enabled    mTLS enabled
```

## Prerequisites

- Kubernetes cluster with Step-CA deployed
- kubectl access to the cluster
- step-cli installed locally

```bash
# Install step-cli (included in devShell)
nix develop

# Or install manually
wget https://dl.step.sm/gh-release/cli/docs-ca-install/v0.27.2/step-cli_0.27.2_amd64.deb
sudo dpkg -i step-cli_0.27.2_amd64.deb
```

## Step 1: Deploy Step-CA

Step-CA is the certificate authority that issues and manages certificates.

```bash
# Deploy Step-CA
kubectl apply -f manifests/kubernetes/base/identity/step-ca.yaml

# Wait for Step-CA to be ready
kubectl wait --for=condition=ready pod -l app.kubernetes.io/name=step-ca \
  -n flexstack-identity --timeout=300s

# Get the CA fingerprint
kubectl exec -n flexstack-identity deploy/step-ca -- \
  step certificate fingerprint /home/step/certs/root_ca.crt
```

## Step 2: Bootstrap step-cli

Configure your local step-cli to trust the CA:

```bash
# Get the CA URL
CA_URL="https://step-ca.flexstack-identity.svc.cluster.local:9000"

# Get the root certificate
kubectl get secret -n flexstack-identity step-ca-secrets \
  -o jsonpath='{.data.root_ca\.crt}' | base64 -d > /tmp/root_ca.crt

# Bootstrap step-cli
step ca bootstrap \
  --ca-url "$CA_URL" \
  --fingerprint "$(kubectl exec -n flexstack-identity deploy/step-ca -- \
    step certificate fingerprint /home/step/certs/root_ca.crt)" \
  --root /tmp/root_ca.crt
```

## Step 3: Generate Service Certificates

### Option A: Manual Certificate Generation

```bash
# Generate a certificate for a service
SERVICE_NAME="my-service"
NAMESPACE="default"

step ca certificate \
  "${SERVICE_NAME}.${NAMESPACE}.svc.cluster.local" \
  "${SERVICE_NAME}.crt" \
  "${SERVICE_NAME}.key" \
  --provisioner "admin" \
  --san "${SERVICE_NAME}" \
  --san "${SERVICE_NAME}.${NAMESPACE}" \
  --san "${SERVICE_NAME}.${NAMESPACE}.svc" \
  --san "${SERVICE_NAME}.${NAMESPACE}.svc.cluster.local"
```

### Option B: Automated Certificate Injection (Recommended)

Use cert-manager or step-issuer for automatic certificate management:

```bash
# Install cert-manager
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.14.0/cert-manager.yaml

# Configure Step-CA as an issuer (see Step 6)
```

## Step 4: Configure Services for mTLS

### Example: NATS with mTLS

The NATS configuration already includes authentication. To add mTLS:

```yaml
# Add to nats.yaml ConfigMap
data:
  nats.conf: |
    # ... existing config ...

    tls {
      cert_file: "/etc/nats/certs/tls.crt"
      key_file: "/etc/nats/certs/tls.key"
      ca_file: "/etc/nats/certs/ca.crt"
      verify: true
      verify_and_map: true
    }
```

### Example: Application with mTLS

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: my-app
spec:
  template:
    spec:
      volumes:
        - name: certs
          secret:
            secretName: my-app-tls
      containers:
        - name: app
          volumeMounts:
            - name: certs
              mountPath: /etc/tls
              readOnly: true
          env:
            - name: TLS_CERT
              value: /etc/tls/tls.crt
            - name: TLS_KEY
              value: /etc/tls/tls.key
            - name: TLS_CA
              value: /etc/tls/ca.crt
```

## Step 5: Certificate Rotation

Step-CA handles automatic certificate rotation:

```yaml
# Add init container for certificate renewal
initContainers:
  - name: cert-renewer
    image: smallstep/step-ca:0.27.2
    command:
      - /bin/sh
      - -c
      - |
        while true; do
          step ca renew /etc/tls/tls.crt /etc/tls/tls.key \
            --force \
            --ca-url https://step-ca.flexstack-identity.svc.cluster.local:9000 \
            --root /etc/tls/ca.crt
          sleep 3600  # Renew every hour
        done
    volumeMounts:
      - name: certs
        mountPath: /etc/tls
```

## Step 6: cert-manager Integration

For production use, integrate Step-CA with cert-manager:

```bash
# Install step-issuer for cert-manager
kubectl apply -f https://github.com/smallstep/step-issuer/releases/latest/download/step-issuer.yaml

# Create a StepClusterIssuer
cat <<EOF | kubectl apply -f -
apiVersion: certmanager.step.sm/v1beta1
kind: StepClusterIssuer
metadata:
  name: step-ca-issuer
spec:
  url: https://step-ca.flexstack-identity.svc.cluster.local:9000
  caBundle: $(kubectl get secret -n flexstack-identity step-ca-secrets \
    -o jsonpath='{.data.root_ca\.crt}')
  provisioner:
    name: admin
    kid: admin
    passwordRef:
      name: step-ca-secrets
      key: provisioner-password
      namespace: flexstack-identity
EOF
```

Then request certificates using cert-manager:

```yaml
apiVersion: cert-manager.io/v1
kind: Certificate
metadata:
  name: my-service-tls
  namespace: default
spec:
  secretName: my-service-tls
  duration: 24h
  renewBefore: 8h
  issuerRef:
    name: step-ca-issuer
    kind: StepClusterIssuer
  dnsNames:
    - my-service.default.svc.cluster.local
    - my-service.default.svc
    - my-service
```

## Step 7: Verify mTLS

### Test Certificate

```bash
# Verify certificate details
step certificate inspect my-service.crt

# Verify certificate chain
step certificate verify my-service.crt \
  --roots /tmp/root_ca.crt
```

### Test Connection

```bash
# Test mTLS connection
curl --cert my-service.crt \
     --key my-service.key \
     --cacert /tmp/root_ca.crt \
     https://other-service:8443/health
```

## Step 8: Monitoring

Monitor certificate expiry:

```bash
# List all certificates
kubectl get certificates -A

# Check certificate expiry
kubectl get certificate my-service-tls -o jsonpath='{.status.notAfter}'

# Set up Prometheus alerts (see observability stack)
```

## Security Best Practices

1. **Short-lived certificates**: Use 24-hour validity with automatic renewal
2. **Least privilege**: Issue certificates only for required SANs
3. **Audit logging**: Enable Step-CA audit logs
4. **Backup**: Regular backup of Step-CA data
5. **Rotation**: Rotate CA certificates annually

## Troubleshooting

### Certificate not trusted

```bash
# Check CA certificate is correct
kubectl get secret -n flexstack-identity step-ca-secrets \
  -o jsonpath='{.data.root_ca\.crt}' | base64 -d | \
  step certificate inspect -
```

### Certificate expired

```bash
# Manually renew certificate
step ca renew my-service.crt my-service.key \
  --force \
  --ca-url https://step-ca.flexstack-identity.svc.cluster.local:9000
```

### Connection refused

```bash
# Check Step-CA is running
kubectl get pods -n flexstack-identity -l app.kubernetes.io/name=step-ca

# Check Step-CA logs
kubectl logs -n flexstack-identity deploy/step-ca
```

## References

- [Step-CA Documentation](https://smallstep.com/docs/step-ca)
- [cert-manager Documentation](https://cert-manager.io/docs/)
- [step-issuer GitHub](https://github.com/smallstep/step-issuer)
- [mTLS Best Practices](https://smallstep.com/blog/zero-trust-swiss-army-knife/)

## Related Documentation

- [Step-CA Manifest](../../manifests/kubernetes/base/identity/step-ca.yaml)
- [Identity Services](../../manifests/kubernetes/base/identity/)
- [Security Architecture](../SECURITY.md)
