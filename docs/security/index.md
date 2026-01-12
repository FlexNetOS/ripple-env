---
title: Security
description: Security configuration and best practices
tags:
  - security
  - mtls
  - secrets
---

# Security

ripple-env implements security best practices including mTLS, secrets management, and supply chain security.

## Overview

```mermaid
graph TB
    subgraph "Identity & Access"
        MTLS[mTLS<br/>Mutual TLS]
        VAULT[Vault<br/>Secrets]
        OIDC[OIDC/OAuth2]
    end

    subgraph "Supply Chain"
        SBOM[SBOM<br/>Generation]
        SIGN[Signing<br/>Cosign]
        SCAN[Scanning<br/>Trivy]
    end

    subgraph "Runtime"
        SEC[Seccomp]
        NET[Network Policies]
        RBAC[RBAC]
    end

    MTLS --> SEC
    VAULT --> RBAC
    OIDC --> RBAC

    SBOM --> SCAN
    SIGN --> SCAN

    style MTLS fill:#4051b5,color:#fff
    style VAULT fill:#4051b5,color:#fff
    style SBOM fill:#00bfa5,color:#fff
```

## Quick Reference

```bash
# Generate SBOM
sbom-generate

# Run security audit
sbom-audit

# Initialize mTLS certificates
step-ca-init
```

## Documentation

<div class="grid cards" markdown>

-   :material-shield-lock:{ .lg .middle } __mTLS Configuration__

    ---

    Configure mutual TLS for service communication

    [:octicons-arrow-right-24: mTLS Guide](mtls.md)

-   :material-key:{ .lg .middle } __Secrets Management__

    ---

    Managing secrets with agenix and Vault

    [:octicons-arrow-right-24: Secrets Guide](secrets.md)

-   :material-package-variant-closed:{ .lg .middle } __Supply Chain Security__

    ---

    SBOM generation and vulnerability scanning

    [:octicons-arrow-right-24: Supply Chain](supply-chain.md)

</div>

## Security Checklist

- [ ] Enable mTLS for all service communication
- [ ] Store secrets in Vault or agenix
- [ ] Generate and verify SBOMs
- [ ] Enable network policies
- [ ] Configure RBAC for Kubernetes
- [ ] Run regular vulnerability scans
