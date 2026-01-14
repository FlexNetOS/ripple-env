# Script Contract: init-step-ca.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/init-step-ca.sh`

---

## Purpose

Initialize Step-CA (Certificate Authority) for ARIA mTLS infrastructure. Generates Root CA, Intermediate CA, JWK provisioner, and updates configuration files with cryptographic values. Essential for zero-trust service-to-service authentication.

---

## Invocation

```bash
./scripts/init-step-ca.sh
```

**Interactive:** Prompts for confirmation if CA already exists (lines 40-52).

**Requirements:**
- `step` CLI installed (available in Nix devShell)
- `jq` for JSON manipulation
- `openssl` for password generation

---

## Outputs

**Standard Output:**
```
🔐 Initializing Step-CA for ARIA
=================================
PKI Directory: /path/to/config/step-ca/pki

📝 Generating CA password...
   ✓ Password saved to: config/step-ca/secrets/password.txt

🔑 Generating Root CA...
   ✓ Root CA generated

🔑 Generating Intermediate CA...
   ✓ Intermediate CA generated

🔑 Generating JWK Provisioner...
   ✓ JWK Provisioner generated

📝 Extracting provisioner values...
📝 Updating ca.json...
   ✓ ca.json updated

📝 Updating defaults.json with root CA fingerprint...
   ✓ defaults.json updated

✅ Step-CA initialization complete!

📋 Summary:
   Root CA:          config/step-ca/pki/root_ca.crt
   Intermediate CA:  config/step-ca/pki/intermediate_ca.crt
   Fingerprint:      abc123...
   Password file:    config/step-ca/secrets/password.txt

🚀 Next steps:
   1. Start the CA: docker-compose -f docker-compose.identity.yml up step-ca
   2. Test health:   step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt
   3. Bootstrap:     step ca bootstrap --ca-url https://localhost:9000 --fingerprint abc123...

⚠️  IMPORTANT: Keep config/step-ca/secrets/password.txt secure!
   In production, store it in HashiCorp Vault or a Kubernetes Secret
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success |
| `1` | Error (step-cli missing, re-init declined) |

---

## Side Effects

### Directories Created (line 27)
- `config/step-ca/pki/` - Public key infrastructure (certificates, keys)
- `config/step-ca/db/` - Step-CA database
- `config/step-ca/secrets/` - Sensitive credentials

### Files Created

**Certificates and Keys (lines 54-73):**
- `pki/root_ca.crt` - Root CA certificate (10 years, lines 54-60)
- `pki/root_ca_key` - Root CA private key (unencrypted)
- `pki/intermediate_ca.crt` - Intermediate CA certificate (5 years, lines 64-73)
- `pki/intermediate_ca_key` - Intermediate CA private key (encrypted with password)

**Provisioner (lines 75-83):**
- `secrets/provisioner.pub.json` - JWK public key (EC P-256)
- `secrets/provisioner.key.json` - JWK private key (encrypted with password)

**Password (lines 30-35):**
- `secrets/password.txt` - 32-byte random password (base64-encoded)

### Files Modified (lines 92-112)

**ca.json (lines 92-103):**
- Updates JWK provisioner coordinates (x, y)
- Injects encrypted private key

**defaults.json (lines 106-112):**
- Updates root CA fingerprint

---

## Safety Classification

**🟡 CAUTION** - Generates cryptographic keys and overwrites existing CA if confirmed.

---

## Idempotency

**⚠️ INTERACTIVE** - Prompts before overwriting existing CA (lines 40-52).

---

## Execution Steps

### 1. Check Dependencies (lines 19-24)

**Evidence:**
```bash
if ! command -v step &> /dev/null; then
    echo "❌ Error: step-cli not found"
    echo "   Please run: nix develop"
    exit 1
fi
```

### 2. Generate Password (lines 30-35)

**Evidence:**
```bash
if [ ! -f "$SECRETS_DIR/password.txt" ]; then
    echo "📝 Generating CA password..."
    openssl rand -base64 32 > "$SECRETS_DIR/password.txt"
    chmod 600 "$SECRETS_DIR/password.txt"
fi
```

**Password:** 32 bytes = 256 bits of entropy (highly secure).

### 3. Check Existing CA (lines 40-52)

**Evidence:**
```bash
if [ -f "$PKI_DIR/root_ca.crt" ]; then
    echo "⚠️  CA already initialized"
    read -p "Do you want to re-initialize? This will DELETE existing certificates [y/N]: " -n 1 -r
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi
    rm -rf "${PKI_DIR:?}"/* "${DB_DIR:?}"/*
fi
```

**Safety:** Requires explicit confirmation, uses parameter expansion safety check (`${PKI_DIR:?}`).

### 4. Generate Root CA (lines 54-62)

**Evidence:**
```bash
step certificate create "ARIA Root CA" \
    "$PKI_DIR/root_ca.crt" "$PKI_DIR/root_ca_key" \
    --profile root-ca \
    --no-password \
    --insecure \
    --not-after=87600h
```

**Parameters:**
- `--profile root-ca` - Root CA constraints (CA:TRUE, pathlen:1)
- `--no-password` - Unencrypted (for automation, secured by file permissions)
- `--insecure` - Skip secure prompts (automation)
- `--not-after=87600h` - 10 years validity

### 5. Generate Intermediate CA (lines 64-73)

**Evidence:**
```bash
step certificate create "ARIA Intermediate CA" \
    "$PKI_DIR/intermediate_ca.crt" "$PKI_DIR/intermediate_ca_key" \
    --profile intermediate-ca \
    --ca "$PKI_DIR/root_ca.crt" \
    --ca-key "$PKI_DIR/root_ca_key" \
    --password-file "$PASSWORD_FILE" \
    --not-after=43800h
```

**Parameters:**
- `--profile intermediate-ca` - Intermediate CA constraints (CA:TRUE, pathlen:0)
- `--ca` and `--ca-key` - Sign with Root CA
- `--password-file` - Encrypted with generated password
- `--not-after=43800h` - 5 years validity

**Best practice:** Intermediate CA expires before Root CA, allows rotation without trusting new root.

### 6. Generate JWK Provisioner (lines 75-83)

**Evidence:**
```bash
step crypto jwk create \
    "$SECRETS_DIR/provisioner.pub.json" \
    "$SECRETS_DIR/provisioner.key.json" \
    --kty EC \
    --crv P-256 \
    --password-file "$PASSWORD_FILE"
```

**JWK (JSON Web Key):** Used for API authentication to Step-CA.

**Algorithm:** ECDSA P-256 (secure, efficient).

### 7. Extract Provisioner Values (lines 86-89)

**Evidence:**
```bash
PROV_X=$(jq -r .x "$SECRETS_DIR/provisioner.pub.json")
PROV_Y=$(jq -r .y "$SECRETS_DIR/provisioner.pub.json")
PROV_KEY=$(jq -r .key "$SECRETS_DIR/provisioner.key.json")
```

**Values:** EC public key coordinates (x, y) and encrypted private key.

### 8. Update ca.json (lines 92-103)

**Evidence:**
```bash
jq \
    --arg x "$PROV_X" \
    --arg y "$PROV_Y" \
    --arg key "$PROV_KEY" \
    '.authority.provisioners[0].key.x = $x |
     .authority.provisioners[0].key.y = $y |
     .authority.provisioners[0].encryptedKey = $key' \
    "$CA_CONFIG" > "$CA_CONFIG.tmp" && mv "$CA_CONFIG.tmp" "$CA_CONFIG"
```

**Purpose:** Injects real cryptographic values into Step-CA configuration.

### 9. Update defaults.json (lines 106-112)

**Evidence:**
```bash
FINGERPRINT=$(step certificate fingerprint "$PKI_DIR/root_ca.crt")
jq --arg fp "$FINGERPRINT" '.fingerprint = $fp' "$DEFAULTS_CONFIG" > "$DEFAULTS_CONFIG.tmp" && \
    mv "$DEFAULTS_CONFIG.tmp" "$DEFAULTS_CONFIG"
```

**Fingerprint:** SHA-256 hash of Root CA, used for bootstrapping clients.

### 10. Set Permissions (lines 115-116)

**Evidence:**
```bash
chmod 600 "$PKI_DIR"/*_key "$SECRETS_DIR"/*.json
chmod 644 "$PKI_DIR"/*.crt
```

**Security:** Private keys readable only by owner, certificates public.

---

## Certificate Validity Periods

**Root CA:** 10 years (87600 hours)
- Long-lived, rarely rotated
- Requires re-trust if renewed

**Intermediate CA:** 5 years (43800 hours)
- Rotated without client re-trust
- Signs service certificates

**Service Certificates:** Typically 90 days (via `generate-service-certs.sh`)
- Short-lived for security
- Automated renewal

---

## Step-CA Architecture

**Two-tier PKI:**
```
Root CA (10y, offline)
    └─ Intermediate CA (5y, online)
           ├─ vault.aria.local (90d)
           ├─ keycloak.aria.local (90d)
           ├─ kong.aria.local (90d)
           └─ ... (other services)
```

**Benefits:**
- Root CA compromise protection (rarely used)
- Intermediate CA rotation without re-trust
- Short-lived leaf certificates (security)

---

## Next Steps

**Displayed in output (lines 127-133):**

1. **Start CA:**
   ```bash
   docker-compose -f docker-compose.identity.yml up step-ca
   ```

2. **Test health:**
   ```bash
   step ca health --ca-url https://localhost:9000 --root config/step-ca/pki/root_ca.crt
   ```

3. **Bootstrap client:**
   ```bash
   step ca bootstrap --ca-url https://localhost:9000 --fingerprint <fingerprint>
   ```

---

## Security Considerations

**Password storage:** File-based (development). Production should use:
- HashiCorp Vault
- Kubernetes Secrets
- AWS Secrets Manager
- Azure Key Vault

**Root CA key:** Unencrypted for automation. Production should:
- Keep offline
- Use hardware security module (HSM)
- Require multi-party approval

**File permissions:** 600 for keys, 644 for certificates.

---

## References

### Source Code
- **Main script:** `scripts/init-step-ca.sh` (135 lines)
- **Password generation:** lines 30-35
- **Root CA creation:** lines 54-62
- **Intermediate CA creation:** lines 64-73
- **JWK provisioner:** lines 75-83
- **Config updates:** lines 86-112

### Related Files
- **CA config:** `config/step-ca/ca.json`
- **Client defaults:** `config/step-ca/defaults.json`
- **Verification:** `scripts/verify-mtls-setup.sh`
- **Cert generation:** `scripts/generate-service-certs.sh`
- **Compose file:** `docker-compose.identity.yml`

### External Resources
- [Step-CA](https://smallstep.com/docs/step-ca/)
- [X.509 Certificate Standard](https://tools.ietf.org/html/rfc5280)
- [JWK Specification](https://tools.ietf.org/html/rfc7517)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 24/60 contracts complete
