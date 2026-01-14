# Secret Rotation Procedures

Monthly and emergency procedures for rotating API keys, certificates, and encrypted secrets.

## What's in the repo (evidence)

- Certificate rotation: `scripts/rotate-certs.sh`
- Agenix encrypted secrets: `secrets/secrets.nix`, `secrets/*.age`
- Secret detection: `.secrets.baseline`, `detect-secrets` pre-commit hook
- Secret management docs: `docs/SECRETS.md`
- Environment templates: `.env.example`, `.env.agixt.example`

## Goal

1. Rotate secrets regularly to limit exposure window
2. Maintain zero-downtime during certificate rotation
3. Keep encrypted secrets synchronized with authorized SSH keys
4. Update external provider API keys through their dashboards

## Quick start

### Step-CA Certificate Rotation (Automated)

Rotate mTLS certificates for service-to-service authentication:

```bash
# Rotate all certificates (zero-downtime)
./scripts/rotate-certs.sh

# Check certificate expiry dates
./scripts/rotate-certs.sh --check

# Dry-run to see what would be rotated
./scripts/rotate-certs.sh --dry-run
```

**Frequency**: Automatic rotation when certificates are within 30 days of expiry
**Evidence**: `scripts/rotate-certs.sh` handles Vault, NATS, Keycloak, PostgreSQL certificates

### Agenix Encrypted Secrets (Re-keying)

When adding/removing SSH keys or rotating encrypted secrets:

```bash
# 1. Update SSH keys in secrets/secrets.nix
$EDITOR secrets/secrets.nix

# 2. Re-encrypt all secrets with new keys
cd secrets
agenix -r

# 3. Verify re-encryption worked
git status  # Should show modified .age files

# 4. Commit the re-encrypted secrets
git add *.age secrets.nix
git commit -m "chore: rotate agenix keys"
```

**Frequency**: When team members join/leave, or quarterly
**Evidence**: `docs/SECRETS.md` lines 89-96

### External API Keys (Manual Dashboard Rotation)

API keys for external providers must be rotated through their web dashboards:

**OpenAI**:
1. Log in to https://platform.openai.com/account/api-keys
2. Create new API key
3. Update `.env`: `OPENAI_API_KEY=sk-new-key`
4. Delete old key from OpenAI dashboard
5. Test: `curl -H "Authorization: Bearer $OPENAI_API_KEY" https://api.openai.com/v1/models`

**Anthropic**:
1. Log in to https://console.anthropic.com/settings/keys
2. Create new API key
3. Update `.env`: `ANTHROPIC_API_KEY=new-key`
4. Delete old key from Anthropic console
5. Test with Claude API call

**Other Providers**:
- Follow similar pattern: create new key → update `.env` → delete old key
- See `docs/PROVIDERS.md` for full provider list

**Frequency**: Quarterly or immediately if compromised
**Evidence**: `.env.example` contains API key placeholders

### Vault Token Rotation

Rotate HashiCorp Vault root/app tokens:

```bash
# 1. Generate new token (requires existing valid token)
export VAULT_ADDR=http://localhost:8200
vault token create -policy=admin -ttl=8760h

# 2. Update .env with new token
echo "VAULT_TOKEN=hvs.new-token" >> .env

# 3. Revoke old token
vault token revoke <old-token-id>

# 4. Verify new token works
vault status
```

**Frequency**: Annually or when team changes
**Evidence**: Vault is deployed in `docker/docker-compose.identity.yml`

### GitHub Secrets (Web UI)

Rotate secrets used in GitHub Actions CI/CD:

1. Navigate to: Settings → Secrets and variables → Actions
2. Update secrets:
   - `DOCKER_HUB_TOKEN`
   - `NIX_CACHE_PRIV_KEY` (if using cachix)
   - Any provider API keys used in CI
3. Re-run latest workflow to verify

**Frequency**: Quarterly or when access changes
**Evidence**: `.github/workflows/` contains CI pipelines

## Emergency Rotation Checklist

If an API key or secret is compromised:

### Immediate Actions (Within 1 Hour)

- [ ] Rotate compromised credential immediately through provider dashboard
- [ ] Update `.env` or encrypted secret file
- [ ] Revoke old credential from provider
- [ ] Check logs for unauthorized usage (see `docs/OBSERVABILITY-QUICK-START.md`)
- [ ] If certificate compromised: `./scripts/rotate-certs.sh --force`

### Follow-up Actions (Within 24 Hours)

- [ ] Review how secret was exposed
- [ ] Run secret scan: `detect-secrets scan --baseline .secrets.baseline`
- [ ] Update `.secrets.baseline` if needed
- [ ] Document incident (even if internal)
- [ ] Consider rotating related secrets

### Prevention

- [ ] Enable pre-commit hooks: `pre-commit install`
- [ ] Never commit `.env` files (verify `.gitignore`)
- [ ] Use `agenix` for secrets in Nix configs
- [ ] Audit secret access quarterly

## Rollback

If rotation causes service outages:

```bash
# Restore previous .env from backup
cp .env.backup .env

# Restore previous certificates (if you kept backup)
cp -r docker/data/step-ca/certs.backup docker/data/step-ca/certs

# Restart affected services
docker compose restart
```

**Prevention**: Always test new credentials in dev/staging first.

## Verification

After rotation, verify services still work:

```bash
# Run end-to-end validation
./scripts/validate-e2e.sh

# Check service health
docker compose ps

# Test API with new keys
curl -H "Authorization: Bearer $NEW_API_KEY" <endpoint>
```

**Exit codes**:
- `0` = All services healthy
- `1` = Warnings (investigate but not critical)
- `2` = Critical failures (rollback immediately)

**Evidence**: `scripts/validate-e2e.sh` tests all critical services

## Related docs

- [Secrets Management](../secrets/SECRETS.md) - Full agenix/detect-secrets documentation
- [mTLS Setup](../getting-started/MTLS_SETUP.md) - Certificate infrastructure details
- [Providers](../providers/PROVIDERS.md) - Complete provider authentication matrix
- [Observability](../getting-started/quick-start/OBSERVABILITY-QUICK-START.md) - Logging for rotation audits
