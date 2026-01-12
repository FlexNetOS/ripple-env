# Secrets Management

This document describes how secrets are managed in the ROS2 Humble development environment.

## Overview

We use a layered approach to secrets management:

| Layer | Tool | Purpose |
|-------|------|---------|
| Development | `.env` files | Local development secrets (gitignored) |
| Pre-commit | detect-secrets | Prevent accidental secret commits |
| Encryption | agenix | Encrypted secrets for NixOS deployments |

## Quick Start

### Development Environment

1. Copy the example environment file:

   ```bash
   cp .env.example .env
   ```

2. Edit `.env` with your actual credentials:

   ```bash
   $EDITOR .env
   ```

3. The `.envrc` automatically loads `.env` when you enter the directory.

### Pre-commit Secret Detection

Secret detection is automatically enabled via pre-commit hooks:

```bash
# Install pre-commit hooks (one-time setup)
pre-commit install

# Run secret detection manually
pre-commit run detect-secrets --all-files

# Update the baseline after intentional changes
detect-secrets scan --baseline .secrets.baseline
```

## Agenix (Encrypted Secrets)

For NixOS deployments, we use [agenix](https://github.com/ryantm/agenix) to encrypt secrets with SSH keys.

### Setup

1. **Add your SSH public key** to `secrets/secrets.nix`:

   ```nix
   let
     developers = [
       "ssh-ed25519 AAAAC3... your-key-here"
     ];
   in { ... }
   ```

2. **Create a new secret**:

   ```bash
   # Enter the development shell (includes agenix CLI)
   nix develop

   # Create an encrypted secret
   cd secrets
   agenix -e api-key.age
   ```

3. **Use the secret in NixOS configuration**:

   ```nix
   { config, ... }:
   {
     age.secrets.api-key = {
       file = ../secrets/api-key.age;
       owner = "myservice";
     };

     # Access at runtime: config.age.secrets.api-key.path
   }
   ```

### Re-keying Secrets

When adding new SSH keys or rotating keys:

```bash
cd secrets
agenix -r  # Re-encrypt all secrets with updated keys
```

### Best Practices

1. **Never commit unencrypted secrets** - Use `.age` extension for encrypted files only
2. **Rotate keys regularly** - Update SSH keys and re-encrypt secrets periodically
3. **Limit access** - Only add keys for users/systems that need specific secrets
4. **Use separate keys** - Different secrets for dev/staging/production

## Environment Files

### .env.example

Template file with placeholder values. Safe to commit.

```bash
# API Keys (replace with real values in .env)
OPENAI_API_KEY=sk-your-key-here
ANTHROPIC_API_KEY=your-key-here
```

### .env

Your local secrets. **Never commit this file.**

### .env.agixt.example

AGiXT-specific configuration template with development defaults.

## Secret Detection

### Baseline Management

The `.secrets.baseline` file tracks known false positives:

```bash
# Scan and update baseline
detect-secrets scan --baseline .secrets.baseline

# Audit the baseline
detect-secrets audit .secrets.baseline
```

### Supported Detectors

- AWS keys
- Private keys
- High entropy strings
- API tokens
- Database URLs
- And more...

## CI/CD Integration

### GitHub Actions

For GitHub Actions, use encrypted secrets:

1. Go to Repository Settings > Secrets and variables > Actions
2. Add secrets with appropriate names
3. Reference in workflows:

   ```yaml
   env:
     OPENAI_API_KEY: ${{ secrets.OPENAI_API_KEY }}
   ```

### NixOS Deployments

For NixOS systems, secrets are decrypted at activation time:

```nix
{
  imports = [ inputs.agenix.nixosModules.default ];

  age.secrets.database-password = {
    file = ./secrets/database-password.age;
    owner = "postgres";
    group = "postgres";
    mode = "0400";
  };
}
```

## Troubleshooting

### "No secret key available"

Your SSH key isn't authorized to decrypt this secret. Check `secrets/secrets.nix`.

### "Pre-commit hook failed: detect-secrets"

A potential secret was detected. Either:

1. Remove the secret from your code
2. Add it to `.secrets.baseline` if it's a false positive

```bash
detect-secrets audit .secrets.baseline
```

### "Permission denied" on decrypted secret

Check the `owner`, `group`, and `mode` settings in your agenix configuration.

## Security Checklist

- [ ] `.env` is in `.gitignore`
- [ ] Pre-commit hooks are installed
- [ ] SSH keys are added to `secrets/secrets.nix`
- [ ] Secrets are encrypted with `.age` extension
- [ ] CI/CD uses encrypted secrets (not environment variables in code)
- [ ] Production secrets are separate from development

## References

- [Agenix Documentation](https://github.com/ryantm/agenix)
- [detect-secrets](https://github.com/Yelp/detect-secrets)
- [OWASP Secrets Management](https://cheatsheetseries.owasp.org/cheatsheets/Secrets_Management_Cheat_Sheet.html)
