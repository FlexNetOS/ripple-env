# Supply Chain Security

This document describes the supply chain security measures implemented in this repository.

## Overview

Supply chain security ensures that all dependencies and build inputs are verified, reproducible, and auditable. This project implements multiple layers of protection:

1. **Dual Channel Strategy** - Stable and unstable nixpkgs variants
2. **Input Pinning** - All flake inputs are locked with cryptographic hashes
3. **SBOM Generation** - Software Bill of Materials for all dependencies
4. **Attestation** - SLSA provenance for releases

## Dual Channel Strategy

The flake provides two variants for different use cases:

### Development (Unstable)

```bash
# Default shell - uses nixos-unstable for latest packages
nix develop

# Or explicitly
nix develop .#default
```

- Uses `nixos-unstable` channel
- Latest package versions
- Best for development and testing
- May contain newer, less-tested packages

### Production (Stable)

```bash
# Stable shell - uses nixos-24.11
nix develop .#stable
```

- Uses `nixos-24.11` channel (pinned stable release)
- Well-tested package versions
- Security updates backported
- Recommended for production deployments

### NixOS Images

Both stable and unstable image variants are available:

| Image Type | Development | Production |
|------------|-------------|------------|
| WSL2 | `wsl-ripple` | `wsl-ripple-stable` |
| ISO | `iso-ros2` | `iso-ros2-stable` |
| VM | `vm-ros2` | `vm-ros2-stable` |

Build production images:

```bash
# WSL2 stable image
nix build .#nixosConfigurations.wsl-ripple-stable.config.system.build.tarballBuilder

# ISO stable image
nix build .#nixosConfigurations.iso-ros2-stable.config.system.build.isoImage

# VM stable image
nix build .#nixosConfigurations.vm-ros2-stable.config.system.build.vm
```

## Input Pinning

All flake inputs are pinned in `flake.lock` with cryptographic hashes (narHash).

### Verifying Inputs

```bash
# Verify all inputs are properly pinned
verify-inputs

# Check for unstable inputs
verify-inputs .
```

The verification checks:
- Lock file integrity
- All inputs have narHash values
- Detection of unstable branch references
- Flake evaluation success

### Updating Inputs

```bash
# Update all inputs (creates new lock file)
nix flake update

# Update specific input
nix flake lock --update-input nixpkgs-stable

# Show input metadata
nix flake metadata
```

### Input Hash Verification

Each input in `flake.lock` contains:

```json
{
  "locked": {
    "lastModified": 1234567890,
    "narHash": "sha256-...",
    "owner": "nixos",
    "repo": "nixpkgs",
    "rev": "abc123...",
    "type": "github"
  }
}
```

The `narHash` is a cryptographic hash of the input's contents, ensuring:
- Content integrity
- Reproducible builds
- Tamper detection

## SBOM Generation

Software Bill of Materials documents all dependencies.

### CLI Commands

```bash
# Generate Nix-specific SBOM (recommended)
sbom-nix                           # CycloneDX format
sbom-nix . sbom-spdx.json spdx     # SPDX format

# Generate filesystem SBOM
sbom                               # Current directory
sbom ./src cyclonedx-json          # Specific directory

# Full supply chain audit
supply-chain-audit                 # Generates all SBOMs + report
```

### Output Formats

| Format | Use Case | Command |
|--------|----------|---------|
| CycloneDX | Modern compliance, VEX | `sbom-nix . out.json cyclonedx` |
| SPDX | License compliance, legal | `sbom-nix . out.json spdx` |
| Table | Human-readable | `sbom . table` |

### CI/CD Integration

The SBOM workflow (`.github/workflows/sbom.yml`) automatically:

1. Generates SBOMs on push/PR to main
2. Scans for vulnerabilities using Grype
3. Signs SBOMs with Cosign (keyless)
4. Uploads to releases

## Attestation

SLSA (Supply-chain Levels for Software Artifacts) provenance provides cryptographic proof of build provenance.

### Attestation Workflow

The attestation workflow (`.github/workflows/attestation.yml`) runs on releases and:

1. **Verifies Inputs** - Checks flake.lock integrity
2. **Generates Provenance** - Creates SLSA v1.0 provenance document
3. **Signs Artifacts** - Uses Sigstore/Cosign for keyless signing
4. **Creates Bundle** - Packages attestations for verification

### Verifying Attestations

```bash
# Download and verify release attestation
cosign verify-blob \
  --signature provenance.json.sig \
  --certificate provenance.json.cert \
  --certificate-identity-regexp "https://github.com/FlexNetOS/ripple-env/*" \
  --certificate-oidc-issuer "https://token.actions.githubusercontent.com" \
  provenance.json
```

### SLSA Levels

| Level | Requirement | Status |
|-------|-------------|--------|
| SLSA 1 | Documented build process | Yes |
| SLSA 2 | Hosted build service | Yes (GitHub Actions) |
| SLSA 3 | Hardened build platform | Yes |

## Security Commands

Available commands in the development shell:

| Command | Description |
|---------|-------------|
| `sbom` | Generate filesystem SBOM |
| `sbom-nix` | Generate Nix-specific SBOM |
| `verify-inputs` | Verify flake input integrity |
| `supply-chain-audit` | Run comprehensive audit |
| `vuln-scan` | Scan for vulnerabilities |
| `sign-artifact` | Sign artifacts with Cosign |

## Best Practices

### For Development

1. Use `nix develop` for day-to-day work
2. Run `verify-inputs` before pushing changes
3. Update inputs regularly: `nix flake update`

### For Production

1. Use stable variants: `nix develop .#stable`
2. Build stable images: `*-stable` configurations
3. Generate and review SBOMs before deployment
4. Verify attestations for releases

### For Releases

1. Tag releases: `git tag v1.0.0`
2. Attestation workflow runs automatically
3. SBOM and provenance attached to release
4. Verify signatures before deploying

## Vulnerability Management

### Scanning Dependencies

```bash
# Quick scan
vuln-scan

# Scan from SBOM
vuln-scan sbom.json grype

# Full audit with vulnerability report
supply-chain-audit
```

### Triaging Vulnerabilities

1. Critical/High: Address before release
2. Medium: Assess impact, plan remediation
3. Low: Document and monitor

### Remediation

```bash
# Update vulnerable input
nix flake lock --update-input nixpkgs

# Rebuild and verify
nix develop .#stable
verify-inputs
```

## Related Documentation

- [Getting Started](getting-started/GETTING_STARTED.md) - Initial setup
- [Troubleshooting](TROUBLESHOOTING.md) - Common issues
- [NIX_FLAKE_MODULARIZATION.md](NIX_FLAKE_MODULARIZATION.md) - Flake structure

## External Resources

- [SLSA Framework](https://slsa.dev/)
- [Sigstore](https://sigstore.dev/)
- [CycloneDX Specification](https://cyclonedx.org/)
- [SPDX Specification](https://spdx.dev/)
- [Nix Flakes Reference](https://nixos.wiki/wiki/Flakes)
