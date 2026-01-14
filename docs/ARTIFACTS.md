# Artifacts & Publication

**Status:** Complete
**Last Updated:** 2026-01-14
**Purpose:** Document artifact generation, storage, and publication workflows

---

## Overview

This document describes how FlexStack artifacts are built, stored, and distributed.

---

## Artifact Types

| Type | Format | Build System | Distribution |
|------|--------|--------------|--------------|
| NixOS Images | tar.gz, ISO, QCOW2 | Nix Flake | Local build |
| Helm Chart | tgz | Helm | OCI Registry / Manual |
| Docker Images | OCI | Docker Compose | Local Registry |
| SBOM | SPDX/CycloneDX | GitHub Actions | Release Artifacts |
| Nix Flake | Nix | Nix | FlakeHub |

---

## NixOS Images

### Build Commands

| Image | Build Command | Output |
|-------|---------------|--------|
| WSL | `nix build .#nixosConfigurations.wsl-ripple.config.system.build.tarballBuilder` | `result/nixos-wsl.tar.gz` |
| ISO | `nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage` | `result/iso/nixos-ros2-*.iso` |
| VM | `nix build .#nixosConfigurations.vm-ros2.config.system.build.vm` | `result/bin/run-nixos-ros2-vm` |

### Stable Variants

For production use, append `-stable` to the configuration name:

```bash
# Production WSL tarball
nix build .#nixosConfigurations.wsl-ripple-stable.config.system.build.tarballBuilder

# Production ISO
nix build .#nixosConfigurations.iso-ros2-stable.config.system.build.isoImage
```

### Distribution

NixOS images are built locally and not published to a central registry. For distribution:

1. **Internal Distribution:** Share via MinIO S3 bucket
2. **External Distribution:** Upload to GitHub Releases

---

## Docker Images

### Local Development

Docker images are pulled from public registries and run locally. No custom images are published.

### Image Sources

| Service | Image | Registry |
|---------|-------|----------|
| LocalAI | `localai/localai:v2.24.2-aio-cpu` | Docker Hub |
| AGiXT | `joshxt/agixt:main` | Docker Hub |
| Keycloak | `quay.io/keycloak/keycloak:26.0` | Quay.io |
| Temporal | `temporalio/auto-setup:1.26.2` | Docker Hub |
| Neo4j | `neo4j:5-community` | Docker Hub |
| Kong | `kong:3.9.0-alpine` | Docker Hub |
| NATS | `nats:2.10.24-alpine` | Docker Hub |
| OPA | `openpolicyagent/opa:0.71.0-rootless` | Docker Hub |

### Custom Images

The repository includes one custom Dockerfile:

| Image | Dockerfile | Purpose |
|-------|------------|---------|
| open-lovable | `config/dockerfiles/Dockerfile.open-lovable` | Open Lovable UI |

**Build locally:**
```bash
docker build -t open-lovable:latest -f config/dockerfiles/Dockerfile.open-lovable .
```

### Future: Container Registry

When publishing custom images, use GitHub Container Registry (GHCR):

```yaml
# .github/workflows/docker-publish.yml (future)
name: Publish Docker Images

on:
  push:
    tags: ['v*']

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Login to GHCR
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}

      - name: Build and push
        uses: docker/build-push-action@v5
        with:
          push: true
          tags: ghcr.io/${{ github.repository }}/open-lovable:${{ github.ref_name }}
```

---

## Helm Chart

### Location

```
charts/flexstack/
├── Chart.yaml          # Metadata
├── values.yaml         # Default values
└── templates/          # Kubernetes manifests
```

### Packaging

```bash
# Package chart
helm package charts/flexstack

# Output: flexstack-1.0.0.tgz
```

### Publication Options

#### Option 1: GitHub Releases (Current)

Upload `flexstack-1.0.0.tgz` to GitHub Releases.

```bash
# Create release with Helm chart
gh release create v1.0.0 flexstack-1.0.0.tgz
```

#### Option 2: OCI Registry (Recommended)

Push to GitHub Container Registry:

```bash
# Login
helm registry login ghcr.io -u $GITHUB_USER -p $GITHUB_TOKEN

# Push
helm push flexstack-1.0.0.tgz oci://ghcr.io/flexnetos/charts

# Install from OCI
helm install flexstack oci://ghcr.io/flexnetos/charts/flexstack --version 1.0.0
```

#### Option 3: GitHub Pages

Host a Helm repository on GitHub Pages:

```yaml
# .github/workflows/helm-publish.yml (future)
name: Publish Helm Chart

on:
  push:
    tags: ['v*']

jobs:
  publish:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Package chart
        run: helm package charts/flexstack

      - name: Update index
        run: |
          helm repo index . --url https://flexnetos.github.io/ripple-env/charts

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: .
```

---

## SBOM (Software Bill of Materials)

### Generation

SBOM is generated via GitHub Actions on each release.

**Workflow:** `.github/workflows/sbom.yml`

### Formats

| Format | Tool | Output |
|--------|------|--------|
| SPDX | syft | `sbom-spdx.json` |
| CycloneDX | syft | `sbom-cyclonedx.json` |

### Generation Commands

```bash
# Generate SPDX SBOM for Docker image
syft localai/localai:v2.24.2-aio-cpu -o spdx-json > sbom-localai.spdx.json

# Generate CycloneDX SBOM
syft localai/localai:v2.24.2-aio-cpu -o cyclonedx-json > sbom-localai.cdx.json

# Generate SBOM for filesystem
syft dir:. -o spdx-json > sbom-repo.spdx.json
```

### Distribution

SBOMs are attached to GitHub Releases as artifacts:

```yaml
# Part of release.yml workflow
- name: Generate SBOM
  run: |
    syft dir:. -o spdx-json > sbom-${{ github.ref_name }}.spdx.json

- name: Upload SBOM
  uses: softprops/action-gh-release@v1
  with:
    files: sbom-*.json
```

---

## Nix Flake

### FlakeHub Publication

Nix flakes are published to [FlakeHub](https://flakehub.com) on tagged releases.

**Workflow:** `.github/workflows/flakehub-publish-tagged.yml`

```yaml
name: "Publish tags to FlakeHub"

on:
  workflow_dispatch:
    inputs:
      tag:
        description: "The existing tag to publish to FlakeHub"
        required: true

jobs:
  flakehub-publish:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          ref: ${{ inputs.tag }}

      - uses: DeterminateSystems/nix-installer-action@main
      - uses: DeterminateSystems/flakehub-push@main
        with:
          visibility: "public"
          tag: ${{ inputs.tag }}
```

### Usage

```nix
{
  inputs.ripple-env.url = "https://flakehub.com/f/FlexNetOS/ripple-env/*.tar.gz";

  outputs = { ripple-env, ... }: {
    # Use modules from ripple-env
  };
}
```

---

## Release Workflow

### Automated Release

**Workflow:** `.github/workflows/release.yml`

On version tag push:

1. **Build Artifacts**
   - Package Helm chart
   - Generate SBOM
   - Build NixOS images (optional)

2. **Create GitHub Release**
   - Auto-generate changelog
   - Attach artifacts
   - Publish release notes

3. **Publish to Registries**
   - FlakeHub (Nix flake)
   - GHCR (Helm chart, optional)

### Manual Release

```bash
# Tag release
git tag -a v1.0.0 -m "Release v1.0.0"
git push origin v1.0.0

# GitHub Actions triggers automatically
```

---

## Storage Locations

### Local Development

| Artifact | Location |
|----------|----------|
| Nix build outputs | `./result/` |
| Docker volumes | Named volumes |
| Helm packages | `./charts/` |

### CI/CD

| Artifact | Location |
|----------|----------|
| Build artifacts | GitHub Actions cache |
| Release assets | GitHub Releases |
| Helm charts | GHCR / GitHub Pages |
| SBOM | GitHub Releases |
| Nix flakes | FlakeHub |

---

## Verification

### Verify Helm Chart

```bash
# Verify signature (future, requires signing)
helm verify flexstack-1.0.0.tgz

# Lint chart
helm lint charts/flexstack

# Template dry-run
helm template flexstack charts/flexstack --debug
```

### Verify SBOM

```bash
# Validate SPDX format
spdx-sbom-generator validate sbom.spdx.json

# Scan for vulnerabilities
grype sbom:sbom.spdx.json
```

### Verify Nix Flake

```bash
# Check flake outputs
nix flake check

# Show flake info
nix flake show
```

---

## References

- [Helm Chart Best Practices](https://helm.sh/docs/chart_best_practices/)
- [SPDX Specification](https://spdx.dev/)
- [CycloneDX Specification](https://cyclonedx.org/)
- [FlakeHub Documentation](https://flakehub.com/docs)
- [GitHub Container Registry](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)
