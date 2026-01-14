# Script Contract: prepare-offline.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/prepare-offline.sh`

---

## Purpose

Offline/air-gapped installation preparation tool. Downloads and caches Nix packages, Pixi/Conda packages, Docker images, and AI models for later offline use. Supports minimal and full package sets, selective component caching, and creates portable tarball bundles for transfer to air-gapped environments.

---

## Invocation

```bash
./scripts/prepare-offline.sh [OPTIONS]
```

**Options:**
- `--cache-dir DIR` - Base cache directory (default: /var/cache/ripple-env)
- `--nix` - Prepare Nix packages only
- `--pixi` - Prepare Pixi/Conda packages only
- `--docker` - Prepare Docker images only
- `--models` - Prepare AI models only
- `--all` - Prepare everything (default)
- `--minimal` - Minimal packages for basic operation
- `--full` - Full packages including optional components
- `--help` - Show help

**Examples:**
```bash
./scripts/prepare-offline.sh                      # Minimal packages, all components
./scripts/prepare-offline.sh --full               # Full packages, all components
./scripts/prepare-offline.sh --nix --docker       # Only Nix + Docker
./scripts/prepare-offline.sh --cache-dir /mnt/cache --full
```

---

## Side Effects

### Cache Directory Structure (lines 108-117)
```bash
mkdir -p "$CACHE_DIR"/{nix-store,conda-channels,docker-images,models,git-mirrors,git-bundles}
```

Creates:
- `$CACHE_DIR/nix-store/` - Nix binary cache
- `$CACHE_DIR/conda-channels/` - Pixi/Conda package cache
- `$CACHE_DIR/docker-images/` - Docker image tarballs
- `$CACHE_DIR/models/` - AI model files (GGUF)

### Bundle Archive (lines 330-352)
Creates `ripple-env-offline-{minimal|full}-YYYYMMDD.tar.gz` in project root.

---

## Safety Classification

**🟢 SAFE** - Downloads and caches packages only.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Re-running updates cache, skips existing files.

---

## Key Features

### Nix Store Caching (lines 119-158)

```bash
prepare_nix() {
    log_info "Preparing Nix packages..."

    local nix_cache="$CACHE_DIR/nix-store"

    # Build the devshell and copy closure to cache
    log_info "Building devshell closure..."
    nix build "$PROJECT_ROOT#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').default" \
        --out-link "$CACHE_DIR/result-devshell"

    # Copy the closure to local cache
    log_info "Copying Nix store closure to cache..."
    nix copy --to "file://$nix_cache" "$CACHE_DIR/result-devshell"

    # Build additional shells based on package set
    if [[ "$PACKAGE_SET" == "full" ]]; then
        for shell in minimal ros2 identity; do
            log_info "Building $shell shell..."
            if nix build "$PROJECT_ROOT#devShells.$(nix eval --impure --raw --expr 'builtins.currentSystem').$shell" \
                --out-link "$CACHE_DIR/result-$shell" 2>/dev/null; then
                nix copy --to "file://$nix_cache" "$CACHE_DIR/result-$shell"
            fi
        done
    fi
}
```

**Full package set includes:**
- Default devshell
- Minimal shell
- ROS2 shell
- Identity shell

### Pixi/Conda Caching (lines 160-194)

```bash
prepare_pixi() {
    log_info "Preparing Pixi/Conda packages..."

    local conda_cache="$CACHE_DIR/conda-channels"

    # Install packages to populate cache
    log_info "Installing Pixi packages to populate cache..."
    cd "$PROJECT_ROOT"
    pixi install

    # Get pixi cache location
    local pixi_cache_dir
    pixi_cache_dir=$(pixi info --json 2>/dev/null | jq -r '.cache_dir // empty' || echo "$HOME/.cache/rattler")

    # Copy conda packages to our cache
    if [[ -d "$pixi_cache_dir" ]]; then
        log_info "Copying Pixi cache to $conda_cache..."
        cp -r "$pixi_cache_dir"/* "$conda_cache/" 2>/dev/null || true
    fi
}
```

**Populates cache from:**
- All Pixi environments (default, cuda, aios, llmops, etc.)
- Conda-forge packages
- RoboStack-humble packages

### Docker Image Sets (lines 196-267)

**Minimal Package Set (lines 197-201):**
```bash
DOCKER_IMAGES_MINIMAL=(
    "redis:7-alpine"
    "postgres:17.2-alpine"
    "nats:2.10.24-alpine"
)
```

**Full Package Set (lines 203-216):**
```bash
DOCKER_IMAGES_FULL=(
    "${DOCKER_IMAGES_MINIMAL[@]}"
    "localai/localai:v2.24.2-aio-cpu"
    "temporalio/auto-setup:1.26.2"
    "temporalio/ui:2.34.0"
    "minio/minio:RELEASE.2025-09-07T16-13-09Z"
    "grafana/grafana:11.4.0"
    "prom/prometheus:v2.48.0"
    "hashicorp/vault:1.18"
    "quay.io/keycloak/keycloak:26.0"
    "neo4j:5-community"
    "kong:3.9.0-alpine"
    "nginx:1.27.3-alpine"
)
```

**Image Pull and Save (lines 238-251):**
```bash
for image in "${images[@]}"; do
    log_info "Pulling $image..."
    if docker pull "$image"; then
        local filename
        filename=$(echo "$image" | tr '/:' '_')
        log_info "Saving $image to $docker_cache/$filename.tar..."
        docker save "$image" -o "$docker_cache/$filename.tar"
        log_success "Saved $image"
    fi
done
```

**Manifest Generation (lines 253-263):**
```bash
cat > "$docker_cache/manifest.json" << EOF
{
  "created": "$(date -Iseconds)",
  "package_set": "$PACKAGE_SET",
  "images": [
$(printf '    "%s",\n' "${images[@]}" | sed '$ s/,$//')
  ]
}
EOF
```

### AI Model Caching (lines 268-327)

**Model Sets:**
```bash
AI_MODELS_MINIMAL=()

AI_MODELS_FULL=(
    "https://huggingface.co/TheBloke/Mistral-7B-Instruct-v0.2-GGUF/resolve/main/mistral-7b-instruct-v0.2.Q4_K_M.gguf"
)
```

**Download Logic (lines 296-313):**
```bash
for model_url in "${models[@]}"; do
    local filename
    filename=$(basename "$model_url")
    local dest="$models_cache/localai/$filename"

    if [[ -f "$dest" ]]; then
        log_info "Model $filename already exists, skipping"
        continue
    fi

    log_info "Downloading $filename..."
    if curl -L -o "$dest" "$model_url"; then
        log_success "Downloaded $filename"
    else
        log_warn "Failed to download $filename"
    fi
done
```

### Bundle Creation (lines 329-352)

```bash
create_bundle() {
    log_info "Creating offline bundle archive..."

    local bundle_name
    bundle_name="ripple-env-offline-${PACKAGE_SET}-$(date +%Y%m%d).tar.gz"
    local bundle_path="$PROJECT_ROOT/$bundle_name"

    # Create archive (excluding large model files if minimal)
    if [[ "$PACKAGE_SET" == "minimal" ]]; then
        tar -czf "$bundle_path" \
            -C "$(dirname "$CACHE_DIR")" \
            --exclude='*.gguf' \
            --exclude='*.tar' \
            "$(basename "$CACHE_DIR")"
    else
        tar -czf "$bundle_path" \
            -C "$(dirname "$CACHE_DIR")" \
            "$(basename "$CACHE_DIR")"
    fi

    log_success "Offline bundle created: $bundle_path"
    log_info "Bundle size: $(du -h "$bundle_path" | cut -f1)"
}
```

**Exclusions for minimal:**
- `*.gguf` files (AI models, typically 2-8GB each)
- `*.tar` files (Docker images, can be pulled on first run)

---

## Usage Workflow

1. **On internet-connected machine:**
   ```bash
   ./scripts/prepare-offline.sh --full --all
   ```

2. **Transfer to air-gapped machine:**
   ```bash
   scp ripple-env-offline-full-20260113.tar.gz airgapped-host:/tmp/
   ```

3. **On air-gapped machine:**
   ```bash
   tar -xzf /tmp/ripple-env-offline-full-20260113.tar.gz -C /
   export RIPPLE_OFFLINE=1
   export RIPPLE_CACHE_DIR=/var/cache/ripple-env
   nix develop --option substituters "file:///var/cache/ripple-env/nix-store"
   ```

---

## References

- **Main script:** `scripts/prepare-offline.sh` (395 lines)
- **Cache setup:** lines 108-117
- **Nix preparation:** lines 119-158
- **Pixi preparation:** lines 160-194
- **Docker images:** lines 196-267
- **AI models:** lines 268-327
- **Bundle creation:** lines 329-352
- **Related:** `config/offline.yaml` (offline configuration)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 54/60 (90%)
