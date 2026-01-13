#!/usr/bin/env bash
# FlexNetOS Configuration Validation Script

set -euo pipefail

echo "Validating FlexNetOS configurations..."

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo "WARNING: Docker Compose not found; skipping compose validation"
fi

# Validate Docker Compose files
if [ ${#COMPOSE[@]} -gt 0 ]; then
    shopt -s nullglob
    files=(
        docker-compose*.yml
        docker/*.yml
        docker/docker-compose*.yml
    )
    shopt -u nullglob

    for file in "${files[@]}"; do
        # Only validate existing files (nullglob should already handle this).
        [ -f "$file" ] || continue
        echo "Validating $file..."
        "${COMPOSE[@]}" -f "$file" config > /dev/null
    done
fi

# Validate Nix configuration
echo "Validating Nix configuration..."
if command -v nix >/dev/null 2>&1; then
    nix flake check
else
    echo "WARNING: nix not found; skipping 'nix flake check'"
fi

echo "All configurations validated successfully!"
