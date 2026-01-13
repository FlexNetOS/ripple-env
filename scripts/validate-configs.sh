#!/usr/bin/env bash
# FlexNetOS Configuration Validation Script

set -euo pipefail

echo "Validating FlexNetOS configurations..."

# Validate Docker Compose files
for file in docker/*.yml; do
    echo "Validating $file..."
    docker-compose -f "$file" config > /dev/null
done

# Validate Nix configuration
echo "Validating Nix configuration..."
nix flake check

echo "All configurations validated successfully!"
