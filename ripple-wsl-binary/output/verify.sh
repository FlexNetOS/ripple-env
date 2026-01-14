#!/usr/bin/env bash
# FlexNetOS Verification Script

set -euo pipefail

echo "========================================"
echo " FlexNetOS Verification"
echo "========================================"

# Check if running inside WSL
if [[ -z "${WSL_DISTRO_NAME:-}" ]]; then
    echo "Error: This script must be run inside WSL"
    exit 1
fi

# Check if FlexNetOS
if [[ "${WSL_DISTRO_NAME}" != *"FlexNetOS"* ]]; then
    echo "Warning: This doesn't appear to be FlexNetOS"
fi

# Test basic functionality
echo "Testing basic functionality..."

# Check systemd
if systemctl --version >/dev/null 2>&1; then
    echo "✓ Systemd is working"
else
    echo "✗ Systemd not working"
    exit 1
fi

# Check Docker
if docker --version >/dev/null 2>&1; then
    echo "✓ Docker is installed"
else
    echo "✗ Docker not installed"
    exit 1
fi

# Check user
if [[ "$(whoami)" == "flexnet" ]]; then
    echo "✓ Default user working"
else
    echo "✗ Default user not working (current: $(whoami))"
fi

# Check FlexNetOS files
if [[ -f /etc/flexnetos/config.yaml ]]; then
    echo "✓ FlexNetOS configuration present"
else
    echo "✗ FlexNetOS configuration missing"
fi

# Check services
services=("docker" "flexnetos-init")
for service in "${services[@]}"; do
    if systemctl is-active "$service" >/dev/null 2>&1; then
        echo "✓ Service '$service' is running"
    else
        echo "✗ Service '$service' is not running"
    fi
done

# Check health
if [[ -f /var/lib/flexnetos/health.json ]]; then
    echo "✓ Health monitoring active"
    echo "Health status:"
    cat /var/lib/flexnetos/health.json
else
    echo "✗ Health monitoring not active"
fi

echo "========================================"
echo " Verification completed!"
echo "========================================"
