#!/usr/bin/env bash
# FlexNetOS Security Audit Script

set -euo pipefail

echo "Running FlexNetOS security audit..."

# Run Trivy vulnerability scan
echo "Scanning for vulnerabilities..."
trivy fs --security-checks vuln,config .

# Check for secrets
echo "Checking for exposed secrets..."
grep -r "password\|secret\|key\|token" --include="*.yml" --include="*.yaml" --include="*.env" . || true

echo "Security audit completed!"
