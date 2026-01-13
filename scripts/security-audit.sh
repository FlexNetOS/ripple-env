#!/usr/bin/env bash
# FlexNetOS Security Audit Script

set -euo pipefail

echo "Running FlexNetOS security audit..."

# Run Trivy vulnerability scan
echo "Scanning for vulnerabilities..."
trivy fs --security-checks vuln,config .

# Check for secrets using dedicated tools
echo "Checking for exposed secrets..."

# Try to use gitleaks if available
if command -v gitleaks &> /dev/null; then
    echo "Running gitleaks scan..."
    gitleaks detect --source . --verbose --no-git || true
# Fall back to detect-secrets if available
elif command -v detect-secrets &> /dev/null; then
    echo "Running detect-secrets scan..."
    detect-secrets scan --baseline .secrets.baseline || true
# Fall back to trufflehog if available
elif command -v trufflehog &> /dev/null; then
    echo "Running trufflehog scan..."
    trufflehog filesystem . --json || true
else
    echo "Warning: No dedicated secret scanning tool found (gitleaks, detect-secrets, trufflehog)"
    echo "Falling back to basic grep pattern matching..."
    grep -r "password\|secret\|key\|token" --include="*.yml" --include="*.yaml" --include="*.env" . || true
fi

echo "Security audit completed!"
