# SBOM (Software Bill of Materials) commands
# Generate, validate, and analyze SBOMs for supply chain security
{ pkgs, ... }:

with pkgs; [
  # Generate SBOM with syft
  (writeShellScriptBin "sbom-generate" ''
    #!/usr/bin/env bash
    set -euo pipefail

    TARGET="''${1:-.}"
    OUTPUT="''${2:-sbom.json}"
    FORMAT="''${3:-cyclonedx-json}"

    echo "Generating SBOM for: $TARGET"
    echo "Output: $OUTPUT"
    echo "Format: $FORMAT"
    echo ""

    ${syft}/bin/syft "$TARGET" -o "$FORMAT" > "$OUTPUT"

    echo ""
    echo "SBOM generated: $OUTPUT"
    echo "Package count: $(jq '.components | length' "$OUTPUT" 2>/dev/null || echo 'N/A')"
  '')

  # Scan SBOM for vulnerabilities with grype
  (writeShellScriptBin "sbom-scan" ''
    #!/usr/bin/env bash
    set -euo pipefail

    SBOM="''${1:-sbom.json}"

    if [ ! -f "$SBOM" ]; then
      echo "SBOM file not found: $SBOM"
      echo "Generate one first: sbom-generate"
      exit 1
    fi

    echo "Scanning SBOM for vulnerabilities: $SBOM"
    echo ""

    ${grype}/bin/grype sbom:"$SBOM" --output table
  '')

  # Sign artifact with cosign
  (writeShellScriptBin "sbom-sign" ''
    #!/usr/bin/env bash
    set -euo pipefail

    ARTIFACT="''${1:-}"
    if [ -z "$ARTIFACT" ]; then
      echo "Usage: sbom-sign <artifact>"
      echo ""
      echo "Signs an artifact (container image, binary, etc.) with cosign"
      exit 1
    fi

    echo "Signing: $ARTIFACT"
    ${cosign}/bin/cosign sign "$ARTIFACT"
  '')

  # Verify signed artifact
  (writeShellScriptBin "sbom-verify" ''
    #!/usr/bin/env bash
    set -euo pipefail

    ARTIFACT="''${1:-}"
    if [ -z "$ARTIFACT" ]; then
      echo "Usage: sbom-verify <artifact>"
      exit 1
    fi

    echo "Verifying: $ARTIFACT"
    ${cosign}/bin/cosign verify "$ARTIFACT"
  '')

  # Full supply chain audit
  (writeShellScriptBin "sbom-audit" ''
    #!/usr/bin/env bash
    set -euo pipefail

    TARGET="''${1:-.}"
    SBOM_FILE="sbom-$(date +%Y%m%d-%H%M%S).json"

    echo "=== Supply Chain Security Audit ==="
    echo "Target: $TARGET"
    echo "Date: $(date -u +"%Y-%m-%d %H:%M:%S UTC")"
    echo ""

    # Generate SBOM
    echo "1. Generating SBOM..."
    ${syft}/bin/syft "$TARGET" -o cyclonedx-json > "$SBOM_FILE"
    PACKAGE_COUNT=$(jq '.components | length' "$SBOM_FILE" 2>/dev/null || echo '0')
    echo "   Packages found: $PACKAGE_COUNT"
    echo ""

    # Scan for vulnerabilities
    echo "2. Scanning for vulnerabilities..."
    VULN_OUTPUT=$(${grype}/bin/grype sbom:"$SBOM_FILE" --output json 2>/dev/null || echo '{"matches":[]}')
    CRITICAL=$(echo "$VULN_OUTPUT" | jq '[.matches[] | select(.vulnerability.severity == "Critical")] | length' 2>/dev/null || echo '0')
    HIGH=$(echo "$VULN_OUTPUT" | jq '[.matches[] | select(.vulnerability.severity == "High")] | length' 2>/dev/null || echo '0')
    MEDIUM=$(echo "$VULN_OUTPUT" | jq '[.matches[] | select(.vulnerability.severity == "Medium")] | length' 2>/dev/null || echo '0')
    LOW=$(echo "$VULN_OUTPUT" | jq '[.matches[] | select(.vulnerability.severity == "Low")] | length' 2>/dev/null || echo '0')

    echo "   Critical: $CRITICAL"
    echo "   High: $HIGH"
    echo "   Medium: $MEDIUM"
    echo "   Low: $LOW"
    echo ""

    # Container scan if applicable
    if command -v ${trivy}/bin/trivy &> /dev/null && [ -f "Dockerfile" ]; then
      echo "3. Container security scan..."
      ${trivy}/bin/trivy fs --scanners vuln,secret,config . --severity HIGH,CRITICAL --quiet || true
      echo ""
    fi

    echo "=== Summary ==="
    echo "SBOM saved to: $SBOM_FILE"
    echo "Total packages: $PACKAGE_COUNT"
    echo "Critical/High vulnerabilities: $CRITICAL/$HIGH"

    if [ "$CRITICAL" -gt 0 ]; then
      echo ""
      echo "WARNING: Critical vulnerabilities found!"
      exit 1
    fi
  '')
]
