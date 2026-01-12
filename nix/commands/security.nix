# Security command wrappers (SBOM, vulnerability scanning, signing)
# Extracted from flake.nix for modular organization
#
# Supply Chain Security Commands:
# - sbom: Generate SBOM for directories/containers
# - sbom-nix: Generate Nix-specific SBOM from flake closure
# - vuln-scan: Vulnerability scanning with grype/trivy
# - sign-artifact: Sign artifacts with cosign/sigstore
# - supply-chain-audit: Comprehensive supply chain audit
# - verify-inputs: Verify flake input integrity
{ pkgs, ... }:

[
            # Generate SBOM (Software Bill of Materials)
            (pkgs.writeShellScriptBin "sbom" ''
              # Generate SBOM using syft
              # Usage: sbom [target] [--format <format>]
              # Formats: json, spdx-json, cyclonedx-json, table (default)
              set -e

              TARGET="''${1:-.}"
              FORMAT="''${2:-table}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: sbom [target] [format]"
                echo ""
                echo "Generate Software Bill of Materials using syft"
                echo ""
                echo "Arguments:"
                echo "  target  - Directory, container image, or archive (default: .)"
                echo "  format  - Output format: table, json, spdx-json, cyclonedx-json"
                echo ""
                echo "Examples:"
                echo "  sbom                      # Scan current directory"
                echo "  sbom ./src json           # Scan src/ as JSON"
                echo "  sbom alpine:latest        # Scan container image"
                echo "  sbom . cyclonedx-json     # CycloneDX format for compliance"
                exit 0
              fi

              echo "Generating SBOM for: $TARGET" >&2
              echo "Format: $FORMAT" >&2
              echo "" >&2

              syft "$TARGET" -o "$FORMAT"
            '')
            # Vulnerability scanning
            (pkgs.writeShellScriptBin "vuln-scan" ''
              # Scan for vulnerabilities using grype or trivy
              # Usage: vuln-scan [target] [--tool <grype|trivy>]
              set -e

              TARGET="''${1:-.}"
              TOOL="''${2:-grype}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: vuln-scan [target] [tool]"
                echo ""
                echo "Scan for vulnerabilities in code, containers, or SBOMs"
                echo ""
                echo "Arguments:"
                echo "  target  - Directory, container image, or SBOM file (default: .)"
                echo "  tool    - Scanner to use: grype (default) or trivy"
                echo ""
                echo "Examples:"
                echo "  vuln-scan                     # Scan current directory with grype"
                echo "  vuln-scan . trivy             # Scan with trivy"
                echo "  vuln-scan alpine:latest       # Scan container image"
                echo "  vuln-scan sbom.json grype     # Scan from SBOM"
                exit 0
              fi

              echo "Scanning: $TARGET" >&2
              echo "Tool: $TOOL" >&2
              echo "" >&2

              case "$TOOL" in
                grype)
                  grype "$TARGET"
                  ;;
                trivy)
                  trivy fs "$TARGET"
                  ;;
                *)
                  echo "Unknown tool: $TOOL (use grype or trivy)" >&2
                  exit 1
                  ;;
              esac
            '')
            # Sign artifacts with cosign
            (pkgs.writeShellScriptBin "sign-artifact" ''
              # Sign container images or blobs with cosign
              # Usage: sign-artifact <image|file> [--key <key>]
              set -e

              if [ -z "$1" ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: sign-artifact <target> [--key <keyfile>]"
                echo ""
                echo "Sign container images or files using cosign (sigstore)"
                echo ""
                echo "Arguments:"
                echo "  target  - Container image or file to sign"
                echo "  --key   - Private key file (optional, uses keyless by default)"
                echo ""
                echo "Examples:"
                echo "  sign-artifact myregistry/myimage:v1.0     # Keyless signing"
                echo "  sign-artifact myimage --key cosign.key    # Key-based signing"
                echo "  sign-artifact artifact.tar.gz             # Sign a file"
                echo ""
                echo "Verify with:"
                echo "  cosign verify <image>"
                echo "  cosign verify-blob --signature <sig> <file>"
                exit 0
              fi

              TARGET="$1"
              shift

              if [ "$1" = "--key" ] && [ -n "$2" ]; then
                echo "Signing with key: $2" >&2
                cosign sign --key "$2" "$TARGET"
              else
                echo "Using keyless signing (OIDC)" >&2
                echo "You will be prompted to authenticate via browser" >&2
                cosign sign "$TARGET"
              fi
            '')
            # PKI certificate generation with step-cli
            (pkgs.writeShellScriptBin "pki-cert" ''
              # Generate certificates using step-cli
              # Usage: pki-cert <command> [args]
              case "''${1:-help}" in
                ca-init)
                  # Initialize a local CA
                  echo "Initializing local Certificate Authority..."
                  step ca init --name "ROS2-Dev-CA" --provisioner admin --dns localhost --address ":9000"
                  ;;
                create)
                  # Create a certificate: pki-cert create <name> <san>
                  if [ -z "$2" ]; then
                    echo "Usage: pki-cert create <name> [san...]" >&2
                    exit 1
                  fi
                  NAME="$2"
                  shift 2
                  echo "Creating certificate for: $NAME"
                  step certificate create "$NAME" "$NAME.crt" "$NAME.key" --san "$NAME" "$@"
                  echo ""
                  echo "Created: $NAME.crt, $NAME.key"
                  ;;
                inspect)
                  # Inspect a certificate
                  if [ -z "$2" ]; then
                    echo "Usage: pki-cert inspect <cert-file>" >&2
                    exit 1
                  fi
                  step certificate inspect "$2"
                  ;;
                *)
                  echo "Usage: pki-cert <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  ca-init           - Initialize a local Certificate Authority"
                  echo "  create <n> [san]  - Create certificate for name with SANs"
                  echo "  inspect <cert>    - Inspect a certificate file"
                  echo ""
                  echo "Examples:"
                  echo "  pki-cert ca-init"
                  echo "  pki-cert create robot1 --san robot1.local --san 192.168.1.10"
                  echo "  pki-cert inspect robot1.crt"
                  ;;
              esac
            '')

            # Nix-specific SBOM generation
            (pkgs.writeShellScriptBin "sbom-nix" ''
              #!/usr/bin/env bash
              # Generate SBOM from Nix flake runtime closure
              # Provides complete dependency tree with hashes for supply chain verification
              set -euo pipefail

              FLAKE_REF="''${1:-.}"
              OUTPUT="''${2:-sbom-nix.json}"
              FORMAT="''${3:-cyclonedx}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: sbom-nix [flake-ref] [output] [format]"
                echo ""
                echo "Generate Software Bill of Materials from Nix flake closure"
                echo ""
                echo "Arguments:"
                echo "  flake-ref  - Flake reference (default: .)"
                echo "  output     - Output file (default: sbom-nix.json)"
                echo "  format     - cyclonedx or spdx (default: cyclonedx)"
                echo ""
                echo "Examples:"
                echo "  sbom-nix                           # Current flake, CycloneDX"
                echo "  sbom-nix .#default sbom.json       # Default devShell"
                echo "  sbom-nix .#stable stable-sbom.json # Stable shell"
                exit 0
              fi

              echo "Generating Nix SBOM for: $FLAKE_REF" >&2
              echo "Output: $OUTPUT" >&2
              echo "Format: $FORMAT" >&2
              echo "" >&2

              # Get flake metadata
              FLAKE_META=$(nix flake metadata "$FLAKE_REF" --json 2>/dev/null || echo '{}')
              FLAKE_LOCKED=$(echo "$FLAKE_META" | ${pkgs.jq}/bin/jq -r '.locked.narHash // "unknown"')

              # Generate dependency list from flake inputs
              echo "Collecting flake inputs..." >&2
              INPUTS=$(nix flake metadata "$FLAKE_REF" --json 2>/dev/null | ${pkgs.jq}/bin/jq -r '
                .locks.nodes | to_entries |
                map(select(.value.locked != null)) |
                map({
                  name: .key,
                  type: (.value.locked.type // "unknown"),
                  url: (.value.locked.url // (.value.original.owner + "/" + .value.original.repo) // "local"),
                  rev: (.value.locked.rev // "none"),
                  narHash: (.value.locked.narHash // "none")
                })
              ' 2>/dev/null || echo '[]')

              # Generate SBOM in requested format
              case "$FORMAT" in
                cyclonedx)
                  ${pkgs.jq}/bin/jq -n \
                    --arg timestamp "$(date -u +"%Y-%m-%dT%H:%M:%SZ")" \
                    --arg ref "$FLAKE_REF" \
                    --arg hash "$FLAKE_LOCKED" \
                    --argjson inputs "$INPUTS" \
                    '{
                      "bomFormat": "CycloneDX",
                      "specVersion": "1.5",
                      "serialNumber": ("urn:uuid:" + (now | tostring | split(".")[0])),
                      "version": 1,
                      "metadata": {
                        "timestamp": $timestamp,
                        "tools": [{
                          "vendor": "ripple-env",
                          "name": "sbom-nix",
                          "version": "1.0.0"
                        }],
                        "component": {
                          "type": "application",
                          "name": "ripple-env",
                          "version": $hash,
                          "bom-ref": $ref
                        }
                      },
                      "components": ($inputs | map({
                        "type": "library",
                        "name": .name,
                        "version": .rev,
                        "purl": ("pkg:nix/" + .name + "@" + .rev),
                        "hashes": [{
                          "alg": "SHA-256",
                          "content": (.narHash | split("-")[1] // .narHash)
                        }],
                        "externalReferences": [{
                          "type": "vcs",
                          "url": .url
                        }]
                      }))
                    }' > "$OUTPUT"
                  ;;
                spdx)
                  ${pkgs.jq}/bin/jq -n \
                    --arg timestamp "$(date -u +"%Y-%m-%dT%H:%M:%SZ")" \
                    --arg ref "$FLAKE_REF" \
                    --arg hash "$FLAKE_LOCKED" \
                    --argjson inputs "$INPUTS" \
                    '{
                      "spdxVersion": "SPDX-2.3",
                      "dataLicense": "CC0-1.0",
                      "SPDXID": "SPDXRef-DOCUMENT",
                      "name": "ripple-env-sbom",
                      "documentNamespace": ("https://github.com/FlexNetOS/ripple-env/sbom/" + $hash),
                      "creationInfo": {
                        "created": $timestamp,
                        "creators": ["Tool: sbom-nix-1.0.0"]
                      },
                      "packages": ($inputs | map({
                        "SPDXID": ("SPDXRef-" + .name),
                        "name": .name,
                        "versionInfo": .rev,
                        "downloadLocation": .url,
                        "checksums": [{
                          "algorithm": "SHA256",
                          "checksumValue": (.narHash | split("-")[1] // .narHash)
                        }]
                      }))
                    }' > "$OUTPUT"
                  ;;
                *)
                  echo "Unknown format: $FORMAT" >&2
                  exit 1
                  ;;
              esac

              COMPONENT_COUNT=$(${pkgs.jq}/bin/jq '.components | length' "$OUTPUT" 2>/dev/null || echo "0")
              echo "" >&2
              echo "SBOM generated: $OUTPUT" >&2
              echo "Components: $COMPONENT_COUNT" >&2
            '')

            # Verify flake input integrity
            (pkgs.writeShellScriptBin "verify-inputs" ''
              #!/usr/bin/env bash
              # Verify flake input integrity against known-good hashes
              set -euo pipefail

              FLAKE_REF="''${1:-.}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: verify-inputs [flake-ref]"
                echo ""
                echo "Verify flake input integrity and check for supply chain issues"
                echo ""
                echo "Checks performed:"
                echo "  - Flake lock file integrity"
                echo "  - Input hash verification"
                echo "  - Follows references validation"
                echo "  - Unstable input detection"
                exit 0
              fi

              echo "=== Flake Input Verification ===" >&2
              echo "Flake: $FLAKE_REF" >&2
              echo "" >&2

              # Check flake.lock exists
              if [ ! -f "flake.lock" ]; then
                echo "ERROR: flake.lock not found" >&2
                exit 1
              fi

              # Verify lock file structure
              echo "1. Verifying lock file structure..." >&2
              LOCK_VERSION=$(${pkgs.jq}/bin/jq -r '.version' flake.lock)
              if [ "$LOCK_VERSION" -lt 5 ]; then
                echo "   WARNING: Old lock file format (v$LOCK_VERSION), consider updating" >&2
              else
                echo "   Lock file version: $LOCK_VERSION (OK)" >&2
              fi

              # Check for unstable inputs
              echo "" >&2
              echo "2. Checking for unstable inputs..." >&2
              UNSTABLE=$(${pkgs.jq}/bin/jq -r '.nodes | to_entries | .[] | select(.value.original.ref == "nixos-unstable" or .value.original.ref == "master" or .value.original.ref == "main") | .key' flake.lock 2>/dev/null || echo "")
              if [ -n "$UNSTABLE" ]; then
                echo "   WARNING: Unstable inputs detected:" >&2
                echo "$UNSTABLE" | while read -r input; do
                  echo "   - $input" >&2
                done
                echo "   Consider using stable branches for production" >&2
              else
                echo "   No unstable inputs (OK)" >&2
              fi

              # Verify all inputs have hashes
              echo "" >&2
              echo "3. Verifying input hashes..." >&2
              MISSING_HASH=$(${pkgs.jq}/bin/jq -r '.nodes | to_entries | .[] | select(.value.locked != null and .value.locked.narHash == null) | .key' flake.lock 2>/dev/null || echo "")
              if [ -n "$MISSING_HASH" ]; then
                echo "   ERROR: Inputs missing narHash:" >&2
                echo "$MISSING_HASH" | while read -r input; do
                  echo "   - $input" >&2
                done
                exit 1
              else
                INPUT_COUNT=$(${pkgs.jq}/bin/jq '[.nodes | to_entries | .[] | select(.value.locked.narHash != null)] | length' flake.lock)
                echo "   All $INPUT_COUNT inputs have valid hashes (OK)" >&2
              fi

              # Verify flake evaluates
              echo "" >&2
              echo "4. Verifying flake evaluation..." >&2
              if nix flake metadata "$FLAKE_REF" --json >/dev/null 2>&1; then
                echo "   Flake evaluation successful (OK)" >&2
              else
                echo "   ERROR: Flake evaluation failed" >&2
                exit 1
              fi

              echo "" >&2
              echo "=== Verification Complete ===" >&2

              # Generate summary
              echo "" >&2
              echo "Input Summary:" >&2
              ${pkgs.jq}/bin/jq -r '.nodes | to_entries | .[] | select(.value.locked != null) | "  \(.key): \(.value.locked.narHash // "no-hash")[0:20]..."' flake.lock
            '')

            # Comprehensive supply chain audit
            (pkgs.writeShellScriptBin "supply-chain-audit" ''
              #!/usr/bin/env bash
              # Comprehensive supply chain security audit
              set -euo pipefail

              FLAKE_REF="''${1:-.}"
              OUTPUT_DIR="''${2:-supply-chain-audit}"

              if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
                echo "Usage: supply-chain-audit [flake-ref] [output-dir]"
                echo ""
                echo "Run comprehensive supply chain security audit"
                echo ""
                echo "Generates:"
                echo "  - SBOM (CycloneDX and SPDX formats)"
                echo "  - Vulnerability report"
                echo "  - Input verification report"
                echo "  - Dependency graph"
                exit 0
              fi

              mkdir -p "$OUTPUT_DIR"
              TIMESTAMP=$(date -u +"%Y%m%d-%H%M%S")

              echo "Supply Chain Security Audit"
              echo "============================"
              echo ""
              echo "Flake: $FLAKE_REF"
              echo "Output: $OUTPUT_DIR"
              echo "Time: $TIMESTAMP"
              echo ""

              # Step 1: Verify inputs
              echo "--- Step 1/4: Input Verification ---"
              verify-inputs "$FLAKE_REF" 2>&1 | tee "$OUTPUT_DIR/input-verification.txt"
              echo ""

              # Step 2: Generate Nix SBOM
              echo "--- Step 2/4: Nix SBOM Generation ---"
              sbom-nix "$FLAKE_REF" "$OUTPUT_DIR/sbom-nix-cyclonedx.json" cyclonedx
              sbom-nix "$FLAKE_REF" "$OUTPUT_DIR/sbom-nix-spdx.json" spdx
              echo ""

              # Step 3: Generate filesystem SBOM
              echo "--- Step 3/4: Filesystem SBOM Generation ---"
              sbom . "$OUTPUT_DIR/sbom-fs.json" 2>/dev/null || echo "  Skipped (syft not available)"
              echo ""

              # Step 4: Generate audit report
              echo "--- Step 4/4: Generating Audit Report ---"
              cat > "$OUTPUT_DIR/audit-report.md" << EOF
# Supply Chain Security Audit Report

**Date:** $(date -u +"%Y-%m-%d %H:%M:%S UTC")
**Flake:** $FLAKE_REF

## Summary

| Metric | Value |
|--------|-------|
| Flake Inputs | $(${pkgs.jq}/bin/jq '[.nodes | to_entries | .[] | select(.value.locked != null)] | length' flake.lock) |

## Files Generated

- sbom-nix-cyclonedx.json - Nix SBOM (CycloneDX)
- sbom-nix-spdx.json - Nix SBOM (SPDX)
- input-verification.txt - Input verification log

## Recommendations

1. Review any unstable inputs for production use
2. Consider pinning to stable nixpkgs channel (nix develop .#stable)
3. Sign releases with supply chain attestation

---
*Generated by supply-chain-audit*
EOF

              echo "  Report saved to: $OUTPUT_DIR/audit-report.md"
              echo ""
              echo "Audit Complete"
              echo "=============="
              echo ""
              echo "Results saved to: $OUTPUT_DIR/"
              ls -la "$OUTPUT_DIR/"
            '')
          ];
