# Security command wrappers (SBOM, vulnerability scanning, signing)
# Extracted from flake.nix for modular organization
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
          ];
