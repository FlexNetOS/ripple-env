#!/usr/bin/env bash
# =============================================================================
# Container Security Scanner
# =============================================================================
# Scans all Docker images used by the ARIA platform for vulnerabilities
# using Trivy. Generates reports in multiple formats.
#
# Requirements:
#   - trivy (install via nix or https://aquasecurity.github.io/trivy/)
#   - docker
#
# Usage:
#   ./scripts/scan-containers.sh              # Scan all images
#   ./scripts/scan-containers.sh --critical   # Only CRITICAL vulnerabilities
#   ./scripts/scan-containers.sh --report     # Generate HTML report
#   ./scripts/scan-containers.sh <image>      # Scan specific image
# =============================================================================

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
DOCKER_DIR="$PROJECT_ROOT/docker"
REPORT_DIR="$PROJECT_ROOT/reports/security"
SEVERITY="CRITICAL,HIGH"
OUTPUT_FORMAT="table"
GENERATE_REPORT=false

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

show_help() {
    cat << EOF
Container Security Scanner for ARIA Platform

Usage: $0 [OPTIONS] [IMAGE]

Options:
    --critical      Only show CRITICAL vulnerabilities
    --high          Show HIGH and CRITICAL vulnerabilities (default)
    --all           Show all severities
    --report        Generate HTML report in reports/security/
    --json          Output in JSON format
    --sarif         Output in SARIF format (for GitHub Security)
    --list          List all images to be scanned
    --help          Show this help message

Examples:
    $0                              # Scan all images, HIGH+CRITICAL
    $0 --critical                   # Only CRITICAL vulnerabilities
    $0 --report                     # Generate HTML report
    $0 hashicorp/vault:1.18         # Scan specific image
    $0 --json > results.json        # Export as JSON

EOF
}

check_requirements() {
    if ! command -v trivy &>/dev/null; then
        log_error "trivy is not installed"
        log_info "Install with: nix develop or visit https://aquasecurity.github.io/trivy/"
        exit 1
    fi

    if ! command -v docker &>/dev/null; then
        log_error "docker is not installed"
        exit 1
    fi
}

extract_images() {
    # Extract unique images from all docker-compose files
    grep -h "image:" "$DOCKER_DIR"/docker-compose*.yml 2>/dev/null | \
        sed 's/.*image: //' | \
        sed 's/"//g' | \
        sed "s/'//g" | \
        grep -v '^\${' | \
        grep -v '^#' | \
        sort -u
}

list_images() {
    log_info "Docker images used by ARIA platform:"
    echo ""
    extract_images | while read -r image; do
        echo "  - $image"
    done
    echo ""
    log_info "Total: $(extract_images | wc -l) unique images"
}

scan_image() {
    local image=$1
    local format=${2:-table}
    local output_file=${3:-}

    log_info "Scanning: $image"

    local trivy_args=(
        "image"
        "--severity" "$SEVERITY"
        "--ignore-unfixed"
        "--format" "$format"
    )

    if [[ -n "$output_file" ]]; then
        trivy_args+=("--output" "$output_file")
    fi

    trivy_args+=("$image")

    if ! trivy "${trivy_args[@]}"; then
        log_warn "Scan completed with findings for: $image"
        return 1
    fi

    return 0
}

scan_all_images() {
    local failed=0
    local scanned=0
    local total
    total=$(extract_images | wc -l)

    log_info "Scanning $total Docker images..."
    echo ""

    while read -r image; do
        ((scanned++))
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "[$scanned/$total] $image"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

        # Pull image first
        if ! docker pull "$image" 2>/dev/null; then
            log_warn "Could not pull image: $image (may be local-only)"
        fi

        if ! scan_image "$image" "$OUTPUT_FORMAT"; then
            ((failed++))
        fi

        echo ""
    done < <(extract_images)

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "SCAN SUMMARY"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Total images scanned: $scanned"
    echo "Images with vulnerabilities: $failed"
    echo "Severity filter: $SEVERITY"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

    if [[ $failed -gt 0 ]]; then
        log_warn "$failed image(s) have $SEVERITY vulnerabilities"
        return 1
    else
        log_success "No $SEVERITY vulnerabilities found"
        return 0
    fi
}

generate_report() {
    mkdir -p "$REPORT_DIR"
    local timestamp
    timestamp=$(date +%Y%m%d-%H%M%S)
    local report_file="$REPORT_DIR/container-scan-$timestamp.html"

    log_info "Generating security report..."

    # Create HTML report header
    cat > "$report_file" << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <title>ARIA Container Security Report</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 40px; }
        h1 { color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px; }
        h2 { color: #555; margin-top: 30px; }
        .summary { background: #f8f9fa; padding: 20px; border-radius: 8px; margin: 20px 0; }
        .critical { color: #dc3545; font-weight: bold; }
        .high { color: #fd7e14; font-weight: bold; }
        .medium { color: #ffc107; }
        .low { color: #28a745; }
        table { border-collapse: collapse; width: 100%; margin: 20px 0; }
        th, td { border: 1px solid #ddd; padding: 12px; text-align: left; }
        th { background-color: #007bff; color: white; }
        tr:nth-child(even) { background-color: #f8f9fa; }
        .timestamp { color: #6c757d; font-size: 0.9em; }
        pre { background: #f4f4f4; padding: 15px; border-radius: 4px; overflow-x: auto; }
    </style>
</head>
<body>
    <h1>🔒 ARIA Container Security Report</h1>
EOF

    echo "    <p class=\"timestamp\">Generated: $(date)</p>" >> "$report_file"
    echo "    <div class=\"summary\">" >> "$report_file"
    echo "        <h2>Summary</h2>" >> "$report_file"
    echo "        <p>Severity filter: <strong>$SEVERITY</strong></p>" >> "$report_file"
    echo "        <p>Total images: <strong>$(extract_images | wc -l)</strong></p>" >> "$report_file"
    echo "    </div>" >> "$report_file"

    echo "    <h2>Scan Results</h2>" >> "$report_file"

    # Scan each image and append to report
    while read -r image; do
        echo "    <h3>$image</h3>" >> "$report_file"
        echo "    <pre>" >> "$report_file"

        # Pull and scan
        docker pull "$image" 2>/dev/null || true
        trivy image --severity "$SEVERITY" --ignore-unfixed "$image" 2>&1 | \
            sed 's/</\&lt;/g; s/>/\&gt;/g' >> "$report_file" || true

        echo "    </pre>" >> "$report_file"
    done < <(extract_images)

    # Close HTML
    cat >> "$report_file" << 'EOF'
    <hr>
    <p class="timestamp">Report generated by ARIA Container Security Scanner</p>
</body>
</html>
EOF

    log_success "Report generated: $report_file"

    # Also generate JSON summary
    local json_file="$REPORT_DIR/container-scan-$timestamp.json"
    log_info "Generating JSON summary..."

    echo "{" > "$json_file"
    echo "  \"timestamp\": \"$(date -Iseconds)\"," >> "$json_file"
    echo "  \"severity_filter\": \"$SEVERITY\"," >> "$json_file"
    echo "  \"images\": [" >> "$json_file"

    local first=true
    while read -r image; do
        if [[ "$first" != "true" ]]; then
            echo "," >> "$json_file"
        fi
        first=false
        echo -n "    \"$image\"" >> "$json_file"
    done < <(extract_images)

    echo "" >> "$json_file"
    echo "  ]" >> "$json_file"
    echo "}" >> "$json_file"

    log_success "JSON summary: $json_file"
}

# Main
main() {
    check_requirements

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                show_help
                exit 0
                ;;
            --critical)
                SEVERITY="CRITICAL"
                shift
                ;;
            --high)
                SEVERITY="CRITICAL,HIGH"
                shift
                ;;
            --all)
                SEVERITY="CRITICAL,HIGH,MEDIUM,LOW"
                shift
                ;;
            --report)
                GENERATE_REPORT=true
                shift
                ;;
            --json)
                OUTPUT_FORMAT="json"
                shift
                ;;
            --sarif)
                OUTPUT_FORMAT="sarif"
                shift
                ;;
            --list)
                list_images
                exit 0
                ;;
            -*)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
            *)
                # Scan specific image
                scan_image "$1" "$OUTPUT_FORMAT"
                exit $?
                ;;
        esac
    done

    # Generate report or scan all
    if [[ "$GENERATE_REPORT" == "true" ]]; then
        generate_report
    else
        scan_all_images
    fi
}

main "$@"
