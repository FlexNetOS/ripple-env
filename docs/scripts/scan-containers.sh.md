# Script Contract: scan-containers.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/scan-containers.sh`

---

## Purpose

Container security scanner that uses Trivy to scan all Docker images used by the ARIA platform for vulnerabilities. Extracts images from Docker Compose files, pulls them, scans for CVEs, and generates reports in multiple formats (table, JSON, SARIF, HTML). Supports severity filtering (CRITICAL, HIGH, MEDIUM, LOW) and can scan specific images or all images used by the platform.

---

## Invocation

```bash
./scripts/scan-containers.sh [OPTIONS] [IMAGE]
```

**Options:**
- `--critical` - Only show CRITICAL vulnerabilities
- `--high` - Show HIGH and CRITICAL vulnerabilities (default)
- `--all` - Show all severities (CRITICAL, HIGH, MEDIUM, LOW)
- `--report` - Generate HTML report in `reports/security/`
- `--json` - Output in JSON format
- `--sarif` - Output in SARIF format (for GitHub Security)
- `--list` - List all images to be scanned
- `--help` - Show help message

**Examples:**
```bash
./scripts/scan-containers.sh                              # Scan all, HIGH+CRITICAL
./scripts/scan-containers.sh --critical                   # Only CRITICAL
./scripts/scan-containers.sh --report                     # Generate HTML report
./scripts/scan-containers.sh hashicorp/vault:1.18         # Scan specific image
./scripts/scan-containers.sh --json > results.json        # Export as JSON
./scripts/scan-containers.sh --sarif > report.sarif       # GitHub Security format
./scripts/scan-containers.sh --list                       # List all images
```

**Requirements:**
- `trivy` (install via Nix or https://aquasecurity.github.io/trivy/)
- `docker` (for pulling images)
- `jq` (optional, for report generation)

---

## Outputs

**Standard Output (table format - default):**
```
[INFO] Scanning 15 Docker images...

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[1/15] hashicorp/vault:1.18
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[INFO] Scanning: hashicorp/vault:1.18

vault:1.18 (alpine 3.20.3)
==========================
Total: 2 (HIGH: 2, CRITICAL: 0)

┌───────────────┬────────────────┬──────────┬────────┬───────────────────┬───────────────┬──────────────────────────────────────┐
│    Library    │ Vulnerability  │ Severity │ Status │ Installed Version │ Fixed Version │                Title                 │
├───────────────┼────────────────┼──────────┼────────┼───────────────────┼───────────────┼──────────────────────────────────────┤
│ libcrypto3    │ CVE-2024-9143  │ HIGH     │ fixed  │ 3.3.2-r0          │ 3.3.2-r1      │ openssl: Low-level invalid GF(2^m)   │
│ libssl3       │ CVE-2024-9143  │ HIGH     │ fixed  │ 3.3.2-r0          │ 3.3.2-r1      │ openssl: Low-level invalid GF(2^m)   │
└───────────────┴────────────────┴──────────┴────────┴───────────────────┴───────────────┴──────────────────────────────────────┘

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SCAN SUMMARY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Total images scanned: 15
Images with vulnerabilities: 8
Severity filter: CRITICAL,HIGH
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[WARN] 8 image(s) have CRITICAL,HIGH vulnerabilities
```

**Standard Output (clean scan):**
```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
SCAN SUMMARY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Total images scanned: 15
Images with vulnerabilities: 0
Severity filter: CRITICAL,HIGH
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
[SUCCESS] No CRITICAL,HIGH vulnerabilities found
```

**File Outputs (--report):**
- `reports/security/container-scan-YYYYMMDD-HHMMSS.html` - HTML report with styling
- `reports/security/container-scan-YYYYMMDD-HHMMSS.json` - JSON summary

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (no vulnerabilities found at specified severity) |
| `1` | Failure (vulnerabilities found or scan error) |

---

## Side Effects

### Report Files (--report flag, line 175)

**Evidence:**
```bash
generate_report() {
    mkdir -p "$REPORT_DIR"
    local timestamp
    timestamp=$(date +%Y%m%d-%H%M%S)
    local report_file="$REPORT_DIR/container-scan-$timestamp.html"
```

**Creates:**
- `reports/security/container-scan-<timestamp>.html`
- `reports/security/container-scan-<timestamp>.json`

### Docker Image Pulls (line 146, 226)

**Evidence:**
```bash
# Pull image first
if ! docker pull "$image" 2>/dev/null; then
    log_warn "Could not pull image: $image (may be local-only)"
fi
```

**Downloads images** from Docker registries (if not local)

---

## Safety Classification

**🟢 SAFE** - Read-only scanning, no system modifications except report files.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly. Each run creates new timestamped reports.

---

## Image Extraction

**Evidence:** Lines 81-90

```bash
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
```

**Process:**
1. Searches all `docker/docker-compose*.yml` files (line 83)
2. Extracts lines with `image:` (line 83)
3. Removes quotes and `image:` prefix (lines 84-86)
4. Filters out env variable refs (`${VAR}`) and comments (lines 87-88)
5. Returns unique sorted list (line 89)

**Example images found:**
- hashicorp/vault:1.18
- prom/prometheus:latest
- grafana/grafana:latest
- nats:2.10-alpine
- openpolicyagent/opa:latest
- n8nio/n8n:latest
- localai/localai:latest
- kong:3.8

---

## Trivy Scanning

**Evidence:** Lines 102-128

```bash
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
```

**Trivy arguments:**
- `image` - Scan Docker image (line 110)
- `--severity CRITICAL,HIGH` - Filter by severity (line 111, default line 26)
- `--ignore-unfixed` - Ignore vulnerabilities without fixes (line 112)
- `--format table` - Output format (line 113, default line 27)

**Return codes:**
- `0` - No vulnerabilities found
- `1` - Vulnerabilities found (line 124)

---

## Severity Filtering

**Evidence:** Lines 26, 283-294

```bash
# Default
SEVERITY="CRITICAL,HIGH"

# Options
--critical)
    SEVERITY="CRITICAL"
    ;;
--high)
    SEVERITY="CRITICAL,HIGH"
    ;;
--all)
    SEVERITY="CRITICAL,HIGH,MEDIUM,LOW"
    ;;
```

**Severity levels:**
- `CRITICAL` - Remote code execution, privilege escalation
- `HIGH` - Data exposure, authentication bypass
- `MEDIUM` - Information disclosure, DoS
- `LOW` - Minor issues

**Default:** CRITICAL + HIGH (lines 26, 288)

---

## Scan All Images

**Evidence:** Lines 130-172

```bash
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

    # Summary
    if [[ $failed -gt 0 ]]; then
        log_warn "$failed image(s) have $SEVERITY vulnerabilities"
        return 1
    else
        log_success "No $SEVERITY vulnerabilities found"
        return 0
    fi
}
```

**Process:**
1. Counts total images (line 134)
2. Iterates through images with progress (lines 139-155)
3. Pulls each image (line 146)
4. Scans with Trivy (line 150)
5. Tracks failures (line 151)
6. Returns summary (lines 165-171)

---

## HTML Report Generation

**Evidence:** Lines 174-270

### Report Structure (lines 183-207)

```bash
cat > "$report_file" << 'EOF'
<!DOCTYPE html>
<html>
<head>
    <title>ARIA Container Security Report</title>
    <style>
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; margin: 40px; }
        h1 { color: #333; border-bottom: 2px solid #007bff; padding-bottom: 10px; }
        .summary { background: #f8f9fa; padding: 20px; border-radius: 8px; margin: 20px 0; }
        .critical { color: #dc3545; font-weight: bold; }
        .high { color: #fd7e14; font-weight: bold; }
        table { border-collapse: collapse; width: 100%; margin: 20px 0; }
        th { background-color: #007bff; color: white; }
    </style>
</head>
<body>
    <h1>🔒 ARIA Container Security Report</h1>
EOF
```

**Styling:**
- Modern sans-serif font (line 189)
- Color-coded severity (lines 193-196)
- Bootstrap-inspired colors (blue primary, red critical, orange high)
- Responsive tables (lines 197-199)

### Report Content (lines 209-231)

```bash
{
    echo "    <p class=\"timestamp\">Generated: $(date)</p>"
    echo "    <div class=\"summary\">"
    echo "        <h2>Summary</h2>"
    echo "        <p>Severity filter: <strong>$SEVERITY</strong></p>"
    echo "        <p>Total images: <strong>$(extract_images | wc -l)</strong></p>"
    echo "    </div>"
} >> "$report_file"

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
```

**HTML escaping:** `sed 's/</\&lt;/g; s/>/\&gt;/g'` (line 228) prevents XSS

### JSON Summary (lines 243-269)

```bash
local json_file="$REPORT_DIR/container-scan-$timestamp.json"

{
    echo "{"
    echo "  \"timestamp\": \"$(date -Iseconds)\","
    echo "  \"severity_filter\": \"$SEVERITY\","
    echo "  \"images\": ["
} > "$json_file"

local first=true
while read -r image; do
    if [[ "$first" != "true" ]]; then
        echo "," >> "$json_file"
    fi
    first=false
    echo -n "    \"$image\"" >> "$json_file"
done < <(extract_images)

{
    echo ""
    echo "  ]"
    echo "}"
} >> "$json_file"
```

**JSON structure:**
```json
{
  "timestamp": "2026-01-13T10:30:00-05:00",
  "severity_filter": "CRITICAL,HIGH",
  "images": [
    "hashicorp/vault:1.18",
    "prom/prometheus:latest"
  ]
}
```

---

## Output Formats

| Format | Flag | Use Case | Details |
|--------|------|----------|---------|
| **table** | (default) | Human-readable CLI output | Color-coded, formatted tables |
| **json** | `--json` | CI/CD integration, parsing | Machine-readable JSON |
| **sarif** | `--sarif` | GitHub Security tab | SARIF format for code scanning |
| **html** | `--report` | Executive reports, sharing | Styled HTML with embedded CSS |

**SARIF usage (lines 303-305):**
```bash
--sarif)
    OUTPUT_FORMAT="sarif"
    shift
    ;;
```

**GitHub Security integration:**
```yaml
# .github/workflows/security.yml
- name: Scan containers
  run: ./scripts/scan-containers.sh --sarif > results.sarif
- name: Upload to GitHub Security
  uses: github/codeql-action/upload-sarif@v2
  with:
    sarif_file: results.sarif
```

---

## List Images

**Evidence:** Lines 92-100

```bash
list_images() {
    log_info "Docker images used by ARIA platform:"
    echo ""
    extract_images | while read -r image; do
        echo "  - $image"
    done
    echo ""
    log_info "Total: $(extract_images | wc -l) unique images"
}
```

**Usage:**
```bash
./scripts/scan-containers.sh --list
```

**Output:**
```
[INFO] Docker images used by ARIA platform:

  - grafana/grafana:latest
  - hashicorp/vault:1.18
  - kong:3.8
  - localai/localai:latest
  - n8nio/n8n:latest
  - nats:2.10-alpine
  - openpolicyagent/opa:latest
  - prom/prometheus:latest

[INFO] Total: 8 unique images
```

---

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: Container Security Scan
on:
  push:
    branches: [main]
  schedule:
    - cron: '0 2 * * *'  # Daily at 2 AM

jobs:
  scan:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Install Trivy
        run: |
          wget -qO - https://aquasecurity.github.io/trivy-repo/deb/public.key | sudo apt-key add -
          echo "deb https://aquasecurity.github.io/trivy-repo/deb $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/trivy.list
          sudo apt-get update && sudo apt-get install trivy

      - name: Scan containers
        run: ./scripts/scan-containers.sh --critical --sarif > results.sarif

      - name: Upload SARIF to GitHub Security
        uses: github/codeql-action/upload-sarif@v2
        with:
          sarif_file: results.sarif

      - name: Generate HTML report
        if: failure()
        run: ./scripts/scan-containers.sh --report

      - name: Upload report artifact
        if: failure()
        uses: actions/upload-artifact@v4
        with:
          name: security-report
          path: reports/security/
```

---

## Troubleshooting

### Trivy Not Found

**Symptoms:** "trivy is not installed" error (line 70)

**Fix:**
```bash
# Via Nix (recommended)
nix develop

# Or install directly
wget -qO - https://aquasecurity.github.io/trivy-repo/deb/public.key | sudo apt-key add -
echo "deb https://aquasecurity.github.io/trivy-repo/deb $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/trivy.list
sudo apt-get update && sudo apt-get install trivy
```

### No Images Found

**Symptoms:** "Total: 0 unique images"

**Fix:**
```bash
# Check compose files exist
ls docker/docker-compose*.yml

# Verify images defined
grep "image:" docker/docker-compose*.yml
```

### Image Pull Failed

**Symptoms:** "Could not pull image" warning (line 147)

**Possible causes:**
- Private registry (needs authentication)
- Local-only image (built locally)
- Network issues

**Fix:**
```bash
# Login to Docker Hub
docker login

# Or skip pull for local images
docker images | grep <image-name>
```

### Permission Denied

**Symptoms:** Cannot write to reports directory

**Fix:**
```bash
# Create directory with proper permissions
mkdir -p reports/security
chmod 755 reports/security
```

---

## References

### Source Code
- **Main script:** `scripts/scan-containers.sh` (333 lines)
- **Image extraction:** lines 81-90
- **Scanning logic:** lines 102-128
- **Scan all:** lines 130-172
- **HTML report:** lines 174-270
- **Main function:** lines 273-332

### Related Files
- **Docker Compose files:** `docker/docker-compose*.yml`
- **Report directory:** `reports/security/`

### External Resources
- [Trivy Documentation](https://aquasecurity.github.io/trivy/)
- [Trivy GitHub](https://github.com/aquasecurity/trivy)
- [SARIF Format](https://sarifweb.azurewebsites.net/)
- [GitHub Code Scanning](https://docs.github.com/en/code-security/code-scanning)
- [CVE Database](https://cve.mitre.org/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 39/60 contracts complete (65%)
