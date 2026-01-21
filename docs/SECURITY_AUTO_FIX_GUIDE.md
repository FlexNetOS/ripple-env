# Security Auto-Fix Agent Integration Guide

This guide explains how to integrate the Security Auto-Fix Agent with your development workflow and CI/CD pipeline.

## Quick Start

### 1. Basic Usage

When you encounter security vulnerabilities in your code:

```bash
# In GitHub Copilot Chat
@security_auto_fix Review and fix security issues in the current file
```

### 2. After Security Scans

When CI/CD security scans fail:

```bash
# Check the security scan results
gh run view [run-id] --log-failed

# Then invoke the agent
@security_auto_fix Fix the vulnerabilities identified in the latest security scan
```

## Integration with Security Workflows

### CodeQL Integration

When CodeQL detects vulnerabilities:

1. **View Alerts**:
   ```bash
   gh api repos/:owner/:repo/code-scanning/alerts
   ```

2. **Invoke Agent**:
   ```bash
   @security_auto_fix Fix CodeQL alert #[alert-number] in [file]
   ```

3. **Verify Fix**:
   ```bash
   # Re-run CodeQL analysis
   gh workflow run security.yml
   ```

### Trivy Filesystem Scan

When Trivy finds vulnerabilities in dependencies:

1. **Run Scan Locally**:
   ```bash
   trivy fs --severity HIGH,CRITICAL --format json . > trivy-results.json
   ```

2. **Review Results**:
   ```bash
   jq '.Results[] | select(.Vulnerabilities)' trivy-results.json
   ```

3. **Fix with Agent**:
   ```bash
   @security_auto_fix Update vulnerable dependencies identified in trivy-results.json
   ```

### Container Security

When container image scans fail:

1. **Scan Image**:
   ```bash
   trivy image <image:tag> --severity HIGH,CRITICAL
   ```

2. **Fix with Agent**:
   ```bash
   @security_auto_fix Harden the Dockerfile at docker/Dockerfile based on Trivy scan results
   ```

### Secrets Detection

When Gitleaks detects secrets:

1. **Run Detection**:
   ```bash
   gitleaks detect --source . --verbose --report-path gitleaks-report.json
   ```

2. **Remove Secrets**:
   ```bash
   @security_auto_fix Remove secrets detected in gitleaks-report.json and replace with environment variables
   ```

## Workflow Examples

### Example 1: Fixing Dependency Vulnerabilities

```bash
# 1. Identify vulnerable dependencies
pixi list | grep -i vulnerable
# or
cargo audit

# 2. Get details
gh api repos/FlexNetOS/ripple-env/dependabot/alerts

# 3. Fix with agent
@security_auto_fix Update the following vulnerable dependencies:
- requests (current: 2.25.0, fix: 2.31.0, CVE: CVE-2023-32681)
- cryptography (current: 3.4.8, fix: 41.0.0, CVE: CVE-2023-23931)

# 4. Verify
pixi install
pixi run test
```

### Example 2: Hardening Docker Images

```bash
# 1. Scan current images
trivy image --severity HIGH,CRITICAL myapp:latest

# 2. Fix with agent
@security_auto_fix Please harden docker/Dockerfile:
- Update base image from ubuntu:20.04 to latest LTS
- Add non-root user
- Remove unnecessary packages
- Implement multi-stage build

# 3. Rebuild and verify
docker build -t myapp:latest -f docker/Dockerfile .
trivy image myapp:latest
```

### Example 3: Code Vulnerability Remediation

```bash
# 1. Run CodeQL or review security alert
gh api repos/FlexNetOS/ripple-env/code-scanning/alerts/123

# 2. Fix with agent
@security_auto_fix Fix the SQL injection vulnerability in src/database/queries.py:
- Use parameterized queries instead of string concatenation
- Add input validation
- Add unit test to verify fix

# 3. Verify
pytest tests/test_queries.py
```

## CI/CD Integration

### GitHub Actions Workflow

Add automatic security fixing to your workflow:

```yaml
name: Security Auto-Fix

on:
  schedule:
    - cron: '0 2 * * 1'  # Weekly on Monday at 2 AM
  workflow_dispatch:

jobs:
  security-scan-and-fix:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          token: ${{ secrets.GITHUB_TOKEN }}

      - name: Run Trivy Scan
        uses: aquasecurity/trivy-action@0.28.0
        with:
          scan-type: 'fs'
          format: 'json'
          output: 'trivy-results.json'
          severity: 'HIGH,CRITICAL'
        continue-on-error: true

      - name: Check for Vulnerabilities
        id: check_vulns
        run: |
          if [ -f trivy-results.json ]; then
            VULN_COUNT=$(jq '[.Results[]?.Vulnerabilities // []] | add | length // 0' < trivy-results.json)
            echo "count=$VULN_COUNT" >> $GITHUB_OUTPUT
            if [ "$VULN_COUNT" -gt 0 ]; then
              echo "::warning::Found $VULN_COUNT vulnerabilities"
            fi
          fi

      - name: Create Issue with Fix Instructions
        if: steps.check_vulns.outputs.count > 0
        uses: actions/github-script@v7
        with:
          script: |
            const fs = require('fs');
            const results = JSON.parse(fs.readFileSync('trivy-results.json', 'utf8'));
            
            let body = '## 🔒 Security Vulnerabilities Detected\n\n';
            body += 'The automated security scan found vulnerabilities that need attention.\n\n';
            body += '### How to Fix with Security Auto-Fix Agent\n\n';
            body += '```\n';
            body += '@security_auto_fix Fix the following vulnerabilities:\n\n';
            
            // Add vulnerability details
            for (const result of results.Results || []) {
              if (result.Vulnerabilities) {
                for (const vuln of result.Vulnerabilities) {
                  body += `- ${vuln.PkgName}: ${vuln.InstalledVersion} → ${vuln.FixedVersion || 'No fix available'}\n`;
                  body += `  CVE: ${vuln.VulnerabilityID}, Severity: ${vuln.Severity}\n`;
                }
              }
            }
            body += '```\n\n';
            body += '### Manual Steps\n\n';
            body += '1. Review the vulnerabilities in the attached report\n';
            body += '2. Use `@security_auto_fix` in GitHub Copilot Chat\n';
            body += '3. Review and test the proposed fixes\n';
            body += '4. Create a PR with the fixes\n';
            
            await github.rest.issues.create({
              owner: context.repo.owner,
              repo: context.repo.repo,
              title: '🔒 Security Vulnerabilities Need Attention',
              body: body,
              labels: ['security', 'automated']
            });

      - name: Upload Scan Results
        uses: actions/upload-artifact@v4
        with:
          name: security-scan-results
          path: trivy-results.json
```

### Pre-commit Hook

Add security checking to pre-commit:

```yaml
# .pre-commit-config.yaml
repos:
  - repo: https://github.com/gitleaks/gitleaks
    rev: v8.18.0
    hooks:
      - id: gitleaks
        name: Detect secrets
        description: Detect hardcoded secrets using Gitleaks
        entry: gitleaks protect --staged --verbose
        language: system
        pass_filenames: false

  # Add custom hook to suggest agent usage
  - repo: local
    hooks:
      - id: security-reminder
        name: Security Auto-Fix Reminder
        entry: bash -c 'echo "💡 Tip: Use @security_auto_fix agent to automatically fix security issues"'
        language: system
        pass_filenames: false
        always_run: true
        verbose: true
```

## Best Practices

### 1. Review Before Committing
Always review agent-generated fixes:
```bash
git diff  # Review changes
git add -p  # Stage changes selectively
```

### 2. Test Thoroughly
```bash
# Run relevant tests
pixi run test
cargo test
nix flake check

# Run security scans again
trivy fs --severity HIGH,CRITICAL .
```

### 3. Document Changes
When committing fixes:
```bash
git commit -m "security: Fix CVE-2023-XXXXX in package-name

- Updated package-name from x.y.z to a.b.c
- Removed hard-coded secret in config.py
- Applied security patch for vulnerability

Fixed by: @security_auto_fix agent
Verified: Trivy scan clean"
```

### 4. Monitor Effectiveness
Track security fixes:
```bash
# View security-related commits
git log --grep="security:" --grep="CVE" --grep="vulnerability" --oneline

# Check current security status
gh api repos/FlexNetOS/ripple-env/code-scanning/alerts --jq 'length'
```

## Troubleshooting

### Agent Suggests Breaking Changes

If the fix would break functionality:
1. Ask agent to suggest minimal fix
2. Request backwards-compatible approach
3. Get human review before applying

### Dependency Update Conflicts

If dependency updates cause conflicts:
```bash
# Check dependency tree
pixi tree package-name
# or
cargo tree -p package-name

# Ask agent for compatible versions
@security_auto_fix Find a version of [package] that fixes CVE-XXXX but maintains compatibility with [other-package]
```

### False Positives

If scan reports false positives:
1. Add to ignore files:
   - `.trivyignore` for Trivy
   - `.gitleaks.toml` for Gitleaks
2. Document why it's a false positive
3. Update security documentation

## Metrics and Reporting

### Track Security Improvements

```bash
# Count open security issues
gh api repos/FlexNetOS/ripple-env/code-scanning/alerts --jq '[.[] | select(.state=="open")] | length'

# Count fixed issues (in last 30 days)
gh api repos/FlexNetOS/ripple-env/code-scanning/alerts --jq '[.[] | select(.state=="fixed" and (.fixed_at | fromdateiso8601) > (now - 2592000))] | length'

# Generate security report
gh api repos/FlexNetOS/ripple-env/code-scanning/alerts --jq 'group_by(.rule.severity) | map({severity: .[0].rule.severity, count: length})'
```

## Support and Resources

- **Documentation**: `.github/agents/README.md`
- **Security Workflow**: `.github/workflows/security.yml`
- **Security Policy**: `SECURITY.md`
- **Claude Agent**: `.claude/agents/security-agent.md`

For questions or issues, create a GitHub issue with the `[security]` tag.
