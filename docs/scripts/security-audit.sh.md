# Script Contract: security-audit.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/security-audit.sh`

---

## Purpose

FlexNetOS security audit script for vulnerability scanning and secret detection. Runs Trivy for CVE/misconfig scanning and gitleaks/detect-secrets/trufflehog for exposed secrets. Designed for CI/CD integration and pre-commit hooks.

---

## Invocation

```bash
./scripts/security-audit.sh
```

**Environment Variables:**
- `SECURITY_AUDIT_REQUIRE_TRIVY=1` - Fail if Trivy missing (default: 0, warnings only)
- `SECURITY_AUDIT_REQUIRE_SECRETS=1` - Fail on secret detection (default: 0, warnings only)
- `SECURITY_AUDIT_GITLEAKS_MODE=git|dir` - Gitleaks scan mode (default: dir)
- `SECURITY_AUDIT_GITLEAKS_LOG_OPTS` - Git log options for history scan

---

## Outputs

**Standard Output:**
```
Running FlexNetOS security audit...

Scanning for vulnerabilities...
[Trivy scan output...]

Checking for exposed secrets...
Running gitleaks scan...
[Gitleaks scan output...]

Security audit completed!
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (no issues or warnings only) |
| `1` | Error (Trivy missing if required) |
| `2` | Error (invalid SECURITY_AUDIT_GITLEAKS_MODE) |
| `{gitleaks_rc}` | Error (gitleaks exit code if SECURITY_AUDIT_REQUIRE_SECRETS=1) |

---

## Side Effects

**None** - Read-only scans, no file modifications.

---

## Safety Classification

**🟢 SAFE** - Read-only security scans, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Security Scan: Vulnerabilities (Trivy)

**Evidence:** Lines 8-37

### Trivy Configuration

**Command (line 29):**
```bash
trivy fs --quiet --scanners vuln,misconfig "${ignorefile_args[@]}" "${trivy_skip_args[@]}" .
```

**Scanners:**
- `vuln` - CVE vulnerability scanning
- `misconfig` - Configuration security issues (Dockerfile, K8s, Terraform, etc.)

**Ignore file support (lines 13-18):**
```bash
ignorefile_args=()
if [ -f ".trivyignore.yaml" ]; then
    ignorefile_args=(--ignorefile .trivyignore.yaml)
elif [ -f ".trivyignore" ]; then
    ignorefile_args=(--ignorefile .trivyignore)
fi
```

**Priority:** `.trivyignore.yaml` > `.trivyignore`

### Helm Chart Skip Logic

**Evidence:** Lines 20-27

```bash
trivy_skip_args=()
if [ -d "charts/flexstack" ] && [ ! -d "charts/flexstack/charts" ]; then
    trivy_skip_args=(--skip-dirs charts/flexstack)
fi
```

**Rationale:** Trivy attempts to render Helm charts. If dependencies not vendored locally, produces errors and skips scan.

**Condition:** Only scans chart if `charts/flexstack/charts/` exists (dependencies pulled).

### Trivy Unavailable Behavior

**Evidence:** Lines 30-37

**Default (SECURITY_AUDIT_REQUIRE_TRIVY=0):**
```bash
echo "Warning: trivy not found; skipping vulnerability scan."
echo "Tip: install trivy (recommended) or run a container-based scan via docker/trivy.yml if available."
```

**Strict (SECURITY_AUDIT_REQUIRE_TRIVY=1):**
```bash
echo "Error: trivy not found. Install trivy or set SECURITY_AUDIT_REQUIRE_TRIVY=0 to skip."
exit 1
```

---

## Security Scan: Secrets Detection

**Evidence:** Lines 39-98

### Tool Preference Cascade

**1. Gitleaks (preferred)** - Lines 43-85
**2. detect-secrets** - Lines 86-89
**3. trufflehog** - Lines 90-93
**4. Basic grep** - Lines 94-97

### Gitleaks Configuration

**Scan modes (lines 48-68):**

**Directory mode (default):**
```bash
SECURITY_AUDIT_GITLEAKS_MODE=dir
gitleaks detect --source . --no-git
```
- Scans working tree only
- Ignores git history
- Faster, actionable for current state

**Git mode:**
```bash
SECURITY_AUDIT_GITLEAKS_MODE=git
gitleaks detect --source .
```
- Scans full git history
- Finds historical leaks
- Slower, comprehensive

**Config file support (lines 73-76):**
```bash
gitleaks_config_args=()
if [ -f ".gitleaks.toml" ]; then
    gitleaks_config_args=(--config ".gitleaks.toml")
fi
```

**Command (line 78):**
```bash
gitleaks detect --source . --verbose --redact=100 "${gitleaks_config_args[@]}" "${gitleaks_mode_args[@]}"
```

**Flags:**
- `--verbose` - Detailed output
- `--redact=100` - Redact first 100 chars of secrets (avoid printing real tokens)

**Exit code handling (lines 79-85):**
```bash
gitleaks_rc=$?
if [ "${gitleaks_rc}" != "0" ]; then
    echo "Warning: gitleaks reported potential leaks (exit ${gitleaks_rc})."
    if [ "${SECURITY_AUDIT_REQUIRE_SECRETS}" = "1" ]; then
        exit "${gitleaks_rc}"
    fi
fi
```

**Default:** Warnings only
**Strict:** Fail on any leaks

### detect-secrets Fallback

**Evidence:** Lines 87-89

```bash
detect-secrets scan --baseline .secrets.baseline || true
```

**Baseline file:** `.secrets.baseline` (tracks known/allowed secrets)

**Always succeeds:** `|| true` prevents script exit

### trufflehog Fallback

**Evidence:** Lines 91-93

```bash
trufflehog filesystem . --json || true
```

**Output:** JSON format for parsing

**Always succeeds:** `|| true` prevents script exit

### Grep Fallback (Last Resort)

**Evidence:** Lines 95-97

```bash
grep -r "password\|secret\|key\|token" --include="*.yml" --include="*.yaml" --include="*.env" . || true
```

**Patterns:** password, secret, key, token

**File types:** YAML, ENV files only

**Limitation:** High false-positive rate, no context awareness

---

## CI/CD Integration

### GitHub Actions

```yaml
- name: Security Audit
  run: |
    SECURITY_AUDIT_REQUIRE_TRIVY=1 \
    SECURITY_AUDIT_REQUIRE_SECRETS=1 \
    ./scripts/security-audit.sh
```

### GitLab CI

```yaml
security_audit:
  script:
    - export SECURITY_AUDIT_REQUIRE_TRIVY=1
    - export SECURITY_AUDIT_REQUIRE_SECRETS=1
    - ./scripts/security-audit.sh
```

### Pre-commit Hook

```bash
# .git/hooks/pre-commit
#!/bin/bash
SECURITY_AUDIT_GITLEAKS_MODE=dir ./scripts/security-audit.sh
```

---

## Trivy Ignore Files

### .trivyignore.yaml (Structured)

```yaml
vulnerabilities:
  - id: CVE-2024-1234
    paths:
      - vendor/old-lib/**
    expiration: 2024-12-31
    reason: "No fix available, mitigated by firewall"

misconfigurations:
  - id: AVD-DS-0001
    paths:
      - test/**
    reason: "Test environment only"
```

### .trivyignore (Simple)

```
# Vendor directory false positives
CVE-2024-1234

# Test environment misconfigs
AVD-DS-0001
```

---

## Gitleaks Configuration

### .gitleaks.toml

```toml
[extend]
# Use default gitleaks config
useDefault = true

[allowlist]
description = "Allowlist for generated/vendor files"

[[allowlist.paths]]
description = "Vendor dependencies"
paths = [
    "vendor/**",
    "node_modules/**"
]

[[allowlist.regexes]]
description = "Test fixtures"
regex = "test/fixtures/.*"

[[allowlist.regexes]]
description = "Example values"
regex = "example|sample|placeholder"
```

---

## Common Trivy Findings

### Vulnerabilities
- **CVEs in dependencies** - Update packages
- **Known exploits** - Apply patches
- **End-of-life software** - Migrate to supported versions

### Misconfigurations
- **Dockerfile issues** - USER not set, secrets in ENV
- **Kubernetes** - Privileged pods, missing resource limits
- **Terraform** - Unencrypted storage, public access

---

## Common Secret Patterns

### Detected by Gitleaks
- AWS keys: `AKIA[0-9A-Z]{16}`
- GitHub tokens: `ghp_[a-zA-Z0-9]{36}`
- Private keys: `-----BEGIN (RSA|OPENSSH) PRIVATE KEY-----`
- Passwords in URLs: `://[^:]+:[^@]+@`

### False Positives
- Example/placeholder values
- Test fixtures
- Encrypted values
- Environment variable names (not values)

---

## Scan Performance

**Trivy:**
- Small repos: < 10 seconds
- Medium repos: 30-60 seconds
- Large repos: 2-5 minutes

**Gitleaks (dir mode):**
- Fast: 5-30 seconds

**Gitleaks (git mode):**
- Slow: 1-10 minutes (depends on history)

**Optimization:**
- Use `.trivyignore` to skip vendor/node_modules
- Use gitleaks dir mode for pre-commit
- Cache Trivy DB in CI

---

## Integration with ARIA

**Part of:** Security best practices for FlexNetOS/ARIA

**Runs before:**
- Git commits (pre-commit hook)
- Pull requests (CI checks)
- Deployments (release gates)

**Complements:**
- `verify-mtls-setup.sh` - Certificate validation
- `scan-containers.sh` - Runtime container scanning

---

## References

### Source Code
- **Main script:** `scripts/security-audit.sh` (101 lines)
- **Trivy scan:** lines 8-37
- **Secret detection:** lines 39-98
- **Gitleaks config:** lines 43-85

### Related Files
- **Trivy ignore:** `.trivyignore.yaml` or `.trivyignore`
- **Gitleaks config:** `.gitleaks.toml`
- **Secrets baseline:** `.secrets.baseline`
- **Container scanning:** `scripts/scan-containers.sh`

### External Resources
- [Trivy](https://trivy.dev/)
- [Gitleaks](https://github.com/gitleaks/gitleaks)
- [detect-secrets](https://github.com/Yelp/detect-secrets)
- [trufflehog](https://github.com/trufflesecurity/trufflehog)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 32/60 contracts complete
