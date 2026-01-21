# GitHub Copilot Custom Agent Testing Guide

This document describes how to test the Security Auto-Fix Agent.

## Manual Testing

### 1. Test Agent Recognition

In GitHub Copilot Chat, verify the agent is recognized:

```
@security_auto_fix Hello, are you available?
```

Expected: Agent responds confirming its role and capabilities.

### 2. Test Code Vulnerability Detection

Create a test file with a known vulnerability:

**test_vulnerability.py**:
```python
import sqlite3

def get_user(user_id):
    conn = sqlite3.connect('database.db')
    cursor = conn.cursor()
    # SQL Injection vulnerability
    query = f"SELECT * FROM users WHERE id = {user_id}"
    cursor.execute(query)
    return cursor.fetchone()
```

Then test the agent:
```
@security_auto_fix Fix the SQL injection vulnerability in test_vulnerability.py
```

Expected: Agent should suggest using parameterized queries.

### 3. Test Dependency Update

Test with vulnerable dependency information:

```
@security_auto_fix Update requests package from version 2.25.0 to 2.31.0 to fix CVE-2023-32681
```

Expected: Agent should update the dependency in the appropriate file (pixi.toml, requirements.txt, etc.)

### 4. Test Secret Removal

Create a file with a hard-coded secret:

**test_secrets.py**:
```python
API_KEY = "sk-1234567890abcdef"
SECRET_TOKEN = "ghp_abcdefghijklmnopqrstuvwxyz"
```

Test:
```
@security_auto_fix Remove hard-coded secrets from test_secrets.py and use environment variables
```

Expected: Agent should replace with `os.environ.get()` calls.

### 5. Test Container Hardening

Create a vulnerable Dockerfile:

**test.Dockerfile**:
```dockerfile
FROM ubuntu:20.04
RUN apt-get update && apt-get install -y python3
COPY . /app
WORKDIR /app
CMD ["python3", "app.py"]
```

Test:
```
@security_auto_fix Harden test.Dockerfile following security best practices
```

Expected: Agent should update base image, add non-root user, clean apt cache, etc.

## Automated Testing

### Validation Script

Run the agent validation script:

```bash
./scripts/validate-agents.sh
```

Expected output:
```
✅ All agent files validated successfully!
```

### Integration Tests

#### Test 1: Agent File Format

```bash
# Verify YAML frontmatter exists
grep -A 7 "^---$" .github/agents/security-auto-fix.agent.md | grep "name: security_auto_fix"
```

#### Test 2: Agent Documentation

```bash
# Verify documentation files exist
test -f .github/agents/README.md && echo "✅ README.md exists"
test -f docs/SECURITY_AUTO_FIX_GUIDE.md && echo "✅ Guide exists"
```

#### Test 3: SECURITY.md Integration

```bash
# Verify SECURITY.md references the agent
grep -q "Security Auto-Fix Agent" SECURITY.md && echo "✅ SECURITY.md updated"
```

## CI/CD Testing

### GitHub Actions Workflow Test

Create a test workflow to verify agent accessibility:

**.github/workflows/test-agent.yml**:
```yaml
name: Test Security Auto-Fix Agent

on:
  workflow_dispatch:

jobs:
  test-agent-format:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Validate agent files
        run: |
          ./scripts/validate-agents.sh
      
      - name: Check agent documentation
        run: |
          test -f .github/agents/security-auto-fix.agent.md
          test -f .github/agents/README.md
          test -f docs/SECURITY_AUTO_FIX_GUIDE.md
          grep -q "security_auto_fix" .github/agents/security-auto-fix.agent.md
```

## Real-World Testing Scenarios

### Scenario 1: Fix Recent Security Scan

1. Run security scan:
   ```bash
   trivy fs --severity HIGH,CRITICAL . > scan-results.txt
   ```

2. Invoke agent:
   ```
   @security_auto_fix Review scan-results.txt and fix all HIGH and CRITICAL vulnerabilities
   ```

3. Verify fixes:
   ```bash
   trivy fs --severity HIGH,CRITICAL .
   # Should show fewer or no issues
   ```

### Scenario 2: Dependency Vulnerability

1. Check for vulnerable dependencies:
   ```bash
   pixi list
   cargo audit
   npm audit
   ```

2. Ask agent to fix:
   ```
   @security_auto_fix Update all dependencies with known CVEs to their patched versions
   ```

3. Verify updates:
   ```bash
   pixi list
   cargo audit
   git diff pixi.toml Cargo.toml
   ```

### Scenario 3: Secret Detection

1. Run secret detection:
   ```bash
   gitleaks detect --source . --verbose --report-path secrets.json
   ```

2. Fix with agent:
   ```
   @security_auto_fix Remove all secrets found in secrets.json
   ```

3. Verify:
   ```bash
   gitleaks detect --source . --verbose
   # Should show no secrets
   ```

## Testing Checklist

Before considering the agent production-ready:

- [ ] Agent file format validated
- [ ] YAML frontmatter correct
- [ ] Documentation complete and accurate
- [ ] Agent responds to basic queries
- [ ] Can fix SQL injection vulnerabilities
- [ ] Can update vulnerable dependencies
- [ ] Can remove hard-coded secrets
- [ ] Can harden Docker containers
- [ ] Integration with security workflows verified
- [ ] Error handling tested (invalid requests)
- [ ] Edge cases handled (no fix available, breaking changes needed)

## Known Limitations

Document any limitations discovered during testing:

1. **Cannot fix architectural issues** - Agent is for tactical fixes only
2. **Requires human review for breaking changes** - Agent will escalate
3. **Limited to supported languages** - Python, Rust, JavaScript, Shell
4. **Dependency conflicts** - May need manual resolution

## Feedback Loop

After testing, document:

1. **What worked well**:
   - [To be filled during testing]

2. **What needs improvement**:
   - [To be filled during testing]

3. **Enhancement requests**:
   - [To be filled during testing]

## Performance Metrics

Track these metrics during testing:

- **Response time**: How quickly does agent respond?
- **Fix accuracy**: What % of fixes work without modification?
- **False positives**: How often does agent suggest unnecessary changes?
- **Coverage**: What % of security issues can it fix?

## Continuous Improvement

Based on testing results:

1. Update agent instructions in `.github/agents/security-auto-fix.agent.md`
2. Enhance documentation with real examples
3. Add more specific patterns for common vulnerabilities
4. Update decision rules based on feedback

---

Last updated: January 2026
