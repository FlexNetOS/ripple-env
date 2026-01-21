# Security Policy

## Supported Versions

| Version | Supported          |
| ------- | ------------------ |
| main    | :white_check_mark: |
| feature branches | :x: |

Only the `main` branch receives security updates. Feature branches should be kept up-to-date with main.

## Reporting a Vulnerability

**Please do NOT report security vulnerabilities through public GitHub issues.**

### How to Report

1. **Email**: Send details to the repository maintainers (check repository owner's profile)
2. **GitHub Security Advisories**: Use the "Security" tab to report privately
3. **Response Time**: Expect initial response within 48 hours

### What to Include

- Description of the vulnerability
- Steps to reproduce
- Potential impact assessment
- Suggested fix (if available)

### What to Expect

1. **Acknowledgment**: Within 48 hours
2. **Initial Assessment**: Within 1 week
3. **Fix Timeline**: Depends on severity
   - Critical: 24-48 hours
   - High: 1 week
   - Medium: 2 weeks
   - Low: Next release

## Security Considerations

This is a **development environment** for ROS2 robotics. Consider these aspects:

### Environment Assumptions

1. **Trusted Network**: This environment assumes a trusted development network
2. **Local Development**: Not intended for production robot control without hardening
3. **Developer Machine**: Assumes the developer machine is secure

### Known Security Patterns

#### Shell Hook Execution

The flake.nix uses `eval "$(pixi shell-hook)"` which executes code from the Pixi environment. This is standard practice but:

- Ensure `pixi.lock` is reviewed before committing
- Don't add untrusted channels to `pixi.toml`

#### Bootstrap Scripts

The `bootstrap.sh` and `bootstrap.ps1` scripts:

- Download and execute the Nix installer from Determinate Systems
- Use HTTPS with TLS 1.2 minimum
- Users should verify the installer URL before running

#### CUDA Configuration

When using the CUDA devshell (`.#cuda`):

- Only available on Linux
- Requires NVIDIA drivers on the host system
- Binary cache at `cache.nixos-cuda.org` is trusted

### Best Practices

1. **Review Dependencies**
   ```bash
   # Check Nix dependencies
   nix flake metadata

   # Check Pixi dependencies
   cat pixi.lock | head -100
   ```

2. **Scan for Vulnerabilities**
   ```bash
   # Container/filesystem scanning
   trivy fs .

   # Nix-specific scanning (if available)
   vulnix ./result
   ```

3. **Keep Dependencies Updated**
   ```bash
   # Update Nix inputs
   nix flake update

   # Update Pixi packages
   pixi update
   ```

4. **Use Pre-commit Hooks**
   ```bash
   pip install pre-commit
   pre-commit install
   pre-commit run --all-files
   ```

## Security Features

### Implemented

- [x] TLS 1.2+ for all downloads
- [x] No hardcoded secrets in configuration
- [x] Platform-specific guards for CUDA (Linux-only)
- [x] Trivy included for security scanning
- [x] Pre-commit hooks for secret detection
- [x] **Security Auto-Fix Agent** for automated vulnerability remediation ([docs](docs/SECURITY_AUTO_FIX_GUIDE.md))

### Planned

- [ ] Dependabot/Renovate for automated updates
- [ ] SBOM (Software Bill of Materials) generation
- [ ] Signed commits enforcement
- [ ] Branch protection rules documentation

## Automated Security Fixes

This repository includes a **GitHub Copilot Custom Agent** that automatically fixes security vulnerabilities.

### Quick Start

Use the Security Auto-Fix Agent in GitHub Copilot Chat:

```
@security_auto_fix Fix the HIGH and CRITICAL vulnerabilities in [file/directory]
```

**Capabilities:**
- Fixes code vulnerabilities (SQL injection, XSS, command injection, etc.)
- Updates vulnerable dependencies to patched versions
- Removes hard-coded secrets and implements secure alternatives
- Hardens container configurations
- Fixes configuration security issues

**Documentation:**
- [Security Auto-Fix Agent Guide](docs/SECURITY_AUTO_FIX_GUIDE.md)
- [Agent Configuration](.github/agents/security-auto-fix.agent.md)
- [Agent Usage Examples](.github/agents/README.md)

### Integration with Security Workflows

The agent integrates with existing security scanning:
- **CodeQL**: Automatically fix code-level vulnerabilities
- **Trivy**: Update vulnerable dependencies and container images
- **Gitleaks**: Remove hard-coded secrets
- **Grype**: Remediate SBOM vulnerabilities

See [SECURITY_AUTO_FIX_GUIDE.md](docs/SECURITY_AUTO_FIX_GUIDE.md) for integration examples.

## Third-Party Dependencies

### Trusted Sources

| Source | Purpose | Trust Level |
|--------|---------|-------------|
| nixpkgs | Nix packages | High (community-reviewed) |
| robostack-humble | ROS2 packages | Medium (conda-forge derived) |
| conda-forge | Python packages | Medium (community-reviewed) |
| Determinate Systems | Nix installer | High (commercial entity) |
| cache.nixos-cuda.org | CUDA binaries | High (NixOS maintainers) |

### Supply Chain Considerations

1. **Lock Files**: Both `flake.lock` and `pixi.lock` are committed
2. **Binary Caches**: Use official caches only
3. **GitHub Actions**: Pin actions to full SHA where possible

## Incident Response

If you discover a security issue in a deployed environment:

1. **Isolate**: Disconnect affected systems from network
2. **Assess**: Determine scope and impact
3. **Report**: Follow vulnerability reporting process above
4. **Patch**: Apply fixes to clean environment
5. **Review**: Conduct post-incident analysis

## Contact

For security concerns, please contact the repository maintainers through:

1. GitHub Security Advisories (preferred)
2. Direct message to repository owner
3. Email (check owner's profile)

---

*Last updated: January 2026*
