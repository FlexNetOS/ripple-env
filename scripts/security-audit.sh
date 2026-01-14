#!/usr/bin/env bash
# FlexNetOS Security Audit Script

set -euo pipefail

echo "Running FlexNetOS security audit..."

# Run Trivy vulnerability scan
echo "Scanning for vulnerabilities..."
SECURITY_AUDIT_REQUIRE_TRIVY="${SECURITY_AUDIT_REQUIRE_TRIVY:-0}"
SECURITY_AUDIT_REQUIRE_SECRETS="${SECURITY_AUDIT_REQUIRE_SECRETS:-0}"
if command -v trivy &> /dev/null; then
    ignorefile_args=()
    if [ -f ".trivyignore" ]; then
        ignorefile_args=(--ignorefile .trivyignore)
    fi

    trivy fs --security-checks vuln,config "${ignorefile_args[@]}" .
else
    if [ "$SECURITY_AUDIT_REQUIRE_TRIVY" = "1" ]; then
      echo "Error: trivy not found. Install trivy or set SECURITY_AUDIT_REQUIRE_TRIVY=0 to skip."
      exit 1
    fi
    echo "Warning: trivy not found; skipping vulnerability scan."
    echo "Tip: install trivy (recommended) or run a container-based scan via docker/trivy.yml if available."
fi

# Check for secrets using dedicated tools
echo "Checking for exposed secrets..."

# Try to use gitleaks if available
if command -v gitleaks &> /dev/null; then
    echo "Running gitleaks scan..."
    # Default to a worktree scan (directory mode) to keep the audit actionable on
    # repos that may have historical false-positives. You can opt into a full git
    # history scan by setting SECURITY_AUDIT_GITLEAKS_MODE=git.
    SECURITY_AUDIT_GITLEAKS_MODE="${SECURITY_AUDIT_GITLEAKS_MODE:-dir}"
    SECURITY_AUDIT_GITLEAKS_LOG_OPTS="${SECURITY_AUDIT_GITLEAKS_LOG_OPTS:-}"

    gitleaks_mode_args=()
    case "${SECURITY_AUDIT_GITLEAKS_MODE}" in
        dir)
            gitleaks_mode_args=(--no-git)
            ;;
        git)
            if [ ! -d ".git" ]; then
                echo "Warning: .git not found; falling back to directory scan."
                gitleaks_mode_args=(--no-git)
            elif [ -n "${SECURITY_AUDIT_GITLEAKS_LOG_OPTS}" ]; then
                gitleaks_mode_args=(--log-opts="${SECURITY_AUDIT_GITLEAKS_LOG_OPTS}")
            fi
            ;;
        *)
            echo "Error: SECURITY_AUDIT_GITLEAKS_MODE must be 'dir' or 'git'."
            exit 2
            ;;
    esac

    # Redact secrets from stdout (avoid printing real tokens/keys into logs).
    # Note: this repo provides a `.gitleaks.toml` allowlist to reduce noise from
    # generated/vendor directories when scanning the worktree.
    gitleaks_config_args=()
    if [ -f ".gitleaks.toml" ]; then
        gitleaks_config_args=(--config ".gitleaks.toml")
    fi
    gitleaks_rc=0
    gitleaks detect --source . --verbose --redact=100 "${gitleaks_config_args[@]}" "${gitleaks_mode_args[@]}" || gitleaks_rc=$?
    if [ "${gitleaks_rc}" != "0" ]; then
        echo "Warning: gitleaks reported potential leaks (exit ${gitleaks_rc})."
        if [ "${SECURITY_AUDIT_REQUIRE_SECRETS}" = "1" ]; then
            echo "Error: secret scan failed and SECURITY_AUDIT_REQUIRE_SECRETS=1 is set."
            exit "${gitleaks_rc}"
        fi
    fi
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
