#!/usr/bin/env python3
"""
ARIA Verification Script Generator

Generates verify-components.sh from ARIA_MANIFEST.yaml.

Usage:
    python scripts/generate-verification.py [--manifest PATH] [--output PATH]
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

try:
    import yaml
except ImportError:
    print("Missing pyyaml. Install with: pip install pyyaml")
    sys.exit(1)


def load_manifest(path: Path) -> dict:
    """Load and parse ARIA_MANIFEST.yaml."""
    with open(path) as f:
        return yaml.safe_load(f)


def generate_header(manifest: dict) -> str:
    """Generate script header."""
    version = manifest.get("version", "unknown")
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    return f'''#!/usr/bin/env bash
# =============================================================================
# ARIA Component Verification Script
# =============================================================================
# AUTO-GENERATED from ARIA_MANIFEST.yaml v{version}
# Generated: {timestamp}
#
# Usage:
#   ./scripts/verify-components.sh [--profile PROFILE] [--verbose]
#
# Profiles:
#   minimal  - Core tools only (fastest)
#   ci       - CI verification (core + robotics)
#   default  - Standard development
#   full     - Everything including Docker
# =============================================================================

set -euo pipefail

# Colors
RED='\\033[0;31m'
GREEN='\\033[0;32m'
YELLOW='\\033[0;33m'
BLUE='\\033[0;34m'
NC='\\033[0m' # No Color

# Counters
PASSED=0
FAILED=0
SKIPPED=0

# Configuration
PROFILE="${{1:-default}}"
VERBOSE="${{VERBOSE:-false}}"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --profile)
            PROFILE="$2"
            shift 2
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        *)
            PROFILE="$1"
            shift
            ;;
    esac
done

echo -e "${{BLUE}}==============================================================================${{NC}}"
echo -e "${{BLUE}}ARIA Component Verification${{NC}}"
echo -e "${{BLUE}}==============================================================================${{NC}}"
echo ""
echo -e "Profile: ${{YELLOW}}$PROFILE${{NC}}"
echo -e "Date: $(date)"
echo ""

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------

verify_command() {{
    local id="$1"
    local name="$2"
    local cmd="$3"
    local pattern="${{4:-}}"
    local required="${{5:-false}}"

    printf "  %-30s " "$name"

    if output=$(eval "$cmd" 2>&1); then
        if [[ -z "$pattern" ]] || echo "$output" | grep -qE "$pattern"; then
            echo -e "${{GREEN}}PASS${{NC}}"
            ((PASSED++))
            return 0
        else
            echo -e "${{YELLOW}}WARN${{NC}} (unexpected output)"
            if [[ "$VERBOSE" == "true" ]]; then
                echo "    Output: $output"
            fi
            ((PASSED++))
            return 0
        fi
    else
        if [[ "$required" == "true" ]]; then
            echo -e "${{RED}}FAIL${{NC}}"
            ((FAILED++))
            return 1
        else
            echo -e "${{YELLOW}}SKIP${{NC}} (optional)"
            ((SKIPPED++))
            return 0
        fi
    fi
}}

verify_http() {{
    local id="$1"
    local name="$2"
    local url="$3"
    local timeout="${{4:-5}}"

    printf "  %-30s " "$name"

    if curl -sf --max-time "$timeout" "$url" > /dev/null 2>&1; then
        echo -e "${{GREEN}}PASS${{NC}}"
        ((PASSED++))
        return 0
    else
        echo -e "${{YELLOW}}SKIP${{NC}} (not running)"
        ((SKIPPED++))
        return 0
    fi
}}

section() {{
    echo ""
    echo -e "${{BLUE}}--- $1 ---${{NC}}"
}}

'''


def generate_nix_checks(manifest: dict, profile_categories: set) -> str:
    """Generate Nix component verification checks."""
    lines = ['section "Nix Components"', ""]

    components = manifest.get("nix_components", [])
    for comp in components:
        category = comp.get("category", "")
        if profile_categories and category not in profile_categories:
            continue

        comp_id = comp.get("id", "")
        name = comp.get("name", comp_id)
        verify = comp.get("verify", {})
        cmd = verify.get("command", "")
        pattern = verify.get("pattern", "")
        required = comp.get("required", False)
        devshell = comp.get("devshell", "default")

        if not cmd:
            continue

        # Escape for shell
        cmd_escaped = cmd.replace('"', '\\"')
        pattern_escaped = pattern.replace('"', '\\"')
        required_str = "true" if required else "false"

        lines.append(
            f'verify_command "{comp_id}" "{name}" "{cmd_escaped}" "{pattern_escaped}" "{required_str}"'
        )

    return "\n".join(lines)


def generate_pixi_checks(manifest: dict, profile_categories: set, profile_envs: set) -> str:
    """Generate Pixi component verification checks."""
    lines = ['section "Pixi Components"', ""]

    components = manifest.get("pixi_components", [])
    for comp in components:
        category = comp.get("category", "")
        env = comp.get("environment", "default")

        # Filter by category or environment
        if profile_categories and category not in profile_categories:
            continue
        if profile_envs and env not in profile_envs:
            continue

        comp_id = comp.get("id", "")
        name = comp.get("name", comp_id)
        verify = comp.get("verify", {})
        cmd = verify.get("command", "")
        pattern = verify.get("pattern", "")
        required = comp.get("required", False)

        if not cmd:
            continue

        cmd_escaped = cmd.replace('"', '\\"')
        pattern_escaped = pattern.replace('"', '\\"')
        required_str = "true" if required else "false"

        lines.append(
            f'verify_command "{comp_id}" "{name}" "{cmd_escaped}" "{pattern_escaped}" "{required_str}"'
        )

    return "\n".join(lines)


def generate_docker_checks(manifest: dict, include_docker: bool) -> str:
    """Generate Docker service verification checks."""
    if not include_docker:
        return ""

    lines = ['section "Docker Services"', ""]

    services = manifest.get("docker_services", [])
    for svc in services:
        svc_id = svc.get("id", "")
        name = svc.get("name", svc_id)
        verify = svc.get("verify", {})

        verify_type = verify.get("type", "command")
        if verify_type == "http":
            url = verify.get("url", "")
            timeout = verify.get("timeout", 5)
            if url:
                lines.append(f'verify_http "{svc_id}" "{name}" "{url}" "{timeout}"')
        else:
            cmd = verify.get("command", "")
            if cmd:
                cmd_escaped = cmd.replace('"', '\\"')
                lines.append(f'verify_command "{svc_id}" "{name}" "{cmd_escaped}" "" "false"')

    return "\n".join(lines)


def generate_command_checks(manifest: dict, profile_devshell: str) -> str:
    """Generate command wrapper verification checks."""
    lines = ['section "Command Wrappers"', ""]

    commands = manifest.get("commands", [])
    for cmd in commands:
        cmd_devshell = cmd.get("devshell", "default")

        # Only check commands available in current devshell
        if profile_devshell == "default" and cmd_devshell == "full":
            continue

        cmd_id = cmd.get("id", "")
        name = cmd.get("name", cmd_id)
        verify = cmd.get("verify", {})
        verify_cmd = verify.get("command", "")

        if not verify_cmd:
            continue

        cmd_escaped = verify_cmd.replace('"', '\\"')
        lines.append(f'verify_command "{cmd_id}" "{name}" "{cmd_escaped}" "" "false"')

    return "\n".join(lines)


def generate_footer() -> str:
    """Generate script footer with summary."""
    return '''
# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------

echo ""
echo -e "${BLUE}==============================================================================${NC}"
echo -e "${BLUE}Verification Summary${NC}"
echo -e "${BLUE}==============================================================================${NC}"
echo ""
echo -e "  Passed:  ${GREEN}$PASSED${NC}"
echo -e "  Failed:  ${RED}$FAILED${NC}"
echo -e "  Skipped: ${YELLOW}$SKIPPED${NC}"
echo ""

if [[ $FAILED -gt 0 ]]; then
    echo -e "${RED}VERIFICATION FAILED${NC}"
    exit 1
else
    echo -e "${GREEN}VERIFICATION PASSED${NC}"
    exit 0
fi
'''


def get_profile_config(manifest: dict, profile_name: str) -> tuple:
    """Get configuration for a profile."""
    profiles = manifest.get("profiles", {})
    profile = profiles.get(profile_name, {})

    categories = set(profile.get("categories", []))
    devshell = profile.get("devshell", "default")
    environments = set(profile.get("environments", ["default"]))
    include_docker = profile.get("include_docker", False)

    return categories, devshell, environments, include_docker


def generate_profile_switch(manifest: dict) -> str:
    """Generate case statement for profile selection."""
    profiles = manifest.get("profiles", {})

    lines = ["# Profile-specific verification", 'case "$PROFILE" in']

    for profile_name, profile in profiles.items():
        categories, devshell, environments, include_docker = get_profile_config(
            manifest, profile_name
        )

        lines.append(f"    {profile_name})")

        # Generate checks for this profile
        nix_checks = generate_nix_checks(manifest, categories)
        pixi_checks = generate_pixi_checks(manifest, categories, environments)
        docker_checks = generate_docker_checks(manifest, include_docker)
        cmd_checks = generate_command_checks(manifest, devshell)

        for check_block in [nix_checks, pixi_checks, docker_checks, cmd_checks]:
            if check_block.strip():
                lines.append(check_block)

        lines.append("        ;;")
        lines.append("")

    lines.append("    *)")
    lines.append('        echo "Unknown profile: $PROFILE"')
    lines.append('        echo "Available profiles: ' + ", ".join(profiles.keys()) + '"')
    lines.append("        exit 1")
    lines.append("        ;;")
    lines.append("esac")

    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Generate verification script from manifest")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("ARIA_MANIFEST.yaml"),
        help="Path to manifest file",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("scripts/verify-components.sh"),
        help="Output script path",
    )
    args = parser.parse_args()

    # Load manifest
    try:
        manifest = load_manifest(args.manifest)
    except FileNotFoundError:
        print(f"ERROR: Manifest not found: {args.manifest}")
        sys.exit(1)

    # Generate script
    script_parts = [
        generate_header(manifest),
        generate_profile_switch(manifest),
        generate_footer(),
    ]

    script = "\n".join(script_parts)

    # Write output
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w") as f:
        f.write(script)

    # Make executable
    args.output.chmod(0o755)

    print(f"Generated: {args.output}")
    print(f"  Profiles: {len(manifest.get('profiles', {}))}")
    print(f"  Nix components: {len(manifest.get('nix_components', []))}")
    print(f"  Pixi components: {len(manifest.get('pixi_components', []))}")
    print(f"  Docker services: {len(manifest.get('docker_services', []))}")
    print(f"  Commands: {len(manifest.get('commands', []))}")


if __name__ == "__main__":
    main()
