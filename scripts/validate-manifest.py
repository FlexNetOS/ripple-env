#!/usr/bin/env python3
"""
ARIA Manifest Validator

Validates ARIA_MANIFEST.yaml against the JSON schema and performs
additional semantic checks.

Usage:
    python scripts/validate-manifest.py [--schema PATH] [--manifest PATH]
"""

import argparse
import json
import sys
from pathlib import Path

try:
    import yaml
    import jsonschema
    from jsonschema import Draft202012Validator
except ImportError:
    print("Missing dependencies. Install with: pip install pyyaml jsonschema")
    sys.exit(1)


def load_yaml(path: Path) -> dict:
    """Load and parse a YAML file."""
    with open(path) as f:
        return yaml.safe_load(f)


def load_json(path: Path) -> dict:
    """Load and parse a JSON file."""
    with open(path) as f:
        return json.load(f)


def validate_schema(manifest: dict, schema: dict) -> list[str]:
    """Validate manifest against JSON schema."""
    errors = []
    validator = Draft202012Validator(schema)
    for error in sorted(validator.iter_errors(manifest), key=lambda e: str(e.path)):
        path = ".".join(str(p) for p in error.path) or "(root)"
        errors.append(f"Schema error at {path}: {error.message}")
    return errors


def validate_unique_ids(manifest: dict) -> list[str]:
    """Check that all component IDs are unique."""
    errors = []
    all_ids = set()

    for section in ["nix_components", "pixi_components", "docker_services", "commands"]:
        if section not in manifest:
            continue
        for item in manifest[section]:
            item_id = item.get("id")
            if item_id in all_ids:
                errors.append(f"Duplicate ID found: {item_id} in {section}")
            all_ids.add(item_id)

    return errors


def validate_sources_exist(manifest: dict, root: Path) -> list[str]:
    """Check that referenced source files exist."""
    errors = []

    # Check required files
    if "validation" in manifest and "required_files" in manifest["validation"]:
        for req_file in manifest["validation"]["required_files"]:
            file_path = root / req_file["path"]
            if not file_path.exists():
                errors.append(f"Required file missing: {req_file['path']}")

    # Check docker-compose sources
    if "docker_services" in manifest:
        for service in manifest["docker_services"]:
            source = service.get("source", "")
            if source.startswith("docker/"):
                file_path = root / source
                if not file_path.exists():
                    errors.append(f"Docker compose file missing: {source}")

    return errors


def validate_profiles(manifest: dict) -> list[str]:
    """Validate that profile references are valid."""
    errors = []

    if "profiles" not in manifest:
        return errors

    valid_devshells = set()
    if "devshells" in manifest:
        valid_devshells = {ds["id"] for ds in manifest["devshells"]}

    for profile_name, profile in manifest["profiles"].items():
        if "devshell" in profile and profile["devshell"] not in valid_devshells:
            errors.append(
                f"Profile '{profile_name}' references unknown devshell: {profile['devshell']}"
            )

    return errors


def validate_verify_commands(manifest: dict) -> list[str]:
    """Check that verify commands are properly formed."""
    errors = []

    for section in ["nix_components", "pixi_components", "docker_services", "commands"]:
        if section not in manifest:
            continue
        for item in manifest[section]:
            verify = item.get("verify", {})
            if not verify:
                continue

            verify_type = verify.get("type", "command")
            if verify_type == "http" and "url" not in verify:
                errors.append(
                    f"Component '{item['id']}' has HTTP verify type but no URL"
                )
            if verify_type == "command" and "command" not in verify:
                errors.append(
                    f"Component '{item['id']}' has command verify type but no command"
                )

    return errors


def main():
    parser = argparse.ArgumentParser(description="Validate ARIA Manifest")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path("ARIA_MANIFEST.yaml"),
        help="Path to manifest file",
    )
    parser.add_argument(
        "--schema",
        type=Path,
        default=Path("config/schemas/aria-manifest.schema.json"),
        help="Path to JSON schema",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("."),
        help="Project root directory",
    )
    args = parser.parse_args()

    # Resolve paths
    root = args.root.resolve()
    manifest_path = (root / args.manifest).resolve()
    schema_path = (root / args.schema).resolve()

    print(f"Validating manifest: {manifest_path}")
    print(f"Using schema: {schema_path}")
    print()

    # Load files
    try:
        manifest = load_yaml(manifest_path)
    except FileNotFoundError:
        print(f"ERROR: Manifest not found: {manifest_path}")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"ERROR: Invalid YAML in manifest: {e}")
        sys.exit(1)

    try:
        schema = load_json(schema_path)
    except FileNotFoundError:
        print(f"WARNING: Schema not found: {schema_path}")
        print("Skipping schema validation")
        schema = None
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON in schema: {e}")
        sys.exit(1)

    all_errors = []
    all_warnings = []

    # Schema validation
    if schema:
        print("Checking JSON schema compliance...")
        schema_errors = validate_schema(manifest, schema)
        all_errors.extend(schema_errors)
        if not schema_errors:
            print("  Schema validation passed")
        else:
            print(f"  Found {len(schema_errors)} schema errors")

    # Semantic validations
    print("Checking unique IDs...")
    id_errors = validate_unique_ids(manifest)
    all_errors.extend(id_errors)
    if not id_errors:
        print("  All IDs are unique")
    else:
        print(f"  Found {len(id_errors)} duplicate IDs")

    print("Checking source file references...")
    source_errors = validate_sources_exist(manifest, root)
    all_warnings.extend(source_errors)  # Warnings, not errors
    if not source_errors:
        print("  All source files exist")
    else:
        print(f"  Found {len(source_errors)} missing files (warnings)")

    print("Checking profile references...")
    profile_errors = validate_profiles(manifest)
    all_errors.extend(profile_errors)
    if not profile_errors:
        print("  All profile references valid")
    else:
        print(f"  Found {len(profile_errors)} invalid profile references")

    print("Checking verify commands...")
    verify_errors = validate_verify_commands(manifest)
    all_errors.extend(verify_errors)
    if not verify_errors:
        print("  All verify commands valid")
    else:
        print(f"  Found {len(verify_errors)} invalid verify commands")

    # Summary
    print()
    print("=" * 60)

    # Count components
    nix_count = len(manifest.get("nix_components", []))
    pixi_count = len(manifest.get("pixi_components", []))
    docker_count = len(manifest.get("docker_services", []))
    cmd_count = len(manifest.get("commands", []))
    profile_count = len(manifest.get("profiles", {}))

    print(f"Components: {nix_count} Nix, {pixi_count} Pixi, {docker_count} Docker, {cmd_count} Commands")
    print(f"Profiles: {profile_count}")
    print()

    if all_warnings:
        print("WARNINGS:")
        for warning in all_warnings:
            print(f"  - {warning}")
        print()

    if all_errors:
        print("ERRORS:")
        for error in all_errors:
            print(f"  - {error}")
        print()
        print(f"Validation FAILED with {len(all_errors)} error(s)")
        sys.exit(1)
    else:
        print("Validation PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()
