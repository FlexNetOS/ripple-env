#!/usr/bin/env python3
"""
Dependency Usage Analytics for ripple-env

Analyzes which packages are actually used in the codebase to help identify:
- Unused dependencies that could be removed
- Missing dependencies that should be added
- Heavy dependencies that could be made optional

Usage:
    python scripts/analyze-dependencies.py [--verbose] [--format json|table]

Output:
    - Used packages with import count
    - Potentially unused packages
    - Suggestions for optimization
"""

import argparse
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Set, Tuple


def get_nix_packages() -> Set[str]:
    """Extract package names from nix/packages/*.nix files."""
    packages = set()
    nix_dir = Path("nix/packages")

    if not nix_dir.exists():
        print(f"Warning: {nix_dir} not found", file=sys.stderr)
        return packages

    # Common package patterns in Nix
    pkg_patterns = [
        r'\b([a-z][a-z0-9_-]+)\b',  # Package names
    ]

    for nix_file in nix_dir.glob("*.nix"):
        try:
            content = nix_file.read_text()
            # Look for package references (simplified)
            for line in content.split('\n'):
                line = line.strip()
                # Skip comments
                if line.startswith('#'):
                    continue
                # Find package names (words in lists)
                if 'pkgs.' in line or line.startswith('-'):
                    for match in re.finditer(r'pkgs\.([a-zA-Z][a-zA-Z0-9_-]*)', line):
                        packages.add(match.group(1))
                    # Direct package names in lists
                    for match in re.finditer(r'^\s*([a-z][a-z0-9_-]+)\s*$', line):
                        packages.add(match.group(1))
        except Exception as e:
            print(f"Warning: Could not parse {nix_file}: {e}", file=sys.stderr)

    return packages


def get_pixi_packages() -> Set[str]:
    """Extract package names from pixi.toml."""
    packages = set()
    pixi_file = Path("pixi.toml")

    if not pixi_file.exists():
        print(f"Warning: {pixi_file} not found", file=sys.stderr)
        return packages

    try:
        content = pixi_file.read_text()
        # Look for dependencies section
        in_deps = False
        for line in content.split('\n'):
            if '[dependencies]' in line or '[build-dependencies]' in line:
                in_deps = True
                continue
            if line.startswith('[') and in_deps:
                in_deps = False
            if in_deps and '=' in line:
                pkg_name = line.split('=')[0].strip().strip('"')
                packages.add(pkg_name)
    except Exception as e:
        print(f"Warning: Could not parse {pixi_file}: {e}", file=sys.stderr)

    return packages


def find_command_usages(packages: Set[str]) -> Dict[str, List[str]]:
    """Find where packages/commands are used in scripts and configs."""
    usages = defaultdict(list)

    # Directories to search
    search_dirs = [
        "scripts",
        ".github/workflows",
        "nix",
        "docker",
        "config",
    ]

    # File patterns to search
    extensions = [".sh", ".py", ".yml", ".yaml", ".nix", ".toml"]

    for search_dir in search_dirs:
        path = Path(search_dir)
        if not path.exists():
            continue

        for ext in extensions:
            for file in path.rglob(f"*{ext}"):
                try:
                    content = file.read_text()
                    for pkg in packages:
                        # Look for package name as command or reference
                        patterns = [
                            rf'\b{re.escape(pkg)}\b',  # Exact match
                            rf'pkgs\.{re.escape(pkg)}',  # Nix reference
                            rf'run: {re.escape(pkg)}',  # GitHub Actions
                        ]
                        for pattern in patterns:
                            if re.search(pattern, content, re.IGNORECASE):
                                usages[pkg].append(str(file))
                                break
                except Exception:
                    pass

    return dict(usages)


def find_python_imports() -> Dict[str, List[str]]:
    """Find Python import statements in the codebase."""
    imports = defaultdict(list)

    for py_file in Path(".").rglob("*.py"):
        if ".pixi" in str(py_file) or "node_modules" in str(py_file):
            continue
        try:
            content = py_file.read_text()
            for line in content.split('\n'):
                # Match import statements
                match = re.match(r'^(?:from\s+(\S+)|import\s+(\S+))', line.strip())
                if match:
                    module = match.group(1) or match.group(2)
                    base_module = module.split('.')[0]
                    imports[base_module].append(str(py_file))
        except Exception:
            pass

    return dict(imports)


def categorize_packages(
    nix_packages: Set[str],
    pixi_packages: Set[str],
    usages: Dict[str, List[str]]
) -> Tuple[Dict[str, List[str]], Set[str], Set[str]]:
    """Categorize packages as used, unused, or unknown."""
    all_packages = nix_packages | pixi_packages

    used = {}
    unused = set()

    for pkg in all_packages:
        if pkg in usages:
            used[pkg] = usages[pkg]
        else:
            unused.add(pkg)

    return used, unused, all_packages


def get_package_size_estimates() -> Dict[str, str]:
    """Rough size estimates for common heavy packages."""
    return {
        "firecracker": "~50MB",
        "vault": "~100MB",
        "kubectl": "~50MB",
        "kubernetes-helm": "~50MB",
        "local-ai": "~500MB+",
        "tensorflow": "~500MB+",
        "pytorch": "~1GB+",
        "ghc": "~500MB+",
        "kata-runtime": "~100MB",
        "containerd": "~50MB",
    }


def format_table(used: Dict[str, List[str]], unused: Set[str]) -> str:
    """Format results as a readable table."""
    lines = []

    lines.append("=" * 70)
    lines.append(" DEPENDENCY USAGE ANALYSIS")
    lines.append("=" * 70)
    lines.append("")

    # Used packages
    lines.append("USED PACKAGES")
    lines.append("-" * 70)
    lines.append(f"{'Package':<30} {'Usage Count':<15} {'Files'}")
    lines.append("-" * 70)

    for pkg in sorted(used.keys()):
        files = used[pkg]
        file_list = ", ".join(files[:3])
        if len(files) > 3:
            file_list += f" (+{len(files) - 3} more)"
        lines.append(f"{pkg:<30} {len(files):<15} {file_list}")

    lines.append("")

    # Unused packages
    lines.append("POTENTIALLY UNUSED PACKAGES")
    lines.append("-" * 70)
    lines.append("(No direct references found - may be used indirectly)")
    lines.append("")

    size_estimates = get_package_size_estimates()
    for pkg in sorted(unused):
        size = size_estimates.get(pkg, "")
        if size:
            lines.append(f"  - {pkg} ({size})")
        else:
            lines.append(f"  - {pkg}")

    lines.append("")

    # Summary
    lines.append("SUMMARY")
    lines.append("-" * 70)
    lines.append(f"  Total packages analyzed: {len(used) + len(unused)}")
    lines.append(f"  Used packages: {len(used)}")
    lines.append(f"  Potentially unused: {len(unused)}")
    lines.append("")

    # Recommendations
    heavy_unused = [p for p in unused if p in size_estimates]
    if heavy_unused:
        lines.append("RECOMMENDATIONS")
        lines.append("-" * 70)
        lines.append("Consider making these heavy packages optional:")
        for pkg in heavy_unused:
            lines.append(f"  - {pkg} ({size_estimates[pkg]})")
        lines.append("")
        lines.append("Suggestion: Use feature flags in nix/packages/dev-tools.nix:")
        lines.append('  { pkgs, withKubernetes ? false, withHeavyVMs ? false, ... }:')
        lines.append('  baseTools ++ lib.optionals withKubernetes [ kubectl helm ]')

    return "\n".join(lines)


def format_json(used: Dict[str, List[str]], unused: Set[str]) -> str:
    """Format results as JSON."""
    size_estimates = get_package_size_estimates()

    result = {
        "summary": {
            "total_packages": len(used) + len(unused),
            "used_packages": len(used),
            "unused_packages": len(unused),
        },
        "used": {
            pkg: {"files": files, "count": len(files)}
            for pkg, files in sorted(used.items())
        },
        "unused": sorted(list(unused)),
        "heavy_unused": [
            {"package": p, "estimated_size": size_estimates[p]}
            for p in sorted(unused)
            if p in size_estimates
        ],
    }

    return json.dumps(result, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze dependency usage in ripple-env"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show detailed analysis"
    )
    parser.add_argument(
        "--format", "-f",
        choices=["table", "json"],
        default="table",
        help="Output format (default: table)"
    )
    args = parser.parse_args()

    # Gather data
    print("Analyzing dependencies...", file=sys.stderr)

    nix_packages = get_nix_packages()
    print(f"  Found {len(nix_packages)} Nix packages", file=sys.stderr)

    pixi_packages = get_pixi_packages()
    print(f"  Found {len(pixi_packages)} Pixi packages", file=sys.stderr)

    all_packages = nix_packages | pixi_packages
    usages = find_command_usages(all_packages)
    print(f"  Found usages for {len(usages)} packages", file=sys.stderr)

    python_imports = find_python_imports()
    print(f"  Found {len(python_imports)} Python imports", file=sys.stderr)

    # Categorize
    used, unused, _ = categorize_packages(nix_packages, pixi_packages, usages)

    # Output
    print("", file=sys.stderr)

    if args.format == "json":
        print(format_json(used, unused))
    else:
        print(format_table(used, unused))


if __name__ == "__main__":
    main()
