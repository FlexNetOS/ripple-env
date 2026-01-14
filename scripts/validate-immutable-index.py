#!/usr/bin/env python3
"""Validate docs/audits/immutable/index.yml.

This is intentionally lightweight and conservative:
- validates structure against docs/audits/immutable/index.schema.json
- enforces unique entry ids
- optionally enforces POSIX-style paths (no backslashes) for portability

Usage:
  python scripts/validate-immutable-index.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator


def repo_root() -> Path:
    # This script lives in scripts/.
    return Path(__file__).resolve().parent.parent


def load_yaml(path: Path):
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at top-level")
    return data


def load_schema(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    p = argparse.ArgumentParser(description="Validate immutable evidence index")
    p.add_argument(
        "--index",
        default="docs/audits/immutable/index.yml",
        help="Path to immutable index YAML (default: docs/audits/immutable/index.yml)",
    )
    p.add_argument(
        "--schema",
        default="docs/audits/immutable/index.schema.json",
        help="Path to JSON schema (default: docs/audits/immutable/index.schema.json)",
    )
    p.add_argument(
        "--strict-paths",
        action="store_true",
        help="Fail if any storage paths contain backslashes",
    )
    args = p.parse_args()

    root = repo_root()
    index_path = (root / args.index).resolve()
    schema_path = (root / args.schema).resolve()

    if not index_path.exists():
        print(f"Error: index file not found: {index_path}")
        return 2
    if not schema_path.exists():
        print(f"Error: schema file not found: {schema_path}")
        return 2

    data = load_yaml(index_path)
    schema = load_schema(schema_path)

    v = Draft202012Validator(schema)
    errors = sorted(v.iter_errors(data), key=lambda e: list(e.absolute_path))
    if errors:
        print("❌ immutable index schema validation failed:")
        for e in errors:
            loc = "/".join(str(x) for x in e.absolute_path)
            print(f"  - {loc or '<root>'}: {e.message}")
        return 1

    entries = data.get("entries", [])
    if not isinstance(entries, list):
        print("❌ entries must be a list")
        return 1

    seen: set[str] = set()
    dupes: list[str] = []
    for ent in entries:
        if not isinstance(ent, dict):
            continue
        eid = ent.get("id")
        if isinstance(eid, str):
            if eid in seen:
                dupes.append(eid)
            seen.add(eid)

        if args.strict_paths:
            storage = ent.get("storage")
            if isinstance(storage, dict):
                for k in ("path", "archive_path", "member_prefix"):
                    vpath = storage.get(k)
                    if isinstance(vpath, str) and "\\" in vpath:
                        print(f"❌ entry {eid!r}: storage.{k} contains backslashes: {vpath}")
                        return 1

    if dupes:
        print("❌ duplicate entry ids:")
        for d in dupes:
            print(f"  - {d}")
        return 1

    print("✅ immutable index is valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
