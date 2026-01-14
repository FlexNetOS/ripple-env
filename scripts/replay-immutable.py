#!/usr/bin/env python3
"""Replay immutable evidence entries.

Today this is intentionally conservative and focuses on *preflight correctness*:
- validate that selected entries exist on disk (expanded) or that rollup metadata is present

As the repo gains real snapshot content, this script will be extended to:
- extract rollups into a temp dir
- run config validation / replay tasks against the extracted snapshot

Usage:
  python scripts/replay-immutable.py --ids '["2026-01-14/sha-abc123..."]'
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

INDEX_PATH = Path("docs/audits/immutable/index.yml")


def repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def load_index(root: Path) -> dict:
    p = root / INDEX_PATH
    if not p.exists():
        raise FileNotFoundError(str(p))
    data = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError("immutable index must be a mapping")
    return data


def by_id(index: dict) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for ent in index.get("entries", []) or []:
        if not isinstance(ent, dict):
            continue
        eid = ent.get("id")
        if isinstance(eid, str) and eid:
            out[eid] = ent
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Replay immutable evidence entries")
    ap.add_argument("--ids", required=True, help="JSON array of entry ids")
    args = ap.parse_args()

    try:
        ids = json.loads(args.ids)
    except Exception as e:
        print(f"Error: --ids must be JSON: {e}")
        return 2

    if not isinstance(ids, list) or not all(isinstance(x, str) for x in ids):
        print("Error: --ids must be a JSON array of strings")
        return 2

    root = repo_root()
    index = load_index(root)
    ent_map = by_id(index)

    failures = 0

    for eid in ids:
        ent = ent_map.get(eid)
        if not ent:
            print(f"❌ missing entry id in index.yml: {eid}")
            failures += 1
            continue

        storage = ent.get("storage") if isinstance(ent.get("storage"), dict) else {}
        stype = storage.get("type")

        if stype == "expanded":
            path = storage.get("path")
            if not isinstance(path, str) or not path:
                print(f"❌ {eid}: storage.path missing")
                failures += 1
                continue
            p = root / Path(path)
            if not p.exists():
                print(f"❌ {eid}: expanded path not found: {path}")
                failures += 1
                continue
            print(f"✅ {eid}: expanded snapshot present: {path}")

        elif stype == "rollup":
            archive_path = storage.get("archive_path")
            member_prefix = storage.get("member_prefix")
            if not isinstance(archive_path, str) or not archive_path:
                print(f"❌ {eid}: storage.archive_path missing")
                failures += 1
                continue
            if not isinstance(member_prefix, str) or not member_prefix:
                print(f"❌ {eid}: storage.member_prefix missing")
                failures += 1
                continue
            apath = root / Path(archive_path)
            if not apath.exists():
                print(f"❌ {eid}: rollup archive not found: {archive_path}")
                failures += 1
                continue
            print(f"✅ {eid}: rollup archive present: {archive_path} (member_prefix={member_prefix})")

        else:
            print(f"❌ {eid}: unknown storage.type: {stype!r}")
            failures += 1

    if failures:
        print(f"\nReplay preflight failures: {failures}")
        return 1

    print("\nReplay preflight OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
