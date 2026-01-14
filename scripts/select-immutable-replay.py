#!/usr/bin/env python3
"""Select immutable evidence snapshot ids to replay.

Policy:
- Select entries whose *replay-relevant* fields changed between base and HEAD.
  Replay-relevant fields:
  - storage (type/path/archive_path/member_prefix)
  - integrity (entire object, if present)

This is designed to support "changed-only" replay selection.

Outputs:
- Prints a JSON array of selected entry ids to stdout.
- If running in GitHub Actions, also writes `snapshots=<json>` to $GITHUB_OUTPUT.

Usage:
  python scripts/select-immutable-replay.py
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import yaml

INDEX_PATH = "docs/audits/immutable/index.yml"


def repo_root() -> Path:
    out = subprocess.check_output(["git", "rev-parse", "--show-toplevel"])
    return Path(out.decode("utf-8", errors="replace").strip())


def run_git(args: list[str], cwd: Path) -> str:
    p = subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        capture_output=True,
        text=True,
    )
    if p.returncode != 0:
        raise RuntimeError(p.stderr.strip() or f"git {' '.join(args)} failed")
    return p.stdout


def load_yaml_text(text: str) -> dict:
    data = yaml.safe_load(text) or {}
    if not isinstance(data, dict):
        return {}
    return data


def load_index_from_worktree(root: Path) -> dict:
    p = root / INDEX_PATH
    if not p.exists():
        return {"version": 1, "entries": []}
    return load_yaml_text(p.read_text(encoding="utf-8"))


def load_index_from_ref(root: Path, ref: str) -> dict:
    try:
        text = run_git(["show", f"{ref}:{INDEX_PATH}"], cwd=root)
    except Exception:
        return {"version": 1, "entries": []}
    return load_yaml_text(text)


def entry_key_fields(ent: dict) -> dict:
    storage = ent.get("storage") if isinstance(ent.get("storage"), dict) else {}
    integrity = ent.get("integrity") if isinstance(ent.get("integrity"), dict) else {}

    # Only include replay-relevant fields.
    return {
        "storage": {
            "type": storage.get("type"),
            "path": storage.get("path"),
            "archive_path": storage.get("archive_path"),
            "member_prefix": storage.get("member_prefix"),
        },
        "integrity": integrity,
    }


def index_entries_by_id(index: dict) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for ent in index.get("entries", []) or []:
        if not isinstance(ent, dict):
            continue
        eid = ent.get("id")
        if isinstance(eid, str) and eid:
            out[eid] = ent
    return out


def detect_base_ref(root: Path) -> str:
    # PRs: compare against merge-base with origin/<base>
    base_branch = os.getenv("GITHUB_BASE_REF", "").strip()
    if base_branch:
        # Ensure origin/<base> exists (checkout should use fetch-depth: 0)
        base = f"origin/{base_branch}"
        try:
            mb = run_git(["merge-base", "HEAD", base], cwd=root).strip()
            if mb:
                return mb
        except Exception as exc:
            # Best-effort: if we cannot determine merge-base, fall back to other strategies.
            print(f"select-immutable-replay: failed to detect merge-base with {base!r}: {exc}", file=sys.stderr)

    # Push: compare against previous commit
    try:
        prev = run_git(["rev-parse", "HEAD~1"], cwd=root).strip()
        if prev:
            return prev
    except Exception as exc:
        # Best-effort: if previous commit cannot be resolved, fall back to HEAD.
        print("select-immutable-replay: failed to resolve previous commit (HEAD~1):", exc, file=sys.stderr)

    # Fallback: compare against HEAD (yields empty selection)
    return "HEAD"


def main() -> int:
    root = repo_root()

    base_ref = detect_base_ref(root)
    base_index = load_index_from_ref(root, base_ref)
    head_index = load_index_from_worktree(root)

    base_map = index_entries_by_id(base_index)
    head_map = index_entries_by_id(head_index)

    selected: list[str] = []

    for eid, head_ent in head_map.items():
        base_ent = base_map.get(eid)
        if base_ent is None:
            selected.append(eid)
            continue

        if entry_key_fields(base_ent) != entry_key_fields(head_ent):
            selected.append(eid)

    selected.sort()
    payload = json.dumps(selected)
    print(payload)

    gh_out = os.getenv("GITHUB_OUTPUT")
    if gh_out:
        with open(gh_out, "a", encoding="utf-8", newline="\n") as f:
            f.write(f"snapshots={payload}\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
