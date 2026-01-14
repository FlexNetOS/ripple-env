#!/usr/bin/env python3
"""Hash ledger generator for ripple-env.

Generates sha256 ledgers based on tracked files in the Git index (`git ls-files -s`)
and the policy in `hashes.yml`.

Hashes are computed from Git blob contents (not working-tree bytes). This makes
the output deterministic across platforms and Git settings (e.g. CRLF/autocrlf).

Ledgers:
  - full    -> HASHES.full.txt
  - configs -> HASHES.configs.txt

Output format matches `docs/HASHES.txt`:
  <sha256-hex>␠␠./path/relative/to/repo

Usage examples:
  python scripts/generate-hashes.py --ledger full
  python scripts/generate-hashes.py --ledger configs
  python scripts/generate-hashes.py --all
  python scripts/generate-hashes.py --all --check

Notes:
  - Paths are normalized to POSIX separators.
  - Only tracked files are considered; untracked/build artifacts are ignored.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Iterable, Sequence

try:
    import yaml
except ImportError:
    print("Missing pyyaml. Install with: pip install -r scripts/requirements.txt")
    sys.exit(1)


@dataclass(frozen=True)
class LedgerPolicy:
    name: str
    include: list[str]
    exclude: list[str]


@dataclass(frozen=True)
class Policy:
    algorithm: str
    global_exclude: list[str]
    ledgers: dict[str, LedgerPolicy]


@dataclass(frozen=True)
class IndexEntry:
    path_posix: str
    blob_sha: str


def run_git(args: Sequence[str], repo_root: Path) -> bytes:
    return subprocess.check_output(["git", *args], cwd=str(repo_root))


def get_repo_root() -> Path:
    try:
        out = subprocess.check_output(["git", "rev-parse", "--show-toplevel"])
    except Exception:
        print("Error: not inside a git repository (git rev-parse failed)")
        sys.exit(2)

    root = out.decode("utf-8", errors="replace").strip()
    return Path(root)


def load_policy(path: Path) -> Policy:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("hashes.yml must be a YAML mapping")

    algo = data.get("algorithm", "sha256")
    if algo != "sha256":
        raise ValueError(f"Unsupported algorithm: {algo} (only sha256 supported)")

    global_exclude = data.get("exclude", []) or []
    if not isinstance(global_exclude, list):
        raise ValueError("hashes.yml: exclude must be a list")

    ledgers_raw = data.get("ledgers", {})
    if not isinstance(ledgers_raw, dict) or not ledgers_raw:
        raise ValueError("hashes.yml: ledgers must be a non-empty mapping")

    ledgers: dict[str, LedgerPolicy] = {}
    for name, v in ledgers_raw.items():
        if not isinstance(v, dict):
            raise ValueError(f"hashes.yml: ledgers.{name} must be a mapping")
        include = v.get("include", []) or []
        exclude = v.get("exclude", []) or []
        if not isinstance(include, list) or not isinstance(exclude, list):
            raise ValueError(f"hashes.yml: ledgers.{name}.include/exclude must be lists")
        ledgers[name] = LedgerPolicy(name=name, include=[str(x) for x in include], exclude=[str(x) for x in exclude])

    return Policy(algorithm=algo, global_exclude=[str(x) for x in global_exclude], ledgers=ledgers)


def normalize_path(p: str) -> str:
    # git returns POSIX paths; keep normalization defensive.
    p = p.replace("\\", "/")
    while p.startswith("./"):
        p = p[2:]
    return p


def path_matches_any(path_posix: str, patterns: Iterable[str]) -> bool:
    pp = PurePosixPath(path_posix)
    for pat in patterns:
        pat = normalize_path(str(pat))
        if not pat:
            continue
        # Treat '**' as "match everything" convenience.
        if pat == "**":
            return True
        try:
            if pp.match(pat):
                return True
        except Exception:
            # If a bad pattern slips in, fail closed by not matching.
            continue
    return False


def iter_index_entries(repo_root: Path) -> list[IndexEntry]:
    """Return tracked paths and blob SHAs from the index.

    We intentionally hash Git *blob contents* rather than working-tree files.
    This makes the ledger deterministic across platforms (e.g. CRLF vs LF)
    and ensures generation works even when a tracked file is missing on disk
    (sparse checkouts, deleted-but-not-staged files, etc.).
    """

    out = run_git(["ls-files", "-s", "-z"], repo_root)
    records = out.split(b"\x00")
    entries: list[IndexEntry] = []
    for rec in records:
        if not rec:
            continue

        # Format: "<mode> <sha> <stage>\t<path>"
        try:
            header, path_b = rec.split(b"\t", 1)
            parts = header.split()
            if len(parts) < 3:
                continue
            blob_sha = parts[1].decode("ascii", errors="replace")
            stage = parts[2].decode("ascii", errors="replace")
            # Only include the normal stage-0 entry.
            if stage != "0":
                continue
            path_posix = normalize_path(path_b.decode("utf-8", errors="replace"))
            if not path_posix:
                continue
            entries.append(IndexEntry(path_posix=path_posix, blob_sha=blob_sha))
        except Exception:
            continue

    return entries


def should_include(path_posix: str, include: list[str]) -> bool:
    if not include or include == ["**"] or "**" in include:
        return True
    return path_matches_any(path_posix, include)


def is_excluded(path_posix: str, excludes: list[str]) -> bool:
    return path_matches_any(path_posix, excludes)


def sha256_git_blob(repo_root: Path, blob_sha: str) -> str:
    """Compute sha256 of a Git blob's raw contents."""
    h = hashlib.sha256()

    proc = subprocess.Popen(
        ["git", "cat-file", "-p", blob_sha],
        cwd=str(repo_root),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert proc.stdout is not None

    for chunk in iter(lambda: proc.stdout.read(1024 * 1024), b""):
        h.update(chunk)

    rc = proc.wait()
    if rc != 0:
        err = b""
        if proc.stderr is not None:
            err = proc.stderr.read() or b""
        msg = err.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"git cat-file failed for {blob_sha}: {msg}")

    return h.hexdigest()


def generate_ledger_lines(repo_root: Path, entries: Sequence[IndexEntry]) -> list[str]:
    lines: list[str] = []
    for e in entries:
        digest = sha256_git_blob(repo_root, e.blob_sha)
        lines.append(f"{digest}  ./{e.path_posix}")
    return lines


def write_or_check(out_path: Path, content: str, check: bool) -> None:
    if check:
        existing = out_path.read_text(encoding="utf-8") if out_path.exists() else ""
        if existing != content:
            print(f"Ledger out of date: {out_path}")
            print("Re-generate with: python scripts/generate-hashes.py --all")
            sys.exit(3)
        return

    out_path.write_text(content, encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate hash ledgers for ripple-env")
    parser.add_argument("--config", default="hashes.yml", help="Path to hashes.yml (default: hashes.yml)")
    parser.add_argument("--ledger", choices=["full", "configs"], help="Generate a single ledger")
    parser.add_argument("--all", action="store_true", help="Generate all ledgers (full + configs)")
    parser.add_argument("--check", action="store_true", help="Fail if output differs from existing files")
    parser.add_argument("--stdout", action="store_true", help="Write ledger to stdout (only valid with --ledger)")

    args = parser.parse_args()

    if not args.ledger and not args.all:
        parser.error("Specify --ledger {full,configs} or --all")
    if args.stdout and not args.ledger:
        parser.error("--stdout requires --ledger")

    repo_root = get_repo_root()
    policy_path = (repo_root / args.config).resolve()
    if not policy_path.exists():
        print(f"Error: hashes policy not found: {policy_path}")
        return 2

    policy = load_policy(policy_path)
    tracked_entries = iter_index_entries(repo_root)

    def build_list(ledger_name: str) -> list[IndexEntry]:
        lp = policy.ledgers.get(ledger_name)
        if lp is None:
            raise ValueError(f"Ledger not defined in hashes.yml: {ledger_name}")

        selected: list[IndexEntry] = []
        for e in tracked_entries:
            rel = e.path_posix
            if is_excluded(rel, policy.global_exclude):
                continue
            if is_excluded(rel, lp.exclude):
                continue
            if not should_include(rel, lp.include):
                continue
            selected.append(e)

        selected.sort(key=lambda x: x.path_posix)
        return selected

    outputs = {
        "full": repo_root / "HASHES.full.txt",
        "configs": repo_root / "HASHES.configs.txt",
    }

    if args.ledger:
        entries = build_list(args.ledger)
        lines = generate_ledger_lines(repo_root, entries)
        content = "\n".join(lines) + ("\n" if lines else "")
        if args.stdout:
            sys.stdout.write(content)
        else:
            write_or_check(outputs[args.ledger], content, check=args.check)
        return 0

    # --all
    for ledger_name in ("full", "configs"):
        entries = build_list(ledger_name)
        lines = generate_ledger_lines(repo_root, entries)
        content = "\n".join(lines) + ("\n" if lines else "")
        write_or_check(outputs[ledger_name], content, check=args.check)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
