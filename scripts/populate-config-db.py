#!/usr/bin/env python3
"""
populate-config-db.py - Parse configs and populate SQLite database

Usage: python3 scripts/populate-config-db.py [--reset]
"""

import sqlite3
import json
import re
import sys
import os
from pathlib import Path
from datetime import datetime

# Paths
SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent
DB_PATH = REPO_ROOT / "data" / "config.db"
SCHEMA_PATH = REPO_ROOT / "data" / "schema.sql"

# Colors
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'

def log_info(msg): print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")
def log_success(msg): print(f"{Colors.GREEN}[OK]{Colors.NC} {msg}")
def log_warn(msg): print(f"{Colors.YELLOW}[WARN]{Colors.NC} {msg}")
def log_error(msg): print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}")


def init_db(reset=False):
    """Initialize the database with schema."""
    log_info(f"Initializing database at {DB_PATH}")

    if reset and DB_PATH.exists():
        log_warn("Resetting database...")
        DB_PATH.unlink()

    # Ensure data directory exists
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)

    conn = sqlite3.connect(DB_PATH)
    with open(SCHEMA_PATH) as f:
        conn.executescript(f.read())
    conn.commit()
    conn.close()
    log_success("Database schema applied")


def parse_flake_inputs(conn):
    """Parse flake.nix inputs section."""
    log_info("Parsing flake.nix inputs...")
    flake_file = REPO_ROOT / "flake.nix"

    if not flake_file.exists():
        log_error("flake.nix not found")
        return

    content = flake_file.read_text()

    # Parse simple url inputs: name.url = "...";
    simple_pattern = r'(\w[\w-]*?)\.url\s*=\s*"([^"]+)"'
    for match in re.finditer(simple_pattern, content):
        name, url = match.groups()
        conn.execute(
            "INSERT OR REPLACE INTO flake_inputs (name, url, is_active) VALUES (?, ?, 1)",
            (name, url)
        )
        log_success(f"  Found input: {name} -> {url}")

    # Parse block inputs with follows
    block_pattern = r'(\w[\w-]*?)\s*=\s*\{[^}]*url\s*=\s*"([^"]+)"[^}]*(?:inputs\.(\w+)\.follows)?[^}]*\}'
    for match in re.finditer(block_pattern, content, re.DOTALL):
        name, url = match.group(1), match.group(2)
        follows = match.group(3) if match.group(3) else None

        # Skip if already inserted with simple pattern
        existing = conn.execute("SELECT name FROM flake_inputs WHERE name=?", (name,)).fetchone()
        if existing:
            if follows:
                conn.execute("UPDATE flake_inputs SET follows=? WHERE name=?", (follows, name))
        else:
            conn.execute(
                "INSERT OR REPLACE INTO flake_inputs (name, url, follows, is_active) VALUES (?, ?, ?, 1)",
                (name, url, follows)
            )
            log_success(f"  Found input: {name} -> {url} (follows: {follows or 'none'})")

    conn.commit()
    parse_flake_lock(conn)


def parse_flake_lock(conn):
    """Parse flake.lock for revision info."""
    log_info("Parsing flake.lock...")
    lock_file = REPO_ROOT / "flake.lock"

    if not lock_file.exists():
        log_warn("flake.lock not found")
        return

    try:
        with open(lock_file) as f:
            lock_data = json.load(f)

        nodes = lock_data.get("nodes", {})
        for name, node in nodes.items():
            if name == "root":
                continue

            locked = node.get("locked", {})
            rev = locked.get("rev", "")
            lastmod = locked.get("lastModified", "")

            if lastmod and isinstance(lastmod, int):
                lastmod = datetime.fromtimestamp(lastmod).strftime("%Y-%m-%d")

            if rev:
                conn.execute(
                    "UPDATE flake_inputs SET locked_rev=?, locked_date=? WHERE name=?",
                    (rev, str(lastmod), name)
                )

        conn.commit()
        log_success("  Lock info updated")
    except json.JSONDecodeError as e:
        log_warn(f"  Failed to parse flake.lock: {e}")


def parse_workflows(conn):
    """Parse GitHub workflow files."""
    log_info("Parsing GitHub workflows...")
    workflow_dir = REPO_ROOT / ".github" / "workflows"

    if not workflow_dir.exists():
        log_warn("No .github/workflows directory found")
        return

    for workflow_file in workflow_dir.glob("*.y*ml"):
        content = workflow_file.read_text()
        filename = workflow_file.name

        # Extract name
        name_match = re.search(r'^name:\s*["\']?([^"\'\n]+)', content, re.MULTILINE)
        workflow_name = name_match.group(1).strip() if name_match else filename

        # Extract triggers
        triggers = []
        trigger_match = re.search(r'^on:\s*\n((?:\s+.+\n)*)', content, re.MULTILINE)
        if trigger_match:
            trigger_text = trigger_match.group(1)
            for trigger in ['push', 'pull_request', 'workflow_dispatch', 'schedule', 'release']:
                if re.search(rf'^\s+{trigger}:', trigger_text, re.MULTILINE):
                    triggers.append(trigger)

        # Insert workflow
        conn.execute(
            "INSERT OR REPLACE INTO workflows (name, file_path, trigger_events, is_active) VALUES (?, ?, ?, 1)",
            (workflow_name, f".github/workflows/{filename}", json.dumps(triggers))
        )
        log_success(f"  Found workflow: {workflow_name} ({filename})")

        # Get workflow ID
        cursor = conn.execute("SELECT id FROM workflows WHERE file_path=?", (f".github/workflows/{filename}",))
        workflow_id = cursor.fetchone()[0]

        # Extract jobs
        jobs_match = re.search(r'^jobs:\s*\n((?:\s+.+\n)*)', content, re.MULTILINE)
        if jobs_match:
            jobs_text = jobs_match.group(1)
            for job_match in re.finditer(r'^  (\w[\w-]*):\s*\n', jobs_text, re.MULTILINE):
                job_name = job_match.group(1)
                if job_name in ['name', 'on', 'env', 'permissions']:
                    continue

                # Get runs-on
                job_section = re.search(rf'^  {job_name}:.*?(?=^  \w|\Z)', content, re.MULTILINE | re.DOTALL)
                runs_on = ""
                if job_section:
                    runs_on_match = re.search(r'runs-on:\s*([^\n]+)', job_section.group())
                    if runs_on_match:
                        runs_on = runs_on_match.group(1).strip().strip('"\'')

                conn.execute(
                    "INSERT OR IGNORE INTO workflow_jobs (workflow_id, name, runs_on) VALUES (?, ?, ?)",
                    (workflow_id, job_name, runs_on)
                )

        # Extract secrets
        secrets = set(re.findall(r'\$\{\{\s*secrets\.([A-Z0-9_]+)', content))
        for secret in secrets:
            conn.execute(
                "INSERT OR IGNORE INTO workflow_secrets (workflow_id, secret_name) VALUES (?, ?)",
                (workflow_id, secret)
            )

    conn.commit()


def parse_references(conn):
    """Parse README reference sources."""
    log_info("Parsing README reference sources...")
    readme_file = REPO_ROOT / "README.md"

    if not readme_file.exists():
        log_warn("README.md not found")
        return

    content = readme_file.read_text()
    current_section = ""

    for line in content.split('\n'):
        # Track section headers
        section_match = re.match(r'^##\s+(.+)', line)
        if section_match:
            current_section = section_match.group(1)

        # Parse markdown table rows with links
        # Format: | **name** | description | [link](url) |
        row_match = re.match(r'\|\s*\*\*([^\*]+)\*\*\s*\|\s*([^\|]+)\s*\|\s*\[([^\]]+)\]\(([^\)]+)\)', line)
        if row_match:
            name, desc, _, url = row_match.groups()
            conn.execute(
                "INSERT OR REPLACE INTO reference_sources (name, description, url, category) VALUES (?, ?, ?, ?)",
                (name.strip(), desc.strip(), url.strip(), current_section)
            )

    conn.commit()
    cursor = conn.execute("SELECT COUNT(*) FROM reference_sources")
    count = cursor.fetchone()[0]
    log_success(f"  Loaded {count} reference sources")


def parse_capability_registry(conn):
    """Parse capability registry JSON."""
    log_info("Parsing capability registry...")
    registry_file = REPO_ROOT / "manifests" / "capability-registry" / "api-registry.json"

    if not registry_file.exists():
        log_warn("api-registry.json not found")
        return

    try:
        with open(registry_file) as f:
            registry = json.load(f)

        for cat in registry.get("categories", []):
            conn.execute(
                """INSERT OR REPLACE INTO capability_categories
                   (category_id, name, description, api_count, path, use_cases)
                   VALUES (?, ?, ?, ?, ?, ?)""",
                (cat["id"], cat["name"], cat.get("description", ""),
                 cat.get("count", 0), cat.get("path", ""),
                 json.dumps(cat.get("useCases", [])))
            )

        conn.commit()
        cursor = conn.execute("SELECT SUM(api_count) FROM capability_categories")
        total = cursor.fetchone()[0] or 0
        log_success(f"  Loaded capability registry with {total} APIs")
    except (json.JSONDecodeError, KeyError) as e:
        log_warn(f"  Failed to parse registry: {e}")


def run_validation(conn):
    """Run validation checks."""
    log_info("Running validation checks...")
    flake_file = REPO_ROOT / "flake.nix"
    content = flake_file.read_text()

    # Check for possibly unused inputs
    cursor = conn.execute("SELECT name FROM flake_inputs WHERE is_active=1")
    for (input_name,) in cursor.fetchall():
        ref_count = len(re.findall(re.escape(input_name), content))
        if ref_count < 3:
            conn.execute(
                """INSERT INTO config_issues (source, severity, issue_type, description, file_path)
                   VALUES (?, ?, ?, ?, ?)""",
                ('flake', 'warning', 'possibly_unused',
                 f'Input "{input_name}" may be unused (only {ref_count} references)', 'flake.nix')
            )
            log_warn(f"  Possibly unused input: {input_name}")

    conn.commit()
    log_success("Validation complete")


def print_summary(conn):
    """Print database summary."""
    print("\n" + "=" * 42)
    print("         Configuration Database          ")
    print("=" * 42 + "\n")
    print(f"Database: {DB_PATH}\n")
    print("Contents:")

    queries = [
        ("Flake inputs", "SELECT COUNT(*) FROM flake_inputs"),
        ("Workflows", "SELECT COUNT(*) FROM workflows"),
        ("Workflow jobs", "SELECT COUNT(*) FROM workflow_jobs"),
        ("Workflow secrets", "SELECT COUNT(*) FROM workflow_secrets"),
        ("Reference sources", "SELECT COUNT(*) FROM reference_sources"),
        ("API categories", "SELECT COUNT(*) FROM capability_categories"),
        ("Config issues", "SELECT COUNT(*) FROM config_issues WHERE is_resolved=0"),
    ]

    for label, query in queries:
        cursor = conn.execute(query)
        count = cursor.fetchone()[0]
        print(f"  {label + ':':<20} {count}")

    print("\nQuick queries:")
    print(f"  python3 scripts/query-config-db.py summary")
    print(f"  python3 scripts/query-config-db.py issues")
    print(f"  python3 scripts/query-config-db.py inputs")
    print()


def main():
    print(f"\n{Colors.BLUE}[INFO]{Colors.NC} Configuration Database Builder\n")

    reset = "--reset" in sys.argv

    init_db(reset)

    conn = sqlite3.connect(DB_PATH)
    try:
        parse_flake_inputs(conn)
        parse_workflows(conn)
        parse_references(conn)
        parse_capability_registry(conn)
        run_validation(conn)
        print_summary(conn)
    finally:
        conn.close()

    log_success("Done!")


if __name__ == "__main__":
    main()
