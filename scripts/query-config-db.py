#!/usr/bin/env python3
"""
query-config-db.py - Query the configuration database

Usage: python3 scripts/query-config-db.py <command> [args]
"""

import sqlite3
import sys
import json
from pathlib import Path
from typing import List, Tuple, Any

# Paths
SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent
DB_PATH = REPO_ROOT / "data" / "config.db"

# Colors
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    BOLD = '\033[1m'
    NC = '\033[0m'


def check_db():
    """Check if database exists."""
    if not DB_PATH.exists():
        print(f"{Colors.RED}Database not found.{Colors.NC} Run: python3 scripts/populate-config-db.py")
        sys.exit(1)


def get_conn():
    """Get database connection."""
    return sqlite3.connect(DB_PATH)


def print_table(headers: List[str], rows: List[Tuple[Any, ...]]):
    """Print formatted table."""
    if not rows:
        print("  (no results)")
        return

    # Calculate column widths
    widths = [len(h) for h in headers]
    for row in rows:
        for i, cell in enumerate(row):
            widths[i] = max(widths[i], len(str(cell) if cell else ""))

    # Print header
    header_line = " | ".join(h.ljust(widths[i]) for i, h in enumerate(headers))
    print(f"  {header_line}")
    print(f"  {'-+-'.join('-' * w for w in widths)}")

    # Print rows
    for row in rows:
        row_line = " | ".join(str(cell if cell else "").ljust(widths[i]) for i, cell in enumerate(row))
        print(f"  {row_line}")


def cmd_inputs():
    """List all flake inputs."""
    print(f"{Colors.CYAN}=== Flake Inputs ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("""
        SELECT name, url, follows, COALESCE(locked_rev, 'UNLOCKED') as rev
        FROM flake_inputs ORDER BY name
    """)
    print_table(["Name", "URL", "Follows", "Locked Rev"], cursor.fetchall())
    conn.close()


def cmd_inputs_unlocked():
    """List inputs without lock entries."""
    print(f"{Colors.YELLOW}=== Unlocked Flake Inputs ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT name, url FROM flake_inputs WHERE locked_rev IS NULL")
    print_table(["Name", "URL"], cursor.fetchall())
    conn.close()


def cmd_workflows():
    """List all workflows."""
    print(f"{Colors.CYAN}=== Workflows ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT name, file_path, trigger_events FROM workflows ORDER BY name")
    print_table(["Name", "File", "Triggers"], cursor.fetchall())
    conn.close()


def cmd_jobs(workflow_filter: str = None):
    """List workflow jobs."""
    conn = get_conn()
    if workflow_filter:
        print(f"{Colors.CYAN}=== Jobs matching '{workflow_filter}' ==={Colors.NC}")
        cursor = conn.execute("""
            SELECT w.name, j.name, j.runs_on
            FROM workflow_jobs j JOIN workflows w ON j.workflow_id = w.id
            WHERE w.name LIKE ? OR w.file_path LIKE ?
            ORDER BY w.name, j.name
        """, (f"%{workflow_filter}%", f"%{workflow_filter}%"))
    else:
        print(f"{Colors.CYAN}=== All Workflow Jobs ==={Colors.NC}")
        cursor = conn.execute("""
            SELECT w.name, j.name, j.runs_on
            FROM workflow_jobs j JOIN workflows w ON j.workflow_id = w.id
            ORDER BY w.name, j.name
        """)
    print_table(["Workflow", "Job", "Runs On"], cursor.fetchall())
    conn.close()


def cmd_secrets():
    """List secrets by workflow."""
    print(f"{Colors.CYAN}=== Secrets Required by Workflows ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("""
        SELECT w.name, GROUP_CONCAT(s.secret_name, ', ')
        FROM workflow_secrets s
        JOIN workflows w ON s.workflow_id = w.id
        GROUP BY w.id ORDER BY w.name
    """)
    print_table(["Workflow", "Secrets"], cursor.fetchall())
    conn.close()


def cmd_secrets_list():
    """List all unique secrets."""
    print(f"{Colors.CYAN}=== All Unique Secrets ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT DISTINCT secret_name FROM workflow_secrets ORDER BY secret_name")
    print_table(["Secret Name"], cursor.fetchall())
    conn.close()


def cmd_references():
    """List reference sources."""
    print(f"{Colors.CYAN}=== Reference Sources ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT name, category, url FROM reference_sources ORDER BY category, name")
    print_table(["Name", "Category", "URL"], cursor.fetchall())
    conn.close()


def cmd_capabilities():
    """List capability categories."""
    print(f"{Colors.CYAN}=== Capability Registry ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT name, api_count, use_cases FROM capability_categories ORDER BY api_count DESC")
    print_table(["Category", "API Count", "Use Cases"], cursor.fetchall())
    conn.close()


def cmd_capabilities_total():
    """Show total API count."""
    print(f"{Colors.CYAN}=== Total APIs ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("SELECT SUM(api_count), COUNT(*) FROM capability_categories")
    total, categories = cursor.fetchone()
    print(f"  Total APIs: {total or 0}")
    print(f"  Categories: {categories}")
    conn.close()


def cmd_issues():
    """Show unresolved issues."""
    print(f"{Colors.YELLOW}=== Unresolved Issues ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("""
        SELECT severity, source, issue_type, description, file_path
        FROM config_issues WHERE is_resolved = 0
        ORDER BY CASE severity WHEN 'error' THEN 1 WHEN 'warning' THEN 2 ELSE 3 END
    """)
    print_table(["Severity", "Source", "Type", "Description", "File"], cursor.fetchall())
    conn.close()


def cmd_issues_errors():
    """Show only errors."""
    print(f"{Colors.RED}=== Errors Only ==={Colors.NC}")
    conn = get_conn()
    cursor = conn.execute("""
        SELECT source, description, file_path
        FROM config_issues WHERE is_resolved = 0 AND severity = 'error'
    """)
    print_table(["Source", "Description", "File"], cursor.fetchall())
    conn.close()


def cmd_summary():
    """Database summary."""
    print(f"{Colors.CYAN}=== Database Summary ==={Colors.NC}\n")
    conn = get_conn()

    print(f"{Colors.BLUE}Flake Configuration:{Colors.NC}")
    cursor = conn.execute("SELECT COUNT(*) FROM flake_inputs")
    print(f"  Inputs: {cursor.fetchone()[0]}")
    cursor = conn.execute("SELECT COUNT(*) FROM flake_inputs WHERE locked_rev IS NOT NULL")
    print(f"  Locked: {cursor.fetchone()[0]}")

    print(f"\n{Colors.BLUE}GitHub Workflows:{Colors.NC}")
    cursor = conn.execute("SELECT COUNT(*) FROM workflows")
    print(f"  Workflows: {cursor.fetchone()[0]}")
    cursor = conn.execute("SELECT COUNT(*) FROM workflow_jobs")
    print(f"  Jobs: {cursor.fetchone()[0]}")
    cursor = conn.execute("SELECT COUNT(DISTINCT secret_name) FROM workflow_secrets")
    print(f"  Unique secrets: {cursor.fetchone()[0]}")

    print(f"\n{Colors.BLUE}References:{Colors.NC}")
    cursor = conn.execute("SELECT COUNT(*) FROM reference_sources")
    print(f"  Reference sources: {cursor.fetchone()[0]}")
    cursor = conn.execute("SELECT COALESCE(SUM(api_count), 0) FROM capability_categories")
    print(f"  APIs in registry: {cursor.fetchone()[0]}")

    print(f"\n{Colors.BLUE}Issues:{Colors.NC}")
    cursor = conn.execute("""
        SELECT severity, COUNT(*) FROM config_issues
        WHERE is_resolved = 0 GROUP BY severity
    """)
    issues = cursor.fetchall()
    if issues:
        for severity, count in issues:
            color = Colors.RED if severity == 'error' else Colors.YELLOW if severity == 'warning' else Colors.NC
            print(f"  {color}{severity}: {count}{Colors.NC}")
    else:
        print(f"  {Colors.GREEN}No issues!{Colors.NC}")

    conn.close()


def cmd_sql(query: str):
    """Run arbitrary SQL query."""
    conn = get_conn()
    try:
        cursor = conn.execute(query)
        if cursor.description:
            headers = [d[0] for d in cursor.description]
            print_table(headers, cursor.fetchall())
        else:
            print(f"  Rows affected: {cursor.rowcount}")
            conn.commit()
    except sqlite3.Error as e:
        print(f"{Colors.RED}SQL Error:{Colors.NC} {e}")
    finally:
        conn.close()


def cmd_validate():
    """Run validation checks and report."""
    print(f"{Colors.CYAN}=== Running Validation ==={Colors.NC}\n")
    conn = get_conn()

    # Check 1: Unlocked inputs
    cursor = conn.execute("SELECT name FROM flake_inputs WHERE locked_rev IS NULL")
    unlocked = cursor.fetchall()
    if unlocked:
        print(f"{Colors.YELLOW}[WARN]{Colors.NC} Unlocked flake inputs:")
        for (name,) in unlocked:
            print(f"  - {name}")
    else:
        print(f"{Colors.GREEN}[OK]{Colors.NC} All flake inputs are locked")

    # Check 2: Workflows needing secrets
    cursor = conn.execute("""
        SELECT DISTINCT s.secret_name
        FROM workflow_secrets s
        ORDER BY s.secret_name
    """)
    secrets = cursor.fetchall()
    if secrets:
        print(f"\n{Colors.BLUE}[INFO]{Colors.NC} Required secrets ({len(secrets)}):")
        for (secret,) in secrets:
            print(f"  - {secret}")

    # Check 3: Existing issues
    cursor = conn.execute("""
        SELECT severity, COUNT(*) FROM config_issues
        WHERE is_resolved = 0 GROUP BY severity
    """)
    issues = cursor.fetchall()
    if issues:
        print(f"\n{Colors.YELLOW}[WARN]{Colors.NC} Unresolved issues:")
        for severity, count in issues:
            print(f"  - {severity}: {count}")
    else:
        print(f"\n{Colors.GREEN}[OK]{Colors.NC} No configuration issues detected")

    conn.close()


def show_help():
    """Show help message."""
    print("""
Configuration Database Query Tool

Usage: python3 scripts/query-config-db.py <command> [args]

Commands:
  inputs              List all flake inputs
  inputs-unlocked     List inputs without lock entries
  workflows           List all workflows
  jobs [filter]       List workflow jobs (optionally filter)
  secrets             List secrets by workflow
  secrets-list        List all unique secrets
  references          List reference sources
  capabilities        List capability categories
  capabilities-total  Show total API count
  issues              Show unresolved issues
  issues-errors       Show only errors
  summary             Database summary
  validate            Run validation checks
  sql "QUERY"         Run arbitrary SQL query

Examples:
  python3 scripts/query-config-db.py summary
  python3 scripts/query-config-db.py jobs "verify"
  python3 scripts/query-config-db.py sql "SELECT * FROM flake_inputs"
""")


def main():
    check_db()

    if len(sys.argv) < 2:
        show_help()
        return

    cmd = sys.argv[1]
    args = sys.argv[2:]

    commands = {
        "inputs": cmd_inputs,
        "inputs-unlocked": cmd_inputs_unlocked,
        "workflows": cmd_workflows,
        "jobs": lambda: cmd_jobs(args[0] if args else None),
        "secrets": cmd_secrets,
        "secrets-list": cmd_secrets_list,
        "references": cmd_references,
        "capabilities": cmd_capabilities,
        "capabilities-total": cmd_capabilities_total,
        "issues": cmd_issues,
        "issues-errors": cmd_issues_errors,
        "summary": cmd_summary,
        "validate": cmd_validate,
        "sql": lambda: cmd_sql(args[0]) if args else print("Usage: sql 'QUERY'"),
        "help": show_help,
        "--help": show_help,
        "-h": show_help,
    }

    if cmd in commands:
        commands[cmd]()
    else:
        print(f"Unknown command: {cmd}")
        show_help()


if __name__ == "__main__":
    main()
