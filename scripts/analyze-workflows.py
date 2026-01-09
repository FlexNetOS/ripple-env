#!/usr/bin/env python3
"""
analyze-workflows.py - Deep analysis of GitHub workflow files

Usage: python3 scripts/analyze-workflows.py [command]

Commands:
  all           Full analysis of all workflows
  issues        Show potential issues
  nix           Show all nix commands used
  actions       Show all GitHub actions used
  deps          Show job dependencies
  matrix        Show matrix configurations
  envvars       Show environment variables
  permissions   Show permission requirements
"""

import os
import re
import sys
import yaml
import json
import sqlite3
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Any, Optional

# Paths
SCRIPT_DIR = Path(__file__).parent
REPO_ROOT = SCRIPT_DIR.parent
WORKFLOW_DIR = REPO_ROOT / ".github" / "workflows"
DB_PATH = REPO_ROOT / "data" / "config.db"

# Colors
class C:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    BOLD = '\033[1m'
    NC = '\033[0m'


def load_workflow(path: Path) -> Optional[Dict]:
    """Load and parse a workflow YAML file."""
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"{C.RED}Error loading {path}: {e}{C.NC}")
        return None


def get_all_workflows() -> Dict[str, Dict]:
    """Load all workflow files."""
    workflows = {}
    for path in WORKFLOW_DIR.glob("*.y*ml"):
        data = load_workflow(path)
        if data:
            workflows[path.name] = {
                "path": path,
                "data": data,
                "name": data.get("name", path.stem)
            }
    return workflows


def extract_nix_commands(workflow: Dict) -> List[Dict]:
    """Extract all nix-related commands from a workflow."""
    nix_commands = []
    jobs = workflow.get("data", {}).get("jobs", {})

    for job_name, job in jobs.items():
        steps = job.get("steps", [])
        for i, step in enumerate(steps):
            run_cmd = step.get("run", "")
            if any(cmd in run_cmd for cmd in ["nix ", "nix-", "nom ", "cachix"]):
                # Extract individual nix commands
                for line in run_cmd.split("\n"):
                    line = line.strip()
                    if line and any(cmd in line for cmd in ["nix ", "nix-", "nom ", "cachix"]):
                        nix_commands.append({
                            "job": job_name,
                            "step": step.get("name", f"step-{i}"),
                            "command": line[:100] + ("..." if len(line) > 100 else "")
                        })
    return nix_commands


def extract_actions(workflow: Dict) -> List[Dict]:
    """Extract all GitHub actions used."""
    actions = []
    jobs = workflow.get("data", {}).get("jobs", {})

    for job_name, job in jobs.items():
        steps = job.get("steps", [])
        for step in steps:
            uses = step.get("uses", "")
            if uses:
                actions.append({
                    "job": job_name,
                    "step": step.get("name", "unnamed"),
                    "action": uses
                })
    return actions


def extract_job_deps(workflow: Dict) -> List[Dict]:
    """Extract job dependency graph."""
    deps = []
    jobs = workflow.get("data", {}).get("jobs", {})

    for job_name, job in jobs.items():
        needs = job.get("needs", [])
        if isinstance(needs, str):
            needs = [needs]
        deps.append({
            "job": job_name,
            "needs": needs,
            "runs_on": job.get("runs-on", "unknown"),
            "if": job.get("if", "")[:50] if job.get("if") else ""
        })
    return deps


def extract_env_vars(workflow: Dict) -> Dict[str, List[str]]:
    """Extract environment variables."""
    env_vars = defaultdict(list)
    data = workflow.get("data", {})

    # Workflow-level env
    for key in data.get("env", {}).keys():
        env_vars["workflow"].append(key)

    # Job-level env
    jobs = data.get("jobs", {})
    for job_name, job in jobs.items():
        for key in job.get("env", {}).keys():
            env_vars[f"job:{job_name}"].append(key)

        # Step-level env
        for step in job.get("steps", []):
            for key in step.get("env", {}).keys():
                env_vars[f"step:{job_name}"].append(key)

    return dict(env_vars)


def extract_permissions(workflow: Dict) -> Dict:
    """Extract permission requirements."""
    data = workflow.get("data", {})
    perms = {}

    # Workflow-level permissions
    if "permissions" in data:
        perms["workflow"] = data["permissions"]

    # Job-level permissions
    jobs = data.get("jobs", {})
    for job_name, job in jobs.items():
        if "permissions" in job:
            perms[f"job:{job_name}"] = job["permissions"]

    return perms


def detect_issues(workflow: Dict) -> List[Dict]:
    """Detect potential issues in a workflow."""
    issues = []
    data = workflow.get("data", {})
    jobs = data.get("jobs", {})
    workflow_name = workflow.get("name", "unknown")

    # Check for nix commands without cache setup
    has_nix_cache = False
    has_nix_command = False

    for job_name, job in jobs.items():
        steps = job.get("steps", [])
        step_names = [s.get("uses", "") + s.get("name", "") for s in steps]

        # Check for nix cache
        if any("magic-nix-cache" in s or "cachix" in s for s in step_names):
            has_nix_cache = True

        for step in steps:
            run_cmd = step.get("run", "")
            uses = step.get("uses", "")

            # Check for nix usage
            if "nix " in run_cmd or "nix-" in run_cmd or "nom " in run_cmd:
                has_nix_command = True

            # Check for potential issues in run commands
            if "nix flake check" in run_cmd and "--no-build" not in run_cmd:
                issues.append({
                    "severity": "warning",
                    "job": job_name,
                    "issue": "nix flake check without --no-build may be slow",
                    "suggestion": "Add --no-build flag for faster CI"
                })

            if "nix develop" in run_cmd and ".#" not in run_cmd:
                issues.append({
                    "severity": "info",
                    "job": job_name,
                    "issue": "nix develop without explicit shell selector",
                    "suggestion": "Consider using .#default or .#full explicitly"
                })

            # Check for hardcoded versions in actions
            if uses and "@" in uses:
                version = uses.split("@")[1]
                if version in ["master", "main"]:
                    issues.append({
                        "severity": "warning",
                        "job": job_name,
                        "issue": f"Action {uses} uses branch instead of version tag",
                        "suggestion": "Pin to a specific version for reproducibility"
                    })

        # Check job dependencies
        needs = job.get("needs", [])
        if isinstance(needs, str):
            needs = [needs]
        for need in needs:
            if need not in jobs:
                issues.append({
                    "severity": "error",
                    "job": job_name,
                    "issue": f"Job depends on non-existent job: {need}",
                    "suggestion": "Fix the 'needs' reference"
                })

        # Check for missing checkout
        has_checkout = any("checkout" in s.get("uses", "") for s in steps)
        has_run = any(s.get("run") for s in steps)
        # Detect summary jobs that only write to GITHUB_STEP_SUMMARY (don't need checkout)
        run_content = " ".join(s.get("run") or "" for s in steps)
        # Check for file access commands at start of line or after shell operators
        uses_file_access = bool(re.search(r'(^|\n|;|\||&&)\s*(cat|ls|head|tail|grep|find)\s', run_content))
        is_summary_only = ("$GITHUB_STEP_SUMMARY" in run_content and not uses_file_access)
        if has_run and not has_checkout and not is_summary_only:
            issues.append({
                "severity": "warning",
                "job": job_name,
                "issue": "Job has run steps but no checkout action",
                "suggestion": "Add actions/checkout if accessing repo files"
            })

    # Check for nix without cache (only if using nix-installer-action)
    # Skip if nix commands are inside WSL/VM scripts (not GitHub Actions native nix)
    has_nix_installer = any(
        "nix-installer-action" in step.get("uses", "")
        for job in jobs.values()
        for step in job.get("steps", [])
    )
    if has_nix_command and not has_nix_cache and has_nix_installer:
        issues.append({
            "severity": "warning",
            "job": "workflow",
            "issue": "Workflow uses nix but has no cache setup",
            "suggestion": "Add DeterminateSystems/magic-nix-cache-action"
        })

    return issues


def analyze_all():
    """Full analysis of all workflows."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Workflow Analysis Report{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    all_issues = []

    for filename, workflow in sorted(workflows.items()):
        print(f"{C.BLUE}## {workflow['name']}{C.NC} ({filename})")
        print()

        data = workflow["data"]
        jobs = data.get("jobs", {})

        # Basic stats
        print(f"  Jobs: {len(jobs)}")
        triggers = []
        on_config = data.get("on", {})
        if isinstance(on_config, dict):
            triggers = list(on_config.keys())
        elif isinstance(on_config, list):
            triggers = on_config
        print(f"  Triggers: {', '.join(triggers)}")

        # Nix commands
        nix_cmds = extract_nix_commands(workflow)
        if nix_cmds:
            print(f"  Nix commands: {len(nix_cmds)}")

        # Issues
        issues = detect_issues(workflow)
        if issues:
            print(f"  {C.YELLOW}Issues: {len(issues)}{C.NC}")
            for issue in issues:
                all_issues.append({"workflow": filename, **issue})

        print()

    # Summary
    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Summary{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    print(f"Total workflows: {len(workflows)}")
    print(f"Total issues found: {len(all_issues)}")

    if all_issues:
        errors = [i for i in all_issues if i["severity"] == "error"]
        warnings = [i for i in all_issues if i["severity"] == "warning"]
        infos = [i for i in all_issues if i["severity"] == "info"]

        if errors:
            print(f"  {C.RED}Errors: {len(errors)}{C.NC}")
        if warnings:
            print(f"  {C.YELLOW}Warnings: {len(warnings)}{C.NC}")
        if infos:
            print(f"  {C.BLUE}Info: {len(infos)}{C.NC}")


def show_issues():
    """Show all detected issues."""
    workflows = get_all_workflows()

    print(f"\n{C.YELLOW}{'='*60}{C.NC}")
    print(f"{C.BOLD}Workflow Issues{C.NC}")
    print(f"{C.YELLOW}{'='*60}{C.NC}\n")

    all_issues = []
    for filename, workflow in sorted(workflows.items()):
        issues = detect_issues(workflow)
        for issue in issues:
            all_issues.append({"workflow": filename, **issue})

    # Group by severity
    errors = [i for i in all_issues if i["severity"] == "error"]
    warnings = [i for i in all_issues if i["severity"] == "warning"]
    infos = [i for i in all_issues if i["severity"] == "info"]

    if errors:
        print(f"{C.RED}## ERRORS ({len(errors)}){C.NC}\n")
        for i in errors:
            print(f"  [{i['workflow']}] {i['job']}")
            print(f"    Issue: {i['issue']}")
            print(f"    Fix: {i['suggestion']}")
            print()

    if warnings:
        print(f"{C.YELLOW}## WARNINGS ({len(warnings)}){C.NC}\n")
        for i in warnings:
            print(f"  [{i['workflow']}] {i['job']}")
            print(f"    Issue: {i['issue']}")
            print(f"    Fix: {i['suggestion']}")
            print()

    if infos:
        print(f"{C.BLUE}## INFO ({len(infos)}){C.NC}\n")
        for i in infos:
            print(f"  [{i['workflow']}] {i['job']}")
            print(f"    Issue: {i['issue']}")
            print(f"    Fix: {i['suggestion']}")
            print()

    if not all_issues:
        print(f"{C.GREEN}No issues detected!{C.NC}")


def show_nix():
    """Show all nix commands."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Nix Commands in Workflows{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    for filename, workflow in sorted(workflows.items()):
        nix_cmds = extract_nix_commands(workflow)
        if nix_cmds:
            print(f"{C.BLUE}## {workflow['name']}{C.NC} ({filename})")
            for cmd in nix_cmds:
                print(f"  [{cmd['job']}] {cmd['command']}")
            print()


def show_actions():
    """Show all GitHub actions used."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}GitHub Actions Used{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    all_actions = defaultdict(list)

    for filename, workflow in sorted(workflows.items()):
        actions = extract_actions(workflow)
        for action in actions:
            all_actions[action["action"]].append(filename)

    for action, files in sorted(all_actions.items()):
        print(f"  {action}")
        print(f"    Used in: {', '.join(set(files))}")
        print()


def show_deps():
    """Show job dependencies."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Job Dependencies{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    for filename, workflow in sorted(workflows.items()):
        deps = extract_job_deps(workflow)
        if deps:
            print(f"{C.BLUE}## {workflow['name']}{C.NC} ({filename})")
            for dep in deps:
                needs_str = ", ".join(dep["needs"]) if dep["needs"] else "(none)"
                cond_str = f" [if: {dep['if']}]" if dep["if"] else ""
                print(f"  {dep['job']} <- {needs_str} ({dep['runs_on']}){cond_str}")
            print()


def show_envvars():
    """Show environment variables."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Environment Variables{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    for filename, workflow in sorted(workflows.items()):
        env_vars = extract_env_vars(workflow)
        if env_vars:
            print(f"{C.BLUE}## {workflow['name']}{C.NC} ({filename})")
            for scope, vars in env_vars.items():
                print(f"  [{scope}] {', '.join(vars)}")
            print()


def show_permissions():
    """Show permission requirements."""
    workflows = get_all_workflows()

    print(f"\n{C.CYAN}{'='*60}{C.NC}")
    print(f"{C.BOLD}Permission Requirements{C.NC}")
    print(f"{C.CYAN}{'='*60}{C.NC}\n")

    for filename, workflow in sorted(workflows.items()):
        perms = extract_permissions(workflow)
        if perms:
            print(f"{C.BLUE}## {workflow['name']}{C.NC} ({filename})")
            for scope, perm in perms.items():
                if isinstance(perm, dict):
                    perm_str = ", ".join(f"{k}:{v}" for k, v in perm.items())
                else:
                    perm_str = str(perm)
                print(f"  [{scope}] {perm_str}")
            print()


def store_to_db():
    """Store analysis results to database."""
    workflows = get_all_workflows()

    conn = sqlite3.connect(DB_PATH)

    # Clear existing workflow data
    conn.execute("DELETE FROM workflow_steps")
    conn.execute("DELETE FROM workflow_jobs")
    conn.execute("DELETE FROM workflow_secrets")
    conn.execute("DELETE FROM workflows")
    conn.execute("DELETE FROM config_issues WHERE source='workflow'")

    for filename, workflow in workflows.items():
        data = workflow["data"]
        jobs = data.get("jobs", {})

        # Get triggers
        triggers = []
        on_config = data.get("on", {})
        if isinstance(on_config, dict):
            triggers = list(on_config.keys())
        elif isinstance(on_config, list):
            triggers = on_config

        # Insert workflow
        conn.execute(
            "INSERT INTO workflows (name, file_path, trigger_events, is_active) VALUES (?, ?, ?, 1)",
            (workflow["name"], f".github/workflows/{filename}", json.dumps(triggers))
        )
        workflow_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

        # Insert jobs
        for job_name, job in jobs.items():
            needs = job.get("needs", [])
            if isinstance(needs, str):
                needs = [needs]

            conn.execute(
                "INSERT INTO workflow_jobs (workflow_id, name, runs_on, needs, if_condition) VALUES (?, ?, ?, ?, ?)",
                (workflow_id, job_name, job.get("runs-on", ""), json.dumps(needs), job.get("if", ""))
            )
            job_id = conn.execute("SELECT last_insert_rowid()").fetchone()[0]

            # Insert steps
            for i, step in enumerate(job.get("steps", [])):
                conn.execute(
                    """INSERT INTO workflow_steps
                       (job_id, step_order, name, uses_action, run_command, env_vars, with_params)
                       VALUES (?, ?, ?, ?, ?, ?, ?)""",
                    (job_id, i, step.get("name", ""), step.get("uses", ""),
                     step.get("run", "")[:500], json.dumps(step.get("env", {})),
                     json.dumps(step.get("with", {})))
                )

        # Extract and store secrets
        content = workflow["path"].read_text()
        secrets = set(re.findall(r'\$\{\{\s*secrets\.([A-Z0-9_]+)', content))
        for secret in secrets:
            conn.execute(
                "INSERT INTO workflow_secrets (workflow_id, secret_name) VALUES (?, ?)",
                (workflow_id, secret)
            )

        # Store issues
        issues = detect_issues(workflow)
        for issue in issues:
            conn.execute(
                """INSERT INTO config_issues (source, severity, issue_type, description, file_path)
                   VALUES (?, ?, ?, ?, ?)""",
                ("workflow", issue["severity"], "workflow_issue",
                 f"[{issue['job']}] {issue['issue']} - {issue['suggestion']}",
                 f".github/workflows/{filename}")
            )

    conn.commit()
    conn.close()
    print(f"{C.GREEN}Stored analysis to database{C.NC}")


def show_help():
    print(__doc__)


def main():
    commands = {
        "all": analyze_all,
        "issues": show_issues,
        "nix": show_nix,
        "actions": show_actions,
        "deps": show_deps,
        "envvars": show_envvars,
        "permissions": show_permissions,
        "store": store_to_db,
        "help": show_help,
        "--help": show_help,
        "-h": show_help,
    }

    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"

    if cmd in commands:
        commands[cmd]()
    else:
        print(f"Unknown command: {cmd}")
        show_help()


if __name__ == "__main__":
    main()
