# Script Contract: query-config-db.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/query-config-db.sh`

---

## Purpose

Interactive command-line query tool for the configuration database. Provides pre-built commands for common queries (flake inputs, workflows, secrets, issues) and supports arbitrary SQL queries. Uses color-coded output for readability and offers an interactive SQLite shell for advanced queries. Companion script to `populate-config-db.sh`.

---

## Invocation

```bash
./scripts/query-config-db.sh <command> [args]
```

**Commands:**
- `inputs` - List all flake inputs with URLs and lock status
- `inputs-unlocked` - List inputs without locked revisions
- `workflows` - List all GitHub Actions workflows
- `jobs [workflow]` - List workflow jobs (optionally filter by workflow name)
- `secrets` - List secrets grouped by workflow
- `secrets-list` - List all unique secret names
- `references` - List README reference sources
- `capabilities` - List capability categories with API counts
- `capabilities-total` - Show total API count across all categories
- `issues` - Show unresolved configuration issues
- `issues-errors` - Show only error-level issues
- `summary` - Database summary with counts
- `sql "QUERY"` - Run arbitrary SQL query
- `shell` - Open interactive SQLite shell
- `help`, `--help`, `-h` - Show help message

**Examples:**
```bash
./scripts/query-config-db.sh inputs
./scripts/query-config-db.sh inputs-unlocked
./scripts/query-config-db.sh jobs "verify-ai"
./scripts/query-config-db.sh secrets
./scripts/query-config-db.sh issues
./scripts/query-config-db.sh summary
./scripts/query-config-db.sh sql "SELECT * FROM flake_inputs WHERE name LIKE '%nix%'"
./scripts/query-config-db.sh shell
```

**Requirements:**
- `sqlite3` command available
- Database file: `data/config.db` (created by `populate-config-db.sh`)

---

## Outputs

**Standard Output (inputs command):**
```
=== Flake Inputs ===
name              url                                      follows  rev
----------------  ---------------------------------------  -------  ----------
flake-parts       github:hercules-ci/flake-parts                    abc123def
home-manager      github:nix-community/home-manager        nixpkgs  def456ghi
nixpkgs           github:NixOS/nixpkgs/nixos-24.11                  ghi789jkl
pixi              github:prefix-dev/pixi                   nixpkgs  UNLOCKED
ros2              github:lopsided98/nix-ros-overlay                 jkl012mno
```

**Standard Output (workflows command):**
```
=== Workflows ===
name                      file_path                        trigger_events
------------------------  -------------------------------  ---------------------------
CI Build and Test         .github/workflows/ci.yml         [push,pull_request]
Container Security Scan   .github/workflows/security.yml   [schedule,workflow_dispatch]
Deploy to Production      .github/workflows/deploy.yml     [workflow_dispatch,release]
E2E Validation            .github/workflows/e2e.yml        [push,pull_request]
```

**Standard Output (issues command):**
```
=== Unresolved Issues ===
severity  source    issue_type       description                                     file_path
--------  --------  ---------------  ---------------------------------------------   -----------
warning   flake     possibly_unused  Input "example-input" may be unused             flake.nix
info      workflow  no_secrets       Workflow "E2E Validation" uses no secrets       .github/workflows/
```

**Standard Output (summary command):**
```
=== Database Summary ===

Flake Configuration:
count
--------
8 inputs
7 locked

GitHub Workflows:
count
--------------
5 workflows
18 jobs
12 unique secrets

References:
count
-----------------------------
42 reference sources
156 APIs in capability registry

Issues:
severity  count
--------  -----
warning   1
```

**Standard Output (shell command):**
```
Opening SQLite shell...
Tables: flake_inputs, workflows, workflow_jobs, workflow_secrets, reference_sources, capability_categories, config_issues
Views: v_flake_inputs_status, v_workflow_summary, v_unresolved_issues

SQLite version 3.45.1
Enter ".help" for usage hints.
sqlite>
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (query executed) |
| `1` | Failure (database not found, unknown command) |

---

## Side Effects

**None** - Read-only queries, no database modifications.

---

## Safety Classification

**🟢 SAFE** - Read-only database queries.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly with same results.

---

## Database Check

**Evidence:** Lines 20-25

```bash
check_db() {
    if [[ ! -f "$DB_PATH" ]]; then
        echo -e "${RED}Database not found.${NC} Run: ./scripts/populate-config-db.sh"
        exit 1
    fi
}
```

**Database location:** `data/config.db` (line 9)

**Prerequisite:** Must run `populate-config-db.sh` first

---

## Query Function

**Evidence:** Lines 27-30

```bash
# Pretty print query results
query() {
    sqlite3 -header -column "$DB_PATH" "$1"
}
```

**SQLite flags:**
- `-header` - Display column headers (line 29)
- `-column` - Column mode for aligned output (line 29)

**Alternative modes:**
- `-json` - JSON output
- `-csv` - CSV output
- `-html` - HTML output

---

## Flake Inputs Commands

### List All Inputs (lines 33-36)

**Evidence:**
```bash
cmd_inputs() {
    echo -e "${CYAN}=== Flake Inputs ===${NC}"
    query "SELECT name, url, follows, COALESCE(locked_rev, 'UNLOCKED') as rev FROM flake_inputs ORDER BY name;"
}
```

**Query features:**
- `COALESCE(locked_rev, 'UNLOCKED')` - Show "UNLOCKED" for NULL revisions
- `ORDER BY name` - Alphabetical sort

### List Unlocked Inputs (lines 38-41)

**Evidence:**
```bash
cmd_inputs_unlocked() {
    echo -e "${YELLOW}=== Unlocked Flake Inputs ===${NC}"
    query "SELECT name, url FROM flake_inputs WHERE locked_rev IS NULL;"
}
```

**Purpose:** Identify inputs without `flake.lock` entries

**Use case:** Security auditing, dependency tracking

---

## Workflow Commands

### List All Workflows (lines 43-46)

**Evidence:**
```bash
cmd_workflows() {
    echo -e "${CYAN}=== Workflows ===${NC}"
    query "SELECT name, file_path, trigger_events FROM workflows ORDER BY name;"
}
```

**Columns:**
- `name` - Workflow display name
- `file_path` - Relative path to YAML file
- `trigger_events` - JSON array of triggers (e.g., "[push,pull_request]")

### List Workflow Jobs (lines 48-63)

**Evidence:**
```bash
cmd_workflow_jobs() {
    local workflow="${1:-}"
    if [[ -n "$workflow" ]]; then
        # Escape single quotes to safely embed the workflow name in an SQL string literal
        local safe_workflow="${workflow//\'/\'\'}"
        echo -e "${CYAN}=== Jobs in '$workflow' ===${NC}"
        query "SELECT j.name, j.runs_on FROM workflow_jobs j
               JOIN workflows w ON j.workflow_id = w.id
               WHERE w.name LIKE '%$safe_workflow%' OR w.file_path LIKE '%$safe_workflow%';"
    else
        echo -e "${CYAN}=== All Workflow Jobs ===${NC}"
        query "SELECT w.name as workflow, j.name as job, j.runs_on
               FROM workflow_jobs j JOIN workflows w ON j.workflow_id = w.id
               ORDER BY w.name, j.name;"
    fi
}
```

**Modes:**
1. **Filtered** - With workflow argument (lines 50-56)
2. **All jobs** - Without argument (lines 58-61)

**SQL injection protection:** Escapes single quotes (line 52)

**Pattern matching:** `LIKE '%$safe_workflow%'` matches partial names

---

## Secrets Commands

### List Secrets by Workflow (lines 65-72)

**Evidence:**
```bash
cmd_secrets() {
    echo -e "${CYAN}=== Secrets Required by Workflows ===${NC}"
    query "SELECT w.name as workflow, GROUP_CONCAT(s.secret_name, ', ') as secrets
           FROM workflow_secrets s
           JOIN workflows w ON s.workflow_id = w.id
           GROUP BY w.id
           ORDER BY w.name;"
}
```

**SQL features:**
- `GROUP_CONCAT(s.secret_name, ', ')` - Joins multiple secrets with comma separator
- `GROUP BY w.id` - One row per workflow

**Example output:**
```
workflow                  secrets
------------------------  ---------------------------------
CI Build and Test         GITHUB_TOKEN, CODECOV_TOKEN
Deploy to Production      AWS_ACCESS_KEY, AWS_SECRET_KEY, GITHUB_TOKEN
```

### List All Unique Secrets (lines 74-77)

**Evidence:**
```bash
cmd_secrets_list() {
    echo -e "${CYAN}=== All Unique Secrets ===${NC}"
    query "SELECT DISTINCT secret_name FROM workflow_secrets ORDER BY secret_name;"
}
```

**Purpose:** Inventory all secrets across all workflows

**Use case:** Secret rotation, access control auditing

---

## Reference Sources Command

**Evidence:** Lines 79-82

```bash
cmd_references() {
    echo -e "${CYAN}=== Reference Sources ===${NC}"
    query "SELECT name, category, url FROM reference_sources ORDER BY category, name;"
}
```

**Source:** README.md table entries parsed by `populate-config-db.sh`

**Columns:**
- `name` - Resource name (e.g., "ROS2 Docs")
- `category` - Section header from README
- `url` - Documentation URL

---

## Capability Registry Commands

### List Categories (lines 84-87)

**Evidence:**
```bash
cmd_capabilities() {
    echo -e "${CYAN}=== Capability Registry ===${NC}"
    query "SELECT name, api_count, use_cases FROM capability_categories ORDER BY api_count DESC;"
}
```

**Sort:** By `api_count DESC` - Largest categories first

### Total API Count (lines 89-92)

**Evidence:**
```bash
cmd_capabilities_total() {
    echo -e "${CYAN}=== Total APIs ===${NC}"
    query "SELECT SUM(api_count) as total_apis, COUNT(*) as categories FROM capability_categories;"
}
```

**Aggregates:** Total APIs across all categories

---

## Issues Commands

### List Unresolved Issues (lines 94-99)

**Evidence:**
```bash
cmd_issues() {
    echo -e "${YELLOW}=== Unresolved Issues ===${NC}"
    query "SELECT severity, source, issue_type, description, file_path
           FROM config_issues WHERE is_resolved = 0
           ORDER BY CASE severity WHEN 'error' THEN 1 WHEN 'warning' THEN 2 ELSE 3 END;"
}
```

**Filters:** `is_resolved = 0` - Only active issues

**Sort:** By severity (error → warning → info) using `CASE` expression

### List Errors Only (lines 101-105)

**Evidence:**
```bash
cmd_issues_errors() {
    echo -e "${RED}=== Errors Only ===${NC}"
    query "SELECT source, description, file_path FROM config_issues
           WHERE is_resolved = 0 AND severity = 'error';"
}
```

**Purpose:** Focus on critical issues requiring immediate attention

---

## Summary Command

**Evidence:** Lines 107-128

```bash
cmd_summary() {
    echo -e "${CYAN}=== Database Summary ===${NC}"
    echo ""
    echo -e "${BLUE}Flake Configuration:${NC}"
    query "SELECT COUNT(*) || ' inputs' as count FROM flake_inputs;"
    query "SELECT COUNT(*) || ' locked' as count FROM flake_inputs WHERE locked_rev IS NOT NULL;"

    echo ""
    echo -e "${BLUE}GitHub Workflows:${NC}"
    query "SELECT COUNT(*) || ' workflows' as count FROM workflows;"
    query "SELECT COUNT(*) || ' jobs' as count FROM workflow_jobs;"
    query "SELECT COUNT(*) || ' unique secrets' as count FROM (SELECT DISTINCT secret_name FROM workflow_secrets);"

    echo ""
    echo -e "${BLUE}References:${NC}"
    query "SELECT COUNT(*) || ' reference sources' as count FROM reference_sources;"
    query "SELECT COALESCE(SUM(api_count), 0) || ' APIs in capability registry' as count FROM capability_categories;"

    echo ""
    echo -e "${BLUE}Issues:${NC}"
    query "SELECT severity, COUNT(*) as count FROM config_issues WHERE is_resolved = 0 GROUP BY severity;"
}
```

**Sections:**
1. **Flake Configuration** - Input counts (lines 110-112)
2. **GitHub Workflows** - Workflow/job/secret counts (lines 115-118)
3. **References** - Documentation and API counts (lines 121-123)
4. **Issues** - Issue breakdown by severity (lines 126-127)

**String concatenation:** `COUNT(*) || ' inputs'` - Appends label to count

---

## Arbitrary SQL Command

**Evidence:** Lines 130-133

```bash
cmd_sql() {
    # Run arbitrary SQL
    query "$1"
}
```

**Usage:**
```bash
./scripts/query-config-db.sh sql "SELECT name, url FROM flake_inputs WHERE name LIKE '%nix%'"
./scripts/query-config-db.sh sql "SELECT COUNT(*) FROM workflows WHERE trigger_events LIKE '%schedule%'"
```

**Power user feature** - Direct database access

---

## Interactive SQLite Shell

**Evidence:** Lines 135-141

```bash
cmd_shell() {
    echo -e "${CYAN}Opening SQLite shell...${NC}"
    echo "Tables: flake_inputs, workflows, workflow_jobs, workflow_secrets, reference_sources, capability_categories, config_issues"
    echo "Views: v_flake_inputs_status, v_workflow_summary, v_unresolved_issues"
    echo ""
    sqlite3 "$DB_PATH"
}
```

**Lists available tables and views** (lines 137-138)

**Shell commands:**
```sql
-- List tables
.tables

-- Schema
.schema flake_inputs

-- Exit
.quit

-- Export to CSV
.mode csv
.output workflows.csv
SELECT * FROM workflows;
.output stdout

-- Pretty formatting
.mode column
.headers on
SELECT * FROM flake_inputs;
```

---

## Color Coding

**Evidence:** Lines 11-17

```bash
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
```

**Usage patterns:**
- `CYAN` - Section headers (e.g., "=== Flake Inputs ===")
- `BLUE` - Subsection headers in summary
- `YELLOW` - Warnings (unlocked inputs, issues)
- `RED` - Errors (database not found, error-level issues)

---

## Command Dispatch

**Evidence:** Lines 175-199

```bash
main() {
    check_db

    local cmd="${1:-help}"
    shift || true

    case "$cmd" in
        inputs)           cmd_inputs ;;
        inputs-unlocked)  cmd_inputs_unlocked ;;
        workflows)        cmd_workflows ;;
        jobs)             cmd_workflow_jobs "$@" ;;
        secrets)          cmd_secrets ;;
        secrets-list)     cmd_secrets_list ;;
        references)       cmd_references ;;
        capabilities)     cmd_capabilities ;;
        capabilities-total) cmd_capabilities_total ;;
        issues)           cmd_issues ;;
        issues-errors)    cmd_issues_errors ;;
        summary)          cmd_summary ;;
        sql)              cmd_sql "$1" ;;
        shell)            cmd_shell ;;
        help|--help|-h)   show_help ;;
        *)                echo "Unknown command: $cmd"; show_help; exit 1 ;;
    esac
}
```

**Default command:** `help` if no arguments (line 178)

**Argument forwarding:**
- `"$@"` - All remaining arguments to `jobs` command (line 185)
- `"$1"` - First argument to `sql` command (line 194)

---

## Example Queries

### Find Workflows Using Specific Secret

```bash
./scripts/query-config-db.sh sql "
SELECT w.name, w.file_path
FROM workflows w
JOIN workflow_secrets s ON w.id = s.workflow_id
WHERE s.secret_name = 'GITHUB_TOKEN'
"
```

### Find Unlocked Inputs with Follows

```bash
./scripts/query-config-db.sh sql "
SELECT name, url, follows
FROM flake_inputs
WHERE locked_rev IS NULL AND follows IS NOT NULL
"
```

### Count Jobs by Runner

```bash
./scripts/query-config-db.sh sql "
SELECT runs_on, COUNT(*) as job_count
FROM workflow_jobs
GROUP BY runs_on
ORDER BY job_count DESC
"
```

### Find Most Common Secrets

```bash
./scripts/query-config-db.sh sql "
SELECT secret_name, COUNT(*) as workflow_count
FROM workflow_secrets
GROUP BY secret_name
ORDER BY workflow_count DESC
"
```

### Audit Workflow Triggers

```bash
./scripts/query-config-db.sh sql "
SELECT name, trigger_events
FROM workflows
WHERE trigger_events LIKE '%schedule%'
"
```

---

## Integration with populate-config-db.sh

**Workflow:**
```bash
# 1. Populate database
./scripts/populate-config-db.sh

# 2. Query database
./scripts/query-config-db.sh summary
./scripts/query-config-db.sh issues

# 3. Update after config changes
./scripts/populate-config-db.sh

# 4. Re-query
./scripts/query-config-db.sh inputs-unlocked
```

**Typical use cases:**
1. **Initial audit** - Run `summary`, `issues`, `secrets-list`
2. **Dependency review** - Run `inputs`, `inputs-unlocked`
3. **CI/CD analysis** - Run `workflows`, `jobs`, `secrets`
4. **Documentation inventory** - Run `references`, `capabilities`
5. **Custom queries** - Use `sql` or `shell` commands

---

## Troubleshooting

### Database Not Found

**Symptoms:** "Database not found" error (line 22)

**Fix:**
```bash
# Populate database first
./scripts/populate-config-db.sh

# Then query
./scripts/query-config-db.sh summary
```

### SQLite Command Not Found

**Symptoms:** Command not found errors

**Fix:**
```bash
# Install sqlite3
# Ubuntu/Debian
sudo apt-get install sqlite3

# macOS (usually pre-installed)
brew install sqlite

# Verify
sqlite3 --version
```

### Column Alignment Issues

**Symptoms:** Misaligned output in column mode

**Workaround:**
```bash
# Use other output modes
./scripts/query-config-db.sh sql "SELECT * FROM workflows" | sqlite3 -json data/config.db
./scripts/query-config-db.sh sql "SELECT * FROM workflows" | sqlite3 -csv data/config.db
```

### Empty Results

**Symptoms:** No data returned

**Possible causes:**
1. Database not populated - Run `populate-config-db.sh`
2. Query filters too restrictive - Check WHERE clauses
3. Data not in expected format - Check source files

**Debug:**
```bash
# Check table contents
./scripts/query-config-db.sh shell
sqlite> SELECT COUNT(*) FROM flake_inputs;
sqlite> .quit

# Verify data populated
./scripts/query-config-db.sh summary
```

---

## References

### Source Code
- **Main script:** `scripts/query-config-db.sh` (202 lines)
- **Database check:** lines 20-25
- **Query function:** lines 27-30
- **Flake commands:** lines 33-41
- **Workflow commands:** lines 43-63
- **Secrets commands:** lines 65-77
- **Reference commands:** lines 79-82
- **Capability commands:** lines 84-92
- **Issues commands:** lines 94-105
- **Summary command:** lines 107-128
- **SQL/shell commands:** lines 130-141
- **Main dispatch:** lines 175-199

### Related Files
- **Database:** `data/config.db`
- **Populate script:** `scripts/populate-config-db.sh`
- **Schema:** `data/schema.sql`

### External Resources
- [SQLite Documentation](https://www.sqlite.org/docs.html)
- [SQLite Command Line](https://www.sqlite.org/cli.html)
- [SQL Tutorial](https://www.sqlitetutorial.net/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 43/60 contracts complete (71.7%)
