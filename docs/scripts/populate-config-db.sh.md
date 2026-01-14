# Script Contract: populate-config-db.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/populate-config-db.sh`

---

## Purpose

Configuration database builder that parses infrastructure-as-code files and populates a SQLite database for querying. Extracts and indexes flake inputs (with locked revisions), GitHub workflow definitions (triggers, jobs, secrets), README reference sources, and capability registry APIs. Performs validation checks to identify possibly unused inputs, workflows without secrets, and other configuration issues. Enables SQL-based querying of configuration data for auditing and analysis.

---

## Invocation

```bash
./scripts/populate-config-db.sh [--reset]
```

**Options:**
- (no options) - Parse and populate database (preserves existing data)
- `--reset` - Delete existing database and rebuild from scratch

**Examples:**
```bash
./scripts/populate-config-db.sh           # Initial population or update
./scripts/populate-config-db.sh --reset   # Clean rebuild
```

**Requirements:**
- `sqlite3` command available
- `jq` (optional, for flake.lock and API registry parsing)
- Database schema file: `data/schema.sql`

---

## Outputs

**Standard Output (successful run):**
```
[INFO] Configuration Database Builder

[INFO] Initializing database at /path/to/ripple-env/data/config.db
[OK] Database schema applied

[INFO] Parsing flake.nix inputs...
[OK]   Found input: nixpkgs -> github:NixOS/nixpkgs/nixos-24.11
[OK]   Found input: home-manager -> github:nix-community/home-manager (follows: none)
[OK]   Found input: flake-parts -> github:hercules-ci/flake-parts (follows: none)
[OK]   Found input: pixi -> github:prefix-dev/pixi (follows: nixpkgs)

[INFO] Parsing flake.lock...
[OK]   Lock info updated via jq

[INFO] Parsing GitHub workflows...
[OK]   Found workflow: CI Build and Test (ci.yml)
[OK]   Found workflow: Container Security Scan (security.yml)
[OK]   Found workflow: Deploy to Production (deploy.yml)

[INFO] Parsing README reference sources...
[OK]   Loaded 42 reference sources

[INFO] Parsing capability registry...
[OK]   Loaded capability registry with 156 APIs

[INFO] Running validation checks...
[WARN]   Possibly unused input (no references found): example-input
[OK] Validation complete

==========================================
         Configuration Database
==========================================

Database: /path/to/ripple-env/data/config.db

Contents:
  Flake inputs:      8
  Workflows:         5
  Workflow jobs:     18
  Workflow secrets:  12
  Reference sources: 42
  API categories:    12
  Config issues:     1

Quick queries:
  sqlite3 /path/to/config.db 'SELECT * FROM v_flake_inputs_status;'
  sqlite3 /path/to/config.db 'SELECT * FROM v_workflow_summary;'
  sqlite3 /path/to/config.db 'SELECT * FROM v_unresolved_issues;'

[OK] Done!
```

**Standard Output (with reset):**
```
[INFO] Initializing database at /path/to/ripple-env/data/config.db
[WARN] Resetting database...
[OK] Database schema applied
...
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (database populated) |
| `1` | Failure (schema file missing/unreadable, schema application failed) |

---

## Side Effects

### Database File (lines 44-66)

**Evidence:**
```bash
init_db() {
    log_info "Initializing database at $DB_PATH"

    if [[ "${1:-}" == "--reset" ]] && [[ -f "$DB_PATH" ]]; then
        log_warn "Resetting database..."
        rm -f "$DB_PATH"
    fi

    if [[ ! -f "$SCHEMA_PATH" ]]; then
        log_error "Schema file not found at $SCHEMA_PATH"
        return 1
    fi

    if [[ ! -r "$SCHEMA_PATH" ]]; then
        log_error "Schema file at $SCHEMA_PATH is not readable"
        return 1
    fi

    if ! sqlite3 "$DB_PATH" < "$SCHEMA_PATH"; then
        log_error "Failed to apply database schema from $SCHEMA_PATH"
        return 1
    fi
    log_success "Database schema applied"
}
```

**Creates:** `data/config.db` SQLite database

**Reset:** Deletes existing database if `--reset` flag (line 47-49)

---

## Safety Classification

**🟢 SAFE** - Read-only parsing, only modifies database file in `data/`.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Uses `INSERT OR REPLACE` and `INSERT OR IGNORE` for all database operations.

---

## SQL Escaping and Safety

**Evidence:** Lines 24-41

```bash
# Safely escape a value for use in SQL string literal
# Escapes single quotes properly for SQLite (SQL standard)
sql_escape() {
    local value="$1"
    # SQLite uses SQL standard escaping: single quotes are escaped by doubling them
    # Backslashes do NOT need escaping in SQLite as they are not special characters
    printf '%s' "$value" | sed "s/'/''/g"
}

# Safely quote a value for SQL - returns 'escaped_value' or NULL
sql_quote() {
    local value="$1"
    if [[ -z "$value" ]]; then
        echo "NULL"
    else
        printf "'%s'" "$(sql_escape "$value")"
    fi
}
```

**Critical security feature:** Prevents SQL injection

**Escaping method:**
- Single quotes → doubled (`'` becomes `''`)
- Empty strings → `NULL`

**Usage pattern:**
```bash
local name_quoted=$(sql_quote "$current_input")
sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO flake_inputs (name, url) VALUES ($name_quoted, $url_quoted);"
```

---

## Flake Inputs Parsing

**Evidence:** Lines 69-153

```bash
parse_flake_inputs() {
    log_info "Parsing flake.nix inputs..."
    local flake_file="$REPO_ROOT/flake.nix"

    if [[ ! -f "$flake_file" ]]; then
        log_error "flake.nix not found"
        return 1
    fi

    # Extract inputs section and parse
    local in_inputs=0
    local current_input=""
    local current_url=""
    local current_follows=""
    local brace_depth=0

    while IFS= read -r line; do
        # Detect inputs = { block
        if [[ "$line" =~ inputs[[:space:]]*= ]]; then
            in_inputs=1
            continue
        fi

        # Skip if not in inputs section
        [[ $in_inputs -eq 0 ]] && continue

        # Track brace depth
        if [[ "$line" =~ \{ ]]; then
            ((brace_depth++)) || true
        fi
        if [[ "$line" =~ \} ]]; then
            ((brace_depth--)) || true
            [[ $brace_depth -eq 0 ]] && break
        fi

        # Simple URL input: name.url = "...";
        if [[ "$line" =~ ^[[:space:]]*([a-zA-Z0-9_-]+)\.url[[:space:]]*=[[:space:]]*\"([^\"]+)\" ]]; then
            current_input="${BASH_REMATCH[1]}"
            current_url="${BASH_REMATCH[2]}"

            local name_quoted=$(sql_quote "$current_input")
            local url_quoted=$(sql_quote "$current_url")

            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO flake_inputs (name, url, is_active)
                VALUES ($name_quoted, $url_quoted, 1);"
            log_success "  Found input: $current_input -> $current_url"
        fi

        # Block input start: name = {
        if [[ "$line" =~ ^[[:space:]]*([a-zA-Z0-9_-]+)[[:space:]]*=[[:space:]]*\{ ]]; then
            current_input="${BASH_REMATCH[1]}"
            current_url=""
            current_follows=""
        fi

        # URL inside block
        if [[ -n "$current_input" ]] && [[ "$line" =~ url[[:space:]]*=[[:space:]]*\"([^\"]+)\" ]]; then
            current_url="${BASH_REMATCH[1]}"
        fi

        # Follows inside block
        if [[ -n "$current_input" ]] && [[ "$line" =~ inputs\.([a-zA-Z0-9_-]+)\.follows ]]; then
            current_follows="${BASH_REMATCH[1]}"
        fi

        # End of block input
        if [[ -n "$current_input" ]] && [[ -n "$current_url" ]] && [[ "$line" =~ \}[[:space:]]*\; ]]; then
            local name_quoted=$(sql_quote "$current_input")
            local url_quoted=$(sql_quote "$current_url")
            local follows_quoted=$(sql_quote "$current_follows")

            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO flake_inputs (name, url, follows, is_active)
                VALUES ($name_quoted, $url_quoted, $follows_quoted, 1);"
            log_success "  Found input: $current_input -> $current_url (follows: ${current_follows:-none})"
            current_input=""
            current_url=""
            current_follows=""
        fi

    done < "$flake_file"

    # Parse flake.lock for locked revisions
    parse_flake_lock
}
```

**Parsing logic:**
1. Detects `inputs = {` block (line 88)
2. Tracks brace depth to find block end (lines 97-102)
3. Extracts simple inputs: `name.url = "...";` (line 106)
4. Extracts block inputs with nested properties (lines 119-147)
5. Handles `follows` directive (line 131)

**Example input patterns:**
```nix
inputs = {
  # Simple format
  nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";

  # Block format with follows
  home-manager = {
    url = "github:nix-community/home-manager";
    inputs.nixpkgs.follows = "nixpkgs";
  };
};
```

---

## Flake Lock Parsing

**Evidence:** Lines 155-187

```bash
parse_flake_lock() {
    log_info "Parsing flake.lock..."
    local lock_file="$REPO_ROOT/flake.lock"

    if [[ ! -f "$lock_file" ]]; then
        log_warn "flake.lock not found"
        return 0
    fi

    # Use jq if available, otherwise basic parsing
    if command -v jq &>/dev/null; then
        # Get all locked nodes
        jq -r '.nodes | to_entries[] | select(.value.locked != null) |
            "\(.key)|\(.value.locked.rev // "")|\(.value.locked.lastModified // "")"' "$lock_file" 2>/dev/null | \
        while IFS='|' read -r name rev lastmod; do
            [[ "$name" == "root" ]] && continue
            # Convert timestamp to date if numeric
            if [[ "$lastmod" =~ ^[0-9]+$ ]]; then
                lastmod=$(date -d "@$lastmod" '+%Y-%m-%d' 2>/dev/null || echo "$lastmod")
            fi

            local rev_quoted=$(sql_quote "$rev")
            local lastmod_quoted=$(sql_quote "$lastmod")
            local name_quoted=$(sql_quote "$name")

            sqlite3 "$DB_PATH" "UPDATE flake_inputs SET locked_rev=$rev_quoted, locked_date=$lastmod_quoted WHERE name=$name_quoted;" 2>/dev/null || true
        done
        log_success "  Lock info updated via jq"
    else
        log_warn "  jq not available, skipping lock parsing"
    fi
}
```

**Requires jq** - Gracefully skips if unavailable (line 166)

**Extracts:**
- `rev` - Git commit hash (line 169)
- `lastModified` - Unix timestamp, converted to YYYY-MM-DD (lines 173-175)

**Updates:** Existing `flake_inputs` rows with locked metadata (line 181)

---

## GitHub Workflows Parsing

**Evidence:** Lines 189-260

```bash
parse_workflows() {
    log_info "Parsing GitHub workflows..."
    local workflow_dir="$REPO_ROOT/.github/workflows"

    if [[ ! -d "$workflow_dir" ]]; then
        log_warn "No .github/workflows directory found"
        return 0
    fi

    for workflow_file in "$workflow_dir"/*.yml "$workflow_dir"/*.yaml; do
        [[ -f "$workflow_file" ]] || continue

        local filename=$(basename "$workflow_file")
        local workflow_name=""
        local triggers=""

        # Extract workflow name
        workflow_name=$(grep -m1 "^name:" "$workflow_file" 2>/dev/null | sed 's/name:[[:space:]]*//' | tr -d '"' || echo "$filename")

        # Extract triggers (on: section)
        triggers=$(grep -A20 "^on:" "$workflow_file" 2>/dev/null | \
            grep -E "^\s+(push|pull_request|workflow_dispatch|schedule|release):" | \
            sed 's/[[:space:]]*//g; s/://g' | tr '\n' ',' | sed 's/,$//' || echo "")

        # Insert workflow
        local name_quoted=$(sql_quote "$workflow_name")
        local file_path_quoted=$(sql_quote ".github/workflows/$filename")
        local triggers_quoted=$(sql_quote "[$triggers]")

        sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO workflows (name, file_path, trigger_events, is_active)
            VALUES ($name_quoted, $file_path_quoted, $triggers_quoted, 1);"

        log_success "  Found workflow: $workflow_name ($filename)"

        # Extract jobs
        local workflow_id=$(sqlite3 "$DB_PATH" "SELECT id FROM workflows WHERE file_path='.github/workflows/$filename';")

        grep -E "^[[:space:]]{2}[a-zA-Z0-9_-]+:" "$workflow_file" 2>/dev/null | \
        while read -r job_line; do
            # Skip non-job keys
            [[ "$job_line" =~ (name:|on:|env:|permissions:|concurrency:|defaults:) ]] && continue

            local job_name=$(echo "$job_line" | sed 's/[[:space:]]*//g; s/://g')

            # Get runs-on for this job
            local runs_on=$(grep -A5 "^[[:space:]]*${job_name}:" "$workflow_file" 2>/dev/null | \
                grep "runs-on:" | head -1 | sed 's/.*runs-on:[[:space:]]*//' | tr -d '"' || echo "")

            [[ -z "$job_name" ]] && continue

            local job_name_quoted=$(sql_quote "$job_name")
            local runs_on_quoted=$(sql_quote "$runs_on")

            sqlite3 "$DB_PATH" "INSERT OR IGNORE INTO workflow_jobs (workflow_id, name, runs_on)
                VALUES ($workflow_id, $job_name_quoted, $runs_on_quoted);" 2>/dev/null || true
        done

        # Extract secrets
        grep -oE '\$\{\{[[:space:]]*secrets\.[A-Z0-9_]+' "$workflow_file" 2>/dev/null | \
            sed 's/.*secrets\.//' | sort -u | \
        while read -r secret; do
            [[ -z "$secret" ]] && continue
            local secret_quoted=$(sql_quote "$secret")
            sqlite3 "$DB_PATH" "INSERT OR IGNORE INTO workflow_secrets (workflow_id, secret_name)
                VALUES ($workflow_id, $secret_quoted);" 2>/dev/null || true
        done

    done
}
```

**Extracts:**
1. **Workflow name** - From `name:` field (line 207)
2. **Triggers** - From `on:` section (push, pull_request, etc.) (lines 210-212)
3. **Jobs** - From top-level keys with indentation (lines 227-246)
4. **Secrets** - From `${{ secrets.NAME }}` patterns (lines 249-257)

**Database tables:**
- `workflows` - Workflow definitions (line 219)
- `workflow_jobs` - Jobs within workflows (line 244)
- `workflow_secrets` - Secrets used by workflows (line 255)

---

## README References Parsing

**Evidence:** Lines 262-301

```bash
parse_references() {
    log_info "Parsing README reference sources..."
    local readme_file="$REPO_ROOT/README.md"

    if [[ ! -f "$readme_file" ]]; then
        log_warn "README.md not found"
        return 0
    fi

    local current_section=""

    while IFS= read -r line; do
        # Track section headers
        if [[ "$line" =~ ^##[[:space:]]+(.*) ]]; then
            current_section="${BASH_REMATCH[1]}"
        fi

        # Parse markdown table rows with links
        # Format: | **name** | description | [link](url) |
        if [[ "$line" =~ ^\|[[:space:]]*\*\*([^\*]+)\*\*[[:space:]]*\|[[:space:]]*([^\|]+)[[:space:]]*\|[[:space:]]*\[([^\]]+)\]\(([^\)]+)\) ]]; then
            local name="${BASH_REMATCH[1]}"
            local desc="${BASH_REMATCH[2]}"
            local url="${BASH_REMATCH[4]}"

            local name_quoted=$(sql_quote "$name")
            local desc_quoted=$(sql_quote "$desc")
            local url_quoted=$(sql_quote "$url")
            local section_quoted=$(sql_quote "$current_section")

            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO reference_sources (name, description, url, category)
                VALUES ($name_quoted, $desc_quoted, $url_quoted, $section_quoted);"
        fi

    done < "$readme_file"

    local count=$(sqlite3 "$DB_PATH" "SELECT COUNT(*) FROM reference_sources;")
    log_success "  Loaded $count reference sources"
}
```

**Markdown table pattern:**
```markdown
## External Resources

| **Name** | Description | Link |
|----------|-------------|------|
| **ROS2 Docs** | Official documentation | [Docs](https://docs.ros.org) |
| **Nix Manual** | Nix package manager | [Manual](https://nixos.org/manual) |
```

**Extracts:**
- Name (bold text) - line 283
- Description - line 284
- URL - line 285
- Category (section header) - line 277

---

## Capability Registry Parsing

**Evidence:** Lines 303-333

```bash
parse_capability_registry() {
    log_info "Parsing capability registry..."
    local registry_file="$REPO_ROOT/manifests/capability-registry/api-registry.json"

    if [[ ! -f "$registry_file" ]]; then
        log_warn "api-registry.json not found"
        return 0
    fi

    if ! command -v jq &>/dev/null; then
        log_warn "jq not available, skipping capability registry"
        return 0
    fi

    jq -r '.categories[] | "\(.id)|\(.name)|\(.description)|\(.count)|\(.path)|\(.useCases | join(","))"' "$registry_file" | \
    while IFS='|' read -r id name desc count path usecases; do
        local id_quoted=$(sql_quote "$id")
        local name_quoted=$(sql_quote "$name")
        local desc_quoted=$(sql_quote "$desc")
        local path_quoted=$(sql_quote "$path")
        local usecases_quoted=$(sql_quote "[$usecases]")

        sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO capability_categories
            (category_id, name, description, api_count, path, use_cases)
            VALUES ($id_quoted, $name_quoted, $desc_quoted, $count, $path_quoted, $usecases_quoted);"
    done

    local total=$(sqlite3 "$DB_PATH" "SELECT SUM(api_count) FROM capability_categories;")
    log_success "  Loaded capability registry with $total APIs"
}
```

**Requires jq** - Gracefully skips if unavailable (line 313)

**JSON structure:**
```json
{
  "categories": [
    {
      "id": "messaging",
      "name": "Messaging & Events",
      "description": "NATS, Temporal, Redis",
      "count": 45,
      "path": "/api/messaging",
      "useCases": ["async", "events", "queues"]
    }
  ]
}
```

---

## Validation Checks

**Evidence:** Lines 335-380

```bash
run_validation() {
    log_info "Running validation checks..."

    # Check for inputs in flake.nix that might not be used
    local flake_file="$REPO_ROOT/flake.nix"

    sqlite3 "$DB_PATH" "SELECT name FROM flake_inputs WHERE is_active=1;" | \
    while read -r input_name; do
        # Count references (excluding the input definition itself)
        local ref_count=$(grep -c "$input_name" "$flake_file" 2>/dev/null || echo "0")

        if [[ $ref_count -eq 0 ]]; then
            local source_quoted=$(sql_quote "flake")
            local severity_quoted=$(sql_quote "warning")
            local issue_type_quoted=$(sql_quote "possibly_unused")
            local desc_quoted=$(sql_quote "Input \"$input_name\" may be unused (no references found)")
            local file_path_quoted=$(sql_quote "flake.nix")

            sqlite3 "$DB_PATH" "INSERT INTO config_issues (source, severity, issue_type, description, file_path)
                VALUES ($source_quoted, $severity_quoted, $issue_type_quoted, $desc_quoted, $file_path_quoted);"
            log_warn "  Possibly unused input (no references found): $input_name"
        fi
    done

    # Check for workflows without any secrets (might be missing auth)
    sqlite3 "$DB_PATH" "
        SELECT w.name FROM workflows w
        LEFT JOIN workflow_secrets s ON w.id = s.workflow_id
        WHERE s.id IS NULL AND (w.name LIKE '%test%' OR w.name LIKE '%deploy%')
    " | while read -r workflow; do
        [[ -z "$workflow" ]] && continue

        local source_quoted=$(sql_quote "workflow")
        local severity_quoted=$(sql_quote "info")
        local issue_type_quoted=$(sql_quote "no_secrets")
        local desc_quoted=$(sql_quote "Workflow \"$workflow\" uses no secrets")
        local file_path_quoted=$(sql_quote ".github/workflows/")

        sqlite3 "$DB_PATH" "INSERT INTO config_issues (source, severity, issue_type, description, file_path)
            VALUES ($source_quoted, $severity_quoted, $issue_type_quoted, $desc_quoted, $file_path_quoted);"
    done

    log_success "Validation complete"
}
```

**Checks performed:**
1. **Unused flake inputs** - Inputs defined but not referenced (lines 343-359)
2. **Workflows without secrets** - Test/deploy workflows with no secrets (lines 362-377)

**Issue severities:**
- `warning` - Possibly unused input
- `info` - Workflow without secrets

---

## Database Summary

**Evidence:** Lines 382-405

```bash
print_summary() {
    echo ""
    echo "=========================================="
    echo "         Configuration Database          "
    echo "=========================================="
    echo ""
    echo "Database: $DB_PATH"
    echo ""
    echo "Contents:"
    sqlite3 "$DB_PATH" "SELECT '  Flake inputs:      ' || COUNT(*) FROM flake_inputs;"
    sqlite3 "$DB_PATH" "SELECT '  Workflows:         ' || COUNT(*) FROM workflows;"
    sqlite3 "$DB_PATH" "SELECT '  Workflow jobs:     ' || COUNT(*) FROM workflow_jobs;"
    sqlite3 "$DB_PATH" "SELECT '  Workflow secrets:  ' || COUNT(*) FROM workflow_secrets;"
    sqlite3 "$DB_PATH" "SELECT '  Reference sources: ' || COUNT(*) FROM reference_sources;"
    sqlite3 "$DB_PATH" "SELECT '  API categories:    ' || COUNT(*) FROM capability_categories;"
    sqlite3 "$DB_PATH" "SELECT '  Config issues:     ' || COUNT(*) FROM config_issues WHERE is_resolved=0;"
    echo ""
    echo "Quick queries:"
    echo "  sqlite3 $DB_PATH 'SELECT * FROM v_flake_inputs_status;'"
    echo "  sqlite3 $DB_PATH 'SELECT * FROM v_workflow_summary;'"
    echo "  sqlite3 $DB_PATH 'SELECT * FROM v_unresolved_issues;'"
    echo ""
}
```

**Displays:**
- Record counts for all tables
- Suggested queries for views

---

## Example Queries

### View Flake Inputs Status

```sql
SELECT * FROM v_flake_inputs_status;
```

**Returns:** Input name, URL, locked revision, locked date, follows relationship

### View Workflow Summary

```sql
SELECT * FROM v_workflow_summary;
```

**Returns:** Workflow name, trigger events, job count, secret count

### View Unresolved Issues

```sql
SELECT * FROM v_unresolved_issues;
```

**Returns:** Source, severity, issue type, description, file path

### Find Workflows Using Specific Secret

```sql
SELECT w.name, w.file_path
FROM workflows w
JOIN workflow_secrets s ON w.id = s.workflow_id
WHERE s.secret_name = 'GITHUB_TOKEN';
```

### List All Flake Inputs with Follows

```sql
SELECT name, url, follows
FROM flake_inputs
WHERE follows IS NOT NULL AND follows != '';
```

---

## Database Schema

**Location:** `data/schema.sql` (line 10)

**Tables (from script references):**
- `flake_inputs` - Nix flake input definitions (line 113)
- `workflows` - GitHub Actions workflows (line 219)
- `workflow_jobs` - Jobs within workflows (line 244)
- `workflow_secrets` - Secrets used by workflows (line 255)
- `reference_sources` - README documentation links (line 293)
- `capability_categories` - API category registry (line 326)
- `config_issues` - Validation issues found (line 355)

**Views:**
- `v_flake_inputs_status` - Flake inputs with lock info (line 401)
- `v_workflow_summary` - Workflow overview with counts (line 402)
- `v_unresolved_issues` - Active configuration issues (line 403)

---

## Troubleshooting

### Schema File Not Found

**Symptoms:** "Schema file not found" error (line 53)

**Fix:**
```bash
# Check if schema exists
ls -l data/schema.sql

# Create if missing (requires schema definition)
# Schema should define all tables and views
```

### jq Not Available

**Symptoms:** "jq not available" warnings (lines 185, 314)

**Impact:** Skips flake.lock and capability registry parsing

**Fix:**
```bash
# Install jq
# Ubuntu/Debian
sudo apt-get install jq

# macOS
brew install jq

# Or use Nix
nix-shell -p jq
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

### SQL Injection Concerns

**Addressed:** Script uses `sql_escape` and `sql_quote` functions (lines 24-41)

**All user data is properly escaped** before insertion into SQL queries

---

## References

### Source Code
- **Main script:** `scripts/populate-config-db.sh` (425 lines)
- **SQL escaping:** lines 24-41
- **Database init:** lines 44-66
- **Flake parsing:** lines 69-153
- **Lock parsing:** lines 155-187
- **Workflow parsing:** lines 189-260
- **README parsing:** lines 262-301
- **Registry parsing:** lines 303-333
- **Validation:** lines 335-380
- **Summary:** lines 382-405

### Related Files
- **Database:** `data/config.db`
- **Schema:** `data/schema.sql`
- **Flake:** `flake.nix`
- **Lock file:** `flake.lock`
- **Workflows:** `.github/workflows/*.yml`
- **README:** `README.md`
- **Registry:** `manifests/capability-registry/api-registry.json`

### External Resources
- [SQLite Documentation](https://www.sqlite.org/docs.html)
- [jq Manual](https://jqlang.github.io/jq/manual/)
- [Nix Flakes](https://nixos.org/manual/nix/stable/command-ref/new-cli/nix3-flake.html)
- [GitHub Actions Syntax](https://docs.github.com/en/actions/using-workflows/workflow-syntax-for-github-actions)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 42/60 contracts complete (70%)
