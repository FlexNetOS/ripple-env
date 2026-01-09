#!/usr/bin/env bash
# populate-config-db.sh - Parse configs and populate SQLite database
# Usage: ./scripts/populate-config-db.sh [--reset]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DB_PATH="$REPO_ROOT/data/config.db"
SCHEMA_PATH="$REPO_ROOT/data/schema.sql"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[OK]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Initialize database
init_db() {
    log_info "Initializing database at $DB_PATH"

    if [[ "${1:-}" == "--reset" ]] && [[ -f "$DB_PATH" ]]; then
        log_warn "Resetting database..."
        rm -f "$DB_PATH"
    fi

    sqlite3 "$DB_PATH" < "$SCHEMA_PATH"
    log_success "Database schema applied"
}

# Parse flake.nix inputs
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

            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO flake_inputs (name, url, is_active)
                VALUES ('$current_input', '$current_url', 1);"
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
            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO flake_inputs (name, url, follows, is_active)
                VALUES ('$current_input', '$current_url', $([ -n "$current_follows" ] && echo "'$current_follows'" || echo "NULL"), 1);"
            log_success "  Found input: $current_input -> $current_url (follows: ${current_follows:-none})"
            current_input=""
            current_url=""
            current_follows=""
        fi

    done < "$flake_file"

    # Parse flake.lock for locked revisions
    parse_flake_lock
}

# Parse flake.lock for revision info
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
            sqlite3 "$DB_PATH" "UPDATE flake_inputs SET locked_rev='$rev', locked_date='$lastmod' WHERE name='$name';" 2>/dev/null || true
        done
        log_success "  Lock info updated via jq"
    else
        log_warn "  jq not available, skipping lock parsing"
    fi
}

# Parse GitHub workflow files
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
        sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO workflows (name, file_path, trigger_events, is_active)
            VALUES ('$workflow_name', '.github/workflows/$filename', '[$triggers]', 1);"

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

            sqlite3 "$DB_PATH" "INSERT OR IGNORE INTO workflow_jobs (workflow_id, name, runs_on)
                VALUES ($workflow_id, '$job_name', '$runs_on');" 2>/dev/null || true
        done

        # Extract secrets
        grep -oE '\$\{\{[[:space:]]*secrets\.[A-Z0-9_]+' "$workflow_file" 2>/dev/null | \
            sed 's/.*secrets\.//' | sort -u | \
        while read -r secret; do
            [[ -z "$secret" ]] && continue
            sqlite3 "$DB_PATH" "INSERT OR IGNORE INTO workflow_secrets (workflow_id, secret_name)
                VALUES ($workflow_id, '$secret');" 2>/dev/null || true
        done

    done
}

# Parse README reference sources
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

            # Escape single quotes
            desc="${desc//\'/\'\'}"

            sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO reference_sources (name, description, url, category)
                VALUES ('$name', '$desc', '$url', '$current_section');"
        fi

    done < "$readme_file"

    local count=$(sqlite3 "$DB_PATH" "SELECT COUNT(*) FROM reference_sources;")
    log_success "  Loaded $count reference sources"
}

# Parse capability registry JSON
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
        desc="${desc//\'/\'\'}"
        sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO capability_categories
            (category_id, name, description, api_count, path, use_cases)
            VALUES ('$id', '$name', '$desc', $count, '$path', '[$usecases]');"
    done

    local total=$(sqlite3 "$DB_PATH" "SELECT SUM(api_count) FROM capability_categories;")
    log_success "  Loaded capability registry with $total APIs"
}

# Run validation checks
run_validation() {
    log_info "Running validation checks..."

    # Check for inputs in flake.nix that might not be used
    # This is a simple heuristic check
    local flake_file="$REPO_ROOT/flake.nix"

    sqlite3 "$DB_PATH" "SELECT name FROM flake_inputs WHERE is_active=1;" | \
    while read -r input_name; do
        # Count references (excluding the input definition itself)
        local ref_count=$(grep -c "$input_name" "$flake_file" 2>/dev/null || echo "0")

        if [[ $ref_count -lt 3 ]]; then
            sqlite3 "$DB_PATH" "INSERT INTO config_issues (source, severity, issue_type, description, file_path)
                VALUES ('flake', 'warning', 'possibly_unused', 'Input \"$input_name\" may be unused (only $ref_count references)', 'flake.nix');"
            log_warn "  Possibly unused input: $input_name"
        fi
    done

    # Check for workflows without any secrets (might be missing auth)
    sqlite3 "$DB_PATH" "
        SELECT w.name FROM workflows w
        LEFT JOIN workflow_secrets s ON w.id = s.workflow_id
        WHERE s.id IS NULL AND w.name LIKE '%test%' OR w.name LIKE '%deploy%'
    " | while read -r workflow; do
        [[ -z "$workflow" ]] && continue
        sqlite3 "$DB_PATH" "INSERT INTO config_issues (source, severity, issue_type, description, file_path)
            VALUES ('workflow', 'info', 'no_secrets', 'Workflow \"$workflow\" uses no secrets', '.github/workflows/');"
    done

    log_success "Validation complete"
}

# Print summary
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

# Main
main() {
    echo ""
    log_info "Configuration Database Builder"
    echo ""

    init_db "${1:-}"
    parse_flake_inputs
    parse_workflows
    parse_references
    parse_capability_registry
    run_validation
    print_summary

    log_success "Done!"
}

main "$@"
