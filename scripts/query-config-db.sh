#!/usr/bin/env bash
# query-config-db.sh - Query the configuration database
# Usage: ./scripts/query-config-db.sh <command> [args]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DB_PATH="$REPO_ROOT/data/config.db"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# Check database exists
check_db() {
    if [[ ! -f "$DB_PATH" ]]; then
        echo -e "${RED}Database not found.${NC} Run: ./scripts/populate-config-db.sh"
        exit 1
    fi
}

# Pretty print query results
query() {
    sqlite3 -header -column "$DB_PATH" "$1"
}

# Commands
cmd_inputs() {
    echo -e "${CYAN}=== Flake Inputs ===${NC}"
    query "SELECT name, url, follows, COALESCE(locked_rev, 'UNLOCKED') as rev FROM flake_inputs ORDER BY name;"
}

cmd_inputs_unlocked() {
    echo -e "${YELLOW}=== Unlocked Flake Inputs ===${NC}"
    query "SELECT name, url FROM flake_inputs WHERE locked_rev IS NULL;"
}

cmd_workflows() {
    echo -e "${CYAN}=== Workflows ===${NC}"
    query "SELECT name, file_path, trigger_events FROM workflows ORDER BY name;"
}

cmd_workflow_jobs() {
    local workflow="${1:-}"
    if [[ -n "$workflow" ]]; then
        echo -e "${CYAN}=== Jobs in '$workflow' ===${NC}"
        query "SELECT j.name, j.runs_on FROM workflow_jobs j
               JOIN workflows w ON j.workflow_id = w.id
               WHERE w.name LIKE '%$workflow%' OR w.file_path LIKE '%$workflow%';"
    else
        echo -e "${CYAN}=== All Workflow Jobs ===${NC}"
        query "SELECT w.name as workflow, j.name as job, j.runs_on
               FROM workflow_jobs j JOIN workflows w ON j.workflow_id = w.id
               ORDER BY w.name, j.name;"
    fi
}

cmd_secrets() {
    echo -e "${CYAN}=== Secrets Required by Workflows ===${NC}"
    query "SELECT w.name as workflow, GROUP_CONCAT(s.secret_name, ', ') as secrets
           FROM workflow_secrets s
           JOIN workflows w ON s.workflow_id = w.id
           GROUP BY w.id
           ORDER BY w.name;"
}

cmd_secrets_list() {
    echo -e "${CYAN}=== All Unique Secrets ===${NC}"
    query "SELECT DISTINCT secret_name FROM workflow_secrets ORDER BY secret_name;"
}

cmd_references() {
    echo -e "${CYAN}=== Reference Sources ===${NC}"
    query "SELECT name, category, url FROM reference_sources ORDER BY category, name;"
}

cmd_capabilities() {
    echo -e "${CYAN}=== Capability Registry ===${NC}"
    query "SELECT name, api_count, use_cases FROM capability_categories ORDER BY api_count DESC;"
}

cmd_capabilities_total() {
    echo -e "${CYAN}=== Total APIs ===${NC}"
    query "SELECT SUM(api_count) as total_apis, COUNT(*) as categories FROM capability_categories;"
}

cmd_issues() {
    echo -e "${YELLOW}=== Unresolved Issues ===${NC}"
    query "SELECT severity, source, issue_type, description, file_path
           FROM config_issues WHERE is_resolved = 0
           ORDER BY CASE severity WHEN 'error' THEN 1 WHEN 'warning' THEN 2 ELSE 3 END;"
}

cmd_issues_errors() {
    echo -e "${RED}=== Errors Only ===${NC}"
    query "SELECT source, description, file_path FROM config_issues
           WHERE is_resolved = 0 AND severity = 'error';"
}

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

cmd_sql() {
    # Run arbitrary SQL
    query "$1"
}

cmd_shell() {
    echo -e "${CYAN}Opening SQLite shell...${NC}"
    echo "Tables: flake_inputs, workflows, workflow_jobs, workflow_secrets, reference_sources, capability_categories, config_issues"
    echo "Views: v_flake_inputs_status, v_workflow_summary, v_unresolved_issues"
    echo ""
    sqlite3 "$DB_PATH"
}

# Help
show_help() {
    cat << EOF
Configuration Database Query Tool

Usage: $0 <command> [args]

Commands:
  inputs              List all flake inputs
  inputs-unlocked     List inputs without lock entries
  workflows           List all workflows
  jobs [workflow]     List workflow jobs (optionally filter by workflow)
  secrets             List secrets by workflow
  secrets-list        List all unique secrets
  references          List reference sources
  capabilities        List capability categories
  capabilities-total  Show total API count
  issues              Show unresolved issues
  issues-errors       Show only errors
  summary             Database summary
  sql "QUERY"         Run arbitrary SQL query
  shell               Open SQLite shell

Examples:
  $0 inputs
  $0 jobs "verify-ai"
  $0 sql "SELECT * FROM flake_inputs WHERE name LIKE '%nix%'"

EOF
}

# Main
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

main "$@"
