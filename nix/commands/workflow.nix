# Workflow management command wrappers
# Includes: gh-issues, db-query, temporal-ctl, n8n-ctl, workflow-status, codebase-db
# Extracted from flake.nix for modular organization
{ pkgs, ... }:

[
            # Shared helper function for opening URLs
            (pkgs.writeShellScriptBin "open-url-helper" ''
              # Helper to open URLs in the default browser
              URL="$1"
              if [ -z "$URL" ]; then
                echo "Error: No URL provided" >&2
                exit 1
              fi
              
              echo "Opening: $URL"
              if command -v xdg-open >/dev/null 2>&1; then
                xdg-open "$URL"
              elif command -v open >/dev/null 2>&1; then
                open "$URL"
              else
                echo "Open in browser: $URL"
              fi
            '')
            # GitHub issues query tool
            (pkgs.writeShellScriptBin "gh-issues" ''
              # Query and manage GitHub issues
              # Usage: gh-issues [list|search|create|view|close]

              if ! command -v gh >/dev/null 2>&1; then
                echo "Error: gh CLI not found" >&2
                echo "Install with: nix develop" >&2
                exit 1
              fi

              case "''${1:-list}" in
                list)
                  # List issues with optional state filter
                  STATE="''${2:-open}"
                  LIMIT="''${3:-20}"
                  
                  # Validate STATE parameter
                  if [[ ! "$STATE" =~ ^(open|closed|all)$ ]]; then
                    echo "Error: STATE must be one of: open, closed, all" >&2
                    exit 1
                  fi
                  
                  # Validate LIMIT parameter
                  if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
                    echo "Error: LIMIT must be a positive integer" >&2
                    exit 1
                  fi
                  
                  echo "Issues ($STATE):"
                  gh issue list --state "$STATE" --limit "$LIMIT"
                  ;;
                search)
                  # Search issues by keyword
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues search <query> [state]" >&2
                    exit 1
                  fi
                  STATE="''${3:-all}"
                  echo "Searching issues for: $2"
                  gh issue list --search "$2" --state "$STATE"
                  ;;
                labels)
                  # List issues by label
                  if [ -z "$2" ]; then
                    echo "Available labels:"
                    gh label list
                  else
                    echo "Issues with label '$2':"
                    gh issue list --label "$2"
                  fi
                  ;;
                assigned)
                  # List issues assigned to user
                  USER="''${2:-@me}"
                  echo "Issues assigned to $USER:"
                  gh issue list --assignee "$USER"
                  ;;
                create)
                  # Create a new issue interactively
                  shift
                  gh issue create "$@"
                  ;;
                view)
                  # View issue details
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues view <issue-number>" >&2
                    exit 1
                  fi
                  gh issue view "$2"
                  ;;
                close)
                  # Close an issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues close <issue-number> [reason]" >&2
                    exit 1
                  fi
                  REASON="''${3:-completed}"
                  gh issue close "$2" --reason "$REASON"
                  ;;
                reopen)
                  # Reopen an issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues reopen <issue-number>" >&2
                    exit 1
                  fi
                  gh issue reopen "$2"
                  ;;
                comment)
                  # Add comment to issue
                  if [ -z "$2" ]; then
                    echo "Usage: gh-issues comment <issue-number> [message]" >&2
                    exit 1
                  fi
                  shift
                  ISSUE="$1"
                  shift
                  if [ -n "$1" ]; then
                    gh issue comment "$ISSUE" --body "$*"
                  else
                    gh issue comment "$ISSUE"
                  fi
                  ;;
                stats)
                  # Show issue statistics
                  echo "Issue Statistics"
                  echo "================"
                  echo ""
                  echo "Open issues:   $(gh issue list --state open --json number --jq 'length')"
                  # Note: Using limit of 1000 for closed issues - counts may be inaccurate if more exist
                  echo "Closed issues: $(gh issue list --state closed --limit 1000 --json number --jq 'length') (max 1000)"
                  echo ""
                  echo "By label: (this may take a while for repositories with many labels)"
                  # Note: Queries GitHub for each label sequentially - performance scales with label count
                  gh label list --json name --jq '.[].name' | while read -r label; do
                    count=$(gh issue list --label "$label" --json number --jq 'length' 2>/dev/null || echo "0")
                    [ "$count" != "0" ] && echo "  $label: $count"
                  done
                  ;;
                *)
                  echo "Usage: gh-issues <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  list [state] [limit]     - List issues (default: open, 20)"
                  echo "  search <query> [state]   - Search issues by keyword"
                  echo "  labels [label]           - List labels or issues with label"
                  echo "  assigned [user]          - Issues assigned to user (@me default)"
                  echo "  create                   - Create new issue interactively"
                  echo "  view <num>               - View issue details"
                  echo "  close <num> [reason]     - Close issue (completed|not_planned)"
                  echo "  reopen <num>             - Reopen closed issue"
                  echo "  comment <num> [msg]      - Add comment to issue"
                  echo "  stats                    - Show issue statistics"
                  ;;
              esac
            '')
            # Database query tool
            (pkgs.writeShellScriptBin "db-query" ''
              # Query databases (PostgreSQL/SQLite)
              # Usage: db-query [pg|sqlite] <database> [query]

              DB_TYPE="''${1:-help}"

              case "$DB_TYPE" in
                pg|postgres)
                  # PostgreSQL query
                  DB_NAME="''${2:-postgres}"
                  DB_HOST="''${DB_HOST:-localhost}"
                  DB_PORT="''${DB_PORT:-5432}"
                  DB_USER="''${DB_USER:-postgres}"

                  if [ -z "$3" ]; then
                    # Interactive mode
                    echo "Connecting to PostgreSQL: $DB_NAME@$DB_HOST:$DB_PORT"
                    if command -v pgcli >/dev/null 2>&1; then
                      pgcli -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME"
                    else
                      psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME"
                    fi
                  else
                    # Execute query - WARNING: For complex queries with special characters,
                    # use interactive mode to avoid shell interpretation issues
                    shift 2
                    # Use printf to properly quote the query
                    QUERY="$*"
                    psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" -c "$QUERY"
                  fi
                  ;;
                sqlite)
                  # SQLite query
                  DB_FILE="''${2:-database.db}"

                  if [ ! -f "$DB_FILE" ]; then
                    echo "Error: SQLite database not found: $DB_FILE" >&2
                    exit 1
                  fi

                  if [ -z "$3" ]; then
                    # Interactive mode
                    echo "Opening SQLite database: $DB_FILE"
                    sqlite3 "$DB_FILE"
                  else
                    # Execute query - WARNING: For complex queries with special characters,
                    # use interactive mode to avoid shell interpretation issues
                    shift 2
                    # Use variable to properly quote the query
                    QUERY="$*"
                    sqlite3 "$DB_FILE" "$QUERY"
                  fi
                  ;;
                temporal)
                  # Query Temporal database
                  DB_NAME="temporal"
                  DB_HOST="''${TEMPORAL_DB_HOST:-localhost}"
                  DB_PORT="''${TEMPORAL_DB_PORT:-5432}"
                  DB_USER="''${TEMPORAL_DB_USER:-temporal}"

                  echo "Connecting to Temporal database..."
                  PGPASSWORD="''${TEMPORAL_DB_PASSWORD:-temporal}" psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" "''${@:2}"
                  ;;
                n8n)
                  # Query n8n database
                  DB_NAME="n8n_db"
                  DB_HOST="''${N8N_DB_HOST:-localhost}"
                  DB_PORT="''${N8N_DB_PORT:-5432}"
                  DB_USER="''${N8N_DB_USER:-n8n}"
                  
                  if [ -z "$N8N_DB_PASSWORD" ]; then
                    echo "Error: N8N_DB_PASSWORD environment variable must be set" >&2
                    exit 1
                  fi

                  echo "Connecting to n8n database..."
                  PGPASSWORD="$N8N_DB_PASSWORD" psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" "''${@:2}"
                  ;;
                tables)
                  # List tables in a database
                  shift
                  if [ "$1" = "pg" ]; then
                    DB_NAME="''${2:-postgres}"
                    DB_HOST="''${DB_HOST:-localhost}"
                    DB_PORT="''${DB_PORT:-5432}"
                    DB_USER="''${DB_USER:-postgres}"
                    psql -h "$DB_HOST" -p "$DB_PORT" -U "$DB_USER" -d "$DB_NAME" -c "\dt"
                  elif [ "$1" = "sqlite" ]; then
                    DB_FILE="''${2:-database.db}"
                    if [ ! -f "$DB_FILE" ]; then
                      echo "Error: SQLite database not found: $DB_FILE" >&2
                      exit 1
                    fi
                    sqlite3 "$DB_FILE" ".tables"
                  else
                    echo "Usage: db-query tables [pg|sqlite] <database>" >&2
                  fi
                  ;;
                *)
                  echo "Usage: db-query <type> <database> [query]"
                  echo ""
                  echo "Database types:"
                  echo "  pg|postgres <db> [query] - PostgreSQL database"
                  echo "  sqlite <file> [query]    - SQLite database file"
                  echo "  temporal [query]         - Temporal workflow database"
                  echo "  n8n [query]              - n8n automation database"
                  echo "  tables <type> <db>       - List tables in database"
                  echo ""
                  echo "Environment variables:"
                  echo "  DB_HOST, DB_PORT, DB_USER - PostgreSQL connection"
                  echo "  TEMPORAL_DB_* / N8N_DB_*  - Service-specific settings"
                  echo "  N8N_DB_PASSWORD           - Required for n8n database access"
                  echo ""
                  echo "Security Notes:"
                  echo "  - Use interactive mode (no query argument) for sensitive queries"
                  echo "  - Passwords in PGPASSWORD are visible in process listings"
                  echo "  - Consider using ~/.pgpass for PostgreSQL authentication"
                  echo ""
                  echo "Examples:"
                  echo "  db-query pg mydb 'SELECT * FROM users'"
                  echo "  db-query sqlite ./data.db '.schema'"
                  echo "  db-query temporal 'SELECT * FROM executions LIMIT 10'"
                  ;;
              esac
            '')
            # Temporal workflow management
            (pkgs.writeShellScriptBin "temporal-ctl" ''
              # Temporal workflow management
              # Usage: temporal-ctl [status|workflows|start|query]

              # Validate environment variables
              validate_host() {
                # Allow hostname:port or just hostname format
                if [[ "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?(:[0-9]+)?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid host format: $1" >&2
                exit 1
              }
              
              validate_url() {
                # Basic URL validation for http/https
                if [[ "$1" =~ ^https?://[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?(:[0-9]+)?(/.*)?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid URL format: $1" >&2
                exit 1
              }
              
              validate_namespace() {
                if [[ "$1" =~ ^[a-zA-Z0-9._-]+$ ]]; then
                  return 0
                fi
                echo "Error: Invalid namespace format: $1 (must be alphanumeric with .-_)" >&2
                exit 1
              }

              TEMPORAL_ADDRESS="''${TEMPORAL_ADDRESS:-localhost:7233}"
              TEMPORAL_NAMESPACE="''${TEMPORAL_NAMESPACE:-default}"
              TEMPORAL_UI="''${TEMPORAL_UI:-http://localhost:8088}"
              
              validate_host "$TEMPORAL_ADDRESS"
              validate_namespace "$TEMPORAL_NAMESPACE"
              validate_url "$TEMPORAL_UI"

              case "''${1:-status}" in
                status)
                  echo "Temporal Status"
                  echo "==============="
                  echo "  Address:   $TEMPORAL_ADDRESS"
                  echo "  Namespace: $TEMPORAL_NAMESPACE"
                  echo "  Web UI:    $TEMPORAL_UI"
                  echo ""
                  if command -v temporal >/dev/null 2>&1; then
                    temporal operator cluster health --address "$TEMPORAL_ADDRESS" 2>/dev/null && echo "  Status: HEALTHY" || echo "  Status: UNAVAILABLE"
                  elif curl -sf "$TEMPORAL_UI" >/dev/null 2>&1; then
                    echo "  Status: UI REACHABLE (CLI not installed)"
                  else
                    echo "  Status: UNAVAILABLE"
                    echo ""
                    echo "Start with: docker compose -f docker-compose.temporal.yml up -d"
                  fi
                  ;;
                workflows|list)
                  # List workflows
                  if command -v temporal >/dev/null 2>&1; then
                    echo "Recent Workflows (namespace: $TEMPORAL_NAMESPACE):"
                    temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" "''${@:2}"
                  else
                    echo "Temporal CLI not installed. Using API..."
                    curl -s "$TEMPORAL_UI/api/v1/namespaces/$TEMPORAL_NAMESPACE/workflows" 2>/dev/null | jq '.executions[:10]' || echo "Unable to query workflows"
                  fi
                  ;;
                running)
                  # List running workflows
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Running'"
                  ;;
                failed)
                  # List failed workflows
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Failed'"
                  ;;
                describe)
                  # Describe a workflow
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl describe <workflow-id>" >&2
                    exit 1
                  fi
                  temporal workflow describe --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
                  ;;
                history)
                  # Get workflow history
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl history <workflow-id>" >&2
                    exit 1
                  fi
                  temporal workflow show --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
                  ;;
                terminate)
                  # Terminate a workflow
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  if [ -z "$2" ]; then
                    echo "Usage: temporal-ctl terminate <workflow-id> [reason]" >&2
                    exit 1
                  fi
                  REASON="''${3:-Terminated via temporal-ctl}"
                  temporal workflow terminate --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2" --reason "$REASON"
                  ;;
                namespaces)
                  # List namespaces
                  if ! command -v temporal >/dev/null 2>&1; then
                    echo "Error: temporal CLI not found" >&2
                    echo "Install from: https://docs.temporal.io/cli" >&2
                    exit 1
                  fi
                  temporal operator namespace list --address "$TEMPORAL_ADDRESS"
                  ;;
                ui)
                  # Open Temporal UI
                  open-url-helper "$TEMPORAL_UI"
                  ;;
                *)
                  echo "Usage: temporal-ctl <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  status                - Check Temporal service status"
                  echo "  workflows|list        - List recent workflows"
                  echo "  running               - List running workflows"
                  echo "  failed                - List failed workflows"
                  echo "  describe <id>         - Describe workflow by ID"
                  echo "  history <id>          - Show workflow event history"
                  echo "  terminate <id> [msg]  - Terminate a workflow"
                  echo "  namespaces            - List namespaces"
                  echo "  ui                    - Open Temporal Web UI"
                  echo ""
                  echo "Environment:"
                  echo "  TEMPORAL_ADDRESS   - Server address (default: localhost:7233)"
                  echo "  TEMPORAL_NAMESPACE - Namespace (default: default)"
                  echo "  TEMPORAL_UI        - Web UI URL (default: http://localhost:8088)"
                  echo ""
                  echo "Security Note:"
                  echo "  Use HTTPS URLs for TEMPORAL_UI in production environments"
                  ;;
              esac
            '')
            # n8n workflow management
            (pkgs.writeShellScriptBin "n8n-ctl" ''
              # n8n workflow automation management
              # Usage: n8n-ctl [status|workflows|executions|start|stop]
              
              # Validate environment variables
              validate_host() {
                # Hostname format validation
                if [[ "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?$ ]]; then
                  return 0
                fi
                echo "Error: Invalid host format: $1" >&2
                exit 1
              }
              
              validate_port() {
                if [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le 65535 ]; then
                  return 0
                fi
                echo "Error: Invalid port number: $1 (must be 1-65535)" >&2
                exit 1
              }
              
              validate_workflow_id() {
                # Require alphanumeric start, allow hyphens/underscores in the middle
                if [[ "$1" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
                  return 0
                fi
                echo "Error: Invalid workflow ID format: $1 (must start with alphanumeric)" >&2
                exit 1
              }

              N8N_HOST="''${N8N_HOST:-localhost}"
              N8N_PORT="''${N8N_PORT:-5678}"
              
              validate_host "$N8N_HOST"
              validate_port "$N8N_PORT"
              
              N8N_URL="http://$N8N_HOST:$N8N_PORT"
              N8N_API_KEY="''${N8N_API_KEY:-}"

              # API helper
              n8n_api() {
                if [ -n "$N8N_API_KEY" ]; then
                  curl -sf -H "X-N8N-API-KEY: $N8N_API_KEY" "$N8N_URL/api/v1$1"
                else
                  curl -sf "$N8N_URL/api/v1$1"
                fi
              }
              
              # API helper with method and data
              n8n_api_request() {
                local method="$1"
                local path="$2"
                local data="$3"
                
                if [ -n "$N8N_API_KEY" ]; then
                  curl -sf -X "$method" -H "Content-Type: application/json" \
                    -H "X-N8N-API-KEY: $N8N_API_KEY" \
                    -d "$data" "$N8N_URL/api/v1$path"
                else
                  curl -sf -X "$method" -H "Content-Type: application/json" \
                    -d "$data" "$N8N_URL/api/v1$path"
                fi
              }

              case "''${1:-status}" in
                status)
                  echo "n8n Status"
                  echo "=========="
                  echo "  URL: $N8N_URL"
                  echo ""
                  echo "  Security Note: For production, configure N8N_HOST to point to HTTPS endpoints"
                  echo "                 (e.g., N8N_HOST=n8n.example.com N8N_PORT=443)"
                  echo ""
                  if curl -sf "$N8N_URL/healthz" >/dev/null 2>&1; then
                    echo "  Status: HEALTHY"
                    echo ""
                    echo "  Workflows: $(n8n_api '/workflows' 2>/dev/null | jq '.data | length' || echo 'N/A')"
                  else
                    echo "  Status: UNAVAILABLE"
                    echo ""
                    echo "Start with: docker compose -f docker-compose.automation.yml up -d"
                  fi
                  ;;
                workflows|list)
                  echo "n8n Workflows:"
                  n8n_api '/workflows' | jq -r '.data[] | "  [\(.active | if . then "ON" else "OFF" end)] \(.id): \(.name)"' 2>/dev/null || echo "Unable to list workflows"
                  ;;
                executions)
                  # List recent executions
                  LIMIT="''${2:-10}"
                  
                  # Validate LIMIT parameter
                  if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
                    echo "Error: LIMIT must be a positive integer" >&2
                    exit 1
                  fi
                  
                  echo "Recent Executions (last $LIMIT):"
                  n8n_api "/executions?limit=$LIMIT" | jq -r '.data[] | "  [\(.status)] \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list executions"
                  ;;
                failed)
                  # List failed executions
                  echo "Failed Executions:"
                  n8n_api '/executions?status=error&limit=20' | jq -r '.data[] | "  \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list failed executions"
                  ;;
                activate)
                  # Activate a workflow
                  if [ -z "$2" ]; then
                    echo "Usage: n8n-ctl activate <workflow-id>" >&2
                    exit 1
                  fi
                  
                  validate_workflow_id "$2"
                  
                  n8n_api_request "PATCH" "/workflows/$2" '{"active": true}' | jq '.active' 2>/dev/null
                  echo "Workflow $2 activated"
                  ;;
                deactivate)
                  # Deactivate a workflow
                  if [ -z "$2" ]; then
                    echo "Usage: n8n-ctl deactivate <workflow-id>" >&2
                    exit 1
                  fi
                  
                  validate_workflow_id "$2"
                  
                  n8n_api_request "PATCH" "/workflows/$2" '{"active": false}' | jq '.active' 2>/dev/null
                  echo "Workflow $2 deactivated"
                  ;;
                ui)
                  # Open n8n UI
                  open-url-helper "$N8N_URL"
                  ;;
                *)
                  echo "Usage: n8n-ctl <command> [args]"
                  echo ""
                  echo "Commands:"
                  echo "  status              - Check n8n service status"
                  echo "  workflows|list      - List all workflows"
                  echo "  executions [limit]  - List recent executions"
                  echo "  failed              - List failed executions"
                  echo "  activate <id>       - Activate a workflow"
                  echo "  deactivate <id>     - Deactivate a workflow"
                  echo "  ui                  - Open n8n Web UI"
                  echo ""
                  echo "Environment:"
                  echo "  N8N_HOST    - n8n host (default: localhost)"
                  echo "  N8N_PORT    - n8n port (default: 5678)"
                  echo "  N8N_API_KEY - API key for authentication"
                  echo ""
                  echo "Security Notes:"
                  echo "  - Configure N8N_HOST to use HTTPS endpoints in production"
                  echo "  - Always set N8N_API_KEY for authenticated access"
                  ;;
              esac
            '')
            # Workflow status dashboard
            (pkgs.writeShellScriptBin "workflow-status" ''
              # Combined workflow status dashboard
              # Usage: workflow-status [all|github|temporal|n8n|docker]

              check_service() {
                local name="$1"
                local url="$2"
                local status
                if curl -sf --connect-timeout 2 "$url" >/dev/null 2>&1; then
                  status="UP"
                else
                  status="DOWN"
                fi
                printf "  %-20s %s\n" "$name:" "$status"
              }

              case "''${1:-all}" in
                all)
                  echo "========================================"
                  echo "       Workflow Infrastructure Status"
                  echo "========================================"
                  echo ""

                  echo "Services:"
                  check_service "Temporal" "http://localhost:8088"
                  check_service "n8n" "http://localhost:5678/healthz"
                  check_service "Prometheus" "http://localhost:9090/-/ready"
                  check_service "NATS" "http://localhost:8222/varz"
                  check_service "Keycloak" "http://localhost:8080"
                  check_service "Vault" "http://localhost:8200/v1/sys/health"
                  echo ""

                  echo "GitHub Issues:"
                  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
                    OPEN=$(gh issue list --state open --json number --jq 'length' 2>/dev/null || echo "?")
                    printf "  %-20s %s\n" "Open issues:" "$OPEN"
                  else
                    echo "  (gh CLI not authenticated)"
                  fi
                  echo ""

                  echo "Docker Containers:"
                  if command -v docker >/dev/null 2>&1; then
                    RUNNING=$(docker ps -q 2>/dev/null | wc -l)
                    printf "  %-20s %s\n" "Running:" "$RUNNING"
                  else
                    echo "  (docker not available)"
                  fi
                  echo ""

                  echo "========================================"
                  ;;
                github)
                  echo "GitHub Status"
                  echo "============="
                  if command -v gh >/dev/null 2>&1; then
                    echo ""
                    echo "Issues:"
                    echo "  Open:   $(gh issue list --state open --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo "  Closed: $(gh issue list --state closed --limit 1000 --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo ""
                    echo "Pull Requests:"
                    echo "  Open:   $(gh pr list --state open --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo "  Merged: $(gh pr list --state merged --limit 100 --json number --jq 'length' 2>/dev/null || echo 'N/A')"
                    echo ""
                    echo "Recent Activity:"
                    gh issue list --limit 5 2>/dev/null | head -5
                  else
                    echo "gh CLI not available"
                  fi
                  ;;
                temporal)
                  exec temporal-ctl status
                  ;;
                n8n)
                  exec n8n-ctl status
                  ;;
                docker)
                  echo "Docker Container Status"
                  echo "======================="
                  if command -v docker >/dev/null 2>&1; then
                    docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null || echo "Docker not running"
                  else
                    echo "Docker not available"
                  fi
                  ;;
                *)
                  echo "Usage: workflow-status [all|github|temporal|n8n|docker]"
                  echo ""
                  echo "Commands:"
                  echo "  all      - Show all services status (default)"
                  echo "  github   - GitHub issues and PRs status"
                  echo "  temporal - Temporal workflow engine status"
                  echo "  n8n      - n8n automation platform status"
                  echo "  docker   - Docker containers status"
                  ;;
              esac
            '')
            # Codebase database loader and query tool
            (pkgs.writeShellScriptBin "codebase-db" ''
              # Load ALL codebase files into SQLite database for comprehensive querying
              # Designed for configuration workspaces - indexes everything
              # Usage: codebase-db [init|load|query|search|files|stats|configs|deps|envvars|...]

              DB_FILE="''${CODEBASE_DB:-./codebase.db}"
              CODEBASE_ROOT="''${CODEBASE_ROOT:-.}"

              # Directories to exclude (only truly non-content dirs)
              EXCLUDE_DIRS=".git node_modules __pycache__ .cache .npm .cargo .mypy_cache .pytest_cache"

              # Max file size to load (5MB - generous for config files)
              MAX_FILE_SIZE=5242880

              init_db() {
                echo "Initializing codebase database: $DB_FILE"
                sqlite3 "$DB_FILE" << 'SCHEMA'
                  DROP TABLE IF EXISTS files;
                  DROP TABLE IF EXISTS file_lines;
                  DROP TABLE IF EXISTS files_fts;
                  DROP TABLE IF EXISTS metadata;

                  CREATE TABLE files (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    path TEXT UNIQUE NOT NULL,
                    relative_path TEXT NOT NULL,
                    filename TEXT NOT NULL,
                    extension TEXT,
                    directory TEXT NOT NULL,
                    size_bytes INTEGER,
                    line_count INTEGER,
                    content TEXT,
                    file_type TEXT,
                    is_config INTEGER DEFAULT 0,
                    is_hidden INTEGER DEFAULT 0,
                    loaded_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                  );

                  CREATE TABLE file_lines (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    file_id INTEGER NOT NULL,
                    line_number INTEGER NOT NULL,
                    content TEXT,
                    trimmed_content TEXT,
                    indent_level INTEGER DEFAULT 0,
                    FOREIGN KEY (file_id) REFERENCES files(id)
                  );

                  CREATE TABLE metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                  );

                  CREATE INDEX idx_files_path ON files(path);
                  CREATE INDEX idx_files_relative_path ON files(relative_path);
                  CREATE INDEX idx_files_filename ON files(filename);
                  CREATE INDEX idx_files_extension ON files(extension);
                  CREATE INDEX idx_files_directory ON files(directory);
                  CREATE INDEX idx_files_file_type ON files(file_type);
                  CREATE INDEX idx_files_is_config ON files(is_config);
                  CREATE INDEX idx_file_lines_file_id ON file_lines(file_id);
                  CREATE INDEX idx_file_lines_content ON file_lines(content);
                  CREATE INDEX idx_file_lines_trimmed ON file_lines(trimmed_content);

                  CREATE VIRTUAL TABLE files_fts USING fts5(
                    path, filename, content, tokenize='porter unicode61'
                  );

                  CREATE VIRTUAL TABLE lines_fts USING fts5(
                    content, tokenize='porter unicode61'
                  );
SCHEMA
                echo "Database initialized with enhanced schema."
              }

              # Determine file type based on extension/name
              get_file_type() {
                local file="$1"
                local ext="$2"
                local name=$(basename "$file")
                case "$name" in
                  Dockerfile*|*.dockerfile) echo "dockerfile" ;;
                  Makefile|GNUmakefile) echo "makefile" ;;
                  CMakeLists.txt) echo "cmake" ;;
                  *.nix) echo "nix" ;;
                  *.py) echo "python" ;;
                  *.sh|*.bash|*.zsh) echo "shell" ;;
                  *.ps1|*.psm1) echo "powershell" ;;
                  *.yml|*.yaml) echo "yaml" ;;
                  *.json) echo "json" ;;
                  *.toml) echo "toml" ;;
                  *.xml) echo "xml" ;;
                  *.md|*.markdown) echo "markdown" ;;
                  *.rst) echo "rst" ;;
                  *.txt) echo "text" ;;
                  *.ini|*.cfg|*.conf) echo "ini" ;;
                  *.env*|.env*) echo "env" ;;
                  *.lock) echo "lockfile" ;;
                  *.rs) echo "rust" ;;
                  *.go) echo "go" ;;
                  *.js|*.mjs|*.cjs) echo "javascript" ;;
                  *.ts|*.tsx) echo "typescript" ;;
                  *.lua) echo "lua" ;;
                  *.vim) echo "vim" ;;
                  *.css|*.scss|*.sass) echo "css" ;;
                  *.html|*.htm) echo "html" ;;
                  *.sql) echo "sql" ;;
                  *.rego) echo "rego" ;;
                  *.hcl|*.tf) echo "hcl" ;;
                  *) echo "other" ;;
                esac
              }

              # Check if file is a config file
              is_config_file() {
                local file="$1"
                local name=$(basename "$file")
                case "$name" in
                  *.nix|*.toml|*.yaml|*.yml|*.json|*.ini|*.cfg|*.conf|*.env*|.env*|\
                  *.xml|*.hcl|*.tf|flake.lock|Cargo.lock|package-lock.json|pixi.lock|\
                  .gitignore|.gitattributes|.editorconfig|.envrc|.pre-commit*|\
                  Dockerfile*|docker-compose*|Makefile|CMakeLists.txt|*.cmake|\
                  tsconfig.json|jsconfig.json|*.config.js|*.config.ts|\
                  pyproject.toml|setup.py|setup.cfg|requirements*.txt|\
                  Gemfile|Rakefile|*.gemspec|Cargo.toml|go.mod|go.sum)
                    echo 1 ;;
                  *) echo 0 ;;
                esac
              }

              load_files() {
                echo "Loading ALL files from: $CODEBASE_ROOT"
                echo "Database: $DB_FILE"
                echo "Max file size: $MAX_FILE_SIZE bytes"
                echo ""

                # Record start time
                start_time=$(date +%s)

                # Build find exclusions
                EXCLUDE_ARGS=""
                for dir in $EXCLUDE_DIRS; do
                  EXCLUDE_ARGS="$EXCLUDE_ARGS -path '*/$dir' -prune -o -path '*/$dir/*' -prune -o"
                done

                # Find ALL files (no extension filtering)
                count=0
                skipped=0
                binary=0

                eval "find '$CODEBASE_ROOT' $EXCLUDE_ARGS -type f -print" 2>/dev/null | while read -r filepath; do
                  filename=$(basename "$filepath")
                  relative_path="''${filepath#$CODEBASE_ROOT/}"
                  directory=$(dirname "$relative_path")
                  extension="''${filename##*.}"
                  [ "$extension" = "$filename" ] && extension=""
                  is_hidden=0
                  [[ "$filename" == .* ]] && is_hidden=1

                  # Get file size
                  size=$(stat -c%s "$filepath" 2>/dev/null || stat -f%z "$filepath" 2>/dev/null || echo 0)

                  # Skip files that are too large
                  if [ "$size" -gt "$MAX_FILE_SIZE" ]; then
                    echo "  Skipped (>5MB): $relative_path"
                    skipped=$((skipped + 1))
                    continue
                  fi

                  # Skip binary files
                  if file "$filepath" 2>/dev/null | grep -qE "binary|executable|ELF|Mach-O|PE32|archive|compressed"; then
                    binary=$((binary + 1))
                    continue
                  fi

                  # Get file type and config status
                  file_type=$(get_file_type "$filepath" "$extension")
                  is_config=$(is_config_file "$filepath")

                  # Read content
                  content=$(cat "$filepath" 2>/dev/null | sed "s/'/''/g" || echo "")
                  line_count=$(echo "$content" | wc -l | tr -d ' ')

                  # Insert file record
                  sqlite3 "$DB_FILE" "INSERT OR REPLACE INTO files
                    (path, relative_path, filename, extension, directory, size_bytes, line_count, content, file_type, is_config, is_hidden)
                    VALUES ('$filepath', '$relative_path', '$filename', '$extension', '$directory', $size, $line_count, '$content', '$file_type', $is_config, $is_hidden);"

                  # Get file ID and insert into FTS
                  file_id=$(sqlite3 "$DB_FILE" "SELECT id FROM files WHERE path='$filepath';")
                  sqlite3 "$DB_FILE" "INSERT INTO files_fts(rowid, path, filename, content) VALUES ($file_id, '$filepath', '$filename', '$content');"

                  # Clear old lines and insert new
                  sqlite3 "$DB_FILE" "DELETE FROM file_lines WHERE file_id=$file_id;"

                  line_num=1
                  echo "$content" | while IFS= read -r line; do
                    escaped_line=$(echo "$line" | sed "s/'/''/g")
                    trimmed=$(echo "$line" | sed 's/^[[:space:]]*//' | sed "s/'/''/g")
                    indent=$(echo "$line" | sed 's/[^[:space:]].*//' | wc -c)
                    indent=$((indent - 1))
                    sqlite3 "$DB_FILE" "INSERT INTO file_lines (file_id, line_number, content, trimmed_content, indent_level)
                      VALUES ($file_id, $line_num, '$escaped_line', '$trimmed', $indent);"
                    line_num=$((line_num + 1))
                  done

                  # Insert lines into FTS
                  sqlite3 "$DB_FILE" "INSERT INTO lines_fts(rowid, content)
                    SELECT id, content FROM file_lines WHERE file_id=$file_id;"

                  count=$((count + 1))
                  echo "  [$file_type] $relative_path ($line_count lines)"
                done

                end_time=$(date +%s)
                duration=$((end_time - start_time))

                # Store metadata
                sqlite3 "$DB_FILE" "INSERT OR REPLACE INTO metadata (key, value) VALUES
                  ('root_path', '$CODEBASE_ROOT'),
                  ('load_time', '$duration'),
                  ('last_loaded', datetime('now'));"

                echo ""
                echo "Loading complete in ''${duration}s. Run 'codebase-db stats' for summary."
              }

              case "''${1:-help}" in
                init)
                  init_db
                  ;;
                load)
                  if [ ! -f "$DB_FILE" ]; then
                    init_db
                  fi
                  load_files
                  ;;
                reload)
                  init_db
                  load_files
                  ;;
                query|q)
                  # Execute arbitrary SQL query
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db query '<SQL>'" >&2
                    echo ""
                    echo "Examples:"
                    echo "  codebase-db query 'SELECT path FROM files WHERE extension=\"nix\"'"
                    echo "  codebase-db query 'SELECT * FROM files WHERE content LIKE \"%TODO%\"'"
                    exit 1
                  fi
                  shift
                  sqlite3 -header -column "$DB_FILE" "$*"
                  ;;
                search|s)
                  # Full-text search in file contents
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db search <pattern>" >&2
                    exit 1
                  fi
                  shift
                  PATTERN="$*"
                  echo "Searching for: $PATTERN"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT f.path, fl.line_number, fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 100;"
                  ;;
                grep|g)
                  # Search with context (like grep)
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db grep <pattern> [context_lines]" >&2
                    exit 1
                  fi
                  PATTERN="$2"
                  CONTEXT="''${3:-2}"
                  echo "Searching for: $PATTERN (context: $CONTEXT lines)"
                  echo ""
                  sqlite3 "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number || ': ' || fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 50;" | while read -r line; do
                    echo "$line"
                  done
                  ;;
                files|ls)
                  # List files, optionally filtered by extension
                  EXT="$2"
                  if [ -n "$EXT" ]; then
                    echo "Files with extension: $EXT"
                    sqlite3 -column "$DB_FILE" \
                      "SELECT path, line_count, size_bytes FROM files WHERE extension='$EXT' ORDER BY path;"
                  else
                    echo "All indexed files:"
                    sqlite3 -column "$DB_FILE" \
                      "SELECT path, extension, line_count FROM files ORDER BY path;"
                  fi
                  ;;
                functions|fn)
                  # Find function definitions (basic pattern matching)
                  LANG="''${2:-all}"
                  echo "Function definitions ($LANG):"
                  echo ""
                  case "$LANG" in
                    py|python)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'py' AND fl.content LIKE '%def %(%'
                         ORDER BY f.path, fl.line_number;"
                      ;;
                    sh|bash)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension IN ('sh', 'bash') AND (fl.content LIKE '%() {%' OR fl.content LIKE '%function %')
                         ORDER BY f.path, fl.line_number;"
                      ;;
                    nix)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'nix' AND fl.content LIKE '% = %{%' AND fl.content LIKE '%:%'
                         ORDER BY f.path, fl.line_number
                         LIMIT 50;"
                      ;;
                    *)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.path, fl.line_number, substr(fl.content, 1, 80)
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE fl.content LIKE '%def %' OR fl.content LIKE '%function %' OR fl.content LIKE '%fn %' OR fl.content LIKE '%func %'
                         ORDER BY f.path, fl.line_number
                         LIMIT 100;"
                      ;;
                  esac
                  ;;
                todos)
                  # Find TODO/FIXME comments
                  echo "TODO/FIXME items:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%TODO%' OR fl.content LIKE '%FIXME%' OR fl.content LIKE '%XXX%' OR fl.content LIKE '%HACK%'
                     ORDER BY f.path, fl.line_number;"
                  ;;
                issues)
                  # Find potential issues (errors, warnings in code)
                  echo "Potential issues (error handling, warnings):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%error%' OR fl.content LIKE '%Error%' OR fl.content LIKE '%WARNING%' OR fl.content LIKE '%WARN%' OR fl.content LIKE '%panic%' OR fl.content LIKE '%fail%'
                     ORDER BY f.path, fl.line_number
                     LIMIT 100;"
                  ;;

                # === CONFIGURATION QUERY TOOLS ===

                configs)
                  # List all configuration files
                  echo "Configuration Files:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, size_bytes
                     FROM files WHERE is_config = 1
                     ORDER BY file_type, relative_path;"
                  ;;
                dirs)
                  # List directories with file counts
                  echo "Directories:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT directory, COUNT(*) as files, SUM(line_count) as lines, SUM(size_bytes) as bytes
                     FROM files
                     GROUP BY directory
                     ORDER BY files DESC;"
                  ;;
                types)
                  # List files grouped by type
                  TYPE="$2"
                  if [ -n "$TYPE" ]; then
                    echo "Files of type: $TYPE"
                    sqlite3 -header -column "$DB_FILE" \
                      "SELECT relative_path, line_count, size_bytes
                       FROM files WHERE file_type = '$TYPE'
                       ORDER BY relative_path;"
                  else
                    echo "File types:"
                    sqlite3 -header -column "$DB_FILE" \
                      "SELECT file_type, COUNT(*) as count, SUM(line_count) as lines
                       FROM files GROUP BY file_type ORDER BY count DESC;"
                  fi
                  ;;
                hidden)
                  # List hidden files
                  echo "Hidden Files:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count
                     FROM files WHERE is_hidden = 1
                     ORDER BY relative_path;"
                  ;;
                envvars)
                  # Find environment variable definitions and usages
                  echo "Environment Variables:"
                  echo ""
                  echo "=== Definitions (export VAR=) ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%export %=%'
                        OR fl.content LIKE '%ENV %=%'
                        OR fl.content LIKE '%environment%=%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  echo ""
                  echo "=== Usages (\$VAR or \${VAR}) ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%\$%' AND fl.content NOT LIKE '%#%\$%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                secrets)
                  # Find potential secrets (API keys, passwords, tokens)
                  echo "Potential Secrets (REVIEW CAREFULLY):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 120)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE (fl.content LIKE '%password%' OR fl.content LIKE '%PASSWORD%'
                        OR fl.content LIKE '%secret%' OR fl.content LIKE '%SECRET%'
                        OR fl.content LIKE '%api_key%' OR fl.content LIKE '%API_KEY%'
                        OR fl.content LIKE '%apikey%' OR fl.content LIKE '%APIKEY%'
                        OR fl.content LIKE '%token%' OR fl.content LIKE '%TOKEN%'
                        OR fl.content LIKE '%private_key%' OR fl.content LIKE '%PRIVATE_KEY%'
                        OR fl.content LIKE '%credential%' OR fl.content LIKE '%CREDENTIAL%'
                        OR fl.content LIKE '%aws_access%' OR fl.content LIKE '%AWS_ACCESS%')
                       AND f.filename NOT LIKE '%.md'
                       AND f.filename NOT LIKE '%.txt'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                imports)
                  # Find import/require statements
                  LANG="''${2:-all}"
                  echo "Import statements ($LANG):"
                  echo ""
                  case "$LANG" in
                    py|python)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'py' AND (fl.content LIKE 'import %' OR fl.content LIKE 'from %import%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    nix)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension = 'nix' AND (fl.content LIKE '%import %' OR fl.content LIKE '%inputs.%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    js|ts|javascript|typescript)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, fl.content
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE f.extension IN ('js', 'ts', 'tsx', 'jsx')
                           AND (fl.content LIKE '%import %' OR fl.content LIKE '%require(%')
                         ORDER BY f.relative_path, fl.line_number;"
                      ;;
                    *)
                      sqlite3 -column "$DB_FILE" \
                        "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                         FROM file_lines fl JOIN files f ON fl.file_id = f.id
                         WHERE fl.content LIKE '%import %' OR fl.content LIKE '%require(%' OR fl.content LIKE '%include %'
                         ORDER BY f.relative_path, fl.line_number
                         LIMIT 200;"
                      ;;
                  esac
                  ;;
                deps)
                  # Find dependency definitions
                  echo "Dependencies:"
                  echo ""
                  echo "=== Package definitions ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE (f.filename IN ('package.json', 'Cargo.toml', 'go.mod', 'pyproject.toml', 'requirements.txt', 'Gemfile', 'pixi.toml', 'flake.nix')
                        OR f.filename LIKE 'requirements%.txt')
                       AND fl.trimmed_content != ''
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 200;"
                  ;;
                ports)
                  # Find port numbers in configuration
                  echo "Port Numbers:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%port%'
                        OR fl.content LIKE '%PORT%'
                        OR fl.content LIKE '%:80%'
                        OR fl.content LIKE '%:443%'
                        OR fl.content LIKE '%:8080%'
                        OR fl.content LIKE '%:3000%'
                        OR fl.content LIKE '%:5000%'
                        OR fl.content LIKE '%:9090%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                urls)
                  # Find URLs and endpoints
                  echo "URLs and Endpoints:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 150)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%http://%'
                        OR fl.content LIKE '%https://%'
                        OR fl.content LIKE '%localhost%'
                        OR fl.content LIKE '%127.0.0.1%'
                        OR fl.content LIKE '%0.0.0.0%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 150;"
                  ;;
                paths)
                  # Find path definitions
                  echo "Path Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%PATH%'
                        OR fl.content LIKE '%path =%'
                        OR fl.content LIKE '%path:%'
                        OR fl.content LIKE '%/usr/%'
                        OR fl.content LIKE '%/etc/%'
                        OR fl.content LIKE '%/home/%'
                        OR fl.content LIKE '%/var/%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                services)
                  # Find service definitions
                  echo "Service Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%service%'
                        OR fl.content LIKE '%Service%'
                        OR fl.content LIKE '%systemd%'
                        OR fl.content LIKE '%ExecStart%'
                        OR fl.content LIKE '%container%'
                        OR fl.content LIKE '%docker%'
                        OR fl.content LIKE '%podman%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                options)
                  # Find option/setting definitions (especially for Nix)
                  echo "Option/Setting Definitions:"
                  echo ""
                  echo "=== Nix Options ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE f.extension = 'nix'
                       AND (fl.content LIKE '%mkOption%' OR fl.content LIKE '%mkEnableOption%' OR fl.content LIKE '%options.%')
                     ORDER BY f.relative_path, fl.line_number;"
                  echo ""
                  echo "=== General Settings ==="
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%enable%=%'
                        OR fl.content LIKE '%enabled%:%'
                        OR fl.content LIKE '%config.%=%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                modules)
                  # Find module definitions
                  echo "Module Definitions:"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT f.relative_path || ':' || fl.line_number, substr(fl.content, 1, 100)
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%module%'
                        OR fl.content LIKE '%Module%'
                        OR fl.content LIKE '%MODULE%'
                        OR fl.content LIKE '%{ config,%'
                        OR fl.content LIKE '%{ lib,%'
                     ORDER BY f.relative_path, fl.line_number
                     LIMIT 100;"
                  ;;
                fts)
                  # Full-text search using FTS5 (more powerful)
                  if [ -z "$2" ]; then
                    echo "Usage: codebase-db fts <query>" >&2
                    echo ""
                    echo "FTS5 query syntax:"
                    echo "  'word1 word2'     - Both words (AND)"
                    echo "  'word1 OR word2'  - Either word"
                    echo "  '\"exact phrase\"'  - Exact phrase"
                    echo "  'word*'           - Prefix match"
                    echo "  'word1 NOT word2' - Exclude word2"
                    exit 1
                  fi
                  shift
                  QUERY="$*"
                  echo "FTS Search: $QUERY"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT path, filename, snippet(files_fts, 2, '>>>', '<<<', '...', 50) as match
                     FROM files_fts
                     WHERE files_fts MATCH '$QUERY'
                     ORDER BY rank
                     LIMIT 50;"
                  ;;
                context)
                  # Show context around a specific line
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db context <file-pattern> <line-number> [context-lines]" >&2
                    exit 1
                  fi
                  FILE_PATTERN="$2"
                  LINE_NUM="$3"
                  CTX="''${4:-5}"
                  START=$((LINE_NUM - CTX))
                  END=$((LINE_NUM + CTX))
                  [ $START -lt 1 ] && START=1
                  echo "Context for $FILE_PATTERN:$LINE_NUM (±$CTX lines):"
                  echo ""
                  sqlite3 -column "$DB_FILE" \
                    "SELECT fl.line_number || ': ' || fl.content
                     FROM file_lines fl JOIN files f ON fl.file_id = f.id
                     WHERE f.relative_path LIKE '%$FILE_PATTERN%'
                       AND fl.line_number BETWEEN $START AND $END
                     ORDER BY fl.line_number;"
                  ;;
                diff)
                  # Compare two files
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db diff <file1-pattern> <file2-pattern>" >&2
                    exit 1
                  fi
                  FILE1="$2"
                  FILE2="$3"
                  echo "Comparing files matching: $FILE1 vs $FILE2"
                  echo ""
                  sqlite3 "$DB_FILE" "SELECT content FROM files WHERE relative_path LIKE '%$FILE1%' LIMIT 1;" > /tmp/cdb_diff1.txt
                  sqlite3 "$DB_FILE" "SELECT content FROM files WHERE relative_path LIKE '%$FILE2%' LIMIT 1;" > /tmp/cdb_diff2.txt
                  diff -u /tmp/cdb_diff1.txt /tmp/cdb_diff2.txt || true
                  rm -f /tmp/cdb_diff1.txt /tmp/cdb_diff2.txt
                  ;;
                duplicates)
                  # Find duplicate content
                  echo "Files with identical content:"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT GROUP_CONCAT(relative_path, ', ') as files, line_count, size_bytes
                     FROM files
                     GROUP BY content
                     HAVING COUNT(*) > 1
                     ORDER BY size_bytes DESC
                     LIMIT 20;"
                  ;;
                large)
                  # Find largest files
                  LIMIT="''${2:-20}"
                  echo "Largest files (top $LIMIT):"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, size_bytes
                     FROM files
                     ORDER BY size_bytes DESC
                     LIMIT $LIMIT;"
                  ;;
                recent)
                  # Files loaded in current session (by ID, since we don't track mtime)
                  LIMIT="''${2:-20}"
                  echo "Recently loaded files (last $LIMIT):"
                  echo ""
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT relative_path, file_type, line_count, loaded_at
                     FROM files
                     ORDER BY id DESC
                     LIMIT $LIMIT;"
                  ;;
                tree)
                  # Show directory tree structure
                  echo "Directory Tree:"
                  echo ""
                  sqlite3 "$DB_FILE" \
                    "SELECT DISTINCT directory FROM files ORDER BY directory;" | while read -r dir; do
                    depth=$(echo "$dir" | tr -cd '/' | wc -c)
                    indent=$(printf '%*s' $((depth * 2)) '')
                    name=$(basename "$dir")
                    count=$(sqlite3 "$DB_FILE" "SELECT COUNT(*) FROM files WHERE directory = '$dir';")
                    echo "''${indent}📁 $name/ ($count files)"
                  done
                  ;;

                stats)
                  # Show comprehensive database statistics
                  echo "═══════════════════════════════════════════════════════"
                  echo "           CODEBASE DATABASE STATISTICS"
                  echo "═══════════════════════════════════════════════════════"
                  echo ""
                  echo "Database: $DB_FILE"
                  echo ""
                  echo "── Overview ──────────────────────────────────────────"
                  sqlite3 "$DB_FILE" << 'STATS'
                    SELECT 'Total files:' as metric, COUNT(*) as value FROM files
                    UNION ALL SELECT 'Config files:', COUNT(*) FROM files WHERE is_config = 1
                    UNION ALL SELECT 'Hidden files:', COUNT(*) FROM files WHERE is_hidden = 1
                    UNION ALL SELECT 'Total lines:', SUM(line_count) FROM files
                    UNION ALL SELECT 'Total size (KB):', ROUND(SUM(size_bytes) / 1024.0, 2) FROM files
                    UNION ALL SELECT 'Directories:', COUNT(DISTINCT directory) FROM files;
STATS
                  echo ""
                  echo "── Files by Type ─────────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT file_type as type, COUNT(*) as files, SUM(line_count) as lines, ROUND(SUM(size_bytes)/1024.0, 1) as kb
                     FROM files GROUP BY file_type ORDER BY files DESC LIMIT 15;"
                  echo ""
                  echo "── Files by Extension ────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT extension as ext, COUNT(*) as files, SUM(line_count) as lines
                     FROM files WHERE extension != '' GROUP BY extension ORDER BY files DESC LIMIT 15;"
                  echo ""
                  echo "── Top Directories ───────────────────────────────────"
                  sqlite3 -header -column "$DB_FILE" \
                    "SELECT directory as dir, COUNT(*) as files, SUM(line_count) as lines
                     FROM files GROUP BY directory ORDER BY files DESC LIMIT 10;"
                  echo ""
                  # Show metadata if available
                  if sqlite3 "$DB_FILE" "SELECT 1 FROM metadata LIMIT 1;" 2>/dev/null | grep -q 1; then
                    echo "── Load Metadata ─────────────────────────────────────"
                    sqlite3 -column "$DB_FILE" "SELECT key, value FROM metadata;"
                  fi
                  ;;
                export)
                  # Export search results to file
                  if [ -z "$2" ] || [ -z "$3" ]; then
                    echo "Usage: codebase-db export <pattern> <output-file>" >&2
                    exit 1
                  fi
                  PATTERN="$2"
                  OUTPUT="$3"
                  echo "Exporting matches for '$PATTERN' to $OUTPUT..."
                  sqlite3 "$DB_FILE" \
                    "SELECT f.path || ':' || fl.line_number || ': ' || fl.content
                     FROM file_lines fl
                     JOIN files f ON fl.file_id = f.id
                     WHERE fl.content LIKE '%$PATTERN%'
                     ORDER BY f.path, fl.line_number;" > "$OUTPUT"
                  echo "Exported $(wc -l < "$OUTPUT") matches."
                  ;;
                shell)
                  # Open interactive SQLite shell
                  echo "Opening SQLite shell for: $DB_FILE"
                  echo "Tables: files, file_lines, files_fts"
                  echo ""
                  sqlite3 "$DB_FILE"
                  ;;
                *)
                  echo "Usage: codebase-db <command> [args]"
                  echo ""
                  echo "Loads ALL files in the workspace into SQLite for comprehensive querying."
                  echo ""
                  echo "DATABASE COMMANDS:"
                  echo "  init              - Initialize empty database"
                  echo "  load              - Load ALL files into database"
                  echo "  reload            - Reinitialize and reload all files"
                  echo "  stats             - Show database statistics"
                  echo "  shell             - Open interactive SQLite shell"
                  echo ""
                  echo "SEARCH COMMANDS:"
                  echo "  search <pattern>  - Search in file contents (LIKE)"
                  echo "  fts <query>       - Full-text search (FTS5 with ranking)"
                  echo "  grep <pattern>    - Search with file:line output"
                  echo "  query '<SQL>'     - Execute arbitrary SQL query"
                  echo "  context <f> <ln>  - Show context around a line"
                  echo "  export <p> <f>    - Export search results to file"
                  echo ""
                  echo "FILE LISTING:"
                  echo "  files [ext]       - List files (optionally by extension)"
                  echo "  types [type]      - List files by type (nix, yaml, shell...)"
                  echo "  configs           - List all configuration files"
                  echo "  dirs              - List directories with file counts"
                  echo "  hidden            - List hidden files"
                  echo "  tree              - Show directory tree structure"
                  echo "  large [n]         - Show largest files"
                  echo "  recent [n]        - Show recently loaded files"
                  echo "  duplicates        - Find files with identical content"
                  echo ""
                  echo "CODE ANALYSIS:"
                  echo "  functions [lang]  - Find function definitions"
                  echo "  imports [lang]    - Find import/require statements"
                  echo "  modules           - Find module definitions"
                  echo "  todos             - Find TODO/FIXME comments"
                  echo "  issues            - Find error handling patterns"
                  echo "  diff <f1> <f2>    - Compare two files"
                  echo ""
                  echo "CONFIGURATION ANALYSIS:"
                  echo "  envvars           - Find environment variables"
                  echo "  secrets           - Find potential secrets (passwords, keys)"
                  echo "  deps              - Find dependency definitions"
                  echo "  ports             - Find port numbers"
                  echo "  urls              - Find URLs and endpoints"
                  echo "  paths             - Find path definitions"
                  echo "  services          - Find service definitions"
                  echo "  options           - Find option definitions (Nix mkOption)"
                  echo ""
                  echo "ENVIRONMENT:"
                  echo "  CODEBASE_DB   - Database file (default: ./codebase.db)"
                  echo "  CODEBASE_ROOT - Root directory to scan (default: .)"
                  echo ""
                  echo "EXAMPLES:"
                  echo "  codebase-db load                     # Load entire workspace"
                  echo "  codebase-db fts 'error AND handling' # FTS5 boolean search"
                  echo "  codebase-db configs                  # List all config files"
                  echo "  codebase-db types nix                # List all Nix files"
                  echo "  codebase-db envvars                  # Find env variables"
                  echo "  codebase-db secrets                  # Audit for secrets"
                  echo "  codebase-db context flake.nix 100 10 # Show lines 90-110"
                  echo "  codebase-db query 'SELECT * FROM files WHERE is_config=1'"
                  ;;
              esac
            '')
          ];
