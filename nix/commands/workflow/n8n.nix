# n8n workflow automation commands
# Extracted from workflow.nix for modular organization
{ pkgs, ... }:

[
  # n8n workflow management
  (pkgs.writeShellScriptBin "n8n-ctl" ''
    # n8n workflow automation management
    # Usage: n8n-ctl [status|workflows|executions|start|stop]

    validate_host() {
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
      echo "Error: Invalid port number: $1" >&2
      exit 1
    }

    validate_workflow_id() {
      if [[ "$1" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
        return 0
      fi
      echo "Error: Invalid workflow ID format: $1" >&2
      exit 1
    }

    N8N_HOST="''${N8N_HOST:-localhost}"
    N8N_PORT="''${N8N_PORT:-5678}"

    validate_host "$N8N_HOST"
    validate_port "$N8N_PORT"

    N8N_URL="http://$N8N_HOST:$N8N_PORT"
    N8N_API_KEY="''${N8N_API_KEY:-}"

    n8n_api() {
      if [ -n "$N8N_API_KEY" ]; then
        curl -sf -H "X-N8N-API-KEY: $N8N_API_KEY" "$N8N_URL/api/v1$1"
      else
        curl -sf "$N8N_URL/api/v1$1"
      fi
    }

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
        LIMIT="''${2:-10}"
        if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
          echo "Error: LIMIT must be a positive integer" >&2
          exit 1
        fi
        echo "Recent Executions (last $LIMIT):"
        n8n_api "/executions?limit=$LIMIT" | jq -r '.data[] | "  [\(.status)] \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list executions"
        ;;
      failed)
        echo "Failed Executions:"
        n8n_api '/executions?status=error&limit=20' | jq -r '.data[] | "  \(.id) - \(.workflowData.name // "Unknown") (\(.startedAt))"' 2>/dev/null || echo "Unable to list failed executions"
        ;;
      activate)
        if [ -z "$2" ]; then
          echo "Usage: n8n-ctl activate <workflow-id>" >&2
          exit 1
        fi
        validate_workflow_id "$2"
        n8n_api_request "PATCH" "/workflows/$2" '{"active": true}' | jq '.active' 2>/dev/null
        echo "Workflow $2 activated"
        ;;
      deactivate)
        if [ -z "$2" ]; then
          echo "Usage: n8n-ctl deactivate <workflow-id>" >&2
          exit 1
        fi
        validate_workflow_id "$2"
        n8n_api_request "PATCH" "/workflows/$2" '{"active": false}' | jq '.active' 2>/dev/null
        echo "Workflow $2 deactivated"
        ;;
      ui)
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
        ;;
    esac
  '')
]
