# Temporal workflow management commands
# Extracted from workflow.nix for modular organization
{ pkgs, ... }:

[
  # Temporal workflow management
  (pkgs.writeShellScriptBin "temporal-ctl" ''
    # Temporal workflow management
    # Usage: temporal-ctl [status|workflows|start|query]

    validate_host() {
      if [[ "$1" =~ ^[a-zA-Z0-9]([a-zA-Z0-9.-]*[a-zA-Z0-9])?(:[0-9]+)?$ ]]; then
        return 0
      fi
      echo "Error: Invalid host format: $1" >&2
      exit 1
    }

    validate_url() {
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
      echo "Error: Invalid namespace format: $1" >&2
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
        if command -v temporal >/dev/null 2>&1; then
          echo "Recent Workflows (namespace: $TEMPORAL_NAMESPACE):"
          temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" "''${@:2}"
        else
          echo "Temporal CLI not installed. Using API..."
          curl -s "$TEMPORAL_UI/api/v1/namespaces/$TEMPORAL_NAMESPACE/workflows" 2>/dev/null | jq '.executions[:10]' || echo "Unable to query workflows"
        fi
        ;;
      running)
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
          exit 1
        fi
        temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Running'"
        ;;
      failed)
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
          exit 1
        fi
        temporal workflow list --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --query "ExecutionStatus='Failed'"
        ;;
      describe)
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
          exit 1
        fi
        if [ -z "$2" ]; then
          echo "Usage: temporal-ctl describe <workflow-id>" >&2
          exit 1
        fi
        temporal workflow describe --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
        ;;
      history)
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
          exit 1
        fi
        if [ -z "$2" ]; then
          echo "Usage: temporal-ctl history <workflow-id>" >&2
          exit 1
        fi
        temporal workflow show --namespace "$TEMPORAL_NAMESPACE" --address "$TEMPORAL_ADDRESS" --workflow-id "$2"
        ;;
      terminate)
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
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
        if ! command -v temporal >/dev/null 2>&1; then
          echo "Error: temporal CLI not found" >&2
          exit 1
        fi
        temporal operator namespace list --address "$TEMPORAL_ADDRESS"
        ;;
      ui)
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
        ;;
    esac
  '')
]
