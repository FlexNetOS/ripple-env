# Workflow status dashboard commands
# Extracted from workflow.nix for modular organization
{ pkgs, ... }:

[
  # Shared helper function for opening URLs
  (pkgs.writeShellScriptBin "open-url-helper" ''
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
        ;;
    esac
  '')
]
