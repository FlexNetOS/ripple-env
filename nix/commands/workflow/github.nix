# GitHub CLI workflow commands
# Extracted from workflow.nix for modular organization
{ pkgs, ... }:

[
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
        STATE="''${2:-open}"
        LIMIT="''${3:-20}"
        if [[ ! "$STATE" =~ ^(open|closed|all)$ ]]; then
          echo "Error: STATE must be one of: open, closed, all" >&2
          exit 1
        fi
        if ! [[ "$LIMIT" =~ ^[0-9]+$ ]] || [ "$LIMIT" -lt 1 ]; then
          echo "Error: LIMIT must be a positive integer" >&2
          exit 1
        fi
        echo "Issues ($STATE):"
        gh issue list --state "$STATE" --limit "$LIMIT"
        ;;
      search)
        if [ -z "$2" ]; then
          echo "Usage: gh-issues search <query> [state]" >&2
          exit 1
        fi
        STATE="''${3:-all}"
        echo "Searching issues for: $2"
        gh issue list --search "$2" --state "$STATE"
        ;;
      labels)
        if [ -z "$2" ]; then
          echo "Available labels:"
          gh label list
        else
          echo "Issues with label '$2':"
          gh issue list --label "$2"
        fi
        ;;
      assigned)
        USER="''${2:-@me}"
        echo "Issues assigned to $USER:"
        gh issue list --assignee "$USER"
        ;;
      create)
        shift
        gh issue create "$@"
        ;;
      view)
        if [ -z "$2" ]; then
          echo "Usage: gh-issues view <issue-number>" >&2
          exit 1
        fi
        gh issue view "$2"
        ;;
      close)
        if [ -z "$2" ]; then
          echo "Usage: gh-issues close <issue-number> [reason]" >&2
          exit 1
        fi
        REASON="''${3:-completed}"
        gh issue close "$2" --reason "$REASON"
        ;;
      reopen)
        if [ -z "$2" ]; then
          echo "Usage: gh-issues reopen <issue-number>" >&2
          exit 1
        fi
        gh issue reopen "$2"
        ;;
      comment)
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
        echo "Issue Statistics"
        echo "================"
        echo ""
        echo "Open issues:   $(gh issue list --state open --json number --jq 'length')"
        echo "Closed issues: $(gh issue list --state closed --limit 1000 --json number --jq 'length') (max 1000)"
        echo ""
        echo "By label:"
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
]
