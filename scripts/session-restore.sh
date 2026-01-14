#!/usr/bin/env bash
# Restore session after WSL reload

set -euo pipefail

SESSION_DIR="$HOME/.ripple-sessions"

echo "🔄 Restoring ripple-env session..."

# Check if session directory exists
if [[ ! -d "$SESSION_DIR" ]]; then
    echo "ℹ️ No session data found at $SESSION_DIR"
    echo "   Run 'scripts/session-save.sh' to save current session"
    return 0
fi

# Find the most recent session files
LATEST_DIR=$(find "$SESSION_DIR" -name "last-directory.txt" -type f 2>/dev/null | head -1)
LATEST_ENV=$(find "$SESSION_DIR" -name "env-*.txt" -type f 2>/dev/null | sort -r | head -1)
LATEST_HISTORY=$(find "$SESSION_DIR" -name "history-*.txt" -type f 2>/dev/null | sort -r | head -1)

# Restore working directory
if [[ -f "$LATEST_DIR" ]]; then
    LAST_DIR=$(cat "$LATEST_DIR")
    if [[ -d "$LAST_DIR" ]]; then
        cd "$LAST_DIR"
        echo "📍 Restored directory: $LAST_DIR"
    else
        echo "⚠️ Last directory no longer exists: $LAST_DIR"
    fi
fi

# Load stable environment if available
if [[ -f "scripts/stable-env.sh" ]]; then
    echo "🛡️ Loading stable environment..."
    source scripts/stable-env.sh
fi

# Restore environment variables (selectively)
if [[ -f "$LATEST_ENV" ]]; then
    echo "🔄 Restoring environment variables..."
    # Only restore safe, non-path variables
    grep -E "(RIPPLE|PIX|ROS)" "$LATEST_ENV" | while IFS= read -r line; do
        # Skip PATH modifications and potentially dangerous variables
        if [[ ! "$line" =~ PATH|LD_LIBRARY_PATH|DYLD_LIBRARY_PATH ]]; then
            export "$line" 2>/dev/null || true
        fi
    done
fi

# Show git status if in git repo
if [[ -d .git ]] && command -v git >/dev/null 2>&1; then
    echo "📁 Git status:"
    git status --short --branch 2>/dev/null | head -5
fi

# Show recent history if available
if [[ -f "$LATEST_HISTORY" ]]; then
    echo "📋 Recent command history:"
    tail -5 "$LATEST_HISTORY" 2>/dev/null | sed 's/^/  /' || true
fi

echo "✅ Session recovery complete!"
echo "   Use 'direnv allow' if needed to reload environment"
echo "   Use 'scripts/session-save.sh' to save current state"