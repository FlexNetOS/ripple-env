#!/usr/bin/env bash
# Save current session state for recovery after WSL reloads

set -euo pipefail

SESSION_DIR="$HOME/.ripple-sessions"
TIMESTAMP=$(date +%Y%m%d-%H%M%S)

echo "💾 Saving ripple-env session state..."

# Create session directory
mkdir -p "$SESSION_DIR"

# Save current working directory
echo "$PWD" > "$SESSION_DIR/last-directory.txt"

# Save environment variables related to ripple
env | grep -E "(RIPPLE|PIX|ROS|NIX)" > "$SESSION_DIR/env-$TIMESTAMP.txt" 2>/dev/null || true

# Save command history
history > "$SESSION_DIR/history-$TIMESTAMP.txt" 2>/dev/null || true

# Save git status if in git repo
if [[ -d .git ]]; then
    git status --porcelain > "$SESSION_DIR/git-status-$TIMESTAMP.txt" 2>/dev/null || true
    git branch --show-current > "$SESSION_DIR/git-branch-$TIMESTAMP.txt" 2>/dev/null || true
fi

# Save pixi environment info if available
if command -v pixi >/dev/null 2>&1; then
    pixi info > "$SESSION_DIR/pixi-info-$TIMESTAMP.txt" 2>/dev/null || true
fi

# Save nix info if available
if command -v nix >/dev/null 2>&1; then
    nix --version > "$SESSION_DIR/nix-version-$TIMESTAMP.txt" 2>/dev/null || true
fi

echo "✅ Session saved to $SESSION_DIR"
echo "   Timestamp: $TIMESTAMP"
echo "   Directory: $PWD"
echo "   Files saved:"
ls -la "$SESSION_DIR"/*-$TIMESTAMP.txt 2>/dev/null | sed 's/.*\///' | sed 's/^/     - /' || echo "     - No additional files"