#!/usr/bin/env bash
# Session Start Hook - Initializes workspace for Claude Code
# This runs automatically when a new Claude Code session starts

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  ROS2 Humble Environment - Session Initialization${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"

# Check if we're in the right directory
if [[ ! -f "flake.nix" ]]; then
    echo -e "${YELLOW}Warning: Not in ripple-env root directory${NC}"
fi

# Check Nix availability
if command -v nix &> /dev/null; then
    echo -e "${GREEN}✓ Nix available:${NC} $(nix --version 2>/dev/null | head -1)"
else
    echo -e "${RED}✗ Nix not found - run bootstrap.sh first${NC}"
fi

# Check direnv
if command -v direnv &> /dev/null; then
    echo -e "${GREEN}✓ direnv available${NC}"
else
    echo -e "${YELLOW}○ direnv not found${NC}"
fi

# Check pixi
if command -v pixi &> /dev/null; then
    echo -e "${GREEN}✓ pixi available${NC}"
else
    echo -e "${YELLOW}○ pixi not installed yet${NC}"
fi

# Git status summary
if [[ -d ".git" ]]; then
    BRANCH=$(git branch --show-current 2>/dev/null || echo "unknown")
    CHANGES=$(git status --porcelain 2>/dev/null | wc -l | tr -d ' ')
    echo -e "${GREEN}✓ Git branch:${NC} ${BRANCH} (${CHANGES} uncommitted changes)"

    # Show recent commits
    echo -e "\n${BLUE}Recent commits:${NC}"
    git log --oneline -3 2>/dev/null || echo "  No commits yet"
fi

# Check for lock file freshness
if [[ -f "flake.lock" ]]; then
    LOCK_AGE=$(( ($(date +%s) - $(stat -c %Y flake.lock 2>/dev/null || stat -f %m flake.lock 2>/dev/null)) / 86400 ))
    if [[ $LOCK_AGE -gt 30 ]]; then
        echo -e "${YELLOW}⚠ flake.lock is ${LOCK_AGE} days old - consider 'nix flake update'${NC}"
    fi
fi

if [[ -f "pixi.lock" ]]; then
    PIXI_AGE=$(( ($(date +%s) - $(stat -c %Y pixi.lock 2>/dev/null || stat -f %m pixi.lock 2>/dev/null)) / 86400 ))
    if [[ $PIXI_AGE -gt 30 ]]; then
        echo -e "${YELLOW}⚠ pixi.lock is ${PIXI_AGE} days old - consider 'pixi update'${NC}"
    fi
fi

echo -e "\n${BLUE}Available commands:${NC}"
echo "  /build        - Build ROS2 packages"
echo "  /test         - Run tests"
echo "  /flake-check  - Validate Nix flake"
echo "  /update-deps  - Update dependencies"
echo ""
echo -e "${BLUE}Enter dev shell:${NC} nom develop"
echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
