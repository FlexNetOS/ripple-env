#!/usr/bin/env bash
# Environment variables for ripple-env AI/ML tools
# This file contains the environment variables that were previously in .envrc

# Default editor (Helix)
export EDITOR="${EDITOR:-hx}"
export VISUAL="${VISUAL:-hx}"

# LocalAI models directory
export LOCALAI_MODELS_PATH="${LOCALAI_MODELS_PATH:-$HOME/.local/share/localai/models}"

# AIOS Agent OS
export AIOS_DIR="${AIOS_DIR:-$HOME/.local/share/aios}"
export AIOS_PORT="${AIOS_PORT:-8000}"

# PromptCache
export PROMPTCACHE_DIR="${PROMPTCACHE_DIR:-$HOME/.local/share/prompt-cache}"
export PROMPTCACHE_PORT="${PROMPTCACHE_PORT:-8080}"

# Create directories if they don't exist
mkdir -p "$LOCALAI_MODELS_PATH" 2>/dev/null || true
mkdir -p "$AIOS_DIR" 2>/dev/null || true
mkdir -p "$PROMPTCACHE_DIR" 2>/dev/null || true

echo "🎯 AI/ML environment variables loaded:"
echo "   EDITOR: $EDITOR"
echo "   LOCALAI_MODELS_PATH: $LOCALAI_MODELS_PATH"
echo "   AIOS_DIR: $AIOS_DIR (port: $AIOS_PORT)"
echo "   PROMPTCACHE_DIR: $PROMPTCACHE_DIR (port: $PROMPTCACHE_PORT)"

# ROS2 workspace configuration (uncomment if needed)
# layout ros2
# layout colcon