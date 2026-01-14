#!/usr/bin/env bash
# Stable environment setup for WSL
# Use this instead of regular commands to prevent reloads

echo "🛡️ Loading WSL-stable environment..."

# Set conservative resource limits (prevents WSL crashes)
export NIX_BUILD_CORES=2
export NIX_MAX_JOBS=2
export CMAKE_BUILD_PARALLEL_LEVEL=2

# Reduce memory pressure (key for WSL stability)
export MALLOC_ARENA_MAX=2
export MALLOC_MMAP_THRESHOLD_=131072
export MALLOC_TRIM_THRESHOLD_=131072
export MALLOC_TOP_PAD_=131072

# WSL-specific optimizations
export WSLENV="NIX_BUILD_CORES/w:NIX_MAX_JOBS/w"

# Stable PATH setup (prevents command not found)
export PATH="/usr/lib/wsl/lib:/usr/local/bin:/usr/bin:/bin:/home/nixos/.nix-profile/bin:$PATH"

# Make NVIDIA WSL GPU libraries discoverable (needed for CUDA + nvidia-smi)
# In WSL2 the Windows NVIDIA driver exposes user-space libraries under:
#   /usr/lib/wsl/lib
# NixOS does not run ldconfig by default, so we add it explicitly.
export LD_LIBRARY_PATH="/usr/lib/wsl/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Load AI/ML environment variables
if [[ -f "scripts/env-vars.sh" ]]; then
    source scripts/env-vars.sh
fi

# Load nix if available
if [[ -f ~/.nix-profile/etc/profile.d/nix.sh ]]; then
    source ~/.nix-profile/etc/profile.d/nix.sh
fi

# Create safe command wrappers with correct options
timeout_pixi() { 
    if [[ "$1" == "install" ]]; then
        # Use --frozen for install to prevent lockfile updates
        timeout 300 pixi install --frozen "${@:2}"
    else
        timeout 300 pixi "$@"
    fi
}
timeout_nix() { timeout 600 nix "$@"; }
timeout_docker() { timeout 120 docker "$@"; }

# Create aliases for current session
alias pixi-safe='timeout_pixi'
alias nix-safe='timeout_nix'
alias docker-safe='timeout_docker'
alias ripple-save='scripts/session-save.sh'
alias ripple-restore='scripts/session-restore.sh'
alias load-env='source scripts/env-vars.sh'

echo "✅ WSL-stable environment loaded!"
echo "   • NIX_BUILD_CORES: $NIX_BUILD_CORES (limited for WSL)"
echo "   • NIX_MAX_JOBS: $NIX_MAX_JOBS (limited for WSL)"
echo "   • Memory optimizations: ENABLED"
echo "   • Safe commands: pixi-safe, nix-safe, docker-safe"
echo "   • Session tools: ripple-save, ripple-restore"
echo "   • Environment variables: Loaded (use 'load-env' to reload)"
echo ""
echo "💡 Use 'pixi-safe install' instead of 'pixi install'"