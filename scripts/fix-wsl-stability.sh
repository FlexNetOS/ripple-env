#!/usr/bin/env bash

# WSL Stability Fix Script for Ripple Environment
# This script addresses common WSL stability issues

set -euo pipefail

echo "🔧 WSL Stability Fix Script"
echo "=========================="

# Function to check if running in WSL
check_wsl() {
    if [[ -f /proc/version ]] && grep -qi microsoft /proc/version; then
        return 0
    else
        return 1
    fi
}

# Function to fix WSL-specific issues
fix_wsl_issues() {
    echo "🐧 Detected WSL environment - applying fixes..."
    
    # Fix 1: Reduce memory pressure
    echo "📊 Setting memory limits..."
    if [[ -f /etc/wsl.conf ]]; then
        echo "[wsl2]" | sudo tee -a /etc/wsl.conf
        echo "memory=8GB" | sudo tee -a /etc/wsl.conf
        echo "processors=4" | sudo tee -a /etc/wsl.conf
        echo "swap=2GB" | sudo tee -a /etc/wsl.conf
    fi
    
    # Fix 2: Optimize Docker for WSL
    echo "🐳 Configuring Docker for WSL..."
    if command -v docker &> /dev/null; then
        # Reduce Docker memory usage
        docker system prune -f --volumes || true
        
        # Configure Docker daemon for WSL
        cat > /tmp/daemon.json << 'EOF'
{
  "default-ulimits": {
    "nofile": {
      "Soft": 65536,
      "Hard": 65536
    }
  },
  "max-concurrent-downloads": 3,
  "max-concurrent-uploads": 2,
  "storage-driver": "overlay2"
}
EOF
        
        if [[ -d /etc/docker ]]; then
            sudo cp /tmp/daemon.json /etc/docker/daemon.json || true
        fi
    fi
    
    # Fix 3: Configure Nix for WSL
    echo "❄️ Configuring Nix for WSL..."
    if [[ -d /nix ]]; then
        # Reduce Nix memory usage
        export NIX_BUILD_CORES=2
        export NIX_MAX_JOBS=2
        
        # Add to shell profile
        echo 'export NIX_BUILD_CORES=2' >> ~/.bashrc
        echo 'export NIX_MAX_JOBS=2' >> ~/.bashrc
    fi
    
    # Fix 4: Optimize pixi for WSL
    echo "📦 Configuring pixi for WSL..."
    if command -v pixi &> /dev/null; then
        # Create pixi config for WSL
        mkdir -p ~/.config/pixi
        cat > ~/.config/pixi/config.toml << 'EOF'
[default_channels]
channels = ["conda-forge", "robostack-humble"]

[solve_strategy]
max_attempts = 3
timeout = 300

[repodata]
cache_dir = "~/.cache/pixi"
fetch_threads = 2
EOF
    fi
    
    # Fix 5: Create stable environment script
    echo "🛡️ Creating stable environment script..."
    cat > scripts/stable-env.sh << 'EOF'
#!/usr/bin/env bash
# Stable environment setup for WSL

# Set conservative resource limits
export NIX_BUILD_CORES=2
export NIX_MAX_JOBS=2
export CMAKE_BUILD_PARALLEL_LEVEL=2

# Reduce memory pressure
export MALLOC_ARENA_MAX=2
export MALLOC_MMAP_THRESHOLD_=131072
export MALLOC_TRIM_THRESHOLD_=131072
export MALLOC_TOP_PAD_=131072

# WSL-specific optimizations
export WSLENV="NIX_BUILD_CORES/w:NIX_MAX_JOBS/w"

# Stable PATH setup
export PATH="/usr/local/bin:/usr/bin:/bin:$PATH"

# Load nix if available
if [[ -f ~/.nix-profile/etc/profile.d/nix.sh ]]; then
    source ~/.nix-profile/etc/profile.d/nix.sh
fi

# Load direnv if available
if command -v direnv &> /dev/null; then
    eval "$(direnv hook bash)"
fi

echo "✅ Stable environment loaded"
echo "   NIX_BUILD_CORES: $NIX_BUILD_CORES"
echo "   NIX_MAX_JOBS: $NIX_MAX_JOBS"
echo "   PATH length: ${#PATH}"
EOF
    
    chmod +x scripts/stable-env.sh
    
    # Fix 6: Create session persistence helper
    echo "💾 Creating session persistence helper..."
    cat > scripts/session-save.sh << 'EOF'
#!/usr/bin/env bash
# Save current session state for recovery

SESSION_DIR="$HOME/.ripple-sessions"
mkdir -p "$SESSION_DIR"

# Save environment
env | grep -E "(NIX|PIX|ROS|DOCKER)" > "$SESSION_DIR/env-$(date +%Y%m%d-%H%M%S).txt"

# Save working directory
echo "$PWD" > "$SESSION_DIR/last-directory.txt"

# Save command history
history > "$SESSION_DIR/history-$(date +%Y%m%d-%H%M%S).txt"

echo "💾 Session saved to $SESSION_DIR"
EOF
    
    chmod +x scripts/session-save.sh
    
    # Fix 7: Create recovery script
    echo "🔄 Creating recovery script..."
    cat > scripts/session-restore.sh << 'EOF'
#!/usr/bin/env bash
# Restore session after WSL reload

SESSION_DIR="$HOME/.ripple-sessions"

if [[ -d "$SESSION_DIR" ]]; then
    # Restore last directory
    if [[ -f "$SESSION_DIR/last-directory.txt" ]]; then
        LAST_DIR=$(cat "$SESSION_DIR/last-directory.txt")
        if [[ -d "$LAST_DIR" ]]; then
            cd "$LAST_DIR"
            echo "📍 Restored directory: $LAST_DIR"
        fi
    fi
    
    # Load stable environment
    if [[ -f scripts/stable-env.sh ]]; then
        source scripts/stable-env.sh
    fi
    
    echo "🔄 Session recovery complete"
else
    echo "ℹ️ No session data found"
fi
EOF
    
    chmod +x scripts/session-restore.sh
}

# Function to create robust aliases
create_robust_aliases() {
    echo "🔨 Creating robust aliases..."
    
    cat >> ~/.bashrc << 'EOF'

# WSL-stable aliases for ripple-env
alias ripple-enter='source scripts/stable-env.sh && echo "✅ Entered stable ripple environment"'
alias ripple-save='scripts/session-save.sh'
alias ripple-restore='scripts/session-restore.sh'
alias pixi-safe='timeout 300 pixi'
alias nix-safe='timeout 600 nix'
alias docker-safe='timeout 120 docker'

# Safe directory navigation
alias cd-ripple='cd /home/nixos/ripple-env && ripple-restore'

# Memory-aware commands
alias pixi-install='pixi-safe install --no-update-lockfile'
alias nix-build='nix-safe build --max-jobs 2 --cores 2'
alias nix-develop='nix-safe develop --max-jobs 2 --cores 2'
EOF
}

# Function to create WSL configuration
create_wsl_config() {
    echo "⚙️ Creating WSL configuration..."
    
    cat > /tmp/.wslconfig << 'EOF'
[wsl2]
memory=8GB
processors=4
swap=2GB
localhostForwarding=true
nestedVirtualization=false

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
EOF
    
    echo "ℹ️ To apply WSL config, copy .wslconfig to your Windows user directory:"
    echo "   cp /tmp/.wslconfig /mnt/c/Users/\$USER/.wslconfig"
    echo "   Then restart WSL: wsl --shutdown"
}

# Main execution
main() {
    if check_wsl; then
        fix_wsl_issues
        create_robust_aliases
        create_wsl_config
        
        echo ""
        echo "✅ WSL stability fixes applied!"
        echo ""
        echo "🎯 Next steps:"
        echo "   1. Copy .wslconfig to Windows: cp /tmp/.wslconfig /mnt/c/Users/\$USER/.wslconfig"
        echo "   2. Restart WSL: wsl --shutdown"
        echo "   3. Use 'ripple-enter' to enter stable environment"
        echo "   4. Use 'pixi-safe' and 'nix-safe' for timeout-protected commands"
        echo "   5. Use 'ripple-save' before long operations"
        echo "   6. Use 'ripple-restore' after WSL reloads"
        echo ""
        echo "💡 Pro tips:"
        echo "   - Always run 'ripple-save' before heavy operations"
        echo "   - Use 'cd-ripple' to return to project with recovery"
        echo "   - Commands have 5-10 minute timeouts to prevent hangs"
        echo "   - Memory limits prevent WSL crashes"
        
    else
        echo "ℹ️ Not running in WSL - skipping WSL-specific fixes"
    fi
}

# Run main function
main "$@"