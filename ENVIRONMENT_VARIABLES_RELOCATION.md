# Environment Variables Relocation Summary

## 📁 What Was Moved

The following environment variables were removed from `.envrc` and relocated to prevent loss during WSL reloads:

### **Original .envrc Content (Removed):**
```bash
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

# Uncomment these if you're in a ROS2 workspace:
# layout ros2
# layout colcon
```

## 📂 New Locations

### **1. Primary Location: `home.nix`**
- **Purpose**: Managed by home-manager for persistent user configuration
- **Benefits**: Survives WSL reloads, version controlled, reproducible
- **Usage**: Applied automatically after `home-manager switch`

### **2. Fallback Location: `scripts/env-vars.sh`**
- **Purpose**: Standalone script for manual loading
- **Benefits**: Can be sourced independently, works without home-manager
- **Usage**: `source scripts/env-vars.sh`

### **3. Auto-loaded: `scripts/stable-env.sh`**
- **Purpose**: Loaded automatically with WSL-stable environment
- **Benefits**: Always available when using safe commands
- **Usage**: `source scripts/stable-env.sh`

## 🔗 Files Modified

1. **`home.nix`**: Added environment variables to home-manager configuration
2. **`scripts/env-vars.sh`**: Created standalone environment variables script
3. **`scripts/stable-env.sh`**: Updated to auto-load environment variables
4. **`WSL_STABILITY_GUIDE.md`**: Added documentation about relocation

## ✅ How to Use

### **Option 1: With Home Manager (Recommended)**
```bash
# Set up home-manager
./scripts/setup-home-manager.sh

# Apply configuration (includes environment variables)
home-manager switch

# Variables are now persistent and survive WSL reloads
```

### **Option 2: Manual Loading**
```bash
# Load with stable environment (auto-loads env vars)
source scripts/stable-env.sh

# Or load environment variables separately
source scripts/env-vars.sh
```

### **Option 3: Direnv Integration**
```bash
# If you want them back in direnv, create .envrc.local
cat > .envrc.local << 'EOF'
source scripts/env-vars.sh
EOF

# Then reload direnv
direnv reload
```

## 🔄 Benefits of Relocation

1. **WSL Reload Resilience**: Environment variables survive WSL crashes/reloads
2. **Multiple Access Methods**: Can be loaded via home-manager, scripts, or direnv
3. **Better Organization**: Separated from direnv loading logic
4. **Version Controlled**: All configurations are in git
5. **Reproducible**: Same environment across different systems

## 📊 Verification

After relocation, verify environment variables are available:

```bash
# Check editor
echo "EDITOR: $EDITOR"
echo "VISUAL: $VISUAL"

# Check AI/ML paths
echo "LOCALAI_MODELS_PATH: $LOCALAI_MODELS_PATH"
echo "AIOS_DIR: $AIOS_DIR (port: $AIOS_PORT)"
echo "PROMPTCACHE_DIR: $PROMPTCACHE_DIR (port: $PROMPTCACHE_PORT)"

# Check directories exist
ls -la "$LOCALAI_MODELS_PATH" 2>/dev/null || echo "Directory will be created when needed"
```

## 💡 Pro Tips

1. **Use home-manager for persistence**: `home-manager switch` applies all configs
2. **Use stable environment for safety**: `source scripts/stable-env.sh` loads everything
3. **Create .envrc.local for custom overrides**: Add personal configurations
4. **Check WSL_STABILITY_GUIDE.md**: For complete WSL stability information

**The environment variables are now safely relocated and will survive WSL reloads!**