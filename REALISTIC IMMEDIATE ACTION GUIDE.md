# 🎯 REALISTIC IMMEDIATE ACTION GUIDE

## ✅ WHAT'S ACTUALLY WORKING RIGHT NOW

### Core Infrastructure
- **Stable Environment**: `scripts/stable-env.sh` - Memory limits, timeout protection
- **Session Management**: `scripts/session-save.sh` & `scripts/session-restore.sh` 
- **Home Manager**: Configured and working (environment variables persist)
- **Pixi**: Available with multiple environments (docs, vectordb-chromadb, vectordb-ruvector, qudag)

### Safe Commands (with timeout protection)
- `pixi-safe install --skip vectordb-ruvector` (5min timeout)
- `nix-safe develop` (10min timeout) 
- `docker-safe system prune -f` (2min timeout)

## 🚨 IMMEDIATE ISSUES TO KNOW

1. **Pixi Lockfile**: Out of date - needs updating before install works
2. **WSL Memory**: Still experiencing timeout issues on heavy operations
3. **Package Conflicts**: Had to remove git/gh from nix profile for home-manager

## 📋 STEP-BY-STEP IMMEDIATE ACTIONS

### Every Time You Start Work
```bash
# 1. Load stable environment (ESSENTIAL - do this first!)
source scripts/stable-env.sh

# 2. Save session before any heavy operations
scripts/session-save.sh

# 3. Test basic commands (these work)
pixi info                    # Shows environments
home-manager --version       # Shows 25.11-pre
nix --version                # Available
```

### After WSL Reload
```bash
# Restore your session
scripts/session-restore.sh
```

### Use Safe Commands Only
```bash
# These work with timeout protection:
pixi-safe install --skip vectordb-ruvector    # 5min timeout
nix-safe develop                             # 10min timeout
docker-safe system prune -f                  # 2min timeout
```

## ⚠️ WHAT TO AVOID (causes hangs/timeout)
```bash
pixi install                    # Times out on dependency resolution
nix develop                     # No timeout protection
docker operations               # No timeout protection
Multiple heavy ops at once      # Memory pressure
```

## 🔧 NEXT STEPS TO COMPLETE

### 1. Fix Pixi Lockfile (when you have time)
```bash
# This will take time - use timeout protection
source scripts/stable-env.sh
timeout 600 pixi install --skip vectordb-ruvector
```

### 2. Set Up WSL Memory Limits (Windows side)
```powershell
# Copy .wslconfig to Windows
# Then: wsl --shutdown
```

### 3. Use Home Manager Environment
```bash
home-manager switch    # Already working!
# Environment variables now persist through reloads
```

## 🎯 WORKING WORKFLOW

```bash
# Every time you start work:
source scripts/stable-env.sh
scripts/session-save.sh

# Do your work with safe commands:
pixi-safe install --skip vectordb-ruvector
nix-safe develop

# Before heavy operations:
scripts/session-save.sh

# After WSL reload:
scripts/session-restore.sh
```

## 📝 WINDOWS REFERENCE COMMANDS

### WSL Management (PowerShell as Admin)
```powershell
# Check WSL status
wsl --status

# Shutdown WSL (when needed)
wsl --shutdown

# Restart specific distro
wsl --terminate Ubuntu

# Check memory usage
wsl --system
```

### File Access from Windows
```powershell
# Access WSL files from Windows Explorer
\\wsl$\Ubuntu\home\nixos\ripple-env

# Copy files between Windows and WSL
copy C:\path\to\file \\wsl$\Ubuntu\home\nixos\ripple-env\
```

## 🚨 EMERGENCY RECOVERY

### If WSL Hangs Completely
1. **Windows**: `wsl --shutdown` (PowerShell as Admin)
2. **Wait 30 seconds**
3. **Restart WSL**: `wsl`
4. **Restore session**: `scripts/session-restore.sh`

### If Commands Keep Timing Out
1. **Check memory**: `free -h`
2. **Clear caches**: `sudo sync && echo 3 | sudo tee /proc/sys/vm/drop_caches`
3. **Use skip options**: `pixi install --skip vectordb-ruvector`
4. **Reduce parallel jobs**: `export NIX_BUILD_CORES=1`

## 📁 KEY FILES TO KNOW

- `scripts/stable-env.sh` - Load this first every time
- `scripts/session-save.sh` - Save before heavy operations
- `scripts/session-restore.sh` - Restore after WSL reload
- `home.nix` - Home manager config (environment variables)
- `pixi.toml` - Project dependencies
- `.wslconfig` - WSL memory limits (Windows side)

## ✅ VERIFICATION CHECKLIST

After each WSL reload:
- [ ] `source scripts/stable-env.sh` loads without errors
- [ ] `pixi info` shows environments
- [ ] `home-manager --version` shows 25.11-pre
- [ ] `scripts/session-restore.sh` restores working directory
- [ ] Environment variables are set (check `echo $EDITOR`)

---

**The core infrastructure is working! The main remaining issue is the pixi lockfile update, which needs to be done carefully with timeout protection.**