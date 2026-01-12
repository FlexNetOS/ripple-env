# Common Pitfalls and How to Avoid Them

This guide documents common mistakes users make and how to avoid or fix them. Learn from others' experiences!

---

## Table of Contents

- [Nix Pitfalls](#nix-pitfalls)
- [ROS2 Pitfalls](#ros2-pitfalls)
- [Docker Pitfalls](#docker-pitfalls)
- [Pixi Pitfalls](#pixi-pitfalls)
- [Development Workflow Pitfalls](#development-workflow-pitfalls)
- [WSL2 Pitfalls](#wsl2-pitfalls)

---

## Nix Pitfalls

### Pitfall 1: Running `nix develop` Outside the Repository

**Symptom:** Error about missing `flake.nix`

```
error: getting status of '/home/user/flake.nix': No such file or directory
```

**Cause:** You're not in the ripple-env directory.

**Solution:**
```bash
cd /path/to/ripple-env
nix develop
```

**Prevention:** Use an alias in your shell:
```bash
alias ripple='cd ~/ripple-env && nix develop'
```

---

### Pitfall 2: Expecting Instant First Build

**Symptom:** "nix develop is hanging" or "taking forever"

**Cause:** First run downloads all packages. This is normal and expected.

**Solution:** Wait. First run takes 5-15 minutes depending on internet speed.

**Prevention:**
- Use `nix develop -L` to see download progress
- Schedule first setup when you have time

---

### Pitfall 3: Editing flake.nix Without Rebuilding

**Symptom:** Changes to flake.nix not taking effect.

**Cause:** The shell was already loaded before your changes.

**Solution:**
```bash
exit  # Leave current shell
nix develop --rebuild  # Force rebuild
```

**Prevention:** Always exit and re-enter after flake changes.

---

### Pitfall 4: Mixing System and Nix Packages

**Symptom:** Strange version conflicts, "library not found" errors.

**Cause:** Using system-installed packages (apt, brew) alongside Nix packages.

**Solution:**
```bash
# Inside nix develop, check where binaries come from
which python
# Should be: /nix/store/.../python

# NOT: /usr/bin/python
```

**Prevention:**
- Always work inside `nix develop`
- Don't source system ROS2 install inside nix shell
- Use Pixi for Python packages, not pip with --user

---

### Pitfall 5: Forgetting Flakes Aren't Enabled

**Symptom:**
```
error: experimental Nix feature 'flakes' is disabled
```

**Cause:** Nix installed without flakes enabled (rare with Determinate installer).

**Solution:**
```bash
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
sudo systemctl restart nix-daemon  # Linux only
```

---

### Pitfall 6: Garbage Collection Breaks Things

**Symptom:** After `nix-collect-garbage`, development shell is slow again or errors.

**Cause:** GC removed cached packages that are still needed.

**Solution:**
```bash
nix develop  # Will re-download needed packages
```

**Prevention:** Use `nix-collect-garbage --delete-older-than 7d` instead of `-d` to keep recent generations.

---

## ROS2 Pitfalls

### Pitfall 7: Forgetting to Source the Workspace

**Symptom:**
```
Package 'my_package' not found
```

**Cause:** Built packages aren't in your environment.

**Solution:**
```bash
source install/setup.bash
ros2 pkg list | grep my_package
```

**Prevention:** Add to your workflow after every build:
```bash
colcon build && source install/setup.bash
```

---

### Pitfall 8: Building in Wrong Directory

**Symptom:** Build succeeds but packages don't appear.

**Cause:** You're building from inside `src/` or wrong workspace.

**Solution:**
```bash
# Always build from workspace root
cd ~/ros2_ws  # NOT ~/ros2_ws/src
colcon build --symlink-install
```

**Prevention:** Check you see `src/` directory before building:
```bash
ls src/
colcon build --symlink-install
```

---

### Pitfall 9: Not Using --symlink-install

**Symptom:** Changes to Python files require full rebuild.

**Cause:** Without symlinks, files are copied, not linked.

**Solution:**
```bash
# Clean and rebuild with symlinks
rm -rf build install log
colcon build --symlink-install
```

**Prevention:** Always use `--symlink-install` for Python packages during development.

---

### Pitfall 10: DDS Discovery Problems

**Symptom:** Nodes can't find each other, topics show empty.

**Cause:** DDS multicast blocked or domain mismatch.

**Solution:**
```bash
# Use localhost only
export ROS_LOCALHOST_ONLY=1

# Ensure same domain
export ROS_DOMAIN_ID=42  # Same on all machines/terminals
```

**Prevention:** Add to your shell startup:
```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=42
```

---

### Pitfall 11: Missing Package Dependencies

**Symptom:**
```
ImportError: No module named 'some_msgs'
```

**Cause:** Dependency not declared in package.xml.

**Solution:**
```xml
<!-- Add to package.xml -->
<depend>some_msgs</depend>
```

Then:
```bash
rosdep install --from-paths src --ignore-src -y
colcon build
```

---

## Docker Pitfalls

### Pitfall 12: Docker Daemon Not Running

**Symptom:**
```
Cannot connect to the Docker daemon
```

**Cause:** Docker service not started.

**Solution:**
```bash
sudo systemctl start docker
sudo systemctl enable docker  # Start on boot
```

---

### Pitfall 13: Missing Docker Network

**Symptom:**
```
network agentic-network not found
```

**Cause:** Docker network doesn't exist yet.

**Solution:**
```bash
docker network create agentic-network
```

**Prevention:** Create network before starting services, or add to your startup script.

---

### Pitfall 14: Port Conflicts

**Symptom:**
```
Bind for 0.0.0.0:8080 failed: port is already allocated
```

**Cause:** Another service using the same port.

**Solution:**
```bash
# Find what's using the port
sudo lsof -i :8080

# Either stop that service or change port in docker-compose.yml
```

---

### Pitfall 15: Not Using Docker Compose v2 Syntax

**Symptom:**
```
docker-compose: command not found
```

**Cause:** Old syntax; Docker Compose v2 uses `docker compose` (space, not hyphen).

**Solution:**
```bash
# v2 syntax (correct)
docker compose -f docker/docker-compose.yml up -d

# NOT: docker-compose (old v1 syntax)
```

---

### Pitfall 16: Stale Docker Volumes

**Symptom:** Old data persists despite container restart, or database won't initialize.

**Cause:** Docker volumes persist across container restarts.

**Solution:**
```bash
# Remove containers AND volumes
docker compose -f docker/docker-compose.yml down -v

# Restart fresh
docker compose -f docker/docker-compose.yml up -d
```

**Warning:** This deletes all data in volumes!

---

## Pixi Pitfalls

### Pitfall 17: Pixi Not in PATH

**Symptom:**
```
pixi: command not found
```

**Cause:** Pixi not sourced or not installed.

**Solution:**
```bash
# If inside nix develop, exit and re-enter
exit
nix develop

# Or manually source
source ~/.pixi/bin/env
```

---

### Pitfall 18: Conflicting Pixi and pip Packages

**Symptom:** Strange import errors, wrong package versions.

**Cause:** Mixed Pixi (conda) and pip packages.

**Solution:**
```bash
# Use Pixi for everything
pixi add numpy  # NOT pip install numpy

# If you must use pip, use pixi run
pixi run pip install some-package
```

**Prevention:** Prefer Pixi packages. Only use pip for packages not on conda-forge.

---

### Pitfall 19: Forgetting pixi.lock

**Symptom:** CI builds use different versions than local.

**Cause:** `pixi.lock` not committed to git.

**Solution:**
```bash
git add pixi.lock
git commit -m "chore: update pixi.lock"
```

**Prevention:** Always commit `pixi.lock` changes.

---

### Pitfall 20: Running Python Outside Pixi

**Symptom:** Python packages not found.

**Cause:** Using system Python instead of Pixi environment.

**Solution:**
```bash
# Use pixi run
pixi run python my_script.py

# Or activate the environment
eval "$(pixi shell-hook)"
python my_script.py
```

---

## Development Workflow Pitfalls

### Pitfall 21: Not Reading Files Before Editing

**Symptom:** Making incorrect assumptions about code structure.

**Cause:** Jumping to edit without understanding context.

**Solution:** Always read relevant files first:
```bash
cat flake.nix | head -100  # Understand structure
# THEN edit
```

---

### Pitfall 22: Large Commits Without Testing

**Symptom:** Build failures after merge, hard to debug.

**Cause:** Too many changes in one commit.

**Solution:**
```bash
# Test incrementally
colcon build --packages-select my_package
colcon test --packages-select my_package

# Commit when tests pass
git add -p  # Stage specific changes
git commit -m "fix: specific change"
```

---

### Pitfall 23: Ignoring Git Hooks

**Symptom:** CI fails after local commit succeeds.

**Cause:** Pre-commit hooks skipped locally.

**Solution:**
```bash
# Run hooks manually
pre-commit run --all-files

# Don't skip hooks
git commit  # NOT git commit --no-verify
```

---

### Pitfall 24: Modifying Generated Files

**Symptom:** Changes get overwritten, or build breaks.

**Cause:** Editing files in `build/`, `install/`, or `log/`.

**Solution:** Only edit files in `src/`. Regenerate with:
```bash
rm -rf build install log
colcon build --symlink-install
```

---

## WSL2 Pitfalls

### Pitfall 25: Running Bootstrap Without Admin Rights

**Symptom:** Bootstrap script fails with permission errors.

**Cause:** WSL installation requires administrator privileges.

**Solution:**
```powershell
# Run PowerShell as Administrator
Start-Process powershell -Verb RunAs
.\bootstrap.ps1
```

---

### Pitfall 26: WSL2 Disk Growing Forever

**Symptom:** Windows disk fills up with WSL2.

**Cause:** WSL2 VHD doesn't automatically shrink.

**Solution:**
```powershell
# Shutdown WSL
wsl --shutdown

# Compact the disk
Optimize-VHD -Path "$env:USERPROFILE\WSL\NixOS-Ripple\ext4.vhdx" -Mode Full
```

**Prevention:** Enable sparse VHD in `.wslconfig`:
```ini
[experimental]
sparseVhd=true
```

---

### Pitfall 27: Network Issues Between WSL and Windows

**Symptom:** Can't access WSL services from Windows browser.

**Cause:** Localhost forwarding disabled.

**Solution:**
```powershell
# Add to .wslconfig
notepad "$env:USERPROFILE\.wslconfig"
```

```ini
[wsl2]
localhostForwarding=true
```

```powershell
wsl --shutdown
wsl
```

---

### Pitfall 28: Nix Daemon Not Starting in WSL

**Symptom:** Nix commands hang or fail.

**Cause:** Systemd not starting nix-daemon.

**Solution:**
```bash
# Start manually
sudo /nix/var/nix/profiles/default/bin/nix-daemon &

# Or restart WSL
wsl --shutdown
wsl -d NixOS-Ripple
```

---

## Quick Reference: Pitfall Checklist

Before asking for help, verify:

- [ ] You're in the correct directory (`pwd` shows ripple-env)
- [ ] You're inside `nix develop` (`echo $IN_NIX_SHELL`)
- [ ] You've sourced your ROS2 workspace (`source install/setup.bash`)
- [ ] Docker daemon is running (`docker ps`)
- [ ] Network exists (`docker network ls | grep agentic`)
- [ ] Pixi environment is active (`pixi list`)
- [ ] You've tried `nix develop --rebuild` after flake changes

---

## See Also

- [Troubleshooting](TROUBLESHOOTING.md) - Detailed solutions
- [Onboarding Tutorial](ONBOARDING_TUTORIAL.md) - Step-by-step setup
- [Getting Started](GETTING_STARTED.md) - Quick reference
