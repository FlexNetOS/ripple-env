# bootstrap.ps1 — Script Contract

**Type:** PowerShell Script
**Platform:** Windows
**Layer:** L1 (Bootstrap)
**Criticality:** HIGH
**Safety:** caution
**Idempotent:** Yes
**Requires Admin:** Yes
**Owner:** ripple-env maintainers

---

## Purpose

Complete end-to-end installation script for Windows that sets up:
- WSL2 with Virtual Machine Platform
- NixOS-WSL distribution
- Virtual disk configuration (1TB default)
- Full ripple-env development environment inside WSL

---

## Invocation

```powershell
# Must run as Administrator
# Standard bootstrap
.\bootstrap.ps1

# Custom distro name
.\bootstrap.ps1 -DistroName "MyRipple"

# Custom disk size (GB)
.\bootstrap.ps1 -DiskSizeGB 512

# Custom memory and swap
.\bootstrap.ps1 -MemorySizeGB 16 -SwapSizeGB 8

# Resume from previous failure
.\bootstrap.ps1 -Resume

# Force reinstall (skip checks)
.\bootstrap.ps1 -Force

# Clean state and restart
.\bootstrap.ps1 -Clean

# Skip shell installations
.\bootstrap.ps1 -SkipShells
```

---

## Inputs

### Command-Line Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `-DistroName` | String | `NixOS-Ripple` | WSL distribution name |
| `-InstallPath` | Path | `$env:USERPROFILE\WSL\<DistroName>` | Installation directory |
| `-DiskSizeGB` | Int | `1000` | Virtual disk size (GB) |
| `-MemorySizeGB` | Int | `8` | WSL memory limit (GB) |
| `-SwapSizeGB` | Int | `4` | WSL swap size (GB) |
| `-RepoURL` | String | GitHub URL | Repository to clone |
| `-Resume` | Switch | false | Resume from previous state |
| `-Clean` | Switch | false | Clear state and restart |
| `-Force` | Switch | false | Force reinstall |
| `-SkipShells` | Switch | false | Skip optional shells |

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `USERPROFILE` | Required | User profile directory |
| `TEMP` | Required | Temporary files |

### Configuration Files Read

| File | Purpose |
|------|---------|
| `$env:USERPROFILE\.wslconfig` | WSL global configuration |
| `$env:LOCALAPPDATA\ripple-env\bootstrap-state.json` | State persistence |

---

## Outputs

### Files Created/Modified

| File | Action | Purpose |
|------|--------|---------|
| `$env:USERPROFILE\.wslconfig` | Created/Modified | WSL memory and swap settings |
| `<InstallPath>\ext4.vhdx` | Created | WSL virtual disk |
| `<InstallPath>\nixos-wsl.tar.gz` | Downloaded | NixOS-WSL tarball |
| `$env:LOCALAPPDATA\ripple-env\bootstrap-state.json` | Created | State persistence |
| `$env:LOCALAPPDATA\ripple-env\bootstrap.log` | Created | Bootstrap log |

### WSL Distributions

| Distribution | Purpose |
|--------------|---------|
| `NixOS-Ripple` (or custom) | Main development environment |

### Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | General error |
| 2 | Not running as Administrator |
| 3 | Windows version too old |
| 4 | WSL installation failed |
| 5 | NixOS download failed |
| 6 | Disk resize failed |

---

## Side Effects

1. **Enables Windows Features** — WSL2, Virtual Machine Platform
2. **Installs WSL Distribution** — Imports NixOS-WSL tarball
3. **Modifies .wslconfig** — Sets memory and swap limits
4. **Resizes Virtual Disk** — Uses diskpart (CAUTION)
5. **Downloads files** — NixOS-WSL tarball (~500MB)
6. **May require reboot** — After enabling Windows features

---

## Safety Classification

**Rating:** CAUTION

**Justification:**
- Requires Administrator privileges
- Modifies Windows system features
- Uses diskpart for disk operations
- May require reboot
- Virtual disk resize is destructive if interrupted

**Mitigations:**
- State persistence for safe resume
- Confirmation prompts for destructive operations
- Backup recommendations in documentation

---

## Idempotency

**Fully Idempotent:** Yes (with caveats)

Each stage checks current state before acting:
- Windows features: Checks if already enabled
- WSL distro: Checks if already exists
- Disk resize: Only if current size < target

**Caveat:** Disk resize on existing data requires caution.

```powershell
# Safe to run multiple times
.\bootstrap.ps1
.\bootstrap.ps1  # Second run skips completed stages
```

---

## Dependencies

### System Requirements

| Requirement | Minimum | Check |
|-------------|---------|-------|
| Windows Build | 19041+ | `[System.Environment]::OSVersion.Version.Build` |
| PowerShell | 5.1+ | `$PSVersionTable.PSVersion` |
| Administrator | Required | `[Security.Principal.WindowsPrincipal]` |
| Virtualization | Enabled in BIOS | WSL install will fail otherwise |
| Disk Space | 50GB+ | For WSL VHD |

### External Services

| Service | Purpose |
|---------|---------|
| https://github.com/nix-community/NixOS-WSL/releases | NixOS-WSL download |
| https://github.com/FlexNetOS/ripple-env | Repository clone |

---

## Failure Modes

### FM-001: Virtualization Not Enabled

**Symptom:** WSL installation fails
**Cause:** Hardware virtualization disabled in BIOS
**Recovery:**
- Reboot into BIOS
- Enable Intel VT-x or AMD-V
- Re-run bootstrap

### FM-002: Insufficient Disk Space

**Symptom:** VHD creation fails
**Cause:** Not enough space for virtual disk
**Recovery:**
- Free disk space
- Use smaller `-DiskSizeGB` value
- Resume with `-Resume`

### FM-003: Reboot Required

**Symptom:** Script prompts for reboot
**Cause:** Windows features need reboot to activate
**Recovery:**
- Reboot Windows
- Re-run with `-Resume`

### FM-004: Disk Resize Interrupted

**Symptom:** VHD in inconsistent state
**Cause:** Power failure or termination during resize
**Recovery:**
- May need to delete VHD and re-import
- Use `-Clean` flag

### FM-005: Network Timeout

**Symptom:** Download fails
**Cause:** Network connectivity issues
**Recovery:**
- Automatic retry (4 attempts)
- Resume with `-Resume`

---

## Stage Breakdown

| Stage | Function | Lines | Idempotent | Duration |
|-------|----------|-------|------------|----------|
| 1 | Test-WindowsVersion | 260-275 | Yes | <1s |
| 2 | Install-WSL | 311-345 | Yes | 1-5min + reboot |
| 3 | Set-WSLConfig | 462-488 | Yes | <1s |
| 4 | New-NixOSDistro | 354-417 | Yes | 5-15min |
| 5 | Set-VirtualDiskSize | 419-460 | Caution | 1-10min |
| 6 | Initialize-NixOSEnvironment | 490-532 | Yes | 1-2min |
| 7 | Install-ROS2Environment | 534-619 | Yes | 10-30min |
| 8 | Set-DefaultDistro | 621-631 | Yes | <1s |

---

## Examples

### Example 1: Fresh Install

```powershell
# Open PowerShell as Administrator
# Run bootstrap
.\bootstrap.ps1

# Expected output:
# [1/8] Checking Windows version...
# [2/8] Installing WSL2...
# [3/8] Configuring WSL...
# ...
# [8/8] Setting default distro...
# ✅ Bootstrap complete!
#
# Enter development environment:
#   wsl -d NixOS-Ripple
```

### Example 2: Custom Configuration

```powershell
.\bootstrap.ps1 `
    -DistroName "DevEnv" `
    -DiskSizeGB 256 `
    -MemorySizeGB 16 `
    -SwapSizeGB 8

# Creates smaller, more memory-optimized environment
```

### Example 3: Resume After Reboot

```powershell
# After reboot prompted by stage 2
.\bootstrap.ps1 -Resume

# Resumes from stage 3 (skips completed stages)
```

---

## Integration

### Called By

| Caller | Context |
|--------|---------|
| User | Manual execution from PowerShell Admin |

### Calls

| Script/Command | Purpose |
|----------------|---------|
| `wsl --install` | WSL installation |
| `wsl --import` | NixOS distribution import |
| `diskpart` | Virtual disk resize |
| `bootstrap.sh` | Linux-side bootstrap (via WSL) |

---

## .wslconfig Template

The script creates/modifies `.wslconfig`:

```ini
[wsl2]
memory=8GB
swap=4GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

---

## Post-Installation

After bootstrap completes:

```powershell
# Enter WSL
wsl -d NixOS-Ripple

# Inside WSL:
cd ~/ripple-env
nix develop
ros2 --help
```

---

## References

**Evidence Files:**
- `bootstrap.ps1` (806 lines)
- `README.md` lines 96-119
- `docs/GETTING_STARTED.md` lines 52-90
- `.github/workflows/wsl2-build.yml`
- `WSL_STABILITY_GUIDE.md`

**Related Documentation:**
- [Golden Path: Bootstrap](../modules/bootstrap.md)
- [WSL Stability Guide](../wsl/WSL_STABILITY_GUIDE.md)
- [Troubleshooting](../TROUBLESHOOTING.md)

---

**Last Updated:** 2026-01-14
**Contract Version:** 1.0.0
