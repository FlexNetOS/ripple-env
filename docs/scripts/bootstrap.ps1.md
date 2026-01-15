# Script Contract: bootstrap.ps1

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `bootstrap.ps1`

---

## Purpose

Complete end-to-end installation for ripple-env on Windows with WSL2 and NixOS. Sets up a ROS2 Humble development environment by enabling WSL2, creating a NixOS distribution, configuring virtual disk and memory, and running the Linux bootstrap inside WSL.

This is the primary entry point for setting up ripple-env on Windows systems.

---

## Invocation

```powershell
.\bootstrap.ps1 [PARAMETERS]
```

**Requirements:**
- Windows 10 version 2004+ or Windows 11
- Must be run as Administrator
- Requires `#Requires -RunAsAdministrator` (line 1)

**Examples:**
```powershell
# Standard installation
.\bootstrap.ps1

# Custom disk size and memory
.\bootstrap.ps1 -DiskSizeGB 512 -MemorySizeGB 16

# Resume after failure
.\bootstrap.ps1 -Resume

# Clean start with custom log
.\bootstrap.ps1 -Clean -LogFile "C:\Logs\bootstrap.log"

# Clone specific branch or PR
.\bootstrap.ps1 -RepoFetchRef "refs/pull/123/merge"

# Skip shell customization
.\bootstrap.ps1 -SkipShells
```

---

## Inputs

### Parameters
| Parameter | Type | Required | Default | Validation | Description |
|-----------|------|----------|---------|------------|-------------|
| `DistroName` | string | No | `NixOS-Ripple` | - | Name for the WSL distribution |
| `InstallPath` | string | No | `$env:USERPROFILE\WSL\NixOS-Ripple` | - | Installation path for the distro |
| `DiskSizeGB` | int | No | `1024` | 64-4096 | Size of ext4.vhdx in GB |
| `MemorySizeGB` | int | No | `8` | 2-256 | Memory limit for WSL2 in GB |
| `SwapSizeGB` | int | No | `8` | 1-64 | Size of swap in GB |
| `RepoURL` | string | No | `https://github.com/FlexNetOS/ripple-env.git` | Must match `^https?://` | Git URL to clone |
| `RepoFetchRef` | string | No | `""` | - | Git ref/refspec to fetch (e.g., `refs/pull/123/merge`) |
| `SkipShells` | switch | No | `false` | - | Skip shell customization (passes `--skip-shells`) |
| `SkipWSLCheck` | switch | No | `false` | - | Skip WSL installation check |
| `Force` | switch | No | `false` | - | Force replacement of existing distro |
| `Resume` | switch | No | `false` | - | Resume from last saved state |
| `Clean` | switch | No | `false` | - | Clean state and start fresh |
| `LogFile` | string | No | `$env:TEMP\bootstrap-<timestamp>.log` | - | Path to log file |

**Evidence:** Parameter definitions lines 72-90

### Environment Variables
None directly used, but sets environment variables inside WSL:
- `REPO_URL` - Repository URL (line 580)
- `REPO_FETCH_REF` - Git ref to fetch (line 580)
- `REPO_DIR` - Repository directory name (line 580)
- `SKIP_SHELLS` - Whether to skip shells (line 608)

### Configuration Files
| File | Required | Purpose |
|------|----------|---------|
| `$env:USERPROFILE\.wslconfig` | No | Created by script, configures WSL2 memory/swap |

---

## Outputs

### Files Created
| File | Purpose |
|------|---------|
| `$env:LOCALAPPDATA\ripple-env\bootstrap.state.json` | Installation state for resume functionality |
| `$env:USERPROFILE\.wslconfig` | WSL2 configuration (memory, swap, settings) |
| `$InstallPath\ext4.vhdx` | Virtual disk for WSL distribution |
| `$env:TEMP\bootstrap-<timestamp>.log` | Installation log |

**Evidence:**
- State file: lines 101-102
- .wslconfig: lines 469-487
- Log file: lines 120-122

### Standard Output
- Color-coded log messages ([INFO], [SUCCESS], [WARNING], [ERROR], [STEP])
- Installation progress for each stage
- Final summary with usage instructions
- Evidence: lines 125-145

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all stages completed or restart required |
| `1` | Failure - see log for details |

**Failure Points:**
- Not running as Administrator (lines 718-722)
- Unsupported Windows version (lines 726-728)
- NixOS distro creation failure (lines 758-762)
- NixOS initialization failure (lines 773-777)
- ROS2 environment installation failure (lines 780-784)

---

## Side Effects

### System Modifications
1. **Enables Windows Features**
   - Microsoft-Windows-Subsystem-Linux (lines 316-323)
   - VirtualMachinePlatform (lines 326-334)
   - **May require system restart** (lines 669-683)

2. **Installs WSL2**
   - Runs `wsl --install --no-distribution` (line 338)
   - Sets WSL2 as default version (line 342)
   - Evidence: lines 311-345

3. **Creates WSL Distribution**
   - Downloads NixOS-WSL tarball (~300MB+) from GitHub (lines 385-387)
   - Imports as WSL distribution (line 403)
   - Evidence: lines 354-417

4. **Configures Virtual Disk**
   - Resizes ext4.vhdx to specified size (default 1TB)
   - Uses diskpart with `expand vdisk maximum` command
   - **Shuts down WSL** (line 441)
   - Evidence: lines 419-460

5. **Writes .wslconfig**
   - Sets memory limit (default 8GB)
   - Sets swap size (default 8GB)
   - Enables localhost forwarding
   - Enables experimental features (autoMemoryReclaim, sparseVhd)
   - Evidence: lines 462-488

6. **Modifies WSL Distribution**
   - Updates Nix channels (line 513)
   - Enables flakes in `~/.config/nix/nix.conf` (lines 516-517)
   - Clones ripple-env repository (lines 542-585)
   - Runs `bootstrap.sh --ci` inside WSL (lines 587-618)
   - Evidence: lines 490-619

7. **Sets Default Distribution**
   - Runs `wsl --set-default` (line 627)
   - Evidence: lines 621-631

### Network Activity
- Downloads NixOS-WSL from `https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz`
- Clones repository from specified RepoURL
- Downloads packages via Nix inside WSL (implicit in bootstrap.sh)
- Evidence: lines 117, 385-387, 542-585

### State Management
- Creates state directory: `%LOCALAPPDATA%\ripple-env\`
- Writes JSON state file after each successful stage
- State file tracks: `LastCompletedStage`, `StartedAt`, `LastUpdated`, `BootstrapVersion`
- Evidence: lines 189-252

---

## Safety Classification

**🟡 CAUTION**

**Rationale:**
- **Requires Administrator privileges** (enforced at line 1)
- Enables Windows features that require system restart
- Downloads and executes external installer (~300MB)
- Modifies system-wide WSL configuration
- Creates 1TB virtual disk by default
- Shuts down all WSL distributions (line 441)

**Safe for:**
- Fresh Windows development machine setup
- Systems with sufficient disk space (1TB+ free)
- Developer workstations with restart capability

**Unsafe for:**
- Production Windows servers
- Systems with critical WSL workloads (shuts down all distros)
- Systems with limited disk space
- Environments where restart is not acceptable

---

## Idempotency

**✅ IDEMPOTENT**

The script is designed to be safely re-run:

1. **Feature enablement checks** before enabling (lines 316-323, 326-334)
   ```powershell
   if ($wslFeature.State -ne "Enabled") {
       Enable-WindowsOptionalFeature ...
   }
   else {
       Write-ColorOutput "WSL feature already enabled" "Info"
   }
   ```

2. **Distribution existence checks** (lines 347-352, 362-372)
   ```powershell
   if (Test-DistroExists -Name $DistroName) {
       if ($Force) {
           wsl --unregister $DistroName
       }
       else {
           Write-ColorOutput "Distro already exists. Use -Force to replace."
           return $true
       }
   }
   ```

3. **Repository clone checks** (lines 553-561)
   ```powershell
   if [ -d "$repo_dir/.git" ]; then
       echo "Repository already exists, updating..."
       git remote set-url origin "$repo_url"
   else
       git clone "$repo_url" "$repo_dir"
   fi
   ```

4. **State persistence** enables resume functionality (lines 189-252)
   - Each stage saved after completion
   - `-Resume` flag skips completed stages
   - `-Clean` flag clears state for fresh start

**Evidence:** All stage functions check `Test-ShouldSkipStage` before execution.

---

## Dependencies

### Required Windows Features
| Feature | Checked By | Evidence |
|---------|------------|----------|
| Windows build 19041+ | Test-WindowsVersion | lines 260-275 |
| Microsoft-Windows-Subsystem-Linux | Install-WSL | lines 316-323 |
| VirtualMachinePlatform | Install-WSL | lines 326-334 |

### Required PowerShell Cmdlets
| Cmdlet | Purpose |
|--------|---------|
| `Get-WindowsOptionalFeature` | Check feature status |
| `Enable-WindowsOptionalFeature` | Enable Windows features |
| `Invoke-WebRequest` | Download NixOS-WSL tarball |
| `Test-Path` | Check file/directory existence |
| `New-Item` | Create directories |
| `Set-Content` | Write configuration files |
| `ConvertTo-Json` / `ConvertFrom-Json` | State file serialization |

### Required External Commands
| Command | Purpose | Stage |
|---------|---------|-------|
| `wsl` | WSL management | All stages |
| `diskpart` | Virtual disk resizing | ConfigureVirtualDisk |
| `git` | Repository cloning | Inside WSL |
| `bash` | Execute scripts in WSL | Inside WSL |

### External Services
| Service | Purpose | Evidence |
|---------|---------|----------|
| `https://github.com/nix-community/NixOS-WSL/releases` | NixOS-WSL tarball | line 117 |
| Git repository (RepoURL) | Clone ripple-env | lines 542-585 |
| Nix binary cache | Package downloads inside WSL | Implicit |

### Required Files in Repository (Inside WSL)
| File | Stage | Evidence |
|------|-------|----------|
| `bootstrap.sh` | InstallROS2Environment | lines 594-602 |

---

## Failure Modes

### Administrator Privilege Missing
**Symptom:** Script fails to start
**Exit Code:** 1
**Recovery:** Run PowerShell as Administrator
**Evidence:** lines 718-722

**Example:**
```
[ERROR] This script must be run as Administrator
[INFO] Right-click PowerShell and select 'Run as Administrator'
```

### Unsupported Windows Version
**Symptom:** Windows build < 19041
**Exit Code:** 1
**Recovery:** Update Windows to version 2004+
**Evidence:** lines 267-271

**Example:**
```
[ERROR] Windows build 18363 detected. WSL2 requires build 19041 or higher.
[ERROR] Please update Windows to version 2004 or later.
```

### Network Download Failure
**Symptom:** Cannot download NixOS-WSL tarball
**Exit Code:** 1 (after retries)
**Recovery:** Automatic retry with exponential backoff (4 attempts)
**Evidence:** lines 385-393

**Example:**
```
[WARNING] Attempt 1 failed: ... Retrying in 2s...
[WARNING] Attempt 2 failed: ... Retrying in 4s...
[ERROR] Failed to download NixOS-WSL after multiple attempts
```

### Empty Download
**Symptom:** Downloaded file is 0 bytes
**Exit Code:** 1
**Recovery:** Manual intervention or re-run
**Evidence:** lines 396-399

**Example:**
```
[ERROR] Downloaded file is missing or empty
```

### WSL Import Failure
**Symptom:** `wsl --import` fails
**Exit Code:** 1
**Recovery:** Check disk space, WSL status
**Evidence:** lines 405-408

**Example:**
```
[ERROR] Failed to import NixOS distribution
```

### Virtual Disk Not Found
**Symptom:** ext4.vhdx doesn't exist at expected location
**Exit Code:** Success (non-fatal warning)
**Recovery:** Manual disk expansion inside WSL
**Evidence:** lines 428-432

**Example:**
```
[WARNING] Virtual disk not found at expected location
```

### NixOS Initialization Failure
**Symptom:** Cannot start NixOS or run initialization commands
**Exit Code:** 1
**Recovery:** Check WSL status, re-run with `-Resume`
**Evidence:** lines 501-527

**Example:**
```
[ERROR] Failed to start NixOS
[INFO] To resume, run: .\bootstrap.ps1 -Resume
```

### Bootstrap Script Failure
**Symptom:** bootstrap.sh fails inside WSL
**Exit Code:** 1
**Recovery:** Resume with `-Resume` flag
**Evidence:** lines 610-614

**Example:**
```
[ERROR] Bootstrap script encountered errors
[INFO] You can resume the installation by running: .\bootstrap.ps1 -Resume
```

### Restart Required
**Symptom:** Windows features enabled, restart needed
**Exit Code:** 0
**Recovery:** Restart and re-run script
**Evidence:** lines 744-749, 669-683

**Example:**
```
========================================
  RESTART REQUIRED
========================================
Windows features have been enabled that require a restart.
Would you like to restart now? (y/N)
```

---

## Retry and Backoff Strategy

### Network Operations
**Function:** `Invoke-WithRetry` (lines 151-183)

**Parameters:**
- `ScriptBlock`: PowerShell code to retry
- `Description`: Human-readable operation name
- `MaxAttempts`: Maximum retry attempts (default 4)
- `InitialDelaySeconds`: Starting delay (default 2)

**Backoff Strategy:**
- Initial delay: 2 seconds
- Exponential backoff: delay × 2 after each failure
- Maximum delay: 60 seconds
- Default attempts: 4

**Evidence:**
```powershell
# lines 178-179
$delay = [Math]::Min($delay * 2, 60)
```

**Applied to:**
- NixOS-WSL download (lines 385-387)

**Retry configuration:** lines 96-98
```powershell
$script:NetworkRetryAttempts = 4
$script:NetworkRetryDelaySeconds = 2
```

---

## Stage Execution Order

The script executes 8 stages in order:

| # | Stage | Function | Evidence |
|---|-------|----------|----------|
| 1 | CheckWindowsVersion | Verify Windows build >= 19041 | lines 725-730 |
| 2 | InstallWSL | Enable WSL features, install WSL2 | lines 732-752 |
| 3 | ConfigureWSL | Write .wslconfig | lines 754-755 |
| 4 | CreateNixOSDistro | Download and import NixOS-WSL | lines 757-762 |
| 5 | ConfigureVirtualDisk | Resize ext4.vhdx | lines 764-765 |
| 6 | InitializeNixOS | Update channels, enable flakes | lines 772-777 |
| 7 | InstallROS2Environment | Clone repo, run bootstrap.sh | lines 779-784 |
| 8 | SetDefaultDistro | Set as default WSL distro | lines 786-787 |

**Stage Definition:** lines 105-114

**Resume Logic:**
- State saved after each stage (lines 207-214)
- Stages skipped if already completed in resume mode (lines 231-252)
- State cleared on successful completion (line 790)

---

## Logging System

### Log Levels
| Level | Function | Color | Purpose |
|-------|----------|-------|---------|
| Info | `Write-ColorOutput` | Blue | General information |
| Success | `Write-ColorOutput` | Green | Successful operations |
| Warning | `Write-ColorOutput` | Yellow | Non-fatal warnings |
| Error | `Write-ColorOutput` | Red | Fatal errors |
| Step | `Write-ColorOutput` | Cyan | Stage headers |

**Evidence:** lines 125-145

### Log Format
**Console:** `[LEVEL] message`
**File:** `[timestamp] [LEVEL] message`

**Example:**
```
[2026-01-13 14:25:30] [INFO] Downloading NixOS-WSL...
```

**Evidence:** lines 131-144

---

## WSL Configuration

The script creates `.wslconfig` with the following settings:

```ini
[wsl2]
memory=8GB
swap=8GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
```

**Evidence:** lines 471-480

**Purpose:**
- `memory`: Limit WSL2 memory usage (default 8GB)
- `swap`: Set swap size (default 8GB)
- `localhostForwarding`: Enable localhost port forwarding
- `autoMemoryReclaim`: Gradual memory reclamation (experimental)
- `sparseVhd`: Automatic VHDX disk space reclamation (experimental)

---

## Virtual Disk Resize

**Function:** `Set-VirtualDiskSize` (lines 419-460)

**Process:**
1. Locate `ext4.vhdx` in installation path (line 426)
2. Shutdown WSL (line 441)
3. Use diskpart to resize (lines 446-450)
4. Convert GB to MB for diskpart (line 436)

**Diskpart Script:**
```diskpart
select vdisk file="<path>"
expand vdisk maximum=<size_in_MB>
```

**Evidence:** lines 446-449

**Note:** Requires manual filesystem expansion inside WSL after resize (warning at line 455)

---

## Git Repository Handling

**Function:** `Install-ROS2Environment` (lines 534-619)

### Repository Clone Strategy
1. **Check for existing repository** (lines 553-561)
   - If `.git` directory exists, update remote URL
   - Otherwise, clone fresh

2. **Optional ref fetching** (lines 563-575)
   - If `RepoFetchRef` provided, fetch that specific ref
   - Checkout fetched ref in detached HEAD state
   - Otherwise, fetch default refs and fast-forward if on branch

3. **Repository directory** (line 580)
   - Default: `~/ripple-env`

**Evidence:**
```bash
# lines 553-575
if [ -d "$repo_dir/.git" ]; then
    cd "$repo_dir"
    git remote set-url origin "$repo_url"
else
    git clone "$repo_url" "$repo_dir"
fi

if [ -n "$repo_fetch_ref" ]; then
    git fetch --prune origin "$repo_fetch_ref"
    git checkout --detach FETCH_HEAD
else
    git fetch --prune origin
    if git symbolic-ref -q HEAD >/dev/null; then
        git pull --ff-only || true
    fi
fi
```

### Use Cases
**Standard installation:**
```powershell
.\bootstrap.ps1  # Uses main branch
```

**Specific branch:**
```powershell
.\bootstrap.ps1 -RepoFetchRef "refs/heads/develop"
```

**Pull request:**
```powershell
.\bootstrap.ps1 -RepoFetchRef "refs/pull/123/merge"
```

**Tag:**
```powershell
.\bootstrap.ps1 -RepoFetchRef "refs/tags/v1.2.3"
```

---

## Bootstrap Script Integration

**Function:** `Install-ROS2Environment` (lines 587-618)

The script runs `bootstrap.sh` inside WSL with:
- CI mode (`--ci`) for non-interactive installation
- Optional `--skip-shells` flag (if `-SkipShells` parameter used)

**Evidence:**
```bash
# lines 590-604
cd ~/ripple-env
chmod +x bootstrap.sh

if [ "${SKIP_SHELLS:-}" = "1" ]; then
    ./bootstrap.sh --ci --skip-shells
else
    ./bootstrap.sh --ci
fi
```

**Environment variables passed:**
- `SKIP_SHELLS`: Set to "1" if `-SkipShells` parameter used (line 607-608)

---

## Summary Display

**Function:** `Show-Summary` (lines 633-667)

Displays:
- Distribution name and location
- Disk, memory, and swap sizes
- Commands to enter the environment
- Quick commands inside WSL (cb, ct, ros2)
- Shell selection examples
- Log and state file locations

**Example Output:**
```
========================================
  ROS2 Humble Environment Ready!
========================================

Distribution: NixOS-Ripple
Location: C:\Users\user\WSL\NixOS-Ripple
Disk Size: 1024GB
WSL Memory: 8GB
Swap Size: 8GB

To enter the environment:
  wsl -d NixOS-Ripple
  cd ~/ripple-env
  direnv allow  # or: nom develop

Quick commands inside WSL:
  cb   - colcon build --symlink-install
  ct   - colcon test
  ros2 - ROS2 CLI
```

**Evidence:** lines 636-666

---

## Platform-Specific Behavior

### Windows 10 vs Windows 11
- Both supported (build 19041+)
- Evidence: lines 66-67, 267

### Architecture Support
Determined by NixOS-WSL tarball:
- x86_64 (Intel/AMD 64-bit)
- aarch64 (ARM64) if available

### Locale Support
- Checks for both English and Japanese WSL status messages (line 299)
  ```powershell
  if ($wslStatus -match "Default Version: 2" -or $wslStatus -match "既定のバージョン: 2")
  ```

---

## Error Handling

### Try-Catch Block
**Evidence:** lines 798-805

Catches all uncaught exceptions and displays:
- Error message
- Script stack trace
- Exit code 1

**Example:**
```powershell
try {
    Main
}
catch {
    Write-ColorOutput "An error occurred: $_" "Error"
    Write-ColorOutput $_.ScriptStackTrace "Error"
    exit 1
}
```

---

## References

### Source Code
- **Main script:** `bootstrap.ps1` (806 lines)
- **Retry logic:** lines 151-183
- **State management:** lines 189-252
- **WSL installation:** lines 311-345
- **NixOS distro creation:** lines 354-417
- **Virtual disk resize:** lines 419-460
- **WSL configuration:** lines 462-488
- **NixOS initialization:** lines 490-532
- **ROS2 environment setup:** lines 534-619

### Related Files
- **Linux equivalent:** `bootstrap.sh` (13 stages)
- **WSL configuration:** `$env:USERPROFILE\.wslconfig`
- **State file:** `%LOCALAPPDATA%\ripple-env\bootstrap.state.json`

### Related Documentation
- [docs/modules/bootstrap.md](../modules/bootstrap.md) - Golden Path GP-1
- [docs/GETTING_STARTED.md](../GETTING_STARTED.md) - User guide
- [docs/TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - Common issues

### External Resources
- [NixOS-WSL Project](https://github.com/nix-community/NixOS-WSL)
- [WSL Documentation](https://docs.microsoft.com/en-us/windows/wsl/)
- [PowerShell Script Requirements](https://docs.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_requires)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 2/60 contracts complete
