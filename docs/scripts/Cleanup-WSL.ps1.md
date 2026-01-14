# Script Contract: Cleanup-WSL.ps1

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/Cleanup-WSL.ps1`

---

## Purpose

WSL2 disk management utility for Windows. Reports VHD size/usage, compacts VHD to reclaim freed space, runs cleanup commands inside WSL, and resizes VHD capacity. Provides both Hyper-V cmdlet and diskpart fallback methods for VHD operations.

---

## Invocation

```powershell
.\scripts\Cleanup-WSL.ps1 [OPTIONS]
```

**Parameters:**
- `-DistroName` - WSL distribution name (default: "NixOS-Ripple")
- `-Report` - Show disk usage report only
- `-Compact` - Compact VHD to reclaim unused space (requires Admin)
- `-Cleanup` - Run cleanup script inside WSL
- `-Aggressive` - Use aggressive cleanup (removes all Nix generations)
- `-Resize <GB>` - Resize VHD to specified size in GB (requires Admin)
- `-DryRun` - Show what would be done without making changes

**Examples:**
```powershell
.\Cleanup-WSL.ps1 -Report                           # Show usage
.\Cleanup-WSL.ps1 -Cleanup                          # Clean inside WSL
.\Cleanup-WSL.ps1 -Cleanup -Aggressive              # Aggressive clean
.\Cleanup-WSL.ps1 -Compact                          # Compact VHD (Admin)
.\Cleanup-WSL.ps1 -Resize 256                       # Resize to 256GB (Admin)
.\Cleanup-WSL.ps1 -Cleanup -Compact -DryRun         # Preview
```

---

## Side Effects

### VHD Compaction (lines 243-307)
- **Requires Admin:** Must run as Administrator
- **Shuts down WSL:** Calls `wsl --shutdown` to safely access VHD
- **Modifies VHD file:** Reclaims freed space using `Optimize-VHD` or diskpart

### VHD Resize (lines 309-366)
- **Requires Admin:** Must run as Administrator
- **Expands VHD:** Increases maximum VHD size (does NOT auto-resize filesystem)
- **Manual step required:** User must run `resize2fs` inside WSL after resize

### WSL Cleanup (lines 194-241)
- Runs `scripts/wsl-cleanup.sh` inside WSL (if present)
- Falls back to manual cleanup commands (Nix GC, Pixi, Docker)
- **Aggressive mode:** Removes all Nix generations with `nix-collect-garbage -d`

---

## Safety Classification

**🔴 DESTRUCTIVE** (compact, resize, aggressive cleanup) - Modifies VHD, deletes Nix generations.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT** - Report/cleanup are safe to repeat, compact/resize have permanent effects.

---

## Key Features

### VHD Path Detection (lines 86-122)

```powershell
function Get-WSLDistroPath {
    param([string]$Name)

    # Common locations for WSL distros
    $possiblePaths = @(
        "$env:USERPROFILE\WSL\$Name",
        "$env:LOCALAPPDATA\Packages\*$Name*\LocalState",
        "$env:LOCALAPPDATA\WSL\$Name"
    )

    foreach ($path in $possiblePaths) {
        $resolved = Resolve-Path $path -ErrorAction SilentlyContinue
        if ($resolved) {
            $vhdx = Get-ChildItem -Path $resolved -Filter "ext4.vhdx" -ErrorAction SilentlyContinue
            if ($vhdx) {
                return $vhdx.FullName
            }
        }
    }

    # Try to find by searching common WSL directories
    $searchPaths = @(
        "$env:USERPROFILE\WSL",
        "$env:LOCALAPPDATA\Packages"
    )

    foreach ($searchPath in $searchPaths) {
        if (Test-Path $searchPath) {
            $vhdx = Get-ChildItem -Path $searchPath -Filter "ext4.vhdx" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($vhdx) {
                return $vhdx.FullName
            }
        }
    }

    return $null
}
```

**Search strategy:**
1. Common known locations (WSL, Packages, LocalAppData)
2. Recursive search if not found in known locations
3. Returns first ext4.vhdx found

### Disk Usage Report (lines 124-192)

```powershell
function Show-DiskReport {
    # VHD information
    if (Test-Path $vhdPath) {
        $vhdInfo = Get-Item $vhdPath
        $vhdSizeGB = [math]::Round($vhdInfo.Length / 1GB, 2)

        Write-Host "VHD File Information:" -ForegroundColor Cyan
        Write-Host "  Path: $vhdPath"
        Write-Host "  Actual Size on Disk: $vhdSizeGB GB"

        # Try to get VHD max size (requires Hyper-V module)
        try {
            $vhdDetails = Get-VHD -Path $vhdPath -ErrorAction Stop
            $maxSizeGB = [math]::Round($vhdDetails.Size / 1GB, 2)
            Write-Host "  Maximum Size: $maxSizeGB GB"
            Write-Host "  VHD Type: $($vhdDetails.VhdFormat) ($($vhdDetails.VhdType))"
        }
        catch {
            Write-Host "  (Install Hyper-V PowerShell module for detailed VHD info)"
        }
    }

    # Get usage from inside WSL
    Write-Host "Usage Inside WSL:" -ForegroundColor Cyan
    try {
        $dfOutput = wsl -d $DistroName -- df -h / 2>&1
        if ($LASTEXITCODE -eq 0) {
            $dfOutput | ForEach-Object { Write-Host "  $_" }
        }
    }
    catch {
        Write-ColorOutput "Could not query WSL filesystem" "Warning"
    }
}
```

**Reports:**
- VHD path and actual size on disk
- VHD maximum size (if Hyper-V module available)
- Filesystem usage inside WSL (via `df -h`)
- .wslconfig settings (if present)

### VHD Compaction (lines 243-307)

```powershell
function Invoke-VHDCompact {
    Write-ColorOutput "Shutting down WSL..." "Info"
    wsl --shutdown
    Start-Sleep -Seconds 3

    Write-ColorOutput "Compacting VHD (this may take a few minutes)..." "Info"

    try {
        # Method 1: Using Hyper-V cmdlet
        Optimize-VHD -Path $vhdPath -Mode Full -ErrorAction Stop
        Write-ColorOutput "VHD compacted using Hyper-V" "Success"
    }
    catch {
        # Method 2: Using diskpart
        Write-ColorOutput "Trying diskpart method..." "Info"
        $diskpartScript = @"
select vdisk file="$vhdPath"
attach vdisk readonly
compact vdisk
detach vdisk
"@
        $diskpartScript | diskpart | Out-Null
        Write-ColorOutput "VHD compacted using diskpart" "Success"
    }

    $afterSize = (Get-Item $vhdPath).Length
    $freedMB = [math]::Round(($beforeSize - $afterSize) / 1MB, 0)

    Write-Host ""
    Write-Host "Before: $([math]::Round($beforeSize / 1GB, 2)) GB"
    Write-Host "After:  $([math]::Round($afterSize / 1GB, 2)) GB"

    if ($freedMB -gt 0) {
        Write-ColorOutput "Freed $freedMB MB" "Success"
    }
}
```

**Two methods:**
1. **Hyper-V (preferred):** `Optimize-VHD -Mode Full`
2. **Diskpart (fallback):** `compact vdisk` command

**Before/after tracking:**
- Reports size before and after compaction
- Calculates freed space in MB

### VHD Resize (lines 309-366)

```powershell
function Invoke-VHDResize {
    param([int]$SizeGB)

    $sizeMB = $SizeGB * 1024
    $sizeBytes = [int64]$SizeGB * 1024 * 1024 * 1024

    Write-ColorOutput "Shutting down WSL..." "Info"
    wsl --shutdown
    Start-Sleep -Seconds 3

    Write-ColorOutput "Resizing VHD..." "Info"

    try {
        # Method 1: Using Hyper-V cmdlet
        Resize-VHD -Path $vhdPath -SizeBytes $sizeBytes -ErrorAction Stop
        Write-ColorOutput "VHD resized using Hyper-V" "Success"
    }
    catch {
        # Method 2: Using diskpart
        Write-ColorOutput "Trying diskpart method..." "Info"
        $diskpartScript = @"
select vdisk file="$vhdPath"
expand vdisk maximum=$sizeMB
"@
        $diskpartScript | diskpart | Out-Null
        Write-ColorOutput "VHD resized using diskpart" "Success"
    }

    Write-Host ""
    Write-ColorOutput "VHD resized to ${SizeGB}GB" "Success"
    Write-Host ""
    Write-Host "IMPORTANT: You must also resize the filesystem inside WSL:"
    Write-Host "  wsl -d $DistroName"
    Write-Host "  sudo resize2fs /dev/sdc"
}
```

**Important:** VHD resize does NOT automatically resize the ext4 filesystem inside. User must:
1. Launch WSL: `wsl -d NixOS-Ripple`
2. Resize filesystem: `sudo resize2fs /dev/sdc`

### WSL Cleanup Integration (lines 194-241)

```powershell
function Invoke-WSLCleanup {
    # Check if cleanup script exists
    $scriptExists = wsl -d $DistroName -- test -f ~/ripple-env/scripts/wsl-cleanup.sh 2>&1
    if ($LASTEXITCODE -ne 0) {
        # Fallback to manual cleanup
        $cleanupCommands = @"
echo "Running Nix garbage collection..."
nix-collect-garbage -d 2>/dev/null || nix store gc 2>/dev/null || true
echo "Optimizing Nix store..."
nix store optimise 2>/dev/null || true
echo "Cleaning Pixi cache..."
pixi clean 2>/dev/null || true
echo "Cleaning Docker..."
docker system prune -a -f 2>/dev/null || true
echo "Done!"
df -h /
"@
        wsl -d $DistroName -- bash -c $cleanupCommands
    }
    else {
        $scriptArgs = "--all"
        if ($Aggressive) {
            $scriptArgs += " --aggressive"
        }
        if ($DryRun) {
            $scriptArgs += " --dry-run"
        }

        Write-ColorOutput "Running wsl-cleanup.sh $scriptArgs" "Info"
        wsl -d $DistroName -- bash -c "cd ~/ripple-env && ./scripts/wsl-cleanup.sh $scriptArgs"
    }
}
```

**Cleanup strategy:**
1. **Preferred:** Run `wsl-cleanup.sh` if present (with aggressive/dry-run flags)
2. **Fallback:** Execute manual cleanup commands (Nix, Pixi, Docker)

---

## Typical Workflow

```powershell
# 1. Check current usage
.\Cleanup-WSL.ps1 -Report

# 2. Clean up inside WSL
.\Cleanup-WSL.ps1 -Cleanup -Aggressive

# 3. Compact VHD to reclaim space (requires Admin)
.\Cleanup-WSL.ps1 -Compact

# 4. Check results
.\Cleanup-WSL.ps1 -Report
```

---

## References

- **Main script:** `scripts/Cleanup-WSL.ps1` (402 lines)
- **VHD path detection:** lines 86-122
- **Disk report:** lines 124-192
- **WSL cleanup:** lines 194-241
- **VHD compaction:** lines 243-307
- **VHD resize:** lines 309-366
- **Related:** `scripts/wsl-cleanup.sh` (Linux-side cleanup script)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 57/57 (100%)
