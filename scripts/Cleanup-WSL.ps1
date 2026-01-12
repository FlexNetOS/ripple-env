<#
.SYNOPSIS
    WSL2 disk cleanup and management utility for Windows.

.DESCRIPTION
    This script helps manage WSL2 disk space from Windows by:
    - Reporting VHD size and usage
    - Compacting VHD to reclaim freed space
    - Running cleanup commands inside WSL
    - Resizing VHD if needed

.PARAMETER DistroName
    Name of the WSL distribution (default: NixOS-ROS2)

.PARAMETER Report
    Show disk usage report only

.PARAMETER Compact
    Compact the VHD to reclaim unused space

.PARAMETER Cleanup
    Run cleanup script inside WSL

.PARAMETER Aggressive
    Use aggressive cleanup (removes all Nix generations)

.PARAMETER Resize
    Resize VHD to specified size in GB

.PARAMETER DryRun
    Show what would be done without making changes

.EXAMPLE
    .\Cleanup-WSL.ps1 -Report
    Shows disk usage report

.EXAMPLE
    .\Cleanup-WSL.ps1 -Cleanup
    Runs cleanup inside WSL

.EXAMPLE
    .\Cleanup-WSL.ps1 -Compact
    Compacts VHD to reclaim space (requires Admin)

.EXAMPLE
    .\Cleanup-WSL.ps1 -Resize 256
    Resizes VHD to 256GB (requires Admin)

.NOTES
    Some operations require Administrator privileges.
#>

[CmdletBinding()]
param(
    [string]$DistroName = "NixOS-ROS2",
    [switch]$Report,
    [switch]$Compact,
    [switch]$Cleanup,
    [switch]$Aggressive,
    [int]$Resize = 0,
    [switch]$DryRun
)

# Colors for output
function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Type = "Info"
    )

    switch ($Type) {
        "Info"    { Write-Host "[INFO] " -ForegroundColor Blue -NoNewline; Write-Host $Message }
        "Success" { Write-Host "[SUCCESS] " -ForegroundColor Green -NoNewline; Write-Host $Message }
        "Warning" { Write-Host "[WARNING] " -ForegroundColor Yellow -NoNewline; Write-Host $Message }
        "Error"   { Write-Host "[ERROR] " -ForegroundColor Red -NoNewline; Write-Host $Message }
        "Header"  { Write-Host "`n=== " -ForegroundColor Cyan -NoNewline; Write-Host $Message -ForegroundColor Cyan -NoNewline; Write-Host " ===" -ForegroundColor Cyan }
    }
}

function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

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

function Show-DiskReport {
    Write-ColorOutput "WSL2 Disk Usage Report" "Header"
    Write-Host ""

    # Check if distro exists
    $distros = wsl --list --quiet 2>&1
    if ($distros -notcontains $DistroName) {
        Write-ColorOutput "Distribution '$DistroName' not found." "Warning"
        Write-Host "Available distributions:"
        wsl --list --verbose
        return
    }

    # Find VHD path
    $vhdPath = Get-WSLDistroPath -Name $DistroName
    if (-not $vhdPath) {
        $vhdPath = "$env:USERPROFILE\WSL\$DistroName\ext4.vhdx"
    }

    Write-Host "Distribution: $DistroName"
    Write-Host ""

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
    else {
        Write-ColorOutput "VHD not found at expected location: $vhdPath" "Warning"
    }

    Write-Host ""

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

    Write-Host ""

    # Show .wslconfig if exists
    $wslConfigPath = "$env:USERPROFILE\.wslconfig"
    if (Test-Path $wslConfigPath) {
        Write-Host "WSL Configuration ($wslConfigPath):" -ForegroundColor Cyan
        Get-Content $wslConfigPath | ForEach-Object { Write-Host "  $_" }
    }
}

function Invoke-WSLCleanup {
    Write-ColorOutput "Running cleanup inside WSL" "Header"

    # Check if cleanup script exists
    $scriptExists = wsl -d $DistroName -- test -f ~/ros2-humble-env/scripts/wsl-cleanup.sh 2>&1
    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Cleanup script not found. Running manual cleanup..." "Warning"

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
        if ($DryRun) {
            Write-ColorOutput "[DRY-RUN] Would run cleanup commands" "Info"
            Write-Host $cleanupCommands
        }
        else {
            wsl -d $DistroName -- bash -c $cleanupCommands
        }
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
        wsl -d $DistroName -- bash -c "cd ~/ros2-humble-env && ./scripts/wsl-cleanup.sh $scriptArgs"
    }

    if ($LASTEXITCODE -eq 0) {
        Write-ColorOutput "Cleanup completed successfully" "Success"
    }
    else {
        Write-ColorOutput "Cleanup encountered errors" "Warning"
    }
}

function Invoke-VHDCompact {
    Write-ColorOutput "Compacting VHD" "Header"

    if (-not (Test-Administrator)) {
        Write-ColorOutput "VHD compaction requires Administrator privileges." "Error"
        Write-Host "Please run PowerShell as Administrator and try again."
        return
    }

    $vhdPath = Get-WSLDistroPath -Name $DistroName
    if (-not $vhdPath) {
        $vhdPath = "$env:USERPROFILE\WSL\$DistroName\ext4.vhdx"
    }

    if (-not (Test-Path $vhdPath)) {
        Write-ColorOutput "VHD not found: $vhdPath" "Error"
        return
    }

    $beforeSize = (Get-Item $vhdPath).Length

    if ($DryRun) {
        Write-ColorOutput "[DRY-RUN] Would compact VHD: $vhdPath" "Info"
        Write-Host "Current size: $([math]::Round($beforeSize / 1GB, 2)) GB"
        return
    }

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
    else {
        Write-Host "No additional space recovered (VHD already optimized)"
    }
}

function Invoke-VHDResize {
    param([int]$SizeGB)

    Write-ColorOutput "Resizing VHD to ${SizeGB}GB" "Header"

    if (-not (Test-Administrator)) {
        Write-ColorOutput "VHD resize requires Administrator privileges." "Error"
        Write-Host "Please run PowerShell as Administrator and try again."
        return
    }

    $vhdPath = Get-WSLDistroPath -Name $DistroName
    if (-not $vhdPath) {
        $vhdPath = "$env:USERPROFILE\WSL\$DistroName\ext4.vhdx"
    }

    if (-not (Test-Path $vhdPath)) {
        Write-ColorOutput "VHD not found: $vhdPath" "Error"
        return
    }

    $sizeMB = $SizeGB * 1024
    $sizeBytes = [int64]$SizeGB * 1024 * 1024 * 1024

    if ($DryRun) {
        Write-ColorOutput "[DRY-RUN] Would resize VHD to ${SizeGB}GB" "Info"
        return
    }

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

# Main execution
Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "    WSL2 Disk Management Utility        " -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""

if ($DryRun) {
    Write-ColorOutput "DRY-RUN MODE - No changes will be made" "Warning"
    Write-Host ""
}

# Default to report if no action specified
if (-not $Report -and -not $Compact -and -not $Cleanup -and $Resize -eq 0) {
    $Report = $true
}

if ($Report) {
    Show-DiskReport
}

if ($Cleanup) {
    Invoke-WSLCleanup
}

if ($Compact) {
    Invoke-VHDCompact
}

if ($Resize -gt 0) {
    Invoke-VHDResize -SizeGB $Resize
}

Write-Host ""
