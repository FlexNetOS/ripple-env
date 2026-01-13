#Requires -RunAsAdministrator
<#
.SYNOPSIS
    Bootstrap script for ripple-env on Windows with WSL2 and NixOS.

.DESCRIPTION
    This script sets up a complete ROS2 Humble development environment on Windows by:
    1. Checking and enabling WSL2 if needed
    2. Creating a NixOS WSL distribution
    3. Setting up ext4.vhdx virtual disk (1TB) and swap
    4. Installing Nix with flakes, pixi, and all development tools
    5. Configuring shells (bash, zsh, nushell)

    Features:
    - Retry logic with exponential backoff for network operations
    - State persistence for resumable installations
    - Detailed logging to file
    - Idempotent installation

.PARAMETER DistroName
    Name for the WSL distribution (default: NixOS-Ripple)

.PARAMETER InstallPath
    Installation path for the distro (default: $env:USERPROFILE\WSL\NixOS-Ripple)

.PARAMETER DiskSizeGB
    Size of the ext4.vhdx disk in GB (default: 1024 = 1TB)

.PARAMETER MemorySizeGB
    Memory limit for WSL2 in GB written to .wslconfig (default: 8)

.PARAMETER SwapSizeGB
    Size of swap in GB (default: 8)

.PARAMETER RepoURL
    Git URL to clone inside WSL (default: https://github.com/FlexNetOS/ripple-env.git)

.PARAMETER RepoFetchRef
    Optional git ref/refspec to fetch inside WSL before running bootstrap.
    Examples: refs/heads/main, refs/tags/v1.2.3, refs/pull/123/merge

.PARAMETER SkipShells
    Skip shell customization steps in bootstrap.sh (passes --skip-shells)

.PARAMETER SkipWSLCheck
    Skip WSL installation check (for already configured systems)

.PARAMETER Resume
    Resume from last saved state (if available)

.PARAMETER Clean
    Clean state and start fresh

.PARAMETER LogFile
    Path to log file (default: auto-generated in temp directory)

.EXAMPLE
    .\bootstrap.ps1

.EXAMPLE
    .\bootstrap.ps1 -DistroName "MyROS2" -DiskSizeGB 512

.EXAMPLE
    .\bootstrap.ps1 -Resume

.NOTES
    Requires Windows 10 version 2004+ or Windows 11
    Must be run as Administrator
#>

[CmdletBinding()]
param(
    [string]$DistroName = "NixOS-Ripple",
    [string]$InstallPath = "$env:USERPROFILE\WSL\NixOS-Ripple",
    [ValidateRange(64, 4096)]
    [int]$DiskSizeGB = 1024,
    [ValidateRange(2, 256)]
    [int]$MemorySizeGB = 8,
    [ValidateRange(1, 64)]
    [int]$SwapSizeGB = 8,
    [ValidatePattern('^https?://')]
    [string]$RepoURL = "https://github.com/FlexNetOS/ripple-env.git",
    [string]$RepoFetchRef = "",
    [switch]$SkipShells,
    [switch]$SkipWSLCheck,
    [switch]$Force,
    [switch]$Resume,
    [switch]$Clean,
    [string]$LogFile = ""
)

# Configuration
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

# Retry configuration
$script:NetworkRetryAttempts = 4
$script:NetworkRetryDelaySeconds = 2

# State management
$script:StateDir = Join-Path $env:LOCALAPPDATA "ripple-env"
$script:StateFile = Join-Path $script:StateDir "bootstrap.state.json"

# Stages in order
$script:Stages = @(
    "CheckWindowsVersion",
    "InstallWSL",
    "ConfigureWSL",
    "CreateNixOSDistro",
    "ConfigureVirtualDisk",
    "InitializeNixOS",
    "InstallROS2Environment",
    "SetDefaultDistro"
)

# URLs and versions
$NixOSWSLRelease = "https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz"

# Initialize log file
if ([string]::IsNullOrEmpty($LogFile)) {
    $LogFile = Join-Path $env:TEMP "bootstrap-$(Get-Date -Format 'yyyyMMdd-HHmmss').log"
}

# Colors for output
function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Type = "Info"
    )

    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"

    switch ($Type) {
        "Info"    { Write-Host "[INFO] " -ForegroundColor Blue -NoNewline; Write-Host $Message }
        "Success" { Write-Host "[SUCCESS] " -ForegroundColor Green -NoNewline; Write-Host $Message }
        "Warning" { Write-Host "[WARNING] " -ForegroundColor Yellow -NoNewline; Write-Host $Message }
        "Error"   { Write-Host "[ERROR] " -ForegroundColor Red -NoNewline; Write-Host $Message }
        "Step"    { Write-Host "`n=== " -ForegroundColor Cyan -NoNewline; Write-Host $Message -ForegroundColor Cyan -NoNewline; Write-Host " ===" -ForegroundColor Cyan }
    }

    # Write to log file
    if (-not [string]::IsNullOrEmpty($script:LogFile)) {
        "[$timestamp] [$Type] $Message" | Out-File -FilePath $script:LogFile -Append -Encoding UTF8
    }
}

# ============================================================================
# RETRY LOGIC WITH EXPONENTIAL BACKOFF
# ============================================================================

function Invoke-WithRetry {
    param(
        [scriptblock]$ScriptBlock,
        [string]$Description = "operation",
        [int]$MaxAttempts = $script:NetworkRetryAttempts,
        [int]$InitialDelaySeconds = $script:NetworkRetryDelaySeconds
    )

    $attempt = 1
    $delay = $InitialDelaySeconds

    while ($attempt -le $MaxAttempts) {
        try {
            Write-ColorOutput "Attempt $attempt/$($MaxAttempts): $Description" "Info"
            $result = & $ScriptBlock
            return $result
        }
        catch {
            if ($attempt -eq $MaxAttempts) {
                Write-ColorOutput "Failed after $MaxAttempts attempts: $Description" "Error"
                Write-ColorOutput "Error: $_" "Error"
                throw
            }

            Write-ColorOutput "Attempt $attempt failed: $_. Retrying in ${delay}s..." "Warning"
            Start-Sleep -Seconds $delay

            # Exponential backoff (max 60 seconds)
            $delay = [Math]::Min($delay * 2, 60)
            $attempt++
        }
    }
}

# ============================================================================
# STATE PERSISTENCE
# ============================================================================

function Initialize-State {
    if (-not (Test-Path $script:StateDir)) {
        New-Item -ItemType Directory -Path $script:StateDir -Force | Out-Null
    }

    if (-not (Test-Path $script:StateFile)) {
        $state = @{
            LastCompletedStage = ""
            StartedAt = (Get-Date -Format "o")
            LastUpdated = (Get-Date -Format "o")
            BootstrapVersion = "2.0"
        }
        $state | ConvertTo-Json | Set-Content -Path $script:StateFile -Encoding UTF8
    }

    $script:State = Get-Content -Path $script:StateFile -Raw | ConvertFrom-Json
}

function Save-State {
    param([string]$Stage)

    $script:State.LastCompletedStage = $Stage
    $script:State.LastUpdated = (Get-Date -Format "o")
    $script:State | ConvertTo-Json | Set-Content -Path $script:StateFile -Encoding UTF8
    Write-ColorOutput "State saved: $Stage" "Info"
}

function Clear-BootstrapState {
    if (Test-Path $script:StateFile) {
        Remove-Item $script:StateFile -Force
        Write-ColorOutput "Bootstrap state cleared" "Info"
    }
}

function Get-LastCompletedStage {
    if (Test-Path $script:StateFile) {
        $state = Get-Content -Path $script:StateFile -Raw | ConvertFrom-Json
        return $state.LastCompletedStage
    }
    return ""
}

function Test-ShouldSkipStage {
    param([string]$Stage)

    if (-not $Resume) {
        return $false
    }

    $lastCompleted = Get-LastCompletedStage
    if ([string]::IsNullOrEmpty($lastCompleted)) {
        return $false
    }

    $lastIdx = [Array]::IndexOf($script:Stages, $lastCompleted)
    $currentIdx = [Array]::IndexOf($script:Stages, $Stage)

    if ($currentIdx -le $lastIdx) {
        Write-ColorOutput "Skipping already completed stage: $Stage" "Info"
        return $true
    }

    return $false
}

function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Test-WindowsVersion {
    Write-ColorOutput "Checking Windows version..." "Info"

    $osVersion = [System.Environment]::OSVersion.Version
    $buildNumber = (Get-ItemProperty "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion").CurrentBuildNumber

    # WSL2 requires Windows 10 version 2004 (build 19041) or higher
    if ($buildNumber -lt 19041) {
        Write-ColorOutput "Windows build $buildNumber detected. WSL2 requires build 19041 or higher." "Error"
        Write-ColorOutput "Please update Windows to version 2004 or later." "Error"
        return $false
    }

    Write-ColorOutput "Windows build $buildNumber - Compatible with WSL2" "Success"
    return $true
}

function Test-WSLInstalled {
    Write-ColorOutput "Checking WSL installation..." "Info"

    try {
        $wslStatus = wsl --status 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-ColorOutput "WSL is installed" "Success"
            return $true
        }
    }
    catch {
        # WSL not installed
    }

    return $false
}

function Test-WSL2Default {
    Write-ColorOutput "Checking WSL default version..." "Info"

    try {
        $wslStatus = wsl --status 2>&1 | Out-String
        if ($wslStatus -match "Default Version: 2" -or $wslStatus -match "既定のバージョン: 2") {
            Write-ColorOutput "WSL2 is the default version" "Success"
            return $true
        }
    }
    catch {
        # Unable to determine
    }

    return $false
}

function Install-WSL {
    Write-ColorOutput "Installing WSL..." "Step"

    # Enable WSL feature
    Write-ColorOutput "Enabling Windows Subsystem for Linux..." "Info"
    $wslFeature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
    if ($wslFeature.State -ne "Enabled") {
        Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart -WarningAction SilentlyContinue | Out-Null
        Write-ColorOutput "WSL feature enabled" "Success"
    }
    else {
        Write-ColorOutput "WSL feature already enabled" "Info"
    }

    # Enable Virtual Machine Platform
    Write-ColorOutput "Enabling Virtual Machine Platform..." "Info"
    $vmFeature = Get-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform
    if ($vmFeature.State -ne "Enabled") {
        Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart -WarningAction SilentlyContinue | Out-Null
        Write-ColorOutput "Virtual Machine Platform enabled" "Success"
    }
    else {
        Write-ColorOutput "Virtual Machine Platform already enabled" "Info"
    }

    # Install WSL
    Write-ColorOutput "Running WSL installation..." "Info"
    wsl --install --no-distribution

    # Set WSL2 as default
    Write-ColorOutput "Setting WSL2 as default version..." "Info"
    wsl --set-default-version 2

    Write-ColorOutput "WSL2 installation complete" "Success"
}

function Test-DistroExists {
    param([string]$Name)

    $distros = wsl --list --quiet 2>&1
    return $distros -contains $Name
}

function New-NixOSDistro {
    if (Test-ShouldSkipStage "CreateNixOSDistro") {
        return $true
    }

    Write-ColorOutput "Setting up NixOS WSL distribution..." "Step"

    # Check if distro already exists
    if (Test-DistroExists -Name $DistroName) {
        if ($Force) {
            Write-ColorOutput "Removing existing distro '$DistroName'..." "Warning"
            wsl --unregister $DistroName
        }
        else {
            Write-ColorOutput "Distro '$DistroName' already exists. Use -Force to replace it." "Warning"
            Save-State "CreateNixOSDistro"
            return $true
        }
    }

    # Create installation directory
    Write-ColorOutput "Creating installation directory: $InstallPath" "Info"
    if (-not (Test-Path $InstallPath)) {
        New-Item -ItemType Directory -Path $InstallPath -Force | Out-Null
    }

    # Download NixOS-WSL with retry logic
    $tarPath = Join-Path $InstallPath "nixos-wsl.tar.gz"
    Write-ColorOutput "Downloading NixOS-WSL (with retry on failure)..." "Info"

    try {
        Invoke-WithRetry -Description "Download NixOS-WSL" -ScriptBlock {
            Invoke-WebRequest -Uri $NixOSWSLRelease -OutFile $tarPath -UseBasicParsing -TimeoutSec 300
        }
        Write-ColorOutput "Download complete" "Success"
    }
    catch {
        Write-ColorOutput "Failed to download NixOS-WSL after multiple attempts: $_" "Error"
        return $false
    }

    # Verify the download
    if (-not (Test-Path $tarPath) -or (Get-Item $tarPath).Length -eq 0) {
        Write-ColorOutput "Downloaded file is missing or empty" "Error"
        return $false
    }

    # Import the distribution
    Write-ColorOutput "Importing NixOS distribution (this may take a few minutes)..." "Info"
    wsl --import $DistroName $InstallPath $tarPath --version 2

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to import NixOS distribution" "Error"
        return $false
    }

    Write-ColorOutput "NixOS distribution '$DistroName' created successfully" "Success"

    # Clean up tarball
    Remove-Item $tarPath -Force -ErrorAction SilentlyContinue

    Save-State "CreateNixOSDistro"
    return $true
}

function Set-VirtualDiskSize {
    if (Test-ShouldSkipStage "ConfigureVirtualDisk") {
        return $true
    }

    Write-ColorOutput "Configuring virtual disk..." "Step"

    $vhdxPath = Join-Path $InstallPath "ext4.vhdx"

    if (-not (Test-Path $vhdxPath)) {
        Write-ColorOutput "Virtual disk not found at expected location" "Warning"
        Save-State "ConfigureVirtualDisk"
        return $true
    }

    # NOTE: diskpart's `expand vdisk maximum=` expects the maximum size in **MB**.
    # We accept DiskSizeGB, so convert to MB.
    $diskSizeMB = [int64]$DiskSizeGB * 1024

    Write-ColorOutput "Resizing virtual disk to ${DiskSizeGB}GB..." "Info"

    # Shutdown WSL first
    wsl --shutdown
    Start-Sleep -Seconds 2

    try {
        # Use diskpart to resize
        $diskpartScript = @"
select vdisk file="$vhdxPath"
expand vdisk maximum=$diskSizeMB
"@
        $diskpartScript | diskpart | Out-Null

        Write-ColorOutput "Virtual disk resized to ${DiskSizeGB}GB" "Success"
    }
    catch {
        Write-ColorOutput "Note: Disk resize requires manual expansion inside WSL" "Warning"
    }

    Save-State "ConfigureVirtualDisk"
    return $true
}

function Set-WSLConfig {
    if (Test-ShouldSkipStage "ConfigureWSL") {
        return $true
    }

    Write-ColorOutput "Configuring WSL settings..." "Step"

    $wslConfigPath = "$env:USERPROFILE\.wslconfig"

    $wslConfig = @"
[wsl2]
memory=${MemorySizeGB}GB
swap=${SwapSizeGB}GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
"@

    Write-ColorOutput "Writing WSL configuration to $wslConfigPath" "Info"
    Set-Content -Path $wslConfigPath -Value $wslConfig -Force

    Write-ColorOutput "WSL configuration complete" "Success"
    Save-State "ConfigureWSL"
    return $true
}

function Initialize-NixOSEnvironment {
    if (Test-ShouldSkipStage "InitializeNixOS") {
        return $true
    }

    Write-ColorOutput "Initializing NixOS environment..." "Step"

    # Wait for NixOS to be ready
    Write-ColorOutput "Waiting for NixOS to initialize (first boot)..." "Info"
    wsl -d $DistroName -- echo "NixOS is ready"

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to start NixOS" "Error"
        return $false
    }

    # Run initial setup commands
    Write-ColorOutput "Running initial NixOS configuration..." "Info"

    # Update channels and rebuild
    $initScript = @'
set -e
echo "Updating Nix channels..."
sudo nix-channel --update

echo "Enabling flakes..."
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" > ~/.config/nix/nix.conf

echo "Initial setup complete"
'@

    wsl -d $DistroName -- bash -c $initScript

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to initialize NixOS environment" "Error"
        return $false
    }

    Write-ColorOutput "NixOS environment initialized" "Success"
    Save-State "InitializeNixOS"
    return $true
}

function Install-ROS2Environment {
    if (Test-ShouldSkipStage "InstallROS2Environment") {
        return $true
    }

    Write-ColorOutput "Setting up ROS2 development environment..." "Step"

    # Clone the repository
    Write-ColorOutput "Cloning ripple-env repository..." "Info"

    $cloneScript = @'
set -euo pipefail

repo_dir="${REPO_DIR:-ripple-env}"
repo_url="${REPO_URL:?REPO_URL is required}"
repo_fetch_ref="${REPO_FETCH_REF:-}"

cd ~

if [ -d "$repo_dir/.git" ]; then
    echo "Repository already exists, updating reminder: origin -> $repo_url"
    cd "$repo_dir"
    git remote set-url origin "$repo_url" || true
else
    echo "Cloning repository..."
    git clone "$repo_url" "$repo_dir"
    cd "$repo_dir"
fi

if [ -n "$repo_fetch_ref" ]; then
    echo "Fetching ref: $repo_fetch_ref"
    git fetch --prune origin "$repo_fetch_ref"
    echo "Checking out fetched ref (detached HEAD)"
    git checkout --detach FETCH_HEAD
else
    echo "No REPO_FETCH_REF provided; fetching default refs"
    git fetch --prune origin
    # If we're on a branch, try a fast-forward update (don't fail if not possible)
    if git symbolic-ref -q HEAD >/dev/null; then
        git pull --ff-only || true
    fi
fi

echo "Repository ready"
'@

    wsl -d $DistroName -- env "REPO_URL=$RepoURL" "REPO_FETCH_REF=$RepoFetchRef" "REPO_DIR=ripple-env" bash -lc $cloneScript

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to clone repository" "Error"
        return $false
    }

    # Run the bootstrap script
    Write-ColorOutput "Running bootstrap script (this may take a while)..." "Info"

    $bootstrapScript = @'
set -e
cd ~/ripple-env

# Make bootstrap executable
chmod +x bootstrap.sh

# Run bootstrap in CI mode (non-interactive)
if [ "${SKIP_SHELLS:-}" = "1" ]; then
    ./bootstrap.sh --ci --skip-shells
else
    ./bootstrap.sh --ci
fi

echo "Bootstrap complete!"
'@

    $skipShellsEnv = if ($SkipShells) { "1" } else { "" }
    wsl -d $DistroName -- env "SKIP_SHELLS=$skipShellsEnv" bash -lc $bootstrapScript

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Bootstrap script encountered errors" "Error"
        Write-ColorOutput "You can resume the installation by running: .\bootstrap.ps1 -Resume" "Info"
        return $false
    }

    Write-ColorOutput "ROS2 development environment installed successfully" "Success"
    Save-State "InstallROS2Environment"
    return $true
}

function Set-DefaultDistro {
    if (Test-ShouldSkipStage "SetDefaultDistro") {
        return $true
    }

    Write-ColorOutput "Setting $DistroName as default WSL distribution..." "Info"
    wsl --set-default $DistroName
    Write-ColorOutput "Default distribution set" "Success"
    Save-State "SetDefaultDistro"
    return $true
}

function Show-Summary {
    Write-ColorOutput "Installation Summary" "Step"

    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "  ROS2 Humble Environment Ready!       " -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Distribution: $DistroName"
    Write-Host "Location: $InstallPath"
    Write-Host "Disk Size: ${DiskSizeGB}GB"
    Write-Host "WSL Memory: ${MemorySizeGB}GB"
    Write-Host "Swap Size: ${SwapSizeGB}GB"
    Write-Host ""
    Write-Host "To enter the environment:" -ForegroundColor Cyan
    Write-Host "  wsl -d $DistroName"
    Write-Host "  cd ~/ripple-env"
    Write-Host "  direnv allow  # or: nom develop"
    Write-Host ""
    Write-Host "Quick commands inside WSL:" -ForegroundColor Cyan
    Write-Host "  cb   - colcon build --symlink-install"
    Write-Host "  ct   - colcon test"
    Write-Host "  ros2 - ROS2 CLI"
    Write-Host ""
    Write-Host "For different shells:" -ForegroundColor Cyan
    Write-Host "  nom develop -c env 'SHELL=/bin/nu' /bin/nu    # Nushell"
    Write-Host "  nom develop -c env 'SHELL=/bin/zsh' /bin/zsh  # Zsh"
    Write-Host ""
    if (-not [string]::IsNullOrEmpty($script:LogFile) -and (Test-Path $script:LogFile)) {
        Write-Host "Log file: $script:LogFile" -ForegroundColor Gray
    }
    Write-Host "State file: $script:StateFile" -ForegroundColor Gray
    Write-Host "To clean state and start fresh: .\bootstrap.ps1 -Clean" -ForegroundColor Gray
    Write-Host ""
}

function Show-RestartRequired {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "  RESTART REQUIRED                     " -ForegroundColor Yellow
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Windows features have been enabled that require a restart."
    Write-Host "Please restart your computer and run this script again."
    Write-Host ""

    $restart = Read-Host "Would you like to restart now? (y/N)"
    if ($restart -eq 'y' -or $restart -eq 'Y') {
        Restart-Computer -Force
    }
}

# Main execution
function Main {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "  ROS2 Humble Environment Bootstrap    " -ForegroundColor Cyan
    Write-Host "  Windows WSL2 + NixOS Setup           " -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host ""

    # Initialize logging
    Write-ColorOutput "Logging to: $script:LogFile" "Info"
    Write-ColorOutput "Bootstrap started at $(Get-Date -Format 'o')" "Info"

    # Handle clean mode
    if ($Clean) {
        Clear-BootstrapState
    }

    # Initialize state tracking
    Initialize-State

    # Report resume status
    if ($Resume) {
        $lastStage = Get-LastCompletedStage
        if (-not [string]::IsNullOrEmpty($lastStage)) {
            Write-ColorOutput "Resuming from stage: $lastStage" "Info"
        }
        else {
            Write-ColorOutput "No previous state found, starting fresh" "Info"
        }
    }

    # Check administrator
    if (-not (Test-Administrator)) {
        Write-ColorOutput "This script must be run as Administrator" "Error"
        Write-ColorOutput "Right-click PowerShell and select 'Run as Administrator'" "Info"
        exit 1
    }

    # Check Windows version
    if (-not (Test-ShouldSkipStage "CheckWindowsVersion")) {
        if (-not (Test-WindowsVersion)) {
            exit 1
        }
        Save-State "CheckWindowsVersion"
    }

    # Check/Install WSL
    if (-not $SkipWSLCheck -and -not (Test-ShouldSkipStage "InstallWSL")) {
        $wslInstalled = Test-WSLInstalled
        $wsl2Default = $false

        if ($wslInstalled) {
            $wsl2Default = Test-WSL2Default
        }

        if (-not $wslInstalled -or -not $wsl2Default) {
            Install-WSL

            # Check if restart is needed
            $wslCheck = wsl --status 2>&1
            if ($LASTEXITCODE -ne 0) {
                Show-RestartRequired
                exit 0
            }
        }
        Save-State "InstallWSL"
    }

    # Configure WSL settings
    Set-WSLConfig

    # Create NixOS distribution
    if (-not (New-NixOSDistro)) {
        Write-ColorOutput "Failed to create NixOS distribution" "Error"
        Write-ColorOutput "To resume, run: .\bootstrap.ps1 -Resume" "Info"
        exit 1
    }

    # Configure virtual disk size
    Set-VirtualDiskSize

    # Shutdown WSL to apply changes
    Write-ColorOutput "Restarting WSL to apply configuration..." "Info"
    wsl --shutdown
    Start-Sleep -Seconds 3

    # Initialize NixOS
    if (-not (Initialize-NixOSEnvironment)) {
        Write-ColorOutput "Failed to initialize NixOS environment" "Error"
        Write-ColorOutput "To resume, run: .\bootstrap.ps1 -Resume" "Info"
        exit 1
    }

    # Install ROS2 environment
    if (-not (Install-ROS2Environment)) {
        Write-ColorOutput "Failed to install ROS2 environment" "Error"
        Write-ColorOutput "To resume, run: .\bootstrap.ps1 -Resume" "Info"
        exit 1
    }

    # Set as default distribution
    Set-DefaultDistro

    # Clear state on successful completion
    Clear-BootstrapState
    Write-ColorOutput "Bootstrap completed successfully at $(Get-Date -Format 'o')" "Success"

    # Show summary
    Show-Summary
}

# Run main
try {
    Main
}
catch {
    Write-ColorOutput "An error occurred: $_" "Error"
    Write-ColorOutput $_.ScriptStackTrace "Error"
    exit 1
}
