#!/usr/bin/env pwsh
# FlexNetOS WSL Installation Script

param(
    [Parameter(Mandatory=$false)]
    [string]$DistributionName = "FlexNetOS",
    
    [Parameter(Mandatory=$false)]
    [string]$InstallPath = "$env:LOCALAPPDATA\Packages\FlexNetOS",
    
    [Parameter(Mandatory=$false)]
    [string]$RootfsPath = "flexnetos-rootfs.tar.gz"
)

$ErrorActionPreference = "Stop"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " FlexNetOS WSL Installation" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# Check if running as administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")
if (-not $isAdmin) {
    Write-Host "This script must be run as Administrator" -ForegroundColor Red
    exit 1
}

# Check if WSL2 is available
try {
    $wslVersion = wsl --version
    Write-Host "WSL version: $wslVersion" -ForegroundColor Green
} catch {
    Write-Host "WSL is not installed. Please install WSL2 first." -ForegroundColor Red
    Write-Host "Run: wsl --install" -ForegroundColor Yellow
    exit 1
}

# Check if distribution already exists
$existingDistro = wsl -l -v | Select-String $DistributionName
if ($existingDistro) {
    Write-Host "Distribution '$DistributionName' already exists." -ForegroundColor Yellow
    $response = Read-Host "Do you want to replace it? (y/N)"
    if ($response -eq "y" -or $response -eq "Y") {
        Write-Host "Unregistering existing distribution..." -ForegroundColor Yellow
        wsl --unregister $DistributionName
    } else {
        Write-Host "Installation cancelled." -ForegroundColor Yellow
        exit 0
    }
}

# Create installation directory
Write-Host "Creating installation directory..." -ForegroundColor Green
New-Item -ItemType Directory -Force -Path $InstallPath | Out-Null

# Import WSL distribution
Write-Host "Importing FlexNetOS distribution..." -ForegroundColor Green
wsl --import $DistributionName $InstallPath $RootfsPath --version 2

if ($LASTEXITCODE -ne 0) {
    Write-Host "WSL import failed!" -ForegroundColor Red
    exit 1
}

Write-Host "FlexNetOS installed successfully!" -ForegroundColor Green

# Configure default user
Write-Host "Configuring default user..." -ForegroundColor Green
try {
    wsl -d $DistributionName -u root usermod --home /home/flexnet flexnet
    wsl -d $DistributionName -u root chsh -s /run/current-system/sw/bin/zsh flexnet
    Write-Host "Default user configured: flexnet" -ForegroundColor Green
} catch {
    Write-Host "Warning: Could not configure default user" -ForegroundColor Yellow
}

Write-Host "========================================" -ForegroundColor Cyan
Write-Host " Installation completed successfully!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "To start FlexNetOS:" -ForegroundColor Yellow
Write-Host "  wsl -d $DistributionName" -ForegroundColor White
Write-Host "  or" -ForegroundColor Gray
Write-Host "  wsl -d $DistributionName -u flexnet" -ForegroundColor White
Write-Host "========================================" -ForegroundColor Cyan
