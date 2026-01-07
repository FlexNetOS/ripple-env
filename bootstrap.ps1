# ROS2 Humble Environment Bootstrap Script for Windows/PowerShell
# Requires: Nix (with WSL2 or native Windows support)

# Set strict mode
$ErrorActionPreference = "Stop"

# Color functions
function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Color = "White"
    )
    Write-Host $Message -ForegroundColor $Color
}

function Write-Error-Message {
    param([string]$Message)
    Write-ColorOutput "ERROR: $Message" "Red"
}

function Write-Success-Message {
    param([string]$Message)
    Write-ColorOutput "SUCCESS: $Message" "Green"
}

function Write-Info-Message {
    param([string]$Message)
    Write-ColorOutput "INFO: $Message" "Yellow"
}

# Check if Nix is installed
function Test-NixInstalled {
    try {
        $nixPath = Get-Command nix -ErrorAction Stop
        Write-Success-Message "Nix is installed at: $($nixPath.Source)"
        
        # Check if flakes are enabled
        $flakeTest = nix eval --expr "1 + 1" 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Error-Message "Nix is installed but flakes are not enabled"
            Write-Host ""
            Write-Host "To enable flakes, add the following to your Nix configuration:"
            Write-Host "  experimental-features = nix-command flakes"
            Write-Host ""
            Write-Host "On WSL2/Linux: ~/.config/nix/nix.conf or /etc/nix/nix.conf"
            Write-Host ""
            return $false
        }
        
        return $true
    }
    catch {
        Write-Error-Message "Nix is not installed or not in PATH"
        Write-Host ""
        Write-Host "To use this environment on Windows, you need either:"
        Write-Host ""
        Write-Host "1. WSL2 with Nix installed (recommended):"
        Write-Host "   - Install WSL2: wsl --install"
        Write-Host "   - Install Nix in WSL2: sh <(curl -L https://nixos.org/nix/install) --daemon"
        Write-Host ""
        Write-Host "2. Use bootstrap.sh from within WSL2"
        Write-Host ""
        Write-Host "Note: Native Windows Nix support is experimental."
        Write-Host ""
        return $false
    }
}

# Check if we're in WSL2
function Test-IsWSL {
    if ($env:WSL_DISTRO_NAME) {
        return $true
    }
    return $false
}

# Check and recommend direnv
function Test-Direnv {
    try {
        $direnvPath = Get-Command direnv -ErrorAction Stop
        Write-Success-Message "direnv is installed at: $($direnvPath.Source)"
        return $true
    }
    catch {
        Write-Info-Message "direnv is not installed (recommended for automatic environment activation)"
        Write-Host ""
        Write-Host "To install direnv:"
        Write-Host "  - On WSL2/Linux: sudo apt install direnv (Ubuntu/Debian)"
        Write-Host "  - On macOS: brew install direnv"
        Write-Host ""
        Write-Host "After installation, add to your shell config:"
        Write-Host "  - bash: echo 'eval `"`$(direnv hook bash)`"' >> ~/.bashrc"
        Write-Host "  - zsh: echo 'eval `"`$(direnv hook zsh)`"' >> ~/.zshrc"
        Write-Host ""
        return $false
    }
}

# Main bootstrap process
function Start-Bootstrap {
    Write-Host "=========================================="
    Write-Host "  ROS2 Humble Environment Bootstrap"
    Write-Host "=========================================="
    Write-Host ""
    
    # Check if we're in WSL2
    if (Test-IsWSL) {
        Write-Info-Message "Running in WSL2 environment"
        Write-Host ""
    }
    
    # Check prerequisites
    if (-not (Test-NixInstalled)) {
        exit 1
    }
    
    Write-Info-Message "Pixi will be provided by the Nix dev shell"
    Write-Host ""
    
    # Check for direnv (recommended)
    $direnvAvailable = Test-Direnv
    Write-Host ""
    
    Write-Info-Message "Entering Nix dev shell and running pixi install..."
    Write-Host ""
    
    # Enter the dev shell and run pixi install
    $nixCommand = @"
set -euo pipefail

# Verify pixi is available
if ! command -v pixi &> /dev/null; then
    echo 'ERROR: pixi is not available in the dev shell' >&2
    exit 1
fi

echo 'Pixi version: `$(pixi --version)`'
echo ''
echo 'Running pixi install...'
pixi install

echo ''
echo 'Bootstrap completed successfully!'
echo ''
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  Next Steps'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo ''
"@
    
    try {
        nix develop --command bash -c $nixCommand
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host ""
            
            # Provide next steps based on whether direnv is available
            if ($direnvAvailable) {
                Write-Host "RECOMMENDED: Use direnv for automatic environment activation" -ForegroundColor Green
                Write-Host ""
                Write-Host "  1. Allow direnv for this directory:"
                Write-Host "     direnv allow"
                Write-Host ""
                Write-Host "  2. The environment will activate automatically when you cd into this directory"
                Write-Host ""
                Write-Host "Or manually enter the environment:"
                Write-Host "  nix develop"
            }
            else {
                Write-Host "To enter the development environment:" -ForegroundColor Yellow
                Write-Host ""
                Write-Host "  nix develop"
                Write-Host ""
                Write-Host "RECOMMENDED: Install direnv for automatic environment activation"
                Write-Host "  (see installation instructions above)"
            }
            
            Write-Host ""
            Write-Success-Message "Setup complete!"
        }
        else {
            Write-Host ""
            Write-Error-Message "Bootstrap failed with exit code $LASTEXITCODE"
            exit $LASTEXITCODE
        }
    }
    catch {
        Write-Host ""
        Write-Error-Message "Bootstrap failed: $_"
        exit 1
    }
}

# Run main function
Start-Bootstrap
