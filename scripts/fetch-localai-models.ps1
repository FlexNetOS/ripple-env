[CmdletBinding(SupportsShouldProcess = $true)]
param(
    [ValidateSet('pull','cache','restore','status')]
    [string]$Action = 'status',

    [string]$RepoRoot,
    [string]$ModelsDir,
    [string]$CacheDir,

    [switch]$UseLfs = $true
)

function Resolve-RepoRoot {
    param([string]$Hint)
    if ($Hint) {
        return (Resolve-Path -LiteralPath $Hint).Path
    }

    try {
        $top = git rev-parse --show-toplevel 2>$null
        if ($LASTEXITCODE -eq 0 -and $top) {
            return $top.Trim()
        }
    } catch {}

    throw "Unable to determine repo root. Pass -RepoRoot or run from inside a git working tree."
}

function Get-DefaultCacheDir {
    if ($env:LOCALAI_MODELS_CACHE_DIR) {
        return $env:LOCALAI_MODELS_CACHE_DIR
    }

    if ($env:LOCALAPPDATA) {
        return (Join-Path $env:LOCALAPPDATA 'FlexNetOS\cache\localai\models')
    }

    # Fallback
    return (Join-Path $HOME '.cache\flexnetos\localai\models')
}

function Ensure-Dir {
    param([string]$Path)
    if (-not (Test-Path -LiteralPath $Path)) {
        if ($PSCmdlet.ShouldProcess($Path, 'Create directory')) {
            New-Item -ItemType Directory -Path $Path -Force | Out-Null
        }
    }
}

function Copy-Tree {
    param(
        [string]$From,
        [string]$To
    )

    Ensure-Dir -Path $To

    # Use robocopy for large files; it is resilient and preserves timestamps.
    $rc = 0
    if ($PSCmdlet.ShouldProcess("$From -> $To", 'Robocopy')) {
        $null = robocopy $From $To /E /Z /R:2 /W:2 /NFL /NDL /NP /NJH /NJS
        $rc = $LASTEXITCODE
    }

    # Robocopy exit codes < 8 are success (includes "extra files" and "mismatched" warnings).
    if ($rc -ge 8) {
        throw "Robocopy failed with exit code $rc"
    }
}

$RepoRoot = Resolve-RepoRoot -Hint $RepoRoot

if (-not $ModelsDir) {
    $ModelsDir = Join-Path $RepoRoot 'docker\data\localai\models'
}
if (-not $CacheDir) {
    $CacheDir = Get-DefaultCacheDir
}

switch ($Action) {
    'pull' {
        if (-not $UseLfs) {
            throw "Action 'pull' currently supports only Git LFS as a source (set -UseLfs)."
        }

        Push-Location $RepoRoot
        try {
            if ($PSCmdlet.ShouldProcess($RepoRoot, 'git lfs pull (LocalAI models)')) {
                git lfs pull --include="docker/data/localai/models/**"
                if ($LASTEXITCODE -ne 0) { throw "git lfs pull failed with exit code $LASTEXITCODE" }
            }
        } finally {
            Pop-Location
        }
    }

    'cache' {
        Ensure-Dir -Path $ModelsDir
        Ensure-Dir -Path $CacheDir
        Copy-Tree -From $ModelsDir -To $CacheDir
    }

    'restore' {
        Ensure-Dir -Path $ModelsDir
        Ensure-Dir -Path $CacheDir
        Copy-Tree -From $CacheDir -To $ModelsDir
    }

    'status' {
        Write-Host "RepoRoot : $RepoRoot"
        Write-Host "ModelsDir: $ModelsDir"
        Write-Host "CacheDir : $CacheDir"

        if (-not (Test-Path -LiteralPath $ModelsDir)) {
            Write-Host "ModelsDir does not exist yet." -ForegroundColor Yellow
            exit 0
        }

        $modelFiles = Get-ChildItem -LiteralPath $ModelsDir -Recurse -File -ErrorAction SilentlyContinue
        $totalMiB = [math]::Round(($modelFiles | Measure-Object -Property Length -Sum).Sum / 1MB, 2)
        Write-Host ("ModelsDir files: {0} (total {1} MiB)" -f $modelFiles.Count, $totalMiB)

        if (Test-Path -LiteralPath $CacheDir) {
            $cacheFiles = Get-ChildItem -LiteralPath $CacheDir -Recurse -File -ErrorAction SilentlyContinue
            $cacheMiB = [math]::Round(($cacheFiles | Measure-Object -Property Length -Sum).Sum / 1MB, 2)
            Write-Host ("CacheDir files:  {0} (total {1} MiB)" -f $cacheFiles.Count, $cacheMiB)
        } else {
            Write-Host "CacheDir does not exist yet." -ForegroundColor Yellow
        }

        Write-Host "\nTips:"
        Write-Host "  - To fetch models from Git LFS:   ./scripts/fetch-localai-models.ps1 -Action pull"
        Write-Host "  - To cache models locally:        ./scripts/fetch-localai-models.ps1 -Action cache"
        Write-Host "  - To restore from cache to repo:  ./scripts/fetch-localai-models.ps1 -Action restore"
        Write-Host "  - Use -WhatIf to dry-run copy actions."
    }
}
