# Script Contract: fetch-localai-models.ps1

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/fetch-localai-models.ps1`

---

## Purpose

PowerShell equivalent of fetch-localai-models.sh for Windows. Fetch, cache, and restore LocalAI models via Git LFS. Uses Robocopy for efficient large file operations. Supports `-WhatIf` for dry-run testing.

---

## Invocation

```powershell
.\scripts\fetch-localai-models.ps1 [-Action <action>] [options]
```

**Parameters:**
- `-Action` - Action to perform: `pull`, `cache`, `restore`, `status` (default: status)
- `-RepoRoot` - Override repository root path
- `-ModelsDir` - Override models directory path
- `-CacheDir` - Override cache directory path
- `-UseLfs` - Use Git LFS for pull (default: true)
- `-WhatIf` - Dry run (shows what would happen)

**Environment Variables:**
- `LOCALAI_MODELS_CACHE_DIR` - Override cache directory
- `LOCALAPPDATA` - Windows local app data (for cache default)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success |
| Non-zero | Error (git failure, robocopy error >= 8, missing repo root) |

### Status Action Output
```
RepoRoot : C:\repos\ripple-env
ModelsDir: C:\repos\ripple-env\docker\data\localai\models
CacheDir : C:\Users\user\AppData\Local\FlexNetOS\cache\localai\models
ModelsDir files: 5 (total 15234.56 MiB)
CacheDir files:  5 (total 15234.56 MiB)

Tips:
  - To fetch models from Git LFS:   ./scripts/fetch-localai-models.ps1 -Action pull
  - To cache models locally:        ./scripts/fetch-localai-models.ps1 -Action cache
  - To restore from cache to repo:  ./scripts/fetch-localai-models.ps1 -Action restore
  - Use -WhatIf to dry-run copy actions.
```

---

## Side Effects

### Pull Action (lines 82-95)
- **Downloads models** from Git LFS into `docker\data\localai\models\`
- **Command:** `git lfs pull --include="docker/data/localai/models/**"`
- **Size:** Potentially gigabytes

### Cache Action (lines 98-102)
- **Copies models** from working tree to cache directory
- **Uses Robocopy** for Windows file copy
- **Preserves:** Timestamps, retries on failures

### Restore Action (lines 104-108)
- **Copies models** from cache to working tree
- **Overwrites** existing models

---

## Safety Classification

**🟡 CAUTION** - Downloads and copies large files, overwrites on restore.

---

## Idempotency

**✅ IDEMPOTENT (pull, status)** - Can be run repeatedly.

**⚠️ DESTRUCTIVE (cache, restore)** - Overwrites destination with source.

---

## Path Resolution

### Repo Root (lines 13-27)

**Evidence:** Function `Resolve-RepoRoot`

```powershell
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
```

**Uses git** if no hint provided.

### Cache Directory (lines 29-40)

**Evidence:** Function `Get-DefaultCacheDir`

**Priority:**
1. `$env:LOCALAI_MODELS_CACHE_DIR` (explicit override)
2. `$env:LOCALAPPDATA\FlexNetOS\cache\localai\models` (Windows standard)
3. `$HOME\.cache\flexnetos\localai\models` (fallback)

**Typical:** `C:\Users\user\AppData\Local\FlexNetOS\cache\localai\models`

### Models Directory (lines 74-76)

```powershell
if (-not $ModelsDir) {
    $ModelsDir = Join-Path $RepoRoot 'docker\data\localai\models'
}
```

---

## Actions

### pull (lines 82-95)

**Command:**
```powershell
Push-Location $RepoRoot
try {
    if ($PSCmdlet.ShouldProcess($RepoRoot, 'git lfs pull (LocalAI models)')) {
        git lfs pull --include="docker/data/localai/models/**"
        if ($LASTEXITCODE -ne 0) { throw "git lfs pull failed with exit code $LASTEXITCODE" }
    }
} finally {
    Pop-Location
}
```

**ShouldProcess:** Respects `-WhatIf` parameter.

**Error handling:** Throws exception on git failure.

### cache (lines 98-102)

```powershell
Ensure-Dir -Path $ModelsDir
Ensure-Dir -Path $CacheDir
Copy-Tree -From $ModelsDir -To $CacheDir
```

### restore (lines 104-108)

```powershell
Ensure-Dir -Path $ModelsDir
Ensure-Dir -Path $CacheDir
Copy-Tree -From $CacheDir -To $ModelsDir
```

### status (lines 110-137)

**Displays:**
- Repository root, models directory, cache directory paths
- File count and total size in MiB for both directories
- Usage tips with commands

**Size calculation:**
```powershell
$modelFiles = Get-ChildItem -LiteralPath $ModelsDir -Recurse -File
$totalMiB = [math]::Round(($modelFiles | Measure-Object -Property Length -Sum).Sum / 1MB, 2)
Write-Host ("ModelsDir files: {0} (total {1} MiB)" -f $modelFiles.Count, $totalMiB)
```

---

## Copy Strategy

**Evidence:** Lines 51-70

**Function:** `Copy-Tree(From, To)`

**Uses Robocopy:**
```powershell
robocopy $From $To /E /Z /R:2 /W:2 /NFL /NDL /NP /NJH /NJS
```

**Robocopy flags:**
- `/E` - Copy subdirectories, including empty
- `/Z` - Restartable mode (for large files)
- `/R:2` - Retry 2 times on failure
- `/W:2` - Wait 2 seconds between retries
- `/NFL /NDL /NP` - No file list, no directory list, no progress
- `/NJH /NJS` - No job header, no job summary

**Exit code handling:**
```powershell
# Robocopy exit codes < 8 are success
if ($rc -ge 8) {
    throw "Robocopy failed with exit code $rc"
}
```

**Robocopy exit codes:**
- 0 - No files copied
- 1 - Files copied successfully
- 2 - Extra files/directories detected
- 4 - Mismatched files/directories
- 8+ - Errors occurred

---

## WhatIf Support

**Evidence:** Line 1, lines 45, 61, 89

**Cmdlet binding:**
```powershell
[CmdletBinding(SupportsShouldProcess = $true)]
```

**ShouldProcess calls:**
```powershell
if ($PSCmdlet.ShouldProcess($Path, 'Create directory'))
if ($PSCmdlet.ShouldProcess("$From -> $To", 'Robocopy'))
if ($PSCmdlet.ShouldProcess($RepoRoot, 'git lfs pull (LocalAI models)'))
```

**Usage:**
```powershell
# Dry run
.\scripts\fetch-localai-models.ps1 -Action cache -WhatIf

# Output: What if: Performing the operation "Robocopy" on target "...".
```

---

## Workflow Examples

### Initial Setup (Windows)

```powershell
# 1. Clone repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# 2. Pull models from Git LFS
.\scripts\fetch-localai-models.ps1 -Action pull

# 3. Cache models locally (optional)
.\scripts\fetch-localai-models.ps1 -Action cache

# 4. Start LocalAI
docker compose -f docker-compose.localai.yml up -d
```

### Test Before Running

```powershell
# Dry run pull
.\scripts\fetch-localai-models.ps1 -Action pull -WhatIf

# Dry run cache
.\scripts\fetch-localai-models.ps1 -Action cache -WhatIf
```

### Check Status

```powershell
# Show paths and sizes
.\scripts\fetch-localai-models.ps1 -Action status
```

---

## Robocopy Advantages

**Windows-optimized:**
- Multi-threaded (faster than cp/rsync on Windows)
- Restartable mode for large files
- Automatic retry on transient failures
- Preserves all NTFS attributes

**Alternative:** PowerShell native `Copy-Item -Recurse` is slower for large trees.

---

## Parameter Validation

**Evidence:** Line 3

**ValidateSet:**
```powershell
[ValidateSet('pull','cache','restore','status')]
[string]$Action = 'status'
```

**Benefit:** Tab completion and automatic validation.

---

## References

### Source Code
- **Main script:** `scripts/fetch-localai-models.ps1` (139 lines)
- **Path resolution:** lines 13-40
- **Copy strategy:** lines 51-70
- **Actions:** lines 81-138

### Related Files
- **Bash equivalent:** `scripts/fetch-localai-models.sh`
- **Environment setup:** `scripts/env-vars.sh`
- **LocalAI compose:** `docker-compose.localai.yml`

### External Resources
- [Git LFS](https://git-lfs.github.com/)
- [LocalAI](https://localai.io/)
- [Robocopy Documentation](https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/robocopy)
- [PowerShell ShouldProcess](https://docs.microsoft.com/en-us/powershell/scripting/developer/cmdlet/supporting-user-feedback)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 22/60 contracts complete
