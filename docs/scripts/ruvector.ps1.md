# Script Contract: ruvector.ps1

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/ruvector.ps1`

---

## Purpose

Windows/PowerShell wrapper for RuVector CLI that solves npm cache issues on Windows systems. Forces repo-local npm cache and prefix directories to avoid conflicts with global npm configurations that may point to missing network drives. Provides seamless `npx ruvector` execution with proper environment isolation.

---

## Invocation

```powershell
.\scripts\ruvector.ps1 [ARGS...]
```

**Arguments:** All arguments passed directly to `npx ruvector`

**Examples:**
```powershell
.\scripts\ruvector.ps1 --version
.\scripts\ruvector.ps1 create ./data/ruvector.db
.\scripts\ruvector.ps1 insert ./data/ruvector.db ./vectors.json
.\scripts\ruvector.ps1 search ./data/ruvector.db --vector "[1,0,0]" --top-k 3
.\scripts\ruvector.ps1 delete ./data/ruvector.db --id 42
.\scripts\ruvector.ps1 list ./data/ruvector.db
```

**Requirements:**
- PowerShell 5.1+ (Windows) or PowerShell Core 7+ (cross-platform)
- Node.js with npm and npx
- Internet connection (for initial RuVector download)

---

## Outputs

**Standard Output (version):**
```
ruvector 0.5.2
```

**Standard Output (create database):**
```
Created RuVector database at ./data/ruvector.db
```

**Standard Output (search):**
```
Top 3 results:
  ID: 15  Distance: 0.05  Vector: [0.98, 0.02, 0.01]
  ID: 27  Distance: 0.12  Vector: [0.95, 0.03, 0.02]
  ID: 8   Distance: 0.18  Vector: [0.92, 0.05, 0.03]
```

**Standard Output (no arguments):**
```
Usage: ruvector <command> [options]

Commands:
  create <db>              Create a new vector database
  insert <db> <file>       Insert vectors from JSON file
  search <db>              Search for similar vectors
  delete <db>              Delete vectors by ID
  list <db>                List all vectors

Options:
  --version                Show version number
  --help                   Show help
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (command completed) |
| `1+` | Failure (RuVector command failed) |
| (exception) | Error (npx not found, other PowerShell errors) |

---

## Side Effects

### npm Cache Directory (line 23)

**Evidence:**
```powershell
$npmCache = Join-Path $repoRoot '.npm-cache'
New-Item -ItemType Directory -Force -Path $npmCache, $npmPrefix | Out-Null
```

**Creates:** `.npm-cache/` directory in repository root

**Purpose:** Isolate npm cache from global configuration

### npm Prefix Directory (line 23)

**Evidence:**
```powershell
$npmPrefix = Join-Path $repoRoot '.npm-prefix'
New-Item -ItemType Directory -Force -Path $npmCache, $npmPrefix | Out-Null
```

**Creates:** `.npm-prefix/` directory in repository root

**Purpose:** Isolate npm global packages from system-wide npm prefix

### Environment Variables (lines 25-26)

**Evidence:**
```powershell
$env:NPM_CONFIG_CACHE = $npmCache
$env:NPM_CONFIG_PREFIX = $npmPrefix
```

**Sets:**
- `NPM_CONFIG_CACHE` - npm cache directory
- `NPM_CONFIG_PREFIX` - npm prefix directory

**Scope:** Process-local (does not modify system or user environment)

### RuVector Package Installation (lines 39, 49)

**Evidence:**
```powershell
& npx --yes ruvector --help
```

**Downloads:** RuVector npm package (if not cached)

**Flag:** `--yes` auto-confirms npx installation prompts

---

## Safety Classification

**🟢 SAFE** - Creates directories, sets process environment variables, executes npx.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly. Creates directories only if missing (line 23).

---

## Problem Statement

**Evidence:** Lines 1-8 (comment block)

```powershell
<#
  RuVector launcher (Windows / PowerShell)

  Why this exists:
  Some environments have a global npm prefix/cache pointing to a missing drive
  (e.g. N:\...), which breaks `npx`.

  This wrapper forces repo-local npm cache/prefix so `npx ruvector ...` works.
#>
```

**Common issue on Windows:**
- Network drive (e.g., `N:\`) configured as npm cache/prefix
- Drive unavailable (disconnected from network)
- `npx` fails with "ENOENT" or "drive not found" errors

**Solution:**
- Override `NPM_CONFIG_CACHE` and `NPM_CONFIG_PREFIX`
- Use local directories within repository

---

## Repository Root Resolution

**Evidence:** Line 19

```powershell
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
```

**Process:**
1. `$PSScriptRoot` - Directory containing this script (`scripts/`)
2. `Join-Path $PSScriptRoot '..'` - Parent directory (repository root)
3. `Resolve-Path` - Canonicalize path (resolve `..\` to absolute path)
4. `.Path` - Extract string path from PathInfo object

**Result:** Absolute path to repository root

---

## Directory Creation

**Evidence:** Line 23

```powershell
New-Item -ItemType Directory -Force -Path $npmCache, $npmPrefix | Out-Null
```

**Parameters:**
- `-ItemType Directory` - Create directories (not files)
- `-Force` - Create if missing, no error if exists
- `-Path $npmCache, $npmPrefix` - Both directories at once
- `| Out-Null` - Suppress output

**Idempotent:** `-Force` flag makes this safe to run repeatedly

---

## npx Availability Check

**Evidence:** Lines 28-30

```powershell
if (-not (Get-Command npx -ErrorAction SilentlyContinue)) {
  throw "npx not found. Install Node.js (includes npm/npx) and try again."
}
```

**Check:** `Get-Command npx`

**Suppresses errors:** `-ErrorAction SilentlyContinue`

**Throws exception** if npx not found (line 29)

---

## Execution Logic

### No Arguments (lines 32-41)

**Evidence:**
```powershell
if ($args.Count -eq 0) {
  $localBin = Join-Path $repoRoot 'node_modules\.bin\ruvector.cmd'
  if (Test-Path $localBin) {
    & $localBin --help
    exit $LASTEXITCODE
  }

  & npx --yes ruvector --help
  exit $LASTEXITCODE
}
```

**Behavior:** Show help if no arguments

**Preference:**
1. Local installation (line 34-36) - `node_modules\.bin\ruvector.cmd`
2. npx download (line 39-40) - `npx --yes ruvector`

### With Arguments (lines 43-50)

**Evidence:**
```powershell
$localBin = Join-Path $repoRoot 'node_modules\.bin\ruvector.cmd'
if (Test-Path $localBin) {
  & $localBin @args
  exit $LASTEXITCODE
}

& npx --yes ruvector @args
exit $LASTEXITCODE
```

**Behavior:** Execute RuVector command with arguments

**Preference:**
1. Local installation (line 44-46)
2. npx download (line 49-50)

**Argument splatting:** `@args` expands to all arguments

**Exit code propagation:** `exit $LASTEXITCODE`

---

## Local vs npx Execution

### Local Installation

**Path:** `node_modules\.bin\ruvector.cmd` (lines 33, 43)

**Faster:** No network download, immediate execution

**Created by:** `npm install ruvector` in project

### npx Download

**Command:** `npx --yes ruvector` (lines 39, 49)

**Flags:**
- `--yes` - Auto-confirm installation prompts (line 39, 49)

**Slower:** Downloads package on first run (cached thereafter)

**Use case:** No local installation, ad-hoc usage

---

## Error Action Preference

**Evidence:** Line 17

```powershell
$ErrorActionPreference = 'Stop'
```

**Behavior:** Stop execution on any error

**Effect:** Script exits immediately on:
- Directory creation failure
- Path resolution failure
- Command not found errors

**Alternative:** `'Continue'` (default) - Continue on non-terminating errors

---

## Exit Code Handling

**Evidence:** Lines 36, 40, 46, 50

```powershell
exit $LASTEXITCODE
```

**Purpose:** Propagate RuVector command exit code to caller

**LASTEXITCODE:** Automatic variable containing last external command's exit code

**Examples:**
- `ruvector create` succeeds → `$LASTEXITCODE = 0` → Script exits 0
- `ruvector search` fails → `$LASTEXITCODE = 1` → Script exits 1

---

## Comparison with ruvector.sh

### Similarities
- Both force local npm cache/prefix
- Both prefer local installation over npx
- Both handle no-argument case (show help)
- Both propagate exit codes

### Differences

| Feature | ruvector.ps1 (Windows) | ruvector.sh (Linux/macOS) |
|---------|------------------------|----------------------------|
| **Language** | PowerShell | Bash |
| **Shebang** | None (PowerShell) | `#!/usr/bin/env bash` |
| **Path resolution** | `Resolve-Path`, `Join-Path` | `cd`, `pwd` |
| **Directory creation** | `New-Item -Force` | `mkdir -p` |
| **Command check** | `Get-Command npx` | `command -v npx` |
| **Argument forwarding** | `@args` | `"$@"` |
| **Exit code** | `$LASTEXITCODE` | `$?` |
| **Local bin path** | `node_modules\.bin\ruvector.cmd` | `node_modules/.bin/ruvector` |

---

## RuVector CLI Commands

### create

**Usage:** `.\scripts\ruvector.ps1 create <db-path>`

**Creates:** SQLite database for vector storage

**Example:**
```powershell
.\scripts\ruvector.ps1 create ./data/ruvector.db
```

### insert

**Usage:** `.\scripts\ruvector.ps1 insert <db-path> <json-file>`

**Inserts:** Vectors from JSON file into database

**JSON format:**
```json
[
  {"id": 1, "vector": [0.1, 0.2, 0.3]},
  {"id": 2, "vector": [0.4, 0.5, 0.6]}
]
```

### search

**Usage:** `.\scripts\ruvector.ps1 search <db-path> --vector "[x,y,z]" --top-k N`

**Searches:** Similar vectors using cosine similarity

**Example:**
```powershell
.\scripts\ruvector.ps1 search ./data/ruvector.db --vector "[1,0,0]" --top-k 5
```

### delete

**Usage:** `.\scripts\ruvector.ps1 delete <db-path> --id <id>`

**Deletes:** Vector by ID

**Example:**
```powershell
.\scripts\ruvector.ps1 delete ./data/ruvector.db --id 42
```

### list

**Usage:** `.\scripts\ruvector.ps1 list <db-path>`

**Lists:** All vectors in database

**Example:**
```powershell
.\scripts\ruvector.ps1 list ./data/ruvector.db
```

---

## Troubleshooting

### npx Not Found

**Symptoms:** "npx not found" exception (line 29)

**Fix:**
```powershell
# Install Node.js (includes npm and npx)
# Download from https://nodejs.org/

# Verify installation
node --version
npm --version
npx --version

# Add to PATH if needed
$env:PATH += ";C:\Program Files\nodejs"
```

### Network Drive Error

**Symptoms:** ENOENT errors, "drive not found"

**This script solves this issue** by forcing local cache/prefix (lines 25-26)

**Manual fix (if script doesn't work):**
```powershell
# Check npm config
npm config get prefix
npm config get cache

# Clear global config
npm config delete prefix
npm config delete cache

# Use script
.\scripts\ruvector.ps1 --version
```

### Permission Denied

**Symptoms:** Cannot create directories in `.npm-cache/` or `.npm-prefix/`

**Fix:**
```powershell
# Run as Administrator
Start-Process powershell -Verb RunAs

# Or change repository permissions
icacls . /grant ${env:USERNAME}:F /T
```

### PowerShell Execution Policy

**Symptoms:** "Cannot be loaded because running scripts is disabled"

**Fix:**
```powershell
# Check current policy
Get-ExecutionPolicy

# Set policy for current user
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

# Or bypass for this script only
powershell -ExecutionPolicy Bypass -File .\scripts\ruvector.ps1 --version
```

### RuVector Download Fails

**Symptoms:** npx hangs or fails to download

**Fix:**
```powershell
# Install locally instead
npm install ruvector

# Script will use local installation (lines 44-46)
.\scripts\ruvector.ps1 --version
```

---

## Integration with ARIA Platform

**Use cases:**
1. **Semantic search** - Find similar embeddings in vector database
2. **Document retrieval** - RAG (Retrieval-Augmented Generation) pipelines
3. **Clustering** - Group similar vectors
4. **Anomaly detection** - Find outliers based on vector distance

**Workflow:**
```powershell
# 1. Create database
.\scripts\ruvector.ps1 create ./data/embeddings.db

# 2. Generate embeddings (via Python/Transformers)
python scripts/generate_embeddings.py --output ./vectors.json

# 3. Insert into database
.\scripts\ruvector.ps1 insert ./data/embeddings.db ./vectors.json

# 4. Search for similar documents
.\scripts\ruvector.ps1 search ./data/embeddings.db --vector "[0.8,0.2,0.1]" --top-k 10
```

---

## References

### Source Code
- **Main script:** `scripts/ruvector.ps1` (51 lines)
- **Repository root resolution:** line 19
- **Directory creation:** line 23
- **Environment variables:** lines 25-26
- **npx check:** lines 28-30
- **No arguments logic:** lines 32-41
- **With arguments logic:** lines 43-50

### Related Files
- **Bash version:** `scripts/ruvector.sh`
- **npm cache:** `.npm-cache/` (created by script)
- **npm prefix:** `.npm-prefix/` (created by script)

### External Resources
- [RuVector NPM Package](https://www.npmjs.com/package/ruvector)
- [PowerShell Documentation](https://learn.microsoft.com/en-us/powershell/)
- [npm Configuration](https://docs.npmjs.com/cli/v10/using-npm/config)
- [npx Documentation](https://docs.npmjs.com/cli/v10/commands/npx)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 44/60 contracts complete (73.3%)
