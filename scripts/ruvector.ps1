<#
  RuVector launcher (Windows / PowerShell)

  Why this exists:
  Some environments have a global npm prefix/cache pointing to a missing drive
  (e.g. N:\...), which breaks `npx`.

  This wrapper forces repo-local npm cache/prefix so `npx ruvector ...` works.

  Usage examples:
    ./scripts/ruvector.ps1 --version
    ./scripts/ruvector.ps1 create ./data/ruvector.db
    ./scripts/ruvector.ps1 insert ./data/ruvector.db ./vectors.json
    ./scripts/ruvector.ps1 search ./data/ruvector.db --vector "[1,0,0]" --top-k 3
#>

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$npmCache = Join-Path $repoRoot '.npm-cache'
$npmPrefix = Join-Path $repoRoot '.npm-prefix'

New-Item -ItemType Directory -Force -Path $npmCache, $npmPrefix | Out-Null

$env:NPM_CONFIG_CACHE = $npmCache
$env:NPM_CONFIG_PREFIX = $npmPrefix

if (-not (Get-Command npx -ErrorAction SilentlyContinue)) {
  throw "npx not found. Install Node.js (includes npm/npx) and try again."
}

if ($args.Count -eq 0) {
  $localBin = Join-Path $repoRoot 'node_modules\.bin\ruvector.cmd'
  if (Test-Path $localBin) {
    & $localBin --help
    exit $LASTEXITCODE
  }

  & npx --yes ruvector --help
  exit $LASTEXITCODE
}

$localBin = Join-Path $repoRoot 'node_modules\.bin\ruvector.cmd'
if (Test-Path $localBin) {
  & $localBin @args
  exit $LASTEXITCODE
}

& npx --yes ruvector @args
exit $LASTEXITCODE
