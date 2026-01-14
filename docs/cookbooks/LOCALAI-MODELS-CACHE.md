# LocalAI models: fetch + cache + Git LFS

This cookbook explains how to get LocalAI model binaries onto a machine **without committing large files into normal Git history**.

## What’s in the repo (evidence)

- Model location used by this repo: `docker/data/localai/models/`
- Git LFS tracking rules for model binaries: `.gitattributes`
- Fetch/cache scripts:
  - `scripts/fetch-localai-models.ps1` (Windows/PowerShell)
  - `scripts/fetch-localai-models.sh` (Linux/WSL)

## Goal

1. Keep the repo clonable/pushable.
2. Still allow developers to get models reliably.
3. Allow a “sync now” path using **Git LFS** while we stabilize.
4. Provide a local **cache** to avoid repeat downloads.

## Quick start

### Windows (PowerShell)

- Show current paths and basic stats:
  - `./scripts/fetch-localai-models.ps1 -Action status`

- Pull model binaries from Git LFS (downloads into the working tree under `docker/data/localai/models/`):
  - `./scripts/fetch-localai-models.ps1 -Action pull`

- Copy working-tree models into your local cache:
  - `./scripts/fetch-localai-models.ps1 -Action cache`

- Restore models from cache back into the working tree:
  - `./scripts/fetch-localai-models.ps1 -Action restore`

Tip: add `-WhatIf` to preview copy operations.

### Linux/WSL (bash)

- `./scripts/fetch-localai-models.sh status`
- `./scripts/fetch-localai-models.sh pull`
- `./scripts/fetch-localai-models.sh cache`
- `./scripts/fetch-localai-models.sh restore`

## Cache location

The cache directory can be overridden with:

- `LOCALAI_MODELS_CACHE_DIR`

If unset:
- PowerShell defaults to `%LOCALAPPDATA%\FlexNetOS\cache\localai\models` (if available)
- bash defaults to `${XDG_CACHE_HOME:-$HOME/.cache}/flexnetos/localai/models`

## Safety notes

- Partial downloads are intentionally ignored (e.g., `**/*.gguf.partial`). Do not commit partials.
- Git LFS content is still “repo-attached” and may have quota/bandwidth implications.
- Long-term, if we outgrow LFS, the scripts can be extended to download from a verified external artifact store (GitHub Releases/S3/etc.) via a signed/hashed manifest.

## Related docs

- `docs/LOCALAI-MODELS.md` (configuration)
- `docs/NETWORK_REQUIREMENTS.md` (network/offline constraints)
