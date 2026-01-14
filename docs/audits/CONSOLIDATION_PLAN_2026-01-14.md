# Ripple Environment Consolidation Plan (2026-01-14)

This document compiles the implementation plan and decisions from the planning discussion. It is intentionally explicit so future work is reproducible and does not silently drift.

## Objectives

1. **`ripple-env/` is the canonical repository root.**
   - Everything of value currently located alongside the repo (in the parent folder) must be **physically moved inside** `ripple-env/`.

2. **Avoid regressions / avoid reintroducing deprecated content.**
   - The repo has been heavily updated; we must not wire archived/snapshot content into runtime paths by accident.

3. **Bundles/artifacts are high value and must be included.**
   - Environment bundles solve multi-environment portability and belong at the **repo root**.

## Definitions / Naming (remove confusion)

- Project name: **Ripple**
- Repo (git) name: **`ripple-env`**
- Parent folder name is not authoritative and may change.

### Portable Path Token

All portable reports generated for audits must use the placeholder token:

- `${RIPPLE_ENV_REPO_ROOT}`

Rules:
- Always render paths using **forward slashes**.
- Only apply path normalization to canonical audit outputs under `docs/audits/**`.
- **Do not** normalize runtime caches (e.g., `.aria/`).

## Canonical locations

### Audits (canonical)

- Canonical audit/report storage:
  - `docs/audits/<YYYY-MM-DD>/...`

Sources:
- `aria-audit/` (parent folder) → `docs/audits/<date>/aria-audit/`
- `.aria/` (repo runtime cache) → copy/export relevant outputs into `docs/audits/<date>/aria/`

`.aria/` remains:
- **Runtime cache only** (`ripple-env/.aria/`)
- Not treated as canonical documentation.

### Archives / Snapshots (reference-only)

- Archived mined content lives under:
  - `archives/<source>/<YYYY-MM-DD>/...`

This includes mined snapshots from `flexnetos-comprehensive/` (promote only canonical gaps; archive everything else).

### Bundles (canonical, repo root)

Exactly two bundle slots at repo root:
- `ripple-env.current.bundle`
- `ripple-env.previous.bundle`

Tagging scheme for blessed bundles:
- `ripple-vX.Y.Z`

Bundle manifest at repo root:
- `ripple-bundles.yml` (maps current/previous to sha256 + tag pointers)

### Bare repo mirror artifact

- Compress the bare repo artifact into:
  - `archives/git-mirrors/<YYYY-MM-DD>/ripple-env-bare.git.tar.gz`

## Git LFS policy (size-based)

- **Any file ≥ 25 MB must be tracked by Git LFS.**
- CI must **hard-fail** if required LFS objects are missing (no “pointer-only” green builds).

Because `.gitattributes` cannot express pure size-based rules, we implement:
- Pattern-based LFS tracking for known artifact types (`*.bundle`, rollup `*.tar.gz`, etc.), plus
- A CI check that fails if any committed non-LFS blob is ≥ 25 MB.

## Hash ledgers

We keep three hash ledgers with distinct purposes:

1. `docs/HASHES.txt`
   - **Do not move**. Kept for MkDocs / documentation continuity.

2. `HASHES.full.txt` (repo root)
   - Exhaustive: hashes of **all git-tracked files**.
   - May be noisy; that is acceptable.

3. `HASHES.configs.txt` (repo root)
   - Stable “surface change” signal.
   - Allowlist-based and must **exclude `docs/**`** to avoid churn.

Policy file:
- `hashes.yml` (repo root)
  - Defines ledger meanings and allowlist globs for `HASHES.configs.txt`.

## Immutable evidence (committed to `main`, milestone-triggered)

Immutable evidence is required, but constant development should not create endless file sprawl.

### Storage

- Expanded snapshots:
  - `docs/audits/immutable/expanded/<YYYY-MM-DD>/<ref>/...`
  - `<ref>` is `ripple-vX.Y.Z` if tagged, else `sha-<12>`.

- Index:
  - `docs/audits/immutable/index.yml` (authoritative mapping)

### Retention and rollups

- Keep newest **N = 10** snapshots expanded.
- Rollups are monthly `.tar.gz` archives tracked in LFS.

Two-stage rollup policy:

1) **Latest month: append-only**
- `docs/audits/immutable/rollups/<YYYY-MM>/append/immutable-rollup-<YYYY-MM>-<seq>.tar.gz`
- Never rewrite an append rollup.

2) **Older months: rewrite + consolidate**
- `docs/audits/immutable/rollups/<YYYY-MM>/canonical/immutable-rollup-<YYYY-MM>.tar.gz`
- Canonical rollups can be rewritten during consolidation.
- Older append rollups are retained as **backup artifacts** (non-referenced in `index.yml`).

When consolidating an older month into a canonical rollup, replay **all snapshots in that month**.

### MkDocs linking

MkDocs must still link to immutable snapshots:
- Expanded snapshots can be linked directly.
- Rolled-up snapshots must be linked via the rollup artifact + extraction instructions.
- We will generate/maintain an MkDocs page from `index.yml` listing snapshots and where they live.

### Narrow milestone triggers

Immutable evidence is committed to `main` only on these milestones:
- Snapshot recorded hashes change (in snapshot manifest YAML)
- Bundle `current/previous` slots change
- Tag `ripple-vX.Y.Z` created
- Monthly older-month canonical rollup rewrite

## Snapshot replay CI

Replay tests must run only for snapshots whose **recorded digests changed**.

Key requirements:
- Replay runs on both:
  - Self-hosted (eventual primary gate)
  - GitHub-hosted (non-blocking backup until self-hosted stabilizes)
- Both runners must have Docker available.
- CI must hard-fail if LFS objects are missing.

## Self-hosted runner contract (Docker replay lane)

We will document this explicitly in `docs/ops/`.

Baseline requirements:
- OS: Linux
- Docker Engine + Compose v2 available
- Permissions: runner user can access Docker socket
- Labels: `[self-hosted, linux, docker]`
- Disk: **≥ 50 GB free (minimum)**, **≥ 100 GB free (recommended)**
- git-lfs installed and able to fetch required objects

Note: A separate existing doc covers Windows+WSL2 bootstrap testing (`.github/docs/self-hosted-runner-setup.md`).

## Implementation sequence (high-level)

1. Add documentation and policy files (this plan, runner contract, hashes policy).
2. Add scripts to generate ledgers and to enforce LFS size threshold.
3. Add CI workflows for replay, LFS hard-fail, and snapshot selection by recorded hash changes.
4. Add immutable index + rollup tooling and MkDocs linking.
5. Execute physical migration (audits, bundles, bare repo tar.gz) under manifest control.
6. Run `make test` and `make test-e2e` after each major step.
