# Adding immutable evidence snapshots

This repository keeps a small, replayable record of **immutable evidence**.

## 1) Choose storage type

### Expanded snapshot (recommended for newest N=10)

Use `storage.type: expanded` when you want the snapshot contents to be directly accessible without extraction.

- Put the snapshot under:
  - `docs/audits/immutable/expanded/<YYYY-MM-DD>/<ref>/...`

### Rollup member (recommended for older snapshots)

Use `storage.type: rollup` when the snapshot is stored inside a monthly rollup archive tracked in Git LFS.

- Rollup archive path (Git LFS):
  - `docs/audits/immutable/rollups/<YYYY-MM>/append/immutable-rollup-<YYYY-MM>-<seq>.tar.gz`
  - or canonical: `docs/audits/immutable/rollups/<YYYY-MM>/canonical/immutable-rollup-<YYYY-MM>.tar.gz`

- Index fields:
  - `storage.archive_path`: repo-relative path to the `.tar.gz`
  - `storage.member_prefix`: prefix within the tarball that contains the snapshot

## 2) Update the authoritative index

Edit `docs/audits/immutable/index.yml` and add an entry:

- Required:
  - `id`: stable identifier (recommend: `<YYYY-MM-DD>/<ref>`)
  - `date`: `YYYY-MM-DD`
  - `ref`: e.g. `ripple-vX.Y.Z` or `sha-<12>`
  - `storage`: see schema

- Recommended:
  - `integrity.sha256`: sha256 of the archive (rollup) or of a deterministic manifest (expanded)
  - `integrity.hashes_full_sha256` and `integrity.hashes_configs_sha256` matching the generated ledgers

## 3) Validate locally

Run:

- `python scripts/validate-immutable-index.py --strict-paths`

## 4) CI behavior

- On pull requests, `.github/workflows/snapshot-replay.yml` selects only entries whose `storage`/`integrity` changed and runs a replay *preflight*.
- The self-hosted lane is gated behind `vars.ENABLE_SELF_HOSTED_REPLAY == 'true'`.
