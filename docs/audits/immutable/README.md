# Immutable Evidence Store

This folder contains **immutable evidence snapshots** used for replayable audit verification.

It is intentionally structured so we can:
- keep a small number of *expanded* snapshots directly in Git,
- keep older snapshots as monthly **rollups** (`.tar.gz`) tracked via **Git LFS**, and
- drive MkDocs pages and CI selection logic from a single authoritative index file.

## Authoritative index

- `docs/audits/immutable/index.yml` is the authoritative mapping of snapshot entries.
- CI and documentation should treat `index.yml` as the source of truth.

## Storage layout

- Expanded snapshots (keep newest N=10):
  - `docs/audits/immutable/expanded/<YYYY-MM-DD>/<ref>/...`

- Monthly rollups (tracked in Git LFS):
  - `docs/audits/immutable/rollups/<YYYY-MM>/append/immutable-rollup-<YYYY-MM>-<seq>.tar.gz`
  - `docs/audits/immutable/rollups/<YYYY-MM>/canonical/immutable-rollup-<YYYY-MM>.tar.gz`

Where `<ref>` is `ripple-vX.Y.Z` if tagged, else `sha-<12>`.

## Milestone-only updates

Per the consolidation plan, updates here should be committed to `main` only when a milestone occurs (tagging, bundle slot change, snapshot hash change, or canonical rollup rewrite).
