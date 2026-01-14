---
title: Immutable Evidence
---

# Immutable Evidence

Immutable evidence snapshots are used for **replayable verification** of the environment.

## Authoritative index

- `docs/audits/immutable/index.yml` is the authoritative mapping.

## Storage layout

- Expanded snapshots (keep newest N=10):
  - `docs/audits/immutable/expanded/<YYYY-MM-DD>/<ref>/...`

- Monthly rollups (tracked in Git LFS):
  - `docs/audits/immutable/rollups/<YYYY-MM>/append/immutable-rollup-<YYYY-MM>-<seq>.tar.gz`
  - `docs/audits/immutable/rollups/<YYYY-MM>/canonical/immutable-rollup-<YYYY-MM>.tar.gz`

## Replay selection

Snapshot replay CI is designed to run only for entries whose recorded integrity fields change.

See: `docs/ops/self-hosted-runner-contract.md`
