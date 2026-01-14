# Archives

This directory is reserved for **consolidated, large, archival artifacts** that must live inside the `ripple-env` repository for audit/replay purposes.

## What belongs here

- Git repository mirrors (bare repos) as `.tar.gz`
- Snapshot rollups referenced by `docs/audits/immutable/index.yml`
- Other large immutable artifacts required for replay

## Rules

- Large archives must be tracked via Git LFS (see `.gitattributes`).
- Prefer **immutable, content-addressed naming**.
- Do not commit partial downloads.

See: `docs/audits/CONSOLIDATION_PLAN_2026-01-14.md`
