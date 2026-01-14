# Self-Hosted Runner Contract (Snapshot Replay Lane)

This document defines the **minimum contract** for any self-hosted GitHub Actions runner that is allowed to execute snapshot replay tests for this repository.

> Why this exists
>
> Snapshot replay jobs are intentionally strict: they require Docker, Git LFS objects, and sufficient disk to unpack archives and run compose validation without flaking.

## Scope

This contract applies to runners intended to execute the following categories of workflows:

- Snapshot replay workflows (extract archives under `archives/**`, validate, replay)
- Bundle validation workflows (requires LFS objects)
- Any workflow requiring `docker` / `docker compose`

This is **not** the same as the Windows+WSL2 bootstrap runner described in `.github/docs/self-hosted-runner-setup.md`.

## Required runner labels

Runners must be registered with the following labels:

- `self-hosted`
- `linux`
- `docker`

Workflows targeting this lane will use:

- `runs-on: [self-hosted, linux, docker]`

## Operating system

- **Required**: Linux (modern distro; examples: Ubuntu 22.04/24.04, Debian 12)
- **Recommended**: Ubuntu LTS

Rationale: GitHub-hosted replay uses `ubuntu-latest`; matching Linux behavior reduces drift.

## Disk and storage

- **Enforced minimum free disk**: **50 GB**
- **Recommended free disk**: **100 GB**

Disk usage drivers:
- Docker images/layers pulled during replay
- Extracted snapshot archives (`*.tar.gz`)
- Git LFS object cache

### Recommended filesystems/paths

- Runner work directory: fast local SSD
- Docker data root: SSD-backed (default is fine if SSD)
- Git LFS storage: allow default (`.git/lfs/objects`) unless you have a centralized cache strategy

## Docker requirements

### Docker Engine

- Docker Engine installed and running
- Runner user can talk to the Docker daemon

### Docker Compose

- **Required**: Compose v2 available as `docker compose`
  - (`docker-compose` v1 may be present, but workflows assume v2 first)

### Permissions

- The runner service account must be able to access the Docker socket.
- Common pattern: add the runner user to the `docker` group.

> Note: this is a security-sensitive decision. Only use self-hosted runners for trusted repos.

## Git + Git LFS requirements

- `git` installed
- `git-lfs` installed
- LFS initialized on the machine (`git lfs install`)

### Hard-fail policy

Replay workflows will **fail** if required LFS objects are not present (pointer-only checkouts are not acceptable).

## Network requirements

The runner must be able to:

- Fetch repository content from GitHub
- Fetch Git LFS objects from GitHub
- Pull container images required for replay (unless images are pre-seeded locally)

## Preflight checklist (what CI will validate)

A runner is considered compliant if it passes the following checks:

1. **Disk free** (≥ 50 GB)
2. **Docker engine reachable**
3. **Compose available** (`docker compose version`)
4. **Git LFS available** (`git lfs version`)
5. **LFS objects present** (workflows will run `git lfs fetch`/`git lfs pull` + `git lfs fsck`)

## Suggested local verification commands

Run these directly on the runner host:

- Disk:
  - `df -h`
- Docker:
  - `docker --version`
  - `docker info`
  - `docker compose version`
  - `docker run --rm hello-world`
- Git LFS:
  - `git lfs version`
  - `git lfs env`

## Maintenance and hygiene

- Periodically prune Docker to reclaim space:
  - `docker system prune -af --volumes` (use with care)
- If LFS storage grows too large:
  - Prefer increasing disk and/or using a dedicated runner for artifact-heavy jobs
  - Avoid deleting `.git/lfs/objects` unless you understand the impact; workflows may re-download

## Security considerations

Self-hosted runners execute repository code. Recommended practices:

- Restrict repo write access
- Use network isolation (VLAN / firewall) for the runner
- Rotate runner tokens as needed
- Treat runner hosts as production-grade assets

