# Documentation Changelog

This file tracks major documentation changes, moves, and merges following the audit prompt's requirement for traceability.

**Format:** YYYY-MM-DD entries with evidence paths

---

## 2026-01-14 — Cross-Reference Sync (Post File Move)

### Fixed Broken References

After file reorganization (NIX_FLAKE_MODULARIZATION.md moved to `nix/` subdirectory), the following cross-references were updated:

| File | Old Reference | New Reference |
|------|---------------|---------------|
| `docs/index.md` | `deployment/index.md` | `getting-started/deployment/index.md` |
| `docs/README.md` | `NIX_FLAKE_MODULARIZATION.md` | `nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/SUPPLY_CHAIN_SECURITY.md` | `NIX_FLAKE_MODULARIZATION.md` | `nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/architecture/nix-flake.md` | `"NIX_FLAKE_MODULARIZATION.md"` | `"nix/NIX_FLAKE_MODULARIZATION.md"` |
| `docs/nix/NIXOS_IMAGES.md` | `NIX_FLAKE_MODULARIZATION.md` | `./NIX_FLAKE_MODULARIZATION.md` |
| `docs/getting-started/VIDEO_WALKTHROUGHS.md` | `../NIX_FLAKE_MODULARIZATION.md` | `../nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/security/SUPPLY_CHAIN_SECURITY.md` | `../NIX_FLAKE_MODULARIZATION.md` | `../nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/BUILDKIT_STARTER_SPEC.md` | `docs/NIX_FLAKE_MODULARIZATION.md` | `docs/nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/audits/workflows/JOB_SUMMARY_ENHANCEMENT_AUDIT.md` | `docs/NIX_FLAKE_MODULARIZATION.md` (2 occurrences) | `docs/nix/NIX_FLAKE_MODULARIZATION.md` |
| `docs/scripts/benchmark-eval.sh.md` | `NIX_FLAKE_MODULARIZATION.md` (2 occurrences) | `docs/nix/NIX_FLAKE_MODULARIZATION.md` |

**Total: 12 broken references fixed across 10 files**

---

## 2026-01-14 — Full Documentation Audit (Phases 1-8)

### Added

#### Module Documentation (`docs/modules/`)
| File | Purpose | Evidence |
|------|---------|----------|
| `modules/INDEX.md` | Module navigation hub | New file |
| `modules/environments.md` | Environment taxonomy | From pixi.toml, flake.nix |
| `modules/toolchain.md` | Package management | From pixi.toml, flake.nix, package.json |
| `modules/ci_cd.md` | CI/CD documentation | From .github/workflows/ |
| `modules/secrets.md` | Secrets management | From config/vault/, secrets/secrets.nix |
| `modules/providers.md` | Provider integrations | From docker-compose files, config/ |
| `modules/observability.md` | Observability stack | From docker-compose.observability.yml |
| `modules/smoke_tests.md` | Smoke test procedures | From scripts/verify-*.sh, scripts/validate-*.sh |

#### Mermaid Graphs (`docs/graphs/`)
| File | Purpose | Evidence |
|------|---------|----------|
| `graphs/env_layering.mmd` | Environment configuration precedence | From flake.nix, pixi.toml, .envrc |

#### Updated

| File | Changes | Evidence |
|------|---------|----------|
| `docs/README.md` | Added Navigation Hub, Module Documentation, Mermaid Graphs sections | Audit prompt requirement |
| `docs/CHANGELOG_DOCS.md` | Created for audit trail | This file |

### Documentation Inventory (from audit)

| Category | Count | Status |
|----------|-------|--------|
| Entrypoints discovered | 180+ | Complete |
| Script contracts | 60+ | Complete |
| Module docs | 8 | Complete |
| Mermaid graphs | 9 | Complete |
| Cookbooks | 9 | Complete |
| ADRs | 6 | Complete |
| Master docs | 24+ | Complete |

---

## 2026-01-13 — Phase 1-6 Completion

### Added

| File | Purpose |
|------|---------|
| `docs/CATALOG.json` | Machine-readable entrypoint catalog |
| `docs/modules/bootstrap.md` | Golden path documentation |
| `docs/graphs/bootstrap_flow.mmd` | Bootstrap flow diagram |
| `docs/graphs/script_dag.mmd` | Script dependency graph |
| `docs/graphs/ci_flow.mmd` | CI/CD flow diagram |

### Script Contracts Added

60+ script contract documents in `docs/scripts/*.md`:
- Bootstrap: `bootstrap.sh.md`, `bootstrap.ps1.md`
- Deployment: `deploy.sh.md`, `deploy-edge.sh.md`, etc.
- Verification: `verify-*.sh.md` (10 scripts)
- Validation: `validate-*.sh.md` (5 scripts)
- Security: `security-audit.sh.md`, `rotate-certs.sh.md`, etc.
- See `docs/scripts/INDEX.md` for complete list

---

## 2026-01-10 — ARIA Audit Reports

### Added

| File | Purpose |
|------|---------|
| `docs/audits/assessment/ARIA-AUDIT-2026-01-10.md` | Initial audit |
| `docs/audits/assessment/ARIA-AUDIT-2026-01-10-v2.md` | Revised audit |
| `docs/audits/backlog/ARIA-TASK-BACKLOG-2026-01-10.md` | Task backlog |

---

## Directory Structure

Current documentation organization:

```
docs/
├── README.md               # Main index (updated 2026-01-14)
├── CATALOG.json            # Machine-readable catalog
├── CHANGELOG_DOCS.md       # This file
├── GLOSSARY.md             # Terminology
├── QUALITY.md              # Doc standards
├── modules/                # Module documentation
│   ├── INDEX.md            # Module index
│   ├── bootstrap.md        # Golden paths
│   ├── environments.md     # Env taxonomy
│   ├── toolchain.md        # Package mgmt
│   ├── ci_cd.md            # CI/CD
│   ├── secrets.md          # Credentials
│   ├── providers.md        # Integrations
│   ├── observability.md    # Monitoring
│   └── smoke_tests.md      # Verification
├── scripts/                # Script contracts
│   ├── INDEX.md            # Script index
│   └── *.md                # 60+ contracts
├── graphs/                 # Mermaid diagrams
│   ├── bootstrap_flow.mmd
│   ├── ci_flow.mmd
│   ├── script_dag.mmd
│   ├── env_layering.mmd
│   └── ...
├── cookbooks/              # Operational recipes
│   ├── INDEX.md            # Cookbook index
│   └── *.md                # 9 cookbooks
├── audits/                 # Audit reports
│   ├── UNKNOWN_REPORT.md   # Tracked unknowns
│   └── ...
├── getting-started/        # Onboarding docs
├── architecture/           # Architecture docs
├── security/               # Security docs
├── providers/              # Provider docs
├── infrastructure/         # Infrastructure docs
└── api/                    # API docs
```

---

## Redirect Stubs

No files were moved during this audit. All new files are additions.

**Policy:** If a doc is moved in the future, leave a redirect stub at the old location:

```markdown
# [Old Title]

**This document has moved to:** [new/location.md](new/location.md)

Redirecting...
```

---

## Quality Metrics

| Metric | Value | Target |
|--------|-------|--------|
| Total doc files | 200+ | - |
| Script coverage | 100% | 100% |
| Module coverage | 100% | 100% |
| Cookbook count | 9 | 8+ |
| Graph count | 9 | 4+ |
| Unknown resolution | 90% | 90%+ |

---

## Next Audit

**Scheduled:** 2026-04-14 (Quarterly)

**Focus Areas:**
- Verify all file references still valid
- Update version-specific documentation
- Review and close remaining unknowns
- Add any new scripts/modules

---

**Last Updated:** 2026-01-14
