# Configuration Database

SQLite database for tracking and validating project configuration.

## Quick Start

```bash
# Build/rebuild the database
python3 scripts/populate-config-db.py --reset

# Query the database
python3 scripts/query-config-db.py summary
python3 scripts/query-config-db.py validate
python3 scripts/query-config-db.py issues
```

## Files

| File | Description |
|------|-------------|
| `schema.sql` | Database schema definition |
| `config.db` | SQLite database (gitignored) |

## Available Queries

```bash
# Flake inputs
python3 scripts/query-config-db.py inputs
python3 scripts/query-config-db.py inputs-unlocked

# GitHub workflows
python3 scripts/query-config-db.py workflows
python3 scripts/query-config-db.py jobs [filter]
python3 scripts/query-config-db.py secrets
python3 scripts/query-config-db.py secrets-list

# References
python3 scripts/query-config-db.py references
python3 scripts/query-config-db.py capabilities
python3 scripts/query-config-db.py capabilities-total

# Issues & Validation
python3 scripts/query-config-db.py issues
python3 scripts/query-config-db.py issues-errors
python3 scripts/query-config-db.py validate

# Custom SQL
python3 scripts/query-config-db.py sql "SELECT * FROM flake_inputs"
```

## Database Schema

### Tables

- **flake_inputs** - Nix flake input definitions
- **devshells** - DevShell configurations
- **devshell_packages** - Packages per devshell
- **workflows** - GitHub workflow files
- **workflow_jobs** - Jobs within workflows
- **workflow_steps** - Steps within jobs
- **workflow_secrets** - Secrets referenced in workflows
- **reference_sources** - Links from README
- **capability_categories** - API registry categories
- **config_issues** - Detected configuration issues

### Views

- **v_flake_inputs_status** - Inputs with lock status
- **v_workflow_summary** - Workflow overview
- **v_unresolved_issues** - Open issues

## Use Cases

1. **Pre-commit validation** - Check for issues before pushing
2. **CI diagnostics** - Quickly identify what secrets/inputs are needed
3. **Dependency tracking** - See what flake inputs are used where
4. **Reference management** - Query all external references

## Rebuilding

The database is derived from source files and can be rebuilt anytime:

```bash
python3 scripts/populate-config-db.py --reset
```
