# Script Contract: validate-configs.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/validate-configs.sh`

---

## Purpose

Validate FlexNetOS/ARIA configurations for syntax correctness. Checks Docker Compose files and Nix flake configuration. Designed for pre-commit hooks and CI/CD pipelines to catch configuration errors early.

---

## Invocation

```bash
./scripts/validate-configs.sh
```

**No arguments or environment variables required.**

---

## Outputs

**Standard Output (success):**
```
Validating FlexNetOS configurations...

Validating docker-compose.yml...
Validating docker-compose.identity.yml...
Validating docker-compose.edge.yml...
Validating docker/docker-compose.observability.yml...
... (all compose files)

Validating Nix configuration...
[Nix flake check output...]

All configurations validated successfully!
```

**Standard Output (failure):**
```
Validating FlexNetOS configurations...

Validating docker-compose.yml...
ERROR: In file './docker-compose.yml', service 'postgres': unsupported key 'invalidkey'

[Error message and exit]
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all configs valid) |
| `1` | Error (invalid Docker Compose syntax, invalid Nix flake) |

---

## Side Effects

**None** - Read-only validation, no file modifications.

---

## Safety Classification

**🟢 SAFE** - Read-only configuration checks, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Validation Checks

### 1. Docker Compose Validation (lines 8-34)

**Evidence:**
```bash
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo "WARNING: Docker Compose not found; skipping compose validation"
fi

if [ ${#COMPOSE[@]} -gt 0 ]; then
    shopt -s nullglob
    files=(
        docker-compose*.yml
        docker/*.yml
        docker/docker-compose*.yml
    )
    shopt -u nullglob

    for file in "${files[@]}"; do
        [ -f "$file" ] || continue
        echo "Validating $file..."
        "${COMPOSE[@]}" -f "$file" config > /dev/null
    done
fi
```

**File patterns:**
- `docker-compose*.yml` - Root-level compose files
- `docker/*.yml` - All YAML files in docker directory
- `docker/docker-compose*.yml` - Compose files in docker subdirectory

**nullglob:** Prevents literal string if no matches (lines 20, 26)

**Validation command:** `docker compose -f <file> config > /dev/null`
- Parses YAML
- Interpolates environment variables
- Checks service definitions
- Validates syntax
- Discards output (only checks exit code)

**Graceful handling:** Skips if Docker Compose unavailable (lines 14-16)

### 2. Nix Flake Validation (lines 36-42)

**Evidence:**
```bash
echo "Validating Nix configuration..."
if command -v nix >/dev/null 2>&1; then
    nix flake check
else
    echo "WARNING: nix not found; skipping 'nix flake check'"
fi
```

**nix flake check:**
- Evaluates flake.nix
- Checks outputs (packages, devShells, nixosConfigurations, etc.)
- Runs internal tests
- Verifies dependencies

**Graceful handling:** Skips if Nix unavailable (lines 40-41)

---

## Docker Compose v2 vs v1

**Evidence:** Lines 9-13

**Preference order:**
1. `docker compose` (v2 plugin)
2. `docker-compose` (v1 standalone)

**Detection:**
```bash
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
fi
```

**Reason:** v2 is current standard, v1 is legacy.

---

## File Discovery

**Evidence:** Lines 20-25

**Glob patterns:**
```bash
shopt -s nullglob
files=(
    docker-compose*.yml
    docker/*.yml
    docker/docker-compose*.yml
)
shopt -u nullglob
```

**nullglob:** If no matches, array is empty (not literal string)

**File check:** `[ -f "$file" ] || continue` (line 30)

**Purpose:** Handles edge cases where pattern matches directories or non-files

---

## Common Validation Errors

### Docker Compose Errors

**Invalid syntax:**
```yaml
services:
  app:
    invalidkey: value  # Unknown key
```
**Error:** `unsupported key 'invalidkey'`

**Missing quotes:**
```yaml
services:
  app:
    environment:
      - KEY=value with spaces  # Missing quotes
```
**Error:** `yaml: line X: could not find expected ':'`

**Invalid port mapping:**
```yaml
services:
  app:
    ports:
      - "abc:80"  # Invalid port number
```
**Error:** `invalid port specification`

**Undefined variable:**
```yaml
services:
  app:
    environment:
      - KEY=${UNDEFINED_VAR}  # Not in environment
```
**Error:** `variable UNDEFINED_VAR is not set` (if strict mode enabled)

### Nix Flake Errors

**Syntax errors:**
```nix
{
  outputs = { self }: {
    packages.default = pkgs.hello  # Undefined pkgs
  };
}
```
**Error:** `undefined variable 'pkgs'`

**Invalid attribute:**
```nix
{
  outputs = { self }: {
    invalid-output = "value";  # Unknown output type
  };
}
```
**Error:** `attribute 'invalid-output' not recognized`

**Dependency issues:**
```nix
{
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nonexistent-branch";
}
```
**Error:** `error: unable to download 'https://api.github.com/repos/NixOS/nixpkgs/...'`

---

## CI/CD Integration

### GitHub Actions

```yaml
- name: Validate Configurations
  run: ./scripts/validate-configs.sh
```

### GitLab CI

```yaml
validate:
  script:
    - ./scripts/validate-configs.sh
```

### Pre-commit Hook

```bash
# .git/hooks/pre-commit
#!/bin/bash
./scripts/validate-configs.sh || {
  echo "Configuration validation failed!"
  exit 1
}
```

---

## Usage Patterns

### Manual Validation

```bash
# Validate all configs
./scripts/validate-configs.sh

# Validate before commit
git add . && ./scripts/validate-configs.sh && git commit
```

### Automated Validation

```bash
# Add to Makefile
validate:
	./scripts/validate-configs.sh

# Run before deploy
make validate && ./scripts/deploy.sh
```

---

## Limitations

**Docker Compose:**
- Requires Docker installed
- Does not check runtime behavior
- Does not validate image availability
- Environment variable interpolation depends on current environment

**Nix:**
- Requires Nix installed
- May download dependencies
- Slow on first run (builds derivations)
- Does not test runtime functionality

**No semantic validation:**
- Checks syntax only
- Does not verify services will work
- Does not check resource availability
- Does not validate network connectivity

---

## Performance

**Docker Compose validation:**
- Fast: < 1 second per file
- Total: 5-15 seconds (10-20 files)

**Nix flake check:**
- First run: 30-300 seconds (downloads/builds)
- Cached: 5-30 seconds
- Depends on flake complexity

**Optimization:**
```bash
# Validate only changed files
git diff --name-only --diff-filter=ACMRTUXB | grep -E '\.yml$' | while read file; do
    docker compose -f "$file" config > /dev/null
done
```

---

## Complementary Scripts

**Related validation:**
- `validate-e2e.sh` - End-to-end integration tests
- `validate-resources.sh` - Resource availability checks
- `security-audit.sh` - Security configuration checks

**Pre-deployment:**
```bash
./scripts/validate-configs.sh
./scripts/security-audit.sh
./scripts/validate-resources.sh
./scripts/deploy.sh
```

---

## References

### Source Code
- **Main script:** `scripts/validate-configs.sh` (45 lines)
- **Compose detection:** lines 8-16
- **Compose validation:** lines 18-34
- **Nix validation:** lines 36-42

### Related Files
- **Compose files:** `docker-compose*.yml`, `docker/*.yml`
- **Nix config:** `flake.nix`, `flake.lock`
- **Validation scripts:** `validate-e2e.sh`, `validate-resources.sh`

### External Resources
- [Docker Compose Specification](https://docs.docker.com/compose/compose-file/)
- [Nix Flakes](https://nixos.wiki/wiki/Flakes)
- [YAML Specification](https://yaml.org/spec/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 34/60 contracts complete
