# Emergency Procedures

Immediate response procedures for security incidents, service outages, and system failures.

## What's in the repo (evidence)

- WSL crash recovery: `scripts/session-restore.sh`, `scripts/wsl-cleanup.sh`
- Service health validation: `scripts/validate-e2e.sh`
- Resource validation: `scripts/validate-resources.sh`
- Bootstrap procedures: `docs/RUNBOOKS.md`
- Observability: `docs/OBSERVABILITY-QUICK-START.md`

## Goal

1. Provide immediate response actions for common emergencies
2. Minimize downtime during incidents
3. Preserve evidence for post-incident analysis
4. Enable quick rollback to last known-good state

## Quick start

### Security Incident: API Key Compromised

**Immediate Actions (Within 1 Hour)**:

```bash
# 1. Rotate compromised credential immediately
# See docs/cookbooks/SECRET_ROTATION.md for detailed procedures

# 2. Check logs for unauthorized usage
docker compose logs | grep "401\|403\|unauthorized"

# 3. Review recent access patterns
# If using Prometheus/Grafana, check dashboards

# 4. If Step-CA certificate compromised
./scripts/rotate-certs.sh --force

# 5. Force restart all services with new credentials
docker compose down
docker compose up -d
```

**Follow-up** (see `docs/cookbooks/SECRET_ROTATION.md` for full checklist)

**Evidence**: `scripts/rotate-certs.sh` exists for cert rotation

### Service Outage: Critical Service Down

```bash
# 1. Identify which service is down
docker compose ps

# 2. Check service logs
docker compose logs <service-name> --tail=100

# 3. Restart the service
docker compose restart <service-name>

# 4. If restart fails, check dependencies
docker compose ps | grep -E "vault|keycloak|nats"

# 5. If dependency issue, restart in order
docker compose restart vault
docker compose restart keycloak  # Wait for Vault healthy
docker compose restart nats      # Wait for Vault healthy

# 6. Verify health
./scripts/validate-e2e.sh
```

**Service Startup Order** (from docker-compose dependencies):
1. Vault (foundational secrets)
2. Keycloak (authentication)
3. NATS (messaging)
4. All other services

**Evidence**: `docs/CONTAINER_SECURITY.md` documents service dependencies

### WSL Crash: System Unresponsive

**Symptoms**: WSL hangs, commands timeout, Nix build fails

```bash
# FROM WINDOWS POWERSHELL (outside WSL):

# 1. Force shutdown WSL
wsl --shutdown

# 2. Check disk usage (common cause)
wsl --list --verbose

# 3. Restart WSL
wsl -d NixOS-Ripple

# INSIDE WSL:

# 4. Restore session state
cd ~/ripple-env
./scripts/session-restore.sh

# 5. Clean up if disk full
./scripts/wsl-cleanup.sh

# 6. Verify environment
nix develop --command echo "Shell works"
pixi run python --version
```

**Common Causes**:
- Disk space exhaustion (run `wsl-cleanup.sh`)
- Memory pressure (check `.wslconfig` settings)
- Nix build parallelization (limited to 2 cores in `stable-env.sh`)

**Evidence**: `scripts/wsl-cleanup.sh`, `scripts/session-restore.sh`, `scripts/stable-env.sh` exist

### Build Failure: Nix or Pixi Won't Install

```bash
# 1. Check available disk space
df -h

# 2. Clean Nix store if needed
nix-collect-garbage -d

# 3. Clean Pixi cache
pixi clean
rm -rf .pixi

# 4. Reset state and retry
rm -rf .pixi
pixi install --frozen

# 5. If Nix flake check fails
nix flake check --show-trace

# 6. Restore lockfiles if corrupted
git restore flake.lock pixi.lock
```

**Evidence**: Standard Nix/Pixi troubleshooting patterns

### Network Failure: Docker Network Conflicts

```bash
# 1. Check for subnet collisions
docker network ls
docker network inspect <network-name>

# 2. Remove conflicting networks
docker compose down
docker network prune -f

# 3. Recreate networks
docker compose up -d

# 4. Verify connectivity
docker compose exec <service> ping -c 1 <other-service>

# 5. Check firewall rules
# Linux: sudo iptables -L
# WSL: Windows Firewall settings
```

**Evidence**: Docker network topology in `docs/CONTAINER_SECURITY.md`

### Authentication Failure: Vault/Keycloak Not Starting

**Vault Unsealing**:

```bash
# 1. Check Vault status
docker compose logs vault | tail -50

# 2. If sealed, unseal manually (production requires operator keys)
export VAULT_ADDR=http://localhost:8200
vault operator unseal <key1>
vault operator unseal <key2>
vault operator unseal <key3>

# 3. Verify unsealed
vault status
```

**Keycloak Issues**:

```bash
# 1. Check Keycloak logs
docker compose logs keycloak | tail -50

# 2. Verify database connectivity
docker compose exec keycloak pg_isready -h postgres

# 3. If database issue, restart PostgreSQL first
docker compose restart postgres
sleep 10
docker compose restart keycloak
```

**Evidence**: Vault and Keycloak deployed in `docker/docker-compose.identity.yml`

### Data Loss: Accidental File Deletion

**CURRENT STATE**: No automated backup procedures exist

**Manual Recovery Options**:

```bash
# 1. Check if file is in Git history
git log --all --full-history -- path/to/file
git restore --source=<commit> path/to/file

# 2. Check if .env file was backed up
ls -la .env.backup .env.*

# 3. Restore from manual backup if exists
cp /path/to/backup/.env .env
cp -r /path/to/backup/docker/data ./docker/

# 4. If no backup, reinitialize from bootstrap
./bootstrap.sh --clean
# Then restore .env manually
```

**UNKNOWN**: Production-grade automated backup strategy
**Recommendation**: See `docs/cookbooks/BACKUP_RESTORE.md` for manual backup procedures

### Escalation Contacts

**UNKNOWN**: Escalation contacts not documented in repository

**Recommendation**: Create `docs/ON_CALL.md` with:
- Primary contact information
- Secondary/backup contacts
- External vendor support contacts
- Escalation matrix by severity level

## Post-Incident Checklist

After resolving any emergency:

- [ ] Document what happened in incident log
- [ ] Update runbooks with lessons learned
- [ ] Run full validation: `./scripts/validate-e2e.sh`
- [ ] Check monitoring dashboards for ongoing issues
- [ ] Schedule post-mortem meeting (if major incident)
- [ ] Update `docs/TROUBLESHOOTING.md` with new solutions

## Prevention

Regular maintenance to prevent emergencies:

```bash
# Weekly
- Update dependencies: see docs/cookbooks/DEPENDENCY_UPDATES.md
- Check disk space: df -h
- Review logs: docker compose logs --since 7d | grep ERROR

# Monthly
- Rotate secrets: see docs/cookbooks/SECRET_ROTATION.md
- Update Nix/Pixi: nix flake update && pixi update
- Test backup restoration: see docs/cookbooks/BACKUP_RESTORE.md

# Quarterly
- Security audit: ./scripts/security-audit.sh
- Disaster recovery drill
- Update escalation contacts
```

**Evidence**: Various maintenance scripts exist in `scripts/`

## Related docs

- [Secret Rotation](SECRET_ROTATION.md) - Credential rotation procedures
- [Backup & Restore](BACKUP_RESTORE.md) - Disaster recovery procedures
- [Troubleshooting](../TROUBLESHOOTING.md) - Common issues and solutions
- [Runbooks](../RUNBOOKS.md) - Operational procedures
- [Observability](../OBSERVABILITY-QUICK-START.md) - Monitoring and alerting
