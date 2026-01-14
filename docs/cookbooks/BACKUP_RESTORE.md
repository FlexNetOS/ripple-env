# Backup & Restore Procedures

Manual backup and disaster recovery procedures for critical data and configuration.

## What's in the repo (evidence)

**CURRENT STATE**: No automated backup scripts found in `scripts/` directory

**Critical data locations**:
- Environment variables: `.env`, `.env.agixt.example`
- Encrypted secrets: `secrets/secrets.nix`, `secrets/*.age`
- Lock files: `flake.lock`, `pixi.lock`
- Docker data volumes: `docker/data/`
- Configuration: `docker-compose.*.yml`, `pixi.toml`, `flake.nix`

## Goal

1. Document what needs to be backed up
2. Provide manual backup procedures until automation exists
3. Enable disaster recovery from total system loss
4. Minimize data loss window (RPO) and recovery time (RTO)

## Quick start

### What to Backup

**Priority 1: Cannot be Regenerated**

| Data | Location | Frequency | Backup Method |
|------|----------|-----------|---------------|
| Environment variables | `.env` | After changes | Git-ignored, external backup |
| Encrypted secrets | `secrets/*.age` | After rotation | Git (already tracked) |
| SSH keys | `secrets/secrets.nix` | After key changes | Git (already tracked) |
| Custom configs | Modified `.yml` files | After changes | Git diff check |

**Priority 2: Expensive to Regenerate**

| Data | Location | Frequency | Backup Method |
|------|----------|-----------|---------------|
| LocalAI models | `docker/data/localai/models/` | After downloads | Git LFS or external |
| Database data | `docker/data/postgres/`, `docker/data/neo4j/` | Daily | Volume backup |
| Build cache | `.pixi/`, `nix/store/` | Weekly | Optional (can rebuild) |
| Logs | `docker/data/*/logs/` | Weekly | Archive + rotate |

**Priority 3: Can be Regenerated**

| Data | Location | Note |
|------|----------|------|
| Lock files | `flake.lock`, `pixi.lock` | Backup recommended but can regenerate |
| Docker images | Docker cache | Can be rebuilt from compose files |
| Nix store | `/nix/store/` | Can be rebuilt (slow) |

**Evidence**: These locations verified in repository structure

### Manual Backup Procedure

```bash
# 1. Create backup directory with timestamp
BACKUP_DIR="$HOME/backups/ripple-env-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"

# 2. Backup environment files (NEVER commit these)
cp .env "$BACKUP_DIR/env.backup"
cp .env.agixt.example "$BACKUP_DIR/" 2>/dev/null || true

# 3. Backup encrypted secrets (already in Git, but backup anyway)
cp -r secrets/ "$BACKUP_DIR/"

# 4. Backup lock files
cp flake.lock pixi.lock "$BACKUP_DIR/"

# 5. Backup docker configurations
cp docker-compose.*.yml "$BACKUP_DIR/"
cp -r docker/data/ "$BACKUP_DIR/docker-data/" 2>/dev/null || true

# 6. Create backup manifest
cat > "$BACKUP_DIR/MANIFEST.txt" << EOF
Backup created: $(date)
Repository: $(git remote get-url origin)
Branch: $(git branch --show-current)
Commit: $(git rev-parse HEAD)
Hostname: $(hostname)
EOF

# 7. Compress backup
cd "$(dirname "$BACKUP_DIR")"
tar -czf "$(basename "$BACKUP_DIR").tar.gz" "$(basename "$BACKUP_DIR")"

# 8. Verify backup
tar -tzf "$(basename "$BACKUP_DIR").tar.gz" | head -20

echo "Backup saved to: $BACKUP_DIR.tar.gz"
```

**Output**: Compressed backup with timestamp

### Backup to External Storage

```bash
# Copy to external drive (example)
cp "$BACKUP_DIR.tar.gz" /mnt/backup-drive/

# Or upload to cloud storage (example - requires setup)
# aws s3 cp "$BACKUP_DIR.tar.gz" s3://my-bucket/ripple-env-backups/
# rclone copy "$BACKUP_DIR.tar.gz" remote:backups/
```

**UNKNOWN**: Production cloud backup strategy not implemented
**Recommendation**: Configure rclone or cloud-specific backup tool

### Restore Procedure

**Scenario**: Complete system loss, starting from scratch

```bash
# 1. Bootstrap fresh system (if needed)
# See docs/RUNBOOKS.md for full bootstrap procedures

# 2. Clone repository
git clone <repo-url> ripple-env
cd ripple-env

# 3. Extract backup
BACKUP_FILE="ripple-env-YYYYMMDD-HHMMSS.tar.gz"
tar -xzf ~/$BACKUP_FILE

# 4. Restore environment variables
cp ripple-env-YYYYMMDD-HHMMSS/env.backup .env

# 5. Restore lock files
cp ripple-env-YYYYMMDD-HHMMSS/flake.lock .
cp ripple-env-YYYYMMDD-HHMMSS/pixi.lock .

# 6. Restore docker data volumes
cp -r ripple-env-YYYYMMDD-HHMMSS/docker-data/* docker/data/

# 7. Restore secrets (if not in Git)
cp -r ripple-env-YYYYMMDD-HHMMSS/secrets/* secrets/

# 8. Install dependencies (using backed-up lock files)
pixi install --frozen

# 9. Enter Nix development shell
nix develop

# 10. Start services
docker compose up -d

# 11. Verify restoration
./scripts/validate-e2e.sh
```

**Expected RTO**: 30-60 minutes (depending on network speed)
**Evidence**: Bootstrap procedures documented in `docs/RUNBOOKS.md`

### Disaster Recovery Testing

Test restoration quarterly to ensure backups work:

```bash
# 1. Perform backup (see above)

# 2. In a separate directory, simulate disaster
mkdir ~/disaster-recovery-test
cd ~/disaster-recovery-test

# 3. Follow restore procedure

# 4. Verify all services start
docker compose ps

# 5. Run validation
./scripts/validate-e2e.sh

# 6. Document any issues found
```

**Frequency**: Quarterly
**Purpose**: Verify backups are complete and restore procedure works

### Database Backup (Postgres, Neo4j)

For production databases with important data:

```bash
# PostgreSQL backup
docker compose exec postgres pg_dump -U postgres flexnetos > backup-postgres.sql

# Neo4j backup (requires neo4j-admin)
docker compose exec neo4j neo4j-admin dump --to=/backup/neo4j-backup.dump

# Copy backup out of container
docker cp <container-id>:/backup/neo4j-backup.dump ./
```

**UNKNOWN**: Automated database backup scheduling
**Recommendation**: Use cron job or CI/CD to schedule daily backups

### Incremental Backup (Advanced)

For large docker data volumes, use rsync for incremental backups:

```bash
# First backup (full)
rsync -av --delete docker/data/ /mnt/backup-drive/ripple-env-data/

# Subsequent backups (only changes)
rsync -av --delete --link-dest=/mnt/backup-drive/ripple-env-data/ \
  docker/data/ /mnt/backup-drive/ripple-env-data-$(date +%Y%m%d)/
```

**Evidence**: Standard rsync practices (not repo-specific)

### Backup Verification Checklist

After each backup:

- [ ] Verify `.env` file is included (NOT in Git)
- [ ] Verify encrypted secrets are included
- [ ] Verify lock files are included
- [ ] Verify docker data volumes are included (if critical data exists)
- [ ] Verify backup is compressed and stored externally
- [ ] Test extraction: `tar -tzf backup.tar.gz | head`
- [ ] Document backup location and access method

### Retention Policy (Recommendation)

**UNKNOWN**: Official retention policy not defined

**Suggested policy**:
- **Daily backups**: Keep for 7 days
- **Weekly backups**: Keep for 4 weeks
- **Monthly backups**: Keep for 12 months
- **Quarterly backups**: Keep for 7 years (if required for compliance)

### Backup Automation (Future Work)

**UNKNOWN**: No automated backup scripts in repository

**Recommendation**: Create `scripts/backup.sh` with:
```bash
#!/usr/bin/env bash
# Automated backup script (FUTURE WORK)

# Features to implement:
# - Scheduled via cron (daily 2 AM)
# - Automatic rotation (keep last 7 daily, 4 weekly, 12 monthly)
# - Upload to cloud storage (S3/GCS/Azure)
# - Notification on failure (email/Slack)
# - Encryption of backup archives
# - Backup verification
```

**Priority**: HIGH (critical for production deployments)

## Security Considerations

1. **Encrypt backups** if storing externally:
   ```bash
   # Encrypt backup before upload
   gpg --encrypt --recipient your@email.com backup.tar.gz
   ```

2. **Never backup to public locations**
3. **Restrict backup file permissions**:
   ```bash
   chmod 600 backup.tar.gz
   ```

4. **Rotate encryption keys** quarterly

## Related docs

- [Emergency Procedures](EMERGENCY_PROCEDURES.md) - Incident response for data loss
- [Secrets Management](../SECRETS.md) - Agenix encrypted secrets
- [Runbooks](../RUNBOOKS.md) - Bootstrap and restore procedures
- [Docker Profiles](../DOCKER_PROFILES.md) - Volume management
