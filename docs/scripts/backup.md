# backup.sh

## Purpose

Automated backup of ripple-env critical data and configurations with optional cloud storage and automatic retention management.

## Invocation

```bash
./scripts/backup.sh [--local-only] [--verify] [--retention DAYS]
```

## Inputs

### Arguments
- `--local-only` - Only backup locally, skip cloud upload
- `--verify` - Verify backup integrity after creation
- `--retention DAYS` - Keep backups for N days (default: 30)
- `--help` - Show help message

### Environment Variables
- `BACKUP_DIR` - Local backup directory (default: `$HOME/backups/ripple-env`)
- `BACKUP_CLOUD_DEST` - Cloud storage destination (rclone remote:path format)
- `BACKUP_RETENTION_DAYS` - Keep backups for N days (default: 30)
- `BACKUP_NOTIFY_EMAIL` - Email address for failure notifications
- `BACKUP_DOCKER_DATA` - Set to `true` to include docker data volumes (default: false)

### Files Read
- `.env` - Environment variables (NEVER committed to Git)
- `.env.agixt.example` - Example environment file
- `secrets/` - Agenix encrypted secrets directory
- `flake.lock` - Nix flake lock file
- `pixi.lock` - Pixi lock file
- `docker-compose*.yml` - Docker compose configurations
- `docker/data/` - Docker data volumes (optional, if `BACKUP_DOCKER_DATA=true`)

## Outputs

### Files Created
- `$BACKUP_DIR/ripple-env-YYYYMMDD-HHMMSS.tar.gz` - Compressed backup archive
- `$BACKUP_DIR/ripple-env-YYYYMMDD-HHMMSS/MANIFEST.txt` - Backup manifest with metadata

### Archive Contents
- `env.backup` - Backed up .env file
- `secrets/` - Encrypted secrets
- `flake.lock` - Nix dependencies
- `pixi.lock` - Pixi dependencies
- `docker-compose*.yml` - Compose configurations
- `docker-data/` - Docker volumes (if enabled)
- `MANIFEST.txt` - Backup metadata (timestamp, commit, hostname)

### Exit Codes
- `0` - Success (backup created and verified)
- `1` - Warning (backup created but verification failed or cloud upload failed)
- `2` - Critical error (backup failed, missing prerequisites)

### Notifications
- Email notification sent to `BACKUP_NOTIFY_EMAIL` on failure (requires `mail` command)

## Side Effects

### File System
- Creates backup directory if it doesn't exist: `$BACKUP_DIR`
- Creates timestamped backup archives
- Deletes backups older than retention period

### Cloud Storage (if configured)
- Uploads compressed backup to `BACKUP_CLOUD_DEST` using rclone

### Resource Usage
- Disk space: ~10MB-1GB depending on docker data inclusion
- Bandwidth: Upload size if cloud storage enabled
- Time: 10-60 seconds depending on size

## Safety Classification

- **Safety**: safe (read-only operations, no modifications to source files)
- **Idempotent**: yes (can be run multiple times, creates new timestamped backups)

## Idempotency

Script is fully idempotent:
- Each run creates a new timestamped backup
- No conflicts with previous backups
- Safe to run multiple times per day
- Automatic cleanup of old backups based on retention policy

## Dependencies

### Required Tools
- `bash` (4.0+)
- `tar` - Archive creation
- `gzip` - Compression
- `git` - Repository metadata extraction

### Optional Tools
- `rclone` - Cloud storage upload (only if `BACKUP_CLOUD_DEST` set)
- `mail` - Email notifications (only if `BACKUP_NOTIFY_EMAIL` set)

### Services
- None (script runs independently)

## Failure Modes

### MISSING_TOOLS
**Error**: "Missing required tools: tar gzip git"
**Solution**: Install missing tools with `pixi add tar gzip git`

### ENV_NOT_FOUND
**Warning**: ".env not found"
**Impact**: Backup succeeds but .env file not included
**Solution**: Ensure .env file exists in repository root

### RCLONE_NOT_INSTALLED
**Warning**: "rclone not found - cloud backup will be skipped"
**Impact**: Backup succeeds locally but not uploaded to cloud
**Solution**: Install rclone with `pixi add rclone` or run with `--local-only`

### VERIFICATION_FAILED
**Exit Code**: 1
**Impact**: Backup created but integrity check failed
**Solution**: Check backup archive manually with `tar -tzf backup.tar.gz`

### CLOUD_UPLOAD_FAILED
**Exit Code**: 1 (warning)
**Impact**: Backup exists locally but not in cloud
**Solution**: Check `BACKUP_CLOUD_DEST` configuration and rclone setup

### DISK_FULL
**Error**: "No space left on device"
**Solution**: Free up disk space or change `BACKUP_DIR` location

## Examples

### Basic Backup
```bash
./scripts/backup.sh
```
Creates backup with default 30-day retention, attempts cloud upload if configured.

### Local Only Backup with Verification
```bash
./scripts/backup.sh --local-only --verify
```
Creates backup locally and verifies integrity, skips cloud upload.

### Extended Retention
```bash
./scripts/backup.sh --retention 60
```
Keeps backups for 60 days instead of default 30.

### Scheduled Backup (Cron)
```bash
# Add to crontab for daily backups at 2 AM
0 2 * * * /path/to/ripple-env/scripts/backup.sh --local-only
```

### Cloud Backup Configuration
```bash
# Configure rclone remote first
rclone config

# Set environment variable
export BACKUP_CLOUD_DEST="s3:my-bucket/ripple-env-backups"

# Run backup
./scripts/backup.sh
```

### Include Docker Data Volumes
```bash
BACKUP_DOCKER_DATA=true ./scripts/backup.sh
```
Note: This significantly increases backup size and time.

## Configuration

### Recommended Setup

**For Development (Local Only)**:
```bash
# ~/.bashrc or ~/.zshrc
export BACKUP_DIR="$HOME/backups/ripple-env"
export BACKUP_RETENTION_DAYS=7

# Run backup weekly
# crontab: 0 2 * * 1 /path/to/backup.sh --local-only
```

**For Production (Cloud Storage)**:
```bash
# Configure rclone for your cloud provider
rclone config  # Follow prompts

# Set environment variables
export BACKUP_CLOUD_DEST="s3:prod-backups/ripple-env"
export BACKUP_RETENTION_DAYS=30
export BACKUP_NOTIFY_EMAIL="ops@example.com"

# Run backup daily at 2 AM
# crontab: 0 2 * * * /path/to/backup.sh --verify
```

## Restoration

To restore from backup, see `docs/cookbooks/BACKUP_RESTORE.md`:

```bash
# Extract backup
tar -xzf ripple-env-YYYYMMDD-HHMMSS.tar.gz

# Restore files
cd ripple-env-YYYYMMDD-HHMMSS
cp env.backup /path/to/ripple-env/.env
cp flake.lock pixi.lock /path/to/ripple-env/
cp -r secrets /path/to/ripple-env/

# Reinstall dependencies
cd /path/to/ripple-env
pixi install --frozen
```

## References

- `docs/cookbooks/BACKUP_RESTORE.md` - Backup and restoration procedures
- `docs/cookbooks/EMERGENCY_PROCEDURES.md` - Disaster recovery
- `.env.example` - Environment variable template
- `docs/SECRETS.md` - Secrets management documentation
