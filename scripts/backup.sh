#!/usr/bin/env bash
#
# Automated backup script for ripple-env
#
# This script creates timestamped backups of critical data and configurations
# following the procedures documented in docs/cookbooks/BACKUP_RESTORE.md
#
# Features:
# - Backs up .env files, secrets, lock files, and configurations
# - Creates compressed archives with manifests
# - Supports local and cloud storage destinations
# - Automatic rotation (configurable retention)
# - Verification and integrity checks
#
# Usage:
#   ./scripts/backup.sh [--local-only] [--verify] [--retention DAYS]
#
# Environment Variables:
#   BACKUP_DIR - Local backup directory (default: $HOME/backups/ripple-env)
#   BACKUP_CLOUD_DEST - Cloud storage destination (rclone remote:path)
#   BACKUP_RETENTION_DAYS - Keep backups for N days (default: 30)
#   BACKUP_NOTIFY_EMAIL - Email for failure notifications
#
# Exit Codes:
#   0 - Success
#   1 - Warning (backup created but verification failed)
#   2 - Critical error (backup failed)

set -euo pipefail

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BACKUP_DIR="${BACKUP_DIR:-$HOME/backups/ripple-env}"
BACKUP_RETENTION_DAYS="${BACKUP_RETENTION_DAYS:-30}"
TIMESTAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP_NAME="ripple-env-${TIMESTAMP}"
LOCAL_BACKUP_PATH="$BACKUP_DIR/$BACKUP_NAME"

# Flags
LOCAL_ONLY=false
VERIFY=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $*"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*" >&2
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

usage() {
    cat << EOF
Usage: $0 [OPTIONS]

Automated backup of ripple-env critical data and configurations.

OPTIONS:
    --local-only        Only backup locally, skip cloud upload
    --verify            Verify backup integrity after creation
    --retention DAYS    Keep backups for N days (default: $BACKUP_RETENTION_DAYS)
    --help              Show this help message

EXAMPLES:
    # Basic backup
    $0

    # Local only backup with verification
    $0 --local-only --verify

    # Keep last 60 days of backups
    $0 --retention 60

CONFIGURATION:
    BACKUP_DIR              Local backup directory (current: $BACKUP_DIR)
    BACKUP_CLOUD_DEST       Cloud destination (rclone remote:path)
    BACKUP_RETENTION_DAYS   Retention period in days (current: $BACKUP_RETENTION_DAYS)
    BACKUP_NOTIFY_EMAIL     Email for failure notifications

See docs/cookbooks/BACKUP_RESTORE.md for more information.
EOF
}

check_prerequisites() {
    log_info "Checking prerequisites..."

    local missing=()

    # Required tools
    command -v tar >/dev/null 2>&1 || missing+=("tar")
    command -v gzip >/dev/null 2>&1 || missing+=("gzip")
    command -v git >/dev/null 2>&1 || missing+=("git")

    # Optional tools (warn if missing)
    if ! command -v rclone >/dev/null 2>&1 && [[ "$LOCAL_ONLY" == "false" ]]; then
        log_warn "rclone not found - cloud backup will be skipped"
        LOCAL_ONLY=true
    fi

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required tools: ${missing[*]}"
        log_error "Install with: pixi add ${missing[*]}"
        return 2
    fi

    # Check repo is clean
    cd "$REPO_ROOT"
    if ! git diff --quiet 2>/dev/null; then
        log_warn "Repository has uncommitted changes"
    fi

    return 0
}

create_backup_directory() {
    log_info "Creating backup directory: $LOCAL_BACKUP_PATH"
    mkdir -p "$LOCAL_BACKUP_PATH"
}

backup_environment_files() {
    log_info "Backing up environment files..."

    # Backup .env files (NEVER commit these to Git)
    if [[ -f "$REPO_ROOT/.env" ]]; then
        cp "$REPO_ROOT/.env" "$LOCAL_BACKUP_PATH/env.backup"
        log_info "  ✓ .env"
    else
        log_warn "  ⚠ .env not found"
    fi

    if [[ -f "$REPO_ROOT/.env.agixt.example" ]]; then
        cp "$REPO_ROOT/.env.agixt.example" "$LOCAL_BACKUP_PATH/"
        log_info "  ✓ .env.agixt.example"
    fi
}

backup_secrets() {
    log_info "Backing up encrypted secrets..."

    if [[ -d "$REPO_ROOT/secrets" ]]; then
        cp -r "$REPO_ROOT/secrets" "$LOCAL_BACKUP_PATH/"
        log_info "  ✓ secrets/ directory"
    else
        log_warn "  ⚠ secrets/ directory not found"
    fi
}

backup_lock_files() {
    log_info "Backing up lock files..."

    if [[ -f "$REPO_ROOT/flake.lock" ]]; then
        cp "$REPO_ROOT/flake.lock" "$LOCAL_BACKUP_PATH/"
        log_info "  ✓ flake.lock"
    fi

    if [[ -f "$REPO_ROOT/pixi.lock" ]]; then
        cp "$REPO_ROOT/pixi.lock" "$LOCAL_BACKUP_PATH/"
        log_info "  ✓ pixi.lock"
    fi
}

backup_configurations() {
    log_info "Backing up docker-compose configurations..."

    for compose_file in "$REPO_ROOT"/docker-compose*.yml; do
        if [[ -f "$compose_file" ]]; then
            cp "$compose_file" "$LOCAL_BACKUP_PATH/"
            log_info "  ✓ $(basename "$compose_file")"
        fi
    done

    # Backup docker data volumes (optional - can be large)
    if [[ -d "$REPO_ROOT/docker/data" ]] && [[ "${BACKUP_DOCKER_DATA:-false}" == "true" ]]; then
        log_info "Backing up docker data volumes (this may take a while)..."
        cp -r "$REPO_ROOT/docker/data" "$LOCAL_BACKUP_PATH/docker-data" 2>/dev/null || {
            log_warn "  ⚠ Some docker data volumes could not be backed up (may require sudo)"
        }
    fi
}

create_manifest() {
    log_info "Creating backup manifest..."

    cd "$REPO_ROOT"

    cat > "$LOCAL_BACKUP_PATH/MANIFEST.txt" << EOF
Backup created: $(date -u +%Y-%m-%dT%H:%M:%S%z)
Repository: $(git remote get-url origin 2>/dev/null || echo "UNKNOWN")
Branch: $(git branch --show-current 2>/dev/null || echo "UNKNOWN")
Commit: $(git rev-parse HEAD 2>/dev/null || echo "UNKNOWN")
Hostname: $(hostname)
Backup script: $0
Backup dir: $LOCAL_BACKUP_PATH

Contents:
EOF

    # List backed up files
    find "$LOCAL_BACKUP_PATH" -type f | while read -r file; do
        echo "  - $(basename "$file")" >> "$LOCAL_BACKUP_PATH/MANIFEST.txt"
    done

    log_info "  ✓ MANIFEST.txt"
}

compress_backup() {
    log_info "Compressing backup..."

    cd "$BACKUP_DIR"
    tar -czf "${BACKUP_NAME}.tar.gz" "$BACKUP_NAME"

    local size=$(du -h "${BACKUP_NAME}.tar.gz" | cut -f1)
    log_info "  ✓ Created ${BACKUP_NAME}.tar.gz ($size)"

    # Keep uncompressed directory for cloud sync (if not local-only)
    if [[ "$LOCAL_ONLY" == "true" ]]; then
        rm -rf "$LOCAL_BACKUP_PATH"
        log_info "  ✓ Removed uncompressed directory"
    fi
}

verify_backup() {
    if [[ "$VERIFY" == "false" ]]; then
        return 0
    fi

    log_info "Verifying backup integrity..."

    cd "$BACKUP_DIR"

    # Test tar extraction
    if tar -tzf "${BACKUP_NAME}.tar.gz" >/dev/null 2>&1; then
        log_info "  ✓ Archive integrity verified"
    else
        log_error "  ✗ Archive verification failed"
        return 1
    fi

    # Check key files are present
    local missing=()
    tar -tzf "${BACKUP_NAME}.tar.gz" | grep -q "env.backup" || missing+=("env.backup")
    tar -tzf "${BACKUP_NAME}.tar.gz" | grep -q "flake.lock" || missing+=("flake.lock")
    tar -tzf "${BACKUP_NAME}.tar.gz" | grep -q "pixi.lock" || missing+=("pixi.lock")

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_warn "  ⚠ Missing expected files: ${missing[*]}"
        return 1
    fi

    log_info "  ✓ All expected files present"
    return 0
}

upload_to_cloud() {
    if [[ "$LOCAL_ONLY" == "true" ]]; then
        log_info "Skipping cloud upload (--local-only flag set)"
        return 0
    fi

    if [[ -z "${BACKUP_CLOUD_DEST:-}" ]]; then
        log_warn "BACKUP_CLOUD_DEST not set - skipping cloud upload"
        log_info "Set BACKUP_CLOUD_DEST='remote:path' to enable cloud backups"
        return 0
    fi

    log_info "Uploading to cloud: $BACKUP_CLOUD_DEST"

    if ! command -v rclone >/dev/null 2>&1; then
        log_error "rclone not installed - cannot upload to cloud"
        return 1
    fi

    cd "$BACKUP_DIR"

    if rclone copy "${BACKUP_NAME}.tar.gz" "$BACKUP_CLOUD_DEST" --progress; then
        log_info "  ✓ Upload successful"
    else
        log_error "  ✗ Upload failed"
        return 1
    fi

    return 0
}

rotate_old_backups() {
    log_info "Rotating backups older than $BACKUP_RETENTION_DAYS days..."

    cd "$BACKUP_DIR"

    local deleted=0

    # Delete old compressed backups
    find "$BACKUP_DIR" -name "ripple-env-*.tar.gz" -type f -mtime "+$BACKUP_RETENTION_DAYS" | while read -r old_backup; do
        rm -f "$old_backup"
        log_info "  ✓ Deleted $(basename "$old_backup")"
        ((deleted++))
    done

    # Delete old uncompressed directories
    find "$BACKUP_DIR" -name "ripple-env-*" -type d -mtime "+$BACKUP_RETENTION_DAYS" | while read -r old_dir; do
        rm -rf "$old_dir"
        log_info "  ✓ Deleted $(basename "$old_dir")"
        ((deleted++))
    done

    if [[ $deleted -eq 0 ]]; then
        log_info "  No old backups to delete"
    else
        log_info "  Deleted $deleted old backup(s)"
    fi
}

send_notification() {
    local status="$1"
    local message="$2"

    if [[ -z "${BACKUP_NOTIFY_EMAIL:-}" ]]; then
        return 0
    fi

    log_info "Sending notification to $BACKUP_NOTIFY_EMAIL"

    # Use mail command if available
    if command -v mail >/dev/null 2>&1; then
        echo "$message" | mail -s "Ripple-env Backup: $status" "$BACKUP_NOTIFY_EMAIL"
    else
        log_warn "mail command not found - cannot send notification"
    fi
}

# Main execution
main() {
    log_info "Starting backup: $BACKUP_NAME"
    log_info "Repository: $REPO_ROOT"

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --local-only)
                LOCAL_ONLY=true
                shift
                ;;
            --verify)
                VERIFY=true
                shift
                ;;
            --retention)
                BACKUP_RETENTION_DAYS="$2"
                shift 2
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                usage
                exit 2
                ;;
        esac
    done

    # Execute backup steps
    check_prerequisites || exit $?
    create_backup_directory
    backup_environment_files
    backup_secrets
    backup_lock_files
    backup_configurations
    create_manifest
    compress_backup

    # Verification
    if ! verify_backup; then
        log_warn "Backup verification failed, but backup was created"
        send_notification "WARNING" "Backup created but verification failed: $BACKUP_NAME"
        exit 1
    fi

    # Cloud upload
    if ! upload_to_cloud; then
        log_warn "Cloud upload failed, but local backup is available"
        send_notification "WARNING" "Cloud upload failed for backup: $BACKUP_NAME"
    fi

    # Cleanup old backups
    rotate_old_backups

    # Success
    log_info "Backup completed successfully: $BACKUP_NAME"
    log_info "Location: $BACKUP_DIR/${BACKUP_NAME}.tar.gz"
    send_notification "SUCCESS" "Backup completed successfully: $BACKUP_NAME"

    exit 0
}

# Run main function
main "$@"
