#!/bin/bash
# =============================================================================
# PostgreSQL Multi-Database Initialization Script
# =============================================================================
# This script is executed when the PostgreSQL container starts for the first time.
# It creates multiple databases for different services from POSTGRES_MULTIPLE_DATABASES.
#
# Usage:
#   Mount this script to /docker-entrypoint-initdb.d/init-multi-db.sh
#
# Environment Variables:
#   POSTGRES_MULTIPLE_DATABASES - Comma-separated list of databases to create
#   POSTGRES_USER - PostgreSQL user (owner of created databases)
#
# =============================================================================

set -e
set -u

# Log function
log() {
    echo "[init-multi-db] $*"
}

# Create a database and grant permissions
create_database() {
    local database="$1"
    log "Creating database: $database"

    psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" <<-EOSQL
        -- Create database if it doesn't exist
        SELECT 'CREATE DATABASE $database'
        WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = '$database')\gexec

        -- Grant privileges
        GRANT ALL PRIVILEGES ON DATABASE $database TO $POSTGRES_USER;
EOSQL

    log "Database '$database' created successfully"
}

# Main execution
main() {
    log "Starting multi-database initialization..."

    # Check if POSTGRES_MULTIPLE_DATABASES is set
    if [ -z "${POSTGRES_MULTIPLE_DATABASES:-}" ]; then
        log "POSTGRES_MULTIPLE_DATABASES is not set, skipping multi-db init"
        return 0
    fi

    log "Databases to create: $POSTGRES_MULTIPLE_DATABASES"

    # Split comma-separated list and create each database
    IFS=',' read -ra DATABASES <<< "$POSTGRES_MULTIPLE_DATABASES"
    for db in "${DATABASES[@]}"; do
        # Trim whitespace
        db=$(echo "$db" | xargs)
        if [ -n "$db" ]; then
            create_database "$db"
        fi
    done

    log "Multi-database initialization complete"

    # List all databases
    log "Current databases:"
    psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" -c "\l"
}

main "$@"
