# Script Contract: init-multi-db.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/init-multi-db.sh`

---

## Purpose

PostgreSQL multi-database initialization script. Creates additional databases in a PostgreSQL container beyond the default `POSTGRES_DB`. Used by Kong Gateway to provision both `kong` and `konga` databases in single PostgreSQL instance.

---

## Invocation

```bash
# Executed automatically by PostgreSQL Docker image
# via POSTGRES_INITDB_SCRIPTS or docker-entrypoint-initdb.d/
```

**Environment Variables (required):**
- `POSTGRES_USER` - PostgreSQL admin user
- `POSTGRES_DB` - Default database (already created by postgres image)
- `POSTGRES_MULTIPLE_DATABASES` - Comma-separated list of additional databases to create

**Example:**
```bash
POSTGRES_MULTIPLE_DATABASES=kong,konga
```

---

## Outputs

**Standard Output:**
```
Multiple database creation requested: kong,konga
Creating database 'kong'
Creating database 'konga'
Multiple databases created successfully
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all databases created or POSTGRES_MULTIPLE_DATABASES not set) |
| `1` | Failure (psql command failed) |

---

## Side Effects

**Creates databases** specified in `POSTGRES_MULTIPLE_DATABASES` environment variable (lines 8-16).

**Grants privileges** to `POSTGRES_USER` on each created database (line 14).

**Idempotency:** Uses conditional CREATE DATABASE (lines 12-13) - skips if already exists.

---

## Safety Classification

**🟢 SAFE** - Idempotent, creates databases only if they don't exist.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly. Uses `WHERE NOT EXISTS` check.

---

## Execution Logic

### Conditional Execution (lines 18-29)

**Evidence:** Only runs if `POSTGRES_MULTIPLE_DATABASES` is set

```bash
if [ -n "${POSTGRES_MULTIPLE_DATABASES:-}" ]; then
    # Process comma-separated list
fi
```

### Database Creation Function (lines 8-16)

**Evidence:** Creates database with conditional logic

```bash
function create_database() {
    local database=$1
    echo "Creating database '$database'"
    psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER" <<-EOSQL
        SELECT 'CREATE DATABASE $database'
        WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = '$database')\gexec
        GRANT ALL PRIVILEGES ON DATABASE $database TO $POSTGRES_USER;
EOSQL
}
```

**Key features:**
- `ON_ERROR_STOP=1` - Exits on first error
- `WHERE NOT EXISTS` - Idempotency check
- `\gexec` - Executes generated SQL command
- `GRANT ALL PRIVILEGES` - Grants full access to admin user

### List Processing (lines 20-27)

**Evidence:** Parses comma-separated database list

```bash
IFS=',' read -ra DBS <<< "$POSTGRES_MULTIPLE_DATABASES"
for db in "${DBS[@]}"; do
    # Trim whitespace
    db=$(echo "$db" | xargs)
    if [ "$db" != "$POSTGRES_DB" ]; then
        create_database "$db"
    fi
done
```

**Whitespace handling:** Uses `xargs` to trim leading/trailing spaces.

**Skip default:** Avoids creating `POSTGRES_DB` (already exists).

---

## PostgreSQL Init Scripts

**Execution context:** Runs inside PostgreSQL container during first start.

**Location:** Mounted to `/docker-entrypoint-initdb.d/` in container.

**Trigger:** PostgreSQL official image automatically runs all `.sh` and `.sql` files in this directory on first initialization.

**Timing:** Executes AFTER database specified by `POSTGRES_DB` is created.

---

## Usage Example

### Kong Database Setup

**docker-compose.identity.yml:**
```yaml
services:
  postgres:
    image: postgres:15-alpine
    environment:
      POSTGRES_DB: postgres
      POSTGRES_USER: postgres
      POSTGRES_PASSWORD: ${POSTGRES_PASSWORD}
      POSTGRES_MULTIPLE_DATABASES: kong,konga
    volumes:
      - ./scripts/init-multi-db.sh:/docker-entrypoint-initdb.d/init-multi-db.sh:ro
```

**Result:** Creates 3 databases:
1. `postgres` (default, by image)
2. `kong` (by script)
3. `konga` (by script)

---

## SQL Technique: Conditional CREATE

**Evidence:** Lines 12-13

**Traditional approach (fails if exists):**
```sql
CREATE DATABASE kong;  -- Error if exists
```

**Idempotent approach (used by script):**
```sql
SELECT 'CREATE DATABASE kong'
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'kong')\gexec
```

**How it works:**
1. `SELECT ... WHERE NOT EXISTS` - Returns command string only if database doesn't exist
2. `\gexec` - Executes the returned string as SQL command
3. If database exists, query returns empty, nothing executed

---

## Error Handling

**Evidence:** Line 11

**ON_ERROR_STOP=1:**
```bash
psql -v ON_ERROR_STOP=1 --username "$POSTGRES_USER"
```

**Behavior:**
- Stops psql on first error
- Script exits with non-zero code
- Container initialization fails
- Database remains empty/incomplete

**Rationale:** Prevents partially initialized databases.

---

## Integration Points

**Used by:**
- Kong Gateway (requires `kong` database)
- Konga (requires `konga` database)
- Custom services needing dedicated PostgreSQL databases

**Alternative:** Separate PostgreSQL containers per service (higher resource usage).

---

## Limitations

**No database deletion:** Script only creates, never deletes.

**No user management:** All databases owned by `POSTGRES_USER`.

**One-time execution:** Only runs on first container start (when data directory is empty).

**Workaround for re-run:**
```bash
# Remove volume and restart
docker compose down -v
docker compose up -d
```

---

## References

### Source Code
- **Main script:** `scripts/init-multi-db.sh` (30 lines)
- **Create function:** lines 8-16
- **List processing:** lines 18-29

### Related Files
- **Kong compose:** `docker-compose.identity.yml`
- **Edge compose:** `docker-compose.edge.yml` (mounts this script)

### External Resources
- [PostgreSQL Docker Image](https://hub.docker.com/_/postgres)
- [Docker Entrypoint Scripts](https://github.com/docker-library/docs/blob/master/postgres/README.md#initialization-scripts)
- [PostgreSQL CREATE DATABASE](https://www.postgresql.org/docs/current/sql-createdatabase.html)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 23/60 contracts complete
