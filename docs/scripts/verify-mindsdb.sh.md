# Script Contract: verify-mindsdb.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/verify-mindsdb.sh`

---

## Purpose

Verify MindsDB installation and connectivity (P2-011, AI/ML Predictions Layer). Checks dependencies, container status, port accessibility, API responsiveness, PostgreSQL database, Web UI, and displays integration examples.

---

## Invocation

```bash
./scripts/verify-mindsdb.sh
```

**Environment Variables:**
- `MINDSDB_HOST` - MindsDB host (default: localhost)
- `MINDSDB_API_PORT` - HTTP API port (default: 47334)
- `MINDSDB_MYSQL_PORT` - MySQL protocol port (default: 47335)
- `MINDSDB_MONGODB_PORT` - MongoDB protocol port (default: 47336)
- `MINDSDB_REQUIRE_RUNNING=1` - Fail if containers not running (default: 0, warnings only)

---

## Outputs

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success or containers not running (if MINDSDB_REQUIRE_RUNNING=0) |
| `1` | Dependencies missing or containers not running (if MINDSDB_REQUIRE_RUNNING=1) |

---

## Side Effects

**Minimal:** Read-only verification, no persistent changes.

**Docker Checks:** Container inspection, database connection tests.

---

## Safety Classification

**🟢 SAFE** - Read-only verification, no destructive operations.

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can be run repeatedly.

---

## Verification Steps (6)

### Step 1: Dependencies (lines 77-112)
Checks for:
- **docker** (required)
- **curl** (required)
- **nc** (netcat) - optional for port checks
- **jq** - optional for JSON parsing

**Evidence:** Lines 82-104.

**Behavior:** Exits 1 if docker or curl missing.

### Step 2: Container Status (lines 114-155)
Checks two containers:
- **mindsdb** - Main MindsDB service
- **mindsdb-db** - PostgreSQL database

**Evidence:** Lines 117-152.

**Health Status:**
- `running + healthy` - Pass
- `running + no healthcheck` - Pass
- `running + unhealthy` - Warning
- `not running` - Fail (or warning if MINDSDB_REQUIRE_RUNNING=0)

**Early Exit:** If containers not running and MINDSDB_REQUIRE_RUNNING=0, exits 0 with setup instructions (lines 393-399).

### Step 3: Port Connectivity (lines 158-184)
Tests 3 ports with netcat:
- **47334** - HTTP API
- **47335** - MySQL protocol
- **47336** - MongoDB protocol

**Evidence:** Lines 160-183.

**Behavior:** Warnings if nc unavailable or ports unreachable (may be normal during startup).

### Step 4: MindsDB API (lines 186-208)
- Tests `/api/status` endpoint
- Parses JSON response with jq (if available)
- Displays status information

**Evidence:** Lines 189-207.

**URL:** `http://localhost:47334/api/status`

### Step 5: PostgreSQL Database (lines 210-228)
- Tests `pg_isready` inside mindsdb-db container
- Counts tables in public schema
- Displays table count

**Evidence:** Lines 215-225.

**Command:**
```bash
docker exec mindsdb-db pg_isready -U mindsdb -d mindsdb
docker exec mindsdb-db psql -U mindsdb -d mindsdb -t \
  -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public';"
```

### Step 6: Web UI (lines 230-243)
- Tests Web UI accessibility at root URL
- Displays browser URL

**Evidence:** Lines 233-240.

**URL:** `http://localhost:47334`

---

## Example Queries Section

**Evidence:** Lines 245-287

**Displays:**
1. **MySQL connection** (port 47335)
2. **Connect to MLflow PostgreSQL**
3. **Create sentiment analysis model** (HuggingFace)
4. **Use model for predictions**
5. **List databases and models**

**Example:**
```sql
-- Connect to MLflow database
CREATE DATABASE mlflow_data
WITH ENGINE = "postgres",
PARAMETERS = {
  "host": "mlflow-db",
  "port": "5432",
  "database": "mlflow",
  "user": "mlflow",
  "password": "changeme"
};

-- Create sentiment model
CREATE MODEL sentiment_model
PREDICT sentiment
USING
  engine = 'huggingface',
  model_name = 'distilbert-base-uncased-finetuned-sst-2-english';

-- Run predictions
SELECT text, sentiment
FROM sentiment_model
WHERE text = 'I love this product!';
```

---

## Integration Examples Section

**Evidence:** Lines 289-341

**Shows connections to ARIA services:**

1. **MLflow database** - Access experiment data
2. **MinIO via S3** - Access model artifacts
3. **Time-series forecasting** - Predict metrics
4. **Classification models** - LightGBM integration
5. **SQL-based predictions** - JOIN with forecast models

---

## Summary Section

**Evidence:** Lines 343-381

**Displays:**
- Service URLs (API, Web Studio, MySQL, MongoDB)
- Container management commands (start, stop, logs, restart)
- MySQL client access command
- Documentation links
- Next steps

---

## MindsDB Context

**P2-011: AI/ML Predictions Layer**
- **Purpose:** SQL-based ML predictions and AutoML
- **Protocols:** HTTP REST, MySQL, MongoDB
- **Database:** PostgreSQL (mindsdb-db container)
- **Studio:** Web-based SQL editor and model manager

**Capabilities:**
- Connect to existing databases (PostgreSQL, MySQL, MongoDB, etc.)
- Train ML models via SQL syntax
- Run predictions as SQL queries
- Time-series forecasting
- NLP models (HuggingFace, OpenAI)
- Integration with MLflow, MinIO, NATS

---

## Compose File Resolution

**Evidence:** Lines 146-149

**Pattern:** Check subdirectory first, fallback to root
```bash
local data_compose_file="docker-compose.data.yml"
if [ -f "docker/docker-compose.data.yml" ]; then
    data_compose_file="docker/docker-compose.data.yml"
fi
```

---

## Key Features

### 1. Graceful Container Unavailability

**Evidence:** Lines 393-399

**Default behavior (MINDSDB_REQUIRE_RUNNING=0):**
- Warns if containers not running
- Skips live checks
- Exits 0 (success)
- Shows docker compose command

**Strict mode (MINDSDB_REQUIRE_RUNNING=1):**
- Fails if containers not running
- Exits 1 (failure)

### 2. Health Status Interpretation

**Evidence:** Lines 122-138

**Handles three cases:**
- `healthy` - Pass
- `none` (no healthcheck) - Pass with note
- Other (starting, unhealthy) - Warning

### 3. PostgreSQL Table Count

**Evidence:** Lines 219-221

**Query:**
```bash
docker exec mindsdb-db psql -U mindsdb -d mindsdb -t \
  -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public';"
```

**Purpose:** Verifies database initialized with schema.

### 4. Comprehensive Documentation

**Evidence:** Lines 245-381

**Three sections:**
1. Example SQL queries (lines 245-287)
2. Integration examples (lines 289-341)
3. Summary with URLs and commands (lines 343-381)

**Rationale:** Self-contained guide for new users.

---

## Next Steps

**Displayed on completion (lines 374-379):**
```
1. Open MindsDB Studio in your browser
2. Connect to existing ARIA databases (MLflow, MinIO, etc.)
3. Create your first ML model
4. Run predictions via SQL queries
```

---

## References

### Source Code
- **Main script:** `scripts/verify-mindsdb.sh` (414 lines)
- **Dependencies check:** lines 77-112
- **Container check:** lines 114-155
- **Port check:** lines 158-184
- **API check:** lines 186-208
- **Database check:** lines 210-228
- **Examples:** lines 245-341
- **Summary:** lines 343-381

### Related Files
- **Compose file:** `docker-compose.data.yml` (or `docker/docker-compose.data.yml`)
- **MLflow integration:** `docker-compose.ml.yml` (inferred)
- **MinIO integration:** `docker-compose.data.yml` (shared)

### External Resources
- [MindsDB Documentation](https://docs.mindsdb.com/)
- [MindsDB Integrations](https://docs.mindsdb.com/integrations/data-integrations)
- [ML Engines](https://docs.mindsdb.com/integrations/ml-integrations)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 20/60 contracts complete
