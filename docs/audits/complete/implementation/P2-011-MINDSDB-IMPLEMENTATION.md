# P2-011: MindsDB Implementation

**Status**: ✅ COMPLETE
**Domain**: L11 - Data & Query
**Team Lead**: L11 Data & Query Domain Team Lead
**Date**: 2026-01-09

---

## Overview

MindsDB has been successfully integrated into the ARIA platform as an AI/ML database layer. This implementation enables ML predictions via SQL, providing a powerful interface for integrating machine learning models with existing data sources.

## What is MindsDB?

MindsDB is an AI-powered database layer that brings machine learning capabilities directly to your database through SQL. It allows you to:
- Create and train ML models using SQL commands
- Run predictions via standard SQL queries
- Connect multiple data sources (PostgreSQL, MySQL, MongoDB, APIs, S3, etc.)
- Integrate pre-trained models from Hugging Face, OpenAI, and more
- Perform time-series forecasting, classification, and NLP tasks

## Architecture

### Components

1. **MindsDB Server** (`mindsdb`)
   - Image: `mindsdb/mindsdb:latest`
   - HTTP API: Port 47334
   - MySQL Protocol: Port 47335
   - MongoDB Protocol: Port 47336
   - Web Studio UI: Port 47334

2. **PostgreSQL Metadata Store** (`mindsdb-db`)
   - Image: `postgres:17.2-alpine`
   - Stores model definitions, configurations, and metadata
   - Port: 5432 (internal)

### Network Topology

```
┌─────────────────────────────────────────────────────────┐
│                    agentic-network                       │
│                                                          │
│  ┌─────────────┐          ┌──────────────┐             │
│  │   MindsDB   │◄─────────┤  MindsDB DB  │             │
│  │             │          │ (PostgreSQL) │             │
│  └──────┬──────┘          └──────────────┘             │
│         │                                               │
│         │ Can connect to:                               │
│         ├─────────► MLflow DB (PostgreSQL)              │
│         ├─────────► MinIO (S3)                          │
│         ├─────────► External APIs                       │
│         └─────────► Other data sources                  │
│                                                          │
└─────────────────────────────────────────────────────────┘
         │
         │ Exposed Ports
         ├── 47334 (HTTP API & Web UI)
         ├── 47335 (MySQL Protocol)
         └── 47336 (MongoDB Protocol)
```

## Files Created/Modified

### Created Files

1. **`/home/user/ripple-env/docker-compose.data.yml`**
   - Docker Compose configuration for MindsDB
   - PostgreSQL metadata store
   - Volume and network definitions

2. **`/home/user/ripple-env/.env.data.example`**
   - Environment variable template for data services
   - Secure credential patterns
   - Integration examples

3. **`/home/user/ripple-env/scripts/verify-mindsdb.sh`**
   - Automated verification script
   - Health checks for all components
   - Example queries and integration patterns

4. **`/home/user/ripple-env/P2-011-MINDSDB-IMPLEMENTATION.md`**
   - This documentation file

### Modified Files

1. **`/home/user/ripple-env/.env.example`**
   - Added MindsDB environment variables section
   - Follows existing security patterns

## Environment Variables

All credentials follow the secure pattern with `${VAR:-default}` syntax:

```bash
# MindsDB API Key
MINDSDB_API_KEY=changeme              # Generate with: openssl rand -base64 32

# MindsDB PostgreSQL Credentials (metadata store)
MINDSDB_DB_HOST=mindsdb-db
MINDSDB_DB_PORT=5432
MINDSDB_DB_USER=mindsdb
MINDSDB_DB_PASSWORD=changeme          # Generate with: openssl rand -base64 32
MINDSDB_DB_NAME=mindsdb
```

### Security Best Practices

- ✅ No hardcoded secrets in docker-compose files
- ✅ Default values use `changeme` placeholder
- ✅ Environment variables with fallback defaults
- ✅ Credentials should be stored in HashiCorp Vault for production
- ✅ File permissions should be `chmod 600 .env.data`

## Deployment

### Prerequisites

```bash
# Ensure the agentic-network exists
docker network create agentic-network 2>/dev/null || true
```

### Start MindsDB

```bash
# Start services
docker compose -f docker-compose.data.yml up -d

# Check status
docker compose -f docker-compose.data.yml ps

# View logs
docker compose -f docker-compose.data.yml logs -f mindsdb
```

### Stop MindsDB

```bash
docker compose -f docker-compose.data.yml down

# Remove volumes (WARNING: destroys all data)
docker compose -f docker-compose.data.yml down -v
```

## Verification

### Automated Verification

Run the comprehensive verification script:

```bash
./scripts/verify-mindsdb.sh
```

This script checks:
- ✅ Dependencies (docker, curl, jq)
- ✅ Container status and health
- ✅ Port connectivity
- ✅ API responsiveness
- ✅ PostgreSQL database
- ✅ Web UI accessibility

### Manual Verification

#### 1. Check Container Health

```bash
docker ps --filter name=mindsdb
```

Expected output:
```
CONTAINER ID   IMAGE                      STATUS                    PORTS
xxxxx          mindsdb/mindsdb:latest     Up 2 minutes (healthy)    0.0.0.0:47334-47336->47334-47336/tcp
xxxxx          postgres:17.2-alpine       Up 2 minutes (healthy)    5432/tcp
```

#### 2. Test HTTP API

```bash
curl http://localhost:47334/api/status
```

#### 3. Access Web UI

Open in browser: http://localhost:47334

#### 4. Connect via MySQL Client

```bash
# Using Docker
docker exec -it mindsdb mysql -h localhost -P 47335 -u mindsdb

# Using local MySQL client
mysql -h localhost -P 47335 -u mindsdb
```

## Usage Examples

### 1. Connect to MLflow Database

```sql
CREATE DATABASE mlflow_data
WITH ENGINE = "postgres",
PARAMETERS = {
  "host": "mlflow-db",
  "port": "5432",
  "database": "mlflow",
  "user": "mlflow",
  "password": "changeme"
};

-- Query MLflow data
SELECT * FROM mlflow_data.metrics LIMIT 10;
```

### 2. Create a Sentiment Analysis Model

```sql
CREATE MODEL sentiment_model
PREDICT sentiment
USING
  engine = 'huggingface',
  model_name = 'distilbert-base-uncased-finetuned-sst-2-english';

-- Use the model
SELECT text, sentiment
FROM sentiment_model
WHERE text = 'I love this product!';
```

### 3. Time-Series Forecasting

```sql
CREATE MODEL forecast_model
FROM mlflow_data.metrics
PREDICT value
ORDER BY timestamp
WINDOW 10
HORIZON 5;

-- Get predictions
SELECT timestamp, value, predicted_value
FROM mlflow_data.metrics
JOIN forecast_model;
```

### 4. Connect to MinIO (S3)

```sql
CREATE DATABASE minio_artifacts
WITH ENGINE = "s3",
PARAMETERS = {
  "aws_access_key_id": "minioadmin",
  "aws_secret_access_key": "minioadmin",
  "bucket": "aria-models",
  "endpoint_url": "http://minio:9000"
};

-- Query S3 files
SELECT * FROM minio_artifacts.files LIMIT 10;
```

### 5. List Available Resources

```sql
-- List all databases
SHOW DATABASES;

-- List all models
SHOW MODELS;

-- Show model details
DESCRIBE sentiment_model;

-- Show database tables
SHOW TABLES FROM mlflow_data;
```

## Integration with ARIA Services

MindsDB can connect to various ARIA platform services:

| Service | Connection Type | Use Case |
|---------|----------------|----------|
| MLflow | PostgreSQL | Query experiment data, create models from metrics |
| MinIO | S3 | Access stored artifacts, train models on data |
| AGiXT | HTTP API | Use AGiXT agents for predictions |
| ClickHouse | Native | Query TensorZero observability data |
| Redis | Redis | Access cached data, predictions |

## Resource Requirements

### Development

```yaml
mindsdb:
  memory: 2GB reserved, 4GB limit
  cpu: 1.0 reserved, 2.0 limit

mindsdb-db:
  memory: 256MB reserved, 1GB limit
```

### Production Recommendations

- **Memory**: 8GB+ for MindsDB (depending on model size)
- **CPU**: 4+ cores (ML operations are CPU-intensive)
- **Storage**: 50GB+ for model storage and cache
- **Database**: Regular backups of `mindsdb_db_data` volume

## Monitoring & Observability

### Health Checks

MindsDB includes health checks:
- Endpoint: `http://localhost:47334/api/status`
- Interval: 30s
- Timeout: 10s
- Retries: 5

PostgreSQL health checks:
- Command: `pg_isready -U mindsdb -d mindsdb`
- Interval: 10s

### Logs

```bash
# View MindsDB logs
docker compose -f docker-compose.data.yml logs -f mindsdb

# View PostgreSQL logs
docker compose -f docker-compose.data.yml logs -f mindsdb-db

# View all logs
docker compose -f docker-compose.data.yml logs -f
```

### Metrics

MindsDB exposes metrics that can be integrated with Prometheus:
- Training time per model
- Prediction latency
- Query performance
- Resource usage

## Troubleshooting

### MindsDB Won't Start

```bash
# Check logs
docker compose -f docker-compose.data.yml logs mindsdb

# Common issue: Database not ready
# Wait for PostgreSQL to be healthy
docker compose -f docker-compose.data.yml ps mindsdb-db

# Restart services
docker compose -f docker-compose.data.yml restart
```

### API Not Responding

```bash
# Check if port is bound
netstat -tulpn | grep 47334

# Check container networking
docker network inspect agentic-network

# Verify healthcheck
docker inspect mindsdb --format='{{.State.Health.Status}}'
```

### Connection Issues with Other Services

```bash
# Verify all services are on same network
docker network inspect agentic-network

# Test connectivity from MindsDB container
docker exec -it mindsdb ping mlflow-db
docker exec -it mindsdb ping minio

# Check DNS resolution
docker exec -it mindsdb nslookup mlflow-db
```

### Database Connection Errors

```bash
# Check PostgreSQL is ready
docker exec mindsdb-db pg_isready -U mindsdb -d mindsdb

# Verify credentials
docker exec -it mindsdb-db psql -U mindsdb -d mindsdb -c "SELECT 1;"

# Reset database (WARNING: destroys data)
docker compose -f docker-compose.data.yml down -v
docker compose -f docker-compose.data.yml up -d
```

## Security Considerations

### Production Deployment

1. **Change Default Credentials**
   ```bash
   # Generate secure passwords
   openssl rand -base64 32
   ```

2. **Restrict Network Access**
   - Use firewall rules to limit port access
   - Consider VPN or private network for sensitive data

3. **Enable TLS/SSL**
   - Configure HTTPS for API access
   - Use SSL for PostgreSQL connections

4. **Secrets Management**
   - Store credentials in HashiCorp Vault
   - Use Kubernetes secrets for K8s deployments
   - Never commit `.env.data` to version control

5. **Regular Updates**
   - Monitor for security patches
   - Update MindsDB image regularly
   - Keep PostgreSQL updated

### Audit Logging

Enable audit logging for compliance:
```sql
-- Track model usage
SELECT * FROM mindsdb.predictors_metadata;

-- Track queries
SELECT * FROM mindsdb.audit_log;
```

## Performance Tuning

### PostgreSQL Optimization

```bash
# Increase shared_buffers for better caching
docker exec mindsdb-db psql -U mindsdb -d mindsdb -c \
  "ALTER SYSTEM SET shared_buffers = '256MB';"

# Increase work_mem for complex queries
docker exec mindsdb-db psql -U mindsdb -d mindsdb -c \
  "ALTER SYSTEM SET work_mem = '16MB';"

# Restart to apply
docker compose -f docker-compose.data.yml restart mindsdb-db
```

### MindsDB Optimization

```yaml
# Increase memory limits in docker-compose.data.yml
deploy:
  resources:
    limits:
      memory: 8gb
      cpus: '4.0'
```

## Backup & Recovery

### Backup

```bash
# Backup PostgreSQL metadata
docker exec mindsdb-db pg_dump -U mindsdb mindsdb | gzip > mindsdb_backup_$(date +%Y%m%d).sql.gz

# Backup volumes
docker run --rm -v mindsdb_storage:/data -v $(pwd):/backup alpine \
  tar czf /backup/mindsdb_storage_$(date +%Y%m%d).tar.gz /data
```

### Recovery

```bash
# Restore PostgreSQL
gunzip -c mindsdb_backup_20260109.sql.gz | \
  docker exec -i mindsdb-db psql -U mindsdb mindsdb

# Restore volumes
docker run --rm -v mindsdb_storage:/data -v $(pwd):/backup alpine \
  tar xzf /backup/mindsdb_storage_20260109.tar.gz -C /
```

## API Reference

### HTTP API Endpoints

- `GET /api/status` - Health check
- `POST /api/sql/query` - Execute SQL query
- `GET /api/projects` - List projects
- `GET /api/databases` - List databases
- `GET /api/models` - List models
- `GET /api/predictors` - List predictors (legacy)

### Example API Usage

```bash
# Query via HTTP API
curl -X POST http://localhost:47334/api/sql/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "SHOW DATABASES;"
  }'
```

## Future Enhancements

### Planned Improvements

1. **Integration with ARIA Observability**
   - Send MindsDB metrics to Prometheus
   - Create Grafana dashboards
   - Alert on model performance degradation

2. **Model Registry Integration**
   - Store models in MLflow
   - Version control for MindsDB models
   - A/B testing framework

3. **Automated Retraining**
   - Schedule model retraining via n8n
   - Use Temporal workflows for orchestration
   - Monitor data drift

4. **Advanced Security**
   - Integrate with Keycloak for authentication
   - Row-level security for data access
   - Encryption at rest

## References

- **MindsDB Documentation**: https://docs.mindsdb.com/
- **Data Integrations**: https://docs.mindsdb.com/integrations/data-integrations
- **ML Integrations**: https://docs.mindsdb.com/integrations/ml-integrations
- **SQL Syntax**: https://docs.mindsdb.com/sql/overview
- **GitHub Repository**: https://github.com/mindsdb/mindsdb

## Support

For issues or questions:
1. Check MindsDB logs: `docker compose -f docker-compose.data.yml logs mindsdb`
2. Run verification script: `./scripts/verify-mindsdb.sh`
3. Consult MindsDB documentation: https://docs.mindsdb.com/
4. Check ARIA project issues on GitHub

---

## Summary

✅ **P2-011 Implementation Complete**

- ✅ MindsDB service added to docker-compose.data.yml
- ✅ PostgreSQL metadata store configured
- ✅ Environment variables following secure patterns
- ✅ No hardcoded secrets
- ✅ Comprehensive documentation
- ✅ Verification script included
- ✅ Integration examples provided
- ✅ Security best practices documented

**Next Steps:**
1. Start services: `docker compose -f docker-compose.data.yml up -d`
2. Run verification: `./scripts/verify-mindsdb.sh`
3. Access Web UI: http://localhost:47334
4. Create your first ML model!
