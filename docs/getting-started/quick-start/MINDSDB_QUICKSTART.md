# MindsDB Quick Start Guide

## 🚀 Get Started in 5 Minutes

### Step 1: Set Up Environment Variables

```bash
# Copy the example environment file
cp .env.data.example .env.data

# (Optional) Edit with secure credentials
nano .env.data
```

### Step 2: Start MindsDB

```bash
# Create the network (if not already exists)
docker network create agentic-network 2>/dev/null || true

# Start MindsDB services
docker compose -f docker-compose.data.yml up -d

# Wait for services to be healthy (30-60 seconds)
watch -n 2 'docker compose -f docker-compose.data.yml ps'
```

### Step 3: Verify Installation

```bash
# Run the automated verification script
./scripts/verify-mindsdb.sh

# Or manually check
curl http://localhost:47334/api/status
```

### Step 4: Access MindsDB

**Web UI**: http://localhost:47334

**MySQL Client**:
```bash
# Using Docker
docker exec -it mindsdb mysql -h localhost -P 47335 -u mindsdb

# Or using local MySQL client
mysql -h localhost -P 47335 -u mindsdb
```

---

## 🎯 Your First ML Model

### Example 1: Sentiment Analysis

```sql
-- Create a sentiment analysis model
CREATE MODEL sentiment_analyzer
PREDICT sentiment
USING
  engine = 'huggingface',
  model_name = 'distilbert-base-uncased-finetuned-sst-2-english';

-- Use it!
SELECT text, sentiment
FROM sentiment_analyzer
WHERE text = 'This is amazing!';
```

### Example 2: Connect to MLflow

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

-- Query experiment data
SELECT * FROM mlflow_data.metrics LIMIT 10;
```

### Example 3: Time-Series Forecasting

```sql
-- Create a forecasting model
CREATE MODEL sales_forecast
FROM sales_data.historical
PREDICT sales_amount
ORDER BY date
WINDOW 30
HORIZON 7;

-- Get predictions
SELECT date, sales_amount, predicted_sales_amount
FROM sales_data.historical
JOIN sales_forecast;
```

---

## 📊 Useful Commands

```sql
-- List all databases
SHOW DATABASES;

-- List all models
SHOW MODELS;

-- Describe a model
DESCRIBE sentiment_analyzer;

-- Drop a model
DROP MODEL sentiment_analyzer;

-- Show model training status
SELECT * FROM mindsdb.models;
```

---

## 🔗 Integration Examples

### Connect to MinIO (S3)

```sql
CREATE DATABASE minio_storage
WITH ENGINE = "s3",
PARAMETERS = {
  "aws_access_key_id": "minioadmin",
  "aws_secret_access_key": "minioadmin",
  "bucket": "aria-models",
  "endpoint_url": "http://minio:9000"
};
```

### Connect to External API

```sql
CREATE DATABASE weather_api
WITH ENGINE = "rest_api",
PARAMETERS = {
  "url": "https://api.openweathermap.org/data/2.5",
  "api_key": "your_api_key"
};
```

---

## 🛠️ Troubleshooting

### Services won't start
```bash
# Check logs
docker compose -f docker-compose.data.yml logs mindsdb

# Restart services
docker compose -f docker-compose.data.yml restart
```

### API not responding
```bash
# Wait for startup (can take 60 seconds)
docker compose -f docker-compose.data.yml logs -f mindsdb

# Check health status
docker inspect mindsdb --format='{{.State.Health.Status}}'
```

### Can't connect to other services
```bash
# Verify network
docker network inspect agentic-network

# Test connectivity
docker exec -it mindsdb ping mlflow-db
```

---

## 📚 Learn More

- **Full Documentation**: [P2-011-MINDSDB-IMPLEMENTATION.md](../../P2-011-MINDSDB-IMPLEMENTATION.md)
- **MindsDB Docs**: https://docs.mindsdb.com/
- **SQL Reference**: https://docs.mindsdb.com/sql/overview
- **Integrations**: https://docs.mindsdb.com/integrations/data-integrations

---

## 🔒 Security Reminders

1. ✅ Change default passwords in `.env.data`
2. ✅ Set file permissions: `chmod 600 .env.data`
3. ✅ Never commit `.env.data` to version control
4. ✅ Use HashiCorp Vault in production

---

## 🎉 What's Next?

1. **Explore Pre-trained Models**: Try different Hugging Face models
2. **Connect Your Data**: Integrate with existing ARIA services
3. **Build Custom Models**: Train models on your own data
4. **Automate Predictions**: Use n8n or Temporal for scheduled predictions
5. **Monitor Performance**: Integrate with Grafana dashboards

---

**Need Help?**
- Check logs: `docker compose -f docker-compose.data.yml logs -f`
- Run verification: `./scripts/verify-mindsdb.sh`
- Read full docs: [P2-011-MINDSDB-IMPLEMENTATION.md](../../P2-011-MINDSDB-IMPLEMENTATION.md)
