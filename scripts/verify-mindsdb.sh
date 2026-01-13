#!/usr/bin/env bash
# =============================================================================
# MindsDB Verification Script
# =============================================================================
# Verifies MindsDB installation and connectivity for P2-011
#
# Usage:
#   ./scripts/verify-mindsdb.sh
#
# Prerequisites:
#   - docker-compose.data.yml services are running
#   - jq installed (for JSON parsing)
#   - curl installed (for API testing)
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
MINDSDB_HOST="${MINDSDB_HOST:-localhost}"
MINDSDB_API_PORT="${MINDSDB_API_PORT:-47334}"
MINDSDB_MYSQL_PORT="${MINDSDB_MYSQL_PORT:-47335}"
MINDSDB_MONGODB_PORT="${MINDSDB_MONGODB_PORT:-47336}"

# =============================================================================
# Helper Functions
# =============================================================================

print_header() {
    echo -e "${BLUE}================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

jq_is_runnable() {
    command -v jq >/dev/null 2>&1 && jq --version >/dev/null 2>&1
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        print_error "Command '$1' is required but not installed"
        return 1
    fi
    return 0
}

# =============================================================================
# Verification Steps
# =============================================================================

verify_dependencies() {
    print_header "Step 1: Checking Dependencies"

    local all_ok=true

    if check_command docker; then
        print_success "docker is installed"
    else
        all_ok=false
    fi

    if check_command curl; then
        print_success "curl is installed"
    else
        all_ok=false
    fi

    if jq_is_runnable; then
        print_success "jq is installed"
    else
        print_warning "jq not available/runnable (optional, for JSON parsing)"
    fi

    if [ "$all_ok" = false ]; then
        print_error "Missing required dependencies"
        exit 1
    fi

    echo ""
}

verify_containers() {
    print_header "Step 2: Checking Container Status"

    local containers=("mindsdb" "mindsdb-db")
    local all_running=true

    for container in "${containers[@]}"; do
        if docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
            local status
            status=$(docker inspect --format='{{.State.Status}}' "$container")
            local health
            health=$(docker inspect --format='{{.State.Health.Status}}' "$container" 2>/dev/null || echo "none")

            if [ "$status" = "running" ]; then
                if [ "$health" = "healthy" ]; then
                    print_success "$container is running and healthy"
                elif [ "$health" = "none" ]; then
                    print_success "$container is running (no healthcheck)"
                else
                    print_warning "$container is running but health status: $health"
                fi
            else
                print_error "$container status: $status"
                all_running=false
            fi
        else
            print_error "$container is not running"
            all_running=false
        fi
    done

    if [ "$all_running" = false ]; then
        local data_compose_file="docker-compose.data.yml"
        if [ -f "docker/docker-compose.data.yml" ]; then
            data_compose_file="docker/docker-compose.data.yml"
        fi
        print_info "Start services with: docker compose -f $data_compose_file up -d"
        exit 1
    fi

    echo ""
}

verify_ports() {
    print_header "Step 3: Checking Port Connectivity"

    local ports=("$MINDSDB_API_PORT:HTTP API" "$MINDSDB_MYSQL_PORT:MySQL" "$MINDSDB_MONGODB_PORT:MongoDB")
    local all_ok=true

    for port_info in "${ports[@]}"; do
        local port="${port_info%%:*}"
        local name="${port_info#*:}"

        if nc -z "$MINDSDB_HOST" "$port" 2>/dev/null; then
            print_success "Port $port ($name) is accessible"
        else
            print_error "Port $port ($name) is not accessible"
            all_ok=false
        fi
    done

    if [ "$all_ok" = false ]; then
        print_warning "Some ports are not accessible. This may be normal during startup."
    fi

    echo ""
}

verify_api() {
    print_header "Step 4: Checking MindsDB API"

    local api_url="http://${MINDSDB_HOST}:${MINDSDB_API_PORT}"

    # Check status endpoint
    print_info "Testing status endpoint: ${api_url}/api/status"
    if response=$(curl -s -f "${api_url}/api/status" 2>/dev/null); then
        print_success "MindsDB API is responding"

        if jq_is_runnable; then
            echo "$response" | jq '.' 2>/dev/null || echo "$response"
        else
            echo "$response"
        fi
    else
        print_error "MindsDB API is not responding"
        print_info "Wait a few moments and try again. MindsDB may still be starting up."
        return 1
    fi

    echo ""
}

verify_database() {
    print_header "Step 5: Checking PostgreSQL Database"

    local db_container="mindsdb-db"

    if docker exec "$db_container" pg_isready -U mindsdb -d mindsdb &>/dev/null; then
        print_success "PostgreSQL database is ready"

        # Check database exists and has tables
        local table_count
        table_count=$(docker exec "$db_container" psql -U mindsdb -d mindsdb -t -c "SELECT COUNT(*) FROM information_schema.tables WHERE table_schema = 'public';" 2>/dev/null | tr -d ' ')
        print_info "Database has $table_count tables"
    else
        print_error "PostgreSQL database is not ready"
        return 1
    fi

    echo ""
}

verify_web_ui() {
    print_header "Step 6: Checking MindsDB Web UI"

    local ui_url="http://${MINDSDB_HOST}:${MINDSDB_API_PORT}"

    if curl -s -f -o /dev/null "${ui_url}" 2>/dev/null; then
        print_success "MindsDB Web UI is accessible"
        print_info "Open in browser: ${ui_url}"
    else
        print_warning "MindsDB Web UI may not be accessible yet"
    fi

    echo ""
}

show_example_queries() {
    print_header "Example MindsDB SQL Queries"

    cat <<'EOF'
# Connect to MindsDB via MySQL protocol:
mysql -h localhost -P 47335 -u mindsdb

# Or using docker:
docker exec -it mindsdb mysql -h localhost -P 47335 -u mindsdb

# Example: Connect to MLflow PostgreSQL
CREATE DATABASE mlflow_data
WITH ENGINE = "postgres",
PARAMETERS = {
  "host": "mlflow-db",
  "port": "5432",
  "database": "mlflow",
  "user": "mlflow",
  "password": "changeme"
};

# Example: Create a sentiment analysis model
CREATE MODEL sentiment_model
PREDICT sentiment
USING
  engine = 'huggingface',
  model_name = 'distilbert-base-uncased-finetuned-sst-2-english';

# Example: Use the model for predictions
SELECT text, sentiment
FROM sentiment_model
WHERE text = 'I love this product!';

# List all databases
SHOW DATABASES;

# List all models
SHOW MODELS;

EOF

    echo ""
}

show_integration_examples() {
    print_header "Integration Examples"

    cat <<'EOF'
# 1. Connect to existing ARIA services:

## Connect to MLflow database:
CREATE DATABASE mlflow_data
WITH ENGINE = "postgres",
PARAMETERS = {
  "host": "mlflow-db",
  "port": "5432",
  "database": "mlflow",
  "user": "mlflow",
  "password": "<from-env>"
};

## Connect to MinIO via S3:
CREATE DATABASE minio_artifacts
WITH ENGINE = "s3",
PARAMETERS = {
  "aws_access_key_id": "<from-env>",
  "aws_secret_access_key": "<from-env>",
  "bucket": "aria-models",
  "endpoint_url": "http://minio:9000"
};

# 2. Create ML models for predictions:

## Time-series forecasting:
CREATE MODEL forecast_model
FROM mlflow_data.metrics
PREDICT value
ORDER BY timestamp
WINDOW 10
HORIZON 5;

## Classification:
CREATE MODEL classifier
PREDICT category
USING
  engine = 'lightgbm',
  target = 'category';

# 3. Query predictions like normal SQL:
SELECT timestamp, value, predicted_value
FROM mlflow_data.metrics
JOIN forecast_model;

EOF

    echo ""
}

print_summary() {
    print_header "Summary"

    local data_compose_file="docker-compose.data.yml"
    if [ -f "docker/docker-compose.data.yml" ]; then
        data_compose_file="docker/docker-compose.data.yml"
    fi

    cat <<EOF
MindsDB is now running and ready to use!

Service URLs:
  - HTTP API:    http://${MINDSDB_HOST}:${MINDSDB_API_PORT}
  - Web Studio:  http://${MINDSDB_HOST}:${MINDSDB_API_PORT}
  - MySQL:       mysql://${MINDSDB_HOST}:${MINDSDB_MYSQL_PORT}
  - MongoDB:     mongodb://${MINDSDB_HOST}:${MINDSDB_MONGODB_PORT}

Container Management:
    - Start:   docker compose -f ${data_compose_file} up -d
    - Stop:    docker compose -f ${data_compose_file} down
    - Logs:    docker compose -f ${data_compose_file} logs -f mindsdb
    - Restart: docker compose -f ${data_compose_file} restart mindsdb

MySQL Client Access:
  docker exec -it mindsdb mysql -h localhost -P 47335 -u mindsdb

Documentation:
  - Official Docs:  https://docs.mindsdb.com/
  - Integrations:   https://docs.mindsdb.com/integrations/data-integrations
  - ML Engines:     https://docs.mindsdb.com/integrations/ml-integrations

Next Steps:
  1. Open MindsDB Studio in your browser
  2. Connect to existing ARIA databases (MLflow, MinIO, etc.)
  3. Create your first ML model
  4. Run predictions via SQL queries

EOF
}

# =============================================================================
# Main Execution
# =============================================================================

main() {
    echo ""
    print_header "MindsDB Verification - P2-011"
    echo ""

    verify_dependencies
    verify_containers
    verify_ports
    verify_api || print_warning "API check failed - MindsDB may still be starting"
    verify_database
    verify_web_ui
    show_example_queries
    show_integration_examples
    print_summary

    print_success "Verification complete!"
    echo ""
}

main "$@"
