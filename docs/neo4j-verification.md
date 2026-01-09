# Neo4j Integration Verification Guide

## Overview
This guide provides verification commands and tests for the Neo4j graph database integration (P3-003) in ARIA's Data & Query Services layer.

## Prerequisites

1. Docker and Docker Compose installed
2. Network created: `docker network create agentic-network`
3. Environment file configured: `cp .env.data.example .env.data`
4. Neo4j password changed in `.env.data`

## Quick Start

### 1. Validate Configuration

```bash
# Validate docker-compose syntax
docker compose -f docker-compose.data.yml config --quiet

# Check if configuration is valid (will output errors if any)
docker compose -f docker-compose.data.yml config
```

### 2. Start Neo4j Service

```bash
# Start only Neo4j
docker compose -f docker-compose.data.yml up -d neo4j

# Start all data services (MindsDB + Neo4j)
docker compose -f docker-compose.data.yml up -d
```

### 3. Check Service Status

```bash
# Check if container is running
docker ps | grep neo4j

# Check logs for any errors
docker logs neo4j

# Check health status
docker inspect neo4j --format='{{.State.Health.Status}}'
```

## Verification Commands

### Connection Tests

#### 1. Test Neo4j Browser (HTTP)

```bash
# Check if HTTP endpoint is responding
curl -I http://localhost:7474

# Expected output: HTTP/1.1 200 OK
```

#### 2. Test Bolt Protocol (Port 7687)

```bash
# Check if Bolt port is open
nc -zv localhost 7687

# Expected output: Connection to localhost 7687 port [tcp/*] succeeded!
```

#### 3. Test Cypher Shell Access

```bash
# Connect using cypher-shell (from inside container)
docker exec -it neo4j cypher-shell -u neo4j -p <your-password>

# Run a test query
# cypher-shell> RETURN 1 AS test;
# Expected output:
# +------+
# | test |
# +------+
# | 1    |
# +------+
```

### Functional Tests

#### 1. Create Test Data

```bash
docker exec -it neo4j cypher-shell -u neo4j -p <your-password> \
  "CREATE (n:TestNode {name: 'ARIA', type: 'System'}) RETURN n;"
```

Expected output:
```
+------------------------------------------------+
| n                                              |
+------------------------------------------------+
| (:TestNode {name: "ARIA", type: "System"})    |
+------------------------------------------------+
```

#### 2. Query Test Data

```bash
docker exec -it neo4j cypher-shell -u neo4j -p <your-password> \
  "MATCH (n:TestNode) RETURN n.name, n.type;"
```

Expected output:
```
+------------------+
| n.name | n.type  |
+------------------+
| "ARIA" | "System"|
+------------------+
```

#### 3. Clean Up Test Data

```bash
docker exec -it neo4j cypher-shell -u neo4j -p <your-password> \
  "MATCH (n:TestNode) DELETE n;"
```

### Python Integration Test

Create a test script `test_neo4j.py`:

```python
from neo4j import GraphDatabase

# Connection configuration
uri = "bolt://localhost:7687"
username = "neo4j"
password = "your-password"  # Use your actual password

# Create driver
driver = GraphDatabase.driver(uri, auth=(username, password))

def test_connection():
    """Test basic Neo4j connection"""
    with driver.session() as session:
        result = session.run("RETURN 1 AS test")
        record = result.single()
        print(f"Connection test: {record['test']}")
        assert record['test'] == 1
        print("✓ Connection successful")

def test_crud_operations():
    """Test Create, Read, Update, Delete operations"""
    with driver.session() as session:
        # Create
        session.run(
            "CREATE (n:TestNode {name: $name, created: timestamp()})",
            name="test-node"
        )
        print("✓ Create successful")

        # Read
        result = session.run("MATCH (n:TestNode {name: $name}) RETURN n", name="test-node")
        node = result.single()
        assert node is not None
        print("✓ Read successful")

        # Update
        session.run(
            "MATCH (n:TestNode {name: $name}) SET n.updated = timestamp()",
            name="test-node"
        )
        print("✓ Update successful")

        # Delete
        session.run("MATCH (n:TestNode {name: $name}) DELETE n", name="test-node")
        print("✓ Delete successful")

def test_relationship():
    """Test relationship creation and traversal"""
    with driver.session() as session:
        # Create nodes with relationship
        session.run("""
            CREATE (a:Model {name: 'bert-classifier'})
            CREATE (b:Dataset {name: 'training-data'})
            CREATE (a)-[:TRAINED_ON]->(b)
        """)
        print("✓ Relationship creation successful")

        # Query relationship
        result = session.run("""
            MATCH (m:Model)-[r:TRAINED_ON]->(d:Dataset)
            RETURN m.name, type(r), d.name
        """)
        record = result.single()
        assert record is not None
        print(f"✓ Relationship query: {record['m.name']} -> {record['d.name']}")

        # Clean up
        session.run("MATCH (n) WHERE n:Model OR n:Dataset DELETE n, n-[*]->()")
        print("✓ Cleanup successful")

if __name__ == "__main__":
    try:
        print("Starting Neo4j verification tests...\n")
        test_connection()
        test_crud_operations()
        test_relationship()
        print("\n✓ All tests passed!")
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
    finally:
        driver.close()
```

Run the test:

```bash
# Install Neo4j Python driver
pip install neo4j

# Run verification script
python test_neo4j.py
```

## Integration with Existing Services

### Access Neo4j from Other Containers

Neo4j is accessible from other containers in the `agentic-network`:

```python
# From any container in agentic-network
uri = "bolt://neo4j:7687"  # Use container name
driver = GraphDatabase.driver(uri, auth=("neo4j", "your-password"))
```

### MindsDB Integration

Neo4j can be used alongside MindsDB for ML-enhanced graph queries. Example workflow:

1. Store entity relationships in Neo4j
2. Use MindsDB for ML predictions
3. Query Neo4j with ML-enriched data

## Performance Verification

### 1. Check Memory Configuration

```bash
docker exec neo4j cat /var/lib/neo4j/conf/neo4j.conf | grep memory
```

### 2. Monitor Resource Usage

```bash
# Container resource usage
docker stats neo4j

# Detailed memory breakdown
docker exec neo4j neo4j-admin server memory-recommendation
```

### 3. Query Performance

```bash
# Run EXPLAIN on a query to see execution plan
docker exec -it neo4j cypher-shell -u neo4j -p <your-password> \
  "EXPLAIN MATCH (n) RETURN count(n);"
```

## Troubleshooting

### Common Issues

1. **Connection refused on port 7474 or 7687**
   ```bash
   # Check if ports are exposed
   docker port neo4j

   # Check firewall rules
   sudo ufw status
   ```

2. **Authentication failed**
   ```bash
   # Reset password (requires restart)
   docker exec neo4j neo4j-admin set-initial-password <new-password>
   docker restart neo4j
   ```

3. **Out of memory errors**
   ```bash
   # Check container memory limits
   docker inspect neo4j --format='{{.HostConfig.Memory}}'

   # Increase heap size in .env.data
   NEO4J_HEAP_MAX=2g
   ```

4. **Health check failing**
   ```bash
   # Check health check logs
   docker inspect neo4j --format='{{json .State.Health}}' | jq

   # Manually run health check command
   docker exec neo4j cypher-shell -u neo4j -p <password> 'RETURN 1'
   ```

## Security Checklist

- [ ] Default password changed from `changeme`
- [ ] `.env.data` file permissions set to 600: `chmod 600 .env.data`
- [ ] `.env.data` excluded from version control (in .gitignore)
- [ ] Strong password (minimum 8 characters, include numbers and symbols)
- [ ] Bolt and HTTP endpoints only exposed on localhost (for production, use reverse proxy)
- [ ] Regular backups configured for `neo4j_data` volume
- [ ] Consider enabling authentication for production use
- [ ] Review and restrict cypher query permissions as needed

## Backup and Restore

### Backup

```bash
# Stop Neo4j
docker compose -f docker-compose.data.yml stop neo4j

# Backup data volume
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine tar czf /backup/neo4j-backup-$(date +%Y%m%d).tar.gz -C /data .

# Restart Neo4j
docker compose -f docker-compose.data.yml start neo4j
```

### Restore

```bash
# Stop Neo4j
docker compose -f docker-compose.data.yml stop neo4j

# Restore data volume
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine sh -c "rm -rf /data/* && tar xzf /backup/neo4j-backup-YYYYMMDD.tar.gz -C /data"

# Restart Neo4j
docker compose -f docker-compose.data.yml start neo4j
```

## Monitoring

### Key Metrics to Monitor

1. **JVM Heap Usage**
   ```bash
   docker exec neo4j neo4j-admin server memory-recommendation
   ```

2. **Page Cache Hit Ratio** (should be > 90% for good performance)
   ```cypher
   CALL dbms.queryJmx("org.neo4j:*") YIELD attributes
   RETURN attributes
   ```

3. **Transaction Throughput**
   ```bash
   docker logs neo4j | grep "transactions"
   ```

4. **Query Performance**
   - Enable query logging in Neo4j config
   - Use EXPLAIN/PROFILE in Cypher queries

## References

- [Neo4j Documentation](https://neo4j.com/docs/)
- [Neo4j Docker Hub](https://hub.docker.com/_/neo4j)
- [Cypher Query Language](https://neo4j.com/docs/cypher-manual/current/)
- [Neo4j Python Driver](https://neo4j.com/docs/python-manual/current/)
- [Neo4j Operations Manual](https://neo4j.com/docs/operations-manual/current/)
