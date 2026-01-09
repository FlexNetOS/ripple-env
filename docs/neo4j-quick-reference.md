# Neo4j Quick Reference

## Service Management

### Start/Stop Neo4j

```bash
# Start Neo4j only
docker compose -f docker-compose.data.yml up -d neo4j

# Start all data services
docker compose -f docker-compose.data.yml up -d

# Stop Neo4j
docker compose -f docker-compose.data.yml stop neo4j

# Restart Neo4j
docker compose -f docker-compose.data.yml restart neo4j

# View logs
docker logs neo4j
docker logs -f neo4j  # Follow logs
```

### Status Checks

```bash
# Check if running
docker ps | grep neo4j

# Health status
docker inspect neo4j --format='{{.State.Health.Status}}'

# Resource usage
docker stats neo4j
```

## Connection Endpoints

| Endpoint | Purpose | URL |
|----------|---------|-----|
| Browser UI | Web interface | http://localhost:7474 |
| Bolt Protocol | Driver connections | bolt://localhost:7687 |
| Container Name | Internal access | neo4j:7687 |

## Cypher Shell Commands

### Access Shell

```bash
# Interactive shell
docker exec -it neo4j cypher-shell -u neo4j -p <password>

# Run single query
docker exec neo4j cypher-shell -u neo4j -p <password> "RETURN 1"
```

### Basic Cypher Queries

```cypher
# Show all nodes (limit 25)
MATCH (n) RETURN n LIMIT 25;

# Count nodes
MATCH (n) RETURN count(n);

# Show all relationships
MATCH ()-[r]->() RETURN type(r), count(r);

# Clear all data (DANGEROUS!)
MATCH (n) DETACH DELETE n;
```

## Common Operations

### Create Node

```cypher
CREATE (n:Person {name: 'Alice', age: 30})
RETURN n;
```

### Create Relationship

```cypher
MATCH (a:Person {name: 'Alice'})
MATCH (b:Person {name: 'Bob'})
CREATE (a)-[r:KNOWS]->(b)
RETURN a, r, b;
```

### Query Pattern

```cypher
MATCH (a:Person)-[r:KNOWS]->(b:Person)
RETURN a.name, b.name;
```

### Update Node

```cypher
MATCH (n:Person {name: 'Alice'})
SET n.age = 31
RETURN n;
```

### Delete Node

```cypher
MATCH (n:Person {name: 'Alice'})
DELETE n;

# Delete with relationships
MATCH (n:Person {name: 'Alice'})
DETACH DELETE n;
```

## Python Integration

### Install Driver

```bash
pip install neo4j
```

### Basic Connection

```python
from neo4j import GraphDatabase
import os

# Connect
driver = GraphDatabase.driver(
    "bolt://localhost:7687",
    auth=("neo4j", os.getenv("NEO4J_PASSWORD"))
)

# Execute query
with driver.session() as session:
    result = session.run("MATCH (n) RETURN count(n) as count")
    count = result.single()["count"]
    print(f"Total nodes: {count}")

driver.close()
```

### Transaction Example

```python
def create_person(tx, name, age):
    result = tx.run(
        "CREATE (p:Person {name: $name, age: $age}) RETURN p",
        name=name, age=age
    )
    return result.single()["p"]

with driver.session() as session:
    person = session.execute_write(create_person, "Alice", 30)
    print(f"Created: {person}")
```

## Environment Variables

```bash
# In .env.data file
NEO4J_USER=neo4j
NEO4J_PASSWORD=your-secure-password
NEO4J_HEAP_INITIAL=512m
NEO4J_HEAP_MAX=1g
NEO4J_PAGECACHE=512m
NEO4J_DEFAULT_DB=neo4j
```

## Backup & Restore

### Backup

```bash
docker compose -f docker-compose.data.yml stop neo4j
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine tar czf /backup/neo4j-$(date +%Y%m%d).tar.gz -C /data .
docker compose -f docker-compose.data.yml start neo4j
```

### Restore

```bash
docker compose -f docker-compose.data.yml stop neo4j
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine sh -c "rm -rf /data/* && tar xzf /backup/neo4j-YYYYMMDD.tar.gz -C /data"
docker compose -f docker-compose.data.yml start neo4j
```

## Troubleshooting

### Reset Password

```bash
docker exec neo4j neo4j-admin set-initial-password <new-password>
docker restart neo4j
```

### Check Configuration

```bash
docker exec neo4j cat /var/lib/neo4j/conf/neo4j.conf | grep memory
```

### View Health Status

```bash
docker inspect neo4j --format='{{json .State.Health}}' | jq
```

### Manual Health Check

```bash
docker exec neo4j cypher-shell -u neo4j -p <password> 'RETURN 1'
```

## Performance Tips

1. **Indexes**: Create indexes for frequently queried properties
   ```cypher
   CREATE INDEX person_name FOR (p:Person) ON (p.name);
   ```

2. **Constraints**: Enforce uniqueness
   ```cypher
   CREATE CONSTRAINT person_id FOR (p:Person) REQUIRE p.id IS UNIQUE;
   ```

3. **EXPLAIN**: Analyze query performance
   ```cypher
   EXPLAIN MATCH (n:Person) RETURN n;
   PROFILE MATCH (n:Person) RETURN n;
   ```

4. **Batching**: Use UNWIND for bulk inserts
   ```cypher
   UNWIND [{name: 'Alice'}, {name: 'Bob'}] AS person
   CREATE (p:Person {name: person.name});
   ```

## Useful Cypher Functions

```cypher
# String functions
RETURN toLower('HELLO');
RETURN toUpper('hello');
RETURN substring('hello', 0, 2);

# Aggregations
MATCH (n:Person)
RETURN count(n), avg(n.age), min(n.age), max(n.age);

# Collections
RETURN size(['a', 'b', 'c']);
RETURN head(['a', 'b', 'c']);
RETURN tail(['a', 'b', 'c']);

# Date/Time
RETURN datetime();
RETURN date();
RETURN timestamp();
```

## Resources

- Full Documentation: `/home/user/ros2-humble-env/docs/neo4j-verification.md`
- Implementation Guide: `/home/user/ros2-humble-env/docs/P3-003-NEO4J-IMPLEMENTATION.md`
- Docker Compose: `/home/user/ros2-humble-env/docker-compose.data.yml`
- Environment Config: `/home/user/ros2-humble-env/.env.data.example`
- Neo4j Docs: https://neo4j.com/docs/
- Cypher Manual: https://neo4j.com/docs/cypher-manual/current/
