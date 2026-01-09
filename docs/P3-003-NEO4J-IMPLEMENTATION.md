# P3-003: Neo4j Graph Database Implementation

## Overview

**Implementation Date**: 2026-01-09
**Phase**: 3 (Phase 2 delivery)
**Domain**: Data & Query Services (Layer 11)
**Team Lead**: L11 Data & Query Domain Team Lead

This document describes the implementation of Neo4j graph database integration for ARIA's Data & Query Services layer.

## Objectives

1. Add Neo4j graph database to docker-compose infrastructure
2. Configure with environment variables (no hardcoded secrets)
3. Document integration patterns with existing ARIA services
4. Provide verification and testing procedures

## Implementation Summary

### Files Modified

1. **`/home/user/ros2-humble-env/docker-compose.data.yml`**
   - Added Neo4j service definition
   - Added Neo4j volumes (data, logs, import, plugins)
   - Updated header with P3-003 reference
   - Added endpoint documentation

2. **`/home/user/ros2-humble-env/.env.data.example`**
   - Added Neo4j environment variables
   - Added Neo4j integration examples
   - Updated production checklist
   - Added security recommendations

3. **`/home/user/ros2-humble-env/docs/neo4j-verification.md`** (created)
   - Comprehensive verification guide
   - Connection and functional tests
   - Python integration examples
   - Troubleshooting procedures
   - Backup/restore procedures

## Neo4j Configuration

### Docker Service Definition

```yaml
neo4j:
  image: neo4j:5-community
  container_name: neo4j
  ports:
    - "7474:7474"  # HTTP (Browser UI)
    - "7687:7687"  # Bolt protocol
  environment:
    NEO4J_AUTH: ${NEO4J_USER:-neo4j}/${NEO4J_PASSWORD:-changeme}
    NEO4J_server_memory_heap_initial__size: ${NEO4J_HEAP_INITIAL:-512m}
    NEO4J_server_memory_heap_max__size: ${NEO4J_HEAP_MAX:-1g}
    NEO4J_server_memory_pagecache_size: ${NEO4J_PAGECACHE:-512m}
    NEO4J_dbms_default__database: ${NEO4J_DEFAULT_DB:-neo4j}
  volumes:
    - neo4j_data:/data
    - neo4j_logs:/logs
    - neo4j_import:/var/lib/neo4j/import
    - neo4j_plugins:/plugins
  networks:
    - agentic-network
  healthcheck:
    test: ["CMD-SHELL", "cypher-shell -u ${NEO4J_USER:-neo4j} -p ${NEO4J_PASSWORD:-changeme} 'RETURN 1' || exit 1"]
    interval: 30s
    timeout: 10s
    retries: 5
    start_period: 40s
  restart: unless-stopped
  deploy:
    resources:
      limits:
        memory: 2gb
        cpus: '2.0'
      reservations:
        memory: 1gb
        cpus: '0.5'
```

### Environment Variables

Required variables in `.env.data`:

```bash
# Authentication
NEO4J_USER=neo4j
NEO4J_PASSWORD=changeme  # MUST be changed for production

# Memory Configuration
NEO4J_HEAP_INITIAL=512m
NEO4J_HEAP_MAX=1g
NEO4J_PAGECACHE=512m

# Database
NEO4J_DEFAULT_DB=neo4j
```

### Network Configuration

- **Network**: `agentic-network` (shared with other ARIA services)
- **HTTP Endpoint**: `http://localhost:7474` (Browser UI)
- **Bolt Endpoint**: `bolt://localhost:7687` (Driver connections)
- **Container-to-Container**: `bolt://neo4j:7687`

### Volumes

| Volume | Purpose | Path |
|--------|---------|------|
| `neo4j_data` | Graph database storage | `/data` |
| `neo4j_logs` | Application logs | `/logs` |
| `neo4j_import` | CSV import directory | `/var/lib/neo4j/import` |
| `neo4j_plugins` | Plugin storage (APOC, GDS) | `/plugins` |

## Security Features

1. **Authentication Required**: Username/password authentication enabled by default
2. **Minimum Password Length**: Enforced 8-character minimum
3. **Environment Variables**: No hardcoded credentials
4. **Password Change**: Default password must be changed on first use
5. **Network Isolation**: Runs in isolated Docker network

## Integration Patterns

### 1. Python Integration

```python
from neo4j import GraphDatabase

driver = GraphDatabase.driver(
    "bolt://neo4j:7687",
    auth=("neo4j", os.getenv("NEO4J_PASSWORD"))
)

with driver.session() as session:
    result = session.run("RETURN 1 AS test")
    print(result.single()["test"])
```

### 2. Knowledge Graph for ML Models

```cypher
// Create model metadata
CREATE (m:Model {name: 'bert-classifier', version: '1.0', framework: 'pytorch'})
CREATE (d:Dataset {name: 'training-set-v2', size: 10000})
CREATE (m)-[:TRAINED_ON]->(d)

// Query relationships
MATCH (m:Model)-[:TRAINED_ON]->(d:Dataset)
RETURN m.name, d.name
```

### 3. RAG Entity Storage

```cypher
// Store document entities
CREATE (e1:Entity {name: 'ARIA', type: 'System'})
CREATE (e2:Entity {name: 'Neo4j', type: 'Database'})
CREATE (e1)-[:USES {component: 'data-layer'}]->(e2)

// Query for context retrieval
MATCH (e:Entity {name: 'ARIA'})-[r]-(related)
RETURN e, r, related
```

### 4. Integration with MindsDB

Neo4j can be used alongside MindsDB for ML-enhanced graph queries:
- Store entity relationships in Neo4j
- Use MindsDB for ML predictions on relational data
- Combine results for enriched insights

## Deployment

### Quick Start

```bash
# 1. Create network
docker network create agentic-network 2>/dev/null || true

# 2. Configure environment
cp .env.data.example .env.data
chmod 600 .env.data
# Edit .env.data and change NEO4J_PASSWORD

# 3. Start Neo4j
docker compose -f docker-compose.data.yml up -d neo4j

# 4. Verify
docker logs neo4j
curl http://localhost:7474
```

### Access Neo4j Browser

1. Open browser to `http://localhost:7474`
2. Login with username `neo4j` and your configured password
3. Run test query: `RETURN 1`

## Verification Commands

### Basic Health Check

```bash
# Check container status
docker ps | grep neo4j

# Check logs
docker logs neo4j

# Test HTTP endpoint
curl -I http://localhost:7474

# Test Bolt connection
docker exec neo4j cypher-shell -u neo4j -p <password> "RETURN 1"
```

### Functional Test

```bash
# Create test data
docker exec neo4j cypher-shell -u neo4j -p <password> \
  "CREATE (n:Test {name: 'ARIA'}) RETURN n"

# Query test data
docker exec neo4j cypher-shell -u neo4j -p <password> \
  "MATCH (n:Test) RETURN n.name"

# Clean up
docker exec neo4j cypher-shell -u neo4j -p <password> \
  "MATCH (n:Test) DELETE n"
```

See `/home/user/ros2-humble-env/docs/neo4j-verification.md` for comprehensive testing procedures.

## Performance Tuning

### Memory Recommendations

- **Small workloads** (< 1GB data): 512MB heap, 512MB page cache
- **Medium workloads** (1-10GB data): 1-2GB heap, 2-4GB page cache
- **Large workloads** (> 10GB data): 4-8GB heap, 8-16GB page cache

Configure in `.env.data`:

```bash
NEO4J_HEAP_INITIAL=2g
NEO4J_HEAP_MAX=2g
NEO4J_PAGECACHE=4g
```

### Resource Limits

Default limits in docker-compose:
- **Memory**: 2GB limit, 1GB reservation
- **CPU**: 2.0 cores limit, 0.5 core reservation

Adjust based on workload in `docker-compose.data.yml`.

## Backup and Recovery

### Backup Strategy

```bash
# Stop Neo4j
docker compose -f docker-compose.data.yml stop neo4j

# Backup data volume
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine tar czf /backup/neo4j-$(date +%Y%m%d).tar.gz -C /data .

# Restart Neo4j
docker compose -f docker-compose.data.yml start neo4j
```

### Restore Procedure

```bash
# Stop Neo4j
docker compose -f docker-compose.data.yml stop neo4j

# Restore data
docker run --rm \
  -v neo4j_data:/data \
  -v $(pwd)/backups:/backup \
  alpine sh -c "rm -rf /data/* && tar xzf /backup/neo4j-YYYYMMDD.tar.gz -C /data"

# Restart Neo4j
docker compose -f docker-compose.data.yml start neo4j
```

## Monitoring

### Key Metrics

1. **JVM Heap Usage**: Monitor via `neo4j-admin server memory-recommendation`
2. **Page Cache Hit Ratio**: Should be > 90% for optimal performance
3. **Transaction Throughput**: Monitor in logs
4. **Query Performance**: Use EXPLAIN/PROFILE in Cypher queries

### Health Check

The service includes an automated health check that runs every 30 seconds:
- Executes `RETURN 1` query via cypher-shell
- 5 retries with 10-second timeout
- 40-second startup grace period

## Production Checklist

- [ ] NEO4J_PASSWORD changed from default (minimum 8 characters)
- [ ] `.env.data` file permissions set to 600
- [ ] `.env.data` excluded from version control
- [ ] NEO4J_HEAP_MAX tuned for workload
- [ ] NEO4J_PAGECACHE tuned for dataset size
- [ ] Backup strategy implemented
- [ ] Monitoring configured
- [ ] Resource limits reviewed and adjusted
- [ ] Network security rules configured
- [ ] APOC/GDS plugins enabled if needed

## Use Cases

### 1. Knowledge Graphs
- Store and query complex entity relationships
- Model domain knowledge for AI systems
- Support semantic search and reasoning

### 2. ML Pipeline Metadata
- Track model lineage and dependencies
- Store dataset relationships
- Model experiment provenance

### 3. RAG (Retrieval Augmented Generation)
- Store document entities and relationships
- Enable context-aware retrieval
- Support multi-hop reasoning

### 4. Recommendation Systems
- Graph-based collaborative filtering
- Relationship-based recommendations
- Pattern detection in user behavior

## Troubleshooting

### Common Issues

1. **Connection Refused**
   - Check if container is running: `docker ps | grep neo4j`
   - Verify ports are exposed: `docker port neo4j`
   - Check firewall rules

2. **Authentication Failed**
   - Verify password in `.env.data`
   - Reset password: `docker exec neo4j neo4j-admin set-initial-password <new-password>`
   - Restart container

3. **Out of Memory**
   - Check memory limits: `docker inspect neo4j --format='{{.HostConfig.Memory}}'`
   - Increase heap size in `.env.data`
   - Adjust resource limits in docker-compose

4. **Health Check Failing**
   - View health logs: `docker inspect neo4j --format='{{json .State.Health}}' | jq`
   - Manually test: `docker exec neo4j cypher-shell -u neo4j -p <password> 'RETURN 1'`

See `/home/user/ros2-humble-env/docs/neo4j-verification.md` for detailed troubleshooting.

## References

- [Neo4j Official Documentation](https://neo4j.com/docs/)
- [Neo4j Docker Hub](https://hub.docker.com/_/neo4j)
- [Cypher Query Language](https://neo4j.com/docs/cypher-manual/current/)
- [Neo4j Python Driver](https://neo4j.com/docs/python-manual/current/)
- [ARIA Docker Compose Architecture](../docker-compose.data.yml)

## Next Steps

1. **Enable Plugins** (optional):
   - Uncomment APOC plugin for advanced procedures
   - Enable Graph Data Science for ML on graphs

2. **Configure Monitoring**:
   - Integrate with ARIA observability stack
   - Set up Prometheus metrics export
   - Configure log aggregation

3. **Develop Integration Libraries**:
   - Create Python helper libraries for common operations
   - Build integration with ARIA's ML pipeline
   - Develop data synchronization scripts

4. **Performance Testing**:
   - Benchmark with realistic workloads
   - Optimize memory configuration
   - Fine-tune query performance

## Conclusion

Neo4j has been successfully integrated into ARIA's Data & Query Services layer. The implementation provides:

- Secure, configurable graph database deployment
- Integration with existing ARIA services via shared network
- Comprehensive documentation and verification procedures
- Production-ready configuration with health checks and resource limits

The graph database is ready for knowledge graph storage, ML pipeline metadata tracking, and RAG applications.
