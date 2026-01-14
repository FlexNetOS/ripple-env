# Troubleshooting Guide

## Common Issues and Solutions

### Container Issues

**Problem:** Container fails to start  
**Solution:** 
```bash
# Check container logs
docker-compose logs <service>

# Check container status
docker ps -a

# Restart container
docker-compose restart <service>
```

**Problem:** Permission denied in container  
**Solution:** 
```bash
# Ensure proper user configuration
docker-compose exec <service> id

# Check file permissions
docker-compose exec <service> ls -la

# Fix permissions if needed
docker-compose exec <service> chmod 755 /path/to/file
```

**Problem:** Container using too much memory  
**Solution:** 
```bash
# Check resource usage
docker stats

# Add resource limits to docker-compose.yml
deploy:
  resources:
    limits:
      memory: 1G
      cpus: '1.0'
```

### Security Issues

**Problem:** Security scan failed  
**Solution:** 
```bash
# Run security fix scripts
./scripts/fix-shell-scripts.sh
./scripts/audit-and-fix-secrets.sh

# Validate fixes
./scripts/validate-critical-fixes.sh
```

**Problem:** Privileged container detected  
**Solution:** 
```bash
# Remove privileged mode
docker-compose.yml:
# Remove: privileged: true
# Add: cap_add: [SYS_PTRACE] # Only if needed
```

**Problem:** Hardcoded secret found  
**Solution:** 
```bash
# Replace with environment variable
# BEFORE: password: "secret123"
# AFTER: password: "${DB_PASSWORD}"

# Add to .env file
DB_PASSWORD=your_secure_password
```

### Configuration Issues

**Problem:** Service configuration error  
**Solution:** 
```bash
# Validate configuration
docker-compose config

# Check YAML syntax
yq eval '.' docker-compose.yml

# Compare with backup
diff docker-compose.yml docker-compose.yml.backup
```

**Problem:** Environment variable not set  
**Solution:** 
```bash
# Check environment file
cat .env

# Validate environment
docker-compose config

# Set missing variable
echo "VARIABLE=value" >> .env
```

### Network Issues

**Problem:** Service can't connect to database  
**Solution:** 
```bash
# Check network connectivity
docker-compose exec <service> ping <database>

# Check service discovery
docker-compose exec <service> nslookup <database>

# Verify database is running
docker-compose ps | grep <database>
```

**Problem:** Port already in use  
**Solution:** 
```bash
# Find process using port
sudo netstat -tulpn | grep :8080

# Change port in docker-compose.yml
ports:
  - "8081:8080"  # Host port 8081 -> Container port 8080
```

### Performance Issues

**Problem:** High CPU usage  
**Solution:** 
```bash
# Check CPU usage
docker stats

# Add CPU limits
deploy:
  resources:
    limits:
      cpus: '2.0'
```

**Problem:** Slow response times  
**Solution:** 
```bash
# Check resource usage
docker stats

# Check logs for errors
docker-compose logs <service>

# Scale service if needed
docker-compose up --scale <service>=3
```

### Database Issues

**Problem:** Database connection refused  
**Solution:** 
```bash
# Check database is running
docker-compose ps | grep postgres

# Check database logs
docker-compose logs postgres

# Verify database initialization
docker-compose exec postgres pg_isready
```

**Problem:** Database corruption  
**Solution:** 
```bash
# Restore from backup
docker-compose down -v
docker-compose up -d postgres

# Run database repair if needed
docker-compose exec postgres pg_dump --help
```

## Emergency Procedures

### System Down
1. Check system status: `docker-compose ps`
2. Check logs: `docker-compose logs`
3. Restart services: `docker-compose restart`
4. Scale up if needed: `docker-compose up --scale service=3`

### Security Breach
1. Isolate affected containers: `docker-compose stop <service>`
2. Review logs: `docker-compose logs <service>`
3. Change credentials: Update .env file
4. Restart with new secrets: `docker-compose up -d`

### Data Loss
1. Check backups: `ls -la backups/`
2. Restore from latest backup
3. Verify data integrity
4. Document incident

## Getting Help

### Documentation
- [Security Guide](security/SECURITY_GUIDE.md)
- [Production Deployment](getting-started/deployment/PRODUCTION_DEPLOYMENT.md)
- [Architecture Overview](architecture/ARCHITECTURE.md)

### Support Channels
- **Security Issues:** security@yourcompany.com
- **Technical Support:** support@yourcompany.com
- **Emergency:** On-call engineer +1-XXX-XXX-XXXX

### Debug Information
When reporting issues, include:
- Docker version: `docker --version`
- Docker Compose version: `docker-compose --version`
- System info: `uname -a`
- Service logs: `docker-compose logs <service>`
- Configuration: `docker-compose config`

## Prevention

### Regular Maintenance
- Update dependencies weekly
- Run security scans daily
- Monitor resource usage
- Review logs regularly

### Best Practices
- Use specific image versions
- Implement proper resource limits
- Follow security guidelines
- Test changes in staging first

### Monitoring
- Set up alerts for failures
- Monitor resource usage
- Track security events
- Log all changes

---

## Service-Specific Troubleshooting

### LocalAI

**Problem:** Model fails to load
```bash
# Check available disk space for models
df -h /var/lib/docker/volumes/

# View LocalAI logs
docker-compose logs localai | grep -i error

# Verify model configuration
cat config/localai/*.yaml

# Check model download status
docker-compose exec localai ls -la /models/
```

**Problem:** Slow inference
```bash
# Check CPU usage
docker stats flexstack-localai

# Increase threads in config
# Edit config/localai/text-small.yaml: threads: 8

# Restart with new config
docker-compose restart localai
```

**Problem:** Out of memory
```bash
# Reduce context size: LOCALAI_CONTEXT_SIZE=1024
# Use smaller model: LOCALAI_MODELS=/aio/cpu/embeddings.yaml
docker-compose up -d localai
```

### AGiXT

**Problem:** Agent not responding
```bash
# Check AGiXT health
curl http://localhost:7437/health

# View agent logs
docker-compose logs agixt

# Restart AGiXT
docker-compose restart agixt
```

**Problem:** Tool execution fails
```bash
# Check tool permissions
docker-compose exec agixt ls -la /agixt/WORKSPACE

# Verify MinIO connection
docker-compose exec agixt curl http://minio:9000/minio/health/live
```

### NATS JetStream

**Problem:** Messages not delivered
```bash
# Check NATS health
curl http://localhost:8222/healthz

# View JetStream info
curl http://localhost:8222/jsz

# Check stream status
nats stream info <stream-name>
```

### Temporal

**Problem:** Workflow stuck
```bash
# Check workflow via UI: http://localhost:8088
# Query workflow status
temporal workflow show -w <workflow-id>

# Check worker logs
docker-compose logs temporal | grep worker
```

### Keycloak

**Problem:** Login fails
```bash
# Check Keycloak logs
docker-compose logs keycloak

# Access admin console: http://localhost:8082/admin
# Default: admin/changeme
```

### Neo4j

**Problem:** Connection refused
```bash
# Check Neo4j is running
docker-compose ps neo4j

# Test connection
docker-compose exec neo4j cypher-shell -u neo4j -p changeme "RETURN 1"
```

### Kong Gateway

**Problem:** Route not found (404)
```bash
# List routes
curl http://localhost:8001/routes

# Check services
curl http://localhost:8001/services
```

### OPA

**Problem:** Policy not taking effect
```bash
# Check OPA health
curl http://localhost:8181/health

# List policies
curl http://localhost:8181/v1/policies

# Test policy
curl -X POST http://localhost:8181/v1/data/ros2/authz/allow \
  -d '{"input": {"user": {"role": "admin"}}}'
```

---

## Log Locations

| Service | Log Access |
|---------|------------|
| All services | `docker-compose logs <service>` |
| LocalAI | stdout (Docker) |
| AGiXT | `/agixt/logs/` |
| Neo4j | `/logs/` volume |
| Kong | `/usr/local/kong/logs/` |
| Grafana | `/var/log/grafana/` |

### Copy Logs Locally
```bash
docker cp flexstack-neo4j:/logs ./neo4j-logs
```

---

## Metrics Endpoints

| Service | URL |
|---------|-----|
| Prometheus | http://localhost:9090/metrics |
| NATS | http://localhost:8222/varz |
| Kong | http://localhost:8001/metrics |
| Node Exporter | http://localhost:9100/metrics |

---

## Quick Diagnostic

```bash
# Overall health
docker-compose ps
docker stats --no-stream

# Network check
docker network inspect flexstack-network

# Volume usage
docker system df -v

# Full diagnostic
docker-compose config && docker-compose logs --tail=50
```
