# Production Deployment Guide

## Pre-Deployment Security Checklist

- [ ] All default passwords changed from .env.example
- [ ] SSL/TLS certificates configured
- [ ] Firewall rules applied
- [ ] Secrets stored in Vault or secure secret management
- [ ] Monitoring configured
- [ ] Backup procedures tested
- [ ] Security scanning completed
- [ ] Access controls configured
- [ ] Network segmentation implemented
- [ ] Incident response plan ready

## Deployment Steps

### 1. Environment Preparation

```bash
# Clone repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# Copy environment template
cp .env.example .env

# Generate secure passwords
./scripts/generate-secrets.sh > .env.secure

# Copy appropriate secrets to .env
cp .env.secure .env

# Set proper file permissions
chmod 600 .env
chmod 700 secrets/
```

### 2. Security Configuration

```bash
# Configure firewall
sudo ufw default deny incoming
sudo ufw allow 22/tcp    # SSH
sudo ufw allow 80/tcp    # HTTP
sudo ufw allow 443/tcp   # HTTPS
sudo ufw allow 8080/tcp  # Application specific
sudo ufw enable

# Configure SSL/TLS
# Place certificates in secrets/ directory
chmod 600 secrets/*.pem
chmod 600 secrets/*.key
```

### 3. Secret Management

```bash
# For development (use proper secret management in production)
# HashiCorp Vault integration
vault secrets enable -path=secret kv
vault kv put secret/aria/keycloak admin_password="$(openssl rand -base64 32)"

# AWS Secrets Manager (alternative)
aws secretsmanager create-secret --name aria/keycloak --secret-string "$(openssl rand -base64 32)"
```

### 4. Deploy Services

```bash
# Deploy with security configurations

# Deploy (baseline)
docker compose -f docker/docker-compose.yml up -d

# Optional: add a security-hardening override file (example shown below)
# docker compose -f docker/docker-compose.yml -f docker/docker-compose.security.yml up -d

# Or use the provided deployment script
./scripts/deploy-production.sh
```

### 5. Security Validation

```bash
# Run security validation
./scripts/validate-security.sh

# Run comprehensive security scan
./scripts/validate-critical-fixes.sh

# Check container security
docker run --rm -v /var/run/docker.sock:/var/run/docker.sock aquasec/trivy image <your-image>
```

## Security Configuration

### Container Security

```yaml
# docker-compose.security.yml (example override file)
version: '3.8'
services:
  app:
    # Run as non-root user
    user: 1001:1001
    
    # Security options
    security_opt:
      - no-new-privileges:true
    
    # Capabilities
    cap_drop:
      - ALL
    cap_add:
      - CHOWN
      - SETGID
      - SETUID
    
    # Read-only filesystem
    read_only: true
    
    # Resource limits
    deploy:
      resources:
        limits:
          cpus: '2.0'
          memory: 2G
        reservations:
          cpus: '1.0'
          memory: 1G
    
    # Health check
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s
```

### Network Security

```yaml
# Network segmentation
networks:
  frontend:
    driver: bridge
  backend:
    driver: bridge
    internal: true  # No internet access
  database:
    driver: bridge
    internal: true

services:
  web:
    networks:
      - frontend
      - backend
  
  app:
    networks:
      - backend
      - database
  
  database:
    networks:
      - database
```

### SSL/TLS Configuration

```nginx
# nginx.conf
server {
    listen 443 ssl http2;
    
    # SSL configuration
    ssl_certificate /etc/ssl/certs/server.crt;
    ssl_certificate_key /etc/ssl/private/server.key;
    
    # Security headers
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains" always;
    add_header X-Content-Type-Options nosniff;
    add_header X-Frame-Options DENY;
    add_header X-XSS-Protection "1; mode=block";
    add_header Content-Security-Policy "default-src 'self'";
    
    # SSL protocols and ciphers
    ssl_protocols TLSv1.2 TLSv1.3;
    ssl_ciphers ECDHE-RSA-AES128-GCM-SHA256:ECDHE-RSA-AES256-GCM-SHA384;
    ssl_prefer_server_ciphers off;
}
```

## Monitoring and Logging

### Security Monitoring

```yaml
# docker-compose.monitoring.yml
version: '3.8'
services:
  prometheus:
    image: prom/prometheus:latest
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
    ports:
      - "9090:9090"
  
  grafana:
    image: grafana/grafana:latest
    environment:
      - GF_SECURITY_ADMIN_PASSWORD=${GRAFANA_PASSWORD}
    ports:
      - "3000:3000"
    volumes:
      - grafana-storage:/var/lib/grafana
```

### Audit Logging

```bash
# Enable audit logging
docker-compose exec service auditctl -w /etc/passwd -p wa -k identity
docker-compose exec service auditctl -w /etc/shadow -p wa -k identity
```

## Backup and Recovery

### Backup Strategy

```bash
# Database backup
docker-compose exec postgres pg_dump -U postgres database > backup.sql

# Volume backup
docker run --rm -v ripple-env_data:/data -v $(pwd):/backup alpine tar czf /backup/data-backup.tar.gz /data

# Configuration backup
tar czf config-backup.tar.gz docker/ .env config/
```

### Recovery Procedures

```bash
# Database restore
docker-compose exec -T postgres psql -U postgres database < backup.sql

# Volume restore
docker run --rm -v ripple-env_data:/data -v $(pwd):/backup alpine tar xzf /backup/data-backup.tar.gz

# Configuration restore
tar xzf config-backup.tar.gz
```

## Scaling

### Horizontal Scaling

```bash
# Scale specific services
docker-compose up --scale web=3 --scale app=2 -d

# Use load balancer
# Configure nginx upstream to multiple app instances
```

### Vertical Scaling

```bash
# Update resource limits
docker-compose up -d --force-recreate
```

## Troubleshooting

### Common Issues

**High Memory Usage:**
```bash
# Check memory usage
docker stats

# Identify memory leaks
docker-compose exec app ps aux --sort=-%mem
```

**Security Alerts:**
```bash
# Check security logs
docker-compose logs security

# Run security scan
trivy image <image-name>
```

**Performance Issues:**
```bash
# Check performance metrics
curl http://localhost:9090/metrics

# Profile application
docker-compose exec app profiler
```

## Maintenance

### Regular Tasks

**Weekly:**
- Update dependencies
- Review security logs
- Check resource usage
- Test backup procedures

**Monthly:**
- Security audit
- Performance review
- Capacity planning
- Documentation update

**Quarterly:**
- Penetration testing
- Disaster recovery drill
- Security training
- Compliance audit

### Emergency Procedures

**Security Incident:**
1. Isolate affected services
2. Review logs
3. Change credentials
4. Apply security patches
5. Document incident

**System Outage:**
1. Check service status
2. Review logs
3. Restore from backup if needed
4. Communicate with users
5. Document root cause

## Support

### Documentation
- [Security Guide](SECURITY_GUIDE.md)
- [Troubleshooting Guide](TROUBLESHOOTING.md)
- [Architecture Overview](ARCHITECTURE.md)

### Emergency Contacts
- **Security Team:** security@yourcompany.com
- **DevOps Team:** devops@yourcompany.com
- **On-Call Engineer:** +1-XXX-XXX-XXXX

### Escalation
1. Review documentation
2. Check logs and metrics
3. Contact support team
4. Escalate to engineering
5. Engage external security team if needed

---

*This guide should be reviewed and updated regularly to reflect changes in security best practices and system architecture.*
