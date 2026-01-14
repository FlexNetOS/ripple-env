# Observability Stack Quick Start Guide

## Services Overview

| Service | URL | Purpose | Container |
|---------|-----|---------|-----------|
| **Prometheus** | http://localhost:9090 | Metrics collection | `prometheus` |
| **Grafana** | http://localhost:3000 | Dashboards & visualization | `grafana` |
| **Loki** | http://localhost:3100 | Log aggregation | `loki` |
| **Alertmanager** | http://localhost:9093 | Alert management | `alertmanager` |
| **Node Exporter** | http://localhost:9100/metrics | Host metrics | `node-exporter` |
| **cAdvisor** | http://localhost:8080 | Container metrics | `cadvisor` |
| **OTel Collector** | http://localhost:4317 (gRPC) | Telemetry pipeline | `otel-collector` |
| **Netdata** ⭐ | http://localhost:19999 | Real-time monitoring | `netdata` |
| **Umami** ⭐ | http://localhost:3001 | Web analytics | `umami` |

⭐ = Newly added (P1-008, P1-009)

## Quick Start

```bash
# 1. Setup environment
cp .env.example .env

# 2. Generate secure secrets for Umami
export UMAMI_APP_SECRET=$(openssl rand -base64 32)
export UMAMI_DB_PASSWORD=$(openssl rand -base64 32)
echo "UMAMI_APP_SECRET=${UMAMI_APP_SECRET}" >> .env
echo "UMAMI_DB_PASSWORD=${UMAMI_DB_PASSWORD}" >> .env

# 3. Create network
docker network create agentic-network 2>/dev/null || true

# 4. Start all services
docker compose -f docker/docker-compose.observability.yml up -d

# 5. Verify
./scripts/verify-observability.sh
```

## Service Access

### Grafana
- URL: http://localhost:3000
- Default login: `admin` / `admin`
- Change password on first login

### Netdata (NEW - P1-008)
- URL: http://localhost:19999
- No authentication required (local access)
- Features:
  - Real-time system metrics (1s resolution)
  - Docker container monitoring
  - 2000+ metrics collected automatically
  - Prometheus-compatible metrics export

### Umami (NEW - P1-009)
- URL: http://localhost:3001
- Default login: `admin` / `umami`
- **IMPORTANT**: Change password immediately!
- Features:
  - Privacy-focused web analytics
  - GDPR compliant
  - No cookies, anonymous tracking
  - Multiple website support

## Configuration Files

```
ripple-env/
├── docker/docker-compose.observability.yml       # Main service definitions
├── .env.example                           # Environment template
├── .env                                   # Your secrets (create from .example)
├── manifests/observability/
│   ├── prometheus.yml                     # Prometheus config
│   ├── loki.yml                          # Loki config
│   ├── promtail.yml                      # Promtail config
│   ├── alertmanager.yml                  # Alertmanager config
│   ├── otel-collector.yaml               # OTel Collector config
│   └── grafana/provisioning/
│       ├── datasources/
│       │   └── datasources.yml           # Auto-configured datasources
│       └── dashboards/
│           └── dashboards.yml            # Dashboard loader
└── scripts/
    └── verify-observability.sh           # Verification script
```

## Environment Variables

### Required for Production

```bash
# Grafana
GRAFANA_ADMIN_USER=admin
GRAFANA_ADMIN_PASSWORD=<secure-password>

# Umami (P1-009)
UMAMI_DB_USER=umami
UMAMI_DB_PASSWORD=<secure-password>
UMAMI_DB_NAME=umami
UMAMI_APP_SECRET=<secure-random-string>

# Netdata Cloud (Optional - P1-008)
NETDATA_CLAIM_TOKEN=<your-token-from-netdata-cloud>
```

### Generate Secure Values

```bash
# Password generation
openssl rand -base64 32

# Alternative
pwgen -s 32 1
```

## Common Commands

### Start/Stop

```bash
# Start all
docker compose -f docker/docker-compose.observability.yml up -d

# Start specific service
docker compose -f docker/docker-compose.observability.yml up -d netdata
docker compose -f docker/docker-compose.observability.yml up -d umami

# Stop all
docker compose -f docker/docker-compose.observability.yml down

# Stop with volume removal (CAUTION: deletes data!)
docker compose -f docker/docker-compose.observability.yml down -v
```

### Logs

```bash
# All services
docker compose -f docker/docker-compose.observability.yml logs -f

# Specific service
docker compose -f docker/docker-compose.observability.yml logs -f netdata
docker compose -f docker/docker-compose.observability.yml logs -f umami

# Last 100 lines
docker compose -f docker/docker-compose.observability.yml logs --tail=100
```

### Status

```bash
# All services status
docker compose -f docker/docker-compose.observability.yml ps

# Health checks
docker ps --filter "name=netdata" --format "{{.Names}}: {{.Status}}"
docker ps --filter "name=umami" --format "{{.Names}}: {{.Status}}"
```

### Restart

```bash
# Restart specific service
docker compose -f docker/docker-compose.observability.yml restart netdata
docker compose -f docker/docker-compose.observability.yml restart umami

# Restart all
docker compose -f docker/docker-compose.observability.yml restart
```

## Netdata Setup (P1-008)

### Access Real-Time Metrics

1. Open http://localhost:19999
2. Navigate through sections:
   - System Overview
   - CPU Performance
   - Memory Usage
   - Disk I/O
   - Network Traffic
   - Docker Containers

### Connect to Netdata Cloud (Optional)

1. Sign up at https://app.netdata.cloud
2. Create a Space
3. Get claim token from UI
4. Add to `.env`:
   ```bash
   NETDATA_CLAIM_TOKEN=<your-token>
   ```
5. Restart:
   ```bash
   docker compose -f docker/docker-compose.observability.yml restart netdata
   ```

### Use in Grafana

Netdata is auto-configured as a Grafana datasource:
- Name: `Netdata`
- Type: Prometheus
- URL: http://localhost:19999/api/v1/allmetrics?format=prometheus
- Update interval: 5s

Create dashboards using Netdata metrics!

## Umami Setup (P1-009)

### Initial Configuration

1. Access http://localhost:3001
2. Login with:
   - Username: `admin`
   - Password: `umami`
3. **Change password immediately!**
4. Go to Settings → Profile → Change Password

### Add Website to Track

1. Click "Settings" → "Websites"
2. Click "Add Website"
3. Enter:
   - Name: Your website name
   - Domain: your-domain.com
4. Get tracking code
5. Add to your website:
   ```html
   <script async defer
     data-website-id="<your-id>"
     src="http://localhost:3001/script.js">
   </script>
   ```

### View Analytics

- Dashboard: Real-time visitor count
- Pages: Most visited pages
- Referrers: Traffic sources
- Browsers: User agents
- OS: Operating systems
- Devices: Desktop/mobile/tablet
- Countries: Geographic data

## Integration with ARIA Platform

### Prometheus Scraping

Add to `manifests/observability/prometheus.yml`:

```yaml
scrape_configs:
  - job_name: 'netdata'
    metrics_path: '/api/v1/allmetrics'
    params:
      format: ['prometheus']
    static_configs:
      - targets: ['localhost:19999']
    scrape_interval: 5s
```

### Grafana Datasources

Auto-configured datasources:
1. **Prometheus** (default) - Application metrics
2. **Loki** - Log aggregation
3. **Netdata** (NEW) - Real-time system metrics

### Alert Rules

Create alerts in Alertmanager for:
- High CPU usage (via Netdata)
- Memory pressure (via Netdata)
- Disk space warnings (via Netdata)
- Container failures (via cAdvisor)

## Troubleshooting

### Netdata Not Starting

```bash
# Check logs
docker compose -f docker/docker-compose.observability.yml logs netdata

# Common issue: Insufficient privileges
# Solution: Verify capabilities in docker-compose.yml
# Required: SYS_PTRACE, SYS_ADMIN
```

### Umami Database Connection Failed

```bash
# Check database health
docker compose -f docker/docker-compose.observability.yml ps umami-db

# Restart database
docker compose -f docker/docker-compose.observability.yml restart umami-db

# Wait for health check, then restart Umami
docker compose -f docker/docker-compose.observability.yml restart umami
```

### Grafana Can't Connect to Netdata

```bash
# Verify Netdata is running
curl http://localhost:19999/api/v1/allmetrics?format=prometheus

# Check Grafana datasource config
docker exec grafana cat /etc/grafana/provisioning/datasources/datasources.yml

# Restart Grafana
docker compose -f docker/docker-compose.observability.yml restart grafana
```

### Service Won't Start

```bash
# Check resource usage
docker stats

# Check network
docker network inspect agentic-network

# Recreate service
docker compose -f docker/docker-compose.observability.yml up -d --force-recreate <service>
```

## Security Best Practices

1. **Change Default Passwords**
   - Grafana: admin/admin → secure password
   - Umami: admin/umami → secure password

2. **Use Strong Secrets**
   ```bash
   openssl rand -base64 32
   ```

3. **Protect .env File**
   ```bash
   chmod 600 .env
   ```

4. **Use Vault for Production**
   ```bash
   vault kv put secret/observability/umami/app-secret value="$(openssl rand -base64 32)"
   ```

5. **Enable TLS**
   - Configure reverse proxy (Kong/Nginx)
   - Use Let's Encrypt certificates
   - Enforce HTTPS

6. **Network Isolation**
   - Services use `agentic-network`
   - No direct external access
   - Access via Kong gateway

## Maintenance

### Backup

```bash
# Netdata config
docker run --rm -v netdata-config:/data -v $(pwd):/backup alpine tar czf /backup/netdata-config.tar.gz /data

# Umami database
docker exec umami-db pg_dump -U umami umami > umami-backup-$(date +%Y%m%d).sql
```

### Update

```bash
# Pull latest images
docker compose -f docker/docker-compose.observability.yml pull

# Recreate containers
docker compose -f docker/docker-compose.observability.yml up -d --force-recreate
```

### Cleanup

```bash
# Remove stopped containers
docker compose -f docker/docker-compose.observability.yml rm

# Remove unused volumes (CAUTION!)
docker volume prune

# Remove unused images
docker image prune -a
```

## Resources

- [Prometheus Documentation](https://prometheus.io/docs/)
- [Grafana Documentation](https://grafana.com/docs/)
- [Netdata Documentation](https://learn.netdata.cloud/)
- [Umami Documentation](https://umami.is/docs)
- [OpenTelemetry Documentation](https://opentelemetry.io/docs/)
- [Loki Documentation](https://grafana.com/docs/loki/latest/)

## Support

For issues or questions:
1. Check logs: `docker compose -f docker/docker-compose.observability.yml logs`
2. Run verification: `./scripts/verify-observability.sh`
3. Consult documentation: `/docs/P1-008-009-IMPLEMENTATION.md`
4. Review audit report: `ARIA_AUDIT_REPORT.md`

---

**Last Updated**: 2026-01-09
**Status**: ✅ P1-008 (Netdata) and P1-009 (Umami) Complete
