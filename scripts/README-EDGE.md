# Edge Services Scripts

This directory contains automation scripts for ARIA Edge Services (P0-003 & P0-004).

## Scripts

### deploy-edge.sh
**Purpose**: Automated deployment of Kong Gateway and AgentGateway

**Usage**:
```bash
./scripts/deploy-edge.sh
```

**What it does**:
1. Checks prerequisites (Docker, Docker Compose)
2. Creates agentic-network if needed
3. Creates config directories
4. Pulls and starts all services
5. Waits for services to be healthy
6. Displays access information

### verify-edge.sh
**Purpose**: Comprehensive verification of edge services

**Usage**:
```bash
./scripts/verify-edge.sh
```

**Tests**:
- Network existence
- Service health checks
- API endpoint accessibility
- Kong Admin API
- Kong Proxy
- Kong Manager UI
- Konga UI
- AgentGateway API
- AgentGateway Admin API
- AgentGateway Metrics
- Integration tests
- Feature flag verification
- Log error checks

**Output**: Color-coded test results with pass/fail summary

### init-multi-db.sh
**Purpose**: Initialize multiple PostgreSQL databases (Kong & Konga)

**Usage**: Automatically executed by docker-compose

**What it does**:
- Creates Kong database
- Creates Konga database
- Sets proper permissions

## Quick Commands

```bash
# Full deployment
./scripts/deploy-edge.sh

# Verify deployment
./scripts/verify-edge.sh

# View service status
docker compose -f docker-compose.edge.yml ps

# View logs
docker compose -f docker-compose.edge.yml logs -f

# Restart services
docker compose -f docker-compose.edge.yml restart

# Stop services
docker compose -f docker-compose.edge.yml down
```

## Troubleshooting

If deployment fails:
1. Check Docker is running: `docker ps`
2. Check network exists: `docker network ls | grep agentic-network`
3. Check logs: `docker compose -f docker-compose.edge.yml logs`
4. Restart services: `docker compose -f docker-compose.edge.yml restart`

## Files Created

- `deploy-edge.sh` - Deployment automation
- `verify-edge.sh` - Verification testing
- `init-multi-db.sh` - Database initialization
