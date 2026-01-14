# Holochain Bootstrap Server Deployment Guide

## Overview

This guide provides instructions for deploying Holochain bootstrap servers and conductors using Docker Compose. The deployment architecture includes:

1. **Holochain Bootstrap Server**: Provides peer discovery and network coordination services
2. **Holochain Conductor**: The runtime environment for Holochain applications (hApps)
3. **Persistent Storage**: Docker volumes for data persistence

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                Docker Network (agentic-network)          │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────────────────┐  ┌─────────────────────┐  │
│  │ Holochain Bootstrap      │  │ Holochain Conductor │  │
│  │ Server                   │  │                     │  │
│  │ ├─ Port 8888 (HTTP)      │  │ ├─ Port 8889 (App)  │  │
│  │ ├─ Peer Discovery        │◄──┼─ Admin: 8888       │  │
│  │ └─ Network Coordination   │  │ └─ Data Volume      │  │
│  └──────────────────────────┘  └─────────────────────┘  │
│                    ▲                       ▲             │
│                    │                       │             │
│                Health Check               Health Check   │
│                                                           │
└─────────────────────────────────────────────────────────┘
                         ▲
                         │
              Persistent Volume (holochain-data)
```

## Prerequisites

- Docker Engine 20.10+
- Docker Compose 2.0+
- curl (for health checks)
- Access to agentic-network (must be created beforehand or set to external: false)

### Create agentic-network (if needed)

```bash
docker network create agentic-network
```

## Quick Start

### 1. Deploy Bootstrap Server and Conductor

```bash
# Navigate to project root
cd /home/user/ripple-env

# Deploy Holochain services
docker compose -f docker/docker-compose.holochain.yml up -d

# Check status
docker compose -f docker/docker-compose.holochain.yml ps
```

### 2. Verify Services Are Running

```bash
# Check bootstrap server health
curl http://localhost:8888/health

# Check conductor health
curl http://localhost:8889/health

# View container logs
docker compose -f docker/docker-compose.holochain.yml logs -f holochain-bootstrap
docker compose -f docker/docker-compose.holochain.yml logs -f holochain-conductor
```

## Configuration

### Environment Variables

The Holochain conductor accepts the following environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `HOLOCHAIN_BOOTSTRAP_URL` | http://holochain-bootstrap:8888 | Bootstrap server URL |
| `HOLOCHAIN_SIGNAL_URL` | ws://holochain-bootstrap:8888 | WebRTC signal server URL |
| `RIPPLE_DATA` | /data | Base data directory for Holochain |
| `HOLOCHAIN_NETWORK_SEED` | flexstack-default-seed | Network seed for DPKI |

### Conductor Configuration File

The conductor is configured via `/manifests/holochain/conductor.yaml`. Key sections:

#### Admin Interface
```yaml
admin_interfaces:
  - driver:
      type: websocket
      port: 8888
      allowed_origins: "*"
```

#### App Interface
```yaml
app_interfaces:
  - driver:
      type: websocket
      port: 8889
      allowed_origins:
        - "http://localhost:*"
        - "https://localhost:*"
```

#### Network Bootstrap
```yaml
network:
  bootstrap_service: "http://holochain-bootstrap:8888"
  transport_pool:
    - type: webrtc
      signal_url: "ws://holochain-bootstrap:8888"
```

## Volume Management

### Data Volume

The `holochain-data` volume persists:
- Conductor state and metadata
- Installed hApps and cells
- Network gossip state
- Key material (via Lair keystore)

#### Backup Data

```bash
# Create a backup of holochain data
docker run --rm -v holochain-data:/data -v $(pwd):/backup \
  alpine tar czf /backup/holochain-data.tar.gz -C /data .

# Restore from backup
docker run --rm -v holochain-data:/data -v $(pwd):/backup \
  alpine tar xzf /backup/holochain-data.tar.gz -C /data
```

#### Clear Data

```bash
# CAUTION: This removes all persistent data
docker volume rm holochain-data
```

## Health Checks

Both services include health check configurations:

### Bootstrap Server Health

```bash
curl -v http://localhost:8888/health
# Expected response: 200 OK
```

### Conductor Health

```bash
curl -v http://localhost:8889/health
# Expected response: 200 OK
```

### Monitoring Health via Docker

```bash
# Check health status
docker ps --format "table {{.Names}}\t{{.Status}}" | grep holochain

# View health check logs
docker inspect holochain-bootstrap | grep -A 5 "Health"
docker inspect holochain-conductor | grep -A 5 "Health"
```

## Accessing the Conductor

### Admin Interface

Connect to the conductor's admin interface:

```bash
# Using WebSocket client
wscat -c ws://localhost:8888
```

### Example Admin API Call

```bash
curl -X POST http://localhost:8888 \
  -H "Content-Type: application/json" \
  -d '{
    "id": "1",
    "jsonrpc": "2.0",
    "method": "conductor_info",
    "params": {}
  }'
```

### App Interface

Connect to installed hApps:

```bash
# Using WebSocket client
wscat -c ws://localhost:8889

# List installed apps
curl -X POST http://localhost:8889 \
  -H "Content-Type: application/json" \
  -d '{
    "id": "1",
    "jsonrpc": "2.0",
    "method": "list_apps",
    "params": {}
  }'
```

## Network Integration

### Using with ROS2

Holochain can be integrated with ROS2 for distributed robotics:

```bash
# In your ROS2 launch file or scripts
export HOLOCHAIN_BOOTSTRAP_URL=http://holochain-bootstrap:8888
export HOLOCHAIN_SIGNAL_URL=ws://holochain-bootstrap:8888

# Run ROS2 node that communicates with Holochain
ros2 run your_package your_node
```

### Multi-Conductor Setup

Deploy multiple conductors connecting to the same bootstrap server:

```bash
# docker-compose.holochain.yml
services:
  holochain-conductor-1:
    image: holochain/holochain:0.3.0
    environment:
      HOLOCHAIN_BOOTSTRAP_URL: "http://holochain-bootstrap:8888"
      RIPPLE_DATA: /data1
    volumes:
      - holochain-data-1:/data1

  holochain-conductor-2:
    image: holochain/holochain:0.3.0
    environment:
      HOLOCHAIN_BOOTSTRAP_URL: "http://holochain-bootstrap:8888"
      RIPPLE_DATA: /data2
    volumes:
      - holochain-data-2:/data2
```

## Troubleshooting

### Bootstrap Server Not Responding

```bash
# Check bootstrap server logs
docker logs holochain-bootstrap

# Verify port binding
netstat -tulpn | grep 8888

# Check network connectivity
docker exec holochain-conductor curl -v http://holochain-bootstrap:8888/health
```

### Conductor Cannot Connect to Bootstrap

```bash
# Verify conductor logs
docker logs holochain-conductor

# Check environment variables
docker inspect holochain-conductor | grep -i holochain

# Test network connectivity between containers
docker exec holochain-conductor ping holochain-bootstrap
```

### Data Persistence Issues

```bash
# Check volume existence
docker volume ls | grep holochain

# Inspect volume
docker volume inspect holochain-data

# Verify volume is mounted
docker inspect holochain-conductor | grep -A 10 Mounts
```

### Port Conflicts

If ports 8888 or 8889 are already in use:

```bash
# Find what's using the ports
lsof -i :8888
lsof -i :8889

# Update docker-compose.holochain.yml ports section
services:
  holochain-bootstrap:
    ports:
      - "9888:8888"  # External port: Internal port
  holochain-conductor:
    ports:
      - "9889:8889"
```

## Production Deployment

### TLS/mTLS Configuration

For production deployments, enable TLS:

```yaml
admin_interfaces:
  - driver:
      type: websocket
      port: 8888
      tls_cert: /etc/holochain/certs/server.crt
      tls_key: /etc/holochain/certs/server.key

app_interfaces:
  - driver:
      type: websocket
      port: 8889
      tls_cert: /etc/holochain/certs/server.crt
      tls_key: /etc/holochain/certs/server.key
```

### Resource Limits

Add resource constraints to docker-compose.yml:

```yaml
services:
  holochain-bootstrap:
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 4G
        reservations:
          cpus: '1'
          memory: 2G

  holochain-conductor:
    deploy:
      resources:
        limits:
          cpus: '4'
          memory: 8G
        reservations:
          cpus: '2'
          memory: 4G
```

### Logging and Monitoring

Enable centralized logging:

```yaml
services:
  holochain-conductor:
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"
        labels: "service=holochain-conductor"
```

## Cleanup

### Stop Services

```bash
docker compose -f docker/docker-compose.holochain.yml down
```

### Remove Volumes (WARNING: Data Loss)

```bash
docker compose -f docker/docker-compose.holochain.yml down -v
```

### Complete Cleanup

```bash
# Stop and remove services
docker compose -f docker/docker-compose.holochain.yml down -v

# Remove images (if no longer needed)
docker rmi holochain/bootstrap-server:latest
docker rmi holochain/holochain:0.3.0

# Remove unused volumes
docker volume prune

# Remove unused networks
docker network prune
```

## Additional Resources

- [Holochain Documentation](https://developer.holochain.org/)
- [Bootstrap Server Documentation](https://github.com/holochain/bootstrap-server)
- [Conductor Configuration Guide](https://developer.holochain.org/guide/latest/conductor/)
- [BUILDKIT_STARTER_SPEC.md Section 9.7](../BUILDKIT_STARTER_SPEC.md#97-holochain-conductor)
- [Related Manifests](../manifests/holochain/)

## Version Information

- Bootstrap Server: latest
- Holochain: 0.3.0
- Docker Compose: 3.8

## Support

For issues or questions:

1. Check container logs: `docker logs <container-name>`
2. Review [Troubleshooting](#troubleshooting) section
3. Consult [Holochain Documentation](https://developer.holochain.org/)
4. Open an issue in the project repository

## Files Modified

- Created: `/docker/docker-compose.holochain.yml`
- Updated: `/manifests/holochain/conductor.yaml` (bootstrap URLs)
- Created: `/docs/HOLOCHAIN-DEPLOYMENT.md`

## Related Tasks

- **P3-001**: Holochain DNA Development
- **P3-002**: Holochain Zome Implementation
- **P3-003**: Holochain Bootstrap Server Deployment (this document)
- **P3-004**: Conductor Network Configuration
