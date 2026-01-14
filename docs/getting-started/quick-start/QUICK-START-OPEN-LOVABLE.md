# Open-Lovable Quick Start Guide

**AI-Powered Web App Builder** | **Port: 3211** | **Task: P2-012**

---

## 🚀 One-Line Deploy

```bash
docker network create agentic-network && \
docker compose -f docker/docker-compose.localai.yml up -d && \
docker compose -f docker/docker-compose.ui.yml up -d --build open-lovable
```

**Access**: http://localhost:3211

---

## 📋 Prerequisites

```bash
# 1. Create network
docker network create agentic-network

# 2. Start LocalAI (required dependency)
docker compose -f docker/docker-compose.localai.yml up -d
```

---

## 🔧 Build & Deploy

```bash
# Build the service
docker compose -f docker/docker-compose.ui.yml build open-lovable

# Start the service
docker compose -f docker/docker-compose.ui.yml up -d open-lovable

# Watch logs
docker compose -f docker/docker-compose.ui.yml logs -f open-lovable
```

---

## ✅ Verify Installation

```bash
# Automated verification
./scripts/verify-open-lovable.sh

# Manual checks
docker ps | grep open-lovable          # Check if running
curl -I http://localhost:3211          # Test HTTP access
docker logs open-lovable | tail -20    # View recent logs
```

---

## 🌐 Access URLs

| Service | URL | Description |
|---------|-----|-------------|
| **Open-Lovable** | http://localhost:3211 | AI web app builder |
| **Lobe Chat** | http://localhost:3210 | Chat interface |
| **MinIO Console** | http://localhost:9001 | S3 storage |

---

## 🔑 Environment Variables (Optional)

Create `.env` file:

```bash
# Version to build
OPEN_LOVABLE_VERSION=main

# AI Backend (uses LocalAI by default)
OPENAI_API_KEY=sk-dummy-key-for-localai
OPENAI_BASE_URL=http://localai:8080/v1

# Optional: Firecrawl for web scraping
FIRECRAWL_API_KEY=your-key-here
FIRECRAWL_API_URL=https://api.firecrawl.dev

# App URL
OPEN_LOVABLE_APP_URL=http://localhost:3211

# Privacy
OPEN_LOVABLE_TELEMETRY_DISABLED=true
```

---

## 🎯 Usage

1. **Open** http://localhost:3211
2. **Enter** a website URL or describe your app
3. **Generate** modern React application with AI
4. **Download** or deploy your project

---

## 🛠️ Common Commands

```bash
# Restart service
docker compose -f docker/docker-compose.ui.yml restart open-lovable

# Stop service
docker compose -f docker/docker-compose.ui.yml stop open-lovable

# Remove service (keeps volumes)
docker compose -f docker/docker-compose.ui.yml down

# Remove everything (including volumes)
docker compose -f docker/docker-compose.ui.yml down -v
docker volume rm ripple-env_open-lovable-data ripple-env_open-lovable-projects

# Rebuild from scratch
docker compose -f docker/docker-compose.ui.yml build --no-cache open-lovable
docker compose -f docker/docker-compose.ui.yml up -d open-lovable
```

---

## 🐛 Troubleshooting

### Service won't start
```bash
# Check logs
docker compose -f docker/docker-compose.ui.yml logs open-lovable

# Verify LocalAI is running
docker ps | grep localai
```

### Build fails
```bash
# Clear cache and rebuild
docker compose -f docker/docker-compose.ui.yml build --no-cache open-lovable
```

### Can't access web UI
```bash
# Check if port is available
lsof -i :3211 || ss -tlnp | grep 3211

# Check container health
docker inspect open-lovable | grep -A 5 Health
```

### AI features not working
```bash
# Test LocalAI connection
docker exec open-lovable ping -c 3 localai

# Check LocalAI models
curl http://localhost:8080/v1/models
```

---

## 📚 Documentation

- **Full Integration Guide**: `docs/P2-012-OPEN-LOVABLE-INTEGRATION.md`
- **Implementation Summary**: `P2-012-IMPLEMENTATION-SUMMARY.md`
- **Verification Script**: `scripts/verify-open-lovable.sh`
- **Dockerfile**: `config/dockerfiles/Dockerfile.open-lovable`

---

## 🔗 Links

- **Repository**: https://github.com/firecrawl/open-lovable
- **Docker Issue**: https://github.com/firecrawl/open-lovable/issues/74
- **Firecrawl**: https://firecrawl.dev

---

## 📊 Service Info

| Property | Value |
|----------|-------|
| **Container Name** | `open-lovable` |
| **Image** | `aria/open-lovable:latest` |
| **Port** | 3211:3000 |
| **Network** | `agentic-network` |
| **Volumes** | `open-lovable-data`, `open-lovable-projects` |
| **Depends On** | LocalAI |

---

**Status**: ✅ Ready to Deploy | **Task**: P2-012 | **Layer**: L13 UI Domain
