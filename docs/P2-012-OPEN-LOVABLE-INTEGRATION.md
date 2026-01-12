# P2-012: Open-Lovable Integration

**Status**: ✅ Implemented
**Priority**: P2 (Medium)
**Layer**: L13 (UI Domain)
**Owner**: L13 UI Domain Team Lead

## Overview

Open-Lovable is an AI-powered web app builder that allows users to clone and recreate any website as a modern React application. This integration adds open-lovable to the ARIA UI stack, enabling automated web application generation powered by LocalAI.

## Repository

- **GitHub**: [firecrawl/open-lovable](https://github.com/firecrawl/open-lovable)
- **Description**: Clone and recreate any website as a modern React app in seconds
- **Technology Stack**: Next.js, React, TypeScript, pnpm

## Implementation Details

### 1. Docker Configuration

Since open-lovable does not have an official Docker image (see [issue #74](https://github.com/firecrawl/open-lovable/issues/74)), we created a custom multi-stage Dockerfile:

**File**: `/home/user/ripple-env/config/dockerfiles/Dockerfile.open-lovable`

**Features**:
- Multi-stage build for optimized image size
- Node.js 20 Alpine base for minimal footprint
- Built from source using pnpm
- Non-root user (nextjs:nodejs) for security
- Health check endpoint configured
- Production-optimized build

### 2. Docker Compose Service

**File**: `/home/user/ripple-env/docker-compose.ui.yml`

**Service Configuration**:
```yaml
open-lovable:
  build:
    context: ./config/dockerfiles
    dockerfile: Dockerfile.open-lovable
    args:
      OPEN_LOVABLE_VERSION: ${OPEN_LOVABLE_VERSION:-main}
  image: aria/open-lovable:latest
  container_name: open-lovable
  ports:
    - "3211:3000"
  environment:
    NODE_ENV: production
    PORT: 3000
    OPENAI_API_KEY: ${OPENAI_API_KEY:-sk-dummy-key-for-localai}
    OPENAI_BASE_URL: ${OPENAI_BASE_URL:-http://localai:8080/v1}
    FIRECRAWL_API_KEY: ${FIRECRAWL_API_KEY:-}
    FIRECRAWL_API_URL: ${FIRECRAWL_API_URL:-https://api.firecrawl.dev}
    NEXT_PUBLIC_APP_URL: ${OPEN_LOVABLE_APP_URL:-http://localhost:3211}
    TELEMETRY_DISABLED: ${OPEN_LOVABLE_TELEMETRY_DISABLED:-true}
  volumes:
    - open-lovable-data:/app/.next/cache
    - open-lovable-projects:/app/projects
  networks:
    - agentic-network
  depends_on:
    - localai
```

**Key Features**:
- **Port**: 3211 (external) → 3000 (internal)
- **AI Backend**: LocalAI integration for fully local operation
- **Persistence**: Two volumes for cache and project storage
- **Network**: Connected to `agentic-network` for service communication
- **Health Checks**: Automatic health monitoring with 30s intervals

### 3. Environment Variables

**File**: `/home/user/ripple-env/.env.example`

| Variable | Default | Description |
|----------|---------|-------------|
| `OPEN_LOVABLE_VERSION` | `main` | Git branch or tag to build from |
| `OPENAI_API_KEY` | `sk-dummy-key-for-localai` | API key for LocalAI (shared with Lobe Chat) |
| `OPENAI_BASE_URL` | `http://localai:8080/v1` | LocalAI endpoint URL |
| `FIRECRAWL_API_KEY` | _(empty)_ | Optional API key for Firecrawl web scraping |
| `FIRECRAWL_API_URL` | `https://api.firecrawl.dev` | Firecrawl API endpoint |
| `OPEN_LOVABLE_APP_URL` | `http://localhost:3211` | Public-facing application URL |
| `OPEN_LOVABLE_TELEMETRY_DISABLED` | `true` | Disable telemetry for privacy |

### 4. Storage Volumes

Two Docker volumes are configured for data persistence:

1. **open-lovable-data**: Next.js build cache
2. **open-lovable-projects**: Generated React projects and application data

## Integration with Existing Stack

### LocalAI Integration

Open-Lovable is configured to use LocalAI as its AI backend, enabling:
- Fully local operation (no external API calls)
- Consistent AI provider across the UI stack
- No external API key requirements
- Privacy-focused deployment

### Network Integration

Connects to the `agentic-network` Docker network, allowing communication with:
- **LocalAI**: For AI-powered generation
- **Lobe Chat**: Potential future integrations
- **Other UI services**: Shared infrastructure

### Service Dependencies

- **Depends on**: LocalAI (required for AI functionality)
- **Optional**: Firecrawl API (for enhanced web scraping)

## Usage

### Quick Start

1. **Ensure prerequisites are running**:
   ```bash
   docker network create agentic-network
   docker-compose -f docker-compose.localai.yml up -d
   ```

2. **Build and start open-lovable**:
   ```bash
   docker-compose -f docker-compose.ui.yml up -d --build open-lovable
   ```

3. **Access the application**:
   ```
   http://localhost:3211
   ```

### Workflow

1. **Open the application** at http://localhost:3211
2. **Enter a website URL** or describe the web app you want to create
3. **AI generates** a modern React application based on your input
4. **Review and download** the generated project
5. **Projects are saved** in the `open-lovable-projects` volume for persistence

### Optional: Firecrawl Integration

For enhanced web scraping capabilities:

1. **Sign up** at [firecrawl.dev](https://firecrawl.dev)
2. **Get an API key**
3. **Add to `.env` file**:
   ```bash
   FIRECRAWL_API_KEY=your-api-key-here
   ```
4. **Restart the service**:
   ```bash
   docker-compose -f docker-compose.ui.yml restart open-lovable
   ```

## Verification Commands

### 1. Build the Service

```bash
docker-compose -f docker-compose.ui.yml build open-lovable
```

**Expected Output**: Successful build completion with image `aria/open-lovable:latest`

### 2. Check Service Status

```bash
docker-compose -f docker-compose.ui.yml ps open-lovable
```

**Expected Output**: Container running with status `Up` and healthy

### 3. Verify Health Check

```bash
docker inspect open-lovable | grep -A 5 "Health"
```

**Expected Output**: Health status showing as `healthy`

### 4. Check Logs

```bash
docker-compose -f docker-compose.ui.yml logs -f open-lovable
```

**Expected Output**: Application startup logs with no errors

### 5. Test Application Access

```bash
curl -I http://localhost:3211
```

**Expected Output**: HTTP 200 response

### 6. Verify LocalAI Connection

```bash
docker exec open-lovable wget -q -O- http://localai:8080/v1/models
```

**Expected Output**: JSON response with available LocalAI models

### 7. Inspect Volumes

```bash
docker volume ls | grep open-lovable
```

**Expected Output**:
```
ripple-env_open-lovable-data
ripple-env_open-lovable-projects
```

### 8. Check Network Connectivity

```bash
docker network inspect agentic-network | grep open-lovable
```

**Expected Output**: Container listed in network members

## Security Considerations

### Implemented Security Measures

1. **Non-root User**: Application runs as `nextjs` user (UID 1001)
2. **Telemetry Disabled**: No data sent to external services by default
3. **Local AI Backend**: Uses LocalAI instead of external OpenAI API
4. **Network Isolation**: Runs in dedicated `agentic-network`
5. **Volume Permissions**: Proper ownership and access controls
6. **Health Monitoring**: Automatic health checks for availability

### Recommended Production Hardening

1. **Enable authentication**: Add auth layer (Keycloak integration)
2. **SSL/TLS**: Use reverse proxy (Kong/Traefik) with HTTPS
3. **Rate limiting**: Implement request rate limits
4. **Resource limits**: Add CPU/memory constraints in docker-compose
5. **Secrets management**: Use Vault for API keys
6. **Image scanning**: Regular Trivy scans for vulnerabilities

## Troubleshooting

### Build Failures

**Issue**: Dockerfile build fails during pnpm install

**Solution**:
```bash
# Rebuild without cache
docker-compose -f docker-compose.ui.yml build --no-cache open-lovable
```

### Service Won't Start

**Issue**: Container exits immediately

**Solution**:
```bash
# Check logs
docker-compose -f docker-compose.ui.yml logs open-lovable

# Verify LocalAI is running
docker-compose -f docker-compose.localai.yml ps
```

### Cannot Access Application

**Issue**: http://localhost:3211 not responding

**Solution**:
```bash
# Verify port binding
docker ps | grep open-lovable

# Check if port is in use
lsof -i :3211

# Restart service
docker-compose -f docker-compose.ui.yml restart open-lovable
```

### AI Generation Not Working

**Issue**: Application loads but AI features fail

**Solution**:
```bash
# Verify LocalAI is accessible
docker exec open-lovable ping -c 3 localai

# Check environment variables
docker exec open-lovable env | grep OPENAI

# Verify LocalAI has models loaded
curl http://localhost:8080/v1/models
```

## Future Enhancements

### Planned Improvements

1. **Multi-model Support**: Configure different LocalAI models for different tasks
2. **Template Library**: Pre-built templates for common application types
3. **Version Control**: Git integration for generated projects
4. **Collaboration**: Multi-user support with Keycloak authentication
5. **Export Options**: Direct deployment to cloud platforms
6. **CI/CD Integration**: Automatic testing of generated applications

### Integration Opportunities

1. **Lobe Chat Integration**: Generate UIs from chat conversations
2. **AGiXT Integration**: Agentic workflows for complex app generation
3. **n8n Integration**: Automated app generation workflows
4. **Temporal Integration**: Long-running generation tasks
5. **Argo CD**: GitOps deployment of generated applications

## References

- **GitHub Repository**: https://github.com/firecrawl/open-lovable
- **Docker Support Issue**: https://github.com/firecrawl/open-lovable/issues/74
- **ARIA Documentation**: `/home/user/ripple-env/README.md`
- **BUILDKIT Spec**: `/home/user/ripple-env/BUILDKIT_STARTER_SPEC.md` (Layer 1.20)
- **Task Reference**: ARIA_AUDIT_REPORT.md (P2-012)

## Change Log

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2026-01-09 | 1.0.0 | L13 UI Domain Team Lead | Initial implementation of P2-012 |

## Sign-off

- ✅ **Code Changes**: Complete
- ✅ **Documentation**: Complete
- ✅ **Environment Variables**: Configured
- ✅ **Verification Commands**: Provided
- ⏳ **Testing**: Pending deployment
- ⏳ **Production Deployment**: Pending approval

---

**Task Status**: Implementation Complete - Ready for Testing
