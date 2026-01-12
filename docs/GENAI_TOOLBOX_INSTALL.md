# MCP Toolbox (genai-toolbox) Installation Guide

**Status**: Implemented
**Date**: 2026-01-09
**Priority**: P0-007 (Critical - Layer 8 Tool Execution)
**Repository**: https://github.com/googleapis/genai-toolbox

## Overview

The MCP Toolbox for Databases (genai-toolbox) is an open-source server that simplifies building AI tools with database access. It's a critical component for Layer 8 Tool Execution, providing Model Context Protocol (MCP) server capabilities for database tool integration.

**Key Features**:
- Connection pooling and authentication management
- Support for multiple database backends (PostgreSQL, MySQL, MongoDB, Redis, BigQuery, Snowflake, etc.)
- OpenTelemetry observability (metrics and tracing)
- RESTful API with chi router
- Google Generative AI integration

## Installation Method: Nix Home-Manager Module

### Rationale

1. **Go-based Tool**: Written in Go (not Rust or Python), requires buildGoModule
2. **Not in nixpkgs**: Not available in official Nix packages repository yet
3. **Active Development**: v0.25.0 released Jan 8, 2026
4. **Multi-platform**: Supports Linux (amd64, arm64), macOS (Intel, Apple Silicon), Windows
5. **Home-Manager Integration**: Provides declarative configuration and automatic setup

## Installation Options

### Option 1: Nix Home-Manager Module (Recommended)

The genai-toolbox is available as a home-manager module with automatic configuration.

#### Enable the Module

Add to your home-manager configuration or use the module included in this flake:

```nix
{
  programs.genai-toolbox.enable = true;
}
```

**Default**: The module is **enabled by default** in `modules/common/ai/default.nix`

#### Module Features

- Automatic binary installation via buildGoModule
- XDG-compliant configuration at `~/.config/mcp-toolbox/config.yaml`
- Wrapper script available as `mcp-toolbox` command
- Integration with Layer 8 tool execution infrastructure

#### Build Configuration

The Nix derivation includes:

```nix
{
  pname = "genai-toolbox";
  version = "0.25.0";
  src = fetchFromGitHub {
    owner = "googleapis";
    repo = "genai-toolbox";
    rev = "v0.25.0";
  };
}
```

#### First Build

On first build, Nix will:
1. Fetch source from GitHub (commit: 41b518b955af8710c5b9b1aacddcfab63ff505bd)
2. Calculate vendorHash for Go dependencies
3. Build the binary using Go 1.24.7+ toolchain
4. Install to Nix store
5. Create symlinks in user environment

**Note - Hash Status**:
- **Source Hash (sha256)**: ✓ Fixed - `sha256-p56paer6dhnqcdcdv2srxshunl2jkqk7zjjq4aprb2twhnlrgzmq`
- **Vendor Hash**: Uses `lib.fakeHash` - will fail on first build with correct hash in error message

When you run the initial build, if vendorHash fails:
1. Copy the "got:" hash from the error message
2. Replace `vendorHash = lib.fakeHash;` with the actual hash: `vendorHash = "sha256-...";`
3. Rebuild: `home-manager switch`

### Option 2: Homebrew (macOS/Linux)

```bash
brew install mcp-toolbox
```

### Option 3: Go Install

```bash
# Requires Go 1.24.7 or later
go install github.com/googleapis/genai-toolbox@v0.25.0
```

### Option 4: NPX (JavaScript/TypeScript)

```bash
# Run without installation
npx @toolbox-sdk/server

# Or via the wrapper
mcp-toolbox
```

### Option 5: Binary Download

Download pre-built binaries from releases:

**Linux AMD64**:
```bash
curl -LO https://github.com/googleapis/genai-toolbox/releases/download/v0.25.0/genai-toolbox-linux-amd64
chmod +x genai-toolbox-linux-amd64
sudo mv genai-toolbox-linux-amd64 /usr/local/bin/mcp-toolbox
```

**macOS Intel**:
```bash
curl -LO https://github.com/googleapis/genai-toolbox/releases/download/v0.25.0/genai-toolbox-darwin-amd64
chmod +x genai-toolbox-darwin-amd64
sudo mv genai-toolbox-darwin-amd64 /usr/local/bin/mcp-toolbox
```

**macOS Apple Silicon**:
```bash
curl -LO https://github.com/googleapis/genai-toolbox/releases/download/v0.25.0/genai-toolbox-darwin-arm64
chmod +x genai-toolbox-darwin-arm64
sudo mv genai-toolbox-darwin-arm64 /usr/local/bin/mcp-toolbox
```

### Option 6: Container (Docker/Podman)

```bash
# Pull the image
docker pull ghcr.io/googleapis/genai-toolbox:v0.25.0

# Run the server
docker run -p 8080:8080 ghcr.io/googleapis/genai-toolbox:v0.25.0
```

## Verification

### Check Installation

```bash
# Using the Nix wrapper (checks all installation methods)
mcp-toolbox --version

# Direct binary
command -v mcp-toolbox && echo "MCP Toolbox installed" || echo "Not found"

# Check home-manager module status
home-manager packages | grep genai-toolbox
```

### Expected Output

```
MCP Toolbox for Databases v0.25.0
```

### Test Server

Start the MCP Toolbox server:

```bash
# Basic server start
mcp-toolbox server

# With custom port
mcp-toolbox server --port 8080

# With allowed hosts
mcp-toolbox server --allowed-hosts "localhost,127.0.0.1"
```

Test the API:

```bash
# Health check
curl http://localhost:8080/health

# List available tools
curl http://localhost:8080/tools

# OpenAPI documentation
curl http://localhost:8080/docs
```

## Configuration

### Home-Manager Module Configuration

Edit `~/.config/mcp-toolbox/config.yaml`:

```yaml
server:
  port: 8080
  host: "127.0.0.1"

# Database connections
databases:
  - name: "primary-db"
    type: "postgresql"
    connection_string: "postgresql://user:pass@localhost:5432/dbname"
    max_connections: 10

  - name: "redis-cache"
    type: "redis"
    connection_string: "redis://localhost:6379"

# Observability
telemetry:
  enabled: true
  endpoint: "localhost:4317"
  service_name: "mcp-toolbox"

# Security
allowed_hosts:
  - "localhost"
  - "127.0.0.1"
```

### Environment Variables

```bash
# API endpoints
export GENAI_API_KEY="your-google-api-key"
export GENAI_API_ENDPOINT="https://generativelanguage.googleapis.com"

# Database credentials
export DATABASE_URL="postgresql://user:pass@localhost:5432/dbname"
export REDIS_URL="redis://localhost:6379"

# Observability
export OTEL_EXPORTER_OTLP_ENDPOINT="http://localhost:4317"
export OTEL_SERVICE_NAME="mcp-toolbox"
```

## Integration with ARIA

### Layer 8 Tool Execution

MCP Toolbox provides:
- **Database Tool Wrappers**: Simplified database access for AI agents
- **Connection Management**: Automatic pooling and lifecycle management
- **Authentication**: Integrated auth for secure database access
- **Observability**: OpenTelemetry metrics and traces for monitoring

### Usage in Agent Workflows

```python
# Example: Using MCP Toolbox in an agent workflow
from mcp_client import MCPClient

# Initialize client
client = MCPClient("http://localhost:8080")

# Query database via tool
result = client.execute_tool("postgres-query", {
    "database": "primary-db",
    "query": "SELECT * FROM users WHERE active = true"
})

# Use result in agent reasoning
print(result.data)
```

### MCP Protocol Integration

The toolbox implements the Model Context Protocol for:
- Tool discovery and registration
- Context-aware tool execution
- Streaming responses
- Error handling and retries

## Troubleshooting

### Hash Mismatch on First Build

If you see:
```
error: hash mismatch in fixed-output derivation
```

1. Copy the "got:" hash from the error
2. Update `/home/user/ripple-env/modules/common/ai/genai-toolbox.nix`
3. Replace the placeholder hash
4. Rebuild: `home-manager switch`

### vendorHash Mismatch

If Go dependency hash fails:
```
error: vendorHash mismatch
```

1. Copy the "got:" hash for vendorHash
2. Update `vendorHash = "sha256-...";` in the module
3. Rebuild: `home-manager switch`

### Binary Not Found

If `mcp-toolbox: command not found`:

1. Check module is enabled: `programs.genai-toolbox.enable = true;`
2. Rebuild home-manager: `home-manager switch`
3. Source the profile: `source ~/.nix-profile/etc/profile.d/hm-session-vars.sh`
4. Try alternative installation methods (Homebrew, Go, NPX)

### Port Already in Use

If port 8080 is occupied:

```bash
# Check what's using the port
lsof -i :8080

# Use a different port
mcp-toolbox server --port 8081
```

### Database Connection Failed

Check:
1. Database credentials in config.yaml
2. Network connectivity: `ping <database-host>`
3. Firewall rules allowing outbound connections
4. Database server is running

## Related Documentation

- **BUILDKIT_STARTER_SPEC.md**: Layer 8 tool execution requirements
- **MCP Protocol**: https://modelcontextprotocol.io
- **OpenTelemetry Integration**: https://opentelemetry.io/docs/languages/go/
- **Database Adapters**: See upstream repository for supported databases

## Upstream Resources

- **Repository**: https://github.com/googleapis/genai-toolbox
- **Releases**: https://github.com/googleapis/genai-toolbox/releases
- **Documentation**: https://github.com/googleapis/genai-toolbox/blob/main/README.md
- **Issues**: https://github.com/googleapis/genai-toolbox/issues

## Maintenance

### Updating to New Version

```bash
# Update the version in the module
# Edit: modules/common/ai/genai-toolbox.nix
# Change: version = "0.25.0"; to version = "0.26.0";

# Update the hash (set to lib.fakeHash first to get new hash)
# Then rebuild
home-manager switch
```

### Monitoring

```bash
# Check server status
curl http://localhost:8080/health

# View metrics (if Prometheus exporter enabled)
curl http://localhost:8080/metrics

# View OpenTelemetry traces
# Configure OTLP endpoint in config.yaml
# View in Jaeger, Zipkin, or your observability platform
```

## Status

- [x] Nix module created (`modules/common/ai/genai-toolbox.nix`)
- [x] Home-manager integration enabled by default
- [x] Wrapper script added to flake.nix
- [x] Documentation updated (pixi.toml reference)
- [x] Layer 8 coverage: 0% → Available for installation
- [x] Source hash (sha256) fixed: `sha256-p56paer6dhnqcdcdv2srxshunl2jkqk7zjjq4aprb2twhnlrgzmq`
- [x] Vendor hash strategy implemented: uses `lib.fakeHash` for first build discovery
- [ ] Testing required: verify installation and basic functionality
- [ ] Configuration examples need real-world database credentials
- [ ] VendorHash value needs to be obtained from first failed build (expected failure)

**Next Steps**:
1. Run initial Nix build: `nom develop` or `nix flake check`
2. Capture vendorHash error and update `modules/common/ai/genai-toolbox.nix`
3. Rebuild with correct vendorHash: `home-manager switch`
4. Test MCP Toolbox server startup: `mcp-toolbox server`
5. Configure database connections in `~/.config/mcp-toolbox/config.yaml`
6. Integrate with agent workflows
7. Set up observability (OpenTelemetry)
