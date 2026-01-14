# Script Contract: install-all.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/install-all.sh`

---

## Purpose

Unified installation orchestrator for the ROS2 Humble Agentic Environment. Manages complete system setup in 7 phases: pre-flight checks, direnv environment, Nix packages, Pixi packages, Docker network, core services (Docker Compose), and optional Kubernetes/Argo CD. Handles dependencies between components and provides comprehensive verification and health checks.

---

## Invocation

```bash
./scripts/install-all.sh
```

**No arguments** - Fully automated with optional Argo CD prompt.

**Requirements:**
- `nix` - System-level packages and reproducible builds
- `pixi` - Universal package manager (Python, Node, Rust, Go)
- `docker` - Containerized services
- `docker compose` (v2) or `docker-compose` (v1) - Service orchestration
- `direnv` (optional) - Environment variable management

---

## Outputs

**Standard Output (successful installation):**
```
[INFO] Phase 0: Running pre-flight checks...
[SUCCESS] nix is installed
[SUCCESS] pixi is installed
[SUCCESS] docker is installed
[SUCCESS] docker compose (v2) is available

[INFO] Phase 1: Setting up environment with direnv...
[INFO] Found .envrc, allowing direnv...

[INFO] Phase 2: Installing Nix packages...
[INFO] Building Nix development shell...
Nix shell ready

[INFO] Phase 3: Installing Pixi packages...
[INFO] Installing Pixi dependencies...
[INFO] Verifying Pixi packages...
Python 3.12.7
v22.11.0

[INFO] Phase 4: Setting up Docker network...
[SUCCESS] Docker network 'agentic-network' already exists

[INFO] Phase 5: Starting core services...
[INFO] Starting observability services from docker/docker-compose.observability.yml...
[INFO] Starting messaging services from docker/docker-compose.messaging.yml...
[INFO] Starting automation services from docker/docker-compose.automation.yml...
[INFO] Starting edge services from docker/docker-compose.edge.yml...
[INFO] Starting inference services from docker/docker-compose.localai.yml...
[INFO] Starting ui services from docker/docker-compose.ui.yml...

[INFO] Phase 6: Kubernetes/Argo CD setup (optional)...
Install Argo CD? (requires kubectl and K3s/K8s) [y/N]: n
[INFO] Skipping Argo CD installation

[INFO] Phase 7: Running verification checks...

=============================================================================
INSTALLATION SUMMARY
=============================================================================

Pixi Packages:
python                    3.12.7
nodejs                    22.11.0
jupyterlab                4.2.5
mlflow                    2.18.0
transformers              4.46.3

Docker Services:
NAMES                     STATUS                  PORTS
prometheus                Up 5 seconds            0.0.0.0:9090->9090/tcp
grafana                   Up 5 seconds            0.0.0.0:3000->3000/tcp
nats                      Up 5 seconds            0.0.0.0:4222->4222/tcp
temporal                  Up 5 seconds            0.0.0.0:7233->7233/tcp
opa                       Up 5 seconds            0.0.0.0:8181->8181/tcp
n8n                       Up 5 seconds            0.0.0.0:5678->5678/tcp
kong                      Up 5 seconds            0.0.0.0:8000->8000/tcp
localai                   Up 5 seconds            0.0.0.0:8080->8080/tcp

Service Health Checks:
  ✅ Prometheus: healthy
  ✅ Grafana: healthy
  ✅ NATS: healthy
  ✅ OPA: healthy
  ✅ n8n: healthy
  ✅ LocalAI: healthy
  ✅ Kong: healthy
  ✅ Lobe Chat: healthy
  ✅ Temporal UI: healthy

=============================================================================
[SUCCESS] Installation complete!
=============================================================================

Next steps:
  1. Run 'nix develop' to enter the development shell
  2. Run 'pixi shell' to activate the Pixi environment
  3. Access services:
     - Grafana: http://localhost:3000 (admin/admin)
     - Prometheus: http://localhost:9090
     - n8n: http://localhost:5678
     - Temporal UI: http://localhost:8088
     - Lobe Chat: http://localhost:3210
     - LocalAI: http://localhost:8080
```

**Standard Output (missing dependencies):**
```
[INFO] Phase 0: Running pre-flight checks...
[SUCCESS] nix is installed
[WARN] pixi is not installed
[SUCCESS] docker is installed
[WARN] docker compose not available (docker compose / docker-compose)
[ERROR] Missing dependencies: pixi docker-compose
[INFO] Please install missing dependencies before running this script.
[INFO]   - Nix: curl -L https://nixos.org/nix/install | sh
[INFO]   - Pixi: curl -fsSL https://pixi.sh/install.sh | bash
[INFO]   - Docker: https://docs.docker.com/get-docker/
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (all phases complete) |
| `1` | Failure (missing dependencies) |

---

## Side Effects

### Docker Network Creation (line 130)

**Evidence:**
```bash
if docker network ls | grep -q "agentic-network"; then
    log_success "Docker network 'agentic-network' already exists"
else
    log_info "Creating Docker network 'agentic-network'..."
    docker network create agentic-network || log_warn "Failed to create network"
fi
```

**Creates:** `agentic-network` bridge network for inter-service communication

### Docker Compose Services (lines 160-172)

**Evidence:**
```bash
for group in observability messaging automation edge inference ui; do
    compose_file="$(resolve_compose_file "${SERVICE_GROUPS[$group]}")"
    if [ -f "$compose_file" ]; then
        log_info "Starting $group services from $compose_file..."
        "${COMPOSE[@]}" -f "$compose_file" up -d || {
            log_warn "Failed to start $group services"
        }
        # Wait for services to be ready
        sleep 5
    fi
done
```

**Starts services:** ~25+ containers across 6 service groups

### Nix Store (line 93)

**Evidence:**
```bash
nix develop --command echo "Nix shell ready"
```

**Downloads and builds:** Nix packages to `/nix/store/`

### Pixi Environment (line 108)

**Evidence:**
```bash
pixi install || {
    log_warn "Pixi install failed, trying with --frozen..."
    pixi install --frozen || log_error "Pixi installation failed"
}
```

**Creates:** `.pixi/` directory with conda-forge packages

---

## Safety Classification

**🟡 CAUTION** - Installs packages, creates networks, starts containers.

---

## Idempotency

**⚠️ PARTIALLY IDEMPOTENT**
- Nix and Pixi installs are idempotent (lines 93, 108)
- Docker network creation is idempotent (line 126)
- Docker Compose startup is idempotent (line 164)
- **Not idempotent:** Service restarts may cause brief downtime

---

## Installation Phases

### Phase 0: Pre-flight Checks (lines 33-74)

**Evidence:**
```bash
log_info "Phase 0: Running pre-flight checks..."

check_command() {
    if command -v "$1" &> /dev/null; then
        log_success "$1 is installed"
        return 0
    else
        log_warn "$1 is not installed"
        return 1
    fi
}

MISSING_DEPS=()

check_command "nix" || MISSING_DEPS+=("nix")
check_command "pixi" || MISSING_DEPS+=("pixi")
check_command "docker" || MISSING_DEPS+=("docker")

# Prefer v2 plugin (`docker compose`), fallback to legacy `docker-compose`.
COMPOSE=()
if command -v docker >/dev/null 2>&1 && docker compose version >/dev/null 2>&1; then
    log_success "docker compose (v2) is available"
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    log_success "docker-compose (legacy) is available"
    COMPOSE=(docker-compose)
else
    log_warn "docker compose not available (docker compose / docker-compose)"
    MISSING_DEPS+=("docker-compose")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    log_error "Missing dependencies: ${MISSING_DEPS[*]}"
    log_info "Please install missing dependencies before running this script."
    log_info "  - Nix: curl -L https://nixos.org/nix/install | sh"
    log_info "  - Pixi: curl -fsSL https://pixi.sh/install.sh | bash"
    log_info "  - Docker: https://docs.docker.com/get-docker/"
    exit 1
fi
```

**Checks:**
1. `nix` - System-level packages (line 50)
2. `pixi` - Universal package manager (line 51)
3. `docker` - Container runtime (line 52)
4. `docker compose` (v2) or `docker-compose` (v1) - Service orchestration (lines 54-65)

**Exits if missing dependencies** (line 73)

### Phase 1: Environment Setup (lines 76-84)

**Evidence:**
```bash
log_info "Phase 1: Setting up environment with direnv..."

if [ -f ".envrc" ]; then
    log_info "Found .envrc, allowing direnv..."
    direnv allow . 2>/dev/null || log_warn "direnv not configured, skipping"
fi
```

**Purpose:** Load environment variables from `.envrc`

**Optional:** Skips if direnv not available (line 83)

### Phase 2: Nix Packages (lines 86-99)

**Evidence:**
```bash
log_info "Phase 2: Installing Nix packages..."

if [ -f "flake.nix" ]; then
    log_info "Building Nix development shell..."
    nix develop --command echo "Nix shell ready" || {
        log_warn "Nix develop failed, trying flake check..."
        nix flake check || log_warn "Flake check failed"
    }
else
    log_warn "No flake.nix found, skipping Nix installation"
fi
```

**Builds:** Development shell with system packages (kubectl, helm, trivy, etc.)

**Fallback:** `nix flake check` if `nix develop` fails (line 94)

### Phase 3: Pixi Packages (lines 101-119)

**Evidence:**
```bash
log_info "Phase 3: Installing Pixi packages..."

if [ -f "pixi.toml" ]; then
    log_info "Installing Pixi dependencies..."
    pixi install || {
        log_warn "Pixi install failed, trying with --frozen..."
        pixi install --frozen || log_error "Pixi installation failed"
    }

    # Verify key packages
    log_info "Verifying Pixi packages..."
    pixi run python --version || log_warn "Python not available via Pixi"
    pixi run node --version || log_warn "Node.js not available via Pixi"
else
    log_warn "No pixi.toml found, skipping Pixi installation"
fi
```

**Installs:** Python, Node.js, Jupyter, MLflow, Transformers, etc.

**Fallback:** `--frozen` flag to use existing lock file (line 110)

**Verification:** Checks Python and Node.js versions (lines 115-116)

### Phase 4: Docker Network Setup (lines 121-131)

**Evidence:**
```bash
log_info "Phase 4: Setting up Docker network..."

if docker network ls | grep -q "agentic-network"; then
    log_success "Docker network 'agentic-network' already exists"
else
    log_info "Creating Docker network 'agentic-network'..."
    docker network create agentic-network || log_warn "Failed to create network"
fi
```

**Network:** `agentic-network` (bridge driver, default)

**Purpose:** Enable inter-service communication without exposing ports

### Phase 5: Core Services (lines 133-172)

**Evidence:**
```bash
log_info "Phase 5: Starting core services..."

resolve_compose_file() {
    local base="$1"
    if [ -f "docker/$base" ]; then
        echo "docker/$base"
    else
        echo "$base"
    fi
}

# Define service groups in dependency order
declare -A SERVICE_GROUPS=(
    ["observability"]="docker-compose.observability.yml"
    ["messaging"]="docker-compose.messaging.yml"
    ["automation"]="docker-compose.automation.yml"
    ["edge"]="docker-compose.edge.yml"
    ["inference"]="docker-compose.localai.yml"
    ["ui"]="docker-compose.ui.yml"
    ["gitops"]="docker-compose.argo.yml"
)

# Start services in order
for group in observability messaging automation edge inference ui; do
    compose_file="$(resolve_compose_file "${SERVICE_GROUPS[$group]}")"
    if [ -f "$compose_file" ]; then
        log_info "Starting $group services from $compose_file..."
        "${COMPOSE[@]}" -f "$compose_file" up -d || {
            log_warn "Failed to start $group services"
        }
        # Wait for services to be ready
        sleep 5
    else
        log_warn "Compose file $compose_file not found, skipping $group"
    fi
done
```

**Service groups (started in order):**
1. **observability** - Prometheus, Grafana, Netdata
2. **messaging** - NATS, Temporal, Redis
3. **automation** - OPA, n8n, AGiXT
4. **edge** - Kong, Kuma
5. **inference** - LocalAI
6. **ui** - Lobe Chat, Open-Lovable

**Wait time:** 5 seconds between groups (line 168)

**Compose file resolution:** Checks `docker/` subdirectory first (lines 138-145)

### Phase 6: Kubernetes/Argo CD (lines 174-187)

**Evidence:**
```bash
log_info "Phase 6: Kubernetes/Argo CD setup (optional)..."

if [ -f "scripts/install-argocd.sh" ]; then
    read -p "Install Argo CD? (requires kubectl and K3s/K8s) [y/N]: " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        bash scripts/install-argocd.sh || log_warn "Argo CD installation failed"
    else
        log_info "Skipping Argo CD installation"
    fi
fi
```

**Optional:** Interactive prompt (line 180)

**Requirements:** kubectl + K3s/K8s cluster

**Script:** `scripts/install-argocd.sh` (lines 179-186)

### Phase 7: Verification (lines 189-232)

**Evidence:**
```bash
log_info "Phase 7: Running verification checks..."

echo ""
echo "============================================================================="
echo "INSTALLATION SUMMARY"
echo "============================================================================="

# Check Pixi packages
echo ""
echo "Pixi Packages:"
pixi list 2>/dev/null | head -20 || echo "  (pixi not available)"

# Check Docker services
echo ""
echo "Docker Services:"
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}" 2>/dev/null | head -20 || echo "  (docker not available)"

# Check service health
echo ""
echo "Service Health Checks:"

check_service() {
    local name=$1
    local url=$2
    if curl -s --max-time 5 "$url" > /dev/null 2>&1; then
        echo "  ✅ $name: healthy"
    else
        echo "  ❌ $name: not responding"
    fi
}

check_service "Prometheus" "http://localhost:9090/-/healthy"
check_service "Grafana" "http://localhost:3000/api/health"
check_service "NATS" "http://localhost:8222/healthz"
check_service "OPA" "http://localhost:8181/health"
check_service "n8n" "http://localhost:5678/healthz"
check_service "LocalAI" "http://localhost:8080/readyz"
check_service "Kong" "http://localhost:8001/status"
check_service "Lobe Chat" "http://localhost:3210"
check_service "Temporal UI" "http://localhost:8088"
```

**Verification checks:**
1. Pixi package list (line 202)
2. Docker container status (line 207)
3. Service health endpoints (lines 223-231)

**Health check:** 5-second timeout per service (line 216)

---

## Service Health Endpoints

| Service | Health URL | Expected Response |
|---------|------------|-------------------|
| Prometheus | http://localhost:9090/-/healthy | 200 OK |
| Grafana | http://localhost:3000/api/health | 200 OK, JSON {"status":"ok"} |
| NATS | http://localhost:8222/healthz | 200 OK |
| OPA | http://localhost:8181/health | 200 OK, empty body |
| n8n | http://localhost:5678/healthz | 200 OK |
| LocalAI | http://localhost:8080/readyz | 200 OK |
| Kong | http://localhost:8001/status | 200 OK, JSON |
| Lobe Chat | http://localhost:3210 | 200 OK |
| Temporal UI | http://localhost:8088 | 200 OK |

---

## Installation Time

**Estimated duration:**
- **First run:** 15-30 minutes (downloading images, building Nix packages)
- **Subsequent runs:** 2-5 minutes (cached packages, existing images)

**Time breakdown:**
1. Phase 0 (pre-flight): 5 seconds
2. Phase 1 (direnv): 1 second
3. Phase 2 (Nix): 5-15 minutes (first run), 30 seconds (cached)
4. Phase 3 (Pixi): 3-10 minutes (first run), 10 seconds (cached)
5. Phase 4 (network): 1 second
6. Phase 5 (services): 2-5 minutes (pulling images)
7. Phase 6 (Argo CD): 2-5 minutes (if enabled)
8. Phase 7 (verification): 30 seconds

---

## Next Steps After Installation

**Evidence:** Lines 238-248

```bash
echo "Next steps:"
echo "  1. Run 'nix develop' to enter the development shell"
echo "  2. Run 'pixi shell' to activate the Pixi environment"
echo "  3. Access services:"
echo "     - Grafana: http://localhost:3000 (admin/admin)"
echo "     - Prometheus: http://localhost:9090"
echo "     - n8n: http://localhost:5678"
echo "     - Temporal UI: http://localhost:8088"
echo "     - Lobe Chat: http://localhost:3210"
echo "     - LocalAI: http://localhost:8080"
```

**Development workflow:**
```bash
# Enter Nix shell
nix develop

# Or activate Pixi environment
pixi shell

# Access services
open http://localhost:3000  # Grafana

# Deploy additional services
./scripts/deploy.sh
```

---

## Troubleshooting

### Missing Dependencies

**Symptoms:** Exit 1 with "Missing dependencies" error

**Fix:**
```bash
# Install Nix
curl -L https://nixos.org/nix/install | sh
source ~/.nix-profile/etc/profile.d/nix.sh

# Install Pixi
curl -fsSL https://pixi.sh/install.sh | bash

# Install Docker
# Ubuntu/Debian
sudo apt-get install docker.io docker-compose-plugin

# macOS
brew install docker docker-compose
```

### Service Not Starting

**Symptoms:** "Failed to start ... services" warning

**Debug:**
```bash
# Check Docker logs
docker compose -f docker/docker-compose.observability.yml logs

# Check specific service
docker logs prometheus

# Restart services
docker compose -f docker/docker-compose.observability.yml restart
```

### Health Check Failing

**Symptoms:** "❌ Service: not responding"

**Possible causes:**
- Service still starting (wait 30-60 seconds)
- Port conflict (another process using port)
- Configuration error

**Debug:**
```bash
# Check service logs
docker logs <service-name>

# Check port availability
lsof -i :9090  # Prometheus
lsof -i :3000  # Grafana

# Test health endpoint manually
curl -v http://localhost:9090/-/healthy
```

### Nix Build Failed

**Symptoms:** "Nix develop failed" warning (line 94)

**Debug:**
```bash
# Check flake syntax
nix flake check

# Rebuild with verbose output
nix develop -L

# Update flake inputs
nix flake update

# Clear cache
rm -rf ~/.cache/nix
```

### Pixi Install Failed

**Symptoms:** "Pixi installation failed" error (line 110)

**Debug:**
```bash
# Check pixi.toml syntax
pixi info

# Try frozen install
pixi install --frozen

# Clean and reinstall
rm -rf .pixi
pixi install

# Check available platforms
pixi list
```

---

## References

### Source Code
- **Main script:** `scripts/install-all.sh` (249 lines)
- **Phase 0:** lines 33-74
- **Phase 1:** lines 76-84
- **Phase 2:** lines 86-99
- **Phase 3:** lines 101-119
- **Phase 4:** lines 121-131
- **Phase 5:** lines 133-172
- **Phase 6:** lines 174-187
- **Phase 7:** lines 189-232

### Related Files
- **Nix flake:** `flake.nix`
- **Pixi config:** `pixi.toml`
- **direnv config:** `.envrc`
- **Docker Compose:** `docker/docker-compose*.yml`
- **Argo CD installer:** `scripts/install-argocd.sh`

### External Resources
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Pixi Documentation](https://pixi.sh/latest/)
- [Docker Compose](https://docs.docker.com/compose/)
- [direnv](https://direnv.net/)
- [Argo CD](https://argo-cd.readthedocs.io/)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 40/60 contracts complete (66.7%)
