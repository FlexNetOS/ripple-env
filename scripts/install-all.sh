#!/usr/bin/env bash
# =============================================================================
# Unified Installation Script for ROS2 Humble Agentic Environment
# =============================================================================
# This script orchestrates all installations in proper order using:
# - direnv: Environment variable management
# - Pixi: Universal package manager (Python, Node, Rust, Go)
# - Nix: System-level packages and reproducible builds
# - Docker Compose: Containerized services
# =============================================================================

set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

# =============================================================================
# Phase 0: Pre-flight Checks
# =============================================================================
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

# =============================================================================
# Phase 1: Environment Setup (direnv)
# =============================================================================
log_info "Phase 1: Setting up environment with direnv..."

if [ -f ".envrc" ]; then
    log_info "Found .envrc, allowing direnv..."
    direnv allow . 2>/dev/null || log_warn "direnv not configured, skipping"
fi

# =============================================================================
# Phase 2: Nix Packages (System-level tools)
# =============================================================================
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

# =============================================================================
# Phase 3: Pixi Packages (Python, Node, Rust, etc.)
# =============================================================================
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

# =============================================================================
# Phase 4: Docker Network Setup
# =============================================================================
log_info "Phase 4: Setting up Docker network..."

if docker network ls | grep -q "agentic-network"; then
    log_success "Docker network 'agentic-network' already exists"
else
    log_info "Creating Docker network 'agentic-network'..."
    docker network create agentic-network || log_warn "Failed to create network"
fi

# =============================================================================
# Phase 5: Core Services (Docker Compose)
# =============================================================================
log_info "Phase 5: Starting core services..."

resolve_compose_file() {
    # usage: resolve_compose_file docker-compose.observability.yml
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

# =============================================================================
# Phase 6: Kubernetes/Argo CD (Optional)
# =============================================================================
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

# =============================================================================
# Phase 7: Verification
# =============================================================================
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

echo ""
echo "============================================================================="
log_success "Installation complete!"
echo "============================================================================="
echo ""
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
echo ""
