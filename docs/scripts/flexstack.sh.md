# Script Contract: flexstack.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/flexstack.sh`

---

## Purpose

Docker Compose profile manager for FlexStack service orchestration. Manages service groups via profile presets (minimal, dev, ai, observability, identity, full), provides health checks, service status monitoring, and memory requirement calculations. Wrapper around docker-compose with intelligent profile selection.

---

## Invocation

```bash
./scripts/flexstack.sh COMMAND [OPTIONS]
```

**Commands:**
- `up [PROFILE]` - Start services with profile preset
- `down` - Stop all services
- `status` - Show service status
- `logs [SERVICE]` - Show service logs
- `health` - Check service health
- `profiles` - List available profiles
- `requirements [PROFILE]` - Show memory requirements
- `help` - Show help

**Examples:**
```bash
./scripts/flexstack.sh up minimal           # Start core services
./scripts/flexstack.sh up ai                # Start AI inference stack
./scripts/flexstack.sh status               # Show all service status
./scripts/flexstack.sh logs localai         # Show LocalAI logs
./scripts/flexstack.sh health               # Check all health endpoints
./scripts/flexstack.sh requirements full    # Show full stack memory needs
```

---

## Side Effects

### Docker Compose Operations (lines 189-267)
- Starts/stops Docker containers via `docker compose`
- Creates Docker networks and volumes as defined in compose files

### No File System Changes
- Read-only access to docker-compose YAML files
- Does not modify configuration files

---

## Safety Classification

**🟡 CAUTION** - Manages Docker containers (can consume resources).

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Can run `up` repeatedly safely.

---

## Key Features

### Profile Presets (lines 38-55)

```bash
declare -A PROFILE_PRESETS=(
    ["minimal"]="core"
    ["dev"]="dev"
    ["ai"]="core ai"
    ["observability"]="observability"
    ["identity"]="core identity"
    ["full"]="full"
)

declare -A PROFILE_DESCRIPTIONS=(
    ["minimal"]="Core services only (Redis, Postgres, NATS)"
    ["dev"]="Development services (core + dev tools)"
    ["ai"]="AI inference stack (core + LocalAI, models)"
    ["observability"]="Monitoring stack (Prometheus, Grafana, Loki)"
    ["identity"]="Identity services (core + Keycloak, Vault)"
    ["full"]="All services (complete stack)"
)
```

**Profiles map to Docker Compose profiles:**
- `core` → redis, postgres, nats
- `ai` → localai, temporal
- `observability` → prometheus, grafana, loki
- `identity` → keycloak, vault
- `dev` → adminer, pgweb

### Service Groups (lines 57-78)

```bash
declare -A SERVICE_GROUPS=(
    ["core"]="redis postgres nats"
    ["ai"]="localai temporal temporal-ui"
    ["observability"]="prometheus grafana loki"
    ["identity"]="keycloak vault"
    ["dev"]="adminer pgweb"
)

declare -A SERVICE_HEALTH_ENDPOINTS=(
    ["redis"]="http://localhost:6379/ping"
    ["postgres"]="postgresql://localhost:5432/"
    ["nats"]="http://localhost:8222/healthz"
    ["localai"]="http://localhost:8080/readyz"
    ["prometheus"]="http://localhost:9090/-/healthy"
    ["grafana"]="http://localhost:3000/api/health"
    ["keycloak"]="http://localhost:8081/health"
    ["vault"]="http://localhost:8200/v1/sys/health"
)
```

### Memory Requirements (lines 80-113)

```bash
declare -A SERVICE_MEMORY=(
    ["redis"]="256"           # MB
    ["postgres"]="512"
    ["nats"]="256"
    ["localai"]="4096"
    ["temporal"]="1024"
    ["temporal-ui"]="256"
    ["prometheus"]="512"
    ["grafana"]="256"
    ["loki"]="512"
    ["keycloak"]="1024"
    ["vault"]="256"
    ["adminer"]="128"
    ["pgweb"]="128"
)

cmd_requirements() {
    local profile="${1:-minimal}"
    local profiles
    profiles="${PROFILE_PRESETS[$profile]:-$profile}"

    local total_memory=0
    local services=()

    for prof in $profiles; do
        local group_services="${SERVICE_GROUPS[$prof]}"
        for svc in $group_services; do
            services+=("$svc")
            local mem="${SERVICE_MEMORY[$svc]:-0}"
            ((total_memory += mem))
        done
    done

    echo ""
    log_info "Memory Requirements for Profile: $profile"
    echo "==========================================="
    for svc in "${services[@]}"; do
        printf "  %-20s %6d MB\n" "$svc" "${SERVICE_MEMORY[$svc]}"
    done
    echo "==========================================="
    printf "  %-20s %6d MB (%.1f GB)\n" "TOTAL" "$total_memory" "$(echo "scale=1; $total_memory/1024" | bc)"
}
```

**Example output:**
```
Memory Requirements for Profile: ai
===========================================
  redis                   256 MB
  postgres                512 MB
  nats                    256 MB
  localai                4096 MB
  temporal               1024 MB
  temporal-ui             256 MB
===========================================
  TOTAL                  6400 MB (6.2 GB)
```

### Service Status (lines 269-316)

```bash
cmd_status() {
    log_info "Service Status"
    echo "=============="
    echo ""

    # Get all running containers
    if ! docker compose ps --format json 2>/dev/null | jq -r '.[] | "\(.Name)\t\(.State)\t\(.Status)"' 2>/dev/null; then
        # Fallback to text format
        docker compose ps --format "table {{.Name}}\t{{.State}}\t{{.Status}}"
    fi

    echo ""
    log_info "Active Profiles"
    echo "==============="

    # Detect active profiles based on running services
    for profile in "${!PROFILE_PRESETS[@]}"; do
        local profiles="${PROFILE_PRESETS[$profile]}"
        local running=true

        for prof in $profiles; do
            local services="${SERVICE_GROUPS[$prof]}"
            for svc in $services; do
                if ! docker compose ps "$svc" 2>/dev/null | grep -q "Up"; then
                    running=false
                    break 2
                fi
            done
        done

        if $running; then
            log_success "Profile '$profile' is active"
        fi
    done
}
```

### Health Checks (lines 318-371)

```bash
cmd_health() {
    log_info "Service Health Checks"
    echo "====================="
    echo ""

    local healthy=0
    local unhealthy=0

    for service in "${!SERVICE_HEALTH_ENDPOINTS[@]}"; do
        local endpoint="${SERVICE_HEALTH_ENDPOINTS[$service]}"

        # Check if service is running first
        if ! docker compose ps "$service" 2>/dev/null | grep -q "Up"; then
            log_warn "$service: Not running"
            ((unhealthy++))
            continue
        fi

        # Check health endpoint
        if curl -sf "$endpoint" >/dev/null 2>&1; then
            log_success "$service: Healthy"
            ((healthy++))
        elif curl -sf -o /dev/null -w "%{http_code}" "$endpoint" 2>/dev/null | grep -q "200"; then
            log_success "$service: Healthy (HTTP 200)"
            ((healthy++))
        else
            log_error "$service: Unhealthy"
            ((unhealthy++))
        fi
    done

    echo ""
    echo "Summary: $healthy healthy, $unhealthy unhealthy"

    if ((unhealthy > 0)); then
        return 1
    fi
}
```

### Docker Compose Integration (lines 189-267)

```bash
cmd_up() {
    local profile="${1:-minimal}"
    local profiles
    profiles="${PROFILE_PRESETS[$profile]:-$profile}"

    log_info "Starting FlexStack with profile: $profile"
    log_info "Activating profiles: $profiles"

    # Build docker compose command with all profiles
    local compose_cmd=(docker compose)

    for prof in $profiles; do
        compose_cmd+=(--profile "$prof")
    done

    # Start services
    log_info "Running: ${compose_cmd[*]} up -d"
    "${compose_cmd[@]}" up -d

    # Wait for services to start
    sleep 5

    # Check health
    cmd_health
}

cmd_down() {
    log_info "Stopping all FlexStack services"
    docker compose down
    log_success "Services stopped"
}
```

---

## Profile Breakdown

### Minimal Profile
**Services:** redis, postgres, nats
**Memory:** 1,024 MB (1.0 GB)
**Use case:** Core message bus and data stores

### Dev Profile
**Services:** redis, postgres, nats, adminer, pgweb
**Memory:** 1,280 MB (1.2 GB)
**Use case:** Development with database GUIs

### AI Profile
**Services:** redis, postgres, nats, localai, temporal, temporal-ui
**Memory:** 6,400 MB (6.2 GB)
**Use case:** AI inference and workflow orchestration

### Observability Profile
**Services:** prometheus, grafana, loki
**Memory:** 1,280 MB (1.2 GB)
**Use case:** Monitoring and logging

### Identity Profile
**Services:** redis, postgres, nats, keycloak, vault
**Memory:** 2,304 MB (2.2 GB)
**Use case:** Authentication and secrets management

### Full Profile
**Services:** All services
**Memory:** ~10,240 MB (10.0 GB)
**Use case:** Complete stack

---

## References

- **Main script:** `scripts/flexstack.sh` (467 lines)
- **Profile presets:** lines 38-55
- **Service groups:** lines 57-78
- **Memory requirements:** lines 80-113
- **Up/down commands:** lines 189-267
- **Status command:** lines 269-316
- **Health checks:** lines 318-371
- **Related:** `docker/docker-compose*.yml` files

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 55/60 (91.7%)
