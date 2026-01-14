# ADR-006: AGiXT Integration Strategy

## Status
Proposed

## Date
2026-01-09

## Context

The project needs an AI agent orchestration platform for:
- Multi-robot fleet coordination
- Natural language command processing
- Task automation and workflow management
- Integration with ROS2 topics/services

Three key components were evaluated:
1. **AGiXT** - Dynamic AI Agent Automation Platform (github.com/Josh-XT/AGiXT)
2. **AGiXT Rust SDK** - Native Rust bindings (github.com/AGiXT/rust-sdk)
3. **Local LLM inference** - ezlocalai vs LocalAI

Forces:
- AGiXT is Docker-only (no Nix package available)
- This project uses Nix flakes for reproducibility
- ROS2 robotics requires low-latency inference
- Multi-robot scenarios need distributed inference capability

## Decision

**Adopt AGiXT via Docker Compose with LocalAI as the inference backend.**

### Component Decisions

#### 1. AGiXT Platform: Docker Compose

AGiXT requires Docker and cannot be natively integrated into Nix:

| Aspect | Consideration |
|--------|---------------|
| **Tech Stack** | FastAPI + PostgreSQL + MinIO |
| **Ports** | 7437 (API), 3437 (UI), 5432 (DB), 9000/9001 (S3) |
| **Python** | 3.10+ (compatible with ROS2 3.11) |
| **Extensions** | 60+ built-in (GitHub, Discord, IoT, databases) |

**Integration approach**: Run AGiXT in Docker alongside Nix-managed ROS2 environment, communicate via REST API.

#### 2. Local Inference: LocalAI (NOT ezlocalai)

| Criteria | ezlocalai | LocalAI | Winner |
|----------|-----------|---------|--------|
| **Nix Package** | Not available | `pkgs.local-ai` (v2.24.2) | LocalAI |
| **Edge Deployment** | Limited | Excellent (Jetson, ARM64) | LocalAI |
| **Resource Footprint** | 8GB minimum | 2-4GB configurable | LocalAI |
| **Distributed Inference** | Via fallback | P2P federation | LocalAI |
| **OpenAI API Compatible** | Yes | Yes (drop-in) | Tie |
| **Model Support** | GGUF only | GGUF, GGML, Safetensors, etc. | LocalAI |

**Rationale**: LocalAI aligns with Nix-first philosophy and supports robotics edge deployment.

#### 3. AGiXT Rust SDK: Future Integration Path

| Attribute | Value |
|-----------|-------|
| **Version** | 0.1.0 (Alpha) |
| **Maturity** | Pre-release, API unstable |
| **Nix Compatible** | Yes (pure Rust, no C deps) |
| **Async Runtime** | Tokio 1.0 |
| **Recommendation** | Experimental use only |

**Decision**: Track SDK development; consider for native ROS2-AGiXT bridge when stable (v1.0+).

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Nix Development Shell                         │
├─────────────────────────────────────────────────────────────────┤
│  ROS2 Humble (Pixi)     │  LocalAI (Nix)      │  Tools (Nix)    │
│  - rclpy, rclcpp        │  - Port 8080        │  - aichat       │
│  - rosdep, colcon       │  - GGUF models      │  - aider        │
│  - PyTorch/ML stack     │  - CPU/GPU          │  - promptfoo    │
└─────────────────────────────────────────────────────────────────┘
              │                      │
              │ REST API             │ OpenAI API
              ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Compose Stack                          │
├─────────────────────────────────────────────────────────────────┤
│  AGiXT API (7437)       │  PostgreSQL (5432)  │  MinIO (9000)   │
│  - FastAPI backend      │  - Agent memory     │  - File storage │
│  - Extension system     │  - Conversations    │  - Models       │
│  - Chain orchestration  │  - User data        │  - Workspaces   │
├─────────────────────────────────────────────────────────────────┤
│  AGiXT Interactive (3437)                                        │
│  - Next.js UI                                                    │
│  - Agent configuration                                           │
└─────────────────────────────────────────────────────────────────┘
```

### Integration Points

| Interface | Protocol | Purpose |
|-----------|----------|---------|
| ROS2 → AGiXT | REST (7437) | Natural language commands |
| AGiXT → LocalAI | OpenAI API (8080) | LLM inference |
| AGiXT → ROS2 | WebSocket/Webhook | Real-time telemetry |
| AGiXT → NATS | TCP (4222) | Multi-robot messaging |

### Port Allocation (No Conflicts)

| Port | Service | Status |
|------|---------|--------|
| 4222 | NATS | Existing |
| 7437 | AGiXT API | New |
| 3437 | AGiXT UI | New |
| 5432 | PostgreSQL | New (AGiXT) |
| 8080 | LocalAI | New |
| 9000 | MinIO | New |
| 9090 | Prometheus | Existing |

## Consequences

### Positive

- **Powerful orchestration**: AGiXT's 60+ extensions enable diverse integrations
- **Nix-native inference**: LocalAI maintains declarative approach
- **Separation of concerns**: Docker for complex Python services, Nix for robotics
- **Edge-ready**: LocalAI's P2P supports multi-robot fleets
- **Future-proof**: Rust SDK enables native integration path

### Negative

- **Docker dependency**: AGiXT cannot be fully managed by Nix
- **Resource overhead**: Docker + PostgreSQL + MinIO adds ~1-2GB RAM
- **Complexity**: Two orchestration layers (Docker Compose + Nix)
- **Network boundary**: API calls between Nix and Docker environments

### Risks

1. **AGiXT API changes**
   - Mitigation: Pin Docker image versions, track changelog

2. **LocalAI model compatibility**
   - Mitigation: Test models with ROS2 command parsing before deployment

3. **Rust SDK instability**
   - Mitigation: Abstract SDK behind internal interface, pin version

4. **Docker-Nix networking**
   - Mitigation: Use host networking mode, document firewall requirements

## Implementation Plan

### Phase 1: LocalAI Integration
```nix
# flake.nix - Uncomment existing local-ai
commonPackages = with pkgs; [
  local-ai  # Uncomment line 146
];
```

### Phase 2: Docker Compose for AGiXT
```yaml
# docker-compose.agixt.yml
services:
  agixt:
    image: joshxt/agixt:main
    ports: ["7437:7437"]
    environment:
      AGIXT_API_KEY: ${AGIXT_API_KEY}
      OPENAI_API_BASE: http://host.docker.internal:8080/v1
```

### Phase 3: ROS2 Bridge Development
- Create Python ROS2 node for AGiXT API communication
- Implement action server for natural language commands
- Add Prometheus metrics for AGiXT chain execution

### Phase 4: Agent Infrastructure
```
.claude/
├── skills/agixt-orchestration/README.md  # New
├── agents/agixt-agent.md                 # New
└── commands/agixt-chat.md                # New
```

## Alternatives Considered

### 1. AIOS Agent SDK Instead of AGiXT

| Aspect | AIOS | AGiXT |
|--------|------|-------|
| Installation | pip + Pixi | Docker only |
| Extensions | Limited | 60+ |
| UI | None | Full Next.js |
| Maturity | Research | Production |

**Rejected**: AIOS lacks UI and extension ecosystem for production use.

### 2. ezlocalai Instead of LocalAI

**Rejected**: ezlocalai lacks Nix package and requires Docker, negating benefits.

### 3. AGiXT Rust SDK for Native Integration

**Deferred**: SDK is alpha (v0.1.0); reconsider when stable.

### 4. No Agent Platform (Claude Code Only)

**Rejected**: Need persistent agents, memory, and multi-robot coordination beyond CLI.

## References

- [AGiXT GitHub](https://github.com/Josh-XT/AGiXT)
- [AGiXT Rust SDK](https://github.com/AGiXT/rust-sdk)
- [LocalAI Documentation](https://localai.io/)
- [ezlocalai GitHub](https://github.com/DevXT-LLC/ezlocalai)
- [ADR-003: Version Management](adr-003-version-management.md) - Why no 0install
- [CONFLICTS.md](../CONFLICTS.md) - AGiXT Docker constraint documented
