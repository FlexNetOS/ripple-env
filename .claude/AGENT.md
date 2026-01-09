# Agent System

## Overview

This environment is designed as the foundation for an **agentic system** that manages:

- **Robotics** - ROS2 Humble development, simulation, and deployment
- **DevOps** - Infrastructure automation, CI/CD pipelines, container orchestration
- **Personal Life** - Task management, scheduling, automation of daily workflows
- **Work Life** - Project management, code review, documentation, collaboration

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Agent Coordinator                            │
├─────────────────────────────────────────────────────────────────────┤
│                     Architecture & Analysis Layer                    │
├─────────────────┬─────────────────┬─────────────────────────────────┤
│   Architect     │   Pre-Verify    │     Cross-Analysis              │
│   Agent         │   Agent         │     Agent                       │
├─────────────────┴─────────────────┴─────────────────────────────────┤
│                       Domain Agents Layer                            │
├─────────────────────┬─────────────────────┬─────────────────────────┤
│   Robotics Agent    │   DevOps Agent      │   Nix Agent             │
├─────────────────────┴─────────────────────┴─────────────────────────┤
│                    Shared Context & Memory                          │
├─────────────────────────────────────────────────────────────────────┤
│              AI Infrastructure (LocalAI + AGiXT)                    │
├─────────────────────────────────────────────────────────────────────┤
│                    Environment (Nix + Pixi)                         │
└─────────────────────────────────────────────────────────────────────┘
```

## Agent Types

### Architecture & Analysis Agents

#### Architect Agent
- Full-stack system design and framework selection
- Integration planning and scalability analysis
- Technology roadmaps and ADRs (Architecture Decision Records)
- Component diagrams and data flow design

#### Pre-Verify Agent
- Pre-implementation validation and compatibility checks
- Dependency analysis and security review
- Resource estimation and rollback strategy planning
- Blocks risky changes before implementation

#### Cross-Analysis Agent
- Codebase search and pattern discovery
- Impact analysis for proposed changes
- Dependency mapping and data flow tracing
- Technical debt identification

### Domain Agents

#### Robotics Agent
- ROS2 package development and testing
- Simulation environment management (Gazebo, RViz)
- Hardware interface configuration
- URDF/XACRO modeling
- Navigation and perception pipelines

#### DevOps Agent
- GitHub Actions workflow management
- Container image building (Docker, Podman)
- Infrastructure as Code (Terraform, Pulumi, Nix)
- Monitoring and observability setup
- Secret management and security scanning

#### Nix Agent
- Nix flakes and home-manager configuration
- Package management and environment setup
- Cross-platform compatibility (Linux, macOS, WSL2)
- Module development and debugging

### Future Domain Agents (Planned)

> **Note**: These agents are planned but not yet implemented. Agent definition files will be added to `.claude/agents/` when ready.

#### Personal Agent (planned)
- Calendar and scheduling integration
- Task prioritization and tracking
- Note-taking and knowledge management
- Habit tracking and goal setting
- Email and communication management

#### Work Agent (planned)
- Code review and PR management
- Documentation generation
- Meeting notes and action items
- Project timeline tracking
- Team collaboration workflows

## Capabilities

### Environment Management
- Reproducible development environments via Nix
- Package management via Pixi/Conda
- Cross-platform support (Linux, macOS, Windows/WSL2)
- Automatic environment activation with direnv

### Tool Integration
- **Version Control**: git, gh (GitHub CLI)
- **Build Systems**: colcon, cmake, ninja, cargo
- **Languages**: Python, C++, Rust, Bash, Nushell
- **Editors**: Helix with LSP support
- **Shells**: bash, zsh, nushell with starship prompt
- **AI Infrastructure**: LocalAI (local inference), AGiXT (agent platform)
- **Containers**: Docker, Docker Compose

### Memory & Context
- Session persistence across conversations
- Project-specific context loading
- Long-term memory for recurring patterns
- Cross-agent knowledge sharing

## Configuration

Agent behavior is configured through:

1. **RULES.md** - Behavioral constraints and guidelines
2. **SKILL.md** - Available capabilities and tools
3. **CLAUDE.md** - Claude-specific instructions
4. **INDEX.md** - Documentation navigation

## Usage

### Invoking Agents

```bash
# Enter the development environment
nom develop

# Agents are invoked through natural language
# The orchestrator routes to appropriate agent based on context
```

### Agent Commands

Within the environment, agents respond to:

- Natural language requests
- Slash commands (e.g., `/build`, `/test`, `/deploy`)
- Event triggers (file changes, git hooks)
- Scheduled tasks (cron-like patterns)

## Security

- Agents operate within sandboxed environments
- Sensitive operations require explicit confirmation
- Credentials are managed via secure storage (1Password, pass, etc.)
- All actions are logged and auditable

## Extensibility

New agents can be added by:

1. Defining agent capabilities in `SKILL.md`
2. Adding behavioral rules in `RULES.md`
3. Creating tool integrations as Nix modules
4. Registering with the orchestrator

## Roadmap

### Completed
- [x] LocalAI integration for local LLM inference
- [x] AGiXT platform integration via Docker
- [x] AGiXT Rust SDK bridge for ROS2 integration
- [x] XDG Base Directory compliance
- [x] DevPod cloud development support

### In Progress
- [ ] Personal Agent implementation
- [ ] Work Agent implementation
- [ ] Multi-agent collaboration patterns

### Future
- [ ] MCP (Model Context Protocol) server integration
- [ ] Voice interface support
- [ ] Mobile companion app
- [ ] Self-improvement through feedback loops
