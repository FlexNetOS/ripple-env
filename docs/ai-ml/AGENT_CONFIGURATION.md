# Agent Configuration Management

This document describes the unified agent configuration system for ripple-env.

## Overview

The agent configuration system manages:
- **Agents**: Specialized AI assistants for different domains
- **Skills**: Structured knowledge and capabilities
- **Commands**: Slash commands for quick actions
- **Models**: AI model definitions and routing rules
- **MCP Servers**: Model Context Protocol integrations

## Configuration Structure

```
.claude/
├── settings.json          # Core Claude Code settings
├── AGENT.md              # Agentic system architecture
├── CLAUDE.md             # Project instructions
├── RULES.md              # Behavioral guidelines
├── SKILL.md              # Capabilities reference
├── INDEX.md              # Documentation navigation
├── mcp-servers.json      # MCP server configuration
│
├── config/
│   ├── agent-schema.json # Unified JSON Schema
│   ├── models.json       # Multi-model registry
│   ├── schema.json       # Environment schema
│   └── env.template      # API key template
│
├── agents/               # Agent definitions (13 agents)
│   ├── coordinator.md    # Multi-agent orchestrator
│   ├── robotics-agent.md
│   ├── devops-agent.md
│   └── ...
│
├── skills/               # Skill definitions (21 skills)
│   ├── ros2-development/
│   ├── devops/
│   └── ...
│
├── commands/             # Slash commands (11 commands)
│   ├── build.md
│   ├── test.md
│   └── ...
│
├── policies/             # OPA policy files
├── prompts/              # Reusable prompts
└── hooks/                # Session hooks
```

## Agent Interaction Patterns

### 1. Coordinator-Based Routing

The coordinator agent manages task routing using keyword matching:

```
User Request → Coordinator → Domain Agent → Result
                   ↓
            Route by keywords
```

**Routing Rules:**
1. Extract keywords from user request
2. Match against agent trigger keywords
3. Load appropriate agent context
4. Execute with agent's decision rules
5. Return results to user

### 2. Explicit Agent Selection

Users can directly route to agents using `@` mentions:

```
@robotics build the navigation package
@devops create a GitHub Actions workflow
@nix update the flake inputs
```

### 3. Multi-Agent Collaboration

Complex tasks may require multiple agents:

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│  Architect  │───▸│  Pre-Verify  │───▸│   Domain    │
│    Agent    │    │    Agent     │    │   Agents    │
└─────────────┘    └──────────────┘    └─────────────┘
       │                  ▲                    │
       │                  │                    │
       └──────────────────┴────────────────────┘
              Cross-Analysis Agent
```

### 4. Handoff Protocol

When agents hand off tasks:

1. **SUMMARIZE** - Current state and findings
2. **IDENTIFY** - What the next agent needs
3. **PASS** - Relevant files and context
4. **WAIT** - For completion
5. **INTEGRATE** - Results into response

### 5. Parallel Execution

Independent subtasks can run concurrently:

```
          ┌──▸ Agent A ──┐
          │              │
Request ──┼──▸ Agent B ──┼──▸ Merge Results
          │              │
          └──▸ Agent C ──┘
```

## Agent Registry

### Core Domain Agents

| Agent | Model | Trigger Keywords |
|-------|-------|------------------|
| robotics-agent | sonnet | ros2, colcon, launch, topic, node |
| devops-agent | sonnet | workflow, actions, pr, deploy |
| nix-agent | sonnet | flake, nix, module, home-manager |
| kubernetes-agent | sonnet | k8s, kubectl, helm, argo |
| identity-agent | sonnet | auth, keycloak, opa, vault |

### Architecture & Analysis Agents

| Agent | Model | Trigger Keywords |
|-------|-------|------------------|
| architect-agent | opus | design, architecture, framework |
| pre-verify-agent | haiku | verify, validate, check |
| cross-analysis-agent | sonnet | analyze, search, pattern, impact |
| config-consistency-agent | kimi-k2-thinking | consistency, broken, reference |

### Specialized Agents

| Agent | Model | Trigger Keywords |
|-------|-------|------------------|
| security-agent | sonnet | security, vuln, cve, scan |
| migration-agent | sonnet | upgrade, migrate, deprecate |
| test-runner-agent | haiku | test, pytest, coverage |

## Model Assignment Strategy

| Tier | Model | Use Case |
|------|-------|----------|
| **High** | opus | Complex orchestration, architecture |
| **Medium** | sonnet | Domain expertise, analysis |
| **Low** | haiku | Fast validation, testing |
| **Specialized** | kimi-k2-thinking | Cross-file reasoning |

## Configuration Tools

### Validation

```bash
# Validate all configurations
python scripts/agent-config.py validate

# Validate specific type
python scripts/agent-config.py validate agents
python scripts/agent-config.py validate skills
python scripts/agent-config.py validate models
```

### Generation

```bash
# Generate new agent
python scripts/agent-config.py generate agent my-agent --model=sonnet

# Generate new skill
python scripts/agent-config.py generate skill my-skill --category=development

# Generate new command
python scripts/agent-config.py generate command my-command
```

### Listing

```bash
# List all configurations
python scripts/agent-config.py list

# List specific type
python scripts/agent-config.py list agents
python scripts/agent-config.py list skills
```

### Export

```bash
# Export unified configuration
python scripts/agent-config.py export --format=json -o unified-config.json
python scripts/agent-config.py export --format=yaml
```

### Consistency Check

```bash
# Check cross-file consistency
python scripts/agent-config.py check-consistency
```

## Configuration Schema

The unified schema (`agent-schema.json`) validates:

### Agent Definition

```json
{
  "name": "robotics-agent",
  "role": "ROS2 Development Specialist",
  "model": "sonnet",
  "context": "robotics",
  "priority": "high",
  "triggerKeywords": ["ros2", "colcon", "launch"],
  "responsibilities": [
    "Package Development",
    "Testing",
    "Launch Configuration"
  ],
  "handoffs": [
    {
      "to": "devops-agent",
      "when": "CI/CD or deployment is needed"
    }
  ]
}
```

### Skill Definition

```json
{
  "name": "ros2-development",
  "description": "ROS2 Humble development skills",
  "icon": "🤖",
  "category": "robotics",
  "tools": ["colcon", "ros2", "rosdep"]
}
```

### Model Configuration

```json
{
  "id": "claude-sonnet-4-20250514",
  "alias": "sonnet",
  "provider": "anthropic",
  "useCase": "domain leads, analysis",
  "costTier": "medium",
  "capabilities": ["text", "code", "reasoning", "tools"]
}
```

## Environment Variables

Agent configurations can reference environment variables:

| Variable | Purpose |
|----------|---------|
| `ANTHROPIC_API_KEY` | Claude API access |
| `OPENAI_API_KEY` | OpenAI models |
| `MOONSHOT_API_KEY` | Kimi K2 access |
| `LOCALAI_API_KEY` | Local inference |
| `OPENROUTER_API_KEY` | Unified model access |

Template: `.claude/config/env.template`

## Adding New Agents

1. **Generate scaffold:**
   ```bash
   python scripts/agent-config.py generate agent domain-name --model=sonnet
   ```

2. **Edit the generated file** in `.claude/agents/domain-name-agent.md`

3. **Add trigger keywords** to coordinator registry

4. **Create associated skill** if needed:
   ```bash
   python scripts/agent-config.py generate skill domain-name
   ```

5. **Validate configuration:**
   ```bash
   python scripts/agent-config.py validate
   python scripts/agent-config.py check-consistency
   ```

## Adding New Skills

1. **Generate scaffold:**
   ```bash
   python scripts/agent-config.py generate skill skill-name --category=development
   ```

2. **Edit** `.claude/skills/skill-name/SKILL.md`

3. **Reference in agent** configurations as needed

4. **Validate:**
   ```bash
   python scripts/agent-config.py validate skills
   ```

## MCP Server Configuration

MCP servers extend agent capabilities:

```json
{
  "servers": [
    {
      "name": "filesystem",
      "type": "stdio",
      "command": "npx",
      "args": ["-y", "@anthropic/mcp-filesystem"]
    }
  ]
}
```

Configuration: `.claude/mcp-servers.json`

## Permissions Model

The `settings.json` controls agent permissions:

```json
{
  "permissions": {
    "allow": [
      "Bash(git *)",
      "Bash(colcon *)",
      "Read",
      "Write"
    ],
    "deny": [
      "Bash(rm -rf /)",
      "Read(.env)"
    ]
  }
}
```

## Best Practices

1. **Single Responsibility**: Each agent should have a clear domain
2. **Clear Handoffs**: Define explicit handoff conditions
3. **Appropriate Models**: Use cheaper models for simple tasks
4. **Validation**: Always validate after configuration changes
5. **Documentation**: Keep agent documentation up to date

## Troubleshooting

### Agent Not Routing Correctly

1. Check trigger keywords in coordinator
2. Verify agent file exists in `.claude/agents/`
3. Run `agent-config.py check-consistency`

### Skill Not Loading

1. Verify SKILL.md has valid frontmatter
2. Check file path references in agent
3. Run `agent-config.py validate skills`

### Model Errors

1. Check API key environment variable
2. Verify model ID in `models.json`
3. Check routing rules match task type

## Related Documentation

- [AGENT.md](../../.claude/AGENT.md) - Agentic system architecture
- [RULES.md](../../.claude/RULES.md) - Behavioral guidelines
- [INDEX.md](../../.claude/INDEX.md) - Documentation navigation
- [Coordinator](../../.claude/agents/coordinator.md) - Multi-agent orchestration
