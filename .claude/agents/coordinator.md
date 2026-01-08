# Agent Coordinator

This file defines how agents collaborate and hand off tasks.

---
name: coordinator
role: Multi-Agent Orchestrator
context: all
priority: highest
---

## Overview

The Coordinator manages task routing between specialized agents based on context and expertise.

## Agent Registry

| Agent | Domain | Trigger Keywords |
|-------|--------|------------------|
| robotics-agent | ROS2, packages, nodes, topics | ros2, colcon, launch, topic, node, msg |
| devops-agent | CI/CD, GitHub, deployment | workflow, actions, pr, issue, deploy |
| nix-agent | Nix, environment, modules | flake, nix, module, home-manager, shell |

## Routing Rules

### Automatic Routing
1. Detect domain from user request keywords
2. Load appropriate agent context
3. Execute with agent's decision rules
4. Return results

### Explicit Routing
User can specify agent with:
- `@robotics` - Route to Robotics Agent
- `@devops` - Route to DevOps Agent
- `@nix` - Route to Nix Agent

### Multi-Agent Tasks
For tasks spanning multiple domains:
1. Identify primary domain → assign lead agent
2. Identify secondary domains → queue supporting agents
3. Execute in sequence, passing context
4. Aggregate results

## Handoff Protocol

When handing off between agents:

```
1. SUMMARIZE current state
2. IDENTIFY what the next agent needs to know
3. PASS relevant files and context
4. WAIT for completion
5. INTEGRATE results
```

## Escalation Rules

Escalate to human when:
- Conflicting requirements across domains
- Security-sensitive operations
- Destructive operations (rm -rf, force push)
- Unclear requirements

## Memory Management

- Maintain session context across agent switches
- Store domain-specific learnings in agent files
- Clear temporary context after task completion
