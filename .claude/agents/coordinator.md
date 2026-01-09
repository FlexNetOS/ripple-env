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

### Core Domain Agents

| Agent | Domain | Trigger Keywords | Model |
|-------|--------|------------------|-------|
| robotics-agent | ROS2, packages, nodes, topics | ros2, colcon, launch, topic, node, msg | sonnet |
| devops-agent | CI/CD, GitHub, deployment | workflow, actions, pr, issue, deploy | sonnet |
| nix-agent | Nix, environment, modules | flake, nix, module, home-manager, shell | sonnet |
| kubernetes-agent | K8s, Helm, ArgoCD | k8s, kubectl, helm, argo, cluster, pod | sonnet |
| identity-agent | Auth, Keycloak, OPA, Vault | auth, keycloak, opa, vault, token, rbac | sonnet |

### Architecture & Analysis Agents

| Agent | Domain | Trigger Keywords | Model |
|-------|--------|------------------|-------|
| architect-agent | System design, frameworks, integration | design, architecture, framework, integrate, scale, plan | sonnet |
| pre-verify-agent | Validation, compatibility, pre-checks | verify, validate, check, compatible, dependency, security | sonnet |
| cross-analysis-agent | Code search, patterns, impact | analyze, search, find, pattern, impact, usage, trace | sonnet |

### Specialized Agents

| Agent | Domain | Trigger Keywords | Model |
|-------|--------|------------------|-------|
| security-agent | Vulnerabilities, scanning, secrets | security, vuln, cve, scan, secret, trivy | sonnet |
| migration-agent | Upgrades, deprecations, version | upgrade, migrate, deprecate, version, breaking | sonnet |
| test-runner-agent | Testing, coverage, CI | test, pytest, coverage, ci, failing | haiku |
| docs-agent | Documentation, changelog | docs, readme, changelog, api docs | haiku |

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
- `@k8s` - Route to Kubernetes Agent
- `@identity` - Route to Identity Agent
- `@architect` - Route to Architect Agent
- `@verify` - Route to Pre-Verify Agent
- `@analyze` - Route to Cross-Analysis Agent
- `@security` - Route to Security Agent
- `@migrate` - Route to Migration Agent
- `@test` - Route to Test Runner Agent
- `@docs` - Route to Documentation Agent

### Multi-Agent Tasks
For tasks spanning multiple domains:
1. Identify primary domain → assign lead agent
2. Identify secondary domains → queue supporting agents
3. Execute in sequence, passing context
4. Aggregate results

### Parallel Execution
For independent subtasks, execute agents in parallel:
1. Decompose task into independent subtasks
2. Assign each subtask to appropriate agent
3. Launch all agents simultaneously
4. Collect and merge results
5. Resolve any conflicts

### Architecture Workflow
For design and implementation tasks:
```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│  Architect  │───>│  Pre-Verify  │───>│   Domain    │
│    Agent    │    │    Agent     │    │   Agents    │
└─────────────┘    └──────────────┘    └─────────────┘
       │                  ^                    │
       │                  │                    │
       └──────────────────┴────────────────────┘
              Cross-Analysis Agent
```

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
