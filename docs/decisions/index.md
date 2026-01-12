---
title: Architecture Decision Records
description: Documented architecture decisions for ripple-env
tags:
  - architecture
  - adr
  - decisions
---

# Architecture Decision Records

This section contains Architecture Decision Records (ADRs) that document significant technical decisions made during the development of ripple-env.

## What is an ADR?

An Architecture Decision Record is a document that captures an important architectural decision made along with its context and consequences.

## Decision Records

| ADR | Title | Status |
|-----|-------|--------|
| [ADR-001](../adr/adr-001-editor-strategy.md) | Editor Strategy | Accepted |
| [ADR-002](../adr/adr-002-ai-coding-assistants.md) | AI Coding Assistants | Accepted |
| [ADR-003](../adr/adr-003-version-management.md) | Version Management | Accepted |
| [ADR-004](../adr/adr-004-devpod-integration.md) | DevPod Integration | Accepted |
| [ADR-005](../adr/adr-005-xdg-compliance.md) | XDG Compliance | Accepted |
| [ADR-006](../adr/adr-006-agixt-integration.md) | AGiXT Integration | Accepted |

## ADR Template

When creating a new ADR, use the following template:

```markdown
# ADR-XXX: Title

## Status

Proposed | Accepted | Deprecated | Superseded

## Context

What is the issue we're addressing?

## Decision

What is the change we're making?

## Consequences

What are the results of this decision?

### Positive

- Benefit 1
- Benefit 2

### Negative

- Drawback 1
- Drawback 2

## References

- Link 1
- Link 2
```

## Process

1. Copy the template above
2. Create a new file: `adr-XXX-title.md`
3. Fill in the sections
4. Submit a PR for review
5. Once approved, update this index
