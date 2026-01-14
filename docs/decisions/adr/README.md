# Architecture Decision Records

This directory contains Architecture Decision Records (ADRs) documenting significant technical decisions for the ripple-env project.
This project adheres to [Architecture Decision Records](https://github.com/joelparkerhenderson/architecture-decision-record) and [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## ADR Index

| ID | Title | Status | Date |
|----|-------|--------|------|
| [ADR-001](adr-001-editor-strategy.md) | Editor Strategy: Helix Default with Optional LazyVim | Accepted | 2026-01-09 |
| [ADR-002](adr-002-ai-coding-assistants.md) | AI Coding Assistants: aichat + aider + Optional Neovim | Accepted | 2026-01-09 |
| [ADR-003](adr-003-version-management.md) | Version Management: Nix + Pixi (Skip mise/0install) | Accepted | 2026-01-09 |
| [ADR-004](adr-004-devpod-integration.md) | DevPod for Optional Remote Development | Proposed | 2026-01-09 |
| [ADR-005](adr-005-xdg-compliance.md) | XDG Base Directory Compliance | Accepted | 2026-01-09 |
| [ADR-006](adr-006-agixt-integration.md) | AGiXT Integration: Docker + LocalAI | Proposed | 2026-01-09 |

## ADR Template

```markdown
# ADR-XXX: [Title]

## Status
Proposed | Accepted | Deprecated | Superseded

## Date
YYYY-MM-DD

## Context
[Describe the situation and forces at play]

## Decision
[Describe the decision made]

## Consequences
### Positive
- [Benefit 1]

### Negative
- [Drawback 1]

### Risks
- [Risk 1]

## References
- [Link to relevant resources]
```

## References

- [Architecture Decision Records](https://github.com/joelparkerhenderson/architecture-decision-record)
- [Michael Nygard's ADR Template](https://cognitect.com/blog/2011/11/15/documenting-architecture-decisions)
