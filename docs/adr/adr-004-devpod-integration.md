# ADR-004: DevPod for Optional Remote Development

## Status
Proposed

## Date
2026-01-09

## Context

Some development scenarios require remote or cloud-based environments:
- **Resource-intensive tasks**: GPU simulation, ML training
- **Team standardization**: Consistent environments across mixed OS teams
- **CI/CD optimization**: Pre-built environments for faster pipelines
- **Self-hosted runners**: Reproducible GitHub Actions environments

DevPod is an open-source, client-only tool for creating devcontainer environments on any backend (Docker, Kubernetes, SSH, cloud VMs).

Current state:
- Local development works well with Nix + direnv
- No `.devcontainer/` configuration exists
- DevPod is available in nixpkgs

Forces:
- Remote development enables access to powerful hardware
- Containers add overhead to Nix builds (known issue NixOS/nix#11249)
- Team may grow beyond individual contributors
- Current bootstrap scripts already support multiple platforms

## Decision

**Add DevPod as an optional enhancement** with devcontainer configuration that delegates to the existing Nix flake.

### Implementation

1. **Add DevPod to flake.nix** (optional package):
```nix
commonPackages = with pkgs; [
  # ... existing packages
  devpod  # Remote development environments
];
```

2. **Create devcontainer configuration**:
```
.devcontainer/
├── devcontainer.json    # Uses Nix feature
├── setup.sh            # Post-create script
└── README.md           # Usage documentation
```

3. **Strategy**: Devcontainer calls Nix flake (Strategy 1)
   - Minimal duplication
   - flake.nix remains source of truth
   - Works with local `nix develop` OR DevPod

### devcontainer.json

```json
{
  "name": "ROS2 Humble Nix Environment",
  "image": "mcr.microsoft.com/devcontainers/base:ubuntu-22.04",
  "features": {
    "ghcr.io/devcontainers/features/nix:1": {
      "version": "latest",
      "extraNixConfig": "experimental-features = nix-command flakes"
    }
  },
  "postCreateCommand": "bash .devcontainer/setup.sh"
}
```

## Consequences

### Positive
- Enables cloud/remote development for resource-intensive tasks
- Team environment standardization via devcontainer.json
- Compatible with VS Code Remote, JetBrains, SSH-based editors
- No vendor lock-in (unlike GitHub Codespaces)
- Available in nixpkgs (easy installation)

### Negative
- Nix builds in containers are slow (known issue)
- Adds complexity to already sophisticated setup
- Requires infrastructure (Docker, cloud credentials)
- Not needed for most local development

### Risks
- Container overhead may frustrate developers
  - **Mitigation**: Document as optional, use prebuilds
- Storage multiplication with multiple workspaces
  - **Mitigation**: Configure auto-cleanup, shared providers
- Security of mounted credentials (SSH keys, git config)
  - **Mitigation**: Document secure practices

## Implementation Priority

**Medium** - Add as optional enhancement, not replacement for current workflow.

### Phase 1: Foundation
- [ ] Add `devpod` to commonPackages
- [ ] Create `.devcontainer/devcontainer.json`
- [ ] Create `.devcontainer/setup.sh`
- [ ] Document in README

### Phase 2: CI Integration (Future)
- [ ] Evaluate DevPod for GitHub Actions
- [ ] Configure prebuilds for faster startup
- [ ] Compare performance vs current bootstrap

### Phase 3: Team Rollout (Future)
- [ ] Create shared provider configurations
- [ ] Implement cost monitoring
- [ ] Auto-shutdown policies

## Alternatives Considered

1. **GitHub Codespaces**
   - Rejected: Vendor lock-in
   - Rejected: Higher cost at scale

2. **Gitpod**
   - Rejected: Similar lock-in concerns
   - Considered: May revisit for SaaS simplicity

3. **No remote development**
   - Partial: Current local setup works well
   - Keep as primary, add DevPod as option

## References

- [DevPod Documentation](https://devpod.sh/docs)
- [DevPod GitHub](https://github.com/loft-sh/devpod)
- [Nix in devcontainers](https://github.com/devcontainers/features/tree/main/src/nix)
- [Known Nix performance issue](https://github.com/NixOS/nix/issues/11249)
- Research: DevPod research agent report
