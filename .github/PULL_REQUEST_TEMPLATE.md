## Description

<!-- Describe your changes in detail. What problem does this solve? -->

## Type of Change

<!-- Mark with [x] all that apply -->

- [ ] Bug fix (non-breaking change that fixes an issue)
- [ ] New feature (non-breaking change that adds functionality)
- [ ] Breaking change (fix or feature that would cause existing functionality to change)
- [ ] Documentation update
- [ ] Configuration change (flake.nix, pixi.toml, CI)
- [ ] Security update

## Related Issues

<!-- Link any related issues: Fixes #123, Closes #456 -->

## Testing

<!-- Describe the tests you ran to verify your changes -->

- [ ] `nix flake check` passes
- [ ] `pixi install` succeeds
- [ ] `colcon build` succeeds (if ROS2 changes)
- [ ] `colcon test` passes (if ROS2 changes)
- [ ] Manual testing performed

### Test Environment

- **OS**: <!-- e.g., Ubuntu 24.04, macOS 15, WSL2 -->
- **Architecture**: <!-- e.g., x86_64, aarch64 -->
- **GPU** (if applicable): <!-- e.g., NVIDIA RTX 4090, None -->

## Security Checklist

<!-- Required for all PRs -->

- [ ] No hardcoded secrets or credentials
- [ ] No unsafe command injection patterns (e.g., unquoted `$variables` in bash)
- [ ] No privilege escalation risks
- [ ] Dependencies are from trusted sources
- [ ] Environment variables are safely handled

## Documentation

- [ ] README updated (if user-facing changes)
- [ ] CONFLICTS.md updated (if version/compatibility changes)
- [ ] Comments added for complex code
- [ ] CHANGELOG entry added (if applicable)

## Deployment Notes

<!-- Any special considerations for deploying this change? -->

## Screenshots

<!-- If applicable, add screenshots to help explain your changes -->

## Checklist

- [ ] My code follows the project's style guidelines
- [ ] I have performed a self-review of my code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] My changes generate no new warnings
- [ ] I have added tests that prove my fix is effective or my feature works
- [ ] New and existing unit tests pass locally with my changes
