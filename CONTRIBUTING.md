# Contributing to ROS2 Humble Development Environment

Thank you for your interest in contributing! This document provides guidelines and best practices for contributing to this project.

## Code of Conduct

Be respectful, inclusive, and professional. We're all here to build great robotics software.

## Getting Started

### Prerequisites

1. **Nix**: Install via [Determinate Systems installer](https://zero-to-nix.com/start/install)
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
   ```

2. **direnv** (optional but recommended):
   ```bash
   nix profile install nixpkgs#direnv
   echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
   ```

3. **Pre-commit hooks**:
   ```bash
   pip install pre-commit
   pre-commit install
   pre-commit install --hook-type commit-msg
   ```

### Development Environment

```bash
# Clone the repository
git clone https://github.com/FlexNetOS/ros2-humble-env.git
cd ros2-humble-env

# Enter development shell
nix develop

# Or with direnv
direnv allow
```

## How to Contribute

### Reporting Bugs

1. **Search existing issues** first
2. **Create a new issue** with:
   - Clear, descriptive title
   - Steps to reproduce
   - Expected vs actual behavior
   - Environment details (OS, architecture, Nix version)
   - Relevant logs or error messages

### Suggesting Features

1. **Open a discussion** or issue
2. Describe the use case
3. Explain why this benefits the project
4. Consider implementation approach

### Submitting Changes

1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feat/my-feature
   ```
3. **Make your changes**
4. **Run checks**:
   ```bash
   nix flake check
   pixi install
   pre-commit run --all-files
   ```
5. **Commit with conventional commits**
6. **Push and create a Pull Request**

## Commit Message Format

We use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <description>

[optional body]

[optional footer(s)]
```

### Types

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation only |
| `style` | Formatting, no code change |
| `refactor` | Code change that neither fixes nor adds |
| `test` | Adding or updating tests |
| `chore` | Maintenance tasks |
| `ci` | CI/CD changes |
| `build` | Build system changes |
| `perf` | Performance improvements |

### Scopes

| Scope | Description |
|-------|-------------|
| `nix` | flake.nix, lib/, modules/ |
| `pixi` | pixi.toml, ROS2 packages |
| `cuda` | CUDA/GPU configuration |
| `docs` | Documentation |
| `ci` | GitHub Actions |
| `bootstrap` | Bootstrap scripts |

### Examples

```bash
# Feature
git commit -m "feat(nix): add Python 3.13 for non-ROS2 tools"

# Bug fix
git commit -m "fix(cuda): gate CUDA devshell to Linux only"

# Documentation
git commit -m "docs: update CONFLICTS.md with Python version matrix"

# Breaking change
git commit -m "feat(pixi)!: require Python 3.11+"
```

## Code Style

### Nix

- Use `nixfmt-rfc-style` (included in devshell)
- Document options with comments
- Use `lib.mkOption` with proper types
- Follow [Nix best practices](https://nix.dev/guides/best-practices)

```nix
# Good
commonPackages = with pkgs; [
  pixi       # Package manager
  git        # Version control
];

# Avoid
commonPackages = with pkgs; [pixi git];  # No comments, hard to read
```

### Shell Scripts

- Use `shellcheck` (enforced by pre-commit)
- Include `set -euo pipefail` in scripts
- Quote variables: `"$VAR"` not `$VAR`
- Document complex logic

### Python

- Follow PEP 8
- Use type hints
- Format with `black`
- Sort imports with `isort`

### Documentation

- Use Markdown
- Keep lines under 100 characters where practical
- Include code examples
- Update related docs when making changes

## Testing

### Before Submitting

```bash
# Nix flake validation
nix flake check

# Pixi environment
pixi install
pixi run python -c "import rclpy; print('ROS2 ready')"

# Pre-commit hooks
pre-commit run --all-files

# ROS2 build (if applicable)
colcon build --symlink-install
colcon test
```

### CI Checks

All PRs must pass:

- [ ] `nix flake check`
- [ ] Pre-commit hooks
- [ ] Security scan (Trivy)
- [ ] Platform tests (Linux, macOS)

## Branch Naming

```
<type>/<short-description>

Examples:
feat/cuda-13-support
fix/macos-library-path
docs/contributing-guide
```

## Pull Request Process

1. **Fill out the PR template completely**
2. **Link related issues**
3. **Ensure CI passes**
4. **Request review from maintainers**
5. **Address feedback**
6. **Squash commits if requested**

### PR Title Format

Same as commit messages:
```
feat(nix): add CUDA 13.x support with fallback
fix(pixi): resolve PyTorch version constraint
docs: add CONTRIBUTING.md
```

## Branch Protection

The `main` branch has these protections:

- Require pull request reviews (1 reviewer)
- Require status checks to pass
- Require branches to be up to date
- Require signed commits (recommended)

## Release Process

1. **Version bumps** follow [SemVer](https://semver.org/)
2. **Changelog** is updated
3. **Tag** is created: `git tag -s v1.2.3`
4. **Release notes** are generated

## Getting Help

- **Discussions**: Use GitHub Discussions for questions
- **Issues**: For bugs and feature requests
- **Documentation**: Check `docs/` folder and README

## Recognition

Contributors will be recognized in:

- Release notes
- README.md (significant contributions)
- Git history (always)

---

Thank you for contributing to the ROS2 development community!
