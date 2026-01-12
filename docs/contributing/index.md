---
title: Contributing
description: How to contribute to ripple-env
tags:
  - contributing
  - development
---

# Contributing

Thank you for your interest in contributing to ripple-env! This guide will help you get started.

## Quick Start

```bash
# Fork and clone
git clone https://github.com/YOUR_USERNAME/ripple-env.git
cd ripple-env

# Set up development environment
nix develop

# Create a feature branch
git checkout -b feature/your-feature

# Make changes and commit
git add .
git commit -m "feat: add your feature"

# Push and create PR
git push -u origin feature/your-feature
```

## Documentation

<div class="grid cards" markdown>

-   :material-file-document:{ .lg .middle } __Code of Conduct__

    ---

    Community guidelines and expectations

    [:octicons-arrow-right-24: Code of Conduct](code-of-conduct.md)

-   :material-code-braces:{ .lg .middle } __Development Guide__

    ---

    Development workflow and best practices

    [:octicons-arrow-right-24: Development](development.md)

</div>

## Commit Message Format

We use conventional commits:

```
<type>: <description>

[optional body]

[optional footer]
```

### Types

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation changes |
| `style` | Formatting, no code change |
| `refactor` | Code restructuring |
| `test` | Adding tests |
| `chore` | Maintenance tasks |

### Examples

```bash
# Feature
git commit -m "feat: add CUDA support for LocalAI"

# Bug fix
git commit -m "fix: resolve channel conflict in pixi.toml"

# Documentation
git commit -m "docs: update installation guide for WSL2"
```

## Code Style

### Nix

- Use `nixfmt` for formatting
- Document modules with comments
- Use `flake-parts` patterns

### Python

- Follow PEP 8
- Use type hints
- Run `ruff check` and `ruff format`

### Shell

- Use `shellcheck` for linting
- Include help documentation
- Handle errors with `set -e`

## Pull Request Process

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run tests: `colcon test`
5. Submit a pull request
6. Wait for review

!!! tip "Good PRs"
    - Keep changes focused and small
    - Include tests for new features
    - Update documentation as needed
    - Reference related issues
