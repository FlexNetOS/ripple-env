---
title: Development Guide
description: Development workflow and best practices
tags:
  - contributing
  - development
---

# Development Guide

This guide covers the development workflow for contributing to ripple-env.

## Setup

```bash
# Fork and clone
git clone https://github.com/YOUR_USERNAME/ripple-env.git
cd ripple-env

# Enter development shell
nix develop
```

## Workflow

### 1. Create a Branch

```bash
git checkout -b feature/your-feature
```

### 2. Make Changes

Follow the code style guidelines:

- **Nix**: Use `nixfmt`
- **Python**: Use `ruff`
- **Shell**: Use `shellcheck`

### 3. Test

```bash
# Build
cb

# Test
ct

# Check flake
nix flake check
```

### 4. Commit

Use conventional commits:

```bash
git commit -m "feat: add new feature"
```

### 5. Push and PR

```bash
git push -u origin feature/your-feature
```

Then create a pull request on GitHub.

## Code Style

### Nix

```nix
{ config, lib, pkgs, ... }:
{
  options.module-name = {
    enable = lib.mkEnableOption "description";
  };

  config = lib.mkIf config.module-name.enable {
    # configuration
  };
}
```

### Python

```python
from typing import Optional

def example_function(param: str) -> Optional[str]:
    """Example function with type hints."""
    return param if param else None
```

### Shell

```bash
#!/usr/bin/env bash
set -euo pipefail

# Include help text
usage() {
    echo "Usage: $0 [options]"
}
```

## Documentation

Update documentation when:

- Adding new features
- Changing existing behavior
- Fixing bugs with user impact

Build docs locally:

```bash
mkdocs serve
```
