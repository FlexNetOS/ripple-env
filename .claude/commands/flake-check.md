---
name: flake-check
description: Validate the Nix flake configuration
---

Check the Nix flake for errors and validate configuration.

## Usage

Run `nix flake check` to validate the flake.

## Steps

1. Run `nix flake check` for full validation
2. Or `nix flake check --no-build` for quick syntax check
3. Run `nix flake show` to see all outputs

## Common Issues

- Missing input: Add to `inputs` in flake.nix
- Syntax error: Check Nix syntax with `nix-instantiate --parse`
- Evaluation error: Run `nix eval .#output` for specific errors
