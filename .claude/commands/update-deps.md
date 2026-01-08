---
name: update-deps
description: Update project dependencies (Nix and Pixi)
---

Update all project dependencies including Nix flake inputs and Pixi packages.

## Usage

Update both Nix and Pixi dependencies.

## Steps

### Nix Dependencies
```bash
nix flake update                      # Update all inputs
nix flake lock --update-input nixpkgs # Update specific input
```

### Pixi Dependencies
```bash
pixi update                           # Update all packages
pixi add ros-humble-<pkg>             # Add new package
```

## After Updating

1. Test the development shell: `nom develop`
2. Run `nix flake check`
3. Build and test: `cb && ct`
4. Commit lock files if everything works
