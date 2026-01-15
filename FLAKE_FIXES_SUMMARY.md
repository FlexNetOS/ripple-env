# Nix Flake Fixes Summary

**Date:** January 14, 2026

## Changes Made

### 1. Fixed `iso-ros2-stable` Configuration (nix/images/iso.nix)

**Problem:** The `image` option doesn't exist in nixos-24.11 stable, causing the `iso-ros2-stable` configuration to fail.

**Solution:** Made the `image` block conditional - it's only included for unstable builds:

```nix
] ++ (if isStable then [] else [
  # Use new image module format (fixes deprecation warning)
  # Only available in unstable, not in nixos-24.11 stable
  ({ config, ... }: {
    image.fileName = "nixos-ros2-${config.system.nixos.label}-x86_64-linux.iso";
  })
]);
```

Also added `specialArgs = { inherit isStable; };` to pass the `isStable` variable into modules.

### 2. Fixed WSL Config Flake (ripple-wsl-binary/nixos-wsl-config/flake.nix)

**Problem:** Referenced non-existent `nixos-wsl.overlays.default`.

**Solution:** Removed the overlay reference and added a comment explaining that WSL functionality is provided via `nixosModules.wsl` instead:

```nix
pkgs = import nixpkgs {
  inherit system;
  config.allowUnfree = true;
  # Note: nixos-wsl does not export overlays.default
  # WSL functionality is provided via nixosModules.wsl instead
  overlays = [];
};
```

### 3. Deduplicated Docker Configurations

**Problem:** `ripple-wsl-binary/nixos-wsl-config/config/docker/` contained 30+ duplicate docker-compose files that had drifted out of sync with the main `docker/` directory.

**Solution:**
- Removed all duplicate docker config files from WSL directory
- Created a symlink: `ripple-wsl-binary/nixos-wsl-config/config/docker → ../../../docker`
- Added `ripple-wsl-binary/nixos-wsl-config/config/README.md` explaining the relationship

**Impact:** Eliminated ~6,900 lines of duplicate code and prevents future configuration drift.

### 4. Verified ripple-wsl-binary Package Sync

**Findings:**
- WSL docker configs were significantly outdated (older image versions, missing config entries)
- Now resolved via symlink to main docker directory
- WSL flake has a structural issue with `eachDefaultSystem` + `nixosConfigurations` (pre-existing, not caused by fixes)

### 5. Investigated config/dockerfiles Location

**Finding:** The `config/dockerfiles` directory is intentionally located outside `docker/` directory:
- `docker/` - Contains docker-compose files and runtime configs
- `config/dockerfiles/` - Contains custom Dockerfiles for source builds (e.g., `Dockerfile.open-lovable`)

The path `../config/dockerfiles` in `docker/docker-compose.ui.yml` correctly resolves when running from `docker/` directory. This is a design choice to separate source-build Dockerfiles with other configuration files.

## Verification Results

Main flake check (`nix flake check --no-build`):
- ✅ `wsl-ripple` - passes
- ✅ `iso-ripple` - passes (with deprecation warning)
- ✅ `vm-ripple` - passes
- ✅ `wsl-ripple-stable` - passes
- ✅ `iso-ros2-stable` - **NOW PASSES** (was failing before fix)
- ✅ `vm-ros2-stable` - passes
- ⚠️ Some checks have unrelated impure path issues (pre-existing)

WSL config flake:
- ✅ Overlay error resolved
- ⚠️ Has structural issue with `eachDefaultSystem` pattern (pre-existing, separate fix needed)

## Files Modified

1. `nix/images/iso.nix` - Made image option conditional
2. `ripple-wsl-binary/nixos-wsl-config/flake.nix` - Removed invalid overlay
3. `ripple-wsl-binary/nixos-wsl-config/config/docker` - Now a symlink
4. `ripple-wsl-binary/nixos-wsl-config/config/README.md` - New file explaining structure
