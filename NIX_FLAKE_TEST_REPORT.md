# Nix Flakes Test Report

**Date:** January 14, 2026  
**Project:** ripple-env  
**Tester:** DeepAgent  

---

## Summary

| Flake | Location | Status |
|-------|----------|--------|
| Main Flake | `/home/ubuntu/ripple-env/flake.nix` | ⚠️ Partial Pass |
| WSL Config Flake | `ripple-wsl-binary/nixos-wsl-config/flake.nix` | ❌ Fail |

---

## Flake 1: Main Flake (`/home/ubuntu/ripple-env/flake.nix`)

### Evaluation Tests

| Test | Status | Notes |
|------|--------|-------|
| `nix flake show` | ✅ Pass | Flake structure evaluates correctly |
| `nix flake check` | ⚠️ Partial | Fails on `iso-ros2-stable` configuration |
| DevShells evaluation | ✅ Pass | All 6 shells evaluate: cuda, default, full, identity, minimal, stable |
| Checks evaluation | ✅ Pass | All 18 checks evaluate correctly |

### Build Tests

| Check | Status |
|-------|--------|
| `flake-syntax` | ✅ Pass |
| `module-syntax` | ✅ Pass |
| `lib-tests` | ✅ Pass |
| `shell-tests` | ✅ Pass |

### Available DevShells (x86_64-linux)

- `cuda` - CUDA-enabled development shell
- `default` - Standard development shell
- `full` - Full-featured development shell
- `identity` - Identity management shell
- `minimal` - Minimal development shell
- `stable` - Production-ready shell (nixos-24.11)

### Available Checks (x86_64-linux)

- `all-image-tests`
- `basic-services`
- `dev-tools`
- `docker-services`
- `flake-syntax`
- `lib-tests`
- `module-syntax`
- `module-tests-direnv`
- `module-tests-git`
- `module-tests-packages`
- `network`
- `security-hardening`
- `shell-tests`
- `test-direnv-module`
- `test-git-module`
- `test-lib-functions`
- `test-packages-module`
- `test-shell-packages`

### NixOS Configurations

| Configuration | Status | Purpose |
|--------------|--------|---------|
| `wsl-ripple` | ✅ Pass | WSL2 development image (unstable) |
| `iso-ripple` | ⚠️ Warning | ISO installer (deprecation warning on `isoImage.isoName`) |
| `vm-ripple` | ✅ Pass | VM development image (unstable) |
| `wsl-ripple-stable` | ✅ Pass | WSL2 production image (stable) |
| `iso-ros2-stable` | ❌ Fail | ISO production image - **ERROR below** |
| `vm-ros2-stable` | ✅ Pass | VM production image (stable) |

### Error: `iso-ros2-stable` Configuration

```
error: The option `image' does not exist. Definition values:
- In `/nix/store/.../flake.nix':
    {
      fileName = "nixos-ros2-24.11.20250630.50ab793-x86_64-linux.iso";
    }
```

**Root Cause:** The `image` option was added in nixos-unstable but doesn't exist in nixos-24.11 (stable). The `nix/images/iso.nix` file sets both `isoImage` (legacy) and `image` (new) options unconditionally.

**Fix Required:** In `nix/images/iso.nix`, conditionally set the `image` option only when using unstable:

```nix
# Only set new image option on unstable (not available in 24.11)
} // (if isStable then {} else {
  image = {
    fileName = "nixos-ros2-${config.system.nixos.label}-x86_64-linux.iso";
  };
})
```

---

## Flake 2: WSL Config Flake (`ripple-wsl-binary/nixos-wsl-config/flake.nix`)

### Status: ❌ Failed

### Error

```
error: attribute 'overlays' missing
at /home/ubuntu/ripple-env/ripple-wsl-binary/nixos-wsl-config/flake.nix:23:13:
    22|           overlays = [
    23|             nixos-wsl.overlays.default
        |             ^
    24|           ];
```

**Root Cause:** The `nixos-wsl` flake does not export an `overlays` output. The flake incorrectly assumes `nixos-wsl.overlays.default` exists.

**Fix Required:** Remove the overlay reference since nixos-wsl doesn't provide overlays:

```nix
let
  pkgs = import nixpkgs {
    inherit system;
    config.allowUnfree = true;
    # Remove: overlays = [ nixos-wsl.overlays.default ];
  };
```

### Additional Issues

- The flake references modules that may not exist:
  - `./configuration.nix`
  - `./modules/flexnetos-core.nix`
  - `./modules/flexnetos-services.nix`
  - `./modules/flexnetos-security.nix`
  - `./modules/flexnetos-monitoring.nix`

---

## Custom Commands Available

The main flake provides modular commands organized by category:

| Category | File | Purpose |
|----------|------|---------|
| Core | `nix/commands/core.nix` | cb, ct, ctr, ros2-env, update-deps |
| AI | `nix/commands/ai.nix` | localai, agixt, aios, sandbox-runtime |
| ROS2 | `nix/commands/ros2.nix` | ros2-clean, ros2-ws, ros2-topics, ros2-nodes |
| Infra | `nix/commands/infra.nix` | ipfs-ctl, nats-ctl, prom-ctl |
| Security | `nix/commands/security.nix` | sbom, vuln-scan, sign-artifact, pki-cert |
| Dev | `nix/commands/dev.nix` | fmt-nix, dev-check, pre-commit, swc |
| Workflow | `nix/commands/workflow.nix` | gh-issues, db-query, temporal-ctl, n8n-ctl |

---

## Warnings Observed

1. **Git tree dirty warning** - Expected during development
2. **Untrusted flake config** - Binary cache settings require `--accept-flake-config` flag
3. **Deprecation warning** - `isoImage.isoName` has been renamed to `image.fileName` (in unstable only)

---

## Recommendations

### Critical Fixes Required

1. **Fix `nix/images/iso.nix`** - Conditionally set `image` option based on `isStable` flag
2. **Fix `ripple-wsl-binary/nixos-wsl-config/flake.nix`** - Remove non-existent `nixos-wsl.overlays.default` reference

### Suggested Improvements

1. Add missing module files for the WSL config flake or remove references
2. Consider adding a CI workflow to run `nix flake check` on PRs
3. Add `--accept-flake-config` to documentation for using binary caches

---

## Test Commands Used

```bash
# Show flake structure
nix flake show

# Run all checks
nix flake check

# Evaluate specific outputs
nix eval .#devShells.x86_64-linux --apply 'attrs: builtins.attrNames attrs'
nix eval .#checks.x86_64-linux --apply 'attrs: builtins.attrNames attrs'

# Build specific checks
nix build .#checks.x86_64-linux.flake-syntax --no-link
nix build .#checks.x86_64-linux.module-syntax --no-link
nix build .#checks.x86_64-linux.lib-tests --no-link
nix build .#checks.x86_64-linux.shell-tests --no-link
```
