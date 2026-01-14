# bootstrap.sh — Script Contract

**Type:** Bash Script
**Platform:** Linux, macOS
**Layer:** L1 (Bootstrap)
**Criticality:** HIGH
**Safety:** safe
**Idempotent:** Yes
**Owner:** ripple-env maintainers

---

## Purpose

Complete end-to-end installation script for Linux and macOS that sets up:
- Nix package manager with flakes enabled
- direnv for automatic environment activation
- Pixi for Python/Conda package management
- Optional shells (zsh, nushell)
- Development environment verification

---

## Invocation

```bash
# Standard bootstrap
./bootstrap.sh

# CI mode (non-interactive)
./bootstrap.sh --ci

# Skip shell installations
./bootstrap.sh --skip-shells

# Resume from previous failure
./bootstrap.sh --resume

# Clean state and restart
./bootstrap.sh --clean

# With verification profile
./bootstrap.sh --verify --profile default

# Debug mode with logging
./bootstrap.sh --debug --log-file /tmp/bootstrap.log
```

---

## Inputs

### Command-Line Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--ci` | Flag | false | Non-interactive CI mode |
| `--skip-shells` | Flag | false | Skip zsh/nushell installation |
| `--verify` | Flag | false | Run verification after install |
| `--profile` | String | `default` | Verification profile (minimal, ci, default, full) |
| `--resume` | Flag | false | Resume from previous state |
| `--clean` | Flag | false | Clear state and restart |
| `--log-file` | Path | - | Log output to file |
| `--debug` | Flag | false | Enable debug output |
| `-h, --help` | Flag | - | Show help message |

### Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `HOME` | Required | User home directory |
| `SHELL` | Detected | Current shell for hook setup |
| `CI` | - | Detected CI environment |
| `TERM` | - | Terminal type for colors |

### Configuration Files Read

| File | Purpose |
|------|---------|
| `flake.nix` | Nix flake configuration |
| `pixi.toml` | Pixi package configuration |
| `ARIA_MANIFEST.yaml` | Component verification manifest |
| `~/.config/nix/nix.conf` | Nix configuration |
| `~/.bashrc` | Bash configuration |
| `~/.zshrc` | Zsh configuration |

---

## Outputs

### Files Created/Modified

| File | Action | Purpose |
|------|--------|---------|
| `~/.config/nix/nix.conf` | Created/Modified | Enable Nix flakes |
| `~/.bashrc` | Modified | Add direnv hook |
| `~/.zshrc` | Modified | Add direnv hook |
| `/tmp/bootstrap-state.json` | Created | State persistence for resume |
| `~/.local/state/ripple-env/bootstrap.log` | Created | Bootstrap log |

### Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | General error |
| 2 | Unsupported OS/architecture |
| 3 | Network error (after retries) |
| 4 | Nix installation failed |
| 5 | Verification failed |

---

## Side Effects

1. **Installs Nix** — System-wide Nix installation via Determinate Systems installer
2. **Modifies shell configs** — Adds direnv hooks to `.bashrc`/`.zshrc`
3. **Creates directories** — `~/.config/nix/`, `~/.local/state/ripple-env/`
4. **Downloads packages** — Nix packages and pixi packages
5. **Network access** — Required for downloads

---

## Safety Classification

**Rating:** SAFE

**Justification:**
- Idempotent (safe to run multiple times)
- Checks before modifying (won't duplicate hooks)
- State persistence enables safe resume
- No destructive operations
- All downloads verified

---

## Idempotency

**Fully Idempotent:** Yes

Each stage checks current state before acting:
- Nix: Checks if `nix` command exists
- direnv: Checks if `direnv` command exists
- Shell hooks: Checks if hook already in config
- Pixi: Checks if packages installed

```bash
# Safe to run multiple times
./bootstrap.sh
./bootstrap.sh  # Second run skips already-completed stages
```

---

## Dependencies

### System Requirements

| Dependency | Required | Check Command |
|------------|----------|---------------|
| curl | Yes | `command -v curl` |
| git | Installed by script | `command -v git` |
| bash 4+ | Yes | `bash --version` |
| Internet | Yes | Network connectivity |

### External Services

| Service | Purpose |
|---------|---------|
| https://install.determinate.systems/nix | Nix installer |
| https://github.com | Repository access |
| https://nixos.org | Nixpkgs cache |

---

## Failure Modes

### FM-001: Network Timeout

**Symptom:** Download fails during Nix installation
**Cause:** Network connectivity issues
**Recovery:**
- Automatic retry (4 attempts, exponential backoff)
- Run with `--resume` if all retries fail

### FM-002: Disk Space

**Symptom:** Installation fails partway
**Cause:** Insufficient disk space for Nix store
**Recovery:**
- Free space (20GB+ recommended)
- Run `nix-collect-garbage -d`
- Resume with `--resume`

### FM-003: Permission Denied

**Symptom:** Cannot create Nix store
**Cause:** File system permissions
**Recovery:**
- Check `/nix` permissions
- May need sudo for multi-user install

### FM-004: Shell Hook Conflict

**Symptom:** direnv not activating
**Cause:** Conflicting shell configurations
**Recovery:**
- Check `.bashrc`/`.zshrc` for duplicate hooks
- Ensure shell is sourcing config file

---

## Stage Breakdown

| Stage | Function | Lines | Idempotent | Duration |
|-------|----------|-------|------------|----------|
| 1 | detect_system | 471-547 | Yes | <1s |
| 2 | install_nix | 560-624 | Yes | 2-5min |
| 3 | enable_flakes | 627-660 | Yes | <1s |
| 4 | install_direnv | 669-695 | Yes | <30s |
| 5 | install_nom | 698-724 | Yes | <30s |
| 6 | install_git | 727-753 | Yes | <30s |
| 7 | install_gh | 756-782 | Yes | <30s |
| 8 | install_zsh | 785-817 | Skip-able | <30s |
| 9 | install_nushell | 820-852 | Skip-able | <30s |
| 10 | setup_direnv_hooks | 855-882 | Yes | <1s |
| 11 | verify_environment | 885-925 | Always | 1-3min |
| 12 | verify_pixi | 928-952 | Always | 1-5min |
| 13 | verify_manifest | 955-1006 | Optional | 1-2min |

---

## Examples

### Example 1: Fresh Install

```bash
# Download and run
curl -fsSL https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.sh -o bootstrap.sh
chmod +x bootstrap.sh
./bootstrap.sh

# Expected output:
# [1/13] Detecting system...
# [2/13] Installing Nix...
# ...
# [13/13] Verifying manifest...
# ✅ Bootstrap complete!
```

### Example 2: CI Pipeline

```bash
./bootstrap.sh --ci --skip-shells --verify --profile ci

# CI mode:
# - Non-interactive
# - Skips optional shells
# - Runs verification
# - Uses CI profile
```

### Example 3: Resume After Failure

```bash
# First run fails at stage 7
./bootstrap.sh
# Error: Network timeout at install_gh

# Resume from last successful stage
./bootstrap.sh --resume
# Resuming from stage 7...
```

---

## Integration

### Called By

| Caller | Context |
|--------|---------|
| `bootstrap.ps1` | Stage 7: InstallROS2Environment (via WSL) |
| GitHub Actions | `bootstrap-test.yml`, `test-bootstrap.yml` |
| User | Manual execution |

### Calls

| Script/Command | Purpose |
|----------------|---------|
| `scripts/validate-manifest.py` | Stage 13 verification |
| `pixi install` | Stage 12 package installation |
| `nix flake check` | Stage 11 flake verification |

---

## References

**Evidence Files:**
- `bootstrap.sh` (1160 lines)
- `README.md` lines 167-211
- `docs/GETTING_STARTED.md` lines 16-51
- `.github/workflows/bootstrap-test.yml`
- `.github/workflows/test-bootstrap.yml`

**Related Documentation:**
- [Golden Path: Bootstrap](../modules/bootstrap.md)
- [Troubleshooting](../TROUBLESHOOTING.md)
- [Getting Started](../getting-started/GETTING_STARTED.md)

---

**Last Updated:** 2026-01-14
**Contract Version:** 1.0.0
