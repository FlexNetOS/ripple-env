# Script Contract: bootstrap.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `bootstrap.sh`

---

## Purpose

Complete end-to-end installation for ripple-env development environment. Installs all dependencies using Nix, sets up shells, configures direnv, and verifies the environment is ready for development.

This is the primary entry point for setting up ripple-env on Linux/macOS systems.

---

## Invocation

```bash
./bootstrap.sh [OPTIONS]
```

**Options:**
- `--ci` - Run in CI mode (non-interactive, skip optional components)
- `--skip-shells` - Skip installing zsh and nushell
- `--verify` - Run post-install verification using ARIA manifest
- `--profile PROFILE` - Verification profile (minimal, ci, default, full)
- `--resume` - Resume from last saved state (if available)
- `--clean` - Clean state and start fresh
- `--debug` - Enable debug output
- `--log-file PATH` - Write logs to specified file
- `--help, -h` - Show help message

**Examples:**
```bash
# Standard installation
./bootstrap.sh

# CI installation with verification
./bootstrap.sh --ci --verify --profile ci

# Resume after failure
./bootstrap.sh --resume

# Clean start with logging
./bootstrap.sh --clean --log-file install.log
```

---

## Inputs

### Arguments
| Argument | Required | Default | Description |
|----------|----------|---------|-------------|
| `--ci` | No | `false` | Non-interactive mode for CI environments |
| `--skip-shells` | No | `false` | Skip zsh/nushell installation |
| `--verify` | No | `false` | Run post-install verification |
| `--profile` | No | `default` | Verification profile (minimal, ci, default, full) |
| `--resume` | No | `false` | Resume from saved state |
| `--clean` | No | `false` | Clear state before starting |
| `--debug` | No | `false` | Enable debug logging |
| `--log-file` | No | `/tmp/bootstrap-<timestamp>.log` | Log file path |

### Environment Variables
| Variable | Required | Default | Description |
|----------|----------|---------|-------------|
| `TMPDIR` | No | `/tmp` | Temporary directory for downloads |
| `DEBUG` | No | `false` | Enable debug output |
| `NETWORK_RETRY_ATTEMPTS` | No | `4` | Number of retry attempts for network operations |
| `NETWORK_RETRY_DELAY` | No | `2` | Initial delay in seconds between retries |
| `XDG_STATE_HOME` | No | `$HOME/.local/state` | State directory base path |

### Configuration Files
| File | Required | Purpose |
|------|----------|---------|
| `flake.nix` | Yes | Nix flake defining development environment |
| `pixi.toml` | Yes | Pixi package definitions |
| `ARIA_MANIFEST.yaml` | No | Component verification manifest (optional) |
| `scripts/validate-manifest.py` | No | Manifest validator (optional) |
| `scripts/generate-verification.py` | No | Verification script generator (optional) |
| `scripts/verify-components.sh` | No | Generated verification script (optional) |

---

## Outputs

### Files Created
| File | Purpose |
|------|---------|
| `${XDG_STATE_HOME}/ripple-env/bootstrap.state` | Installation state for resume functionality |
| `$HOME/.config/nix/nix.conf` | Nix configuration with flakes enabled |
| `$HOME/.bashrc` | Modified to include direnv hook (if exists) |
| `$HOME/.zshrc` | Modified to include direnv hook (if exists) |
| `/tmp/bootstrap-<timestamp>.log` | Installation log (if not in CI mode) |

### Standard Output
- Color-coded log messages (INFO, SUCCESS, WARN, ERROR, DEBUG)
- Installation progress for each stage
- Final summary with next steps

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success - all stages completed |
| `1` | Failure - see log for details |

**Failure Modes Captured:**
- Network failures during downloads (lines 580-585)
- Unsupported architecture (lines 510-515)
- Unsupported OS (lines 539-542)
- Empty installer download (lines 588-591)
- Nix installation failure (lines 597-607)
- Command not found after installation (lines 616-620, 688-691, 718-720, etc.)
- Flake check failure (lines 909-914)
- Missing flake.nix or pixi.toml (lines 895-898, 938-941)

---

## Side Effects

### System Modifications
1. **Installs Nix package manager** (multi-user mode via Determinate Systems installer)
   - Creates `/nix` directory and store
   - Creates nix-daemon systemd service (Linux)
   - Adds nix daemon to launchd (macOS)
   - Evidence: lines 560-624

2. **Modifies PATH** via Nix profile scripts
   - Sources `/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh`
   - Evidence: lines 405-425

3. **Installs tools via Nix profile**
   - direnv (line 682)
   - nom (nix-output-monitor) (line 711)
   - git (line 740)
   - gh (GitHub CLI) (line 769)
   - zsh (line 804, optional)
   - nushell (line 839, optional)

4. **Modifies shell RC files**
   - Adds `eval "$(direnv hook bash)"` to `~/.bashrc` (lines 863-869)
   - Adds `eval "$(direnv hook zsh)"` to `~/.zshrc` (lines 871-878)
   - Evidence: Not done in CI mode (lines 1130-1135)

5. **Creates Nix configuration**
   - Creates `~/.config/nix/nix.conf` with:
     - `experimental-features = nix-command flakes` (line 647)
     - `accept-flake-config = true` (line 655)
   - Evidence: lines 627-660

6. **Runs pixi install** (populates conda/pixi environments)
   - Evidence: lines 928-952

### State Management
- Creates state directory: `${XDG_STATE_HOME}/ripple-env/`
- Writes state file after each successful stage
- State file tracks: `LAST_COMPLETED_STAGE`, `STARTED_AT`, `LAST_UPDATED`, `BOOTSTRAP_VERSION`
- Evidence: lines 152-251

### Network Activity
- Downloads Nix installer from `https://install.determinate.systems/nix`
- Downloads packages via Nix (nixpkgs channel)
- Downloads Pixi packages via conda-forge
- Evidence: lines 549-557, retry logic lines 108-147

---

## Safety Classification

**🟡 CAUTION**

**Rationale:**
- Modifies system PATH and installs system-wide package manager (Nix)
- Modifies shell configuration files (~/.bashrc, ~/.zshrc)
- Requires root access for multi-user Nix installation (sudo)
- Makes network requests to external servers
- Starts system daemons (nix-daemon)

**Safe for:**
- Fresh development machine setup
- CI environments (with `--ci` flag)
- Idempotent re-runs (detects existing installations)

**Unsafe for:**
- Production servers without review
- Systems with conflicting Nix installations
- Environments with restricted sudo access

---

## Idempotency

**✅ IDEMPOTENT**

The script is designed to be safely re-run:

1. **Command existence checks** before installation (lines 565-570, 674-678, 703-707, etc.)
   ```bash
   if check_command nix; then
       log_info "Nix is already installed"
       nix --version
       save_state "install_nix"
       return 0
   fi
   ```

2. **Nix configuration checks** avoid duplicate entries (lines 644-649, 652-657)
   ```bash
   if grep -qE '^\s*experimental-features\s*=.*\bflakes\b' "$nix_conf"; then
       log_info "Flakes already enabled"
   else
       echo "experimental-features = nix-command flakes" >> "$nix_conf"
   fi
   ```

3. **Shell RC file checks** prevent duplicate hooks (lines 865, 874)
   ```bash
   if ! grep -q 'eval "$(direnv hook bash)"' "$bashrc"; then
       echo 'eval "$(direnv hook bash)"' >> "$bashrc"
   fi
   ```

4. **State persistence** enables resume functionality (lines 152-251)
   - Each stage saved after completion
   - `--resume` flag skips completed stages
   - `--clean` flag clears state for fresh start

**Evidence:** All installation functions follow check-before-install pattern.

---

## Dependencies

### Required Tools (Pre-installation)
| Tool | Purpose | Availability |
|------|---------|--------------|
| `bash` | Script interpreter | System default |
| `curl` | Download Nix installer | System or installed by script |
| `sudo` | Root access for Nix install | System default |
| `uname` | System detection | System default |
| `mktemp` | Temporary file creation | System default |

### Installed Tools (During execution)
| Tool | Stage | Evidence |
|------|-------|----------|
| `nix` | install_nix | lines 560-624 |
| `direnv` | install_direnv | lines 669-695 |
| `nom` | install_nom | lines 698-724 |
| `git` | install_git | lines 727-753 |
| `gh` | install_gh | lines 756-782 |
| `zsh` | install_zsh | lines 785-817 |
| `nushell` | install_nushell | lines 820-852 |

### External Services
| Service | Purpose | Evidence |
|---------|---------|----------|
| `https://install.determinate.systems/nix` | Nix installer | lines 555-556 |
| Nix binary cache | Package downloads | Implicit in nix profile install |
| Conda-forge | Pixi packages | Implicit in pixi install |

### Required Files in Repository
| File | Stage | Evidence |
|------|-------|----------|
| `flake.nix` | verify_environment | lines 895-898 |
| `pixi.toml` | verify_pixi | lines 938-941 |
| `ARIA_MANIFEST.yaml` | verify_manifest | lines 965-969 (optional) |

---

## Failure Modes

### Network Failures
**Symptom:** Download failures, timeouts
**Exit Code:** 1
**Recovery:** Automatic retry with exponential backoff (4 attempts, 2s → 4s → 8s → 16s)
**Evidence:** lines 108-147, 580-585

**Example:**
```
[ERROR] Failed to download Nix installer after multiple attempts
[ERROR] Please check your network connection and try again
```

### Unsupported Platform
**Symptom:** Unsupported OS or architecture
**Exit Code:** 1
**Recovery:** None (manual intervention required)
**Evidence:** lines 510-515, 539-542

**Example:**
```
[ERROR] Unsupported architecture: armv7l
[ERROR] Supported architectures: x86_64, aarch64
```

### Nix Installation Failure
**Symptom:** Installer script fails or nix command not found after install
**Exit Code:** 1
**Recovery:** Check installer output, verify prerequisites
**Evidence:** lines 597-620

**Example:**
```
[ERROR] Nix installation completed but 'nix' command not found in PATH
[ERROR] Try sourcing your shell profile or restarting your shell
```

### Flake Evaluation Failure
**Symptom:** Invalid flake.nix syntax or missing dependencies
**Exit Code:** 1
**Recovery:** Fix flake.nix, run `nix flake check`
**Evidence:** lines 909-914

**Example:**
```
[ERROR] nix flake check failed
[INFO] flake output (for debugging):
<flake structure>
```

### Permission Errors
**Symptom:** Cannot write to ~/.config or ~/.local
**Exit Code:** 1
**Recovery:** Check file permissions, disk space
**Evidence:** Implicit in mkdir -p operations

### State Corruption
**Symptom:** Bootstrap fails to resume properly
**Exit Code:** 1
**Recovery:** Run `./bootstrap.sh --clean` to clear state
**Evidence:** lines 204-207, 383-387

**Usage:**
```bash
./bootstrap.sh --clean
```

---

## Retry and Backoff Strategy

### Network Operations
**Function:** `retry_with_backoff` (lines 108-141)

**Parameters:**
- `max_attempts`: Maximum retry attempts
- `initial_delay`: Starting delay in seconds
- `command`: Command to retry

**Backoff Strategy:**
- Initial delay: 2 seconds (default)
- Exponential backoff: delay × 2 after each failure
- Maximum delay: 60 seconds
- Default attempts: 4

**Evidence:**
```bash
# lines 143-147
retry_network() {
    retry_with_backoff "${NETWORK_RETRY_ATTEMPTS:-4}" "${NETWORK_RETRY_DELAY:-2}" "$@"
}
```

**Applied to:**
- Nix installer download (line 581)
- direnv installation (line 682)
- nom installation (line 711)
- git installation (line 740)
- gh installation (line 769)
- zsh installation (line 804)
- nushell installation (line 839)

---

## Stage Execution Order

The script executes 13 stages in order:

| # | Stage | Function | Evidence |
|---|-------|----------|----------|
| 1 | detect_system | Detect OS and architecture | lines 471-547 |
| 2 | install_nix | Install Nix package manager | lines 560-624 |
| 3 | enable_flakes | Enable flakes in nix.conf | lines 627-660 |
| 4 | install_direnv | Install direnv via Nix | lines 669-695 |
| 5 | install_nom | Install nix-output-monitor | lines 698-724 |
| 6 | install_git | Install git via Nix | lines 727-753 |
| 7 | install_gh | Install GitHub CLI | lines 756-782 |
| 8 | install_zsh | Install zsh (optional) | lines 785-817 |
| 9 | install_nushell | Install nushell (optional) | lines 820-852 |
| 10 | setup_direnv_hooks | Add direnv hooks to shells | lines 855-882 |
| 11 | verify_environment | Verify flake and devshell | lines 885-925 |
| 12 | verify_pixi | Run pixi install | lines 928-952 |
| 13 | verify_manifest | Run ARIA manifest verification | lines 955-1006 |

**Stage Definition:** lines 157-171

**Resume Logic:**
- State saved after each stage (lines 191-202)
- Stages skipped if already completed in resume mode (lines 217-251)
- State cleared on successful completion (line 1150)

---

## Logging System

### Log Levels
| Level | Function | Color | Purpose |
|-------|----------|-------|---------|
| INFO | `log_info` | Blue | General information |
| SUCCESS | `log_success` | Green | Successful operations |
| WARN | `log_warn` | Yellow | Non-fatal warnings |
| ERROR | `log_error` | Red | Fatal errors |
| DEBUG | `log_debug` | Blue | Debug information (requires DEBUG=true) |

**Evidence:** lines 45-100

### Log Destinations
1. **Standard Output:** Color-coded console output
2. **Log File:** Timestamped file output (if `--log-file` specified or not CI mode)
   - Default: `/tmp/bootstrap-<timestamp>.log`
   - Evidence: lines 372-381

### Log Format
**Console:** `[LEVEL] message`
**File:** `[timestamp] [LEVEL] message`

**Evidence:** lines 68-75

---

## CI Mode Behavior

When `--ci` flag is used:

1. **Non-interactive installation** (line 596)
   ```bash
   sh "$installer_script" -- install --no-confirm
   ```

2. **Sets verification profile to "ci"** (line 313)

3. **Skips shell RC modifications** (lines 1130-1132)
   ```bash
   if [ "$CI_MODE" = true ]; then
       log_info "CI mode: skipping shell rc modifications"
   ```

4. **No default log file** (line 373)

5. **Runs all-systems flake check** (lines 905-907)
   ```bash
   if [ "$CI_MODE" = true ]; then
       flake_check_args+=(--all-systems)
   fi
   ```

6. **Forces verification** (line 1142)
   ```bash
   if [ "$RUN_VERIFY" = true ] || [ "$CI_MODE" = true ]; then
       verify_manifest_components
   ```

---

## Verification Workflow

### Verification Profiles
| Profile | Use Case | Evidence |
|---------|----------|----------|
| minimal | Basic checks only | Line 17 |
| ci | CI environment checks | Lines 313, 1142 |
| default | Standard verification | Line 295 |
| full | Comprehensive checks | Line 17 |

### Verification Steps
1. **Validate ARIA_MANIFEST.yaml** (if exists)
   - Uses `scripts/validate-manifest.py`
   - Evidence: lines 971-981

2. **Generate verification script** (if generator exists)
   - Uses `scripts/generate-verification.py`
   - Outputs to `scripts/verify-components.sh`
   - Evidence: lines 984-988

3. **Run component verification**
   - Executes `verify-components.sh` with profile
   - Evidence: lines 991-998

4. **Fallback to basic verification** (if no manifest)
   - Checks core components: nix, direnv, git, gh
   - Checks devshell components: pixi, jq, curl
   - Evidence: lines 1009-1054

---

## Cleanup and Error Handling

### Cleanup Tasks
**Registration:** `register_cleanup` function (lines 259-261)
**Execution:** `run_cleanup` function (lines 263-269)

**Example:**
```bash
register_cleanup "rm -f '$installer_script'"
```

**Current usage:** Removes temporary installer script (line 577)

### Error Trap
**Function:** `cleanup_on_error` (lines 272-283)
**Trap:** `trap cleanup_on_error EXIT` (line 285)

**On error:**
1. Logs exit code
2. Shows last completed stage
3. Suggests resume command
4. Shows log file location
5. Runs cleanup tasks

**Example output:**
```
[ERROR] Bootstrap failed with exit code: 1
[ERROR] Last completed stage: install_nix
[INFO] To resume from this point, run: ./bootstrap.sh --resume
[INFO] See log file for details: /tmp/bootstrap-20260113-142530.log
```

**Trap disabled on success:** line 1156

---

## Platform-Specific Behavior

### Linux
- Multi-user Nix installation (requires sudo)
- Systemd service for nix-daemon
- Checks for /etc/os-release for distribution detection
- Evidence: lines 518-534

### macOS (Darwin)
- Multi-user Nix installation (requires sudo)
- launchd service for nix-daemon
- Distribution set to "macos"
- Evidence: lines 536-538

### Architecture Support
| Architecture | Supported | Normalized Name |
|--------------|-----------|-----------------|
| x86_64 | ✅ Yes | x86_64 |
| amd64 | ✅ Yes | x86_64 |
| aarch64 | ✅ Yes | aarch64 |
| arm64 | ✅ Yes | aarch64 |
| Others | ❌ No | Error |

**Evidence:** lines 503-515

---

## References

### Source Code
- **Main script:** `bootstrap.sh` (1160 lines)
- **Retry logic:** lines 108-147
- **State management:** lines 152-251
- **System detection:** lines 471-547
- **Nix installation:** lines 560-624
- **Tool installations:** lines 669-852
- **Verification:** lines 885-1054

### Related Files
- **Windows equivalent:** `bootstrap.ps1` (8 stages)
- **Verification manifest:** `ARIA_MANIFEST.yaml`
- **Verification scripts:** `scripts/verify-components.sh`, `scripts/validate-manifest.py`, `scripts/generate-verification.py`
- **Environment config:** `flake.nix`, `pixi.toml`

### Related Documentation
- [docs/modules/bootstrap.md](../modules/bootstrap.md) - Golden Path GP-1
- [docs/GETTING_STARTED.md](../GETTING_STARTED.md) - User guide
- [docs/TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - Common issues

### External Resources
- [Determinate Systems Nix Installer](https://github.com/DeterminateSystems/nix-installer)
- [Nix Flakes](https://nixos.wiki/wiki/Flakes)
- [direnv](https://direnv.net/)

---

**Contract Version:** 1.0
**Evidence-Based:** All line numbers verified from source code
**Phase 4 Deliverable:** 1/60 contracts complete
