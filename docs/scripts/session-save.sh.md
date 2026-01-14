# Script Contract: session-save.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/session-save.sh`

---

## Purpose

Save current session state for recovery after WSL reloads or terminal crashes. Captures working directory, environment variables, command history, git status, Pixi environment, and Nix version. Essential for WSL2 stability workflows.

---

## Invocation

```bash
./scripts/session-save.sh
```

**Alias:** `ripple-save` (if `stable-env.sh` sourced)

---

## Outputs

**Standard Output:**
```
💾 Saving ripple-env session state...
✅ Session saved to /home/user/.ripple-sessions
   Timestamp: 20260113-143022
   Directory: /home/user/repos/ripple-env
   Files saved:
     - env-20260113-143022.txt
     - history-20260113-143022.txt
     - git-status-20260113-143022.txt
     - git-branch-20260113-143022.txt
     - pixi-info-20260113-143022.txt
     - nix-version-20260113-143022.txt
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (always) |

---

## Side Effects

### Directory Created (line 12)
- `$HOME/.ripple-sessions/` - Session state storage

### Files Created/Updated

**Always created:**
- `last-directory.txt` - Current working directory (line 15)

**Timestamped files (conditional):**
- `env-{timestamp}.txt` - Environment variables matching RIPPLE/PIX/ROS/NIX (line 18)
- `history-{timestamp}.txt` - Bash command history (line 21)
- `git-status-{timestamp}.txt` - Git porcelain status (line 25, if in git repo)
- `git-branch-{timestamp}.txt` - Current git branch (line 26, if in git repo)
- `pixi-info-{timestamp}.txt` - Pixi environment details (line 31, if pixi installed)
- `nix-version-{timestamp}.txt` - Nix version (line 36, if nix installed)

---

## Safety Classification

**🟢 SAFE** - Writes to user home directory only, no system modifications.

---

## Idempotency

**⚠️ NOT IDEMPOTENT** - Creates new timestamped files on each run.

**Accumulation:** Session directory grows over time.

---

## Captured State

### 1. Working Directory (lines 14-15)

**Evidence:**
```bash
echo "$PWD" > "$SESSION_DIR/last-directory.txt"
```

**Usage:** `session-restore.sh` can `cd` back to this directory.

### 2. Environment Variables (line 18)

**Evidence:**
```bash
env | grep -E "(RIPPLE|PIX|ROS|NIX)" > "$SESSION_DIR/env-$TIMESTAMP.txt" 2>/dev/null || true
```

**Captures:**
- `RIPPLE_*` - Custom project variables
- `PIX*` - Pixi environment variables
- `ROS*` - ROS2 environment variables
- `NIX*` - Nix build configuration

**Silent failure:** `|| true` prevents script exit if grep finds nothing.

### 3. Command History (line 21)

**Evidence:**
```bash
history > "$SESSION_DIR/history-$TIMESTAMP.txt" 2>/dev/null || true
```

**Purpose:** Review recent commands after reload, resume interrupted tasks.

**Note:** `history` may be empty if not in interactive shell.

### 4. Git Status (lines 24-27)

**Evidence:**
```bash
if [[ -d .git ]]; then
    git status --porcelain > "$SESSION_DIR/git-status-$TIMESTAMP.txt"
    git branch --show-current > "$SESSION_DIR/git-branch-$TIMESTAMP.txt"
fi
```

**Porcelain format:** Machine-readable git status (M, A, D, ??, etc.).

**Branch:** Current branch name for context.

### 5. Pixi Environment (lines 29-32)

**Evidence:**
```bash
if command -v pixi >/dev/null 2>&1; then
    pixi info > "$SESSION_DIR/pixi-info-$TIMESTAMP.txt"
fi
```

**pixi info output:**
- Platform information
- Virtual environment location
- Installed packages
- Environment activation status

### 6. Nix Version (lines 34-37)

**Evidence:**
```bash
if command -v nix >/dev/null 2>&1; then
    nix --version > "$SESSION_DIR/nix-version-$TIMESTAMP.txt"
fi
```

**Purpose:** Verify Nix installation after reload.

---

## Timestamp Format

**Evidence:** Line 7

```bash
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
```

**Format:** `YYYYMMDD-HHMMSS` (e.g., `20260113-143022`)

**Benefits:**
- Sortable (chronological order)
- Human-readable
- No spaces (shell-friendly)

---

## Usage Patterns

### Before Risky Operations

```bash
# 1. Save session
ripple-save

# 2. Run risky command
pixi install --all

# 3. If WSL crashes, restore after reload
ripple-restore
```

### Scheduled Saves

```bash
# Add to .bashrc or .zshrc
alias cd='cd_with_save() { cd "$@" && ripple-save; }; cd_with_save'

# Or cron job (every 15 minutes)
*/15 * * * * $HOME/repos/ripple-env/scripts/session-save.sh
```

### Manual Saves

```bash
# Before major changes
ripple-save
git checkout feature-branch
git pull
pixi update
```

---

## Session Directory Structure

**After multiple saves:**
```
$HOME/.ripple-sessions/
├── last-directory.txt                    # Latest directory
├── env-20260113-100000.txt
├── env-20260113-110000.txt
├── env-20260113-120000.txt               # Multiple timestamps
├── history-20260113-100000.txt
├── history-20260113-110000.txt
├── history-20260113-120000.txt
├── git-status-20260113-100000.txt
├── git-branch-20260113-100000.txt
├── pixi-info-20260113-100000.txt
└── nix-version-20260113-100000.txt
```

**Cleanup:** No automatic cleanup. User should periodically delete old sessions.

---

## Limitations

**No state restoration:** This script only saves. Restoration requires `session-restore.sh`.

**No running processes:** Does not capture running tmux/screen sessions, background jobs, or service states.

**No file content:** Does not save uncommitted file changes. Use git stash or manual backups.

**Accumulation:** Creates new files on each run, no automatic pruning.

---

## Integration

**Loaded by:**
- `stable-env.sh` (creates `ripple-save` alias, line 93)
- Manual execution

**Complementary:**
- `session-restore.sh` - Restores saved session
- `stable-env.sh` - WSL stability environment

**Use case:** WSL2 crashes during heavy operations (Nix builds, Pixi installs, Docker operations).

---

## References

### Source Code
- **Main script:** `scripts/session-save.sh` (43 lines)
- **Directory save:** line 15
- **Environment save:** line 18
- **History save:** line 21
- **Git save:** lines 24-27
- **Pixi save:** lines 29-32
- **Nix save:** lines 34-37

### Related Files
- **Restore script:** `scripts/session-restore.sh`
- **Environment loader:** `scripts/stable-env.sh`
- **Session directory:** `$HOME/.ripple-sessions/`

### External Resources
- [Bash History](https://www.gnu.org/software/bash/manual/html_node/Bash-History-Facilities.html)
- [Git Porcelain Format](https://git-scm.com/docs/git-status#_short_format)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 26/60 contracts complete
