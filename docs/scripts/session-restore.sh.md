# Script Contract: session-restore.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/session-restore.sh`

---

## Purpose

Restore session state after WSL reload or terminal crash. Restores working directory, loads stable environment, selectively restores environment variables, shows git status, and displays recent command history. Complement to `session-save.sh`.

---

## Invocation

```bash
source scripts/session-restore.sh
```

**Note:** Must be sourced (not executed) to affect current shell's directory and environment.

**Alias:** `ripple-restore` (if `stable-env.sh` sourced)

---

## Outputs

**Standard Output (successful restore):**
```
🔄 Restoring ripple-env session...
📍 Restored directory: /home/user/repos/ripple-env
🛡️ Loading stable environment...
✅ WSL-stable environment loaded!
🔄 Restoring environment variables...
📁 Git status:
## main...origin/main
📋 Recent command history:
  pixi install
  nix develop
  docker compose up -d
  git status
  ./scripts/session-save.sh
✅ Session recovery complete!
   Use 'direnv allow' if needed to reload environment
   Use 'scripts/session-save.sh' to save current state
```

**No session data:**
```
🔄 Restoring ripple-env session...
ℹ️ No session data found at /home/user/.ripple-sessions
   Run 'scripts/session-save.sh' to save current session
```

### Exit Codes
| Code | Meaning |
|------|---------|
| `0` | Success (always, via return) |

---

## Side Effects

**Changes current directory** to last saved directory (lines 22-31).

**Exports environment variables** from last saved session (lines 39-49).

**Sources stable-env.sh** if available (lines 33-37).

---

## Safety Classification

**🟢 SAFE** - Restores user session only, filtered environment variables.

---

## Idempotency

**✅ IDEMPOTENT** - Can be sourced repeatedly (last session wins).

---

## Restoration Steps

### 1. Check Session Directory (lines 11-15)

**Evidence:**
```bash
if [[ ! -d "$SESSION_DIR" ]]; then
    echo "ℹ️ No session data found at $SESSION_DIR"
    return 0
fi
```

**Early exit:** Uses `return 0` (not `exit`) since script is sourced.

### 2. Find Latest Session Files (lines 17-20)

**Evidence:**
```bash
LATEST_DIR=$(find "$SESSION_DIR" -name "last-directory.txt" -type f | head -1)
LATEST_ENV=$(find "$SESSION_DIR" -name "env-*.txt" -type f | sort -r | head -1)
LATEST_HISTORY=$(find "$SESSION_DIR" -name "history-*.txt" -type f | sort -r | head -1)
```

**Sort strategy:** `sort -r` (reverse) ensures newest timestamp first.

### 3. Restore Working Directory (lines 22-31)

**Evidence:**
```bash
if [[ -f "$LATEST_DIR" ]]; then
    LAST_DIR=$(cat "$LATEST_DIR")
    if [[ -d "$LAST_DIR" ]]; then
        cd "$LAST_DIR"
        echo "📍 Restored directory: $LAST_DIR"
    else
        echo "⚠️ Last directory no longer exists: $LAST_DIR"
    fi
fi
```

**Safety:** Checks if directory still exists before `cd`.

### 4. Load Stable Environment (lines 33-37)

**Evidence:**
```bash
if [[ -f "scripts/stable-env.sh" ]]; then
    echo "🛡️ Loading stable environment..."
    source scripts/stable-env.sh
fi
```

**Purpose:** Restores WSL stability settings, safe command wrappers, and aliases.

**Note:** Only works if restored to correct directory.

### 5. Restore Environment Variables (lines 39-49)

**Evidence:**
```bash
if [[ -f "$LATEST_ENV" ]]; then
    echo "🔄 Restoring environment variables..."
    grep -E "(RIPPLE|PIX|ROS)" "$LATEST_ENV" | while IFS= read -r line; do
        if [[ ! "$line" =~ PATH|LD_LIBRARY_PATH|DYLD_LIBRARY_PATH ]]; then
            export "$line" 2>/dev/null || true
        fi
    done
fi
```

**Security filtering:**
- **Whitelist:** Only RIPPLE, PIX, ROS variables
- **Blacklist:** Excludes PATH, LD_LIBRARY_PATH, DYLD_LIBRARY_PATH
- **Silent failure:** `|| true` prevents errors on invalid exports

**Rationale:** Prevents malicious PATH injection, avoids breaking system paths.

### 6. Show Git Status (lines 51-55)

**Evidence:**
```bash
if [[ -d .git ]] && command -v git >/dev/null 2>&1; then
    echo "📁 Git status:"
    git status --short --branch | head -5
fi
```

**Format:** Short format, branch info, limited to 5 lines.

### 7. Show Recent History (lines 57-61)

**Evidence:**
```bash
if [[ -f "$LATEST_HISTORY" ]]; then
    echo "📋 Recent command history:"
    tail -5 "$LATEST_HISTORY" | sed 's/^/  /' || true
fi
```

**Display:** Last 5 commands, indented for readability.

---

## Security Features

### 1. PATH Protection

**Evidence:** Lines 43-45

**Filters out:**
- `PATH` - System executable paths
- `LD_LIBRARY_PATH` - Linux library paths
- `DYLD_LIBRARY_PATH` - macOS library paths

**Reason:** Prevents attackers from injecting malicious paths into saved sessions.

### 2. Variable Whitelist

**Evidence:** Line 43

**Only restores:**
- `RIPPLE*` - Project variables
- `PIX*` - Pixi environment
- `ROS*` - ROS2 variables

**Excludes:** NIX variables (intentionally, as they're set by environment).

### 3. Silent Failure

**Evidence:** Line 46

```bash
export "$line" 2>/dev/null || true
```

**Behavior:** Invalid exports (malformed syntax, protected variables) fail silently without breaking restoration.

---

## Usage Patterns

### After WSL Reload

```bash
# 1. Open new terminal/WSL session
# 2. Navigate to repo (or let restore do it)
cd ~/repos/ripple-env

# 3. Source restore script
source scripts/session-restore.sh

# 4. Verify environment
pixi info
git status
```

### With Alias

```bash
# If stable-env.sh was sourced before crash
ripple-restore

# Shorthand for:
# source scripts/session-restore.sh
```

### Daily Workflow

```bash
# Morning: restore previous session
ripple-restore

# Work...

# Evening: save session before shutdown
ripple-save
```

---

## Limitations

**No file content restoration:** Does not restore uncommitted changes. Use `git stash` or manual backups.

**No process restoration:** Does not restart Docker containers, background jobs, or tmux sessions.

**No Nix environment activation:** User must run `nix develop` manually if needed.

**No Pixi environment activation:** User must run `pixi shell` or rely on direnv.

**Directory dependency:** Stable environment loading only works if `cd` to correct directory first.

---

## Differences from session-save.sh

| Feature | session-save.sh | session-restore.sh |
|---------|-----------------|-------------------|
| Direction | Save → disk | Disk → environment |
| Must be sourced | No | Yes (to change shell) |
| Creates files | Yes (timestamped) | No |
| Modifies shell | No | Yes (cd, export) |
| Security filtering | None (saves all) | Yes (filters PATH) |

---

## Integration

**Loaded by:**
- Manual sourcing
- `stable-env.sh` (creates `ripple-restore` alias, line 94)

**Complementary:**
- `session-save.sh` - Saves session state
- `stable-env.sh` - WSL stability environment

**Use case:** Recover after WSL2 crashes, terminal closes, or system reboots.

---

## References

### Source Code
- **Main script:** `scripts/session-restore.sh` (65 lines)
- **Session check:** lines 11-15
- **Find latest:** lines 17-20
- **Directory restore:** lines 22-31
- **Env loading:** lines 33-37
- **Variable restore:** lines 39-49
- **Git status:** lines 51-55
- **History display:** lines 57-61

### Related Files
- **Save script:** `scripts/session-save.sh`
- **Environment loader:** `scripts/stable-env.sh`
- **Session directory:** `$HOME/.ripple-sessions/`

### External Resources
- [Bash Source vs Execute](https://www.baeldung.com/linux/bash-source-vs-execute)
- [Bash Return Statement](https://www.gnu.org/software/bash/manual/html_node/Bourne-Shell-Builtins.html#index-return)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 27/60 contracts complete
