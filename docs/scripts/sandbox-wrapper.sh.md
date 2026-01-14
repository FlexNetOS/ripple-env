# Script Contract: sandbox-wrapper.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/sandbox-wrapper.sh`

---

## Purpose

Low-level Docker sandbox wrapper for secure code execution. Configures Docker containers with strict security: no network (default), read-only root filesystem, capability dropping (ALL), non-root user (nobody:nogroup), memory/CPU limits, and timeout enforcement. Supports Python, Bash, Node.js, Rust, and Go.

---

## Invocation

```bash
./sandbox-wrapper.sh <language> <code_file> [options]
```

**Languages:** python, bash, node, rust, go

**Options:**
- `--timeout` - Seconds (default: 60)
- `--memory` - Size (default: 256m)
- `--cpu` - Cores (default: 0.5)
- `--network` - none|bridge (default: none)
- `--env`, `--volume` - Environment/volumes
- `--readonly` - Mount code read-only
- `--user` - UID:GID (default: 65534:65534)

---

## Side Effects

### Docker Container (lines 203-260)
Ephemeral container (`--rm`) with resource limits and security constraints.

---

## Safety Classification

**🟡 CAUTION** - Executes untrusted code.

---

## Docker Security (lines 203-215)

```bash
docker run --rm \
  --network none \
  --memory 256m \
  --cpus 0.5 \
  --user 65534:65534 \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --tmpfs /tmp:rw,noexec,nosuid,size=64m
```

**Key constraints:**
- Read-only root FS
- No capabilities
- No privilege escalation
- Temp FS (64MB, noexec)

---

## Images (lines 171-193)
- Python: python:3.11-slim
- Bash: bash:5.2-alpine
- Node: node:20-alpine
- Rust: rust:1.75-slim
- Go: golang:1.21-alpine

---

## References

- **Main script:** `scripts/sandbox-wrapper.sh` (323 lines)
- **Docker command:** lines 196-262
- **Called by:** `sandbox-agent.sh`
- **Related:** P1-006 (container sandboxing)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 52/60 (86.7%)
