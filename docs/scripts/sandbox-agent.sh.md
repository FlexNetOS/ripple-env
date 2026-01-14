# Script Contract: sandbox-agent.sh

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/sandbox-agent.sh`

---

## Purpose

AI agent code sandbox executor with risk-based profiles (low/medium/high). Loads YAML configurations from `config/sandbox/profiles/`, enforces resource limits, and delegates execution to `sandbox-wrapper.sh`. Provides comprehensive logging, dry-run mode, and override capabilities for timeout/memory/CPU.

---

## Invocation

```bash
./sandbox-agent.sh --risk-level <low|medium|high> --language <lang> --code <file> [options]
```

**Required:**
- `--risk-level` - low|medium|high
- `--language` - python|bash|node|rust|go
- `--code` - Code file to execute

**Optional:**
- `--agent-id` - Custom agent identifier
- `--timeout`, `--memory`, `--cpu` - Override profile limits
- `--env`, `--volume` - Additional env vars/mounts
- `--dry-run` - Preview configuration
- `--help` - Show help

---

## Side Effects

### Log Files (line 241)
Creates `/var/log/sandbox/{risk}-risk_{agent-id}_{timestamp}.log`

---

## Safety Classification

**🟡 CAUTION** - Executes untrusted code in containers.

---

## Risk Profiles (lines 131-155)

### low-risk.yaml
- Memory: 512m+, CPU: 1.0+, Timeout: 300s, Network: bridge

### medium-risk.yaml
- Memory: 256m, CPU: 0.5, Timeout: 60s, Network: none

### high-risk.yaml
- Memory: 128m, CPU: 0.25, Timeout: 30s, Network: none, Max isolation

---

## References

- **Main script:** `scripts/sandbox-agent.sh` (404 lines)
- **Profiles:** `config/sandbox/profiles/{low,medium,high}-risk.yaml`
- **Wrapper:** `sandbox-wrapper.sh` (Docker execution)
- **Related:** P2-001 (sandbox-runtime integration)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 51/60 (85%)
