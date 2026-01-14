# Script Contract: cleanup-ruvector.ps1

**Status:** Phase 4 Complete
**Last Updated:** 2026-01-13
**Location:** `scripts/cleanup-ruvector.ps1`

---

## Purpose

RuVector process cleanup utility for Windows. Stops stuck npx/RuVector server processes by terminating port listeners (8080, 50051, 18080, 19080) and killing any process with "ruvector" in the command line. Designed for post-verification cleanup to unstick terminals.

---

## Invocation

```powershell
.\scripts\cleanup-ruvector.ps1 [-Ports <int[]>]
```

**Parameters:**
- `-Ports` - Array of port numbers to check (default: 8080, 50051, 18080, 19080)

**Examples:**
```powershell
.\scripts\cleanup-ruvector.ps1                      # Default ports
.\scripts\cleanup-ruvector.ps1 -Ports 8080,50051    # Custom ports
```

---

## Side Effects

### Process Termination (lines 28-51)
- **Port listeners (lines 28-38):** Kills processes listening on specified ports
- **RuVector processes (lines 40-51):** Kills any process with "ruvector" in command line

**Targets:**
- HTTP server on 8080, 18080, 19080
- gRPC server on 50051

---

## Safety Classification

**🟡 CAUTION** - Terminates processes (port listeners and ruvector-related).

---

## Idempotency

**✅ FULLY IDEMPOTENT** - Safe to run multiple times (no-op if nothing running).

---

## Key Features

### Port-Based Cleanup (lines 28-38)

```powershell
foreach ($port in $Ports) {
  try {
    $conn = Get-NetTCPConnection -State Listen -LocalPort $port -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($null -ne $conn) {
      Write-Output "[cleanup] stopping listener on port $port (PID $($conn.OwningProcess))"
      Stop-Process -Id $conn.OwningProcess -Force -ErrorAction SilentlyContinue
    }
  } catch {
    Write-Output "[cleanup] (warn) failed checking/stopping port ${port}: $($_.Exception.Message)"
  }
}
```

**Method:**
- Uses `Get-NetTCPConnection` to find listening sockets
- Extracts PID via `OwningProcess` property
- Terminates with `Stop-Process -Force`

### Command-Line Pattern Matching (lines 40-51)

```powershell
try {
  $procs = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue | Where-Object {
    $_.CommandLine -and ($_.CommandLine -match 'ruvector')
  }

  foreach ($p in $procs) {
    Write-Output "[cleanup] stopping ruvector-related process PID $($p.ProcessId)"
    Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
  }
} catch {
  Write-Output "[cleanup] (warn) failed enumerating processes: $($_.Exception.Message)"
}
```

**Safety features:**
- Only targets processes with "ruvector" in command line
- Does NOT kill all node.exe processes indiscriminately
- Best-effort termination (failures are non-fatal)

### Error Handling (lines 24, 35-37, 49-51)

```powershell
$ErrorActionPreference = 'Continue'
```

**Strategy:**
- Continues on errors (does not halt script)
- Logs warnings for failed operations
- Always exits with code 0 (success)

---

## Use Cases

1. **Post-Verification Cleanup**
   - After running `verify-ruvector.sh`, server may still be running
   - Use this script to clean up stuck processes

2. **Development Workflow**
   - RuVector server started with npx may not terminate properly
   - Script ensures clean state for next run

3. **CI/CD Cleanup**
   - Safe to run in pipelines to ensure no leftover processes

---

## References

- **Main script:** `scripts/cleanup-ruvector.ps1` (56 lines)
- **Port cleanup:** lines 28-38
- **Process cleanup:** lines 40-51
- **Related:** `scripts/verify-ruvector.sh` (RuVector verification)
- **Companion:** `scripts/ruvector.ps1` (RuVector launcher)

---

**Contract Version:** 1.0
**Phase 4 Deliverable:** 57/57 (100%)
