<#
  cleanup-ruvector.ps1

  Purpose:
    Unstick terminals / clean up any running RuVector-related processes that may
    have been started via npx/ruvector server during verification.

  What it does:
    1) Stops listeners on common RuVector ports (HTTP 8080, gRPC 50051, plus a
       couple of test ports we commonly use).
    2) Stops any process whose command line contains the string "ruvector".

  Notes:
    - Uses best-effort Stop-Process; failures are non-fatal.
    - Safe-ish: it does NOT kill all node.exe processes; it only targets those
      with "ruvector" in the command line and those bound to the chosen ports.
#>

[CmdletBinding()]
param(
  [int[]]$Ports = @(8080, 50051, 18080, 19080)
)

$ErrorActionPreference = 'Continue'

Write-Output "[cleanup] starting"

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

Write-Output "[cleanup] complete"

exit 0
