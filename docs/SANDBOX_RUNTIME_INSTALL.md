# Sandbox Runtime Installation Guide

**Status**: Defined  
**Date**: 2026-01-09  
**Priority**: P0 (Blocking)  
**Repository**: https://github.com/anthropic-experimental/sandbox-runtime

## Overview

The Anthropic sandbox-runtime provides process-level isolation for executing untrusted code safely. This is a critical component for the Agent Runtime domain.

## Installation Method: Binary Download + Nix Wrapper

### Rationale

1. **Not in nixpkgs**: The sandbox-runtime is not available in the official Nix packages repository
2. **Experimental Status**: The repository is marked as "experimental" and may not have stable releases
3. **Binary Distribution**: Anthropic may provide pre-built binaries for direct download
4. **Nix Wrapper**: We can create a Nix derivation to fetch and wrap the binary

## Installation Options

### Option 1: Build from Source (Recommended)

```bash
# Clone the repository
git clone https://github.com/anthropic-experimental/sandbox-runtime.git
cd sandbox-runtime

# Build using Cargo (Rust toolchain from Nix)
cargo build --release

# Install to local bin
cp target/release/sandbox-runtime ~/.local/bin/

# Or install system-wide via Nix
sudo install -m 755 target/release/sandbox-runtime /usr/local/bin/
```

### Option 2: Nix Derivation (Preferred for reproducibility)

Add to `flake.nix`:

```nix
{
  # In fullExtras section
  sandbox-runtime = pkgs.rustPlatform.buildRustPackage rec {
    pname = "sandbox-runtime";
    version = "0.1.0";  # Update with actual version
    
    src = pkgs.fetchFromGitHub {
      owner = "anthropic-experimental";
      repo = "sandbox-runtime";
      rev = "main";  # Or specific commit/tag
      sha256 = "sha256-AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=";  # Update after first fetch
    };
    
    cargoSha256 = "sha256-BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB=";  # Update after first build
    
    nativeBuildInputs = with pkgs; [ pkg-config ];
    buildInputs = with pkgs; [ openssl ];
    
    meta = with pkgs.lib; {
      description = "Process sandbox for executing untrusted code";
      homepage = "https://github.com/anthropic-experimental/sandbox-runtime";
      license = licenses.mit;  # Verify actual license
      platforms = platforms.unix;
    };
  };
}
```

### Option 3: Docker Container (Alternative)

```yaml
# docker-compose.sandbox.yml
services:
  sandbox-runtime:
    build:
      context: https://github.com/anthropic-experimental/sandbox-runtime.git
      dockerfile: Dockerfile
    container_name: sandbox-runtime
    privileged: true  # Required for process isolation
    volumes:
      - /var/run/docker.sock:/var/run/docker.sock:ro
      - ./data/sandbox:/sandbox
    environment:
      - SANDBOX_TIMEOUT=30s
      - SANDBOX_MEMORY_LIMIT=512M
      - SANDBOX_CPU_LIMIT=1.0
    networks:
      - agentic-network
    restart: unless-stopped
```

## Configuration

### Environment Variables

```bash
# Sandbox configuration
export SANDBOX_RUNTIME_PATH=/usr/local/bin/sandbox-runtime
export SANDBOX_WORK_DIR=/tmp/sandbox
export SANDBOX_TIMEOUT=30
export SANDBOX_MEMORY_LIMIT=512M
export SANDBOX_CPU_SHARES=1024

# Security settings
export SANDBOX_ALLOW_NETWORK=false
export SANDBOX_ALLOW_FS_WRITE=false
export SANDBOX_ALLOW_EXEC=true
```

### Usage Example

```bash
# Execute Python code in sandbox
sandbox-runtime exec python3 << 'EOF'
print("Hello from sandbox!")
import os
print(f"UID: {os.getuid()}")
EOF

# Execute with resource limits
sandbox-runtime exec \
  --timeout 10s \
  --memory 256M \
  --cpu 0.5 \
  python3 -c "print('Limited execution')"

# Execute with file system isolation
sandbox-runtime exec \
  --mount /tmp/input:/input:ro \
  --mount /tmp/output:/output:rw \
  python3 /input/script.py
```

## Integration with Agent Runtime

### Python Integration

```python
import subprocess
import json

def execute_in_sandbox(code: str, timeout: int = 30) -> dict:
    """Execute Python code in sandbox and return result."""
    cmd = [
        "sandbox-runtime", "exec",
        "--timeout", f"{timeout}s",
        "--memory", "512M",
        "--json-output",
        "python3", "-c", code
    ]
    
    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        timeout=timeout + 5
    )
    
    return {
        "stdout": result.stdout,
        "stderr": result.stderr,
        "returncode": result.returncode,
        "success": result.returncode == 0
    }

# Usage
result = execute_in_sandbox("""
import sys
print(f"Python version: {sys.version}")
print("Sandbox execution successful!")
""")

print(result)
```

### Rust Integration

```rust
use std::process::Command;
use serde_json::Value;

pub fn execute_in_sandbox(code: &str, timeout_secs: u64) -> Result<Value, Box<dyn std::error::Error>> {
    let output = Command::new("sandbox-runtime")
        .arg("exec")
        .arg("--timeout")
        .arg(format!("{}s", timeout_secs))
        .arg("--memory")
        .arg("512M")
        .arg("--json-output")
        .arg("python3")
        .arg("-c")
        .arg(code)
        .output()?;
    
    let result = serde_json::json!({
        "stdout": String::from_utf8_lossy(&output.stdout),
        "stderr": String::from_utf8_lossy(&output.stderr),
        "returncode": output.status.code(),
        "success": output.status.success()
    });
    
    Ok(result)
}
```

## Security Considerations

### Isolation Levels

1. **Process Isolation**: Separate process with restricted capabilities
2. **Filesystem Isolation**: Read-only root, temporary writable directories
3. **Network Isolation**: Optional network access control
4. **Resource Limits**: CPU, memory, and time limits

### Best Practices

1. **Always set timeouts**: Prevent infinite loops
2. **Limit memory**: Prevent memory exhaustion
3. **Restrict filesystem access**: Use read-only mounts where possible
4. **Disable network by default**: Only enable when necessary
5. **Log all executions**: Audit trail for security

### Threat Model

**Mitigated**:
- Code execution exploits
- Resource exhaustion attacks
- Filesystem tampering
- Network-based attacks (when disabled)

**Not Mitigated**:
- Side-channel attacks (timing, cache)
- Kernel vulnerabilities
- Hardware exploits

## Verification Commands

```bash
# Check if sandbox-runtime is installed
which sandbox-runtime

# Verify version
sandbox-runtime --version

# Test basic execution
sandbox-runtime exec echo "Hello, sandbox!"

# Test Python execution
sandbox-runtime exec python3 -c "print('Python works!')"

# Test resource limits
sandbox-runtime exec --timeout 1s --memory 128M python3 -c "import time; time.sleep(10)"

# Test filesystem isolation
sandbox-runtime exec ls /

# Test network isolation
sandbox-runtime exec --no-network curl https://example.com
```

## CI/CD Integration

### GitHub Actions

```yaml
name: Sandbox Runtime Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install Rust
        uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
      
      - name: Build sandbox-runtime
        run: |
          git clone https://github.com/anthropic-experimental/sandbox-runtime.git
          cd sandbox-runtime
          cargo build --release
          sudo install -m 755 target/release/sandbox-runtime /usr/local/bin/
      
      - name: Test sandbox-runtime
        run: |
          sandbox-runtime --version
          sandbox-runtime exec echo "CI test passed"
```

## Troubleshooting

### Common Issues

1. **Permission Denied**
   - Solution: Ensure sandbox-runtime has execute permissions: `chmod +x /path/to/sandbox-runtime`

2. **Timeout Errors**
   - Solution: Increase timeout or optimize code execution

3. **Memory Limit Exceeded**
   - Solution: Increase memory limit or optimize code memory usage

4. **Network Access Denied**
   - Solution: Enable network access with `--allow-network` flag

### Debug Mode

```bash
# Enable verbose logging
SANDBOX_LOG_LEVEL=debug sandbox-runtime exec python3 script.py

# Trace system calls
strace -f sandbox-runtime exec python3 script.py

# Monitor resource usage
time sandbox-runtime exec --memory 256M python3 script.py
```

## Alternative Sandbox Solutions

If sandbox-runtime is not suitable, consider:

1. **gVisor** - Google's application kernel for containers
2. **Firecracker** - AWS's microVM technology (already in spec)
3. **Kata Containers** - Lightweight VMs (already in spec)
4. **Bubblewrap** - Unprivileged sandboxing tool
5. **systemd-nspawn** - Lightweight container manager

## Status

- [x] Installation method defined
- [x] Configuration documented
- [x] Integration examples provided
- [x] Security considerations outlined
- [ ] Actual installation completed
- [ ] Verification tests passed
- [ ] Integration with agent runtime tested

## Next Steps

1. Clone sandbox-runtime repository
2. Build from source using Cargo
3. Install to system path
4. Run verification tests
5. Integrate with agent runtime
6. Document performance benchmarks

## References

- [sandbox-runtime GitHub](https://github.com/anthropic-experimental/sandbox-runtime)
- [gVisor Documentation](https://gvisor.dev/)
- [Firecracker Documentation](https://firecracker-microvm.github.io/)
- [Kata Containers](https://katacontainers.io/)
