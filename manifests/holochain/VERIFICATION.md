# Holochain Integration Verification Guide

This guide provides step-by-step verification commands for the Holochain integration in ARIA/FlexStack.

## Prerequisites

Ensure you're in a Nix development shell with Holochain binaries available:

```bash
cd /home/user/ros2-humble-env
nix develop
```

## 1. Verify Binary Installation

### Check Holochain Conductor

```bash
holochain --version
```

**Expected Output:**
```
holochain 0.4.0
```

### Check Holochain CLI

```bash
hc --version
```

**Expected Output:**
```
hc 0.4.0
```

### Check Lair Keystore

```bash
lair-keystore --version
```

**Expected Output:**
```
lair-keystore 0.4.5
```

## 2. Verify Rust Dependencies

### Check Cargo.toml Dependencies

```bash
cd /home/user/ros2-humble-env/rust
grep -A 5 "Holochain Coordination" Cargo.toml
```

**Expected Output:**
```toml
# Holochain Coordination (BUILDKIT_STARTER_SPEC.md Section 9.7)
# P0: Holochain Development Kit - Required for building DNAs and zomes
# P1-002: Holochain crate integration
hdk = "0.4.0"
hdi = "0.5.0"
holochain = "0.4.0"
holochain_types = "0.4.0"
holochain_zome_types = "0.4.0"
```

### Verify Crate Availability (Optional - requires network)

```bash
cd /home/user/ros2-humble-env/rust
cargo tree | grep holochain | head -10
```

**Note:** This will attempt to fetch dependencies and may take time on first run.

## 3. Verify DNA Scaffolding

### Check DNA Directory Structure

```bash
ls -la /home/user/ros2-humble-env/manifests/holochain/dnas/
```

**Expected Output:**
```
total XX
drwxr-xr-x X root root XXXX Jan  9 XX:XX .
drwxr-xr-x X root root XXXX Jan  9 XX:XX ..
drwxr-xr-x X root root XXXX Jan  9 XX:XX agent_registry
drwxr-xr-x X root root XXXX Jan  9 XX:XX artifact_index
-rw-r--r-- X root root XXXX Jan  9 XX:XX index.json
drwxr-xr-x X root root XXXX Jan  9 XX:XX memory_shards
drwxr-xr-x X root root XXXX Jan  9 XX:XX policy_store
drwxr-xr-x X root root XXXX Jan  9 XX:XX resource_mesh
```

### Verify DNA Manifests

```bash
# Check agent_registry DNA manifest
cat /home/user/ros2-humble-env/manifests/holochain/dnas/agent_registry/dna.yaml | head -10

# Check resource_mesh DNA manifest
cat /home/user/ros2-humble-env/manifests/holochain/dnas/resource_mesh/dna.yaml | head -10

# Check policy_store DNA manifest
cat /home/user/ros2-humble-env/manifests/holochain/dnas/policy_store/dna.yaml | head -10

# Check artifact_index DNA manifest
cat /home/user/ros2-humble-env/manifests/holochain/dnas/artifact_index/dna.yaml | head -10

# Check memory_shards DNA manifest
cat /home/user/ros2-humble-env/manifests/holochain/dnas/memory_shards/dna.yaml | head -10
```

**Expected:** Each command should display a valid YAML manifest starting with:
```yaml
---
# [DNA Name] DNA
# Purpose: [Description]
# Status: Initial scaffolding (P0-006)

manifest_version: "1"
name: [dna_name]
...
```

### Verify DNA Index

```bash
cat /home/user/ros2-humble-env/manifests/holochain/dnas/index.json
```

**Expected:** All DNAs should have status: "scaffolding" and include manifest paths.

## 4. Test DNA Initialization

### Test agent_registry DNA

```bash
cd /home/user/ros2-humble-env/manifests/holochain/dnas/agent_registry
hc dna init --path workdir
```

**Expected:** Should create a new DNA project structure in the workdir directory.

### Test resource_mesh DNA

```bash
cd /home/user/ros2-humble-env/manifests/holochain/dnas/resource_mesh
hc dna init --path workdir
```

### Test policy_store DNA

```bash
cd /home/user/ros2-humble-env/manifests/holochain/dnas/policy_store
hc dna init --path workdir
```

### Test artifact_index DNA

```bash
cd /home/user/ros2-humble-env/manifests/holochain/dnas/artifact_index
hc dna init --path workdir
```

### Test memory_shards DNA

```bash
cd /home/user/ros2-humble-env/manifests/holochain/dnas/memory_shards
hc dna init --path workdir
```

## 5. Test Sandbox Generation

### Create Test Sandbox

```bash
cd /tmp
hc sandbox generate ./test-holochain-sandbox
```

**Expected Output:**
```
Generating Holochain sandbox at ./test-holochain-sandbox
✓ Created sandbox directory
✓ Created conductor config
✓ Initialized keystore
```

### Run Test Sandbox (Optional)

```bash
cd /tmp/test-holochain-sandbox
hc sandbox run
```

**Expected:** Conductor should start and show WebSocket ports for admin and app interfaces.

**Note:** Press `Ctrl+C` to stop the sandbox.

## 6. Verify Conductor Configuration

### Check Conductor Config Syntax

```bash
cat /home/user/ros2-humble-env/manifests/holochain/conductor.yaml
```

**Expected:** Should display valid YAML configuration with:
- environment_path
- keystore configuration
- admin_interfaces
- app_interfaces
- network configuration

### Validate Config (Advanced - requires running conductor)

```bash
holochain --config-path /home/user/ros2-humble-env/manifests/holochain/conductor.yaml --check
```

**Note:** This may fail if environment variables like `$RIPPLE_DATA` are not set, which is expected at this stage.

## 7. Verify flake.nix Integration

### Check Holochain Packages Definition

```bash
grep -A 5 "holochainPackages" /home/user/ros2-humble-env/flake.nix
```

**Expected Output:**
```nix
holochainPackages = with pkgs; [
  holochain       # Holochain conductor (agent-centric P2P)
  hc              # Holochain dev CLI (scaffold/package/run)
  lair-keystore   # Secure keystore for Holochain agent keys
];
```

### Check Holochain Source

```bash
grep -A 5 "holochainSrc" /home/user/ros2-humble-env/flake.nix | head -10
```

**Expected:** Should show fetchFromGitHub call to spartan-holochain-counsel/nix-overlay.

## 8. Complete Verification Checklist

Run this comprehensive check:

```bash
cd /home/user/ros2-humble-env

echo "=== Holochain Integration Verification ==="
echo ""

echo "1. Binary Versions:"
holochain --version 2>/dev/null && echo "  ✓ holochain installed" || echo "  ✗ holochain missing"
hc --version 2>/dev/null && echo "  ✓ hc installed" || echo "  ✗ hc missing"
lair-keystore --version 2>/dev/null && echo "  ✓ lair-keystore installed" || echo "  ✗ lair-keystore missing"
echo ""

echo "2. Rust Dependencies:"
grep -q "hdk = \"0.4.0\"" rust/Cargo.toml && echo "  ✓ hdk in Cargo.toml" || echo "  ✗ hdk missing"
grep -q "hdi = \"0.5.0\"" rust/Cargo.toml && echo "  ✓ hdi in Cargo.toml" || echo "  ✗ hdi missing"
grep -q "holochain = \"0.4.0\"" rust/Cargo.toml && echo "  ✓ holochain in Cargo.toml" || echo "  ✗ holochain missing"
echo ""

echo "3. DNA Manifests:"
test -f manifests/holochain/dnas/agent_registry/dna.yaml && echo "  ✓ agent_registry manifest" || echo "  ✗ agent_registry missing"
test -f manifests/holochain/dnas/resource_mesh/dna.yaml && echo "  ✓ resource_mesh manifest" || echo "  ✗ resource_mesh missing"
test -f manifests/holochain/dnas/policy_store/dna.yaml && echo "  ✓ policy_store manifest" || echo "  ✗ policy_store missing"
test -f manifests/holochain/dnas/artifact_index/dna.yaml && echo "  ✓ artifact_index manifest" || echo "  ✗ artifact_index missing"
test -f manifests/holochain/dnas/memory_shards/dna.yaml && echo "  ✓ memory_shards manifest" || echo "  ✗ memory_shards missing"
echo ""

echo "4. DNA Documentation:"
test -f manifests/holochain/dnas/agent_registry/README.md && echo "  ✓ agent_registry README" || echo "  ✗ agent_registry README missing"
test -f manifests/holochain/dnas/resource_mesh/README.md && echo "  ✓ resource_mesh README" || echo "  ✗ resource_mesh README missing"
test -f manifests/holochain/dnas/policy_store/README.md && echo "  ✓ policy_store README" || echo "  ✗ policy_store README missing"
test -f manifests/holochain/dnas/artifact_index/README.md && echo "  ✓ artifact_index README" || echo "  ✗ artifact_index README missing"
test -f manifests/holochain/dnas/memory_shards/README.md && echo "  ✓ memory_shards README" || echo "  ✗ memory_shards README missing"
echo ""

echo "5. Configuration Files:"
test -f manifests/holochain/conductor.yaml && echo "  ✓ conductor.yaml" || echo "  ✗ conductor.yaml missing"
test -f manifests/holochain/versions.json && echo "  ✓ versions.json" || echo "  ✗ versions.json missing"
test -f manifests/holochain/dnas/index.json && echo "  ✓ dnas/index.json" || echo "  ✗ dnas/index.json missing"
test -f manifests/holochain/HOLOCHAIN.md && echo "  ✓ HOLOCHAIN.md" || echo "  ✗ HOLOCHAIN.md missing"
echo ""

echo "6. DNA Status Check:"
grep '"status":' manifests/holochain/dnas/index.json | head -5
echo ""

echo "=== Verification Complete ==="
```

## 9. Troubleshooting

### Binary Not Found

If binaries are not found, ensure you're in the Nix development shell:

```bash
nix develop
```

Or rebuild the environment:

```bash
nix develop --rebuild
```

### DNA Validation Errors

If `hc dna init` fails, ensure:
- You're in the correct DNA directory
- The workdir directory exists
- You have write permissions

### Conductor Config Errors

If conductor.yaml validation fails:
- Check YAML syntax
- Set required environment variables:
  ```bash
  export RIPPLE_DATA=~/.local/share/ripple
  export HOLOCHAIN_BOOTSTRAP_URL=https://bootstrap.holo.host
  export HOLOCHAIN_SIGNAL_URL=wss://signal.holo.host
  ```

## 10. Next Steps

After successful verification:

1. **Implement Integrity Zomes** - Define entry types and validation rules
2. **Implement Coordinator Zomes** - Add CRUD operations and business logic
3. **Build WASM** - Compile zomes to WebAssembly
4. **Package DNAs** - Use `hc dna pack` to create DNA bundles
5. **Test Conductor** - Run conductor with real DNAs
6. **Service Integration** - Connect to AGiXT, ROS2, and Rust services

See `/home/user/ros2-humble-env/manifests/holochain/HOLOCHAIN.md` for detailed implementation roadmap.

## Support Resources

- Holochain Documentation: https://developer.holochain.org
- Holochain Forum: https://forum.holochain.org
- Holochain Discord: https://discord.gg/holochain
- ARIA Project Documentation: See BUILDKIT_STARTER_SPEC.md Section 9.7
