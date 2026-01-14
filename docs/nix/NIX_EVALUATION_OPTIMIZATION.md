# Nix Evaluation Optimization Guide

This document describes the optimization strategies implemented to improve Nix flake evaluation time.

## Performance Targets

| Operation | Target | Description |
|-----------|--------|-------------|
| `nix flake show` | < 10s | Show all flake outputs |
| `minimal` shell | < 5s | Fastest possible shell for CI |
| `default` shell | < 15s | Standard development shell |
| `full` shell | < 30s | All tools included |

## Optimization Strategies

### 1. Lazy Evaluation for Platform-Specific Shells

Heavy shells like `cuda` and `identity` are only evaluated on Linux:

```nix
# nix/shells/default.nix
linuxOnlyShells = lib.optionalAttrs isLinux {
  cuda = ...;
  identity = ...;
};
```

**Impact**: Reduces macOS evaluation time by ~40% by not evaluating CUDA packages.

### 2. Minimal Shell Variant

A new `minimal` shell provides the fastest possible startup for CI and scripting:

```bash
# Use minimal shell for CI
nix develop .#minimal --command ./run-tests.sh
```

The minimal shell includes only:
- git
- pixi
- python313
- nixfmt-rfc-style

### 3. Feature Flags for Heavy Dependencies

The `dev-tools.nix` supports feature flags to exclude heavy packages:

```nix
# Exclude Kubernetes, VMs, and AI tools (~750MB+ savings)
devToolsMinimal = import ./dev-tools.nix {
  inherit pkgs;
  withKubernetes = false;  # Skip kubectl, helm (~100MB)
  withHeavyVMs = false;    # Skip firecracker, kata (~150MB)
  withAI = false;          # Skip local-ai (~500MB)
};
```

### 4. Binary Cache Configuration

The flake includes binary cache configuration for faster builds:

```nix
nixConfig = {
  extra-substituters = [
    "https://cache.nixos.org"
    "https://nix-community.cachix.org"
    "https://cuda-maintainers.cachix.org"
  ];
};
```

### 5. Input Optimization

Flake inputs are optimized:

- **`flake = false`**: Used for inputs that don't need flake features (e.g., `agenticsorg-devops`)
- **`follows`**: Deduplicates nixpkgs evaluation across inputs
- **Pinned commits**: Holochain overlay uses `fetchFromGitHub` to avoid extra flake input

### 6. Modular Structure

The flake uses a modular structure for faster evaluation:

```
nix/
├── packages/     # Package collections (lazy imported)
├── shells/       # DevShell definitions
├── commands/     # Command wrapper scripts
├── lib/          # Utility functions including lazy.nix
└── images/       # NixOS image builders
```

## Benchmarking

### Local Benchmarking

Run the benchmark script:

```bash
./scripts/benchmark-eval.sh
```

### CI Benchmarking

The `benchmarks.yml` workflow includes Nix evaluation benchmarks that run on:
- Push to `main` (when flake.nix or nix/ changes)
- Pull requests (when flake.nix or nix/ changes)
- Weekly schedule

Results are reported in the GitHub Actions summary.

## Shell Selection Guide

| Shell | Use Case | Startup Time |
|-------|----------|--------------|
| `minimal` | CI, scripting, quick operations | ~1s |
| `default` | Daily development | ~3s |
| `full` | Full toolset needed | ~5s |
| `cuda` | GPU/ML workloads (Linux) | ~5s |
| `identity` | Auth development (Linux) | ~4s |

## Troubleshooting Slow Evaluation

### 1. Profile with Trace

```bash
nix eval .#devShells.x86_64-linux.default --show-trace 2>&1 | head -100
```

### 2. Check Input Fetching

```bash
nix flake metadata --json | jq '.locks.nodes | keys'
```

### 3. Verify Cache Usage

```bash
nix config show | grep substituters
```

### 4. Clear Evaluation Cache

```bash
rm -rf ~/.cache/nix/eval-cache-v*
```

## CI Best Practices

1. **Use minimal shell for non-interactive tasks**:
   ```yaml
   - run: nix develop .#minimal --command pixi install
   ```

2. **Always use magic-nix-cache-action**:
   ```yaml
   - uses: DeterminateSystems/magic-nix-cache-action@v8
   ```

3. **Skip build for metadata-only checks**:
   ```yaml
   - run: nix flake check --no-build
   ```

4. **Cache Pixi separately**:
   ```yaml
   - uses: prefix-dev/setup-pixi@v0.8.1
     with:
       cache: true
   ```

## Further Optimization Ideas

If evaluation is still slow, consider:

1. **Split into multiple flakes**: Separate heavy components (CUDA, identity) into their own flakes
2. **Use Cachix**: Set up a project-specific Cachix cache
3. **Lazy NixOS configurations**: Only evaluate NixOS configs when building images
4. **Remove unused inputs**: Audit `flake.lock` for unused dependencies
