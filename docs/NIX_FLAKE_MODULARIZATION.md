# Nix Flake Modularization Plan

This document outlines the strategy for modularizing the monolithic `flake.nix` (155KB, ~3000 lines) into a maintainable, well-organized structure.

## Architecture Clarification

> **Important**: This is a **single flake that loads other flakes** (via inputs), NOT multiple separate flakes or distros. The modularization approach keeps everything in ONE flake while splitting code into multiple files for maintainability.

This pattern follows the [Holo-Host](https://github.com/Holo-Host/holo-host) architecture where:
- One unified flake defines multiple services/configurations
- Uses `flake-parts` for modular composition
- Imports other flakes as inputs (home-manager, nixpkgs, etc.)
- Exports multiple `nixosConfigurations` for different deployment targets

## Reference Projects

| Project | Pattern | URL |
|---------|---------|-----|
| **Holo-Host** | Multi-service monorepo with NixOS containers | [github.com/Holo-Host/holo-host](https://github.com/Holo-Host/holo-host) |
| **kickstart.nix** | flake-parts template aggregating modules | [github.com/ALT-F4-LLC/kickstart.nix](https://github.com/ALT-F4-LLC/kickstart.nix) |
| **Holochain Core** | flake-parts auto-importing modules | [github.com/holochain/holochain](https://github.com/holochain/holochain) |
| **NixOS-WSL** | WSL2 image generation | [github.com/nix-community/NixOS-WSL](https://github.com/nix-community/NixOS-WSL) |
| **kenlasko/nixos-wsl** | Personal config example | [github.com/kenlasko/nixos-wsl](https://github.com/kenlasko/nixos-wsl) |

## Current State

The current `flake.nix` contains multiple concerns in a single file:

| Concern | Approximate Lines | Description |
|---------|-------------------|-------------|
| Package definitions | ~1000 | ROS2, dev tools, AI/ML, infrastructure |
| DevShell definitions | ~800 | default, full, cuda, identity shells |
| Command wrappers | ~600 | cb, ct, localai, agixt, kata, etc. |
| NixOS/Darwin modules | ~400 | System-level configuration |
| Overlays | ~200 | Package overrides and customizations |

## Why Modularize?

### Problems with Monolithic Approach

1. **Maintenance burden**: 3000+ lines is difficult to navigate and review
2. **Single Responsibility Violation**: One file handles packages, shells, modules, and commands
3. **Code Review**: GitHub reviews on large files are unwieldy
4. **Debugging**: Hard to isolate failures to specific functionality
5. **Parallel Development**: Team members conflict when editing the same file
6. **Evaluation Time**: Large files take longer for Nix to evaluate

### Benefits of Modularization

1. **Maintainability**: Smaller files (~300-600 lines each) are easier to understand
2. **Testability**: Can test individual modules independently
3. **Reusability**: Modules can be imported into other projects
4. **Documentation**: Each module can have focused documentation
5. **Team Scaling**: Different maintainers for different modules

## Proposed Structure

```
ros2-humble-env/
├── flake.nix                      # Entry point (~150 lines)
├── flake.lock
├── nix/
│   ├── packages/
│   │   ├── default.nix            # Package collection aggregator
│   │   ├── ros2-packages.nix      # ROS2-specific packages
│   │   ├── dev-tools.nix          # Git, editors, formatters
│   │   ├── ai-ml-tools.nix        # LocalAI, PyTorch, transformers
│   │   ├── infrastructure.nix     # Prometheus, NATS, Vault, etc.
│   │   ├── security-tools.nix     # Trivy, cosign, gVisor, Kata
│   │   └── holochain.nix          # Holochain overlay and packages
│   │
│   ├── shells/
│   │   ├── default.nix            # Shell aggregator
│   │   ├── base-shell.nix         # Minimal shell (fast startup)
│   │   ├── full-shell.nix         # Complete development shell
│   │   ├── cuda-shell.nix         # GPU-enabled shell (Linux only)
│   │   ├── ci-shell.nix           # CI/CD optimized shell
│   │   └── identity-shell.nix     # mTLS/PKI development shell
│   │
│   ├── commands/
│   │   ├── default.nix            # Command aggregator
│   │   ├── ros2-commands.nix      # cb, ct, ros2-clean, ros2-ws
│   │   ├── ai-commands.nix        # localai, agixt, aios, promptfoo
│   │   ├── infra-commands.nix     # vault-dev, kata, neonctl
│   │   └── sandbox-commands.nix   # sandbox-runtime, mcp-toolbox
│   │
│   ├── overlays/
│   │   ├── default.nix            # Overlay aggregator
│   │   ├── ros2-overlay.nix       # ROS2 package overrides
│   │   └── holochain-overlay.nix  # Holochain packages
│   │
│   ├── modules/
│   │   ├── nixos/                 # NixOS system modules
│   │   │   └── default.nix
│   │   ├── darwin/                # macOS/nix-darwin modules
│   │   │   └── default.nix
│   │   └── home-manager/          # User-level modules
│   │       ├── common.nix
│   │       ├── linux.nix
│   │       └── macos.nix
│   │
│   ├── images/                    # NixOS image generation
│   │   ├── default.nix            # Image builders aggregator
│   │   ├── wsl.nix                # WSL2 image configuration
│   │   ├── iso.nix                # ISO installer image
│   │   └── vm.nix                 # QEMU/VirtualBox VM image
│   │
│   └── lib/
│       ├── default.nix            # Library functions
│       └── helpers.nix            # Utility functions
│
├── modules/                       # (existing) Home-manager modules
│   ├── common/
│   ├── linux/
│   └── macos/
│
└── lib/                           # (existing) Exported library
```

## Module Interfaces

### Package Module Interface

```nix
# nix/packages/ros2-packages.nix
{ pkgs, lib, ... }:
{
  # List of packages for this category
  packages = with pkgs; [
    # ROS2 packages here
  ];

  # Optional: package overrides
  overlays = [];

  # Optional: build inputs
  buildInputs = [];
}
```

### Shell Module Interface

```nix
# nix/shells/base-shell.nix
{ pkgs, lib, packages, commands, ... }:
pkgs.mkShell {
  name = "ros2-humble-base";

  packages = packages.base ++ commands.core;

  shellHook = ''
    # Shell initialization
  '';

  # Environment variables
  ROS_DISTRO = "humble";
}
```

### Command Wrapper Interface

```nix
# nix/commands/ros2-commands.nix
{ pkgs, ... }:
{
  cb = pkgs.writeShellScriptBin "cb" ''
    exec colcon build --symlink-install "$@"
  '';

  ct = pkgs.writeShellScriptBin "ct" ''
    exec colcon test "$@"
  '';

  # ... more commands
}
```

## Image Generation

### Tools Overview

| Tool | Purpose | Output Format |
|------|---------|---------------|
| `nixos-rebuild` | Build NixOS system closures | System activation, tarballs |
| `nixos-generators` | Generate various image formats | ISO, VM, Docker, SD card |
| `NixOS-WSL` | WSL2-specific image generation | ext4 filesystem tarball |

### Generation Methods

#### Method 1: nixos-rebuild (Native)

```bash
# Build a WSL-compatible NixOS image
nixos-rebuild build --flake .#wsl-ros2

# Build and switch (on running NixOS)
nixos-rebuild switch --flake .#wsl-ros2
```

#### Method 2: nixos-generators (Multi-format)

```bash
# Install nixos-generators
nix-env -iA nixpkgs.nixos-generators

# Generate ISO installer
nixos-generate -f iso -c ./nix/images/iso.nix

# Generate QEMU VM image
nixos-generate -f qcow -c ./nix/images/vm.nix

# Generate Docker image
nixos-generate -f docker -c ./nix/images/docker.nix

# Generate SD card image (Raspberry Pi)
nixos-generate -f sd-aarch64-installer -c ./nix/images/rpi.nix
```

#### Method 3: Flake Outputs (Recommended)

```bash
# Build WSL image via flake output
nix build .#nixosConfigurations.wsl-ros2.config.system.build.tarballBuilder

# Build ISO via flake output
nix build .#nixosConfigurations.iso-ros2.config.system.build.isoImage

# Build VM via flake output
nix build .#nixosConfigurations.vm-ros2.config.system.build.vm
```

### NixOS-WSL Image Configuration

```nix
# nix/images/wsl.nix
{ nixos-wsl, pkgs, ... }:
nixos-wsl.lib.nixosSystem {
  system = "x86_64-linux";
  modules = [
    nixos-wsl.nixosModules.wsl
    ({ config, ... }: {
      wsl = {
        enable = true;
        defaultUser = "nixos";
        nativeSystemd = true;
      };

      # ROS2 environment pre-configured
      environment.systemPackages = with pkgs; [
        pixi
        git
      ];

      # Auto-enter development environment
      programs.bash.interactiveShellInit = ''
        cd ~/ros2-humble-env && nix develop
      '';
    })
  ];
}
```

### WSL Import Commands

```powershell
# Import the tarball into WSL
wsl --import NixOS-ROS2 $env:USERPROFILE\WSL\NixOS-ROS2 result/nixos-wsl.tar.gz

# Launch the new distro
wsl -d NixOS-ROS2

# Set as default (optional)
wsl --set-default NixOS-ROS2
```

### Available Image Formats

| Format | Command | Use Case |
|--------|---------|----------|
| WSL2 tarball | `nix build .#images.wsl` | Windows development |
| ISO installer | `nix build .#images.iso` | Bare metal installation |
| QEMU/qcow2 | `nix build .#images.vm` | Local testing |
| Docker | `nixos-generate -f docker` | Container deployment |
| SD card | `nixos-generate -f sd-aarch64` | Raspberry Pi/embedded |
| VirtualBox | `nixos-generate -f virtualbox` | Desktop virtualization |
| Amazon EC2 | `nixos-generate -f amazon` | Cloud deployment |

## Migration Strategy

### Phase 1: Extract Packages (Low Risk)

1. Create `nix/packages/` directory structure
2. Move package lists to dedicated files
3. Import in main flake.nix
4. Test: `nix flake check`

### Phase 2: Extract Commands (Low Risk)

1. Create `nix/commands/` directory
2. Move command wrappers to categorized files
3. Update references in shells
4. Test: `nix develop --command cb`

### Phase 3: Extract Shells (Medium Risk)

1. Create `nix/shells/` directory
2. Move devShell definitions
3. Ensure all dependencies resolved
4. Test each shell variant

### Phase 4: Add Image Builders (Additive)

1. Add `nixos-generators` or `NixOS-WSL` as input
2. Create `nix/images/` directory
3. Add image configurations
4. Document build commands

### Phase 5: Refactor Modules (Medium Risk)

1. Move modules to `nix/modules/`
2. Update export paths in flake.nix
3. Test with example configurations

## Backward Compatibility

### Preserved Interfaces

- `nix develop` → default shell
- `nix develop .#full` → full shell
- `nix develop .#cuda` → CUDA shell
- `self.lib.homeManagerModules` → unchanged
- `self.nixosModules.default` → unchanged

### New Interfaces Added

- `nix build .#images.wsl` → WSL tarball
- `nix build .#images.iso` → ISO installer
- `nix build .#images.vm` → VM image

## Testing Strategy

```bash
# Test flake validity
nix flake check

# Test each shell
nix develop --command echo "default works"
nix develop .#full --command echo "full works"
nix develop .#cuda --command echo "cuda works"  # Linux only

# Test commands
nix develop --command cb --help
nix develop --command localai status

# Test image builds (CI)
nix build .#images.wsl --dry-run
```

## Timeline Estimate

| Phase | Effort | Risk | Dependencies |
|-------|--------|------|--------------|
| Phase 1: Packages | 2-4 hours | Low | None |
| Phase 2: Commands | 2-3 hours | Low | Phase 1 |
| Phase 3: Shells | 3-5 hours | Medium | Phase 1, 2 |
| Phase 4: Images | 2-4 hours | Low | None (additive) |
| Phase 5: Modules | 2-3 hours | Medium | None |

**Total: 11-19 hours of focused work**

## Deployment Tools

For deploying NixOS configurations to multiple machines:

| Tool | Purpose | Use Case |
|------|---------|----------|
| [deploy-rs](https://github.com/serokell/deploy-rs) | Multi-host deployment | Fleet management |
| [Colmena](https://github.com/zhaofengli/colmena) | NixOS deployment tool | Parallel deployment |
| [NixOps](https://github.com/NixOS/nixops) | NixOS cloud deployment | Infrastructure as code |

```bash
# Example: Colmena deployment
colmena apply --on @ros2-nodes

# Example: deploy-rs
deploy .#ros2-server
```

## References

### Architecture Patterns

- [Holo-Host Architecture](https://github.com/Holo-Host/holo-host) - Multi-service NixOS flake monorepo
- [ALT-F4-LLC/kickstart.nix](https://github.com/ALT-F4-LLC/kickstart.nix) - flake-parts template
- [Holochain Flake](https://github.com/holochain/holochain) - flake-parts auto-importing

### NixOS Discourse Discussions

- [Breaking up a monolithic flake](https://discourse.nixos.org/t/breaking-up-a-monolithic-flake/30475)
- [Improve Flake Performance in Monorepo](https://discourse.nixos.org/t/improve-flake-performance-in-absurdly-bloated-monorepo/21282)
- [Multiple Nix Flake Packages](https://discourse.nixos.org/t/is-there-advantages-to-multiple-nix-flake-packages/14344)

### Tools Documentation

- [Nix Flakes](https://nixos.wiki/wiki/Flakes) - Flake system overview
- [flake-parts](https://flake.parts/) - Modular flake framework (currently used)
- [nixos-generators](https://github.com/nix-community/nixos-generators) - Image generation
- [NixOS-WSL](https://github.com/nix-community/NixOS-WSL) - WSL integration
- [Nix Module System](https://nixos.wiki/wiki/NixOS_modules) - Module patterns

### WSL Resources

- [kenlasko/nixos-wsl](https://github.com/kenlasko/nixos-wsl) - Example WSL configuration
- [NixOS on WSL Guide](https://medium.com/@piyushkumarsingh.nmims/nixos-on-wsl-the-best-of-both-worlds-bdcaba54ee58)
