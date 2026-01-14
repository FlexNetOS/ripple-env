# Ripple Environment

- Uses the original environment template setup from "ripple-env" to create a WSL2 distro.
- A reproducible and declarative development environment for ROS2 Humble using Nix flakes and pixi for cross-platform compatibility.
- This repository is meant to be used as a "template" repository for robotics projects to offer an easy starting environment with a working ROS2 install.

## 🚨 WSL Stability Issues - Quick Fix Guide

### Problem: Commands fail and window reloads

This is a **common WSL issue** with memory pressure and timeout problems. Here's the immediate fix:

### ⚡ Quick Fix (Run These Now)

```bash
# 1. Apply WSL stability fixes
./scripts/fix-wsl-stability.sh

# 2. Enter stable environment (use this instead of regular commands)
source scripts/stable-env.sh

# 3. Use safe commands with timeouts
pixi-safe install --no-update-lockfile
nix-safe develop
```

### 🎯 Stable Command Aliases

After running the fix script, these **timeout-protected** commands are available:

| Command | Purpose | Timeout |
|---------|---------|---------|
| `ripple-enter` | Enter stable environment | - |
| `pixi-safe` | Safe pixi commands | 5 min |
| `nix-safe` | Safe nix commands | 10 min |
| `docker-safe` | Safe docker commands | 2 min |
| `ripple-save` | Save session state | - |
| `ripple-restore` | Restore after reload | - |

### 🔄 Recovery After Reload

When WSL reloads, run:
```bash
cd-ripple  # Returns to project + restores environment
ripple-restore  # Recovers saved state
```

### 📋 Windows WSL Config (Required)

**Copy this to Windows (run in WSL):**
```bash
cp /tmp/.wslconfig /mnt/c/Users/$USER/.wslconfig
```

**Then restart WSL:**
```powershell
# In Windows PowerShell:
wsl --shutdown
# Wait 10 seconds, then reopen
```

### 🐧 Why This Happens

1. **Memory Pressure**: WSL has limited memory vs native Linux
2. **Timeout Issues**: Long-running commands trigger WSL timeouts
3. **Docker Conflicts**: Network and resource conflicts
4. **Nix Memory Usage**: Heavy builds cause WSL instability

### 💎 The Fix Addresses

✅ **Memory Limits**: Sets 8GB max, 2GB swap
✅ **Timeout Protection**: All commands have safe timeouts
✅ **Resource Optimization**: Reduces parallel jobs
✅ **Session Recovery**: Saves/restores working state
✅ **WSL Configuration**: Optimizes Windows WSL settings

### 💡 Pro Tips

1. **Always save first**: `ripple-save` before heavy operations
2. **Use safe commands**: `pixi-safe` instead of `pixi`
3. **Monitor memory**: `free -h` to check WSL memory usage
4. **Regular cleanup**: `docker system prune -f` weekly

### 🆘 Emergency Recovery

If completely stuck:
```bash
# Minimal recovery
source scripts/stable-env.sh
cd /home/nixos/ripple-env
./scripts/session-restore.sh
```

**This should resolve the reload issues!** The stable environment prevents the memory/timeout problems that cause WSL to disconnect.

## Quick Start

### Windows (WSL2 + NixOS)

```powershell
# Run PowerShell as Administrator
# Download and run the bootstrap script
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.ps1" -OutFile "bootstrap.ps1"
.\bootstrap.ps1
```

### Linux / macOS

```bash
# Clone the repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# Run the bootstrap script (installs everything)
./bootstrap.sh

# Or if you already have nix installed:
nix develop  # (or: nom develop for nicer output)
```

## Goal

### Bootstrap Environment
- ✅ A single script run from Windows PowerShell that checks for Linux and WSL2 install, then installs and updates them as needed.
- ✅ Creates the NixOS Distro, registers it, creates the ext4.vhdx hard disk image 1TB, and swap image.
- ✅ Loads the nix flake and configurations per direnv
- ✅ Sets up pixi package manager, tools, and packages
- ✅ Adds zsh and nushell (bash stays default with nix)
- ✅ Includes `nom` (nix-output-monitor) for nicer output (optional; `nix` still works)
- ✅ Installs git and gh cli

### Agentic System
- 🔄 Agentic system that runs robotics, DevOps, user's personal life and work life
- ✅ Modular and Portable - Configuration modules can be imported into any Nix flake
- ✅ Built for Cross-platform Use - Linux, macOS, and Windows (WSL2) support
- ✅ Clear Configs and Schema - Documented options with types and defaults

### Documentation
- ✅ [Getting Started](docs/GETTING_STARTED.md) - Quick setup guide
- ✅ [Onboarding Tutorial](docs/ONBOARDING_TUTORIAL.md) - Interactive step-by-step tutorial
- ✅ [Progressive Examples](docs/PROGRESSIVE_EXAMPLES.md) - Beginner to advanced examples
- ✅ [Common Pitfalls](docs/COMMON_PITFALLS.md) - Avoid common mistakes
- ✅ [Video Walkthroughs](docs/VIDEO_WALKTHROUGHS.md) - Visual learning resources
- ✅ [Troubleshooting](docs/TROUBLESHOOTING.md) - Solutions to common issues

### Agent Documentation
- ✅ [AGENT.md](.claude/AGENT.md) - Agent system architecture and capabilities
- ✅ [CLAUDE.md](.claude/CLAUDE.md) - Claude Code specific instructions
- ✅ [RULES.md](.claude/RULES.md) - Rules and contribution guidelines
- ✅ [SKILL.md](.claude/SKILL.md) - Available skills and tool reference
- ✅ [INDEX.md](.claude/INDEX.md) - Documentation navigation

## Overview

This repository provides a complete development setup for ROS2 Humble with all necessary build tools and dependencies pre-configured. The environment works on:

- **Windows** (via WSL2 + NixOS)
- **Linux** (native, x86_64 and aarch64)
- **macOS** (x86_64 and arm64)

Using:
- **Nix flakes**: for reproducible, declarative environment setup
- **pixi**: for Python and ROS package management via RoboStack
- **direnv**: for automatic environment activation
- **home-manager modules**: for comprehensive shell and editor configuration

## Getting Started

### Windows Installation (Recommended for Windows users)

The PowerShell bootstrap script (`bootstrap.ps1`) automates the entire setup:

```powershell
# Option 1: Run directly (requires Administrator)
Set-ExecutionPolicy Bypass -Scope Process -Force
.\bootstrap.ps1

# Option 2: With custom options
.\bootstrap.ps1 -DistroName "MyROS2" -DiskSizeGB 512 -SwapSizeGB 16
```

**What it does:**
1. Checks Windows version (requires build 19041+)
2. Enables WSL2 and Virtual Machine Platform
3. Downloads and imports NixOS-WSL distribution
4. Creates 1TB ext4.vhdx virtual disk
5. Configures swap (default 8GB)
6. Runs the Linux bootstrap script inside WSL
7. Sets up the complete ROS2 environment

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `-DistroName` | NixOS-Ripple | Name for the WSL distribution |
| `-InstallPath` | `$HOME\WSL\NixOS-Ripple` | Installation directory |
| `-DiskSizeGB` | 1024 | Virtual disk size (1TB) |
| `-SwapSizeGB` | 8 | Swap size in GB |
| `-SkipWSLCheck` | false | Skip WSL installation check |
| `-Force` | false | Replace existing distro |

**After installation:**
```powershell
# Enter the environment
wsl -d NixOS-Ripple
cd ~/ripple-env
direnv allow

# If you prefer manual activation (direnv optional):
nix develop  # (or: nom develop for nicer output)
```

### Linux / macOS Installation

```bash
./bootstrap.sh
```

This script will:
1. Install Nix with flakes enabled (using Determinate Systems installer)
2. Install direnv, nom, git, gh cli
3. Optionally install zsh and nushell
4. Verify the flake and pixi setup

For CI environments:
```bash
./bootstrap.sh --ci --skip-shells
```

### Manual Installation

If you prefer to install manually:

```bash
# Install nix (experimental installer with flakes)
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# Enter the development environment
nix develop

# Optional (nicer output):
nom develop
```

### Using direnv

If you have direnv installed, simply enter the directory:

```bash
cd ripple-env
direnv allow
```

direnv will automatically load the environment.

## Workspace Structure

```
ripple-env/
├── flake.nix              # Main nix flake configuration
├── flake.lock             # Locked dependency versions
├── pixi.toml              # Pixi workspace definition
├── pixi.lock              # Pixi locked dependencies
├── bootstrap.sh           # Linux/macOS bootstrap script
├── bootstrap.ps1          # Windows PowerShell bootstrap script
├── docker/
│   └── docker-compose.agixt.yml  # AGiXT Docker services
├── .envrc                 # Direnv configuration
├── .env.agixt.example     # AGiXT environment template
├── BUILDKIT_STARTER_SPEC.md  # Stack specification (SSoT)
├── .github/
│   └── workflows/
│       └── bootstrap-test.yml  # CI workflow
├── .claude/               # Agent configuration
│   ├── AGENT.md           # Agent system architecture
│   ├── CLAUDE.md          # Claude Code instructions
│   ├── RULES.md           # Rules and guidelines
│   ├── SKILL.md           # Skills reference
│   ├── INDEX.md           # Documentation navigation
│   ├── settings.json      # Permissions and hooks
│   ├── agents/            # Agent role definitions
│   ├── skills/            # Structured skill definitions
│   └── commands/          # Slash commands
├── rust/                  # Rust workspace
│   ├── Cargo.toml         # Workspace configuration
│   └── agixt-bridge/      # AGiXT Rust SDK bridge
├── docs/
│   └── adr/               # Architecture Decision Records
│       ├── README.md
│       └── adr-001..006   # ADR documents
├── lib/
│   ├── default.nix        # Library utilities
│   └── system.nix         # System builder helpers
└── modules/
    ├── common/            # Cross-platform configurations
    │   ├── default.nix    # Module aggregator
    │   ├── direnv.nix     # Enhanced direnv config
    │   ├── git.nix        # Git configuration
    │   ├── packages.nix   # Common packages
    │   ├── xdg.nix        # XDG Base Directory support
    │   ├── ai/            # AI assistants
    │   │   ├── default.nix
    │   │   ├── aichat.nix # Provider-agnostic chat
    │   │   └── aider.nix  # AI pair programming
    │   ├── nix/           # Nix settings
    │   ├── editor/        # Editors with LSPs
    │   │   ├── default.nix
    │   │   └── neovim.nix # Neovim configuration
    │   └── shell/         # Shell configurations
    │       ├── default.nix
    │       ├── nushell.nix
    │       ├── zsh.nix
    │       ├── bash.nix
    │       ├── zoxide.nix
    │       └── starship.nix
    ├── linux/             # Linux-specific configurations
    │   ├── default.nix
    │   ├── packages.nix
    │   ├── docker.nix
    │   ├── udev.nix       # Device rules for robotics
    │   ├── users.nix
    │   └── systemd.nix
    └── macos/             # macOS-specific configurations
        ├── default.nix
        ├── packages.nix
        ├── homebrew.nix
        ├── system.nix
        └── shell.nix
```

## Environment Details

The workspace includes:

### Core Tools
- **ROS**: ros-humble-desktop with all core ROS packages
- **Build tools**: cmake, ninja, make, compilers, pkg-config
- **ROS tools**: colcon, rosdep, catkin_tools
- **Python**: 3.11.x with development headers

### Development Environment
- **Shells**: bash, zsh, nushell (with starship prompt)
- **Editor**: helix with LSPs for Python, C++, CMake, Nix, YAML, XML
- **AI Assistants**: aichat (lightweight chat), aider (git-integrated pair programming)
- **Navigation**: zoxide (smart cd), fzf (fuzzy finder)
- **Utilities**: bat, eza, ripgrep, fd, jq, yq

### Platforms
- Windows (WSL2), linux-64, linux-aarch64, osx-64, osx-arm64

## Quick Commands

Once in the development environment:

```bash
# Build ROS packages
cb                          # colcon build --symlink-install
colcon build

# Test packages
ct                          # colcon test
ctr                         # colcon test-result --verbose

# Package management
pixi add <PACKAGE_NAME>     # Add a robostack package
pixi search ros-humble-*    # Search for ROS2 packages

- ros: ros-humble-desktop with all core ros packages
- build tools: cmake, ninja, make, compilers, pkg-config
- ros tools: colcon, rosdep, catkin_tools
- python: 3.11.x with development headers
- python modernization tools: ruff, pyupgrade, flynt (for upgrading legacy python code)
- platforms: supports linux-64, linux-aarch64, osx-64, osx-arm64

# Environment info
ros2-env                    # Show ROS2 environment variables

# Update dependencies
update-deps                 # pixi update

# AI assistance
ai "explain ROS2 topics"    # Ask AI anything
ai-code "write a publisher" # Generate code
ai-review                   # Review code (pipe input)

# AI pair programming (aider)
pair                        # Start AI pair programming
pair src/my_node.py         # Work on specific files
pair-voice                  # Voice-to-code mode

# Local AI inference (requires devshell.full)
localai start               # Start LocalAI server
localai status              # Check server status
localai models              # List available models

# AGiXT agent platform (requires Docker)
agixt up                    # Start AGiXT services
agixt status                # Check service status
agixt logs                  # View logs

# LLM evaluation
promptfoo eval              # Run prompt tests

# Development tools
vault-dev                   # Start Vault in dev mode
```

## Using Different Shells

The development environment supports multiple shells:

### Nushell (Recommended)
```bash
nom develop -c env 'SHELL=/bin/nu' /bin/nu
```

### Zsh
```bash
nom develop -c env 'SHELL=/bin/zsh' /bin/zsh
```

### Create an alias
Add to your shell config:
```bash
alias devshell="nom develop -c env 'SHELL=/bin/bash' /bin/bash"
```

## Using Modules in Other Flakes

The modules can be imported into other flakes:

```nix
{
  inputs = {
    ripple-env.url = "github:FlexNetOS/ripple-env";
    home-manager.url = "github:nix-community/home-manager";
  };

  outputs = { self, ripple-env, home-manager, ... }: {
    homeConfigurations.myuser = home-manager.lib.homeManagerConfiguration {
      modules = [
        ripple-env.lib.homeManagerModules.default
        # Your other modules...
      ];
    };
  };
}
```

## NixOS Image Generation

This flake supports generating various NixOS image formats for deployment:

### WSL2 Image

Generate a WSL2-compatible NixOS tarball with ROS2 pre-configured:

```bash
# Build WSL tarball (future capability)
nix build .#images.wsl

# Import to Windows WSL
wsl --import NixOS-Ripple-Custom $env:USERPROFILE\WSL\NixOS-Ripple result/nixos-wsl.tar.gz
```

### Other Image Formats

Using [nixos-generators](https://github.com/nix-community/nixos-generators), you can create:

| Format | Command | Use Case |
|--------|---------|----------|
| WSL | `nix build .#images.wsl` | Windows Subsystem for Linux |
| ISO | `nix build .#images.iso` | Bootable installer |
| VM (QEMU) | `nix build .#images.vm` | Virtual machine image |
| Docker | `nix build .#images.docker` | Container image |
| SD Card | `nix build .#images.sd-aarch64` | Raspberry Pi / ARM devices |

### Architecture Documentation

For detailed information about the flake architecture and planned modularization:

- **[NIX_FLAKE_MODULARIZATION.md](docs/NIX_FLAKE_MODULARIZATION.md)** - Modularization strategy and image generation details

## Adding Packages

### ROS2 Packages (via Pixi)

```bash
pixi add ros-humble-<package-name>
```

Find available packages at [RoboStack channel](https://robostack.github.io/humble.html).

### Python Packages (via Pixi)

```bash
pixi add pygame
```

### Nix Packages

Add to `flake.nix` in the `commonPackages` list.

## Python Modernization Tools

This environment includes tools for upgrading legacy Python code to modern syntax. These tools are particularly useful when working with ROS packages that may use older Python patterns.

### Ruff

An extremely fast Python linter and formatter (10-100x faster than flake8/black), written in Rust. It includes pyupgrade rules built-in.

```bash
# Lint Python files
ruff check .

# Auto-fix issues
ruff check --fix .

# Format code (like black)
ruff format .

# Upgrade Python syntax (using UP rules)
ruff check --select UP --fix .
```

### Pyupgrade

Automatically upgrades Python syntax to newer versions of the language.

```bash
# Upgrade to Python 3.11+ syntax
find . -name "*.py" -exec pyupgrade --py311-plus {} +

# Examples of transformations:
# - set([1, 2]) -> {1, 2}
# - class Foo(object) -> class Foo
# - super(Foo, self) -> super()
# - Optional[int] -> int | None (3.10+)
# - "{}".format(x) -> f"{x}"
```

### Flynt

Converts old-style string formatting (% and .format()) to f-strings.

```bash
# Convert a file
flynt file.py

# Convert entire directory
flynt src/

# Dry run (see changes without modifying)
flynt --dry-run src/

# Also convert string concatenation
flynt --transform-concats src/
```

### Combined Workflow

For a complete modernization of a Python codebase:

```bash
# Step 1: Run ruff to fix common issues
ruff check --fix .

# Step 2: Upgrade syntax with pyupgrade
find . -name "*.py" -not -path "./.venv/*" -exec pyupgrade --py311-plus {} +

# Step 3: Convert to f-strings with flynt
flynt .

# Step 4: Format with ruff
ruff format .

# Step 5: Final lint check
ruff check .
```

## Troubleshooting

### Windows/WSL2

**WSL2 not available:**
- Ensure Windows 10 version 2004+ or Windows 11
- Run `wsl --install` manually if needed
- Restart after enabling WSL features

**Disk space issues:**
- The default 1TB vhdx is sparse (only uses actual data size)
- Reduce with `-DiskSizeGB 256` if needed

**Performance tips:**
- Store projects inside WSL filesystem (`/home/user/`)
- Avoid accessing Windows drives (`/mnt/c/`) for builds

### Linux/macOS

**Nix installation issues:**
- Check https://github.com/DeterminateSystems/nix-installer
- Ensure curl and bash are available

**Flake errors:**
- Run `nix flake update` to refresh dependencies
- Check `nix flake check` for validation

## Links

### ROS2 & Robotics
- [ROS2 Humble documentation](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [RoboStack ROS2-humble packages](https://robostack.github.io/humble.html)
- [ROS2 Design Documentation](https://design.ros2.org/)
- [ROS2 Packages Index](https://index.ros.org/packages/)

### Pixi & Package Management
- [Pixi documentation](https://pixi.sh)
- [Pixi Getting Started](https://pixi.sh/latest/getting_started/)
- [RoboStack GitHub](https://github.com/RoboStack/ros-humble)
- [Conda-forge](https://conda-forge.org/)

### Nix Ecosystem
- [Nix Manual](https://nixos.org/manual/nix/stable/)
- [Nix flakes documentation](https://nixos.wiki/wiki/Flakes)
- [NixOS Wiki](https://wiki.nixos.org/)
- [Nix Pills (learning resource)](https://nixos.org/guides/nix-pills/)
- [Zero to Nix](https://zero-to-nix.com/)
- [Determinate Systems Nix Installer](https://github.com/DeterminateSystems/nix-installer)
- [devenv.sh](https://devenv.sh/)
- [Flake-parts documentation](https://flake.parts/)
- [numtide devshell](https://github.com/numtide/devshell)

### Home Manager
- [Home-manager documentation](https://nix-community.github.io/home-manager/)
- [Home-manager Options Search](https://home-manager-options.extranix.com/)
- [Home-manager GitHub](https://github.com/nix-community/home-manager)

### Windows / WSL2
- [NixOS-WSL](https://github.com/nix-community/NixOS-WSL)
- [WSL Documentation](https://learn.microsoft.com/en-us/windows/wsl/)
- [WSL Best Practices](https://learn.microsoft.com/en-us/windows/wsl/setup/environment)
- [WSL2 Networking](https://learn.microsoft.com/en-us/windows/wsl/networking)

### Shells & Terminal
- [Starship Prompt](https://starship.rs/)
- [Nushell Documentation](https://www.nushell.sh/book/)
- [Zsh Documentation](https://zsh.sourceforge.io/Doc/)
- [direnv Documentation](https://direnv.net/)
- [nix-direnv](https://github.com/nix-community/nix-direnv)

### Editor
- [Helix Editor](https://helix-editor.com/)
- [Helix Documentation](https://docs.helix-editor.com/)
- [Helix Language Support](https://docs.helix-editor.com/lang-support.html)

### Python Modernization
- [Ruff Documentation](https://docs.astral.sh/ruff/)
- [Pyupgrade Repository](https://github.com/asottile/pyupgrade)
- [Flynt Repository](https://github.com/ikamensh/flynt)

## Configuration Sources

This configuration incorporates patterns and modules from:
- [GustavoWidman/nix](https://github.com/GustavoWidman/nix) - Multi-machine Nix configuration
- [RGBCube/ncc](https://github.com/RGBCube/ncc) - Comprehensive NixOS/Darwin configuration

## Agentic Infrastructure Resources

### Integrated Tools

The following tools are **integrated and available** in this environment:

| Tool | Description | Command/Usage |
|------|-------------|---------------|
| **LocalAI** | OpenAI-compatible local inference | `localai start` |
| **AGiXT** | AI Agent Automation Platform (Docker) | `agixt up` |
| **AGiXT Rust SDK** | Rust SDK for AGiXT | `rust/agixt-bridge/` |
| **Prometheus** | Metrics and monitoring | In flake.nix |
| **NATS** | Event bus messaging | `natscli`, `nats-server` |
| **Trivy** | Security vulnerability scanning | `trivy` |
| **OPA** | Policy-as-code decisions | `opa` |
| **Vault** | Secrets management | `vault-dev` |
| **promptfoo** | LLM prompt testing | `promptfoo eval` |
| **sqlx-cli** | Type-safe SQL migrations | `sqlx` |
| **maturin** | PyO3/Rust-Python bindings | `maturin` |
| **AIOS** | AI Agent Operating System (kernel) | `aios start` |
| **Cerebrum** | Agent SDK for AIOS | `pixi run -e aios ...` |

### Evaluated / Planned

The following resources are documented but require additional setup:

| Project | Description | URL |
|---------|-------------|-----|
| **Temporal** | Durable workflows | [temporalio/temporal](https://github.com/temporalio/temporal) |
| **OpenTelemetry** | Observability framework | [open-telemetry](https://github.com/open-telemetry) |

### Training & Machine Learning

| Project | Description | URL |
|---------|-------------|-----|
| **unsloth** | Fast LLM fine-tuning | [unslothai/unsloth](https://github.com/unslothai/unsloth) |
| **ComfyUI** | Modular stable diffusion GUI | [comfyanonymous/ComfyUI](https://github.com/comfyanonymous/ComfyUI) |

### Memory & Context Systems

| Project | Description | URL |
|---------|-------------|-----|
| **memobase** | User Profile-Based Long-Term Memory | [memodb-io/memobase](https://github.com/memodb-io/memobase) |
| **Memori** | SQL Native Memory Layer | [MemoriLabs/Memori](https://github.com/MemoriLabs/Memori) |

### Gateways & API Management

| Project | Description | URL |
|---------|-------------|-----|
| **agentgateway** | AI Agent Gateway | [agentgateway/agentgateway](https://github.com/agentgateway/agentgateway) |
| **Kong** | Cloud-Native API Gateway | [Kong/kong](https://github.com/Kong/kong) |

### Distributed Systems & Networking

| Project | Description | URL |
|---------|-------------|-----|
| **rust-libp2p** | Rust libp2p networking stack | [libp2p/rust-libp2p](https://github.com/libp2p/rust-libp2p) |
| **holochain** | Distributed User Resources | [holochain/holochain](https://github.com/holochain/holochain) |
| **noa-dynamo** | Org Datacenter Scale Distributed Inference | [FlexNetOS/dynamo](https://github.com/FlexNetOS/dynamo/tree/noa-dynamo) |
| **temporal** | Workflow orchestration platform | [temporalio/temporal](https://github.com/temporalio/temporal) |
| **NATS** | Cloud native messaging system | [nats-io/nats-server](https://github.com/nats-io/nats-server) |
| **Kubo** | IPFS implementation in Go | [ipfs/kubo](https://github.com/ipfs/kubo) |

### Databases & Storage

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.13, §1.15

| Project | Description | URL |
|---------|-------------|-----|
| **postgres** | System-of-record database | [postgres/postgres](https://github.com/postgres/postgres) |
| **redis** | Cache and sessions | [redis/redis](https://github.com/redis/redis) |
| **minio** | S3-compatible object store | [minio/minio](https://github.com/minio/minio) |
| **mindsdb** | AI query layer over data sources | [mindsdb/mindsdb](https://github.com/mindsdb/mindsdb) |
| **neo4j** | Graph database | [neo4j/neo4j](https://github.com/neo4j/neo4j) |
| **sqlx** | Rust SQL Toolkit | [launchbadge/sqlx](https://github.com/launchbadge/sqlx) |
| **neon** | Serverless Postgres | [neondatabase/neon](https://github.com/neondatabase/neon) |
| **vCache** | Distributed caching | [vcache-project/vCache](https://github.com/vcache-project/vCache) |

### Monitoring & Observability

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.18

| Project | Description | URL |
|---------|-------------|-----|
| **prometheus** | Monitoring and alerting toolkit | [prometheus/prometheus](https://github.com/prometheus/prometheus) |
| **grafana** | Dashboards and visualization | [grafana/grafana](https://github.com/grafana/grafana) |
| **loki** | Log aggregation system | [grafana/loki](https://github.com/grafana/loki) |
| **netdata** | Real-time infrastructure monitoring | [netdata/netdata](https://github.com/netdata/netdata) |
| **umami** | Privacy-focused web analytics | [umami-software/umami](https://github.com/umami-software/umami) |
| **OpenTelemetry** | Observability framework | [open-telemetry](https://github.com/open-telemetry) |
| **trippy** | Network diagnostic tool | [fujiapple852/trippy](https://github.com/fujiapple852/trippy) |

### Security & Secrets Management

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.4, §1.8

| Project | Description | URL |
|---------|-------------|-----|
| **trivy** | Security vulnerability scanner | [aquasecurity/trivy](https://github.com/aquasecurity/trivy) |
| **vault** | Secrets management | [hashicorp/vault](https://github.com/hashicorp/vault) |
| **vaultwarden** | Bitwarden-compatible server | [dani-garcia/vaultwarden](https://github.com/dani-garcia/vaultwarden) |
| **keycloak** | Identity and access management | [keycloak/keycloak](https://github.com/keycloak/keycloak) |
| **OPA** | Open Policy Agent | [open-policy-agent/opa](https://github.com/open-policy-agent/opa) |
| **step-ca** | PKI automation (mTLS) | [smallstep/cli](https://github.com/smallstep/cli) |
| **kata-containers** | VM-backed container isolation | [kata-containers/kata-containers](https://github.com/kata-containers/kata-containers) |
| **firecracker** | MicroVM pools for isolation | [firecracker-microvm/firecracker](https://github.com/firecracker-microvm/firecracker) |
| **gvisor** | Container sandbox runtime | [githubfoam/gvisor-sandbox](https://github.com/githubfoam/gvisor-sandbox) |

### DevOps & CI/CD

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.6, §1.9

| Project | Description | URL |
|---------|-------------|-----|
| **Argo CD** | GitOps continuous delivery | [argoproj/argo-cd](https://github.com/argoproj/argo-cd) |
| **Argo Rollouts** | Canary/blue-green deployments | [argoproj/argo-rollouts](https://github.com/argoproj/argo-rollouts) |
| **Argo Workflows** | DAG workflow orchestration | [argoproj/argo-workflows](https://github.com/argoproj/argo-workflows) |
| **bytebase** | Database CI/CD | [bytebase/bytebase](https://github.com/bytebase/bytebase) |
| **n8n** | Connector workflow automation | [n8n-io/n8n](https://github.com/n8n-io/n8n) |
| **AppFlowy-Cloud** | Self-hosted Notion alternative backend | [AppFlowy-IO/AppFlowy-Cloud](https://github.com/AppFlowy-IO/AppFlowy-Cloud) |
| **chirpstack** | LoRaWAN Network Server | [chirpstack/chirpstack](https://github.com/chirpstack/chirpstack) |

### Rust Development Tools

| Project | Description | URL |
|---------|-------------|-----|
| **RustCoder** | Rust MCP coding assistant | [cardea-mcp/RustCoder](https://github.com/cardea-mcp/RustCoder) |
| **rusty-tags** | Ctags generator for Rust | [dan-t/rusty-tags](https://github.com/dan-t/rusty-tags) |
| **syn** | Rust syntax parsing library | [dtolnay/syn](https://github.com/dtolnay/syn) |
| **coreutils** | Rust implementation of GNU coreutils | [uutils/coreutils](https://github.com/uutils/coreutils) |
| **kellnr** | Private Rust crate registry | [kellnr/kellnr](https://github.com/kellnr/kellnr) |
| **datafusion** | Rust analytics engine | [apache/datafusion](https://github.com/apache/datafusion) |

### JavaScript/TypeScript Build Tools

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.21

| Project | Description | URL |
|---------|-------------|-----|
| **swc** | Fast Rust-based JS/TS compiler (20x faster than Babel) | [swc-project/swc](https://github.com/swc-project/swc) |
| **esbuild** | Extremely fast JS bundler and minifier | [evanw/esbuild](https://github.com/evanw/esbuild) |

### AI Evaluation & Testing

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.16

| Project | Description | URL |
|---------|-------------|-----|
| **tensorzero** | LLMOps gateway (evals/experiments/telemetry) | [tensorzero/tensorzero](https://github.com/tensorzero/tensorzero) |
| **promptfoo** | LLM prompt testing framework | [promptfoo/promptfoo](https://github.com/promptfoo/promptfoo) |
| **trulens** | LLM app evaluation & tracking | [truera/trulens](https://github.com/truera/trulens) |
| **evals** | OpenAI evaluation framework | [openai/evals](https://github.com/openai/evals) |

### Robotics & Vision

| Project | Description | URL |
|---------|-------------|-----|
| **Josh-XT** | Vision-Driven Autonomous Robot Control | [Josh-XT](https://github.com/Josh-XT) |
| **jj** | Git-compatible VCS | [jj-vcs/jj](https://github.com/jj-vcs/jj) |

### Additional Resources

| Project | Description | URL |
|---------|-------------|-----|
| **daa** | Distributed agent architecture | [ruvnet/daa](https://github.com/ruvnet/daa) |
| **ruvector** | Vector operations library | [ruvnet/ruvector](https://github.com/ruvnet/ruvector) |
| **qudag** | Quantum DAG implementation | [ruvnet/qudag](https://github.com/ruvnet/qudag) |
| **API-mega-list** | Comprehensive API resource list | [cporter202/API-mega-list](https://github.com/cporter202/API-mega-list) |
| **zcf** | Configuration framework | [UfoMiao/zcf](https://github.com/UfoMiao/zcf) |

### Python/Rust Interop (for pixi)

| Project | Description | URL |
|---------|-------------|-----|
| **RustPython** | Python interpreter in Rust | [RustPython/RustPython](https://github.com/RustPython/RustPython) |
| **PyO3** | Rust bindings for Python | [PyO3/pyo3](https://github.com/PyO3/pyo3) |

## UI & Frontend Tools

> **Reference:** BUILDKIT_STARTER_SPEC.md §1.20

| Project | Description | URL |
|---------|-------------|-----|
| **lobe-chat** | Primary operator UI/workspace | [lobehub/lobe-chat](https://github.com/lobehub/lobe-chat) |
| **open-lovable** | UI/codegen tool - clone and recreate any website as React app | [firecrawl/open-lovable](https://github.com/firecrawl/open-lovable) |
| **pixijs** | 2D WebGL rendering library for graphics and data visualization | [pixijs/pixijs](https://github.com/pixijs/pixijs) |
| **shadcn-ui** | Accessible React component library | [shadcn-ui/ui](https://github.com/shadcn-ui/ui) |
| **ComfyUI** | Modular diffusion model GUI with graph/nodes interface | [Comfy-Org/ComfyUI](https://github.com/Comfy-Org/ComfyUI) |
| **dioxus** | Fullstack app framework for web, desktop, and mobile | [DioxusLabs/dioxus](https://github.com/DioxusLabs/dioxus) |
| **vibe-kanban** | Vibe Kanban board | [BloopAI/vibe-kanban](https://github.com/BloopAI/vibe-kanban) |
| **dashy** | Self-hostable personal dashboard | [Lissy93/dashy](https://github.com/Lissy93/dashy) |
| **cc-switch** | Cross-platform desktop All-in-One assistant | [farion1231/cc-switch](https://github.com/farion1231/cc-switch) |
| **Deep-Live-Cam** | Real-time face swap and video deepfake | [hacksider/Deep-Live-Cam](https://github.com/hacksider/Deep-Live-Cam) |
| **dynamic_widget** | Backend-Driven UI toolkit with JSON | [dengyin2000/dynamic_widget](https://github.com/dengyin2000/dynamic_widget) |
| **openui** | AI-driven UI description to live render | [wandb/openui](https://github.com/wandb/openui) |
| **MobileAgent** | Mobile GUI Agent Family | [X-PLUG/MobileAgent](https://github.com/X-PLUG/MobileAgent) |

### Documentation Tools

| Project | Description | URL |
|---------|-------------|-----|
| **architecture-decision-record** | Architecture decision record (ADR) templates | [joelparkerhenderson/architecture-decision-record](https://github.com/joelparkerhenderson/architecture-decision-record) |

## License

This project is licensed under the [MIT License](LICENSE). See the LICENSE file for details.
