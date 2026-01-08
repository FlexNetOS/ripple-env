# Riple Environment and Binary Creation

- Uses the original environment template setup from "ros2-humble-env" to create a WSL2 distro.
- A reproducible and declarative development environment for ROS2 Humble using Nix flakes and pixi for cross-platform compatibility.
- This repository is meant to be used as a "template" repository for robotics projects to offer an easy starting environment with a working ROS2 install.

## Quick Start

### Windows (WSL2 + NixOS)

```powershell
# Run PowerShell as Administrator
# Download and run the bootstrap script
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/FlexNetOS/ros2-humble-env/main/bootstrap.ps1" -OutFile "bootstrap.ps1"
.\bootstrap.ps1
```

### Linux / macOS

```bash
# Clone the repository
git clone https://github.com/FlexNetOS/ros2-humble-env.git
cd ros2-humble-env

# Run the bootstrap script (installs everything)
./bootstrap.sh

# Or if you already have nix installed:
nom develop
```

## Goal

### Bootstrap Environment
- ✅ A single script run from Windows PowerShell that checks for Linux and WSL2 install, then installs and updates them as needed.
- ✅ Creates the NixOS Distro, registers it, creates the ext4.vhdx hard disk image 1TB, and swap image.
- ✅ Loads the nix flake and configurations per direnv
- ✅ Sets up pixi package manager, tools, and packages
- ✅ Adds zsh and nushell (bash stays default with nix)
- ✅ Uses nom instead of nix
- ✅ Installs git and gh cli

### Agentic System
- 🔄 Agentic system that runs robotics, DevOps, user's personal life and work life
- ✅ Modular and Portable - Configuration modules can be imported into any Nix flake
- ✅ Built for Cross-platform Use - Linux, macOS, and Windows (WSL2) support
- ✅ Clear Configs and Schema - Documented options with types and defaults

### Documentation
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
| `-DistroName` | NixOS-ROS2 | Name for the WSL distribution |
| `-InstallPath` | `$HOME\WSL\NixOS-ROS2` | Installation directory |
| `-DiskSizeGB` | 1024 | Virtual disk size (1TB) |
| `-SwapSizeGB` | 8 | Swap size in GB |
| `-SkipWSLCheck` | false | Skip WSL installation check |
| `-Force` | false | Replace existing distro |

**After installation:**
```powershell
# Enter the environment
wsl -d NixOS-ROS2
cd ~/ros2-humble-env
direnv allow  # or: nom develop
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
nom develop
# or
nix develop
```

### Using direnv

If you have direnv installed, simply enter the directory:

```bash
cd ros2-humble-env
direnv allow
```

direnv will automatically load the environment.

## Workspace Structure

```
ros2-humble-env/
├── flake.nix              # Main nix flake configuration
├── flake.lock             # Locked dependency versions
├── pixi.toml              # Pixi workspace definition
├── pixi.lock              # Pixi locked dependencies
├── bootstrap.sh           # Linux/macOS bootstrap script
├── bootstrap.ps1          # Windows PowerShell bootstrap script
├── .envrc                 # Direnv configuration
├── .github/
│   └── workflows/
│       └── bootstrap-test.yml  # CI workflow
├── lib/
│   ├── default.nix        # Library utilities
│   └── system.nix         # System builder helpers
└── modules/
    ├── common/            # Cross-platform configurations
    │   ├── default.nix    # Module aggregator
    │   ├── direnv.nix     # Enhanced direnv config
    │   ├── git.nix        # Git configuration
    │   ├── packages.nix   # Common packages
    │   ├── ai/            # AI assistants (aichat)
    │   ├── nix/           # Nix settings
    │   ├── editor/        # Helix editor with ROS2 LSPs
    │   └── shell/         # Shell configurations
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
    ros2-humble-env.url = "github:FlexNetOS/ros2-humble-env";
    home-manager.url = "github:nix-community/home-manager";
  };

  outputs = { self, ros2-humble-env, home-manager, ... }: {
    homeConfigurations.myuser = home-manager.lib.homeManagerConfiguration {
      modules = [
        ros2-humble-env.lib.homeManagerModules.default
        # Your other modules...
      ];
    };
  };
}
```

## Adding Packages

### ROS2 Packages (via pixi)
```bash
pixi add ros-humble-<package-name>
```

Find available packages at [robostack channel](https://robostack.github.io/humble.html).

### Python Packages (via pixi)
```bash
pixi add pygame
```

### Nix Packages
Add to `flake.nix` in the `commonPackages` list.

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

## Configuration Sources

This configuration incorporates patterns and modules from:
- [GustavoWidman/nix](https://github.com/GustavoWidman/nix) - Multi-machine Nix configuration
- [RGBCube/ncc](https://github.com/RGBCube/ncc) - Comprehensive NixOS/Darwin configuration

## Agentic Infrastructure Resources

The following resources are being evaluated for integration into this agentic development platform:

### AI Agent Platforms & Automation

| Project | Description | URL |
|---------|-------------|-----|
| **AGiXT** | Dynamic AI Agent Automation Platform | [Josh-XT/AGiXT](https://github.com/Josh-XT/AGiXT) |
| **AGiXT Rust SDK** | Rust SDK for AGiXT | [AGiXT/rust-sdk](https://github.com/AGiXT/rust-sdk) |
| **LocalAI** | Distributed, P2P and decentralized inference | [mudler/LocalAI](https://github.com/mudler/LocalAI) |
| **AIOS** | AI Agent Operating System | [agiresearch/AIOS](https://github.com/agiresearch/AIOS) |
| **Cerebrum** | Agent SDK for AIOS | [agiresearch/Cerebrum](https://github.com/agiresearch/Cerebrum) |
| **agentic-flow** | AI agent orchestration framework | [ruvnet/agentic-flow](https://github.com/ruvnet/agentic-flow) |
| **devops** | Agentics DevOps toolkit | [agenticsorg/devops](https://github.com/agenticsorg/devops) |

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

| Project | Description | URL |
|---------|-------------|-----|
| **sqlx** | Rust SQL Toolkit | [launchbadge/sqlx](https://github.com/launchbadge/sqlx) |
| **neon** | Serverless Postgres | [neondatabase/neon](https://github.com/neondatabase/neon) |
| **vCache** | Distributed caching | [vcache-project/vCache](https://github.com/vcache-project/vCache) |

### Monitoring & Observability

| Project | Description | URL |
|---------|-------------|-----|
| **netdata** | Real-time infrastructure monitoring | [netdata/netdata](https://github.com/netdata/netdata) |
| **umami** | Privacy-focused web analytics | [umami-software/umami](https://github.com/umami-software/umami) |
| **prometheus** | Monitoring and alerting toolkit | [prometheus/prometheus](https://github.com/prometheus/prometheus) |
| **OpenTelemetry** | Observability framework | [open-telemetry](https://github.com/open-telemetry) |
| **trippy** | Network diagnostic tool | [fujiapple852/trippy](https://github.com/fujiapple852/trippy) |

### Security & Secrets Management

| Project | Description | URL |
|---------|-------------|-----|
| **trivy** | Security vulnerability scanner | [aquasecurity/trivy](https://github.com/aquasecurity/trivy) |
| **vault** | Secrets management | [hashicorp/vault](https://github.com/hashicorp/vault) |
| **vaultwarden** | Bitwarden-compatible server | [dani-garcia/vaultwarden](https://github.com/dani-garcia/vaultwarden) |
| **keycloak** | Identity and access management | [keycloak/keycloak](https://github.com/keycloak/keycloak) |
| **OPA** | Open Policy Agent | [open-policy-agent/opa](https://github.com/open-policy-agent/opa) |
| **gvisor** | Container sandbox runtime | [githubfoam/gvisor-sandbox](https://github.com/githubfoam/gvisor-sandbox) |

### DevOps & CI/CD

| Project | Description | URL |
|---------|-------------|-----|
| **Argo CD** | GitOps continuous delivery | [FlexNetOS/argo-cd](https://github.com/FlexNetOS/argo-cd) |
| **AppFlowy-Cloud** | Self-hosted Notion alternative backend | [AppFlowy-IO/AppFlowy-Cloud](https://github.com/AppFlowy-IO/AppFlowy-Cloud) |
| **chirpstack** | LoRaWAN Network Server | [chirpstack/chirpstack](https://github.com/chirpstack/chirpstack) |

### Rust Development Tools

| Project | Description | URL |
|---------|-------------|-----|
| **RustCoder** | Rust MCP coding assistant | [cardea-mcp/RustCoder](https://github.com/cardea-mcp/RustCoder) |
| **rusty-tags** | Ctags generator for Rust | [dan-t/rusty-tags](https://github.com/dan-t/rusty-tags) |
| **syn** | Rust syntax parsing library | [dtolnay/syn](https://github.com/dtolnay/syn) |
| **coreutils** | Rust implementation of GNU coreutils | [uutils/coreutils](https://github.com/uutils/coreutils) |
| **swc** | Rust-based JS/TS compiler | [swc-project/swc](https://github.com/swc-project/swc) |
| **kellnr** | Private Rust crate registry | [kellnr/kellnr](https://github.com/kellnr/kellnr) |

### AI Evaluation & Testing

| Project | Description | URL |
|---------|-------------|-----|
| **promptfoo** | LLM prompt testing framework | [promptfoo/promptfoo](https://github.com/promptfoo/promptfoo) |
| **evals** | OpenAI evaluation framework | [openai/evals](https://github.com/openai/evals) |
| **trulens** | LLM app evaluation & tracking | [truera/trulens](https://github.com/truera/trulens) |

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

## License

This project is licensed under the [MIT License](LICENSE). See the LICENSE file for details.
