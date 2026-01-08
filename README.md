# Riple Environment and Binary Creation

- Uses the original environment template setup from "ros2-humble-env" to create a wsl2 distro.
- A reproducible and declarative development environment for ros2 humble using nix flakes and pixi for cross-platform compatibility.
- This repository is meant to be used as a "template" repository for robotics projects to offer an easy starting environment with a working ros2 install.

## Quick Start

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

- A single script run from Windows PowerShell that checks for Linux and WSL2 install, then installs and updates them as needed.
- Creates the NixOS Distro, registers it, creates the ext4.vhdx hard disk image 1TB, and swap image.
- Loads the nix flake and configurations per direnv
- Sets up pixi package manager, tools, and packages
- Adds zsh and nushell (bash stays default with nix)
- Uses nom instead of nix
- Installs git and gh cli

## Overview

This repository provides a complete development setup for ros2 humble with all necessary build tools and dependencies pre-configured. The environment works on both Linux and macOS (x86 and arm64) using:

- **nix flakes**: for reproducible, declarative environment setup
- **pixi**: for python and ros package management via robostack
- **direnv**: for automatic environment activation
- **home-manager modules**: for comprehensive shell and editor configuration

## Getting Started

### Prerequisites

- Nix with flakes enabled
- Git >= 2.19.0

### Bootstrap Script (Recommended)

The easiest way to get started is using the bootstrap script:

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
├── bootstrap.sh           # End-to-end setup script
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
- **ROS**: ros-humble-desktop with all core ros packages
- **Build tools**: cmake, ninja, make, compilers, pkg-config
- **ROS tools**: colcon, rosdep, catkin_tools
- **Python**: 3.11.x with development headers

### Development Environment
- **Shells**: bash, zsh, nushell (with starship prompt)
- **Editor**: helix with LSPs for Python, C++, CMake, Nix, YAML, XML
- **Navigation**: zoxide (smart cd), fzf (fuzzy finder)
- **Utilities**: bat, eza, ripgrep, fd, jq, yq

### Platforms
- linux-64, linux-aarch64, osx-64, osx-arm64

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
        ros2-humble-env.homeManagerModules.default
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

## Links

- [RoboStack ROS2-humble packages](https://robostack.github.io/humble.html)
- [Pixi documentation](https://pixi.sh)
- [Nix flakes documentation](https://nixos.wiki/wiki/Flakes)
- [ROS2 Humble documentation](https://docs.ros.org/en/humble/)
- [Flake-parts devshell documentation](https://flake.parts/options/devshell.html)
- [Home-manager documentation](https://nix-community.github.io/home-manager/)

## Configuration Sources

This configuration incorporates patterns and modules from:
- [GustavoWidman/nix](https://github.com/GustavoWidman/nix) - Multi-machine Nix configuration
- [RGBCube/ncc](https://github.com/RGBCube/ncc) - Comprehensive NixOS/Darwin configuration

## License

This project is licensed under the [MIT License](LICENSE). See the LICENSE file for details.
