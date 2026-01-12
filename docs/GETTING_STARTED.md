# Getting Started with ROS2 Humble Development Environment

This guide walks you through setting up and using the ROS2 Humble development environment from scratch.

## Prerequisites

Before you begin, ensure you have:

- **Operating System**: Linux (Ubuntu 20.04+), macOS (12+), or Windows 11 with WSL2
- **RAM**: 8GB minimum, 16GB recommended
- **Disk Space**: 20GB free for Nix store and packages
- **Internet**: Required for initial setup

## Quick Start (5 minutes)

### Step 1: Install Nix

```bash
# Linux/macOS - Recommended installer
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install

# Restart your shell or run:
. /nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh
```

### Step 2: Clone and Enter Environment

```bash
# Clone the repository
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env

# Enter the development shell (first run takes 5-10 minutes)
nix develop
```

### Step 3: Verify Installation

```bash
# Check ROS2 is available
ros2 --help

# Check Python environment
python --version
pixi --version
```

You're ready to develop ROS2 applications.

## Detailed Setup Guide

### Linux (Ubuntu/Debian)

1. **Install Nix**:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
   ```

2. **Install direnv** (optional but recommended):
   ```bash
   nix profile install nixpkgs#direnv
   echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Clone and setup**:
   ```bash
   git clone https://github.com/FlexNetOS/ripple-env.git
   cd ripple-env
   direnv allow  # If using direnv
   # OR
   nix develop   # Direct entry
   ```

### macOS

1. **Install Nix**:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
   ```

2. **Install direnv** (optional):
   ```bash
   nix profile install nixpkgs#direnv
   echo 'eval "$(direnv hook zsh)"' >> ~/.zshrc
   source ~/.zshrc
   ```

3. **Clone and setup**:
   ```bash
   git clone https://github.com/FlexNetOS/ripple-env.git
   cd ripple-env
   nix develop
   ```

> **Note**: ROS2 packages are primarily available on Linux. macOS provides development tools but limited ROS2 runtime support.

### Windows (WSL2)

1. **Run the bootstrap script** (as Administrator in PowerShell):
   ```powershell
   # Download and run bootstrap
   Invoke-WebRequest -Uri "https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.ps1" -OutFile "bootstrap.ps1"
   .\bootstrap.ps1
   ```

2. **Enter WSL and continue**:
   ```bash
   wsl -d NixOS-Ripple
   cd ~/ripple-env
   nix develop
   ```

## Development Shells

The environment provides multiple shells for different use cases:

```bash
# Default (minimal) development shell (recommended)
nix develop

# Full shell (extra tooling)
nix develop .#full

# CUDA-enabled shell (Linux with NVIDIA GPU)
nix develop .#cuda

# Identity shell (base system tools only)
nix develop .#identity
# Full shell (CI / extra tooling)
nix develop .#full
```

## Working with ROS2

### Create a Workspace

```bash
# Inside the development shell
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a sample package
cd src
ros2 pkg create --build-type ament_python my_robot --dependencies rclpy std_msgs
cd ..
```

### Build and Test

```bash
# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Run tests
colcon test
colcon test-result --verbose
```

### Run a Node

```bash
# In terminal 1
ros2 run my_robot my_node

# In terminal 2
ros2 topic list
ros2 topic echo /my_topic
```

## Using Pixi for Python Packages

Pixi manages Python packages alongside Nix:

```bash
# Install a package
pixi add numpy

# Run Python with package
pixi run python -c "import numpy; print(numpy.__version__)"

# Add ROS2 packages
pixi add ros-humble-navigation2
```

## Common Tasks

### Update the Environment

```bash
# Pull latest changes
git pull origin main

# Update Pixi packages
pixi update

# Rebuild Nix shell
nix develop --rebuild
```

### Add New Dependencies

**For Nix packages** (system tools, compilers):
Edit `flake.nix` and add to `commonPackages`.

**For ROS2/Python packages** (via conda-forge):
```bash
pixi add <package-name>
```

### Use Docker Services

The environment includes Docker Compose configurations for various services:

```bash
# Start observability stack (Prometheus, Grafana, etc.)
docker compose -f docker/docker-compose.observability.yml up -d

# Start LocalAI for local LLM inference
docker compose -f docker/docker-compose.localai.yml up -d

# View all available stacks
ls docker/docker-compose.*.yml
```

## Project Structure

```
ripple-env/
├── flake.nix           # Main Nix configuration
├── pixi.toml           # Python/ROS2 package definitions
├── bootstrap.sh        # Linux/macOS setup script
├── bootstrap.ps1       # Windows setup script
├── docker/             # Docker Compose files for services
├── docs/               # Documentation
├── modules/            # Home-manager modules
├── scripts/            # Utility scripts
└── src/                # ROS2 packages (your code goes here)
```

## Next Steps

- **Build a robot**: Check out [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- **Add AI capabilities**: See [INFERENCE_SETUP.md](INFERENCE_SETUP.md)
- **Deploy to edge**: See [EDGE_DEPLOYMENT.md](EDGE_DEPLOYMENT.md)
- **Set up observability**: See [OBSERVABILITY-QUICK-START.md](OBSERVABILITY-QUICK-START.md)
- **Contribute**: Read [CONTRIBUTING.md](../CONTRIBUTING.md)

## Troubleshooting

For common issues and solutions, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Getting Help

- **GitHub Issues**: [Report bugs or request features](https://github.com/FlexNetOS/ripple-env/issues)
- **Discussions**: [Ask questions](https://github.com/FlexNetOS/ripple-env/discussions)
- **ROS2 Documentation**: [docs.ros.org](https://docs.ros.org/en/humble/)
