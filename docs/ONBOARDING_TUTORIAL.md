# Interactive Onboarding Tutorial

Welcome to the ripple-env development environment! This tutorial guides you through setup and first steps with interactive checkpoints.

## How This Tutorial Works

- Work through each section in order
- Use the checkboxes to track your progress
- Each section builds on the previous one
- Estimated total time: 30-45 minutes

---

## Level 1: Environment Setup (10 minutes)

### Prerequisites Check

Before starting, verify you have:

- [ ] Operating system: Linux, macOS 12+, or Windows 11 with WSL2
- [ ] 8GB RAM minimum (16GB recommended)
- [ ] 20GB free disk space
- [ ] Internet connection

### Step 1.1: Install Nix

Nix is the foundation of our reproducible development environment.

**Linux/macOS:**
```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
```

**Windows (PowerShell as Administrator):**
```powershell
Invoke-WebRequest -Uri "https://raw.githubusercontent.com/FlexNetOS/ripple-env/main/bootstrap.ps1" -OutFile "bootstrap.ps1"
.\bootstrap.ps1
```

**Checkpoint:**
```bash
nix --version
```
- [ ] Nix version is displayed (e.g., `nix (Nix) 2.x.x`)

### Step 1.2: Clone the Repository

```bash
git clone https://github.com/FlexNetOS/ripple-env.git
cd ripple-env
```

**Checkpoint:**
```bash
ls flake.nix
```
- [ ] `flake.nix` file exists

### Step 1.3: Enter Development Shell

```bash
nix develop
```

> **Note:** First run downloads packages and may take 5-15 minutes. Subsequent runs are instant.

**Checkpoint:**
```bash
echo $IN_NIX_SHELL
```
- [ ] Output shows `pure` or `impure`

---

## Level 2: Understanding the Environment (5 minutes)

### Step 2.1: Explore Available Tools

Inside the development shell, check what's available:

```bash
# Check ROS2
ros2 --version

# Check Python
python --version

# Check Pixi
pixi --version

# Check available commands
type colcon
```

**Checkpoint:**
- [ ] All four commands work without errors

### Step 2.2: Understand the Shell Options

The environment provides different shells for different needs:

| Shell | Command | Use Case |
|-------|---------|----------|
| Default | `nix develop` | Daily development (recommended) |
| Full | `nix develop .#full` | CI/CD and extra tooling |
| CUDA | `nix develop .#cuda` | GPU/ML workloads (Linux only) |
| Identity | `nix develop .#identity` | Minimal base tools |

**Checkpoint:**
```bash
# Try switching shells (exit current first)
exit
nix develop .#full
echo "In full shell"
exit
nix develop  # Back to default
```
- [ ] Successfully switched between shells

---

## Level 3: Your First ROS2 Project (10 minutes)

### Step 3.1: Create a Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_tutorial_ws/src
cd ~/ros2_tutorial_ws
```

**Checkpoint:**
- [ ] Directory `~/ros2_tutorial_ws/src` exists

### Step 3.2: Create a Python Package

```bash
cd src
ros2 pkg create --build-type ament_python hello_robot --dependencies rclpy std_msgs
cd ..
```

**Checkpoint:**
```bash
ls src/hello_robot/
```
- [ ] Package files are visible (package.xml, setup.py, etc.)

### Step 3.3: Build the Package

```bash
colcon build --symlink-install
source install/setup.bash
```

**Checkpoint:**
```bash
ros2 pkg list | grep hello_robot
```
- [ ] `hello_robot` appears in package list

### Step 3.4: Create a Simple Node

Edit `src/hello_robot/hello_robot/hello_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.publisher = self.create_publisher(String, 'greetings', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Hello Robot Node started!')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from ROS2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `src/hello_robot/setup.py` entry_points:
```python
entry_points={
    'console_scripts': [
        'hello_node = hello_robot.hello_node:main',
    ],
},
```

Rebuild and run:
```bash
colcon build --packages-select hello_robot
source install/setup.bash
ros2 run hello_robot hello_node
```

**Checkpoint:**
- [ ] Node prints "Hello from ROS2!" messages

---

## Level 4: Using Pixi for Packages (5 minutes)

### Step 4.1: Understand Pixi

Pixi manages Python/ROS2 packages via conda-forge. Check current packages:

```bash
pixi list
```

### Step 4.2: Add a Package

```bash
# Add numpy as an example
pixi add numpy

# Verify
pixi run python -c "import numpy; print(numpy.__version__)"
```

**Checkpoint:**
- [ ] NumPy version is displayed

### Step 4.3: Add a ROS2 Package

```bash
# Add navigation2 (example)
pixi add ros-humble-nav2-msgs

# Verify
pixi run python -c "from nav2_msgs.msg import SpeedLimit; print('nav2_msgs loaded')"
```

**Checkpoint:**
- [ ] `nav2_msgs loaded` is displayed

---

## Level 5: Docker Services (5 minutes)

### Step 5.1: Check Docker

```bash
docker --version
docker compose version
```

**Checkpoint:**
- [ ] Both commands show versions

### Step 5.2: View Available Services

```bash
ls docker/docker-compose.*.yml
```

Key services:
- `docker-compose.observability.yml` - Prometheus, Grafana monitoring
- `docker-compose.localai.yml` - Local LLM inference
- `docker-compose.lightweight.yml` - Minimal stack for low-resource systems

### Step 5.3: Start a Service (Optional)

```bash
# Create required network
docker network create agentic-network 2>/dev/null || true

# Start lightweight observability
docker compose -f docker/docker-compose.lightweight.yml up -d

# Check status
docker compose -f docker/docker-compose.lightweight.yml ps

# Stop when done
docker compose -f docker/docker-compose.lightweight.yml down
```

**Checkpoint:**
- [ ] Services started and stopped successfully

---

## Level 6: Nix Basics (5 minutes)

### Step 6.1: Understanding the Flake

The `flake.nix` defines the entire development environment:

```bash
# View flake outputs
nix flake show

# Check flake health
nix flake check
```

### Step 6.2: Key Flake Concepts

| Concept | Description | File |
|---------|-------------|------|
| Inputs | External dependencies (nixpkgs, etc.) | `flake.nix` inputs section |
| Outputs | What the flake provides (shells, packages) | `flake.nix` outputs section |
| DevShells | Development environments | `nix/shells/default.nix` |
| Packages | Package collections | `nix/packages/default.nix` |

### Step 6.3: Making Changes

To add a package to the environment:

1. Edit `flake.nix` or the appropriate module
2. Rebuild: `nix develop --rebuild`
3. Verify the package is available

**Checkpoint:**
```bash
nix flake show 2>&1 | head -20
```
- [ ] Flake outputs are displayed

---

## Completion Checklist

Review your progress:

### Environment
- [ ] Nix installed and working
- [ ] Repository cloned
- [ ] Development shell accessible
- [ ] Understand shell options

### ROS2
- [ ] Created a workspace
- [ ] Built a package
- [ ] Ran a node
- [ ] Understood message publishing

### Tools
- [ ] Used Pixi to add packages
- [ ] Explored Docker services
- [ ] Understood flake structure

---

## What's Next?

Now that you've completed the basics, explore:

1. **[Progressive Examples](PROGRESSIVE_EXAMPLES.md)** - More complex scenarios
2. **[Common Pitfalls](COMMON_PITFALLS.md)** - Avoid common mistakes
3. **[Video Walkthroughs](VIDEO_WALKTHROUGHS.md)** - Visual learning resources
4. **[Troubleshooting](TROUBLESHOOTING.md)** - When things go wrong

### Recommended Learning Path

```
Beginner              Intermediate              Advanced
   |                       |                       |
   v                       v                       v
[This Tutorial]  -->  [ROS2 Tutorials]  -->  [Custom Nodes]
                             |                     |
                             v                     v
                      [Docker Services]  -->  [Deployment]
                             |                     |
                             v                     v
                      [Nix Customization] --> [Flake Modules]
```

---

## Quick Reference Card

```bash
# Enter development environment
nix develop

# Build ROS2 packages
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Add Python/ROS2 packages
pixi add <package-name>

# Update environment
git pull && nix develop --rebuild

# Start Docker services
docker compose -f docker/docker-compose.<name>.yml up -d

# Check flake
nix flake check
```

---

## Getting Help

- **Stuck?** Check [Troubleshooting](TROUBLESHOOTING.md)
- **Bug?** [Open an issue](https://github.com/FlexNetOS/ripple-env/issues)
- **Question?** [Start a discussion](https://github.com/FlexNetOS/ripple-env/discussions)
