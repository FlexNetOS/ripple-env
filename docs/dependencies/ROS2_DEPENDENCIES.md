# ROS2 Package Dependencies

**Status:** Complete
**Last Updated:** 2026-01-14
**Purpose:** Document ROS2 package dependencies managed via Pixi/RoboStack

---

## Overview

ROS2 packages are managed through Pixi using the RoboStack channel, which provides conda-packaged ROS2 Humble distributions.

---

## Core ROS2 Feature

The ROS2 environment is activated via the `ros` feature in `pixi.toml`:

```bash
# Activate ROS2 environment
pixi run -e default <command>

# Or within nix develop
nix develop
```

---

## Required ROS2 Packages

### Base Packages (feature.ros)

| Package | Version | Purpose |
|---------|---------|---------|
| `ros-humble-desktop` | >=0.10.0,<0.11 | Full desktop installation including RViz, RQt |
| `rosdep` | >=0.26.0,<0.27 | Dependency resolution tool |
| `colcon-common-extensions` | >=0.3.0,<0.4 | Build tool extensions |
| `catkin_tools` | >=0.8.2,<0.10 | Legacy catkin build support |

### What ros-humble-desktop Includes

The `ros-humble-desktop` metapackage includes:

| Category | Packages |
|----------|----------|
| **Core** | ros-humble-ros-base, ros-humble-rclcpp, ros-humble-rclpy |
| **Communication** | ros-humble-std-msgs, ros-humble-geometry-msgs, ros-humble-sensor-msgs |
| **Tools** | ros-humble-ros2cli, ros-humble-ros2launch |
| **Visualization** | ros-humble-rviz2, ros-humble-rqt, ros-humble-rqt-common-plugins |
| **Simulation** | ros-humble-gazebo-ros-pkgs (optional) |
| **TF** | ros-humble-tf2, ros-humble-tf2-ros |

---

## Minimum Set for Basic Functionality

For a minimal ROS2 environment (without desktop tools):

```toml
[feature.ros-minimal.dependencies]
ros-humble-ros-base = ">=0.10.0"
rosdep = ">=0.26.0"
colcon-common-extensions = ">=0.3.0"
```

### Minimum Package List

| Package | Purpose |
|---------|---------|
| `ros-humble-ros-base` | Core ROS2 without GUI tools |
| `ros-humble-rclcpp` | C++ client library |
| `ros-humble-rclpy` | Python client library |
| `ros-humble-std-msgs` | Standard message types |

---

## Channel Configuration

```toml
[feature.ros]
platforms = ["linux-64"]
channels = ["robostack-humble", "conda-forge"]
```

**Channel Priority:**
1. `robostack-humble` - ROS2 Humble packages (highest priority)
2. `conda-forge` - General dependencies

---

## Adding ROS2 Packages

### Via Pixi

```bash
# Add a ROS2 package
pixi add ros-humble-nav2-bringup

# Add to specific feature
pixi add --feature ros ros-humble-moveit
```

### Common Package Categories

| Category | Example Packages |
|----------|------------------|
| Navigation | `ros-humble-nav2-bringup`, `ros-humble-navigation2` |
| Manipulation | `ros-humble-moveit`, `ros-humble-moveit-ros-planning` |
| Perception | `ros-humble-vision-opencv`, `ros-humble-image-pipeline` |
| Simulation | `ros-humble-gazebo-ros-pkgs`, `ros-humble-ros2-control` |
| Drivers | `ros-humble-usb-cam`, `ros-humble-velodyne` |

---

## Environment Isolation

### Feature Solve Groups

ROS2 and CUDA environments are isolated to avoid conflicts:

```toml
[environments]
default = { features = ["ros"], solve-group = "default" }
cuda = { features = ["cuda"], solve-group = "cuda" }
```

**Important:** Do not combine `ros` and `cuda` features in the same solve-group due to incompatible Python/CUDA expectations.

---

## Python Version Constraints

| Environment | Python Version | Notes |
|-------------|----------------|-------|
| ROS2 (ros) | >=3.11,<3.13 | RoboStack compatible |
| AIOS | >=3.11,<3.12 | Strict AIOS requirement |

---

## Verification

### Check ROS2 Installation

```bash
# Enter environment
pixi shell

# Verify ROS2
ros2 --version
ros2 doctor

# List installed packages
ros2 pkg list

# Check environment
printenv | grep ROS
```

### Expected Environment Variables

```bash
ROS_DISTRO=humble
ROS_VERSION=2
AMENT_PREFIX_PATH=/path/to/pixi/.pixi/envs/default
```

---

## Troubleshooting

### Package Not Found

```bash
# Search for package
pixi search ros-humble-<name>

# Update channel cache
pixi update
```

### Version Conflicts

```bash
# Check solve conflicts
pixi info

# Force reinstall
pixi clean && pixi install
```

### Missing System Dependencies

```bash
# Initialize rosdep
rosdep init
rosdep update

# Install dependencies for a package
rosdep install --from-paths src --ignore-src -y
```

---

## References

- [RoboStack](https://robostack.github.io/) - ROS packages for conda
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Pixi Documentation](https://pixi.sh/)
- [docs/CONFLICTS.md](../CONFLICTS.md) - Version coupling information

