---
title: First Steps
description: Your first steps with ripple-env
tags:
  - getting-started
  - tutorial
---

# First Steps

After installing ripple-env, follow these steps to build your first ROS2 package.

## 1. Enter the Development Shell

```bash
cd ripple-env
nix develop
```

This will load all development tools including ROS2 Humble, Python, and build tools.

## 2. Verify the Environment

```bash
# Check ROS2
ros2 --version

# Check Python
python --version

# Check colcon
colcon --version
```

## 3. Build Existing Packages

```bash
# Build all packages
cb    # alias for: colcon build --symlink-install

# Source the setup file
source install/setup.bash
```

## 4. Run Tests

```bash
# Run all tests
ct    # alias for: colcon test

# View results
colcon test-result --verbose
```

## 5. Create a New Package

```bash
cd src

# Create a Python package
ros2 pkg create --build-type ament_python my_package

# Or create a C++ package
ros2 pkg create --build-type ament_cmake my_cpp_package
```

## 6. Build Your Package

```bash
cd ..
colcon build --packages-select my_package
source install/setup.bash
```

## Next Steps

- Read the [Python Environments](../development/python-environments.md) guide
- Explore [AI Integration](../ai-ml/index.md)
- Set up [Observability](../infrastructure/observability.md)
