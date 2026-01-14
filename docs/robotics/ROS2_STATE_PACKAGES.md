# ROS2 State & Storage Packages Installation Method

**Status**: Defined  
**Date**: 2026-01-09  
**Priority**: P0 (Blocking)  

## Overview

This document defines the installation method for ROS2 State & Storage packages:
- `ros2/rclpy` - ROS2 Python client library
- `ros2/rcpp` - ROS2 C++ client library (rclcpp)
- `ros2/rosbag2` - ROS2 bag recording and playback

## Installation Method: Pixi + RoboStack

### Rationale

1. **RoboStack Integration**: These packages are part of the ROS2 Humble distribution provided by RoboStack
2. **Cross-Platform**: Pixi/Conda provides consistent installation across Linux, macOS, and ARM64
3. **Python Compatibility**: Aligns with Python 3.11 requirement for ROS2 Humble
4. **Dependency Management**: Conda handles complex ROS2 dependencies automatically

### Implementation

These packages are already included via the `ros-humble-desktop` meta-package in `pixi.toml`:

```toml
[dependencies]
python = ">=3.11,<3.12"
ros-humble-desktop = ">=0.10.0,<0.11"
```

The `ros-humble-desktop` package includes:
- `rclpy` - Python client library
- `rclcpp` - C++ client library
- `rosbag2` - Bag recording/playback
- All core ROS2 packages

### Verification Commands

```bash
# Activate Pixi environment
pixi shell

# Verify rclpy
python -c "import rclpy; print(f'rclpy version: {rclpy.__version__}')"

# Verify rclcpp (via ros2 CLI)
ros2 pkg list | grep rclcpp

# Verify rosbag2
ros2 bag --help

# List all ROS2 packages
ros2 pkg list

# Check ROS2 environment
ros2 doctor
```

### Additional State Packages (Optional)

For advanced state management, consider adding:

```toml
[dependencies]
# State persistence
ros-humble-sqlite3-vendor = ">=0.15,<0.16"

# Parameter management
ros-humble-rclpy-parameter = ">=0.10,<0.11"

# Lifecycle management
ros-humble-lifecycle = ">=0.20,<0.21"
```

## Storage Configuration

### ROS Bag Storage

Default storage location for rosbag2:

```bash
# Set via environment variable
export ROS_BAG_STORAGE_PATH=./data/rosbags

# Or in launch file
<param name="storage_config_uri" value="$(env ROS_BAG_STORAGE_PATH)"/>
```

### Storage Backends

RoboStack provides multiple storage backends:

1. **SQLite3** (default) - Good for small to medium bags
2. **MCAP** - Better compression and performance
3. **ROS1 Bag** - For ROS1 compatibility

Install additional backends via Pixi:

```toml
[dependencies]
# MCAP storage plugin
ros-humble-rosbag2-storage-mcap = ">=0.15,<0.16"

# ROS1 bag compatibility
ros-humble-rosbag2-bag-v2-plugins = ">=0.15,<0.16"
```

## Integration with Docker

For containerized ROS2 applications, mount the Pixi environment:

```yaml
# docker-compose.ros2.yml
services:
  ros2-node:
    image: ubuntu:22.04
    volumes:
      - ./.pixi/envs/default:/opt/ros2:ro
      - ./data/rosbags:/rosbags
    environment:
      - AMENT_PREFIX_PATH=/opt/ros2
      - PYTHONPATH=/opt/ros2/lib/python3.11/site-packages
      - ROS_BAG_STORAGE_PATH=/rosbags
```

## Performance Optimization

### rosbag2 Optimization

```bash
# Increase cache size for better write performance
ros2 bag record -a --cache-size 10000000

# Use MCAP for better compression
ros2 bag record -a --storage mcap

# Split bags by size
ros2 bag record -a --max-bag-size 1000000000  # 1GB
```

### State Persistence

For high-frequency state updates, consider:

1. **In-memory caching** with periodic persistence
2. **Write-ahead logging** for crash recovery
3. **Compression** for storage efficiency

## CI/CD Integration

### GitHub Actions Workflow

```yaml
name: ROS2 State Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install Pixi
        run: curl -fsSL https://pixi.sh/install.sh | bash
      
      - name: Install dependencies
        run: pixi install
      
      - name: Test rclpy
        run: pixi run python -c "import rclpy; rclpy.init()"
      
      - name: Test rosbag2
        run: pixi run ros2 bag --help
      
      - name: Run ROS2 tests
        run: pixi run colcon test --packages-select your_package
```

## Troubleshooting

### Common Issues

1. **Import Error: No module named 'rclpy'**
   - Solution: Ensure Pixi environment is activated: `pixi shell`

2. **rosbag2: command not found**
   - Solution: Source ROS2 setup: `source .pixi/envs/default/setup.bash`

3. **Storage plugin not found**
   - Solution: Install storage plugin: `pixi add ros-humble-rosbag2-storage-mcap`

### Debug Commands

```bash
# Check ROS2 installation
pixi run ros2 doctor

# List available storage plugins
pixi run ros2 bag info --storage-config-file

# Check Python path
pixi run python -c "import sys; print('\n'.join(sys.path))"

# Verify ROS2 environment
pixi run env | grep ROS
```

## References

- [RoboStack Documentation](https://robostack.github.io/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [rosbag2 Design](https://github.com/ros2/rosbag2/blob/humble/README.md)
- [Pixi Documentation](https://prefix.dev/docs/pixi/overview)

## Status

- [x] Installation method defined (Pixi + RoboStack)
- [x] Verification commands documented
- [x] Storage configuration specified
- [x] CI/CD integration provided
- [ ] Actual installation verified (requires `pixi install`)
- [ ] Performance benchmarks completed

## Next Steps

1. Run `pixi install` to install all dependencies
2. Execute verification commands
3. Set up rosbag2 storage directory
4. Configure CI/CD pipeline
5. Benchmark storage performance
