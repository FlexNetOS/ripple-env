---
name: build
description: Build ROS2 packages with colcon
---

Build ROS2 packages in the current workspace.

## Usage

Run `colcon build --symlink-install` to build all packages with symlink install for development.

## Steps

1. Ensure you're in the development shell (`nom develop` or `direnv allow`)
2. Run the build command
3. Source the workspace if needed: `source install/setup.bash`

## Options

- Build all: `colcon build --symlink-install`
- Build specific: `colcon build --packages-select <pkg>`
- Build with deps: `colcon build --packages-up-to <pkg>`
- Clean build: Remove `build/`, `install/`, `log/` directories first
