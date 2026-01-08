---
name: test
description: Run ROS2 package tests with colcon
---

Run tests for ROS2 packages in the current workspace.

## Usage

Run `colcon test` followed by `colcon test-result --verbose` to see results.

## Steps

1. Ensure packages are built first (`/build`)
2. Run `colcon test`
3. View results with `colcon test-result --verbose`

## Options

- Test all: `colcon test`
- Test specific: `colcon test --packages-select <pkg>`
- Rerun failed: `colcon test --packages-select-test-failures`
