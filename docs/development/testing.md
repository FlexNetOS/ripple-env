---
title: Testing
description: Running and writing tests
tags:
  - testing
  - development
---

# Testing

This guide covers testing strategies and tools available in ripple-env.

## Running Tests

### ROS2 Package Tests

```bash
# Run all tests
ct    # alias for: colcon test

# Run tests for specific package
colcon test --packages-select my_package

# View test results
colcon test-result --verbose
```

### Python Tests

```bash
# Run pytest
pytest tests/

# With coverage
pytest --cov=src tests/

# Verbose output
pytest -v tests/
```

## Test Types

### Unit Tests

Unit tests verify individual functions and classes:

```python
import pytest

def test_addition():
    assert 1 + 1 == 2

def test_ros2_message():
    from std_msgs.msg import String
    msg = String()
    msg.data = "test"
    assert msg.data == "test"
```

### Integration Tests

Integration tests verify component interactions:

```python
import rclpy
from rclpy.node import Node

def test_node_creation():
    rclpy.init()
    node = Node('test_node')
    assert node.get_name() == 'test_node'
    node.destroy_node()
    rclpy.shutdown()
```

### Launch Tests

Launch tests verify launch file behavior:

```python
import launch_testing
import pytest
import unittest

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        # Your launch actions
    ])

class TestLaunch(unittest.TestCase):
    def test_node_started(self, proc_info):
        proc_info.assertWaitForStartup(timeout=10)
```

## CI Integration

Tests run automatically in GitHub Actions:

```yaml
- name: Run tests
  run: |
    nix develop --command bash -c "colcon test"
    nix develop --command bash -c "colcon test-result --verbose"
```

## Best Practices

1. **Write tests first** - Use TDD when possible
2. **Keep tests isolated** - Each test should be independent
3. **Use fixtures** - Share setup code with pytest fixtures
4. **Test edge cases** - Include boundary conditions
5. **Mock external services** - Use mocks for network calls
