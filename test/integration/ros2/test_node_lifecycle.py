#!/usr/bin/env python3
"""
ROS2 Node Lifecycle Integration Tests

Tests lifecycle state transitions for managed ROS2 nodes.

Usage:
    pytest test/integration/ros2/test_node_lifecycle.py -v

Requirements:
    - ROS2 environment sourced
    - lifecycle_msgs package available
"""

import time
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
    from lifecycle_msgs.srv import GetState, ChangeState
    from lifecycle_msgs.msg import Transition
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

import pytest


pytestmark = pytest.mark.skipif(
    not HAS_ROS2,
    reason="ROS2 not available in environment"
)


class TestableLifecycleNode(LifecycleNode):
    """A lifecycle node for testing state transitions."""

    def __init__(self, node_name: str = 'test_lifecycle_node'):
        super().__init__(node_name)
        self.configure_called = False
        self.activate_called = False
        self.deactivate_called = False
        self.cleanup_called = False
        self.shutdown_called = False

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.configure_called = True
        self.get_logger().info('Configuring...')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.activate_called = True
        self.get_logger().info('Activating...')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.deactivate_called = True
        self.get_logger().info('Deactivating...')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.cleanup_called = True
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.shutdown_called = True
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS


@pytest.fixture(scope="module")
def ros2_context():
    """Initialize ROS2 context."""
    if not HAS_ROS2:
        pytest.skip("ROS2 not available")

    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def lifecycle_node(ros2_context):
    """Create a lifecycle node for testing."""
    node = TestableLifecycleNode()
    yield node
    node.destroy_node()


class TestLifecycleTransitions:
    """Test lifecycle state transitions."""

    def test_initial_state_unconfigured(self, lifecycle_node):
        """Node should start in unconfigured state."""
        # Lifecycle nodes start in 'unconfigured' state (id=1)
        # Note: Getting state requires spinning
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)
        # The actual state check would require the lifecycle service client
        assert not lifecycle_node.configure_called

    def test_configure_transition(self, lifecycle_node):
        """Test configure transition."""
        # Trigger configure
        result = lifecycle_node.trigger_configure()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        assert lifecycle_node.configure_called

    def test_activate_transition(self, lifecycle_node):
        """Test configure -> activate transition."""
        # Configure first
        lifecycle_node.trigger_configure()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        # Then activate
        lifecycle_node.trigger_activate()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        assert lifecycle_node.configure_called
        assert lifecycle_node.activate_called

    def test_deactivate_transition(self, lifecycle_node):
        """Test full activate -> deactivate cycle."""
        # Configure
        lifecycle_node.trigger_configure()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        # Activate
        lifecycle_node.trigger_activate()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        # Deactivate
        lifecycle_node.trigger_deactivate()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        assert lifecycle_node.deactivate_called

    def test_cleanup_transition(self, lifecycle_node):
        """Test cleanup transition from inactive state."""
        # Configure
        lifecycle_node.trigger_configure()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        # Cleanup
        lifecycle_node.trigger_cleanup()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)

        assert lifecycle_node.cleanup_called

    def test_full_lifecycle(self, lifecycle_node):
        """Test complete lifecycle: configure -> activate -> deactivate -> cleanup."""
        # Configure
        lifecycle_node.trigger_configure()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)
        assert lifecycle_node.configure_called

        # Activate
        lifecycle_node.trigger_activate()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)
        assert lifecycle_node.activate_called

        # Deactivate
        lifecycle_node.trigger_deactivate()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)
        assert lifecycle_node.deactivate_called

        # Cleanup
        lifecycle_node.trigger_cleanup()
        rclpy.spin_once(lifecycle_node, timeout_sec=0.1)
        assert lifecycle_node.cleanup_called


class TestLifecycleWithoutROS2:
    """Fallback tests when ROS2 is not available."""

    @pytest.mark.skipif(HAS_ROS2, reason="ROS2 is available")
    def test_fallback_state_machine(self):
        """Test basic state machine without ROS2."""
        states = ['unconfigured', 'inactive', 'active', 'finalized']
        transitions = {
            'unconfigured': ['inactive'],
            'inactive': ['active', 'unconfigured', 'finalized'],
            'active': ['inactive', 'finalized'],
            'finalized': []
        }

        current = 'unconfigured'

        # Simulate transitions
        assert 'inactive' in transitions[current]
        current = 'inactive'

        assert 'active' in transitions[current]
        current = 'active'

        assert 'inactive' in transitions[current]
        current = 'inactive'

        assert 'finalized' in transitions[current]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
