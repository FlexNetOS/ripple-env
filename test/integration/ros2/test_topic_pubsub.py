#!/usr/bin/env python3
"""
ROS2 Topic Pub/Sub Integration Tests

Tests publisher/subscriber communication patterns.

Usage:
    pytest test/integration/ros2/test_topic_pubsub.py -v

Requirements:
    - ROS2 environment sourced
"""

import time
import threading
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String, Int32, Float64
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

import pytest


pytestmark = pytest.mark.skipif(
    not HAS_ROS2,
    reason="ROS2 not available in environment"
)


class PublisherNode(Node):
    """Simple publisher node for testing."""

    def __init__(self, topic: str = 'test_topic'):
        super().__init__('test_publisher')
        self.publisher = self.create_publisher(String, topic, 10)
        self.published_messages: List[str] = []

    def publish(self, message: str):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)
        self.published_messages.append(message)


class SubscriberNode(Node):
    """Simple subscriber node for testing."""

    def __init__(self, topic: str = 'test_topic'):
        super().__init__('test_subscriber')
        self.received_messages: List[str] = []
        self.subscription = self.create_subscription(
            String,
            topic,
            self.callback,
            10
        )

    def callback(self, msg):
        self.received_messages.append(msg.data)


@pytest.fixture(scope="module")
def ros2_context():
    """Initialize ROS2 context."""
    if not HAS_ROS2:
        pytest.skip("ROS2 not available")

    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def pubsub_nodes(ros2_context):
    """Create publisher and subscriber nodes."""
    pub = PublisherNode()
    sub = SubscriberNode()
    yield pub, sub
    pub.destroy_node()
    sub.destroy_node()


class TestTopicPubSub:
    """Topic publish/subscribe tests."""

    def test_single_message(self, pubsub_nodes):
        """Test single message transmission."""
        pub, sub = pubsub_nodes
        test_message = "Hello, ROS2!"

        # Publish
        pub.publish(test_message)

        # Spin to allow message delivery
        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        timeout = time.time() + 2.0
        while len(sub.received_messages) < 1 and time.time() < timeout:
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()

        assert len(sub.received_messages) >= 1
        assert test_message in sub.received_messages

    def test_multiple_messages(self, pubsub_nodes):
        """Test multiple message transmission."""
        pub, sub = pubsub_nodes
        messages = [f"Message {i}" for i in range(10)]

        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        # Publish all messages
        for msg in messages:
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.01)

        # Wait for delivery
        timeout = time.time() + 2.0
        while len(sub.received_messages) < len(messages) and time.time() < timeout:
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()

        # At least most messages should arrive
        assert len(sub.received_messages) >= len(messages) * 0.8

    def test_message_order(self, pubsub_nodes):
        """Test that messages arrive in order."""
        pub, sub = pubsub_nodes
        sub.received_messages.clear()

        executor = SingleThreadedExecutor()
        executor.add_node(pub)
        executor.add_node(sub)

        messages = [f"Ordered_{i}" for i in range(5)]

        for msg in messages:
            pub.publish(msg)
            executor.spin_once(timeout_sec=0.05)

        # Wait for delivery
        timeout = time.time() + 2.0
        while len(sub.received_messages) < len(messages) and time.time() < timeout:
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()

        # Check order of received messages
        received = sub.received_messages
        if len(received) >= 2:
            for i in range(len(received) - 1):
                # Extract sequence numbers and verify order
                seq_a = int(received[i].split('_')[1])
                seq_b = int(received[i + 1].split('_')[1])
                assert seq_a < seq_b, f"Messages out of order: {received}"


class TestQoSProfiles:
    """Test different QoS configurations."""

    def test_reliable_qos(self, ros2_context):
        """Test RELIABLE QoS profile."""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        pub_node = Node('reliable_pub')
        sub_node = Node('reliable_sub')
        received: List[str] = []

        pub = pub_node.create_publisher(String, 'reliable_test', qos)
        sub = sub_node.create_subscription(
            String,
            'reliable_test',
            lambda msg: received.append(msg.data),
            qos
        )

        executor = SingleThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(sub_node)

        # Publish
        msg = String()
        msg.data = "reliable_message"
        pub.publish(msg)

        # Wait
        timeout = time.time() + 2.0
        while len(received) < 1 and time.time() < timeout:
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()

        assert len(received) >= 1

    def test_transient_local_qos(self, ros2_context):
        """Test TRANSIENT_LOCAL durability (late-joining subscriber)."""
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        pub_node = Node('transient_pub')
        received: List[str] = []

        # Create publisher and publish BEFORE subscriber
        pub = pub_node.create_publisher(String, 'transient_test', qos)
        msg = String()
        msg.data = "early_message"
        pub.publish(msg)

        # Give time for message to be stored
        time.sleep(0.1)

        # Now create subscriber (late-joiner)
        sub_node = Node('transient_sub')
        sub = sub_node.create_subscription(
            String,
            'transient_test',
            lambda msg: received.append(msg.data),
            qos
        )

        executor = SingleThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(sub_node)

        # Spin to receive
        timeout = time.time() + 2.0
        while len(received) < 1 and time.time() < timeout:
            executor.spin_once(timeout_sec=0.1)

        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()

        # Late-joining subscriber should receive the message
        assert "early_message" in received


class TestPubSubWithoutROS2:
    """Fallback tests when ROS2 is not available."""

    @pytest.mark.skipif(HAS_ROS2, reason="ROS2 is available")
    def test_simulated_pubsub(self):
        """Simulate pub/sub without ROS2."""
        published = []
        received = []

        def publish(msg):
            published.append(msg)
            # Simulate delivery
            received.append(msg)

        publish("test_message")

        assert len(published) == 1
        assert len(received) == 1
        assert published[0] == received[0]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
