#!/usr/bin/env python3
"""
ROS2 Message Throughput Benchmark

Measures message throughput for ROS2 topic communication.
Tests how many messages per second can be transmitted.

Usage:
    pytest test/benchmarks/message_throughput.py -v
    pytest test/benchmarks/message_throughput.py --benchmark-only

Requirements:
    - pytest
    - pytest-benchmark
    - ROS2 environment sourced
"""

import os
import sys
import time
import threading
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Int64
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

import pytest


pytestmark = pytest.mark.skipif(
    not HAS_ROS2,
    reason="ROS2 not available in environment"
)


class ThroughputPublisher(Node):
    """High-frequency publisher for throughput testing."""

    def __init__(self, topic: str = 'throughput_test'):
        super().__init__('throughput_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.publisher = self.create_publisher(Int64, topic, qos)
        self.message_count = 0

    def publish_burst(self, count: int) -> float:
        """Publish a burst of messages and return time taken."""
        start = time.perf_counter()

        for i in range(count):
            msg = Int64()
            msg.data = i
            self.publisher.publish(msg)
            self.message_count += 1

        end = time.perf_counter()
        return end - start


class ThroughputSubscriber(Node):
    """Subscriber that counts received messages."""

    def __init__(self, topic: str = 'throughput_test'):
        super().__init__('throughput_subscriber')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        self.subscription = self.create_subscription(
            Int64,
            topic,
            self.callback,
            qos
        )

        self.received_count = 0
        self.first_receive_time: Optional[float] = None
        self.last_receive_time: Optional[float] = None

    def callback(self, msg):
        now = time.perf_counter()
        if self.first_receive_time is None:
            self.first_receive_time = now
        self.last_receive_time = now
        self.received_count += 1

    def get_throughput(self) -> float:
        """Calculate messages per second."""
        if self.first_receive_time is None or self.last_receive_time is None:
            return 0.0
        if self.received_count < 2:
            return 0.0

        duration = self.last_receive_time - self.first_receive_time
        if duration <= 0:
            return 0.0

        return self.received_count / duration


@pytest.fixture(scope="module")
def ros2_context():
    """Initialize ROS2 context."""
    if not HAS_ROS2:
        pytest.skip("ROS2 not available")

    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def throughput_nodes(ros2_context):
    """Create publisher and subscriber nodes."""
    pub = ThroughputPublisher()
    sub = ThroughputSubscriber()
    yield pub, sub
    pub.destroy_node()
    sub.destroy_node()


class TestMessageThroughput:
    """Message throughput benchmarks."""

    def test_publish_throughput(self, throughput_nodes, benchmark):
        """Benchmark publishing throughput."""
        pub, sub = throughput_nodes

        def publish_1000():
            return pub.publish_burst(1000)

        duration = benchmark(publish_1000)
        throughput = 1000 / duration if duration > 0 else 0

        print(f"\nPublish throughput: {throughput:.0f} msg/s")

    def test_roundtrip_throughput(self, throughput_nodes):
        """Measure end-to-end throughput with subscriber."""
        pub, sub = throughput_nodes
        message_count = 10000

        # Start subscriber spinning in background
        spin_complete = threading.Event()

        def spin_subscriber():
            while not spin_complete.is_set():
                rclpy.spin_once(sub, timeout_sec=0.001)

        spin_thread = threading.Thread(target=spin_subscriber)
        spin_thread.start()

        # Publish messages
        start = time.perf_counter()
        pub.publish_burst(message_count)

        # Wait for messages to arrive
        time.sleep(0.5)
        spin_complete.set()
        spin_thread.join()

        end = time.perf_counter()
        duration = end - start

        received = sub.received_count
        pub_throughput = message_count / duration
        recv_throughput = received / duration if duration > 0 else 0
        loss_rate = (message_count - received) / message_count * 100

        print(f"\nRoundtrip Throughput Results ({message_count} messages):")
        print(f"  Duration: {duration:.3f} s")
        print(f"  Published: {message_count}")
        print(f"  Received: {received}")
        print(f"  Publish rate: {pub_throughput:.0f} msg/s")
        print(f"  Receive rate: {recv_throughput:.0f} msg/s")
        print(f"  Message loss: {loss_rate:.2f}%")

        # With BEST_EFFORT QoS, some loss is acceptable
        assert loss_rate < 50, f"Excessive message loss: {loss_rate:.1f}%"


class TestThroughputWithoutROS2:
    """Fallback tests when ROS2 is not available."""

    @pytest.mark.skipif(HAS_ROS2, reason="ROS2 is available")
    def test_baseline_throughput(self, benchmark):
        """Baseline throughput test without ROS2."""
        messages = []

        def simulate_publish():
            messages.clear()
            for i in range(1000):
                messages.append({"data": i, "timestamp": time.time()})
            return len(messages)

        count = benchmark(simulate_publish)
        assert count == 1000


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
