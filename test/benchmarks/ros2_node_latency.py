#!/usr/bin/env python3
"""
ROS2 Node Latency Benchmark

Measures round-trip latency for ROS2 topic communication.
Useful for evaluating real-time performance of ROS2 nodes.

Usage:
    pytest test/benchmarks/ros2_node_latency.py -v
    pytest test/benchmarks/ros2_node_latency.py --benchmark-only

Requirements:
    - pytest
    - pytest-benchmark
    - ROS2 environment sourced
"""

import os
import sys
import time
import statistics
from typing import List, Optional

# Check if we're in a ROS2 environment
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float64
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

import pytest


# Skip all tests if ROS2 is not available
pytestmark = pytest.mark.skipif(
    not HAS_ROS2,
    reason="ROS2 not available in environment"
)


class LatencyMeasurementNode(Node):
    """Node for measuring pub-sub latency."""

    def __init__(self):
        super().__init__('latency_benchmark_node')

        # QoS profile for low-latency communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(String, 'benchmark_topic', qos)
        self.subscription = self.create_subscription(
            String,
            'benchmark_topic',
            self.listener_callback,
            qos
        )

        self.send_time: Optional[float] = None
        self.receive_time: Optional[float] = None
        self.message_received = False

    def listener_callback(self, msg):
        self.receive_time = time.perf_counter_ns()
        self.message_received = True

    def measure_latency(self) -> float:
        """Publish a message and measure round-trip time."""
        self.message_received = False
        self.send_time = time.perf_counter_ns()

        msg = String()
        msg.data = f"benchmark_{self.send_time}"
        self.publisher.publish(msg)

        # Spin until message received or timeout
        timeout = time.time() + 1.0  # 1 second timeout
        while not self.message_received and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.001)

        if self.message_received:
            latency_ns = self.receive_time - self.send_time
            return latency_ns / 1_000_000  # Convert to milliseconds
        return -1.0  # Timeout


@pytest.fixture(scope="module")
def ros2_context():
    """Initialize ROS2 context for benchmarks."""
    if not HAS_ROS2:
        pytest.skip("ROS2 not available")

    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def latency_node(ros2_context):
    """Create a latency measurement node."""
    node = LatencyMeasurementNode()
    yield node
    node.destroy_node()


class TestROS2Latency:
    """ROS2 latency benchmarks."""

    def test_single_message_latency(self, latency_node, benchmark):
        """Benchmark single message pub-sub latency."""
        def measure():
            return latency_node.measure_latency()

        result = benchmark(measure)
        assert result >= 0, "Message timed out"

    def test_burst_latency(self, latency_node):
        """Measure latency for burst of messages."""
        latencies = []
        for _ in range(100):
            latency = latency_node.measure_latency()
            if latency >= 0:
                latencies.append(latency)

        assert len(latencies) > 90, f"Too many timeouts: {100 - len(latencies)}"

        avg = statistics.mean(latencies)
        std = statistics.stdev(latencies) if len(latencies) > 1 else 0
        p99 = sorted(latencies)[int(len(latencies) * 0.99)] if latencies else 0

        print(f"\nBurst Latency Results (100 messages):")
        print(f"  Average: {avg:.3f} ms")
        print(f"  Std Dev: {std:.3f} ms")
        print(f"  P99: {p99:.3f} ms")
        print(f"  Min: {min(latencies):.3f} ms")
        print(f"  Max: {max(latencies):.3f} ms")

        # Soft assertion - warn if latency is high
        if avg > 10:
            pytest.warns(UserWarning, match="High average latency")


class TestLatencyWithoutROS2:
    """Fallback tests when ROS2 is not available."""

    @pytest.mark.skipif(HAS_ROS2, reason="ROS2 is available")
    def test_fallback_timing(self, benchmark):
        """Basic timing benchmark without ROS2."""
        def measure_time():
            start = time.perf_counter_ns()
            # Simulate some work
            _ = [i * 2 for i in range(1000)]
            end = time.perf_counter_ns()
            return (end - start) / 1_000_000

        result = benchmark(measure_time)
        assert result >= 0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--benchmark-only"])
