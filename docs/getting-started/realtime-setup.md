# Real-Time Kernel Support for ROS2

This guide covers setting up real-time kernel support for ROS2 robotics applications that require deterministic timing guarantees.

## Overview

Real-time kernel support is critical for:
- **Motion control** - Servo loops requiring <1ms jitter
- **Safety systems** - Reliable emergency stop response
- **Sensor fusion** - Synchronized multi-sensor processing
- **DDS performance** - Consistent message latency

## PREEMPT_RT Kernel

### Option 1: NixOS with RT Kernel

```nix
# nix/os/realtime-kernel.nix
{ config, lib, pkgs, ... }:

{
  boot.kernelPackages = pkgs.linuxPackages_rt;

  # Real-time scheduler settings
  boot.kernel.sysctl = {
    "kernel.sched_rt_runtime_us" = -1;  # Disable RT throttling
    "vm.swappiness" = 10;               # Reduce swap usage
    "kernel.nmi_watchdog" = 0;          # Disable NMI watchdog
  };

  # CPU isolation (isolate cores 2-3 for RT tasks)
  boot.kernelParams = [
    "isolcpus=2,3"
    "nohz_full=2,3"
    "rcu_nocbs=2,3"
  ];

  # Disable CPU frequency scaling on isolated cores
  powerManagement.cpuFreqGovernor = "performance";
}
```

### Option 2: Ubuntu with RT Kernel

```bash
# Install RT kernel on Ubuntu
sudo apt install linux-lowlatency

# Or for full PREEMPT_RT
sudo apt install linux-image-rt-amd64

# Verify RT capabilities
uname -a | grep -i preempt
cat /sys/kernel/realtime
```

## DDS Real-Time Configuration

### Fast DDS (Default ROS2)

```xml
<!-- fast_dds_rt.xml -->
<dds>
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>rt_udp</transport_id>
        <type>UDPv4</type>
        <sendBufferSize>1048576</sendBufferSize>
        <receiveBufferSize>1048576</receiveBufferSize>
      </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="rt_participant">
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <userTransports>
          <transport_id>rt_udp</transport_id>
        </userTransports>
      </rtps>
    </participant>

    <data_writer profile_name="rt_writer">
      <historyMemoryPolicy>PREALLOCATED</historyMemoryPolicy>
      <qos>
        <reliability>
          <kind>RELIABLE</kind>
        </reliability>
        <durability>
          <kind>VOLATILE</kind>
        </durability>
      </qos>
    </data_writer>
  </profiles>
</dds>
```

### Cyclone DDS

```xml
<!-- cyclonedds_rt.xml -->
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>auto</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="1MB"/>
      <SocketSendBufferSize min="1MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
    </Tracing>
  </Domain>
</CycloneDDS>
```

## CPU Isolation and Affinity

### Isolate CPUs at Boot

```bash
# Add to kernel command line (GRUB_CMDLINE_LINUX)
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3
```

### Pin ROS2 Nodes to Isolated CPUs

```python
#!/usr/bin/env python3
# rt_node_launcher.py
import os
import subprocess

def set_rt_affinity(pid, cores=[2, 3]):
    """Pin process to isolated cores"""
    core_mask = sum(1 << c for c in cores)
    subprocess.run(['taskset', '-p', hex(core_mask), str(pid)])

def set_rt_priority(pid, priority=80):
    """Set SCHED_FIFO real-time priority"""
    subprocess.run(['chrt', '-f', '-p', str(priority), str(pid)])

# Example: Launch ROS2 node with RT settings
node_cmd = ['ros2', 'run', 'my_package', 'my_node']
proc = subprocess.Popen(node_cmd)
set_rt_affinity(proc.pid)
set_rt_priority(proc.pid)
```

### Using ros2_control RT Executor

```cpp
// RT executor for ros2_control
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sched.h>

void configure_rt_thread() {
  struct sched_param param;
  param.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_WARN(rclcpp::get_logger("rt"), "Failed to set RT scheduler");
  }

  // Lock memory to prevent page faults
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_WARN(rclcpp::get_logger("rt"), "Failed to lock memory");
  }
}
```

## Latency Testing

### cyclictest

```bash
# Install rt-tests
sudo apt install rt-tests

# Run latency test (run as root)
sudo cyclictest -l100000 -m -Sp90 -i200 -h400 -q

# Expected results:
# - Non-RT kernel: Max latency ~1000+ us
# - RT kernel: Max latency <100 us
```

### ROS2 Latency Measurement

```python
#!/usr/bin/env python3
# measure_ros2_latency.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
import time

class LatencyMeasurer(Node):
    def __init__(self):
        super().__init__('latency_measurer')
        self.publisher = self.create_publisher(Header, 'latency_test', 10)
        self.subscription = self.create_subscription(
            Header, 'latency_test', self.callback, 10)
        self.timer = self.create_timer(0.001, self.publish)  # 1kHz
        self.latencies = []

    def publish(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

    def callback(self, msg):
        now = self.get_clock().now()
        sent = rclpy.time.Time.from_msg(msg.stamp)
        latency_us = (now - sent).nanoseconds / 1000
        self.latencies.append(latency_us)

        if len(self.latencies) >= 10000:
            import statistics
            self.get_logger().info(
                f"Latency stats (us): "
                f"min={min(self.latencies):.1f}, "
                f"max={max(self.latencies):.1f}, "
                f"avg={statistics.mean(self.latencies):.1f}, "
                f"stdev={statistics.stdev(self.latencies):.1f}"
            )
            self.latencies = []
```

## Memory Locking

### System-wide Configuration

```bash
# /etc/security/limits.conf
# Allow RT processes to lock memory
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited

# Add user to realtime group
sudo groupadd realtime
sudo usermod -a -G realtime $USER
```

### NixOS Configuration

```nix
# nix/os/realtime-kernel.nix (continued)
{
  security.pam.loginLimits = [
    { domain = "@realtime"; type = "-"; item = "rtprio"; value = "99"; }
    { domain = "@realtime"; type = "-"; item = "memlock"; value = "unlimited"; }
  ];

  users.groups.realtime = {};
  users.users.ros.extraGroups = [ "realtime" ];
}
```

## Network Tuning

### Low-Latency Network Stack

```bash
# /etc/sysctl.d/99-ros2-rt.conf
# Increase network buffers
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.core.rmem_default = 16777216
net.core.wmem_default = 16777216

# Reduce network latency
net.core.netdev_max_backlog = 5000
net.ipv4.tcp_low_latency = 1

# Multicast tuning for DDS
net.ipv4.igmp_max_memberships = 1024
```

### NixOS Network Configuration

```nix
{
  boot.kernel.sysctl = {
    "net.core.rmem_max" = 16777216;
    "net.core.wmem_max" = 16777216;
    "net.ipv4.tcp_low_latency" = 1;
  };
}
```

## Troubleshooting

### Check RT Capabilities

```bash
# Verify RT kernel
uname -a | grep -i rt

# Check if PREEMPT_RT is enabled
cat /sys/kernel/realtime  # Should return "1"

# Check current scheduling policy
chrt -p $$

# List isolated CPUs
cat /sys/devices/system/cpu/isolated
```

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| High jitter | CPU frequency scaling | Set governor to `performance` |
| Latency spikes | Memory allocation | Use `mlockall()` and preallocate |
| Priority inversion | Mutex contention | Use priority inheritance mutexes |
| Network delays | Buffer underruns | Increase socket buffers |

## References

- [ROS2 Real-Time Working Group](https://github.com/ros-realtime)
- [ros2_control RT Documentation](https://control.ros.org/master/doc/ros2_control/doc/real-time.html)
- [PREEMPT_RT Wiki](https://wiki.linuxfoundation.org/realtime/start)
- [cyclictest Documentation](https://wiki.linuxfoundation.org/realtime/documentation/howto/tools/cyclictest/start)
