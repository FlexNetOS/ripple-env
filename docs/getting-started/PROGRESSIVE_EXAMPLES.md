# Progressive Examples

This guide provides examples organized by complexity level. Start at your comfort level and progress upward.

---

## Beginner Level

Examples for users new to Nix, ROS2, or both.

### Example 1: Hello World Node

The simplest possible ROS2 node.

**Goal:** Create a node that prints a message.

```bash
# Create workspace
mkdir -p ~/examples/hello_ws/src
cd ~/examples/hello_ws/src

# Create package
ros2 pkg create --build-type ament_python hello_world --dependencies rclpy
```

**Edit `hello_world/hello_world/talker.py`:**
```python
import rclpy
from rclpy.node import Node

class HelloWorld(Node):
    def __init__(self):
        super().__init__('hello_world')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello, World!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorld()
    rclpy.spin(node)
    rclpy.shutdown()
```

**Update `setup.py` entry_points:**
```python
entry_points={
    'console_scripts': [
        'talker = hello_world.talker:main',
    ],
},
```

**Build and run:**
```bash
cd ~/examples/hello_ws
colcon build --symlink-install
source install/setup.bash
ros2 run hello_world talker
```

**Expected output:**
```
[INFO] [hello_world]: Hello, World!
[INFO] [hello_world]: Hello, World!
...
```

---

### Example 2: Publisher and Subscriber

Two nodes communicating via a topic.

**Goal:** One node publishes messages, another receives them.

```bash
cd ~/examples/hello_ws/src
ros2 pkg create --build-type ament_python pubsub_demo --dependencies rclpy std_msgs
```

**`pubsub_demo/pubsub_demo/publisher.py`:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('publisher')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.publish)
        self.count = 0

    def publish(self):
        msg = String()
        msg.data = f'Message #{self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Publisher())
    rclpy.shutdown()
```

**`pubsub_demo/pubsub_demo/subscriber.py`:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.sub = self.create_subscription(String, 'chatter', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Subscriber())
    rclpy.shutdown()
```

**Update `setup.py`:**
```python
entry_points={
    'console_scripts': [
        'publisher = pubsub_demo.publisher:main',
        'subscriber = pubsub_demo.subscriber:main',
    ],
},
```

**Run in two terminals:**
```bash
# Terminal 1
ros2 run pubsub_demo publisher

# Terminal 2
ros2 run pubsub_demo subscriber
```

---

### Example 3: Using Pixi Packages

Adding and using Python packages via Pixi.

**Goal:** Use NumPy in a ROS2 node.

```bash
# Add numpy
pixi add numpy

# Create package
cd ~/examples/hello_ws/src
ros2 pkg create --build-type ament_python numpy_demo --dependencies rclpy std_msgs
```

**`numpy_demo/numpy_demo/stats_node.py`:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np

class StatsNode(Node):
    def __init__(self):
        super().__init__('stats_node')
        self.pub = self.create_publisher(Float64, 'random_value', 10)
        self.timer = self.create_timer(1.0, self.publish_random)
        self.values = []

    def publish_random(self):
        value = np.random.normal(0, 1)
        self.values.append(value)

        msg = Float64()
        msg.data = value
        self.pub.publish(msg)

        if len(self.values) >= 5:
            mean = np.mean(self.values[-5:])
            std = np.std(self.values[-5:])
            self.get_logger().info(f'Value: {value:.3f}, Mean(5): {mean:.3f}, Std(5): {std:.3f}')
        else:
            self.get_logger().info(f'Value: {value:.3f} (collecting samples...)')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(StatsNode())
    rclpy.shutdown()
```

---

## Intermediate Level

Examples for users comfortable with basics.

### Example 4: Service and Client

Synchronous request-response communication.

**Goal:** Create a service that adds two numbers.

```bash
cd ~/examples/hello_ws/src
ros2 pkg create --build-type ament_python adder_service --dependencies rclpy example_interfaces
```

**`adder_service/adder_service/server.py`:**
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdderServer(Node):
    def __init__(self):
        super().__init__('adder_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.get_logger().info('Adder service ready')

    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'{request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AdderServer())
    rclpy.shutdown()
```

**`adder_service/adder_service/client.py`:**
```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdderClient(Node):
    def __init__(self):
        super().__init__('adder_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a = a
        req.b = b
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = AdderClient()

    a, b = int(sys.argv[1]), int(sys.argv[2])
    result = client.send_request(a, b)
    client.get_logger().info(f'{a} + {b} = {result.sum}')

    client.destroy_node()
    rclpy.shutdown()
```

**Run:**
```bash
# Terminal 1
ros2 run adder_service server

# Terminal 2
ros2 run adder_service client 5 3
```

---

### Example 5: Launch Files

Launching multiple nodes together.

**Goal:** Start publisher and subscriber with one command.

**Create `pubsub_demo/launch/demo.launch.py`:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub_demo',
            executable='publisher',
            name='publisher',
            output='screen',
        ),
        Node(
            package='pubsub_demo',
            executable='subscriber',
            name='subscriber',
            output='screen',
        ),
    ])
```

**Update `setup.py`:**
```python
import os
from glob import glob

# Add to data_files
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/pubsub_demo']),
    ('share/pubsub_demo', ['package.xml']),
    (os.path.join('share', 'pubsub_demo', 'launch'), glob('launch/*.launch.py')),
],
```

**Run:**
```bash
colcon build --packages-select pubsub_demo
ros2 launch pubsub_demo demo.launch.py
```

---

### Example 6: Docker Service Integration

Using the observability stack with ROS2.

**Goal:** Monitor ROS2 node metrics in Grafana.

```bash
# Start observability stack
docker network create agentic-network 2>/dev/null || true
docker compose -f docker/docker-compose.observability.yml up -d

# Check Grafana is running
curl -s http://localhost:3000/api/health | jq .
```

**Create a metrics-publishing node `metrics_demo/metrics_demo/metrics_node.py`:**
```python
import rclpy
from rclpy.node import Node
from prometheus_client import start_http_server, Counter, Gauge
import random

MESSAGES_SENT = Counter('ros2_messages_sent_total', 'Total messages sent')
CURRENT_VALUE = Gauge('ros2_current_value', 'Current sensor value')

class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')
        start_http_server(8000)  # Prometheus metrics endpoint
        self.timer = self.create_timer(1.0, self.publish_metrics)
        self.get_logger().info('Metrics available at http://localhost:8000')

    def publish_metrics(self):
        value = random.uniform(0, 100)
        MESSAGES_SENT.inc()
        CURRENT_VALUE.set(value)
        self.get_logger().info(f'Published metric: {value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MetricsNode())
    rclpy.shutdown()
```

**Add prometheus-client:**
```bash
pixi add prometheus-client
```

**Access metrics:**
- Prometheus: http://localhost:9090
- Grafana: http://localhost:3000 (admin/admin)

---

## Advanced Level

Examples for experienced users.

### Example 7: Custom Nix Shell

Creating a project-specific development shell.

**Goal:** Add custom packages to a devshell.

**Create `my_project/flake.nix`:**
```nix
{
  description = "My ROS2 Project";

  inputs = {
    ripple-env.url = "github:FlexNetOS/ripple-env";
    nixpkgs.follows = "ripple-env/nixpkgs";
  };

  outputs = { self, ripple-env, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; };
    in {
      devShells.${system}.default = pkgs.mkShell {
        inputsFrom = [ ripple-env.devShells.${system}.default ];

        packages = with pkgs; [
          # Add project-specific tools
          htop
          jq
          yq-go
        ];

        shellHook = ''
          echo "Welcome to My ROS2 Project!"
          export PROJECT_NAME="my_robot"
        '';
      };
    };
}
```

**Use it:**
```bash
cd my_project
nix develop
echo $PROJECT_NAME  # "my_robot"
```

---

### Example 8: Multi-Stage Build Pipeline

CI/CD pipeline with Nix caching.

**Goal:** Build and test in GitHub Actions.

**`.github/workflows/ros2-ci.yml`:**
```yaml
name: ROS2 CI

on:
  push:
    branches: [main]
  pull_request:

jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: DeterminateSystems/nix-installer-action@main

      - uses: DeterminateSystems/magic-nix-cache-action@main

      - name: Build and Test
        run: |
          nix develop --command bash -c '
            cd ros2_ws
            colcon build --symlink-install
            colcon test
            colcon test-result --verbose
          '
```

---

### Example 9: Agent Integration

Integrating with the ARIA orchestrator.

**Goal:** Create a ROS2 node that responds to agent commands.

**`agent_bridge/agent_bridge/bridge.py`:**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
from aiohttp import web

class AgentBridge(Node):
    def __init__(self):
        super().__init__('agent_bridge')
        self.pub = self.create_publisher(String, 'agent_commands', 10)
        self.sub = self.create_subscription(
            String, 'agent_responses', self.response_callback, 10
        )
        self.pending_responses = {}
        self.get_logger().info('Agent bridge started')

    def response_callback(self, msg):
        data = json.loads(msg.data)
        request_id = data.get('request_id')
        if request_id in self.pending_responses:
            self.pending_responses[request_id].set_result(data)

    async def handle_command(self, request):
        data = await request.json()
        request_id = data.get('id', str(id(data)))

        # Publish to ROS2
        msg = String()
        msg.data = json.dumps({**data, 'request_id': request_id})
        self.pub.publish(msg)

        # Wait for response
        future = asyncio.Future()
        self.pending_responses[request_id] = future

        try:
            result = await asyncio.wait_for(future, timeout=30.0)
            return web.json_response(result)
        except asyncio.TimeoutError:
            return web.json_response({'error': 'timeout'}, status=504)
        finally:
            del self.pending_responses[request_id]

def main(args=None):
    rclpy.init(args=args)
    node = AgentBridge()

    app = web.Application()
    app.router.add_post('/command', node.handle_command)

    # Run both ROS2 and web server
    import threading
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.start()

    web.run_app(app, port=8080)

    rclpy.shutdown()
```

---

### Example 10: NixOS Image Generation

Building deployable NixOS images.

**Goal:** Create a WSL2 image with ROS2 pre-configured.

```bash
# Build WSL2 tarball
nix build .#nixosConfigurations.wsl-ripple.config.system.build.tarballBuilder

# The result is a script that creates the tarball
./result/bin/nixos-wsl-tarball-builder

# Import to WSL (from PowerShell)
wsl --import MyROS2 $env:USERPROFILE\WSL\MyROS2 nixos-wsl.tar.gz
```

**Customize the image in `nix/images/wsl.nix`:**
```nix
{ config, lib, pkgs, ... }:
{
  # Add custom packages
  environment.systemPackages = with pkgs; [
    neovim
    tmux
    git
  ];

  # Pre-configure ROS2
  environment.variables = {
    ROS_DISTRO = "humble";
    ROS_DOMAIN_ID = "42";
  };

  # Enable services
  services.openssh.enable = true;
}
```

---

## Example Index

| # | Name | Level | Topics |
|---|------|-------|--------|
| 1 | Hello World Node | Beginner | rclpy, nodes, timers |
| 2 | Publisher/Subscriber | Beginner | topics, messages |
| 3 | Using Pixi Packages | Beginner | pixi, numpy |
| 4 | Service and Client | Intermediate | services, request/response |
| 5 | Launch Files | Intermediate | launch, multi-node |
| 6 | Docker Integration | Intermediate | prometheus, grafana |
| 7 | Custom Nix Shell | Advanced | flakes, devshells |
| 8 | CI/CD Pipeline | Advanced | github actions, caching |
| 9 | Agent Integration | Advanced | async, web server |
| 10 | NixOS Image | Advanced | nixos, wsl2 |

---

## Practice Exercises

### Beginner Exercises

1. Modify Example 1 to print a custom message from a parameter
2. Add a third node to Example 2 that counts messages
3. Create a node using pandas instead of numpy

### Intermediate Exercises

1. Create a service that reverses a string
2. Add parameters to the launch file in Example 5
3. Create a Grafana dashboard for Example 6 metrics

### Advanced Exercises

1. Extend Example 7 to include cross-compilation
2. Add code coverage to Example 8
3. Implement authentication in Example 9

---

## See Also

- [Onboarding Tutorial](../ONBOARDING_TUTORIAL.md) - Step-by-step beginner guide
- [Common Pitfalls](../COMMON_PITFALLS.md) - Mistakes to avoid
- [Troubleshooting](../TROUBLESHOOTING.md) - When things go wrong
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html) - Official ROS2 tutorials
