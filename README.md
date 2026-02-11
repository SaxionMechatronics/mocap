# Mocap

A ROS 2 driver for [OptiTrack](https://optitrack.com/) motion capture systems. It connects to a NatNet server and publishes rigid body poses as `geometry_msgs/msg/PoseStamped` topics.

**Features:**

- Automatic rigid body discovery - no manual topic configuration needed
- Supports both multicast and unicast streaming modes
- Cross-architecture - includes pre-built NatNet SDK libraries for both AMD64 and ARM64

## Tested Platforms

| Ubuntu | ROS 2 |
|---|---|
| 22.04 (Jammy) | Humble |
| 24.04 (Noble) | Jazzy |

The [OptiTrack NatNet SDK](https://optitrack.com/support/downloads?cat=developer-tools#natnet-sdk) v4.4 shared libraries are bundled in the `vendor/` directory for both AMD64 and ARM64, no separate installation is required.

## Building

```bash
# Clone into your colcon workspace
cd ~/colcon_ws/src
git clone -b feature/natnet44 https://github.com/SaxionMechatronics/mocap.git

# Build
cd ~/colcon_ws
colcon build --packages-select mocap
source install/setup.bash
```

## Usage

```bash
ros2 run mocap natnet_ros2_node
```

### With custom parameters

```bash
ros2 run mocap natnet_ros2_node --ros-args \
  -p serverIP:=192.168.1.100 \
  -p clientIP:=192.168.1.50 \
  -p serverType:=multicast
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serverIP` | `string` | `192.168.1.245` | IP address of the NatNet server (machine running Motive) |
| `clientIP` | `string` | `192.168.1.136` | IP address of the local machine running this node |
| `serverType` | `string` | `multicast` | Streaming mode: `multicast` or `unicast` |
| `multicastAddress` | `string` | `239.255.42.99` | Multicast group address (only used when `serverType` is `multicast`) |
| `serverCommandPort` | `int` | `1510` | NatNet command port |
| `serverDataPort` | `int` | `1511` | NatNet data port |
| `global_frame` | `string` | `world` | Frame ID set in the `header.frame_id` of every published message |

## Published Topics

For each rigid body defined on the NatNet server, the node publishes:

| Topic | Type | Description |
|---|---|---|
| `<rigid_body_name>/pose` | `geometry_msgs/msg/PoseStamped` | 6-DoF pose (position + orientation) of the rigid body |

Topic names are derived directly from the rigid body names configured in Motive. For example, a rigid body named `drone` produces a topic at `drone/pose`.

## Network Setup

1. Ensure the machine running this node is on the **same subnet** as the OptiTrack server.
2. In Motive, verify that **Data Streaming** is enabled and the streaming settings (IP, ports, multicast/unicast) match the node parameters.
3. If using multicast, confirm that your network infrastructure allows multicast traffic on the configured address.
