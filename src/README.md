# Fish Robot Navigation System

A comprehensive ROS2 (Jazzy) navigation stack for a biomimetic fish robot using Gazebo Harmonic simulation, SLAM Toolbox, and Nav2 for autonomous navigation.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Workflow](#workflow)
- [Nodes](#nodes)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [How to Launch](#how-to-launch)
- [Project Structure](#project-structure)

## Overview

This package provides a complete navigation solution for a fish robot, featuring:

- **Simulation**: Gazebo Harmonic integration with custom fish robot model
- **SLAM**: Real-time mapping using SLAM Toolbox
- **Localization**: Pose estimation using SLAM Toolbox localization mode
- **Navigation**: Autonomous path planning and execution using Nav2
- **Motion Control**: Biomimetic fish-like undulating motion
- **Object Detection**: Camera-based object detection (optional)
- **Waypoint Following**: Automated waypoint navigation

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulator                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐           │
│  │   Camera     │  │    LiDAR     │  │     IMU      │           │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘           │
│         │                 │                 │                   │
└─────────┼─────────────────┼─────────────────┼───────────────────┘
          │                 │                 │
          ▼                 ▼                 ▼
    ┌─────────────────────────────────────────────────────┐
    │            ROS-Gazebo Bridge (gz_bridge)            │
    │  Topics: /*_eye/image, /scan, /imu, /odom, /cmd_vel │
    └─────────────────────────────────────────────────────┘
          │                │                   │
          ▼                ▼                   ▼
    ┌──────────┐    ┌──────────────┐   ┌──────────┐
    │  Camera  │    │   LiDAR      │   │   IMU    │
    │  Bridge  │    │   (scan)     │   │          │
    └──────────┘    └──────┬───────┘   └────┬─────┘
                           │                │
                           ▼                ▼
                  ┌────────────────────────────────┐
                  │ EKF Filter (robot_localization)│
                  │ Fuses: odom + IMU              │
                  └────────────────┬───────────────┘
                                   │
                                   ▼
                    ┌───────────────────────────────┐
                    │   TF Tree                     │
                    │   map → odom → base_footprint │
                    └──────────────┬────────────────┘
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                          │
        ▼                          ▼                          ▼
┌───────────────┐        ┌──────────────┐        ┌──────────────┐
│ SLAM Toolbox  │        │  Nav2 Stack  │        │ Fish Control │
│  (Mapping/    │        │              │        │  (publisher) │
│ Localization) │        │ - Planner    │        │              │
└───────┬───────┘        │ - Controller │        └──────┬───────┘
        │                │ - Costmaps   │               │
        │                └──────┬───────┘               │
        │                       │                       │
        └───────────────────────┼───────────────────────┘
                                │
                                ▼
                    ┌──────────────────────┐
                    │   /cmd_vel           │
                    │   (Twist messages)   │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  Motion Split        │
                    │  planar drive +      │
                    │  joint positions     │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │  Joint Controllers   │
                    │  (Gazebo)            │
                    └──────────────────────┘
```

## Workflow

### 1. Mapping Workflow

```
1. Launch Gazebo world + spawn robot
2. Start SLAM Toolbox (mapping mode)
3. Drive robot manually or via teleop
4. SLAM builds map in real-time
5. Save map using slam_toolbox service
```

### 2. Localization & Navigation Workflow

```
1. Launch Gazebo world + spawn robot
2. Load pre-built map (slam_toolbox_load_map.py)
3. Set initial pose (send_initialpose.py)
4. Start SLAM Toolbox (localization mode)
5. Start Nav2 navigation stack
6. Send navigation goals or follow waypoints
```

### 3. Complete Navigation Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    Initialization Phase                      │
├─────────────────────────────────────────────────────────────┤
│ 1. Gazebo World Launch                                       │
│    └─> Loads world.sdf, spawns robot model                  │
│ 2. Robot State Publisher                                     │
│    └─> Publishes robot_description from URDF                │
│ 3. Gazebo Bridge                                             │
│    └─> Bridges sensor topics (scan, imu, odom, camera)      │
│ 4. Static TF Publishers                                      │
│    └─> Establishes TF chain: odom → base_footprint          │
│ 5. EKF Filter                                                │
│    └─> Fuses odometry + IMU for accurate pose               │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Mapping Phase (Optional)                  │
├─────────────────────────────────────────────────────────────┤
│ 6. SLAM Toolbox (Mapping Mode)                               │
│    └─> Builds map from LiDAR scans                           │
│ 7. Map saved to maps/serialized.*                            │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  Localization Phase                          │
├─────────────────────────────────────────────────────────────┤
│ 8. Load Map (slam_toolbox_load_map.py)                       │
│    └─> Deserializes pre-built map                            │
│ 9. Set Initial Pose (send_initialpose.py)                   │
│    └─> Publishes initial pose estimate                       │
│ 10. SLAM Toolbox (Localization Mode)                         │
│     └─> Localizes robot in known map                         │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   Navigation Phase                            │
├─────────────────────────────────────────────────────────────┤
│ 11. Nav2 Stack                                                │
│     ├─> Global Planner: Plans path to goal                    │
│     ├─> Local Planner: Follows path, avoids obstacles        │
│     ├─> Costmaps: Global + Local obstacle maps                │
│     └─> Behavior Tree: Recovery behaviors                     │
│ 12. Gazebo planar joint controllers                           │
│     └─> Execute constrained X, Y, and yaw motion               │
│ 13. Fish Gait Controller                                      │
│     └─> Synchronizes bounded body motion with /cmd_vel        │
└─────────────────────────────────────────────────────────────┘
```

## Nodes

### Core Nodes

#### 1. `robot_state_publisher`

- **Purpose**: Publishes the robot's kinematic state to TF tree
- **Input**: URDF description (`mogi_bot.urdf`)
- **Output**: TF transforms for robot links
- **Key**: Establishes base_link → scan_link, imu_link transforms

#### 2. `ekf_filter_node` (robot_localization)

- **Purpose**: Fuses odometry and IMU data for accurate pose estimation
- **Input**:
  - `/odom` (wheel odometry from Gazebo)
  - `/imu` (IMU data)
- **Output**:
  - `/odom` (filtered odometry)
  - TF transform: `odom → base_footprint`
- **Configuration**: `config/ekf.yaml`
- **Key Features**:
  - 2D mode enabled
  - Fuses position (x, y) and orientation (yaw) from odom
  - Fuses angular velocity and orientation from IMU

#### 3. `gz_bridge` (ros_gz_bridge)

- **Purpose**: Bridges topics between ROS2 and Gazebo
- **Bridged Topics**:
  - `/scan` (LaserScan): LiDAR data
  - `/imu` (IMU): Inertial measurements
  - `/odom` (Odometry): Wheel odometry
  - `/cmd_vel` (Twist): Velocity commands
  - `/left_eye/camera_info`, `/right_eye/camera_info` (CameraInfo): eye camera calibration
  - `/clock` (Clock): Simulation time
- **Configuration**: `config/gz_bridge.yaml`

#### 4. `gz_image_bridge` (ros_gz_image)

- **Purpose**: Bridges the two eye-camera images from Gazebo to ROS2
- **Input**: `/left_eye/image`, `/right_eye/image` (Gazebo)
- **Output**: `/left_eye/image`, `/right_eye/image` (ROS2, plus `/compressed`)

#### 5. Static Transform Publishers

- **Purpose**: Establishes TF chain for Gazebo sensor frames
- **Transforms**:
  - `scan_link → mogi_bot/base_footprint/gpu_lidar`
  - `imu_link → mogi_bot/base_footprint/imu`

### Custom Nodes

#### 6. `publisher.py` (fish_gait_controller)

- **Purpose**: Drives the planar base and synchronizes a bounded biomimetic body wave with `/cmd_vel`
- **Input**: `/cmd_vel` (Twist messages) and `/odom` (heading feedback)
- **Output**: `/fish_joint_controller/commands` (Float64MultiArray positions)
- **Planar outputs**: `/planar_x_velocity`, `/planar_y_velocity`, and
  `/planar_yaw_velocity` (Float64 velocities)
- **Behavior**:
  - **Forward/Reverse**: Traveling wave with speed-scaled amplitude and frequency
  - **Left/Right**: Curvature bias proportional to commanded yaw rate
  - **Stop/Timeout**: Smoothly returns all body joints to neutral
- **Key Parameters**:
  - `max_tail_angle`: 0.55 rad
  - `min_frequency` / `max_frequency`: 1.2 / 2.4 Hz
  - `command_timeout`: 0.5 s
- **Propulsion**: The controller rotates body-forward speed into world X/Y
  using odometry heading. Gazebo joint controllers constrain the base to X,
  Y, and yaw, preventing wall contacts from pitching or rolling the fish.

#### 7. `object_detection_node.py` (optional)

- **Purpose**: Detects colored objects in camera feed
- **Input**: `/left_eye/image` (Image)
- **Output**: Visual display (OpenCV window)
- **Features**:
  - HSV color-based detection
  - Detects: yellow, red, blue, brown, gray objects
  - Draws bounding boxes
- **Status**: Currently commented out in launch file

### Utility Nodes (Python Package)

#### 8. `follow_waypoints.py`

- **Purpose**: Sends waypoint navigation goals to Nav2
- **Action**: `nav2_msgs/action/FollowWaypoints`
- **Behavior**:
  - Defines waypoints programmatically
  - Sends goal to Nav2 waypoint follower
  - Monitors progress and completion
- **Usage**: Run after Nav2 is active

#### 9. `send_initialpose.py`

- **Purpose**: Publishes initial pose estimate for localization
- **Topic**: `/initialpose` (PoseWithCovarianceStamped)
- **Parameters**:
  - `x`: Initial X position (default: 0.0)
  - `y`: Initial Y position (default: -4.0)
  - `yaw`: Initial orientation (default: 0.0)
- **Usage**: Run after map is loaded, before localization

#### 10. `slam_toolbox_load_map.py`

- **Purpose**: Loads pre-built SLAM map into SLAM Toolbox
- **Service**: `/slam_toolbox/deserialize_map`
- **Map File**: `maps/serialized` (from package share directory)
- **Usage**: Run when starting localization mode

## Packages

### `fishy_fish_navigation`

The main navigation package containing:

- Launch files for robot spawning, mapping, localization, and navigation
- Configuration files for EKF, Nav2, SLAM Toolbox, and controllers
- Python nodes for robot control (`publisher.py`, `object_detection_node.py`)
- Robot description files (URDF, Gazebo plugins)
- World files for Gazebo simulation
- RViz configuration files
- Pre-built maps

### `fishy_fish_navigation_py`

A Python utility package providing command-line tools for common navigation tasks. This package contains standalone ROS2 nodes that can be executed independently to perform specific operations.

**Package Structure**:

```
fishy_fish_navigation_py/
├── fishy_fish_navigation_py/
│   ├── __init__.py
│   ├── follow_waypoints.py      # Waypoint navigation node
│   ├── send_initialpose.py      # Initial pose publisher
│   └── slam_toolbox_load_map.py # Map loader node
├── package.xml
├── setup.py                      # Package setup with entry points
└── setup.cfg
```

**Purpose**: Provides convenient command-line utilities for:

1. **Map Loading**: Loading pre-built SLAM maps into SLAM Toolbox
2. **Initial Pose Setting**: Publishing initial pose estimates for localization
3. **Waypoint Navigation**: Sending waypoint sequences to Nav2

**Installation**: The package is installed via `colcon build` and exposes three console scripts:

- `ros2 run fishy_fish_navigation_py slam_toolbox_load_map`
- `ros2 run fishy_fish_navigation_py send_initialpose`
- `ros2 run fishy_fish_navigation_py follow_waypoints`

**Usage Pattern**: These utilities are typically run in separate terminals after the main navigation stack is launched, providing a modular approach to navigation tasks.

#### Detailed Node Descriptions

##### 1. `slam_toolbox_load_map.py`

**Purpose**: Loads a pre-built SLAM map (pose graph) into SLAM Toolbox for localization mode.

**How it Works**:

- Creates a service client for `/slam_toolbox/deserialize_map`
- Waits for the SLAM Toolbox service to become available
- Resolves the map file path from package share directory
- Sends a `DeserializePoseGraph` service request with:
  - Map file path (default: `fishy_fish_navigation/maps/serialized`)
  - Match type: `1` (PROCESS_FIRST_NODE - processes the first node in the pose graph)
  - Initial pose: (0.0, 0.0, 0.0) - can be adjusted if needed

**Parameters**:

- `map_file` (string, default: `'fishy_fish_navigation/maps/serialized'`): Package-relative path to the serialized map file

**Service Details**:

- **Service**: `/slam_toolbox/deserialize_map`
- **Service Type**: `slam_toolbox/srv/DeserializePoseGraph`
- **Request Fields**:
  - `filename`: Absolute path to serialized map file
  - `match_type`: Processing mode (1 = PROCESS_FIRST_NODE)
  - `initial_pose`: Initial pose estimate (Pose2D)

**Usage**:

```bash
# Default usage (loads map from fishy_fish_navigation/maps/serialized)
ros2 run fishy_fish_navigation_py slam_toolbox_load_map

# With custom map file parameter
ros2 run fishy_fish_navigation_py slam_toolbox_load_map --ros-args -p map_file:=custom_package/maps/my_map
```

**When to Use**:

- Before starting localization mode
- After launching `spawn_robot.launch.py` but before `localization_slam_toolbox.launch.py`
- The map must exist in the package's `maps/` directory

**Important Notes**:

- The node waits indefinitely for the service to become available
- Map file should be the serialized pose graph (`.data` or `.posegraph` files)
- The initial pose in the request is typically overridden by `send_initialpose.py`

##### 2. `send_initialpose.py`

**Purpose**: Publishes an initial pose estimate to the `/initialpose` topic, which is used by AMCL or SLAM Toolbox for localization initialization.

**How it Works**:

- Declares ROS2 parameters for x, y, and yaw
- Creates a publisher for `/initialpose` topic
- Constructs a `PoseWithCovarianceStamped` message with:
  - Position (x, y, z=0.0)
  - Orientation (quaternion converted from yaw angle)
  - Covariance matrix (6x6) representing uncertainty
- Publishes the message once
- Shuts down after 2 seconds

**Parameters**:

- `x` (double, default: `0.0`): Initial X position in map frame (meters)
- `y` (double, default: `-4.0`): Initial Y position in map frame (meters)
- `yaw` (double, default: `0.0`): Initial orientation in radians

**Message Details**:

- **Topic**: `/initialpose`
- **Message Type**: `geometry_msgs/msg/PoseWithCovarianceStamped`
- **Frame ID**: `map`
- **Covariance Matrix**:
  - Position uncertainty: 3.0 m² (x, y, yaw)
  - Represents uncertainty in the initial pose estimate
  - Larger values = less confidence in the initial pose

**Usage**:

```bash
# Default usage (x=0.0, y=-4.0, yaw=0.0)
ros2 run fishy_fish_navigation_py send_initialpose

# Custom initial pose
ros2 run fishy_fish_navigation_py send_initialpose --ros-args -p x:=2.5 -p y:=-1.0 -p yaw:=1.57

# Using degrees (convert to radians: 90° = 1.57 rad)
ros2 run fishy_fish_navigation_py send_initialpose --ros-args -p x:=0.0 -p y:=0.0 -p yaw:=1.5708
```

**When to Use**:

- After loading the map with `slam_toolbox_load_map.py`
- Before or immediately after starting localization
- When the robot's actual position in the map is known
- Can be used multiple times to reset localization if it drifts

**Important Notes**:

- The node publishes once and exits (one-shot publisher)
- This updates the localization estimate only; it does not move or teleport
  the Gazebo fish
- The covariance matrix is hardcoded but can be modified in the source
- Yaw is in radians (0 to 2π, or -π to π)
- The pose should match the robot's actual position in the map for best results

##### 3. `follow_waypoints.py`

**Purpose**: Sends a sequence of waypoints to Nav2's waypoint follower action server for automated navigation.

**How it Works**:

- Creates an action client for `follow_waypoints` action
- Defines waypoints programmatically (hardcoded in `define_waypoints()` method)
- Each waypoint is a `PoseStamped` with:
  - Position (x, y, z)
  - Orientation (quaternion)
  - Frame ID: `map`
- Sends the waypoint sequence as a goal to Nav2
- Monitors progress through feedback callbacks
- Logs completion status

**Action Details**:

- **Action Server**: `/follow_waypoints`
- **Action Type**: `nav2_msgs/action/FollowWaypoints`
- **Goal**: Array of `PoseStamped` messages representing waypoints

**Default Waypoints** (defined in code):

1. **Waypoint 1**: (6.0, 1.5, 0.0) with yaw = 0.0
2. **Waypoint 2**: (-2.0, -8.0, 0.0) with yaw = 1.57 (90°)
3. **Waypoint 3**: (0.0, 0.0, 0.0) with yaw = 0.0 (return to start)

**Callbacks**:

- `goal_response_callback`: Handles goal acceptance/rejection
- `feedback_callback`: Receives progress updates (current waypoint index)
- `get_result_callback`: Handles completion and shuts down the node

**Usage**:

```bash
# Run with default waypoints (hardcoded in the script)
ros2 run fishy_fish_navigation_py follow_waypoints
```

**Customization**:
To use custom waypoints, modify the `define_waypoints()` method in the source code:

```python
def define_waypoints(self):
    waypoints = []
  
    # Add your custom waypoints
    wp1 = PoseStamped()
    wp1.header.frame_id = 'map'
    wp1.pose.position.x = YOUR_X
    wp1.pose.position.y = YOUR_Y
    q = quaternion_from_euler(0, 0, YOUR_YAW)
    wp1.pose.orientation.x = q[0]
    wp1.pose.orientation.y = q[1]
    wp1.pose.orientation.z = q[2]
    wp1.pose.orientation.w = q[3]
    waypoints.append(wp1)
  
    return waypoints
```

**When to Use**:

- After Nav2 navigation stack is fully active
- When you want automated waypoint following
- For repetitive navigation patterns
- After localization is established

**Important Notes**:

- The node waits for the action server to become available
- Waypoints are executed sequentially
- The node shuts down automatically after completion
- Waypoints must be in the `map` frame
- Ensure waypoints are reachable and not in obstacles
- Nav2's waypoint follower must be configured (see `config/navigation.yaml`)

**Integration with Nav2**:

- Requires Nav2's waypoint follower to be active
- Configured in `navigation.yaml` with:
  - `waypoint_follower` plugin
  - `wait_at_waypoint` task executor (pauses 200ms at each waypoint)
  - Action server timeout: 900 seconds

## Launch Files

### 1. `world.launch.py`

- **Purpose**: Launches Gazebo simulator with specified world
- **Arguments**:
  - `world`: World file name (default: `fish_world.sdf`)
- **What it does**:
  - Starts Gazebo Harmonic
  - Loads world from `worlds/` directory
  - Sets up Gazebo environment
- **Usage**: Usually included by other launch files

### 2. `spawn_robot.launch.py`

- **Purpose**: Main robot spawning and sensor setup launch file
- **Arguments**:
  - `world`: World file (default: `fish_world.sdf`)
  - `model`: URDF file (default: `mogi_bot.urdf`)
  - `use_sim_time`: Enable simulation time (default: `True`)
- **Launches**:
  1. `world.launch.py` (Gazebo)
  2. Robot spawner (creates robot in Gazebo)
  3. `robot_state_publisher`
  4. `gz_bridge` (sensor bridges)
  5. `gz_image_bridge` (camera bridge)
  6. `ekf_filter_node`
  7. Static TF publishers (laser and IMU sensor aliases)
  8. Joint state broadcaster (after 10s delay)
  9. Fish joint position controller (after 15s delay)
  10. `fish_gait_controller` (publisher.py, after 20s delay)
- **Key**: This is the foundation for all other operations

### 3. `mapping.launch.py`

- **Purpose**: Launches SLAM mapping mode
- **Arguments**:
  - `rviz`: Open RViz (default: `true`)
  - `rviz_config`: RViz config file (default: `mapping.rviz`)
  - `use_sim_time`: Simulation time (default: `True`)
- **Launches**:
  1. RViz2 (with mapping config)
  2. SLAM Toolbox (online async mapping mode)
- **Configuration**: `config/slam_toolbox_mapping.yaml`
- **Usage**: Run with `spawn_robot.launch.py` to build a map

### 4. `localization_slam_toolbox.launch.py`

- **Purpose**: Launches SLAM localization mode (uses pre-built map)
- **Arguments**:
  - `rviz`: Open RViz (default: `true`)
  - `rviz_config`: RViz config file (default: `localization.rviz`)
  - `use_sim_time`: Simulation time (default: `True`)
- **Launches**:
  1. RViz2 (with localization config)
  2. SLAM Toolbox (localization mode)
- **Configuration**: `config/slam_toolbox_localization.yaml`
- **Map**: Loads from `maps/serialized`
- **Usage**: Run after loading map and setting initial pose

### 5. `navigation_with_slam.launch.py`

- **Purpose**: Launches complete navigation stack with SLAM
- **Arguments**:
  - `rviz`: Open RViz (default: `true`)
  - `rviz_config`: RViz config file (default: `navigation.rviz`)
  - `use_sim_time`: Simulation time (default: `True`)
- **Launches**:
  1. RViz2 (with navigation config)
  2. SLAM Toolbox (mapping mode - for real-time obstacle updates)
  3. Nav2 navigation stack
- **Configuration**:
  - `config/navigation.yaml` (Nav2)
  - `config/slam_toolbox_mapping.yaml` (SLAM)
- **Usage**: Complete autonomous navigation with real-time mapping

## Configuration Files

### `config/ekf.yaml`

- **Purpose**: Extended Kalman Filter configuration
- **Key Settings**:
  - Fuses odometry (x, y, yaw velocity) and IMU (angular velocity, orientation)
  - 2D mode enabled
  - Process noise covariance tuned for fish robot dynamics

### `config/gz_bridge.yaml`

- **Purpose**: Defines ROS-Gazebo topic bridges
- **Topics**: clock, joint_states, odom, cmd_vel, camera, scan, imu

### `config/navigation.yaml`

- **Purpose**: Nav2 navigation stack configuration
- **Components**:
  - **Controller**: DWB local planner (follows path)
  - **Planner**: Navfn global planner (path planning)
  - **Costmaps**: Global and local obstacle maps
  - **Behaviors**: Recovery behaviors (spin, backup, etc.)
  - **Waypoint Follower**: Automated waypoint navigation
- **Key Parameters**:
  - Max velocity: 0.5 m/s
  - Max angular velocity: 1.5 rad/s
  - Robot radius: 0.22 m

### `config/slam_toolbox_mapping.yaml`

- **Purpose**: SLAM Toolbox mapping configuration
- **Mode**: Online async mapping
- **Settings**: Scan matching, loop closure, map resolution

### `config/slam_toolbox_localization.yaml`

- **Purpose**: SLAM Toolbox localization configuration
- **Mode**: Localization only (no mapping)
- **Settings**: Scan matching for pose tracking in known map

### `config/fish_control.yaml`

- **Purpose**: Joint controller configuration
- **Controllers**:
  - `joint_state_broadcaster`
  - `fish_joint_controller`

### `config/waypoints.yaml`

- **Purpose**: Predefined waypoint locations
- **Format**: YAML with pose and orientation for each waypoint

## How to Launch

### Prerequisites

```bash
# Build the workspace
cd /path/to/swishy
colcon build --symlink-install
source install/setup.bash
```

### 1. Mapping (Build a Map)

**Terminal 1**: Launch robot and mapping

```bash
ros2 launch fishy_fish_navigation spawn_robot.launch.py
```

**Terminal 2**: Start SLAM mapping

```bash
ros2 launch fishy_fish_navigation mapping.launch.py
```

**Terminal 3**: Control robot (teleop or manual)

```bash
# Option 1: Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Option 2: Use RViz "2D Goal Pose" to set navigation goals
# (Nav2 will drive the robot)
```

**To Save Map**:

```bash
# After mapping, save the map using slam_toolbox service
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: {data: '/path/to/save/map'}}"
```

### 2. Localization & Navigation (Use Pre-built Map)

**Terminal 1**: Launch robot

```bash
ros2 launch fishy_fish_navigation spawn_robot.launch.py
```

**Terminal 2**: Load map and set initial pose

```bash
# Load the map
ros2 run fishy_fish_navigation_py slam_toolbox_load_map

# Set initial pose (adjust x, y, yaw as needed)
ros2 run fishy_fish_navigation_py send_initialpose --ros-args -p x:=0.0 -p y:=-4.0 -p yaw:=0.0
```

**Terminal 3**: Start localization

```bash
ros2 launch fishy_fish_navigation localization_slam_toolbox.launch.py
```

**Terminal 4**: Start navigation

```bash
ros2 launch fishy_fish_navigation navigation_with_slam.launch.py
```

**Terminal 5**: Send navigation goals

```bash
# Option 1: Use RViz "2D Goal Pose" tool
# Option 2: Use waypoint follower
ros2 run fishy_fish_navigation_py follow_waypoints
```

### 3. Complete Navigation with Real-time SLAM

**Terminal 1**: Launch robot

```bash
ros2 launch fishy_fish_navigation spawn_robot.launch.py
```

**Terminal 2**: Launch navigation with SLAM

```bash
ros2 launch fishy_fish_navigation navigation_with_slam.launch.py
```

**Terminal 3**: Control via RViz or waypoint follower

```bash
# Use RViz "2D Goal Pose" or
ros2 run fishy_fish_navigation_py follow_waypoints
```

### 4. Custom World/Model

```bash
# Use different world
ros2 launch fishy_fish_navigation spawn_robot.launch.py world:=home.sdf

# Use different robot model
ros2 launch fishy_fish_navigation spawn_robot.launch.py model:=custom_robot.urdf
```

### 5. Disable RViz

```bash
ros2 launch fishy_fish_navigation mapping.launch.py rviz:=false
```

## Project Structure

```
src/
├── fishy_fish_navigation/        # Main navigation package
│   ├── config/                    # Configuration files
│   │   ├── ekf.yaml              # EKF filter parameters
│   │   ├── fish_control.yaml     # Joint controller config
│   │   ├── gz_bridge.yaml        # ROS-Gazebo bridge config
│   │   ├── navigation.yaml       # Nav2 navigation config
│   │   ├── slam_toolbox_localization.yaml
│   │   ├── slam_toolbox_mapping.yaml
│   │   └── waypoints.yaml        # Predefined waypoints
│   ├── launch/                    # Launch files
│   │   ├── localization_slam_toolbox.launch.py
│   │   ├── mapping.launch.py
│   │   ├── navigation_with_slam.launch.py
│   │   ├── spawn_robot.launch.py  # Main robot launch
│   │   └── world.launch.py       # Gazebo world launch
│   ├── maps/                      # SLAM maps
│   │   ├── map.pgm               # Occupancy grid map
│   │   ├── map.yaml              # Map metadata
│   │   ├── serialized.data       # SLAM pose graph data
│   │   └── serialized.posegraph  # SLAM pose graph
│   ├── meshes/                    # 3D meshes for visualization
│   ├── models/                    # Gazebo model files
│   ├── rviz/                      # RViz configuration files
│   │   ├── localization.rviz
│   │   ├── mapping.rviz
│   │   └── navigation.rviz
│   ├── src/                       # Python nodes
│   │   ├── object_detection_node.py
│   │   └── publisher.py           # Fish controller
│   ├── urdf/                      # Robot description
│   │   ├── mogi_bot.urdf
│   │   ├── mogi_bot.gazebo
│   │   └── materials.xacro
│   └── worlds/                    # Gazebo world files
│       ├── fish_world.sdf
│       ├── home.sdf
│       └── empty.sdf
└── fishy_fish_navigation_py/      # Python utilities package
    └── fishy_fish_navigation_py/
        ├── follow_waypoints.py
        ├── send_initialpose.py
        └── slam_toolbox_load_map.py
```

## Key Features

### Biomimetic Motion

The fish robot uses a unique undulating motion pattern:

- **Forward Motion**: Three tail joints oscillate with phase shifts to create a traveling body wave
- **Turning**: A curvature bias follows commanded yaw rate
- **Smooth Transitions**: Rate-limited joint targets prevent jerky motion
- **Stable Base**: Simulation-only planar joints prevent Z, roll, and pitch drift

### Sensor Fusion

- **EKF** combines wheel odometry and IMU data for accurate pose estimation
- **SLAM Toolbox** uses LiDAR scans for mapping and localization
- **Nav2** uses costmaps (global + local) for obstacle avoidance

### Modular Design

- Each launch file serves a specific purpose (mapping, localization, navigation)
- Configuration files allow easy tuning without code changes
- Python utilities for common tasks (map loading, initial pose, waypoints)

## Troubleshooting

### TF Tree Issues

- Ensure all static transforms are published
- Check that `robot_state_publisher` is running
- Verify EKF is publishing `odom → base_footprint` transform

### Localization Fails

- Make sure initial pose is set correctly
- Verify map file exists and is loaded
- Check that LiDAR is publishing `/scan` topic

### Navigation Not Working

- Ensure Nav2 lifecycle nodes are active (use `ros2 lifecycle` commands)
- Check that costmaps are being published
- Verify `/cmd_vel` is being received by robot

### Robot Not Moving

- Check joint controllers are spawned (wait for delays in launch file)
- Verify `fish_gait_controller` (publisher.py) is running
- Check `/cmd_vel` and `/fish_joint_controller/commands`
- Check `/planar_x_velocity`, `/planar_y_velocity`, and
  `/planar_yaw_velocity`
- Check Gazebo topics under `/model/mogi_bot/joint/planar_*/cmd_vel`

## License

Apache License 2.0
