# curobo_msgs - ROS 2 Message Definitions for curobo_ros

Custom ROS 2 message, service, and action definitions for the [curobo_ros](https://github.com/Lab-CORO/curobo_ros) GPU-accelerated motion planning package.

## Overview

This package provides the complete interface definitions for curobo_ros, enabling motion planning, kinematics, and collision management services.

**Package Contents:**
- **13 Services**: Motion planning, kinematics, collision management
- **3 Actions**: Trajectory execution, MPC control, data generation
- **Custom Messages**: Object parameters, joint states with validity flags

---

## Table of Contents

- [Services](#services)
  - [Motion Planning](#motion-planning)
  - [Collision Management](#collision-management)
  - [Kinematics](#kinematics)
- [Actions](#actions)
- [Installation](#installation)
- [Usage Examples](#usage-examples)
- [Version History](#version-history)

---

## Services

### Motion Planning

#### 1. TrajectoryGeneration.srv (UPDATED Dec 5, 2025)

Generate collision-free motion trajectory from start to target pose(s).

**Request:**
```
sensor_msgs/JointState start_pose
geometry_msgs/Pose target_pose                  # Single target
geometry_msgs/Pose[] target_poses               # Multiple waypoints (NEW)
float64[] target_joint_positions                # Joint-space target (NEW)
int8[] trajectory_constraints                   # Constraints [theta_x, theta_y, theta_z, x, y, z] (NEW)
int8[] trajectories_contraints                  # Per-waypoint constraints (NEW)
```

**Response:**
```
bool success
string message
sensor_msgs/JointState[] trajectory  # NEW (Dec 5): Full trajectory in response
float64 dt                          # NEW (Dec 5): Time step between waypoints
```

**What's New:**
- Trajectory now returned directly in response (previously only published to topic)
- Multi-point waypoint planning support
- Joint-space planning support
- Trajectory constraints support

**Example:**
```bash
# Single-point Cartesian planning
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}"

# Multi-point planning (NEW)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_poses: [
    {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
    {position: {x: 0.6, y: 0.2, z: 0.4}, orientation: {w: 1.0}}
  ]}"

# Joint-space planning (NEW)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_joint_positions: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}"
```

---

#### 2. SetPlanner.srv (NEW Nov 9, 2025)

Switch between different planning algorithms at runtime.

**Constants:**
```
uint8 CLASSIC = 0       # Classic motion planning
uint8 MPC = 1           # Model Predictive Control
uint8 BATCH = 2         # Batch planning
uint8 CONSTRAINED = 3   # Constrained planning
uint8 MULTIPOINT = 4    # Multi-point planning
uint8 JOINT_SPACE = 5   # Joint-space planning
```

**Request:**
```
uint8 planner_type  # Planner type constant
```

**Response:**
```
bool success
string message
string previous_planner
string current_planner
```

**Example:**
```bash
# Switch to MPC planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# Switch to classic planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 0}"
```

---

### Collision Management

#### 3. AddObject.srv

Add collision object to the scene.

**Request:**
```
string name                 # Unique object identifier
uint8 primitive_type        # 0=CUBOID, 1=CYLINDER, 2=SPHERE, 3=CAPSULE, 4=MESH
float64[] dimensions        # Object dimensions
geometry_msgs/Pose pose     # Object pose in base frame
string mesh_file_path       # Path to mesh file (for MESH type)
```

**Response:**
```
bool success
string message
```

**Example:**
```bash
# Add cuboid (table)
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'table', primitive_type: 0, dimensions: [1.0, 0.8, 0.05], \
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# Add sphere
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'ball', primitive_type: 2, dimensions: [0.1], \
    pose: {position: {x: 0.4, y: 0.2, z: 0.3}, orientation: {w: 1.0}}}"
```

---

#### 4. RemoveObject.srv

Remove specific obstacle from scene.

**Request:**
```
string name  # Object name to remove
```

**Response:**
```
bool success
string message
```

**Example:**
```bash
ros2 service call /unified_planner/remove_object curobo_msgs/srv/RemoveObject "{name: 'table'}"
```

---

#### 5. GetVoxelGrid.srv

Retrieve voxelized representation of collision scene.

**Request:** Empty

**Response:**
```
bool success
string message
sensor_msgs/PointCloud2 voxel_grid  # Occupied voxels as point cloud
```

**Example:**
```bash
ros2 service call /unified_planner/get_voxel_grid curobo_msgs/srv/GetVoxelGrid
```

---

#### 6. GetCollisionDistance.srv

Compute distances to nearest obstacles.

**Request:**
```
sensor_msgs/JointState joint_state
```

**Response:**
```
bool success
string message
float64[] distances     # Distance per collision sphere (meters)
bool in_collision       # True if any collision detected
```

**Example:**
```bash
ros2 service call /unified_planner/get_collision_distance curobo_msgs/srv/GetCollisionDistance \
  "{joint_state: {position: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}}"
```

---

#### 7. SetCollisionCache.srv (NEW Dec 12, 2025)

Configure collision cache sizes to optimize GPU memory usage.

**Request:**
```
int32 obb   # OBB cache size (-1 = don't modify)
int32 mesh  # Mesh cache size (-1 = don't modify)
int32 blox  # Voxel cache size (-1 = don't modify)
```

**Response:**
```
bool success
string message
int32 obb_cache     # Current OBB cache size
int32 mesh_cache    # Current mesh cache size
int32 blox_cache    # Current voxel cache size
```

**Purpose:**
Dynamically adjust collision cache sizes based on scene complexity. Use `-1` to query current values without modification.

**Example:**
```bash
# Increase mesh cache for complex scenes
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: 50, blox: -1}"

# Query current cache sizes
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: -1, blox: -1}"
```

---

### Kinematics

#### 8. Fk.srv

Compute end-effector pose from joint positions (Forward Kinematics).

**Request:**
```
sensor_msgs/JointState joint_state
```

**Response:**
```
bool success
string message
geometry_msgs/Pose end_effector_pose
```

**Example:**
```bash
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}}"
```

---

#### 9. Ik.srv

Compute joint configurations for target pose(s) (Batch Inverse Kinematics).

**Request:**
```
geometry_msgs/Pose[] target_poses
int32 num_seeds  # Number of IK seeds (default: 20, recommended: 30-50)
```

**Response:**
```
bool success
string message
sensor_msgs/JointState[] solutions   # IK solutions (one per target pose)
int32[] success_mask                 # 1 if solution found, 0 if failed
```

**Example:**
```bash
# Single pose IK
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{target_poses: [{position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}], num_seeds: 30}"

# Batch IK for multiple waypoints
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{target_poses: [
    {position: {x: 0.5, y: 0.2, z: 0.4}, orientation: {w: 1.0}},
    {position: {x: 0.6, y: 0.2, z: 0.4}, orientation: {w: 1.0}}
  ], num_seeds: 30}"
```

---

#### 10. IkBatch.srv

Alternative interface for batch inverse kinematics.

**Request/Response:** Similar to Ik.srv with different message structure.

---

### Internal/Advanced Services

#### 11. SceneGenerator.srv
Internal service for generating test scenes.

#### 12. GenerateRM.srv
Internal service for roadmap generation.

#### 13. Collisions.srv
Internal collision query interface.

**Note:** These services are for advanced use cases. See source code for details.

---

## Actions

### 1. SendTrajectory.action

Execute planned trajectory with real-time feedback.

**Goal:**
```
trajectory_msgs/JointTrajectory trajectory
```

**Feedback:**
```
sensor_msgs/JointState current_joint_state
float32 progress  # [0.0 - 1.0]
```

**Result:**
```
bool success
string message
```

**Example:**
```bash
ros2 action send_goal /unified_planner/send_trajectory curobo_msgs/action/SendTrajectory \
  "{trajectory: {...}}" \
  --feedback
```

**Use Case:** Execute trajectory generated by `TrajectoryGeneration` service.

---

### 2. MpcMove.action (NEW Nov 9, 2025)

MPC closed-loop control with continuous replanning.

**Goal:**
```
geometry_msgs/Pose target_pose
```

**Feedback:**
```
sensor_msgs/JointState joint_command  # Current joint command
float32 step_progression              # Progress [0.0 - 1.0]
```

**Result:**
```
bool success
string message
```

**Behavior:**
- Continuously replans at 100 Hz (configurable)
- Automatically avoids dynamic obstacles
- Converges when within threshold (default: 0.01m)
- Can be canceled mid-execution

**Example:**
```bash
# Send MPC move goal with feedback
ros2 action send_goal /unified_planner/mpc_move curobo_msgs/action/MpcMove \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}" \
  --feedback

# Monitor feedback
# Feedback: step_progression: 0.15
# Feedback: step_progression: 0.32
# ...
# Result: success: true

# Cancel active goal
ros2 action cancel_goal /unified_planner/mpc_move <goal_id>
```

**Use Cases:**
- Reactive control in dynamic environments
- Tracking moving targets
- Real-time obstacle avoidance

---

### 3. DataGeneration.action

Internal action for batch data generation (advanced users only).

**Goal/Feedback/Result:** See source code for details.

---

## Installation

Build with your ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select curobo_msgs
source install/setup.bash
```

## Usage Examples

### Example 1: Plan and Execute Workflow

```bash
# 1. Clear existing obstacles
ros2 service call /unified_planner/remove_all_objects std_srvs/srv/Trigger

# 2. Add table obstacle
ros2 service call /unified_planner/add_object curobo_msgs/srv/AddObject \
  "{name: 'table', primitive_type: 0, dimensions: [1.0, 0.8, 0.05], \
    pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

# 3. Generate trajectory (will avoid table)
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}"

# 4. Execute trajectory
ros2 action send_goal /unified_planner/send_trajectory curobo_msgs/action/SendTrajectory \
  "{trajectory: {...}}"
```

### Example 2: Dynamic Environment with MPC

```bash
# 1. Switch to MPC planner
ros2 service call /unified_planner/set_planner curobo_msgs/srv/SetPlanner "{planner_type: 1}"

# 2. Use MPC action for reactive control
ros2 action send_goal /unified_planner/mpc_move curobo_msgs/action/MpcMove \
  "{target_pose: {position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}}" \
  --feedback
```

### Example 3: IK/FK Workflow

```bash
# 1. Forward kinematics: Get pose from joint configuration
ros2 service call /unified_planner/fk_compute curobo_msgs/srv/Fk \
  "{joint_state: {position: [0.0, -0.5, 0.5, 0.0, 1.57, 0.0]}}"

# 2. Inverse kinematics: Get joint configuration from target pose
ros2 service call /unified_planner/ik_batch_poses curobo_msgs/srv/Ik \
  "{target_poses: [{position: {x: 0.5, y: 0.3, z: 0.4}, orientation: {w: 1.0}}], num_seeds: 30}"

# 3. Use IK solution for trajectory planning
ros2 service call /unified_planner/generate_trajectory curobo_msgs/srv/TrajectoryGeneration \
  "{target_joint_positions: [...]}"  # Use IK solution from step 2
```

### Example 4: Collision Cache Optimization

```bash
# For scenes with many mesh objects, increase mesh cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: 50, blox: -1}"

# For point cloud environments, increase blox cache
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: -1, blox: 30}"

# Query current cache sizes
ros2 service call /unified_planner/set_collision_cache \
  curobo_msgs/srv/SetCollisionCache "{obb: -1, mesh: -1, blox: -1}"
```

---

## Integration in Your Package

### package.xml

Add dependency:
```xml
<depend>curobo_msgs</depend>
```

### CMakeLists.txt (C++)

```cmake
find_package(curobo_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node curobo_msgs)
```

### Python

Service client example:
```python
import rclpy
from rclpy.node import Node
from curobo_msgs.srv import TrajectoryGeneration
from geometry_msgs.msg import Pose

class PlannerClient(Node):
    def __init__(self):
        super().__init__('planner_client')
        self.client = self.create_client(
            TrajectoryGeneration,
            '/unified_planner/generate_trajectory'
        )

    def plan_trajectory(self, target_pose):
        request = TrajectoryGeneration.Request()
        request.target_pose = target_pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()
```

Action client example:
```python
from rclpy.action import ActionClient
from curobo_msgs.action import MpcMove

class MpcClient(Node):
    def __init__(self):
        super().__init__('mpc_client')
        self.action_client = ActionClient(
            self,
            MpcMove,
            '/unified_planner/mpc_move'
        )

    def send_goal(self, target_pose):
        goal_msg = MpcMove.Goal()
        goal_msg.target_pose = target_pose

        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
```

---

## Message/Service/Action Reference

### Services by Category

**Motion Planning:**
- `TrajectoryGeneration.srv` - Plan collision-free trajectory
- `SetPlanner.srv` - Switch planner algorithm

**Collision Management:**
- `AddObject.srv` - Add obstacle
- `RemoveObject.srv` - Remove obstacle
- `GetVoxelGrid.srv` - Get voxel representation
- `GetCollisionDistance.srv` - Query collision distances
- `SetCollisionCache.srv` - Configure collision cache

**Kinematics:**
- `Fk.srv` - Forward kinematics
- `Ik.srv` - Batch inverse kinematics
- `IkBatch.srv` - Alternative IK interface

**Internal/Advanced:**
- `SceneGenerator.srv`, `GenerateRM.srv`, `Collisions.srv`

### Actions by Use Case

**Trajectory Execution:**
- `SendTrajectory.action` - Execute pre-planned trajectory

**Reactive Control:**
- `MpcMove.action` - MPC closed-loop control

**Data Generation:**
- `DataGeneration.action` - Internal testing/training

---

## Version History

### December 15, 2025
- Removed collision checker type switch

### December 12, 2025
- **NEW**: Added `SetCollisionCache.srv` for dynamic cache configuration

### December 5, 2025
- **UPDATED**: `TrajectoryGeneration.srv` now returns trajectory in response
- **NEW**: Added trajectory constraints support
- **NEW**: Added joint-space planning support

### November 27, 2025
- **NEW**: Added multi-point waypoint support to `TrajectoryGeneration.srv`

### November 9, 2025
- **NEW**: Added `SetPlanner.srv` for runtime planner switching
- **NEW**: Added `MpcMove.action` for closed-loop MPC control

---

## Related Packages

- **[curobo_ros](https://github.com/Lab-CORO/curobo_ros)** - Main motion planning node
- **[curobo_rviz](https://github.com/Lab-CORO/curobo_rviz)** - RViz visualization plugin
- **[cuRobo](https://curobo.org)** - NVIDIA GPU-accelerated motion planning library

---

## Documentation

- [ROS 2 Interface Reference](../curobo_ros/doc/concepts/ros-interfaces.md) - Complete API documentation
- [Tutorial 1: Your First Trajectory](../curobo_ros/doc/tutorials/01-first-trajectory.md) - Basic usage
- [Tutorial 6: IK/FK Services](../curobo_ros/doc/tutorials/06-ik-fk-services.md) - Kinematics examples
- [Tutorial 5: MPC Planner](../curobo_ros/doc/tutorials/05-mpc-planner.md) - MPC usage

---

## License

See [curobo_ros](https://github.com/Lab-CORO/curobo_ros) for license information.

## Support

For issues and questions:
- GitHub Issues: https://github.com/Lab-CORO/curobo_msgs/issues
- curobo_ros Documentation: https://github.com/Lab-CORO/curobo_ros/tree/main/doc

---

**Quick Links**: [curobo_ros](https://github.com/Lab-CORO/curobo_ros) | [Documentation](../curobo_ros/doc/) | [ROS Interface Reference](../curobo_ros/doc/concepts/ros-interfaces.md)
