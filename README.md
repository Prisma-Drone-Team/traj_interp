# Trajectory Interpolator (traj_interp)

A ROS2 node that interpolates waypoints from external RRT algorithms based on a second-order filter

## 🎯 Features

- ✅ **Filtered Interpolation**: Algorithm adapted from the lee_controller for smooth trajectories.
- ✅ **Continuous Tracking**: Waypoints are followed sequentially without stopping.
- ✅ **Dynamic Path Management**: New paths immediately replace the previous one.
- ✅ **PX4 Compatibility**: Standard interface using px4_msgs.
- ✅ **Fixed Path Switching**: Resolved bug when switching between paths.
- ✅ **Auto-heading**: Yaw automatically calculated based on the direction of movement.
- ✅ **Offboard at Startup**: Offboard mode activated immediately (hover only).
- ✅ **Intelligent Auto-arm**: Automatically arms on the first path or when the drone is landed.
- ✅ **Auto-disarm**: Automatically disarms when landed.

## 🚀 Quick Start

### Build
```bash
cd /your/ros2_workspace
colcon build --packages-select traj_interp
source install/setup.bash
```

### Launch
```bash
ros2 launch traj_interp trajectory_interpolator.launch.py
```

### Testing with Paths
```bash
# Square Path
ros2 topic pub /trajectory_path nav_msgs/Path '{
    header: {frame_id: "map"},
    poses: [
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 2.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
    ]
}' --once

# Linear Path
ros2 topic pub /trajectory_path nav_msgs/Path '{
    header: {frame_id: "map"},
    poses: [
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 3.0, y: 0.0, z: 5.0}, orientation: {w: 1.0}}}
    ]
}' --once
```

## 📊 Topics

### Input
- `/trajectory_path` (nav_msgs/Path) - Waypoints to be followed.
- `/px4/odometry/out` (nav_msgs/Odometry) - Drone position.

### Output  
- `fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode)
- `fmu/in/vehicle_command` (px4_msgs/VehicleCommand)
- `/px4/trajectory_setpoint_enu` (trajectory_msgs/MultiDOFJointTrajectoryPoint)
- `/trajectory_interpolator/status` (std_msgs/String)

## ⚙️ Configurable Parameters

File: `config/trajectory_interpolator.yaml`

```yaml
# Performance
ref_vel_max: 1.0          # Maximum velocity [m/s]
ref_acc_max: 1.0          # Maximum acceleration [m/s²]
ref_jerk_max: 2.0         # Maximum jerk [m/s³]

# Smoothness  
ref_omega: 1.0            # Filter frequency [rad/s] 
ref_zeta: 0.7             # Damping ratio

# Precision
waypoint_tolerance: 0.1   # Waypoint tolerance [m]
control_frequency: 50.0   # Control frequency [Hz]
```

## 🔧 Node States

- **IDLE**: Awaiting path.
- **FOLLOWING_TRAJECTORY**: Following trajectory.
- **STOPPED**: Stopped awaiting a new path (transition state).

## 🐛 Monitoring

```bash
# Node status
ros2 topic echo /trajectory_interpolator/status

# Interpolated setpoints
ros2 topic echo /px4/trajectory_setpoint_enu

# Received path
ros2 topic echo /trajectory_path
```

## 🔄 Second-Order Filter

The algorithm enforces physical limitations for:
- **Jerk** (rate of change of acceleration)
- **Acceleration**
- **Velocity**

Using a second-order filter:
```
acceleration = ω² × error - 2ζω × velocity
```