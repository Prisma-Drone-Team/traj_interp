# Trajectory Interpolator (traj_interp)

**Smooth trajectory interpolation and execution for drone control with PX4 integration.**

## Overview

The `traj_interp` package provides B-spline interpolation for smooth trajectory execution, converting discrete waypoints from the `path_planner` into continuous, smooth trajectories compatible with PX4 offboard control.

## Key Features

- ✅ **B-spline Interpolation**: Smooth trajectory generation from discrete waypoints
- ✅ **Continuous Tracking**: Sequential waypoint following without stopping
- ✅ **Dynamic Path Management**: New paths immediately replace previous ones
- ✅ **PX4 Integration**: Standard interface using px4_msgs
- ✅ **Auto-heading**: Yaw automatically calculated from movement direction
- ✅ **Offboard Control**: Automatic offboard mode activation
- ✅ **Auto-arm/disarm**: Intelligent arming based on flight state
- ✅ **Velocity Control**: Configurable velocity profiles

## Architecture

```
traj_interp/
├── src/
│   ├── trajectory_interpolator.cpp    # Main node
│   └── lib/
│       └── frame_transforms.cpp      # Transform utilities
├── include/traj_interp/
│   ├── trajectory_interpolator.hpp   # Node header  
│   ├── frame_transforms.h           # Transform utilities
│   ├── planner_spline.h             # B-spline interpolation
│   ├── planner.h                    # Base planner interface
│   ├── utils.h                      # General utilities
│   └── matrix/                      # Math matrix library
├── launch/
│   └── trajectory_interpolator.launch.py  # Launch file
└── config/
    └── trajectory_interpolator.yaml        # Parameters
```

## Trajectory Interpolator Node

**Executable**: `trajectory_interpolator`

### Topics

**Input**:
- `/trajectory_path` - Waypoint paths from `path_planner` or `babyk_drone_manager`
- `/px4/odometry/out` - Current drone state
- `/px4/vehicle_status/out` - PX4 vehicle status

**Output**:
- `/px4/trajectory_setpoint/in` - Smooth trajectory commands to PX4
- `/trajectory_interpolator/status` - Interpolator status
- `/trajectory_interpolator/debug` - Debug information

### States

- `IDLE` - Waiting for trajectories
- `INTERPOLATING` - Executing trajectory
- `COMPLETED` - Trajectory completed
- `ARMED` - Drone armed and ready
- `DISARMED` - Drone disarmed

## Configuration

### Main Parameters

```yaml
trajectory_interpolator:
  ros__parameters:
    # Interpolation
    max_velocity: 2.0          # Maximum velocity (m/s)
    max_acceleration: 1.0      # Maximum acceleration (m/s²)
    interpolation_rate: 50.0   # Control loop frequency (Hz)
    
    # B-spline Parameters  
    spline_order: 3           # B-spline order (3 = cubic)
    smoothing_factor: 0.1     # Trajectory smoothing
    
    # Auto Control
    auto_arm: true            # Auto-arm on first trajectory
    auto_disarm: true         # Auto-disarm when landed
    auto_offboard: true       # Auto-offboard mode
    
    # Safety
    position_tolerance: 0.2   # Goal reach tolerance (m)
    velocity_timeout: 10.0    # Velocity timeout (s)
```

## Algorithms

### B-spline Interpolation
The system uses cubic B-splines for smooth trajectory generation:

```cpp
// Cubic B-spline interpolation with velocity constraints
SplineTrajectory spline = generateBSpline(waypoints, max_velocity, max_acceleration);
```

**Benefits**:
- Smooth C² continuous trajectories
- Velocity and acceleration constraints
- Automatic heading calculation
- Real-time trajectory updates

### PX4 Integration
- **Offboard Mode**: Automatic activation on trajectory start
- **Trajectory Setpoints**: Standard PX4 trajectory interface
- **Vehicle Commands**: Arm/disarm and mode switching
- **Status Monitoring**: Real-time vehicle state tracking

## System Integration

The trajectory interpolator works with:

1. **babyk_drone_manager**: Receives direct trajectories for go/takeoff/land commands
2. **path_planner**: Receives planned paths with obstacle avoidance
3. **PX4**: Sends smooth trajectory setpoints for execution
4. **RTABMap**: Uses odometry for position feedback

## Launch

### Basic Launch
```bash
ros2 launch traj_interp trajectory_interpolator.launch.py
```

### With Custom Configuration
```bash
ros2 launch traj_interp trajectory_interpolator.launch.py \
  config_file:=config/trajectory_interpolator.yaml \
  max_velocity:=1.5 \
  auto_arm:=true
```

## Usage Examples

### Square Pattern
```bash
ros2 topic pub /trajectory_path nav_msgs/Path '{
    header: {frame_id: "map"},
    poses: [
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 2.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 2.0, z: 2.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 2.0, z: 2.0}}}
    ]
}' --once
```

### Linear Path
```bash
ros2 topic pub /trajectory_path nav_msgs/Path '{
    header: {frame_id: "map"},
    poses: [
        {header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 2.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 2.0}}},
        {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 0.0, z: 2.0}}}
    ]
}' --once
```

## Troubleshooting

### Drone Not Following Trajectory
- Check offboard mode: `ros2 topic echo /px4/vehicle_status/out`
- Verify trajectory topic: `ros2 topic echo /trajectory_path`
- Check interpolator status: `ros2 topic echo /trajectory_interpolator/status`

### Jerky Movement
- Reduce `max_velocity` parameter
- Increase `smoothing_factor`
- Check `interpolation_rate` (should be ≥50Hz)

### Auto-arm Issues
- Ensure drone is in position mode first
- Check `auto_arm: true` in configuration
- Verify vehicle status topic connectivity

## Performance

**Typical Performance**:
- Control Rate: 50Hz
- Trajectory Smoothness: C² continuous
- Position Accuracy: ±10cm
- Velocity Control: Configurable 0.5-5.0 m/s

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select traj_interp
source install/setup.bash
```

## Dependencies

**ROS 2 Packages**:
- `nav_msgs`, `geometry_msgs`, `trajectory_msgs`
- `px4_msgs` (PX4 communication)
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

**Libraries**:
- Eigen3 (matrix operations)
- Custom matrix library (included)

## Development Notes

- **Real-time Safe**: All operations designed for real-time execution
- **Memory Efficient**: Minimal dynamic allocation during flight
- **Thread Safe**: Proper mutex protection for shared state
- **PX4 Compatible**: Follows PX4 offboard control standards

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