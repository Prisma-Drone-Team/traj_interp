# Trajectory Interpolator (traj_interp)

**Smooth trajectory interpolation and execution for drone control with PX4 integration.**

## Overview

The `traj_interp` package provides second-order filtered interpolation for smooth trajectory execution, converting discrete waypoints from the `path_planner` into continuous, smooth trajectories compatible with PX4 offboard control.

## Key Features

- ✅ **Second-Order Filter**: Smooth trajectory generation with physical constraints
- ✅ **Continuous Tracking**: Sequential waypoint following without stopping
- ✅ **Dynamic Path Management**: New paths immediately replace previous ones
- ✅ **PX4 Integration**: Standard interface using px4_msgs
- ✅ **Auto-heading**: Yaw automatically calculated from movement direction
- ✅ **Offboard Control**: Automatic offboard mode activation
- ✅ **Auto-arm/disarm**: Intelligent arming based on flight state
- ✅ **Velocity Control**: Configurable velocity profiles with acceleration limits

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
│   ├── planner_spline.h             # Second-order filter implementation
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
    # Physical Constraints
    ref_vel_max: 2.0          # Maximum velocity (m/s)
    ref_acc_max: 1.0          # Maximum acceleration (m/s²)
    ref_jerk_max: 2.0         # Maximum jerk (m/s³)
    
    # Second-Order Filter Parameters
    ref_omega: 1.0            # Natural frequency ω (rad/s)
    ref_zeta: 0.7             # Damping ratio ζ
    control_frequency: 50.0   # Control loop frequency (Hz)
    
    # Auto Control
    auto_arm: true            # Auto-arm on first trajectory
    auto_disarm: true         # Auto-disarm when landed
    auto_offboard: true       # Auto-offboard mode
    
    # Safety
    waypoint_tolerance: 0.2   # Goal reach tolerance (m)
    velocity_timeout: 10.0    # Velocity timeout (s)
    vertical_movement_threshold: 0.2  # Threshold to detect takeoff/landing movements
```

## Algorithms

### Second-Order Filter Interpolation
The system uses a second-order filter adapted from the lee_controller for smooth trajectory generation:

```cpp
// Second-order filter with physical constraints
acceleration = ω² × position_error - 2ζω × velocity
```

**Mathematical Model**:
- **Natural Frequency (ω)**: Controls response speed
- **Damping Ratio (ζ)**: Controls oscillation (ζ=0.7 for optimal response)
- **Physical Limits**: Enforces max velocity, acceleration, and jerk

**Benefits**:
- Smooth trajectories with physical constraints
- No overshoot with proper damping
- Real-time computation efficiency
- Automatic velocity and acceleration limiting

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
  ref_vel_max:=1.5 \
  ref_omega:=1.2 \
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
- Reduce `ref_vel_max` parameter
- Increase `ref_zeta` (damping ratio) for smoother response
- Decrease `ref_omega` (natural frequency) for slower response
- Check `control_frequency` (should be ≥50Hz)

### Auto-arm Issues
- Ensure drone is in position mode first
- Check `auto_arm: true` in configuration
- Verify vehicle status topic connectivity

### Takeoff Rotation Issues
- If drone rotates during takeoff, check `vertical_movement_threshold` parameter
- Default value `0.2` means horizontal movement must be >20% of vertical movement for yaw change
- Increase value (e.g., `0.5`) to make system more sensitive to vertical movements
- Decrease value (e.g., `0.1`) to make system less sensitive

## Performance

**Typical Performance**:
- Control Rate: 50Hz
- Filter Response: Second-order with configurable damping
- Position Accuracy: ±10cm
- Velocity Control: Configurable 0.5-5.0 m/s with acceleration limits

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
- **Filter Tuning**: Adjustable ω and ζ parameters for different flight characteristics

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

The algorithm enforces physical limitations using a second-order filter adapted from lee_controller:

**Mathematical Foundation**:
```
acceleration = ω² × (target_position - current_position) - 2ζω × current_velocity
```

**Physical Constraints**:
- **Jerk** (rate of change of acceleration): `ref_jerk_max`
- **Acceleration**: `ref_acc_max`
- **Velocity**: `ref_vel_max`

**Filter Parameters**:
- **ω (omega)**: Natural frequency - controls response speed
- **ζ (zeta)**: Damping ratio - controls oscillation (0.7 = critically damped)

**Tuning Guidelines**:
- Higher ω → Faster response, more aggressive
- Lower ω → Slower response, smoother
- ζ = 0.7 → Optimal balance (no overshoot)
- ζ < 0.7 → Oscillatory behavior
- ζ > 0.7 → Sluggish response