#include "trajectory_interpolator.hpp"
#include <cmath>

TrajectoryInterpolator::TrajectoryInterpolator() : rclcpp::Node("trajectory_interpolator"), _state(IDLE) {
    
    // Declare parameters
    this->declare_parameter("path_topic", "/trajectory_path");
    _path_topic = this->get_parameter("path_topic").as_string();
    RCLCPP_INFO(get_logger(), "path_topic: %s", _path_topic.c_str());
    
    this->declare_parameter("odometry_topic", "/px4/odometry/out");
    _odometry_topic = this->get_parameter("odometry_topic").as_string();
    RCLCPP_INFO(get_logger(), "odometry_topic: %s", _odometry_topic.c_str());
    
    this->declare_parameter("offboard_control_mode_topic", "fmu/in/offboard_control_mode");
    _offboard_control_mode_topic = this->get_parameter("offboard_control_mode_topic").as_string();
    RCLCPP_INFO(get_logger(), "offboard_control_mode_topic: %s", _offboard_control_mode_topic.c_str());
    
    this->declare_parameter("vehicle_command_topic", "fmu/in/vehicle_command");
    _vehicle_command_topic = this->get_parameter("vehicle_command_topic").as_string();
    RCLCPP_INFO(get_logger(), "vehicle_command_topic: %s", _vehicle_command_topic.c_str());
    
    this->declare_parameter("trajectory_setpoint_topic", "/px4/trajectory_setpoint_enu");
    _trajectory_setpoint_topic = this->get_parameter("trajectory_setpoint_topic").as_string();
    RCLCPP_INFO(get_logger(), "trajectory_setpoint_topic: %s", _trajectory_setpoint_topic.c_str());
    
    this->declare_parameter("status_topic", "/trajectory_interpolator/status");
    _status_topic = this->get_parameter("status_topic").as_string();
    RCLCPP_INFO(get_logger(), "status_topic: %s", _status_topic.c_str());
    
    // Filter parameters
    this->declare_parameter("ref_jerk_max", 2.0);
    _ref_jerk_max = this->get_parameter("ref_jerk_max").as_double();
    
    this->declare_parameter("ref_acc_max", 1.0);
    _ref_acc_max = this->get_parameter("ref_acc_max").as_double();
    
    this->declare_parameter("ref_vel_max", 1.0);
    _ref_vel_max = this->get_parameter("ref_vel_max").as_double();
    
    this->declare_parameter("ref_omega", 1.0);
    _ref_omega = this->get_parameter("ref_omega").as_double();
    
    this->declare_parameter("ref_zeta", 0.7);
    _ref_zeta = this->get_parameter("ref_zeta").as_double();
    
    this->declare_parameter("ref_yaw_jerk_max", 1.0);
    _ref_yaw_jerk_max = this->get_parameter("ref_yaw_jerk_max").as_double();
    
    this->declare_parameter("ref_yaw_acc_max", 0.5);
    _ref_yaw_acc_max = this->get_parameter("ref_yaw_acc_max").as_double();
    
    this->declare_parameter("ref_yaw_vel_max", 0.5);
    _ref_yaw_vel_max = this->get_parameter("ref_yaw_vel_max").as_double();
    
    this->declare_parameter("control_frequency", 50.0);
    _control_frequency = this->get_parameter("control_frequency").as_double();
    _dt = 1.0 / _control_frequency;
    
    this->declare_parameter("waypoint_tolerance", 0.1);
    _waypoint_tolerance = this->get_parameter("waypoint_tolerance").as_double();
    
    this->declare_parameter("yaw_tolerance", 0.1);
    _yaw_tolerance = this->get_parameter("yaw_tolerance").as_double();
    
    // TF parameters (like trajectory_planner)
    this->declare_parameter("parent_transform", "map");
    _parent_transf = this->get_parameter("parent_transform").as_string();
    
    this->declare_parameter("child_transform", "odom");
    _child_transf = this->get_parameter("child_transform").as_string();
    
    this->declare_parameter("do_transform", true);
    _do_transform = this->get_parameter("do_transform").as_bool();
    
    this->declare_parameter("tf_buffer_timeout", 0.5);
    _tf_buffer_timeout = this->get_parameter("tf_buffer_timeout").as_double();
    
    RCLCPP_INFO(get_logger(), "TF: %s -> %s, do_transform: %s", 
                _parent_transf.c_str(), _child_transf.c_str(), _do_transform ? "true" : "false");
    
    // Initialize publishers
    _offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>(_offboard_control_mode_topic, 10);
    _vehicle_command_publisher = this->create_publisher<VehicleCommand>(_vehicle_command_topic, 10);
    _trajectory_setpoint_publisher = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(_trajectory_setpoint_topic, 10);
    _status_publisher = this->create_publisher<std_msgs::msg::String>(_status_topic, 10);
    
    // Initialize subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    _path_subscription = this->create_subscription<nav_msgs::msg::Path>(
        _path_topic, 10, std::bind(&TrajectoryInterpolator::path_callback, this, std::placeholders::_1));
    
    _odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        _odometry_topic, qos, std::bind(&TrajectoryInterpolator::odometry_callback, this, std::placeholders::_1));
    
    _vehicle_control_mode_subscription = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos, 
        std::bind(&TrajectoryInterpolator::vehicle_control_mode_callback, this, std::placeholders::_1));
    
    _land_detected_subscription = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", qos,
        std::bind(&TrajectoryInterpolator::land_detected_callback, this, std::placeholders::_1));
    
    // Initialize timers
    _control_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / _control_frequency)),
        std::bind(&TrajectoryInterpolator::control_timer_callback, this));
    
    _status_timer = this->create_wall_timer(
        1s, std::bind(&TrajectoryInterpolator::status_timer_callback, this));
    
    // Initialize TF2
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    
    // Start TF lookup thread (like trajectory_planner)
    std::thread tf_thread(&TrajectoryInterpolator::tf_lookup_loop, this);
    tf_thread.detach();
    
    // Initialize reference values
    _ref_position.setZero();
    _ref_velocity.setZero();
    _ref_acceleration.setZero();
    _cmd_position.setZero();
    
    RCLCPP_INFO(get_logger(), "TrajectoryInterpolator node initialized");
}

void TrajectoryInterpolator::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received new path with %zu waypoints", msg->poses.size());
    
    if (msg->poses.empty()) {
        RCLCPP_WARN(get_logger(), "Received empty path, stopping trajectory");
        _state = STOPPED;
        clear_waypoint_queue();
        _has_target = false;
        return;
    }
    
    // Always clear current waypoint queue and reset state for new path
    clear_waypoint_queue();
    _has_target = false;
    
    // Resample the path to have more waypoints (every 40cm)
    std::vector<geometry_msgs::msg::PoseStamped> resampled_poses = resample_path(msg->poses, 0.4);
    
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        for (const auto& pose : resampled_poses) {
            // Transform each waypoint from map to odom frame
            geometry_msgs::msg::PoseStamped transformed_pose = transform_pose_to_odom(pose);
            if (transformed_pose.header.frame_id != "") {  // Check if transformation was successful
                _waypoint_queue.push(transformed_pose);
            } else {
                RCLCPP_WARN(get_logger(), "Failed to transform waypoint from map to odom, skipping");
            }
        }
    }
    
    // Always start trajectory with first waypoint (regardless of current state)
    if (!_waypoint_queue.empty()) {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        _current_target = _waypoint_queue.front();
        _waypoint_queue.pop();
        _has_target = true;
        
        Eigen::Vector3f target_pos(
            _current_target.pose.position.x,
            _current_target.pose.position.y,
            _current_target.pose.position.z
        );
        
        // Calculate heading direction automatically from current position to target
        float target_yaw = calculate_heading_yaw(_current_position, target_pos);
        
        set_new_target(target_pos, target_yaw);
        _state = FOLLOWING_TRAJECTORY;
        
        // Solo resettare il contatore se non siamo armati per evitare loop infiniti
        if (!_armed) {
            _offboard_setpoint_counter = 0;
            RCLCPP_INFO(get_logger(), "Starting new trajectory (resetting counter for unarmed drone)");
        } else {
            RCLCPP_INFO(get_logger(), "Starting new trajectory (maintaining counter for armed drone)");
        }
        
        // Engage offboard mode immediately when starting trajectory (in case not already)
        if (!_offboard_mode) {
            RCLCPP_INFO(get_logger(), "Engaging offboard mode for trajectory execution");
            engage_offboard_mode();
        }
    } else {
        RCLCPP_ERROR(get_logger(), "No valid waypoints after transformation, cannot start trajectory");
        _state = STOPPED;
    }
        
    // Mark that we received the first path (for arming logic)
    if (!_first_path_received) {
        _first_path_received = true;
        RCLCPP_INFO(get_logger(), "First path received - arming will be enabled");
    }
}

void TrajectoryInterpolator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    _current_attitude = Eigen::Quaternionf(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    
    _current_position = Eigen::Vector3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    
    // Initialize reference values on first odometry message
    if (!_first_odom_received) {
        _ref_position = _current_position;
        _cmd_position = _current_position;
        _ref_yaw = extract_yaw_from_quaternion(_current_attitude);
        _cmd_yaw = _ref_yaw;
        _first_odom_received = true;
        
        // Engage offboard mode immediately at startup (without arming)
        if (!_offboard_mode) {
            RCLCPP_INFO(get_logger(), "Engaging offboard mode at startup");
            engage_offboard_mode();
        }
        
        RCLCPP_INFO(get_logger(), "Initialized reference position: [%.3f, %.3f, %.3f], yaw: %.3f",
                   _ref_position(0), _ref_position(1), _ref_position(2), _ref_yaw);
    }
}

void TrajectoryInterpolator::control_timer_callback() {
    if (!_first_odom_received) {
        return;
    }
    
    // Always publish offboard control mode to maintain offboard
    // publish_offboard_control_mode();
    
    // Increment setpoint counter
    _offboard_setpoint_counter++;
    
    // Only proceed with trajectory interpolation and trajectory setpoint if we have a target
    if (_has_target && _state == FOLLOWING_TRAJECTORY) {
        // Interpolate trajectory using ffilter algorithm
        interpolate_trajectory();
        
        // Publish trajectory setpoint
        publish_trajectory_setpoint();
    } else {
        // Publish current position as setpoint to maintain stability
        publish_trajectory_setpoint();
    }
    
    // Auto-arm when:
    // 1. First path ever received (initial mission), OR
    // 2. Drone is landed and new trajectory started (new mission after landing)
    // This ensures arming happens for first mission and any new mission after landing
    if (_has_target && _state == FOLLOWING_TRAJECTORY && !_armed && _landed && 
        _offboard_setpoint_counter > OFFBOARD_SETPOINTS_REQUIRED && 
        (_first_path_received || _landed)) {
        
        if (_first_path_received) {
            RCLCPP_INFO(get_logger(), "Auto-arming vehicle after %lu setpoints (first path received)", _offboard_setpoint_counter);
        } else {
            RCLCPP_INFO(get_logger(), "Auto-arming vehicle after %lu setpoints (drone was landed)", _offboard_setpoint_counter);
        }
        arm();
    }
    
    // Note: Auto-disarm logic is handled in land_detected_callback
    // when actual landing is detected during flight
    
    // Check if we should advance to next waypoint (at half distance, not full reach)
    if (_has_target && _state == FOLLOWING_TRAJECTORY) {
        Eigen::Vector3f error = _cmd_position - _ref_position;
        float distance_error = error.norm();
        
        // Calculate distance to target for smooth transition logic
        Eigen::Vector3f target_error = _cmd_position - _current_position;
        float distance_to_target = target_error.norm();
        
        // Check if we have next waypoint in queue and we're at ~half distance
        bool should_advance_waypoint = false;
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            if (!_waypoint_queue.empty() && distance_to_target <= _waypoint_tolerance * 2.0) {
                should_advance_waypoint = true;
            }
        }
        
        // Advance to next waypoint when at half distance (for continuous flow)
        if (should_advance_waypoint) {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            if (!_waypoint_queue.empty()) {
                _current_target = _waypoint_queue.front();
                _waypoint_queue.pop();
                
                // Transform the new waypoint with latest TF
                geometry_msgs::msg::PoseStamped transformed_target = transform_pose_to_odom(_current_target);
                if (transformed_target.header.frame_id != "") {
                    // Update target with transformed coordinates
                    Eigen::Vector3f target_pos(
                        transformed_target.pose.position.x,
                        transformed_target.pose.position.y,
                        transformed_target.pose.position.z
                    );
                    
                    // Calculate heading direction automatically from current reference position to target
                    float target_yaw = calculate_heading_yaw(_ref_position, target_pos);
                    
                    set_new_target(target_pos, target_yaw);
                    
                    RCLCPP_INFO(get_logger(), "Advanced to next waypoint (at half distance): [%.3f, %.3f, %.3f], yaw: %.3f", 
                               target_pos(0), target_pos(1), target_pos(2), target_yaw);
                } else {
                    RCLCPP_WARN(get_logger(), "Failed to transform next waypoint, skipping");
                }
            }
        }
        
        // Only check for final completion when no more waypoints and close to last target
        float yaw_error = std::abs(_cmd_yaw - _ref_yaw);
        if (yaw_error > M_PI) {
            yaw_error = 2.0f * M_PI - yaw_error;
        }
        
        bool queue_empty = false;
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            queue_empty = _waypoint_queue.empty();
        }
        
        // Complete trajectory only when no more waypoints AND very close to final target
        if (queue_empty && distance_error < _waypoint_tolerance && yaw_error < _yaw_tolerance) {
            _has_target = false;
            _state = IDLE;
            RCLCPP_INFO(get_logger(), "Trajectory completed - reached final waypoint [%.3f, %.3f, %.3f]", 
                       _cmd_position(0), _cmd_position(1), _cmd_position(2));
        }
    }
}

void TrajectoryInterpolator::status_timer_callback() {
    std_msgs::msg::String status_msg;
    
    switch (_state) {
        case IDLE:
            status_msg.data = "IDLE";
            break;
        case FOLLOWING_TRAJECTORY:
            status_msg.data = "FOLLOWING_TRAJECTORY";
            break;
        case STOPPED:
            status_msg.data = "STOPPED";
            break;
        default:
            status_msg.data = "UNKNOWN";
            break;
    }
    
    _status_publisher->publish(status_msg);
}

void TrajectoryInterpolator::interpolate_trajectory() {   
    // Position interpolation
    Eigen::Vector3f position_error = _cmd_position - _ref_position;
    Eigen::Vector3f desired_acceleration;
    
    for (int i = 0; i < 3; i++) {
        desired_acceleration(i) = _ref_omega * _ref_omega * position_error(i) - 
                                2.0 * _ref_zeta * _ref_omega * _ref_velocity(i);
        
        // Jerk limiting
        float jerk = (desired_acceleration(i) - _ref_acceleration(i)) / _dt;
        if (std::abs(jerk) > _ref_jerk_max) {
            jerk = (jerk > 0.0) ? _ref_jerk_max : -_ref_jerk_max;
        }
        
        desired_acceleration(i) = _ref_acceleration(i) + jerk * _dt;
        
        // Acceleration limiting
        if (std::abs(desired_acceleration(i)) > _ref_acc_max) {
            _ref_acceleration(i) = (desired_acceleration(i) > 0.0) ? _ref_acc_max : -_ref_acc_max;
        } else {
            _ref_acceleration(i) = desired_acceleration(i);
        }
        
        // Velocity integration and limiting
        float desired_velocity = _ref_velocity(i) + _ref_acceleration(i) * _dt;
        if (std::abs(desired_velocity) > _ref_vel_max) {
            _ref_velocity(i) = (desired_velocity > 0.0) ? _ref_vel_max : -_ref_vel_max;
        } else {
            _ref_velocity(i) = desired_velocity;
        }
        
        // Position integration
        _ref_position(i) += _ref_velocity(i) * _dt;
    }
    
    // Yaw interpolation
    float yaw_error = _cmd_yaw - _ref_yaw;
    if (std::abs(yaw_error) > M_PI) {
        yaw_error = yaw_error - 2.0f * M_PI * ((yaw_error > 0) ? 1 : -1);
    }
    
    float desired_yaw_acceleration = _ref_omega * _ref_omega * yaw_error - 
                                   2.0 * _ref_zeta * _ref_omega * _ref_yaw_rate;
    
    // Yaw jerk limiting
    float yaw_jerk = (desired_yaw_acceleration - _ref_yaw_acc) / _dt;
    if (std::abs(yaw_jerk) > _ref_yaw_jerk_max) {
        yaw_jerk = (yaw_jerk > 0.0) ? _ref_yaw_jerk_max : -_ref_yaw_jerk_max;
    }
    
    desired_yaw_acceleration = _ref_yaw_acc + yaw_jerk * _dt;
    
    // Yaw acceleration limiting
    if (std::abs(desired_yaw_acceleration) > _ref_yaw_acc_max) {
        _ref_yaw_acc = (desired_yaw_acceleration > 0.0) ? _ref_yaw_acc_max : -_ref_yaw_acc_max;
    } else {
        _ref_yaw_acc = desired_yaw_acceleration;
    }
    
    // Yaw velocity integration and limiting
    float desired_yaw_rate = _ref_yaw_rate + _ref_yaw_acc * _dt;
    if (std::abs(desired_yaw_rate) > _ref_yaw_vel_max) {
        _ref_yaw_rate = (desired_yaw_rate > 0.0) ? _ref_yaw_vel_max : -_ref_yaw_vel_max;
    } else {
        _ref_yaw_rate = desired_yaw_rate;
    }
    
    // Yaw integration
    _ref_yaw += _ref_yaw_rate * _dt;
    
    // Normalize yaw to [-pi, pi]
    while (_ref_yaw > M_PI) _ref_yaw -= 2.0 * M_PI;
    while (_ref_yaw < -M_PI) _ref_yaw += 2.0 * M_PI;
}

void TrajectoryInterpolator::set_new_target(const Eigen::Vector3f& target_pos, float target_yaw) {
    _cmd_position = target_pos;
    _cmd_yaw = target_yaw;
}

void TrajectoryInterpolator::clear_waypoint_queue() {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    while (!_waypoint_queue.empty()) {
        _waypoint_queue.pop();
    }
    
    // Stop current trajectory if following one
    if (_state == FOLLOWING_TRAJECTORY) {
        _state = STOPPED;
        // Set current reference position as command to stop smoothly
        _cmd_position = _ref_position;
        _cmd_yaw = _ref_yaw;
        _has_target = false;
        // Solo azzerare il contatore se non siamo armati - evita loop infiniti
        if (!_armed) {
            _offboard_setpoint_counter = 0;
            RCLCPP_INFO(get_logger(), "Trajectory stopped, new path received (resetting counter)");
        } else {
            RCLCPP_INFO(get_logger(), "Trajectory stopped, new path received (maintaining counter for armed drone)");
        }
    }
}

void TrajectoryInterpolator::publish_vehicle_command(uint16_t command, float param1, float param2, float param3) {
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _vehicle_command_publisher->publish(msg);
}

void TrajectoryInterpolator::publish_trajectory_setpoint() {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg{};
    
    // Position
    geometry_msgs::msg::Transform transform;
    transform.translation.x = _ref_position(0);
    transform.translation.y = _ref_position(1);
    transform.translation.z = _ref_position(2);
    
    // Orientation (from yaw)
    Eigen::Quaternionf q_ref(Eigen::AngleAxisf(_ref_yaw, Eigen::Vector3f::UnitZ()));
    transform.rotation.w = q_ref.w();
    transform.rotation.x = q_ref.x();
    transform.rotation.y = q_ref.y();
    transform.rotation.z = q_ref.z();
    
    msg.transforms.push_back(transform);
    
    // Velocity
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = _ref_velocity(0);
    velocity.linear.y = _ref_velocity(1);
    velocity.linear.z = _ref_velocity(2);
    velocity.angular.z = _ref_yaw_rate;
    
    msg.velocities.push_back(velocity);
    
    // Acceleration
    geometry_msgs::msg::Twist acceleration;
    acceleration.linear.x = _ref_acceleration(0);
    acceleration.linear.y = _ref_acceleration(1);
    acceleration.linear.z = _ref_acceleration(2);
    acceleration.angular.z = _ref_yaw_acc;
    
    msg.accelerations.push_back(acceleration);
    
    _trajectory_setpoint_publisher->publish(msg);
}

// void TrajectoryInterpolator::publish_offboard_control_mode() {
//     OffboardControlMode msg{};
//     msg.position = true;
//     msg.velocity = false;
//     msg.acceleration = false;
//     msg.attitude = false;
//     msg.body_rate = false;
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     _offboard_control_mode_publisher->publish(msg);
// }

float TrajectoryInterpolator::calculate_heading_yaw(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& target_pos) {
    // Calculate direction vector from current to target position
    Eigen::Vector3f direction = target_pos - current_pos;
    
    // Calculate yaw angle from direction vector (only considering X-Y plane)
    // atan2(y, x) gives angle from positive X-axis to direction vector
    float yaw = std::atan2(direction.y(), direction.x());
    
    return yaw;
}

float TrajectoryInterpolator::extract_yaw_from_quaternion(const Eigen::Quaternionf& q) {
    // Extract yaw from quaternion using rotation matrix
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();
    
    // Yaw is atan2 of rotation matrix elements (2,1) and (2,2) for Z-Y-X Euler
    // For a simpler approach, use atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
                          1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    
    return yaw;
}

// PX4 State callbacks
void TrajectoryInterpolator::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    _armed = msg->flag_armed;
    _offboard_mode = msg->flag_control_offboard_enabled;
}

void TrajectoryInterpolator::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    bool was_landed = _landed;
    _landed = msg->landed;
    
    // If drone just landed while armed (was flying), stop trajectory and disarm
    if (_landed && !was_landed && _armed) {
        RCLCPP_INFO(get_logger(), "Landing detected during flight - stopping trajectory and disarming");
        
        // Stop any active trajectory
        if (_state == FOLLOWING_TRAJECTORY) {
            _state = IDLE;
            _has_target = false;
            clear_waypoint_queue();
        }
        
        // Auto-disarm after landing
        disarm();
    }
}

// PX4 Commands
void TrajectoryInterpolator::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command sent");
}

void TrajectoryInterpolator::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(get_logger(), "Disarm command sent");
}

void TrajectoryInterpolator::engage_offboard_mode() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(get_logger(), "Switching to offboard mode");
}

geometry_msgs::msg::PoseStamped TrajectoryInterpolator::transform_pose_to_odom(const geometry_msgs::msg::PoseStamped& pose_in_map) {
    geometry_msgs::msg::PoseStamped transformed_pose = pose_in_map;  // Copy input
    
    if (_do_transform) {
        try {
            // Transform position using tf2::doTransform like trajectory_planner
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.stamp = this->get_clock()->now();
            point_in.header.frame_id = _parent_transf;  // "map"
            point_in.point.x = pose_in_map.pose.position.x;
            point_in.point.y = pose_in_map.pose.position.y;
            point_in.point.z = pose_in_map.pose.position.z;
            
            tf2::doTransform(point_in, point_out, _tf_map_to_odom);
            
            // Update the transformed pose
            transformed_pose.header.frame_id = _child_transf;  // "odom"
            transformed_pose.header.stamp = this->get_clock()->now();
            transformed_pose.pose.position.x = point_out.point.x;
            transformed_pose.pose.position.y = point_out.point.y;
            transformed_pose.pose.position.z = point_out.point.z;
            
            RCLCPP_INFO(get_logger(), "Transformed waypoint from [%.3f, %.3f, %.3f] in %s to [%.3f, %.3f, %.3f] in %s",
                        pose_in_map.pose.position.x, pose_in_map.pose.position.y, pose_in_map.pose.position.z,
                        _parent_transf.c_str(),
                        transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z,
                        _child_transf.c_str());
                        
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 1000, 
                                 "Transform failed: %s", ex.what());
            transformed_pose.header.frame_id = "";  // Mark as failed
        }
    } else {
        // No transform, just change frame_id to odom
        transformed_pose = pose_in_map;  // Copy the entire pose
        transformed_pose.header.frame_id = _child_transf;
        transformed_pose.header.stamp = this->get_clock()->now();
        RCLCPP_INFO(get_logger(), "Transform disabled, using pose as-is in %s frame: [%.3f, %.3f, %.3f]",
                    _child_transf.c_str(),
                    transformed_pose.pose.position.x, transformed_pose.pose.position.y, transformed_pose.pose.position.z);
    }
    
    return transformed_pose;
}

void TrajectoryInterpolator::tf_lookup_loop() {
    RCLCPP_INFO(get_logger(), "TF lookup thread started");
    
    rclcpp::Rate rate(100);  // 100 Hz like trajectory_planner
    while (rclcpp::ok()) {
        try {
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Duration timeout = rclcpp::Duration::from_seconds(_tf_buffer_timeout);
            _tf_map_to_odom = _tf_buffer->lookupTransform(_child_transf, _parent_transf, now, timeout);
            _tf_odom_to_map = _tf_buffer->lookupTransform(_parent_transf, _child_transf, now, timeout);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Transform error: %s", ex.what());
        }
        rate.sleep();
    }
}

std::vector<geometry_msgs::msg::PoseStamped> TrajectoryInterpolator::resample_path(
    const std::vector<geometry_msgs::msg::PoseStamped>& original_path, double sampling_distance) {
    
    if (original_path.empty()) {
        return {};
    }
    
    if (original_path.size() == 1) {
        return original_path;  // Single waypoint, nothing to resample
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> resampled_path;
    
    // Always include the first waypoint
    resampled_path.push_back(original_path[0]);
    
    for (size_t i = 0; i < original_path.size() - 1; i++) {
        const auto& current_pose = original_path[i];
        const auto& next_pose = original_path[i + 1];
        
        // Calculate distance between current and next waypoint
        double dx = next_pose.pose.position.x - current_pose.pose.position.x;
        double dy = next_pose.pose.position.y - current_pose.pose.position.y;
        double dz = next_pose.pose.position.z - current_pose.pose.position.z;
        double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (segment_length <= sampling_distance) {
            // Segment is short enough, no need to subdivide
            continue;
        }
        
        // Calculate number of intermediate waypoints needed
        int num_segments = static_cast<int>(std::ceil(segment_length / sampling_distance));
        double actual_step = segment_length / num_segments;
        
        // Generate intermediate waypoints
        for (int j = 1; j < num_segments; j++) {
            geometry_msgs::msg::PoseStamped intermediate_pose;
            intermediate_pose.header = current_pose.header;
            
            double ratio = (j * actual_step) / segment_length;
            
            // Linear interpolation of position
            intermediate_pose.pose.position.x = current_pose.pose.position.x + ratio * dx;
            intermediate_pose.pose.position.y = current_pose.pose.position.y + ratio * dy;
            intermediate_pose.pose.position.z = current_pose.pose.position.z + ratio * dz;
            
            // SLERP interpolation for orientation
            tf2::Quaternion q1, q2, q_interpolated;
            tf2::fromMsg(current_pose.pose.orientation, q1);
            tf2::fromMsg(next_pose.pose.orientation, q2);
            q_interpolated = q1.slerp(q2, ratio);
            intermediate_pose.pose.orientation = tf2::toMsg(q_interpolated);
            
            resampled_path.push_back(intermediate_pose);
        }
    }
    
    // Always include the last waypoint
    resampled_path.push_back(original_path.back());
    
    RCLCPP_INFO(get_logger(), "Resampled path: %zu → %zu waypoints (sampling distance: %.2f m)", 
                original_path.size(), resampled_path.size(), sampling_distance);
    
    return resampled_path;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryInterpolator>());
    rclcpp::shutdown();
    return 0;
}
