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
    
    // Z-axis specific parameters (optional, fallback to XY if not provided)
    this->declare_parameter("ref_jerk_max_z", _ref_jerk_max);
    _ref_jerk_max_z = this->get_parameter("ref_jerk_max_z").as_double();
    
    this->declare_parameter("ref_acc_max_z", _ref_acc_max);
    _ref_acc_max_z = this->get_parameter("ref_acc_max_z").as_double();
    
    this->declare_parameter("ref_vel_max_z", _ref_vel_max);
    _ref_vel_max_z = this->get_parameter("ref_vel_max_z").as_double();
    
    this->declare_parameter("ref_omega_z", _ref_omega);
    _ref_omega_z = this->get_parameter("ref_omega_z").as_double();
    
    this->declare_parameter("ref_zeta_z", _ref_zeta);
    _ref_zeta_z = this->get_parameter("ref_zeta_z").as_double();
    
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
    
    this->declare_parameter("vertical_movement_threshold", 0.2);
    _vertical_movement_threshold = this->get_parameter("vertical_movement_threshold").as_double();
    
    this->declare_parameter("resampling_distance", 0.3);  // NEW: Configurable resampling distance (30cm default)
    _resampling_distance = this->get_parameter("resampling_distance").as_double();
    
    // TF parameters (like trajectory_planner)
    this->declare_parameter("parent_transform", "map");
    _parent_transf = this->get_parameter("parent_transform").as_string();
    
    this->declare_parameter("child_transform", "odom");
    _child_transf = this->get_parameter("child_transform").as_string();
    
    this->declare_parameter("do_transform", true);
    _do_transform = this->get_parameter("do_transform").as_bool();
    
    this->declare_parameter("tf_buffer_timeout", 0.5);
    _tf_buffer_timeout = this->get_parameter("tf_buffer_timeout").as_double();
    
    this->declare_parameter("loiter_segment", 8);
    _loiter_segment = this->get_parameter("loiter_segment").as_int();

    this->declare_parameter("tilting_goal_pitch_deg", 20.0);
    _tilting_goal_pitch = this->get_parameter("tilting_goal_pitch_deg").as_double() * M_PI / 180.0; // radianti
    this->declare_parameter("tilting_interp_rate", 0.1); // rad/s
    _tilting_interp_rate = this->get_parameter("tilting_interp_rate").as_double();

    this->declare_parameter("tilting_on_pitch_enabled", true);
    _tilting_on_pitch_enabled = this->get_parameter("tilting_on_pitch_enabled").as_bool();

    this->declare_parameter("choose_final_yaw", true);
    _choose_final_yaw = this->get_parameter("choose_final_yaw").as_bool();
    
    _current_pitch = 0.0;

    RCLCPP_INFO(get_logger(), "TF: %s -> %s, do_transform: %s", 
                _parent_transf.c_str(), _child_transf.c_str(), _do_transform ? "true" : "false");
    
    // Log filter parameters
    RCLCPP_INFO(get_logger(), "Filter params - XY: omega=%.2f, zeta=%.2f, vel_max=%.2f | Z: omega=%.2f, zeta=%.2f, vel_max=%.2f", 
                _ref_omega, _ref_zeta, _ref_vel_max, _ref_omega_z, _ref_zeta_z, _ref_vel_max_z);
    
    // Initialize publishers
    _offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>(_offboard_control_mode_topic, 10);
    _vehicle_command_publisher = this->create_publisher<VehicleCommand>(_vehicle_command_topic, 10);
    _trajectory_setpoint_publisher = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(_trajectory_setpoint_topic, 10);
    _status_publisher = this->create_publisher<std_msgs::msg::String>(_status_topic, 10);
    _transformed_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/trajectory_interpolator/transformed_path", 10);
    _debug_publisher = this->create_publisher<std_msgs::msg::String>("/traj_interp/completed", 10);
    _tilting_pub = this->create_publisher<px4_msgs::msg::TiltingMcDesiredAngles>("fmu/in/tilting_mc_desired_angles", 10);
    
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
    
    _path_mode_subscription = this->create_subscription<std_msgs::msg::String>(
        "/move_manager/path_mode", 10,
        std::bind(&TrajectoryInterpolator::path_mode_callback, this, std::placeholders::_1));

    // Teleop coordination subscribers
    _teleop_active_subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/move_manager/teleop_active", 10,
        std::bind(&TrajectoryInterpolator::teleop_active_callback, this, std::placeholders::_1));
    
    _velocity_increments_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop/velocity_increments", 10,
        std::bind(&TrajectoryInterpolator::velocity_increments_callback, this, std::placeholders::_1));

    // Initialize timers
    _control_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / _control_frequency)),
        std::bind(&TrajectoryInterpolator::control_timer_callback, this));
    
    _status_timer = this->create_wall_timer(
        1s, std::bind(&TrajectoryInterpolator::status_timer_callback, this));
    
    // Initialize TF2
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    
    // Start TF lookup thread
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
    
    // Reset transformed path for new trajectory
    _transformed_path.poses.clear();
    _transformed_path.header.frame_id = "odom";
    _transformed_path.header.stamp = this->get_clock()->now();
    
    // Publish empty path to clear previous visualization
    _transformed_path_publisher->publish(_transformed_path);
    RCLCPP_INFO(get_logger(), "Reset and published empty transformed path for new trajectory");
    
    //
    // Resample the path to have more waypoints (configurable distance)
    // std::vector<geometry_msgs::msg::PoseStamped> resampled_poses = resample_path(msg->poses, _resampling_distance);
    //

    std::vector<geometry_msgs::msg::PoseStamped> resampled_poses;
    int loiter_segment = _loiter_segment;
    int total_points = msg->poses.size();

    if (_path_mode == "circle" && total_points > loiter_segment) {
        // Divide il path in due parti
        std::vector<geometry_msgs::msg::PoseStamped> approach_path(
            msg->poses.begin(), msg->poses.end() - loiter_segment);
            if (!approach_path.empty() && approach_path.size() >= 2) {
                _approach_penultimate = approach_path[approach_path.size() - 2];
            } else if (!approach_path.empty()) {
                _approach_penultimate = approach_path.back();
            }
        std::vector<geometry_msgs::msg::PoseStamped> circle_path(
            msg->poses.end() - loiter_segment, msg->poses.end());

        // Resample separatamente
        auto resampled_approach = resample_path(approach_path, _resampling_distance);
        auto resampled_circle = resample_path(circle_path, _resampling_distance);

        // Salva l'indice di inizio tilting DOPO il resampling
        _tilting_start_index = resampled_approach.size();

        // Unisci i due path
        resampled_poses = resampled_approach;
        resampled_poses.insert(resampled_poses.end(), resampled_circle.begin(), resampled_circle.end());
    } 
    else {
        // Caso normale: resample tutto insieme
        resampled_poses = resample_path(msg->poses, _resampling_distance);
        _tilting_start_index = resampled_poses.size(); // tilting mai abilitato
    }

    // Ora puoi usare _tilting_start_index per abilitare il tilting negli ultimi segmenti
    _total_waypoints = resampled_poses.size();
    _current_waypoint_index = 0;

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        // Store waypoints in MAP frame - transform them only when needed with latest TF
        for (const auto& pose : resampled_poses) {
            _waypoint_queue.push(pose);  // Keep in original MAP frame
        }
    }
    
    // Transform and start with first waypoint immediately
    if (!_waypoint_queue.empty()) {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        geometry_msgs::msg::PoseStamped first_waypoint = _waypoint_queue.front();
        _waypoint_queue.pop();

        // Publish "following_traj" until reaching the last waypoint
        if (_current_waypoint_index < _total_waypoints - 1) {
            std_msgs::msg::String completed_msg;
            completed_msg.data = "following_traj";
            _debug_publisher->publish(completed_msg); 
        }
        
        // Transform the first waypoint with current TF
        geometry_msgs::msg::PoseStamped transformed_pose = transform_pose_to_odom(first_waypoint);
        if (transformed_pose.header.frame_id != "") {
            _current_target = transformed_pose;
            _has_target = true;
            
            Eigen::Vector3f target_pos(
                _current_target.pose.position.x,
                _current_target.pose.position.y,
                _current_target.pose.position.z
            );
            
            // For single waypoint paths, maintain current yaw
            // For multi-waypoint paths, calculate heading direction
            float target_yaw;
            if (resampled_poses.size() == 1) {
                target_yaw = _ref_yaw;
                RCLCPP_INFO(get_logger(), "Single waypoint detected - maintaining current yaw: %.3f", target_yaw);
            } 
            else {
                target_yaw = calculate_heading_yaw(_current_position, target_pos);
            }
            
            set_new_target(target_pos, target_yaw);
            _state = FOLLOWING_TRAJECTORY;
            
            // Reset counter only if not armed to avoid infinite loops
            if (!_armed) {
                _offboard_setpoint_counter = 0;
                RCLCPP_INFO(get_logger(), "Starting new trajectory (resetting counter for unarmed drone)");
            } else {
                RCLCPP_INFO(get_logger(), "Starting new trajectory (maintaining counter for armed drone)");
            }
            
            // Engage offboard mode immediately when starting trajectory
            // if (!_offboard_mode) {
            //     RCLCPP_INFO(get_logger(), "Engaging offboard mode for trajectory execution");
            //     engage_offboard_mode();
            // }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to transform first waypoint, cannot start trajectory");
            _state = STOPPED;
        }
    } else {
        RCLCPP_ERROR(get_logger(), "No waypoints in path, cannot start trajectory");
        _state = STOPPED;
    }
        
    // Mark that we received the first path
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
    
    // DEBUG: Log current position from odometry
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Current position from odometry in frame '%s': [%.3f, %.3f, %.3f]",
                         msg->header.frame_id.c_str(),
                         _current_position(0), _current_position(1), _current_position(2));
    
    // Initialize reference values on first odometry message
    if (!_first_odom_received) {
        _ref_position = _current_position;
        _cmd_position = _current_position;
        _ref_yaw = extract_yaw_from_quaternion(_current_attitude);
        _cmd_yaw = _ref_yaw;
        _first_odom_received = true;
        
        // Engage offboard mode immediately at startup
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

    if (!_offboard_mode && _startup) {
        return;
    }
    
    _offboard_setpoint_counter++;
    
    // Handle teleop mode
    if (_teleop_active) {
        handle_teleop_mode();
        return;
    }
    
    // Trajectory interpolation if we have a target and not stopped
    if (_has_target && _state == FOLLOWING_TRAJECTORY) {
        interpolate_trajectory();
        publish_trajectory_setpoint();
    } else if (_state == STOPPED && !_teleop_active) {
        // If stopped and not in teleop, maintain current position
        _cmd_position = _current_position;
        _ref_velocity = Eigen::Vector3f::Zero();
        _ref_acceleration = Eigen::Vector3f::Zero();
        publish_trajectory_setpoint();
    } else {
        // Publish current position as setpoint to maintain stability
        publish_trajectory_setpoint();
    }
    
    // Auto-arm when trajectory starts and drone is landed
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
    
    // Check if we should advance to next waypoint
    if (_has_target && _state == FOLLOWING_TRAJECTORY) {
        Eigen::Vector3f error = _cmd_position - _ref_position;
        float distance_error = error.norm();
        
        Eigen::Vector3f target_error = _cmd_position - _current_position;
        float distance_to_target = target_error.norm();
        
        // Check if we have next waypoint in queue and we're at half distance
        bool should_advance_waypoint = false;
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            if (!_waypoint_queue.empty() && distance_to_target <= _waypoint_tolerance * 2.0) {
                should_advance_waypoint = true;
            }
        }
        
        // Advance to next waypoint when at half distance for continuous flow
        if (should_advance_waypoint) {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            if (!_waypoint_queue.empty()) {
                // Get next waypoint in MAP frame
                geometry_msgs::msg::PoseStamped next_waypoint_map = _waypoint_queue.front();
                _waypoint_queue.pop();
                
                // Transform with LATEST TF (this is critical for SLAM accuracy)
                geometry_msgs::msg::PoseStamped transformed_target = transform_pose_to_odom(next_waypoint_map);
                if (transformed_target.header.frame_id != "") {
                    _current_target = transformed_target;
                    
                    Eigen::Vector3f target_pos(
                        transformed_target.pose.position.x,
                        transformed_target.pose.position.y,
                        transformed_target.pose.position.z
                    );
                    
                    float target_yaw = calculate_heading_yaw(_ref_position, target_pos);
                    
                    set_new_target(target_pos, target_yaw);
                    
                    // Add transformed waypoint to path for visualization
                    _transformed_path.poses.push_back(transformed_target);
                    _transformed_path.header.frame_id = "odom";
                    _transformed_path.header.stamp = this->get_clock()->now();
                    
                    // Publish updated transformed path
                    _transformed_path_publisher->publish(_transformed_path);
                    
                    RCLCPP_INFO(get_logger(), "Published transformed path with %zu waypoints on topic: /trajectory_interpolator/transformed_path", 
                               _transformed_path.poses.size());
                    
                    RCLCPP_INFO(get_logger(), 
                               "Advanced to next waypoint with LATEST TF: MAP[%.3f,%.3f,%.3f] → ODOM[%.3f,%.3f,%.3f], yaw: %.3f", 
                               next_waypoint_map.pose.position.x, next_waypoint_map.pose.position.y, next_waypoint_map.pose.position.z,
                               target_pos(0), target_pos(1), target_pos(2), target_yaw);
                } else {
                    RCLCPP_WARN(get_logger(), "Failed to transform next waypoint with latest TF, skipping");
                }
                _current_waypoint_index++;
            }
        }
        
        float yaw_error = std::abs(_cmd_yaw - _ref_yaw);
        if (yaw_error > M_PI) {
            yaw_error = 2.0f * M_PI - yaw_error;
        }
        
        bool queue_empty = false;
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            queue_empty = _waypoint_queue.empty();
        }

        std_msgs::msg::String completed_msg;
        // Complete trajectory only when no more waypoints and very close to final target
        if (queue_empty && distance_error < _waypoint_tolerance && yaw_error < _yaw_tolerance) {
            // completed_msg.data = "traj_completed";
            // _debug_publisher->publish(completed_msg);
            _has_target = false;
            _state = IDLE;
            RCLCPP_INFO(get_logger(), "Trajectory completed - reached final waypoint [%.3f, %.3f, %.3f]", 
                       _cmd_position(0), _cmd_position(1), _cmd_position(2));
        }
        // else if(_path_mode == "flyto") {
        //    completed_msg.data = "following_traj";
        //     _debug_publisher->publish(completed_msg); 
        // } // this is useless for now
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
    // Position interpolation with axis-specific parameters
    Eigen::Vector3f position_error = _cmd_position - _ref_position;
    Eigen::Vector3f desired_acceleration;
    
    for (int i = 0; i < 3; i++) {
        // Use Z-specific parameters for vertical axis (i=2), XY parameters for horizontal axes
        double omega = (i == 2) ? _ref_omega_z : _ref_omega;
        double zeta = (i == 2) ? _ref_zeta_z : _ref_zeta;
        double jerk_max = (i == 2) ? _ref_jerk_max_z : _ref_jerk_max;
        
        desired_acceleration(i) = omega * omega * position_error(i) - 
                                2.0 * zeta * omega * _ref_velocity(i);
        
        // Jerk limiting with axis-specific limits
        float jerk = (desired_acceleration(i) - _ref_acceleration(i)) / _dt;
        if (std::abs(jerk) > jerk_max) {
            jerk = (jerk > 0.0) ? jerk_max : -jerk_max;
        }
        
        desired_acceleration(i) = _ref_acceleration(i) + jerk * _dt;
        
        // Acceleration limiting with axis-specific limits
        double acc_max = (i == 2) ? _ref_acc_max_z : _ref_acc_max;
        if (std::abs(desired_acceleration(i)) > acc_max) {
            _ref_acceleration(i) = (desired_acceleration(i) > 0.0) ? acc_max : -acc_max;
        } else {
            _ref_acceleration(i) = desired_acceleration(i);
        }
        
        // Velocity integration and limiting with axis-specific limits
        float desired_velocity = _ref_velocity(i) + _ref_acceleration(i) * _dt;
        double vel_max = (i == 2) ? _ref_vel_max_z : _ref_vel_max;
        if (std::abs(desired_velocity) > vel_max) {
            _ref_velocity(i) = (desired_velocity > 0.0) ? vel_max : -vel_max;
        } else {
            _ref_velocity(i) = desired_velocity;
        }
        
        // Position integration
        _ref_position(i) += _ref_velocity(i) * _dt;
    }
    
    // Yaw interpolation
    float yaw_error = _cmd_yaw - _ref_yaw;

    // Normalize yaw error to [-pi, pi] to always take the shortest path
    while (yaw_error > M_PI) yaw_error -= 2.0f * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

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
    
    // --- Loitering with variable pitch (only last segments) ---
    if (_path_mode == "circle" && _state == FOLLOWING_TRAJECTORY && _current_waypoint_index >= _tilting_start_index && _tilting_on_pitch_enabled) {
        RCLCPP_WARN(get_logger(), "Tilting pitch interpolation enabled: waypoint %d/%d (tilting from index %d), current_pitch: %.2f, goal_pitch: %.2f",
                    _current_waypoint_index, _total_waypoints, _tilting_start_index, _current_pitch, _tilting_goal_pitch);
        
        if (_current_pitch < _tilting_goal_pitch) {
            _current_pitch += _tilting_interp_rate * _dt;
            if (_current_pitch > _tilting_goal_pitch) _current_pitch = _tilting_goal_pitch;
        } else if (_current_pitch > _tilting_goal_pitch) {
            _current_pitch -= _tilting_interp_rate * _dt;
            if (_current_pitch < _tilting_goal_pitch) _current_pitch = _tilting_goal_pitch;
        }
    } 
    else {
        
        if (_current_pitch > 0.0) {
            _current_pitch -= _tilting_interp_rate * _dt;
            if (_current_pitch < 0.0) _current_pitch = 0.0;
        } else if (_current_pitch < 0.0) {
            _current_pitch += _tilting_interp_rate * _dt;
            if (_current_pitch > 0.0) _current_pitch = 0.0;
        }
    }
    px4_msgs::msg::TiltingMcDesiredAngles pitch_msg;
    pitch_msg.pitch_body = _current_pitch;
    pitch_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    _tilting_pub->publish(pitch_msg);
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
        _cmd_position = _ref_position;
        _cmd_yaw = _ref_yaw;
        _has_target = false;
        // Reset counter only if not armed to avoid infinite loops
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
    
    // Position (in odom frame)
    geometry_msgs::msg::Transform transform;
    transform.translation.x = _ref_position(0);
    transform.translation.y = _ref_position(1);
    transform.translation.z = _ref_position(2);
    
    // DEBUG: Log current setpoint being published
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Publishing setpoint in odom frame: [%.3f, %.3f, %.3f]",
                         _ref_position(0), _ref_position(1), _ref_position(2));
    
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
    Eigen::Vector3f direction = target_pos - current_pos;

    float horizontal_distance = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    float vertical_distance = std::abs(direction.z());

    // If movement is predominantly vertical, maintain current yaw
    if (horizontal_distance < _vertical_movement_threshold * vertical_distance) {
        RCLCPP_INFO(get_logger(), "Vertical movement detected (h: %.3f, v: %.3f) - maintaining current yaw: %.3f", 
                   horizontal_distance, vertical_distance, _ref_yaw);
        return _ref_yaw;
    }

    // For horizontal movements, calculate yaw angle from direction vector
    float yaw = std::atan2(direction.y(), direction.x());

    // Normalize yaw to [-pi, pi]
    while (yaw > M_PI) yaw -= 2.0f * M_PI;
    while (yaw < -M_PI) yaw += 2.0f * M_PI;

    if(_choose_final_yaw){
        // If this is the last waypoint and path_mode is "flyto", use the desired yaw from the waypoint orientation
        if ((_current_waypoint_index == _total_waypoints - 2) && _path_mode == "flyto") {
            yaw = utilities::quatToRpy(Eigen::Vector4d(
                _current_target.pose.orientation.w,
                _current_target.pose.orientation.x,
                _current_target.pose.orientation.y,
                _current_target.pose.orientation.z
            ))(2);

            RCLCPP_WARN(get_logger(), "target_yaw (last waypoint, flyto): %.3f", yaw);
            std_msgs::msg::String completed_msg;
            completed_msg.data = "flyto_run";
            _debug_publisher->publish(completed_msg);
        }
        else if (_tilting_on_pitch_enabled && _path_mode == "circle" && _current_waypoint_index > (_tilting_start_index - 2)) {
            // Calculate yaw toward _approach_penultimate
            Eigen::Vector3f center_pos(
                _approach_penultimate.pose.position.x,
                _approach_penultimate.pose.position.y,
                _approach_penultimate.pose.position.z
            );
            Eigen::Vector3f to_center = center_pos - target_pos;
            float center_yaw = std::atan2(to_center.y(), to_center.x());
            // Normalize yaw to [-pi, pi]
            while (center_yaw > M_PI) center_yaw -= 2.0f * M_PI;
            while (center_yaw < -M_PI) center_yaw += 2.0f * M_PI;
            RCLCPP_INFO(get_logger(), "Tilting active: forcing yaw toward circle center: %.3f", center_yaw);
            yaw = center_yaw;
            std_msgs::msg::String completed_msg;
            completed_msg.data = "circle_run";
            _debug_publisher->publish(completed_msg);
        }
    }

    RCLCPP_INFO(get_logger(), "Horizontal movement detected (h: %.3f, v: %.3f) - new yaw: %.3f", 
               horizontal_distance, vertical_distance, yaw);

    return yaw;
}

float TrajectoryInterpolator::extract_yaw_from_quaternion(const Eigen::Quaternionf& q) {
    // Extract yaw from quaternion
    // float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()), 
    //                       1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    Eigen::Vector3d rpy = utilities::quatToRpy(Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()));
    float yaw = rpy(2);
    return yaw;
}

// PX4 State callbacks
void TrajectoryInterpolator::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
    _armed = msg->flag_armed;
    _offboard_mode = msg->flag_control_offboard_enabled;
    if (_offboard_mode) {
        _startup = true;
    }
    if (_was_offboard && !_offboard_mode) {
        RCLCPP_WARN(get_logger(), "Exiting from OFFBOARD: activating safety mode (null setpoints)");
        _state = STOPPED;
        _has_target = false;
    }
    _was_offboard = _offboard_mode;
}

void TrajectoryInterpolator::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    bool was_landed = _landed;
    // Consider landed if either 'landed' or 'maybe_landed' is true
    _landed = msg->landed || msg->maybe_landed;

    // If drone just landed (or maybe landed) while armed, stop trajectory and disarm
    if (_landed && !was_landed && _armed) {
        RCLCPP_INFO(get_logger(), "Landing detected (landed or maybe_landed) during flight - stopping trajectory and disarming");

        if (_state == FOLLOWING_TRAJECTORY) {
            _state = IDLE;
            _has_target = false;
            clear_waypoint_queue();

            // Clear transformed path visualization on landing
            _transformed_path.poses.clear();
            _transformed_path_publisher->publish(_transformed_path);
            RCLCPP_INFO(get_logger(), "Cleared transformed path visualization on landing");
        }
        disarm();
    }
}

void TrajectoryInterpolator::path_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    _path_mode = msg->data;
    RCLCPP_INFO(get_logger(), "Path mode updated: %s", _path_mode.c_str());
}

// PX4 Commands
void TrajectoryInterpolator::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(get_logger(), "Arm command sent");
    if (!_offboard_mode) {
        RCLCPP_INFO(get_logger(), "Engaging offboard mode for trajectory execution");
        engage_offboard_mode();
    }
    std_msgs::msg::String completed_msg;
    completed_msg.data = "armed";
    _debug_publisher->publish(completed_msg);
}

void TrajectoryInterpolator::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(get_logger(), "Disarm command sent");
    std_msgs::msg::String completed_msg;    
    completed_msg.data = "disarmed";
    _debug_publisher->publish(completed_msg);
}

void TrajectoryInterpolator::engage_offboard_mode() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(get_logger(), "Switching to offboard mode");
}

geometry_msgs::msg::PoseStamped TrajectoryInterpolator::transform_pose_to_odom(const geometry_msgs::msg::PoseStamped& pose_in_map) {
    geometry_msgs::msg::PoseStamped transformed_pose = pose_in_map;
    
    if (_do_transform) {
        try {
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.stamp = this->get_clock()->now();
            point_in.header.frame_id = _parent_transf;
            point_in.point.x = pose_in_map.pose.position.x;
            point_in.point.y = pose_in_map.pose.position.y;
            point_in.point.z = pose_in_map.pose.position.z;
            
            tf2::doTransform(point_in, point_out, _tf_map_to_odom);
            
            transformed_pose.header.frame_id = _child_transf;
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
            transformed_pose.header.frame_id = "";
        }
    } else {
        transformed_pose = pose_in_map;
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
    
    rclcpp::Rate rate(100);
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
        return original_path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> resampled_path;
    
    resampled_path.push_back(original_path[0]);
    
    for (size_t i = 0; i < original_path.size() - 1; i++) {
        const auto& current_pose = original_path[i];
        const auto& next_pose = original_path[i + 1];
        
        double dx = next_pose.pose.position.x - current_pose.pose.position.x;
        double dy = next_pose.pose.position.y - current_pose.pose.position.y;
        double dz = next_pose.pose.position.z - current_pose.pose.position.z;
        double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (segment_length <= sampling_distance) {
            continue;
        }
        
        int num_segments = static_cast<int>(std::ceil(segment_length / sampling_distance));
        double actual_step = segment_length / num_segments;
        
        // Generate intermediate waypoints
        for (int j = 1; j < num_segments; j++) {
            geometry_msgs::msg::PoseStamped intermediate_pose;
            intermediate_pose.header = current_pose.header;
            
            double ratio = (j * actual_step) / segment_length;
            
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
    
    resampled_path.push_back(original_path.back());
    
    RCLCPP_INFO(get_logger(), "Resampled path: %zu → %zu waypoints (sampling distance: %.2f m)", 
                original_path.size(), resampled_path.size(), sampling_distance);
    
    return resampled_path;
}

void TrajectoryInterpolator::teleop_active_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool previous_state = _teleop_active;
    _teleop_active = msg->data;
    
    if (_teleop_active && !previous_state) {
        // Teleop just activated - save current position as starting point
        _teleop_base_position = _current_position.cast<double>();
        _teleop_base_yaw = _ref_yaw;
        
        // Initialize command and reference position for seamless transition
        _cmd_position = _current_position;
        _cmd_yaw = _ref_yaw;
        _ref_position = _current_position;
        
        RCLCPP_INFO(get_logger(), "Teleop mode ACTIVATED - starting from position [%.2f, %.2f, %.2f] yaw: %.2f", 
                    _ref_position.x(), _ref_position.y(), _ref_position.z(), _ref_yaw);
        
        _velocity_increments.linear.x = 0.0;
        _velocity_increments.linear.y = 0.0;
        _velocity_increments.linear.z = 0.0;
        _velocity_increments.angular.z = 0.0;
    } else if (!_teleop_active && previous_state) {
        // Teleop just deactivated
        RCLCPP_INFO(get_logger(), "Teleop mode DEACTIVATED - final position [%.2f, %.2f, %.2f] yaw: %.2f", 
                    _ref_position.x(), _ref_position.y(), _ref_position.z(), _ref_yaw);
        
        _cmd_position = _ref_position;
        _cmd_yaw = _ref_yaw;
        
        _state = STOPPED;
        
        _ref_velocity = Eigen::Vector3f::Zero();
        _ref_acceleration = Eigen::Vector3f::Zero();
        _ref_yaw_rate = 0.0f;
    }
}

void TrajectoryInterpolator::velocity_increments_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (_teleop_active) {
        _velocity_increments = *msg;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Received velocity increments: linear[%.2f, %.2f, %.2f] angular[%.2f]",
                             _velocity_increments.linear.x, _velocity_increments.linear.y, 
                             _velocity_increments.linear.z, _velocity_increments.angular.z);
    }
}

void TrajectoryInterpolator::handle_teleop_mode() {
    // In teleop mode, integrate velocity commands to update position incrementally
    double dt = _dt;
    
    _cmd_position.x() += _velocity_increments.linear.x * dt;
    _cmd_position.y() += _velocity_increments.linear.y * dt;
    _cmd_position.z() += _velocity_increments.linear.z * dt;
    
    _cmd_yaw += _velocity_increments.angular.z * dt;
    
    // Update reference position for publication
    _ref_position = _cmd_position;
    _ref_yaw = _cmd_yaw;
    
    _ref_velocity = Eigen::Vector3f(_velocity_increments.linear.x, 
                                   _velocity_increments.linear.y, 
                                   _velocity_increments.linear.z);
    
    _ref_acceleration = Eigen::Vector3f::Zero();
    _ref_yaw_rate = _velocity_increments.angular.z;
    
    publish_trajectory_setpoint();
    
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Teleop mode - Ref pos: [%.2f, %.2f, %.2f], vel: [%.2f, %.2f, %.2f], yaw: %.2f",
                         _ref_position.x(), _ref_position.y(), _ref_position.z(),
                         _ref_velocity.x(), _ref_velocity.y(), _ref_velocity.z(), _ref_yaw);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryInterpolator>());
    rclcpp::shutdown();
    return 0;
}
