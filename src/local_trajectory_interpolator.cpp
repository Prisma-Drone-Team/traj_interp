#include "traj_interp/local_trajectory_interpolator.hpp"
#include <cmath>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

LocalTrajectoryInterpolator::LocalTrajectoryInterpolator() : rclcpp::Node("local_trajectory_interpolator"), _state(IDLE) {
    
    // [TUTTO IL COSTRUTTORE CHE AVEVI GIÀ - mantieni esattamente come prima]
    // Declare parameters
    this->declare_parameter("path_topic", "/trajectory_path");
    _path_topic = this->get_parameter("path_topic").as_string();
    
    this->declare_parameter("odometry_topic", "/px4/odometry/out");
    _odometry_topic = this->get_parameter("odometry_topic").as_string();
    
    this->declare_parameter("offboard_control_mode_topic", "fmu/in/offboard_control_mode");
    _offboard_control_mode_topic = this->get_parameter("offboard_control_mode_topic").as_string();
    
    this->declare_parameter("vehicle_command_topic", "fmu/in/vehicle_command");
    _vehicle_command_topic = this->get_parameter("vehicle_command_topic").as_string();
    
    this->declare_parameter("trajectory_setpoint_topic", "/px4/trajectory_setpoint_enu");
    _trajectory_setpoint_topic = this->get_parameter("trajectory_setpoint_topic").as_string();
    
    this->declare_parameter("status_topic", "/local_traj_interp/status");
    _status_topic = this->get_parameter("status_topic").as_string();
    
    // Local planner specific topics
    this->declare_parameter("obstacle_cloud_map_topic", "/obstacle_cloud");
    _obstacle_cloud_map_topic = this->get_parameter("obstacle_cloud_map_topic").as_string();
    
    this->declare_parameter("obstacle_cloud_body_topic", "/obstacle_cloud_body");
    _obstacle_cloud_body_topic = this->get_parameter("obstacle_cloud_body_topic").as_string();
    
    this->declare_parameter("local_planner_enable_topic", "/local_planner_enable");
    _local_planner_enable_topic = this->get_parameter("local_planner_enable_topic").as_string();
    
    this->declare_parameter("collision_detected_topic", "/collision_detected");
    _collision_detected_topic = this->get_parameter("collision_detected_topic").as_string();
    
    // APF Local Planner parameters
    this->declare_parameter("max_lin_speed", 1.0);
    _max_lin_speed = this->get_parameter("max_lin_speed").as_double();
    
    this->declare_parameter("max_ang_speed", 0.5);
    _max_ang_speed = this->get_parameter("max_ang_speed").as_double();
    
    this->declare_parameter("safety_distance", 1.5);  // Ridotto per essere più permissivo
    _safety_distance = this->get_parameter("safety_distance").as_double();
    
    this->declare_parameter("lookahead_distance", 2.5);
    _lookahead_distance = this->get_parameter("lookahead_distance").as_double();
    
    this->declare_parameter("k_attractive", 3.0);  // Aumentato per bilanciare meglio
    _k_attractive = this->get_parameter("k_attractive").as_double();
    
    this->declare_parameter("k_repulsive", 2.0);  // Ridotto per essere meno aggressivo
    _k_repulsive = this->get_parameter("k_repulsive").as_double();
    
    this->declare_parameter("k_yaw", 1.5);
    _k_yaw = this->get_parameter("k_yaw").as_double();
    
    // APF dynamics parameters
    this->declare_parameter("force_smoothing_factor", 0.8);
    _force_smoothing_factor = this->get_parameter("force_smoothing_factor").as_double();
    
    this->declare_parameter("velocity_damping", 0.9);
    _velocity_damping = this->get_parameter("velocity_damping").as_double();
    
    this->declare_parameter("goal_attraction_threshold", 0.5);
    _goal_attraction_threshold = this->get_parameter("goal_attraction_threshold").as_double();
    
    this->declare_parameter("obstacle_influence_range", 1.8);  // Ridotto per avoidance più locale
    _obstacle_influence_range = this->get_parameter("obstacle_influence_range").as_double();
    
    // Nuovi parametri per potenziali non lineari
    this->declare_parameter("local_avoidance_radius", 3.0);  // Raggio per avoidance locale
    _local_avoidance_radius = this->get_parameter("local_avoidance_radius").as_double();
    
    this->declare_parameter("repulsive_exp_factor", 2.0);  // Fattore esponenziale per repulsione
    _repulsive_exp_factor = this->get_parameter("repulsive_exp_factor").as_double();
    
    this->declare_parameter("min_obstacle_distance", 0.3);  // Distanza minima critica
    _min_obstacle_distance = this->get_parameter("min_obstacle_distance").as_double();
    
    this->declare_parameter("max_repulsive_force", 3.0);  // Forza repulsiva massima
    _max_repulsive_force = this->get_parameter("max_repulsive_force").as_double();
    
    this->declare_parameter("attractive_scale_distance_max", 2.0);  // d_max per scaling tanh (era 1.5)
    _attractive_scale_distance_max = this->get_parameter("attractive_scale_distance_max").as_double();
    
    // Control parameters
    this->declare_parameter("control_frequency", 50.0);
    _control_frequency = this->get_parameter("control_frequency").as_double();
    _dt = 1.0 / _control_frequency;
    
    this->declare_parameter("waypoint_tolerance", 0.1);
    _waypoint_tolerance = this->get_parameter("waypoint_tolerance").as_double();
    
    this->declare_parameter("yaw_tolerance", 0.1);
    _yaw_tolerance = this->get_parameter("yaw_tolerance").as_double();
    
    this->declare_parameter("vertical_movement_threshold", 0.2);
    _vertical_movement_threshold = this->get_parameter("vertical_movement_threshold").as_double();
    
    this->declare_parameter("resampling_distance", 0.3);
    _resampling_distance = this->get_parameter("resampling_distance").as_double();
    
    // TF parameters
    this->declare_parameter("parent_transform", "drone/map");
    _parent_transf = this->get_parameter("parent_transform").as_string();
    
    this->declare_parameter("child_transform", "odom");
    _child_transf = this->get_parameter("child_transform").as_string();
    
    this->declare_parameter("do_transform", true);
    _do_transform = this->get_parameter("do_transform").as_bool();
    
    this->declare_parameter("tf_buffer_timeout", 0.5);
    _tf_buffer_timeout = this->get_parameter("tf_buffer_timeout").as_double();
    
    // Frame parameters
    this->declare_parameter("robot_frame", "base_link");
    _robot_frame = this->get_parameter("robot_frame").as_string();
    
    this->declare_parameter("map_frame", "map");
    _map_frame = this->get_parameter("map_frame").as_string();
    
    this->declare_parameter("odom_frame", "odom");
    _odom_frame = this->get_parameter("odom_frame").as_string();
    
    // Loiter and tilting parameters
    this->declare_parameter("loiter_segment", 8);
    _loiter_segment = this->get_parameter("loiter_segment").as_int();

    this->declare_parameter("tilting_goal_pitch_deg", -20.0);
    _tilting_goal_pitch = this->get_parameter("tilting_goal_pitch_deg").as_double() * M_PI / 180.0;
    
    this->declare_parameter("tilting_interp_rate", 0.05);
    _tilting_interp_rate = this->get_parameter("tilting_interp_rate").as_double();

    this->declare_parameter("tilting_on_pitch_enabled", true);
    _tilting_on_pitch_enabled = this->get_parameter("tilting_on_pitch_enabled").as_bool();

    this->declare_parameter("choose_final_yaw", true);
    _choose_final_yaw = this->get_parameter("choose_final_yaw").as_bool();
    
    _current_pitch = 0.0;

    // Visualization
    this->declare_parameter("enable_visualization", true);
    _enable_visualization = this->get_parameter("enable_visualization").as_bool();
    
    RCLCPP_INFO(get_logger(), "Local Trajectory Interpolator initialized with APF");
    RCLCPP_INFO(get_logger(), "APF params - max_lin: %.2f, max_ang: %.2f, safety_dist: %.2f", 
                _max_lin_speed, _max_ang_speed, _safety_distance);
    
    // Initialize publishers
    _offboard_control_mode_publisher = this->create_publisher<OffboardControlMode>(_offboard_control_mode_topic, 10);
    _vehicle_command_publisher = this->create_publisher<VehicleCommand>(_vehicle_command_topic, 10);
    _cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/px4/cmd_vel", 10);
    _status_publisher = this->create_publisher<std_msgs::msg::String>(_status_topic, 10);
    _transformed_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/local_traj_interp/transformed_path", 10);
    _debug_publisher = this->create_publisher<std_msgs::msg::String>("/local_traj_interp/completed", 10);
    _tilting_pub = this->create_publisher<px4_msgs::msg::TiltingMcDesiredAngles>("fmu/in/tilting_mc_desired_angles", 10);
    
    // Local planner debug publishers
    _forces_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/local_traj_interp/debug_forces", 10);
    _lookahead_pub = this->create_publisher<visualization_msgs::msg::Marker>("/local_traj_interp/lookahead_point", 10);
    
    // Initialize subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    
    _path_subscription = this->create_subscription<nav_msgs::msg::Path>(
        _path_topic, 10, std::bind(&LocalTrajectoryInterpolator::path_callback, this, std::placeholders::_1));
    
    _odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        _odometry_topic, qos, std::bind(&LocalTrajectoryInterpolator::odometry_callback, this, std::placeholders::_1));
    
    _vehicle_control_mode_subscription = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
        "/fmu/out/vehicle_control_mode", qos, 
        std::bind(&LocalTrajectoryInterpolator::vehicle_control_mode_callback, this, std::placeholders::_1));
    
    _land_detected_subscription = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
        "/fmu/out/vehicle_land_detected", qos,
        std::bind(&LocalTrajectoryInterpolator::land_detected_callback, this, std::placeholders::_1));
    
    _path_mode_subscription = this->create_subscription<std_msgs::msg::String>(
        "/move_manager/path_mode", 10,
        std::bind(&LocalTrajectoryInterpolator::path_mode_callback, this, std::placeholders::_1));

    // Teleop coordination subscribers
    _teleop_active_subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/move_manager/teleop_active", 10,
        std::bind(&LocalTrajectoryInterpolator::teleop_active_callback, this, std::placeholders::_1));
    
    _velocity_increments_subscription = this->create_subscription<geometry_msgs::msg::Twist>(
        "/teleop/velocity_increments", 10,
        std::bind(&LocalTrajectoryInterpolator::velocity_increments_callback, this, std::placeholders::_1));

    // Local planner specific subscribers
    _obstacle_cloud_map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        _obstacle_cloud_map_topic, qos,
        std::bind(&LocalTrajectoryInterpolator::obstacle_cloud_map_callback, this, std::placeholders::_1));
    
    _obstacle_cloud_body_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        _obstacle_cloud_body_topic, qos,
        std::bind(&LocalTrajectoryInterpolator::obstacle_cloud_body_callback, this, std::placeholders::_1));
    
    _local_planner_enable_sub = this->create_subscription<std_msgs::msg::Bool>(
        _local_planner_enable_topic, 10,
        std::bind(&LocalTrajectoryInterpolator::local_planner_enable_callback, this, std::placeholders::_1));
    
    _collision_detected_sub = this->create_subscription<std_msgs::msg::Bool>(
        _collision_detected_topic, 10,
        std::bind(&LocalTrajectoryInterpolator::collision_detected_callback, this, std::placeholders::_1));

    // Initialize timers
    _control_timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / _control_frequency)),
        std::bind(&LocalTrajectoryInterpolator::control_timer_callback, this));
    
    _status_timer = this->create_wall_timer(
        1s, std::bind(&LocalTrajectoryInterpolator::status_timer_callback, this));
    
    // Initialize TF2
    _tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);
    
    // Start TF lookup thread
    std::thread tf_thread(&LocalTrajectoryInterpolator::tf_lookup_loop, this);
    tf_thread.detach();
    
    // Initialize APF variables
    _cmd_velocity.setZero();
    
    RCLCPP_INFO(get_logger(), "LocalTrajectoryInterpolator node initialized with APF local planner");
}

// ============================================================================
// FUNZIONI DEL TRAJECTORY INTERPOLATOR ORIGINALE (MANTENUTE)
// ============================================================================

void LocalTrajectoryInterpolator::path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
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
    
    // CRITICAL: Reset state from IDLE when new path arrives
    if (_state == IDLE) {
        RCLCPP_INFO(get_logger(), "🚀 Reactivating system from IDLE for new trajectory");
        _state = STOPPED;  // Will be set to FOLLOWING_TRAJECTORY after loading waypoints
    }
    
    // Reset transformed path for new trajectory
    _transformed_path.poses.clear();
    _transformed_path.header.frame_id = "odom";
    _transformed_path.header.stamp = this->get_clock()->now();
    
    // Publish empty path to clear previous visualization
    _transformed_path_publisher->publish(_transformed_path);
    RCLCPP_INFO(get_logger(), "Reset and published empty transformed path for new trajectory");
    
    std::vector<geometry_msgs::msg::PoseStamped> resampled_poses;
    int loiter_segment = _loiter_segment;
    int total_points = msg->poses.size();

    if (_path_mode == "circle" && total_points > loiter_segment) {
        std::vector<geometry_msgs::msg::PoseStamped> approach_path(
            msg->poses.begin(), msg->poses.end() - loiter_segment);
        if (!approach_path.empty() && approach_path.size() >= 2) {
            _approach_penultimate = approach_path[approach_path.size() - 2];
        } else if (!approach_path.empty()) {
            _approach_penultimate = approach_path.back();
        }
        std::vector<geometry_msgs::msg::PoseStamped> circle_path(
            msg->poses.end() - loiter_segment, msg->poses.end());

        auto resampled_approach = resample_path(approach_path, _resampling_distance);
        auto resampled_circle = resample_path(circle_path, _resampling_distance);

        _tilting_start_index = resampled_approach.size();

        resampled_poses = resampled_approach;
        resampled_poses.insert(resampled_poses.end(), resampled_circle.begin(), resampled_circle.end());
    } 
    else {
        resampled_poses = resample_path(msg->poses, _resampling_distance);
        _tilting_start_index = resampled_poses.size();
    }

    _total_waypoints = resampled_poses.size();
    _current_waypoint_index = 0;

    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        for (const auto& pose : resampled_poses) {
            _waypoint_queue.push(pose);
        }
    }
    
    if (!_waypoint_queue.empty()) {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        geometry_msgs::msg::PoseStamped first_waypoint = _waypoint_queue.front();
        _waypoint_queue.pop();

        if (_current_waypoint_index < _total_waypoints - 1) {
            std_msgs::msg::String completed_msg;
            completed_msg.data = "following_traj";
            _debug_publisher->publish(completed_msg); 
        }
        
        geometry_msgs::msg::PoseStamped transformed_pose = transform_pose_to_odom(first_waypoint);
        if (transformed_pose.header.frame_id != "") {
            _current_target = transformed_pose;
            _has_target = true;
            
            Eigen::Vector3f target_pos(
                _current_target.pose.position.x,
                _current_target.pose.position.y,
                _current_target.pose.position.z
            );
            
            float target_yaw;
            if (resampled_poses.size() == 1) {
                target_yaw = 0.0f; // Use current yaw instead of _ref_yaw which might not be initialized
                RCLCPP_INFO(get_logger(), "Single waypoint detected - maintaining current yaw");
            } 
            else {
                target_yaw = calculate_heading_yaw(_current_position, target_pos);
            }
            
            // In APF version, we don't use set_new_target for filtering
            _state = FOLLOWING_TRAJECTORY;
            
            if (!_armed) {
                _offboard_setpoint_counter = 0;
                RCLCPP_INFO(get_logger(), "Starting new trajectory (resetting counter for unarmed drone)");
            } else {
                RCLCPP_INFO(get_logger(), "Starting new trajectory (maintaining counter for armed drone)");
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to transform first waypoint, cannot start trajectory");
            _state = STOPPED;
        }
    } else {
        RCLCPP_ERROR(get_logger(), "No waypoints in path, cannot start trajectory");
        _state = STOPPED;
    }
        
    if (!_first_path_received) {
        _first_path_received = true;
        RCLCPP_INFO(get_logger(), "First path received - arming will be enabled");
    }
}

void LocalTrajectoryInterpolator::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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
    
    RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "Current position from odometry in frame '%s': [%.3f, %.3f, %.3f]",
                         msg->header.frame_id.c_str(),
                         _current_position(0), _current_position(1), _current_position(2));
    
    if (!_first_odom_received) {
        // Initialize reference values for APF (different from original filter)
        _cmd_velocity.setZero();
        _first_odom_received = true;
        
        if (!_offboard_mode) {
            RCLCPP_INFO(get_logger(), "Engaging offboard mode at startup");
            engage_offboard_mode();
        }
        
        RCLCPP_INFO(get_logger(), "Initialized current position: [%.3f, %.3f, %.3f]",
                   _current_position(0), _current_position(1), _current_position(2));
    }
}

void LocalTrajectoryInterpolator::vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
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

void LocalTrajectoryInterpolator::land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
    bool was_landed = _landed;
    _landed = msg->landed || msg->maybe_landed;

    if (_landed && !was_landed && _armed) {
        RCLCPP_INFO(get_logger(), "Landing detected (landed or maybe_landed) during flight - stopping trajectory and disarming");

        if (_state == FOLLOWING_TRAJECTORY) {
            _state = IDLE;
            _has_target = false;
            clear_waypoint_queue();

            _transformed_path.poses.clear();
            _transformed_path_publisher->publish(_transformed_path);
            RCLCPP_INFO(get_logger(), "Cleared transformed path visualization on landing");
        }
        disarm();
    }
}

void LocalTrajectoryInterpolator::path_mode_callback(const std_msgs::msg::String::SharedPtr msg) {
    _path_mode = msg->data;
    RCLCPP_INFO(get_logger(), "Path mode updated: %s", _path_mode.c_str());
}

void LocalTrajectoryInterpolator::teleop_active_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    bool previous_state = _teleop_active;
    _teleop_active = msg->data;
    
    if (_teleop_active && !previous_state) {
        _teleop_base_position = _current_position.cast<double>();
        
        _cmd_velocity.setZero();
        
        RCLCPP_INFO(get_logger(), "Teleop mode ACTIVATED - starting from position [%.2f, %.2f, %.2f]", 
                    _current_position.x(), _current_position.y(), _current_position.z());
        
        _velocity_increments.linear.x = 0.0;
        _velocity_increments.linear.y = 0.0;
        _velocity_increments.linear.z = 0.0;
        _velocity_increments.angular.z = 0.0;
    } else if (!_teleop_active && previous_state) {
        RCLCPP_INFO(get_logger(), "Teleop mode DEACTIVATED - final position [%.2f, %.2f, %.2f]", 
                    _current_position.x(), _current_position.y(), _current_position.z());
        
        _cmd_velocity.setZero();
        _state = STOPPED;
    }
}

void LocalTrajectoryInterpolator::velocity_increments_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (_teleop_active) {
        _velocity_increments = *msg;
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Received velocity increments: linear[%.2f, %.2f, %.2f] angular[%.2f]",
                             _velocity_increments.linear.x, _velocity_increments.linear.y, 
                             _velocity_increments.linear.z, _velocity_increments.angular.z);
    }
}

void LocalTrajectoryInterpolator::status_timer_callback() {
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

void LocalTrajectoryInterpolator::clear_waypoint_queue() {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    while (!_waypoint_queue.empty()) {
        _waypoint_queue.pop();
    }
    
    if (_state == FOLLOWING_TRAJECTORY) {
        _state = STOPPED;
        _cmd_velocity.setZero();
        _has_target = false;
        if (!_armed) {
            _offboard_setpoint_counter = 0;
            RCLCPP_INFO(get_logger(), "Trajectory stopped, new path received (resetting counter)");
        } else {
            RCLCPP_INFO(get_logger(), "Trajectory stopped, new path received (maintaining counter for armed drone)");
        }
    }
}

void LocalTrajectoryInterpolator::publish_vehicle_command(uint16_t command, float param1, float param2, float param3) {
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

void LocalTrajectoryInterpolator::arm() {
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

void LocalTrajectoryInterpolator::disarm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
    RCLCPP_INFO(get_logger(), "Disarm command sent");
    std_msgs::msg::String completed_msg;    
    completed_msg.data = "disarmed";
    _debug_publisher->publish(completed_msg);
}

void LocalTrajectoryInterpolator::engage_offboard_mode() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    RCLCPP_INFO(get_logger(), "Switching to offboard mode");
}

float LocalTrajectoryInterpolator::calculate_heading_yaw(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& target_pos) {
    Eigen::Vector3f direction = target_pos - current_pos;

    float horizontal_distance = std::sqrt(direction.x() * direction.x() + direction.y() * direction.y());
    float vertical_distance = std::abs(direction.z());

    if (horizontal_distance < _vertical_movement_threshold * vertical_distance) {
        RCLCPP_INFO(get_logger(), "Vertical movement detected (h: %.3f, v: %.3f) - maintaining current heading", 
                   horizontal_distance, vertical_distance);
        return 0.0f; // For APF, we don't have _ref_yaw, return 0
    }

    float yaw = std::atan2(direction.y(), direction.x());

    while (yaw > M_PI) yaw -= 2.0f * M_PI;
    while (yaw < -M_PI) yaw += 2.0f * M_PI;

    RCLCPP_INFO(get_logger(), "Horizontal movement detected (h: %.3f, v: %.3f) - calculated yaw: %.3f", 
               horizontal_distance, vertical_distance, yaw);

    return yaw;
}

float LocalTrajectoryInterpolator::extract_yaw_from_quaternion(const Eigen::Quaternionf& q) {
    Eigen::Vector3d rpy = utilities::quatToRpy(Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()));
    float yaw = rpy(2);
    return yaw;
}

geometry_msgs::msg::PoseStamped LocalTrajectoryInterpolator::transform_pose_to_odom(const geometry_msgs::msg::PoseStamped& pose_in_map) {
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

void LocalTrajectoryInterpolator::tf_lookup_loop() {
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

std::vector<geometry_msgs::msg::PoseStamped> LocalTrajectoryInterpolator::resample_path(
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
        
        for (int j = 1; j < num_segments; j++) {
            geometry_msgs::msg::PoseStamped intermediate_pose;
            intermediate_pose.header = current_pose.header;
            
            double ratio = (j * actual_step) / segment_length;
            
            intermediate_pose.pose.position.x = current_pose.pose.position.x + ratio * dx;
            intermediate_pose.pose.position.y = current_pose.pose.position.y + ratio * dy;
            intermediate_pose.pose.position.z = current_pose.pose.position.z + ratio * dz;
            
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

void LocalTrajectoryInterpolator::handle_teleop_mode() {
    double dt = _dt;

    // In teleop mode, _velocity_increments are in body frame. Rotate to odom frame using current yaw
    Eigen::Vector3d rpy = utilities::quatToRpy(Eigen::Vector4d(
        _current_attitude.w(), _current_attitude.x(), _current_attitude.y(), _current_attitude.z()));
    double yaw = rpy(2);

    double vx_odom = _velocity_increments.linear.x * std::cos(yaw) - _velocity_increments.linear.y * std::sin(yaw);
    double vy_odom = _velocity_increments.linear.x * std::sin(yaw) + _velocity_increments.linear.y * std::cos(yaw);

    _cmd_velocity.x() = vx_odom;
    _cmd_velocity.y() = vy_odom;
    _cmd_velocity.z() = _velocity_increments.linear.z;
    
    // Limit velocity
    if (_cmd_velocity.norm() > _max_lin_speed) {
        _cmd_velocity = _cmd_velocity.normalized() * _max_lin_speed;
    }
    
    publish_velocity_command(_cmd_velocity.cast<double>(), _velocity_increments.angular.z);
    
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Teleop mode - Cmd vel: [%.2f, %.2f, %.2f], ang_z: %.2f",
                         _cmd_velocity.x(), _cmd_velocity.y(), _cmd_velocity.z(),
                         _velocity_increments.angular.z);
}

// ============================================================================
// FUNZIONI DEL LOCAL PLANNER (NUOVE)
// ============================================================================

void LocalTrajectoryInterpolator::obstacle_cloud_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
        if (!_tf_buffer->canTransform(_map_frame, msg->header.frame_id, msg->header.stamp, 1s)) {
            RCLCPP_WARN(get_logger(), "Transform not available from %s to %s", _map_frame.c_str(), msg->header.frame_id.c_str());
            return;
        }

        auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        geometry_msgs::msg::TransformStamped transform = _tf_buffer->lookupTransform(
            _map_frame, msg->header.frame_id, tf2::TimePointZero, 100ms);
        
        tf2::doTransform(*msg, *transformed_cloud, transform);

        std::lock_guard<std::mutex> lock(_map_cloud_mutex);
        pcl::fromROSMsg(*transformed_cloud, *_map_obstacle_cloud);
        if (!_map_obstacle_cloud->empty()) {
            _map_cloud_kdtree.setInputCloud(_map_obstacle_cloud);
        }
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF2 transform error: %s", ex.what());
    }
}

void LocalTrajectoryInterpolator::obstacle_cloud_body_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    try {
        if (!_tf_buffer->canTransform(_robot_frame, msg->header.frame_id, msg->header.stamp, 1s)) {
            RCLCPP_WARN(get_logger(), "Transform not available from %s to %s", _robot_frame.c_str(), msg->header.frame_id.c_str());
            return;
        }

        auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        geometry_msgs::msg::TransformStamped transform = _tf_buffer->lookupTransform(
            _robot_frame, msg->header.frame_id, tf2::TimePointZero, 100ms);
        
        tf2::doTransform(*msg, *transformed_cloud, transform);

        std::lock_guard<std::mutex> lock(_body_cloud_mutex);
        pcl::fromROSMsg(*transformed_cloud, *_body_obstacle_cloud);
        if (!_body_obstacle_cloud->empty()) {
            _body_cloud_kdtree.setInputCloud(_body_obstacle_cloud);
        }
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF2 transform error: %s", ex.what());
    }
}

void LocalTrajectoryInterpolator::local_planner_enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    _local_planner_enabled = msg->data;
    RCLCPP_INFO(get_logger(), "Local planner %s", _local_planner_enabled ? "enabled" : "disabled");
}

void LocalTrajectoryInterpolator::collision_detected_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    _collision_detected = msg->data;
    if (_collision_detected) {
        RCLCPP_WARN(get_logger(), "Collision detected! Stopping trajectory");
        _state = STOPPED;
        _has_target = false;
        clear_waypoint_queue();
    }
}

void LocalTrajectoryInterpolator::compute_apf_velocity() {
    if (!_first_odom_received || !_has_target || _state != FOLLOWING_TRAJECTORY) {
        return;
    }

    // Controlla se siamo vicini al goal finale PRIMA di calcolare le forze
    {
        std::lock_guard<std::mutex> lock(_queue_mutex);
        if (_waypoint_queue.size() <= 1) {  // Solo se è l'ultimo waypoint o no waypoints
            Eigen::Vector3d current_pos_eigen(_current_position.x(), _current_position.y(), _current_position.z());
            Eigen::Vector3d goal_pos = get_goal_point(current_pos_eigen);
            
            double distance_to_final_goal = (current_pos_eigen - goal_pos).norm();
            
            if (distance_to_final_goal < _waypoint_tolerance) {
                // GOAL RAGGIUNTO - Ferma il drone
                RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
                    "GOAL REACHED! Distance: %.3f, Stopping drone", distance_to_final_goal);
                
                // Pubblica velocità zero per fermare il drone
                publish_velocity_command(Eigen::Vector3d::Zero(), 0.0);
                return;  // Esci dalla funzione senza calcolare altri potenziali
            }
        }
    }

    try {
        geometry_msgs::msg::TransformStamped tf_m_b = _tf_buffer->lookupTransform(_map_frame, _robot_frame, tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped tf_o_m = _tf_buffer->lookupTransform(_odom_frame, _map_frame, tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped tf_o_b = _tf_buffer->lookupTransform(_odom_frame, _robot_frame, tf2::TimePointZero);

        Eigen::Vector3d x_m_b = utilities::pose_from_tf(tf_m_b);
        Eigen::Matrix3d R_o_m = utilities::R_from_tf(tf_o_m);
        Eigen::Matrix3d R_o_b = utilities::R_from_tf(tf_o_b);
        Eigen::Vector3d x_o_b = utilities::pose_from_tf(tf_o_b);
        Eigen::Vector3d rpy_o_b = utilities::R2XYZ(R_o_b);
        
        Eigen::Vector3d g_m = get_lookahead_point(x_m_b);
        
        // === FORMULA DEL PAPER: f_a = k_a * s(d_g) * û_g ===
        Eigen::Vector3d goal_direction = (g_m - x_m_b);
        double d_g = goal_direction.norm();  // Distanza al lookahead point
        
        Eigen::Vector3d Fa_o;
        if (d_g > 1e-5) {
            // Calcola versore û_g = (p^g - p^fb) / d_g
            Eigen::Vector3d u_g = goal_direction / d_g;
            
            // Scaling con tangente iperbolica: s(d_g) = tanh(d_g / d_max)
            double s_dg = std::tanh(d_g / _attractive_scale_distance_max);
            
            // Formula del paper: f_a = k_a * s(d_g) * û_g
            Fa_o = R_o_m * _k_attractive * s_dg * u_g;
            
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                "Paper formula - d_g: %.3f, d_max: %.3f, s(d_g): %.3f, ||f_a||: %.3f", 
                d_g, _attractive_scale_distance_max, s_dg, Fa_o.norm());
        } else {
            Fa_o = Eigen::Vector3d::Zero();
        }
        
        // Le forze repulsive ora sono già scalate nelle funzioni di calcolo
        Eigen::Vector3d Fr_m = calculate_repulsive_force_map(x_m_b); 
        Eigen::Vector3d Fr_b = calculate_repulsive_force_body();  

        // DEBUG: Stampa le forze per capire cosa sta succedendo
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
            "Forces - Attractive: [%.3f,%.3f,%.3f] norm=%.3f, Repulsive Map: [%.3f,%.3f,%.3f] norm=%.3f, Repulsive Body: [%.3f,%.3f,%.3f] norm=%.3f", 
            Fa_o.x(), Fa_o.y(), Fa_o.z(), Fa_o.norm(),
            Fr_m.x(), Fr_m.y(), Fr_m.z(), Fr_m.norm(),
            Fr_b.x(), Fr_b.y(), Fr_b.z(), Fr_b.norm());

        // Combinazione delle forze con bilanciamento migliorato
        // Le forze repulsive vengono scalate direttamente dalle funzioni di calcolo
        Eigen::Vector3d Ftot_o = Fa_o + R_o_m * Fr_m + R_o_b * Fr_b;

        Eigen::Vector3d smoothed_force = _force_smoothing_factor * Ftot_o + 
                                        (1.0 - _force_smoothing_factor) * _cmd_velocity.cast<double>();
        
        // Damping progressivo basato sulla distanza dal goal
        double damping_factor = _velocity_damping;
        if (d_g < _goal_attraction_threshold) {
            // Damping più forte vicino al goal per evitare oscillazioni
            damping_factor = _velocity_damping * 0.5;  // Dimezza velocità vicino al goal
        }
        
        smoothed_force *= damping_factor;

        // Applica limiti di velocità con clamp per evitare overshoot
        if (smoothed_force.norm() > _max_lin_speed) {
            smoothed_force = smoothed_force.normalized() * _max_lin_speed;
        }
        
        // Clamp componentwise per controllare meglio le oscillazioni
        smoothed_force = smoothed_force.cwiseMax(Eigen::Vector3d(-_max_lin_speed, -_max_lin_speed, -_max_lin_speed));
        smoothed_force = smoothed_force.cwiseMin(Eigen::Vector3d(_max_lin_speed, _max_lin_speed, _max_lin_speed));

        double wz_o = 0.0;
        double yaw_error = 0.0;

        if (smoothed_force.norm() > 0.05) {
            yaw_error = utilities::angleError(std::atan2(Fa_o.normalized().y(), Fa_o.normalized().x()), rpy_o_b(2));
        }

        wz_o = _k_yaw * yaw_error;
        
        Eigen::Vector3d final_velocity = std::exp(-4.0 * fabs(yaw_error)) * smoothed_force;

        if (fabs(wz_o) > _max_ang_speed) {
            wz_o = _max_ang_speed * (wz_o > 0 ? 1.0 : -1.0);
        }

        _cmd_velocity = final_velocity.cast<float>();
        _cmd_yaw_rate = wz_o;

        publish_velocity_command(final_velocity, wz_o);

        // Debug delle forze per monitorare il bilanciamento
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
            "APF Forces - Attractive: %.3f, Rep_map: %.3f, Rep_body: %.3f, Total: %.3f, Goal_dist: %.3f", 
            Fa_o.norm(), Fr_m.norm(), Fr_b.norm(), Ftot_o.norm(), d_g);

        if (_enable_visualization) {
            publish_debug_visualization(Fa_o, Fr_m, Fr_b, Ftot_o, g_m);
        }

    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Error getting robot position: %s", ex.what());
    }
}

Eigen::Vector3d LocalTrajectoryInterpolator::get_lookahead_point(const Eigen::Vector3d& current_pos) {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    
    if(_waypoint_queue.empty()) {
        return current_pos;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::queue<geometry_msgs::msg::PoseStamped> temp_queue = _waypoint_queue;
    while (!temp_queue.empty()) {
        waypoints.push_back(temp_queue.front());
        temp_queue.pop();
    }

    double min_dist = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto& pose = waypoints[i].pose.position;
        Eigen::Vector3d point(pose.x, pose.y, pose.z);
        double dist = (point - current_pos).norm();
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    size_t target_idx = closest_idx;
    for (size_t i = closest_idx; i < waypoints.size(); ++i) {
        auto& pose = waypoints[i].pose.position;
        Eigen::Vector3d point(pose.x, pose.y, pose.z);
        
        if ((point - current_pos).norm() >= _lookahead_distance) {
            target_idx = i;
            break;
        } else {
            target_idx = waypoints.size() - 1;
        }
    }

    auto& target = waypoints[target_idx].pose.position;
    return Eigen::Vector3d(target.x, target.y, target.z);
}

Eigen::Vector3d LocalTrajectoryInterpolator::get_goal_point(const Eigen::Vector3d& current_pos) {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    if(_waypoint_queue.empty()) {
        return current_pos;
    }

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    std::queue<geometry_msgs::msg::PoseStamped> temp_queue = _waypoint_queue;
    while (!temp_queue.empty()) {
        waypoints.push_back(temp_queue.front());
        temp_queue.pop();
    }

    auto& target = waypoints.back().pose.position;
    return Eigen::Vector3d(target.x, target.y, target.z);
}

Eigen::Vector3d LocalTrajectoryInterpolator::calculate_repulsive_force_map(const Eigen::Vector3d& position) {
    std::lock_guard<std::mutex> lock(_map_cloud_mutex);
    
    if(_map_obstacle_cloud->empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    pcl::PointXYZ search_point(position.x(), position.y(), position.z());
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    // Usa il raggio di influenza come limite assoluto per il search
    _map_cloud_kdtree.radiusSearch(search_point, _obstacle_influence_range, point_indices, point_distances);

    Eigen::Vector3d repulsive_force = Eigen::Vector3d::Zero();
    int considered_obstacles = 0;
    
    for (size_t i = 0; i < point_indices.size(); ++i) {
        auto& pt = _map_obstacle_cloud->points[point_indices[i]];
        Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
        Eigen::Vector3d direction = position - obstacle;
        double distance = direction.norm();
        
        if (distance < 1e-5) continue;
        
        direction.normalize();
        
        // Funzione repulsiva che va smoothly a zero al raggio di influenza
        // Usa una funzione più semplice e robusta: F(d) = k_rep * (r_influence - d)^2 / r_influence^2
        // Questa funzione è continua, derivabile, e va a zero quando d = r_influence
        
        double r_influence = _obstacle_influence_range;
        
        // Se la distanza è maggiore o uguale al raggio di influenza, forza = 0
        if (distance >= r_influence * 0.95) continue;  // Margine per errori numerici
        
        // Funzione quadratica che va a zero al bordo del raggio di influenza
        double distance_factor = (r_influence - distance) / r_influence;
        double force_magnitude = _k_repulsive * distance_factor * distance_factor / (distance * distance);
        
        // Debug per vedere se stiamo effettivamente calcolando forze
        if (considered_obstacles == 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                "First obstacle: dist=%.3f, r_inf=%.3f, factor=%.3f, force=%.3f", 
                distance, r_influence, distance_factor, force_magnitude);
        }
        
        // Limita la forza massima per singolo ostacolo
        force_magnitude = std::min(force_magnitude, _max_repulsive_force / 5.0);
        
        repulsive_force += force_magnitude * direction;
        considered_obstacles++;
        
        // Considera solo i N ostacoli più vicini per evitare accumulo eccessivo
        if (considered_obstacles >= 8) break;
    }

    // Normalizza in base al numero di ostacoli considerati per evitare accumulo
    if (considered_obstacles > 1) {
        repulsive_force /= std::sqrt(considered_obstacles);
    }

    // Applica limite finale sulla forza totale
    if (repulsive_force.norm() > _max_repulsive_force) {
        repulsive_force = repulsive_force.normalized() * _max_repulsive_force;
    }
    
    // Debug info throttled
    if (considered_obstacles > 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
            "Map repulsive force - Obstacles: %d, Force norm: %.3f", 
            considered_obstacles, repulsive_force.norm());
    }
    
    return repulsive_force;
}

Eigen::Vector3d LocalTrajectoryInterpolator::calculate_repulsive_force_body() {
    std::lock_guard<std::mutex> lock(_body_cloud_mutex);
    
    if(_body_obstacle_cloud->empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    pcl::PointXYZ search_point(0.0, 0.0, 0.0);
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    // Body frame usa un raggio di influenza ridotto per essere più reattivo
    double body_influence_range = _obstacle_influence_range * 0.7;
    _body_cloud_kdtree.radiusSearch(search_point, body_influence_range, point_indices, point_distances);

    Eigen::Vector3d repulsive_force = Eigen::Vector3d::Zero();
    int considered_obstacles = 0;
    
    for (size_t i = 0; i < point_indices.size(); ++i) {
        auto& pt = _body_obstacle_cloud->points[point_indices[i]];
        Eigen::Vector3d obstacle(pt.x, pt.y, pt.z);
        Eigen::Vector3d direction = -obstacle;  // Direzione opposta all'ostacolo
        double distance = direction.norm();
        
        if (distance < 1e-5) continue;
        
        direction.normalize();
        
        // Stessa funzione repulsiva smooth del map, ma più aggressiva per body frame
        // F(d) = k_rep * (r_influence - d)^2 / r_influence^2 / d^2
        
        double r_influence = body_influence_range;
        
        // Se la distanza è maggiore o uguale al raggio di influenza, forza = 0
        if (distance >= r_influence * 0.95) continue;  // Margine per errori numerici
        
        // Funzione quadratica più aggressiva per body frame
        double distance_factor = (r_influence - distance) / r_influence;
        double force_magnitude = _k_repulsive * 1.5 * distance_factor * distance_factor / (distance * distance);
        
        // Debug per body frame
        if (considered_obstacles == 0) {
            RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,
                "Body first obstacle: dist=%.3f, r_inf=%.3f, factor=%.3f, force=%.3f", 
                distance, r_influence, distance_factor, force_magnitude);
        }
        
        // Limita forza per singolo ostacolo
        force_magnitude = std::min(force_magnitude, _max_repulsive_force / 3.0);
        
        repulsive_force += force_magnitude * direction;
        considered_obstacles++;
        
        // Meno ostacoli per body frame per evitare accumulo eccessivo
        if (considered_obstacles >= 5) break;
    }

    // Normalizza in base al numero di ostacoli
    if (considered_obstacles > 1) {
        repulsive_force /= std::sqrt(considered_obstacles);
    }

    // Limite finale più restrittivo per body frame
    double max_body_force = _max_repulsive_force * 0.8;
    if (repulsive_force.norm() > max_body_force) {
        repulsive_force = repulsive_force.normalized() * max_body_force;
    }
    
    // Debug info throttled
    if (considered_obstacles > 0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000,
            "Body repulsive force - Obstacles: %d, Force norm: %.3f", 
            considered_obstacles, repulsive_force.norm());
    }
    
    return repulsive_force;
}

void LocalTrajectoryInterpolator::publish_velocity_command(const Eigen::Vector3d& velocity, double angular_z) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = velocity.x();
    cmd.linear.y = velocity.y();
    cmd.linear.z = velocity.z();
    cmd.angular.z = angular_z;
    _cmd_vel_publisher->publish(cmd);
}

visualization_msgs::msg::Marker LocalTrajectoryInterpolator::create_arrow_marker(
    const Eigen::Vector3d& start, const Eigen::Vector3d& force, const std::string frame,
    int id, float r, float g, float b) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = now();
    marker.ns = "forces";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point start_point;
    start_point.x = start.x();
    start_point.y = start.y();
    start_point.z = start.z();
    
    geometry_msgs::msg::Point end_point;
    end_point.x = start.x() + force.x();
    end_point.y = start.y() + force.y();
    end_point.z = start.z() + force.z();
    
    marker.points.push_back(start_point);
    marker.points.push_back(end_point);
    
    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.0;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    return marker;
}

visualization_msgs::msg::Marker LocalTrajectoryInterpolator::create_sphere_marker(
    const Eigen::Vector3d& position, const std::string frame, int id, float r, float g, float b) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = now();
    marker.ns = "lookahead";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    return marker;
}

void LocalTrajectoryInterpolator::publish_debug_visualization(
    const Eigen::Vector3d& attractive_force, const Eigen::Vector3d& repulsive_force_map,
    const Eigen::Vector3d& repulsive_force_body, const Eigen::Vector3d& total_force,
    const Eigen::Vector3d& lookahead_point) {
    
    visualization_msgs::msg::Marker goal_marker = create_sphere_marker(lookahead_point, _map_frame, 0, 0.0, 1.0, 0.0);
    _lookahead_pub->publish(goal_marker);
    
    visualization_msgs::msg::MarkerArray forces_markers;
    
    try {
        geometry_msgs::msg::TransformStamped tf_o_b = _tf_buffer->lookupTransform(_odom_frame, _robot_frame, tf2::TimePointZero);
        Eigen::Vector3d x_o_b = utilities::pose_from_tf(tf_o_b);
        
        geometry_msgs::msg::TransformStamped tf_m_b = _tf_buffer->lookupTransform(_map_frame, _robot_frame, tf2::TimePointZero);
        Eigen::Vector3d x_m_b = utilities::pose_from_tf(tf_m_b);
        
        forces_markers.markers.push_back(create_arrow_marker(x_o_b, attractive_force, _odom_frame, 0, 0.0, 1.0, 0.0));
        forces_markers.markers.push_back(create_arrow_marker(x_m_b, repulsive_force_map, _map_frame, 1, 1.0, 0.0, 0.0));
        forces_markers.markers.push_back(create_arrow_marker(Eigen::Vector3d::Zero(), repulsive_force_body, _robot_frame, 2, 1.0, 0.0, 0.58));
        forces_markers.markers.push_back(create_arrow_marker(x_o_b, total_force, _odom_frame, 3, 0.0, 0.0, 1.0));
        
        _forces_pub->publish(forces_markers);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Error getting transforms for visualization: %s", ex.what());
    }
}

void LocalTrajectoryInterpolator::control_timer_callback() {
    if (!_first_odom_received) {
        return;
    }

    if (!_offboard_mode && _startup) {
        return;
    }
    
    _offboard_setpoint_counter++;
    
    if (_teleop_active) {
        handle_teleop_mode();
        return;
    }
    
    if (_has_target && _state == FOLLOWING_TRAJECTORY && _local_planner_enabled) {
        compute_apf_velocity();
        
        Eigen::Vector3d current_pos_d(_current_position.x(), _current_position.y(), _current_position.z());
        Eigen::Vector3d goal_pos = get_goal_point(current_pos_d);
        double distance_to_goal = (goal_pos - current_pos_d).norm();
        
        if (distance_to_goal < _waypoint_tolerance) {
            _has_target = false;
            _state = IDLE;
            RCLCPP_INFO(get_logger(), "Trajectory completed - reached final waypoint");
            
            std_msgs::msg::String completed_msg;
            completed_msg.data = "traj_completed";
            _debug_publisher->publish(completed_msg);
        }
    } else if (_state == STOPPED && !_teleop_active) {
        RCLCPP_INFO(get_logger(), "Trajectory stopped");
        publish_velocity_command(Eigen::Vector3d::Zero(), 0.0);
    }
    else{
        publish_velocity_command(Eigen::Vector3d::Zero(), 0.0);
    }
    
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
    
    if (_path_mode == "circle" && _state == FOLLOWING_TRAJECTORY && 
        _current_waypoint_index >= _tilting_start_index && _tilting_on_pitch_enabled) {
        
        if (_current_pitch < _tilting_goal_pitch) {
            _current_pitch += _tilting_interp_rate * _dt;
            if (_current_pitch > _tilting_goal_pitch) _current_pitch = _tilting_goal_pitch;
        } else if (_current_pitch > _tilting_goal_pitch) {
            _current_pitch -= _tilting_interp_rate * _dt;
            if (_current_pitch < _tilting_goal_pitch) _current_pitch = _tilting_goal_pitch;
        }
    } else {
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

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalTrajectoryInterpolator>());
    rclcpp::shutdown();
    return 0;
}