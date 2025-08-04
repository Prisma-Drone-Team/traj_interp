#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Utility function to extract yaw from quaternion
float quaternion_to_yaw(const Eigen::Quaternionf& q) {
    const float& q0 = q.w();
    const float& q1 = q.x();
    const float& q2 = q.y(); 
    const float& q3 = q.z();
    return std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

#include <queue>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum InterpolatorState {
    IDLE = 0,
    FOLLOWING_TRAJECTORY = 1,
    STOPPED = 2
};

class TrajectoryInterpolator : public rclcpp::Node {
public:
    TrajectoryInterpolator();

private:
    // Timer callbacks
    void control_timer_callback();
    void status_timer_callback();
    
    // Subscriber callbacks
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    
    // PX4 Commands
    void arm();
    void disarm();
    void engage_offboard_mode();
    
    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr _trajectory_setpoint_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _status_publisher;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _vehicle_control_mode_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_subscription;
    
    // Timers
    rclcpp::TimerBase::SharedPtr _control_timer;
    rclcpp::TimerBase::SharedPtr _status_timer;
    
    // Trajectory interpolation (based on ffilter from lee_controller)
    void interpolate_trajectory();
    void set_new_target(const Eigen::Vector3f& target_pos, float target_yaw);
    
    // Utility functions
    float calculate_heading_yaw(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& target_pos);
    float extract_yaw_from_quaternion(const Eigen::Quaternionf& q);
    void clear_waypoint_queue();
    
    // Utility functions
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    void publish_trajectory_setpoint();
    void publish_offboard_control_mode();
    
    // State variables
    InterpolatorState _state;
    std::atomic<bool> _first_odom_received{false};
    bool _first_path_received{false};
    
    // PX4 State variables
    bool _armed{false};
    bool _landed{true};
    bool _offboard_mode{false};
    
    // Offboard control
    uint64_t _offboard_setpoint_counter{0};
    static constexpr uint64_t OFFBOARD_SETPOINTS_REQUIRED = 10; // Send 10 setpoints before arming
    
    // Current position and attitude
    Eigen::Vector3f _current_position{};
    Eigen::Quaternionf _current_attitude{};
    
    // Trajectory following variables
    std::queue<geometry_msgs::msg::PoseStamped> _waypoint_queue;
    std::mutex _queue_mutex;
    
    geometry_msgs::msg::PoseStamped _current_target;
    bool _has_target{false};
    
    // Interpolation variables (ffilter implementation)
    Eigen::Vector3f _cmd_position{};      // Command position (target)
    Eigen::Vector3f _ref_position{};      // Reference position (filtered)
    Eigen::Vector3f _ref_velocity{};      // Reference velocity
    Eigen::Vector3f _ref_acceleration{};  // Reference acceleration
    
    float _cmd_yaw{0.0f};      // Command yaw
    float _ref_yaw{0.0f};      // Reference yaw (filtered)
    float _ref_yaw_rate{0.0f}; // Reference yaw rate
    float _ref_yaw_acc{0.0f};  // Reference yaw acceleration
    
    // Filter parameters (from ffilter)
    double _ref_jerk_max{2.0};        // Maximum jerk [m/s³]
    double _ref_acc_max{1.0};         // Maximum acceleration [m/s²]
    double _ref_vel_max{1.0};         // Maximum velocity [m/s]
    double _ref_omega{1.0};           // Filter frequency [rad/s]
    double _ref_zeta{0.7};            // Damping ratio
    
    double _ref_yaw_jerk_max{1.0};    // Maximum yaw jerk [rad/s³]
    double _ref_yaw_acc_max{0.5};     // Maximum yaw acceleration [rad/s²]
    double _ref_yaw_vel_max{0.5};     // Maximum yaw velocity [rad/s]
    
    // Timing
    double _control_frequency{50.0};  // Control loop frequency [Hz]
    double _dt;                       // Control loop timestep
    
    // Waypoint following
    double _waypoint_tolerance{0.1};  // Distance tolerance to consider waypoint reached [m]
    double _yaw_tolerance{0.1};       // Yaw tolerance [rad]
    
    // Topic names
    std::string _path_topic;
    std::string _odometry_topic;
    std::string _offboard_control_mode_topic;
    std::string _vehicle_command_topic;
    std::string _trajectory_setpoint_topic;
    std::string _status_topic;
    
    std::atomic<uint64_t> _timestamp{0};
};
