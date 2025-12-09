#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/tilting_mc_desired_angles.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "traj_interp/utils.h"

#include <queue>
#include <vector>
#include <mutex>
#include <atomic>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

enum InterpolatorState {
    IDLE = 0,
    FOLLOWING_TRAJECTORY = 1,
    STOPPED = 2
};

class LocalTrajectoryInterpolator : public rclcpp::Node {
public:
    LocalTrajectoryInterpolator();

private:
    // Timer callbacks
    void control_timer_callback();
    void status_timer_callback();
    
    // Subscriber callbacks
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
    void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
    void teleop_active_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void velocity_increments_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void path_mode_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // Local Planner specific callbacks
    void obstacle_cloud_map_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void obstacle_cloud_body_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void local_planner_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void collision_detected_callback(const std_msgs::msg::Bool::SharedPtr msg);

    // Teleop handling
    void handle_teleop_mode();
    
    // Local Planner APF methods
    void compute_apf_velocity();
    Eigen::Vector3d get_lookahead_point(const Eigen::Vector3d& current_pos);
    Eigen::Vector3d get_goal_point(const Eigen::Vector3d& current_pos);
    Eigen::Vector3d calculate_repulsive_force_map(const Eigen::Vector3d& position);
    Eigen::Vector3d calculate_repulsive_force_body();
    void publish_velocity_command(const Eigen::Vector3d& velocity, double angular_z);
    
    // Visualization methods
    visualization_msgs::msg::Marker create_arrow_marker(const Eigen::Vector3d& start, const Eigen::Vector3d& force, 
                                                       const std::string frame, int id, float r, float g, float b);
    visualization_msgs::msg::Marker create_sphere_marker(const Eigen::Vector3d& position, const std::string frame, 
                                                        int id, float r, float g, float b);
    void publish_debug_visualization(const Eigen::Vector3d& attractive_force, const Eigen::Vector3d& repulsive_force_map,
                                   const Eigen::Vector3d& repulsive_force_body, const Eigen::Vector3d& total_force,
                                   const Eigen::Vector3d& lookahead_point);
    
    // PX4 Commands
    void arm();
    void disarm();
    void engage_offboard_mode();
    
    // Utility functions
    float calculate_heading_yaw(const Eigen::Vector3f& current_pos, const Eigen::Vector3f& target_pos);
    float extract_yaw_from_quaternion(const Eigen::Quaternionf& q);
    void clear_waypoint_queue();
    geometry_msgs::msg::PoseStamped transform_pose_to_odom(const geometry_msgs::msg::PoseStamped& pose_in_map);
    void tf_lookup_loop();
    std::vector<geometry_msgs::msg::PoseStamped> resample_path(const std::vector<geometry_msgs::msg::PoseStamped>& original_path, double sampling_distance = 0.4);
    
    // Utility functions
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0);
    void publish_trajectory_setpoint();
    
    // Publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr _vehicle_command_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;  // Changed to Twist for local planner
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _status_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _transformed_path_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _debug_publisher;
    rclcpp::Publisher<px4_msgs::msg::TiltingMcDesiredAngles>::SharedPtr _tilting_pub;
    
    // Local Planner debug publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _forces_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _lookahead_pub;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _vehicle_control_mode_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _land_detected_subscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _teleop_active_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _velocity_increments_subscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _path_mode_subscription;
    
    // Local Planner specific subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _obstacle_cloud_map_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _obstacle_cloud_body_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _local_planner_enable_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _collision_detected_sub;
    
    // Timers
    rclcpp::TimerBase::SharedPtr _control_timer;
    rclcpp::TimerBase::SharedPtr _status_timer;
    
    // State variables
    InterpolatorState _state;
    std::atomic<bool> _first_odom_received{false};
    bool _first_path_received{false};
    
    // PX4 State variables
    bool _armed{false};
    bool _landed{true};
    bool _offboard_mode{false};
    bool _was_offboard{false};
    bool _startup{false};
    
    // Offboard control
    uint64_t _offboard_setpoint_counter{0};
    static constexpr uint64_t OFFBOARD_SETPOINTS_REQUIRED = 10;
    
    // Current position and attitude
    Eigen::Vector3f _current_position{};
    Eigen::Quaternionf _current_attitude{};
    
    // Trajectory following variables
    std::queue<geometry_msgs::msg::PoseStamped> _waypoint_queue;
    std::mutex _queue_mutex;
    
    geometry_msgs::msg::PoseStamped _current_target;
    bool _has_target{false};
    
    // Path visualization
    nav_msgs::msg::Path _transformed_path;
    
    // APF Local Planner variables
    Eigen::Vector3f _cmd_velocity{};      // Command velocity from APF
    float _cmd_yaw_rate{0.0f};           // Command yaw rate from APF
    
    // APF Parameters
    double _max_lin_speed{1.0};
    double _max_ang_speed{0.5};
    double _safety_distance{2.0};
    double _lookahead_distance{2.5};
    double _k_attractive{2.0};
    double _k_repulsive{5.0};
    double _k_yaw{1.5};
    double _force_smoothing_factor{0.8};
    double _velocity_damping{0.9};
    double _goal_attraction_threshold{0.5};
    double _obstacle_influence_range{2.0};
    
    // Nuovi parametri per potenziali non lineari e avoidance locale
    double _local_avoidance_radius{3.0};      // Raggio per considerare ostacoli
    double _repulsive_exp_factor{2.0};        // Fattore esponenziale per repulsione
    double _min_obstacle_distance{0.3};       // Distanza minima critica
    double _max_repulsive_force{3.0};         // Forza repulsiva massima
    double _attractive_scale_distance_max{2.0}; // d_max per scaling tanh della forza attrattiva (paper)
    
    // Obstacle clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr _map_obstacle_cloud{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::PointCloud<pcl::PointXYZ>::Ptr _body_obstacle_cloud{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::KdTreeFLANN<pcl::PointXYZ> _map_cloud_kdtree;
    pcl::KdTreeFLANN<pcl::PointXYZ> _body_cloud_kdtree;
    std::mutex _map_cloud_mutex;
    std::mutex _body_cloud_mutex;
    
    // Local planner state
    bool _local_planner_enabled{true};
    bool _collision_detected{false};
    
    // Timing
    double _control_frequency{50.0};
    double _dt;
    
    // Waypoint following
    double _waypoint_tolerance{0.1};
    double _yaw_tolerance{0.1};
    double _vertical_movement_threshold{0.2};
    double _resampling_distance{0.3};
    
    // TF2 for coordinate transformations
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    geometry_msgs::msg::TransformStamped _tf_map_to_odom, _tf_odom_to_map;
    std::string _parent_transf{"map"};
    std::string _child_transf{"odom"};
    bool _do_transform{true};
    double _tf_buffer_timeout{0.5};
    
    // Frames
    std::string _robot_frame{"base_link"};
    std::string _map_frame{"map"};
    std::string _odom_frame{"odom"};
    
    // Topic names
    std::string _path_topic;
    std::string _odometry_topic;
    std::string _offboard_control_mode_topic;
    std::string _vehicle_command_topic;
    std::string _trajectory_setpoint_topic;
    std::string _status_topic;
    std::string _obstacle_cloud_map_topic;
    std::string _obstacle_cloud_body_topic;
    std::string _local_planner_enable_topic;
    std::string _collision_detected_topic;
    
    // Teleop coordination
    bool _teleop_active{false};
    geometry_msgs::msg::Twist _velocity_increments;
    Eigen::Vector3d _teleop_base_position;
    double _teleop_base_yaw{0.0};
    
    std::atomic<uint64_t> _timestamp{0};

    // Loiter and tilting parameters
    int _loiter_segment;
    double _tilting_goal_pitch{0.0};
    double _tilting_interp_rate{0.1};
    double _current_pitch{0.0};
    int _tilting_start_index{0};
    int _total_waypoints{0};
    int _current_waypoint_index{0};
    std::string _path_mode{""};
    bool _tilting_on_pitch_enabled;
    bool _choose_final_yaw;
    geometry_msgs::msg::PoseStamped _approach_penultimate;
    
    // Visualization
    bool _enable_visualization{true};
    
    // Stuck prevention
    int _gnron_cnt_{0};
    double _last_goal_distance_{0.0};
};