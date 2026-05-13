// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#ifndef RRT_H
#define RRT_H

#include "nav_msgs/msg/path.hpp"

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rrt/occupancy_grid.h"
#include <tf2_ros/transform_broadcaster.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the Waypoint object
typedef struct Waypoint {
    double x;
    double y;
    // Add any other necessary fields (like theta or velocity) if your cpp file uses them
} Waypoint;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to this struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;


class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // Publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_marker_pub_;

    // Timer for RRT updates
    rclcpp::TimerBase::SharedPtr timer_;

    // Occupancy grid
    std::unique_ptr<LocalOccupancyGrid> occupancy_grid_;

    // Robot pose in map frame
    double robot_map_x_;
    double robot_map_y_;
    double robot_map_yaw_;

    // Random generator
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    // RRT Parameters (loaded from config file)
    double param_x_dist_min_, param_x_dist_max_;
    double param_y_dist_min_, param_y_dist_max_;
    double param_grid_resolution_;
    int param_grid_width_, param_grid_height_;
    double param_grid_origin_x_, param_grid_origin_y_;
    double param_inflation_radius_;
    int param_max_iterations_;
    double param_steer_step_size_;
    double param_collision_check_resolution_;
    double param_goal_tolerance_;
    double param_max_goal_distance_;
    int param_rrt_update_frequency_ms_;
    double param_waypoint_threshold_;
    double param_min_dist_to_goal_;

    // Waypoint tracking variables
    std::vector<Waypoint> waypoints_;
    int current_wp_idx_;

    // Callbacks
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    
    // Main RRT loop
    void run_rrt();

    // Waypoint and Goal functions
    void load_waypoints(const std::string &file);
    void update_goal();
    
    // Coordinate transforms
    void transform_to_map(double x_b, double y_b, double &x_m, double &y_m);
    void map_to_base(double x_m, double y_m, double &x_b, double &y_b);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, int idx);
    
    // Visualization
    void visualize_tree(const std::vector<RRT_Node> &tree);
    
    // RRT* methods (for extra credit)
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};

#endif // RRT_H