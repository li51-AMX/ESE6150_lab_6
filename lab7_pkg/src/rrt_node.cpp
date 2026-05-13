#include "rrt/rrt.h"

#include <cmath>
#include <algorithm>
#include <random>
#include <fstream>
#include <sstream>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// ===================== CONSTRUCTOR =====================
RRT::~RRT() {}

RRT::RRT()
: rclcpp::Node("rrt_node"),
  gen(std::random_device{}())
{
    RCLCPP_INFO(this->get_logger(), "RRT node started");

    // ===== DECLARE AND LOAD PARAMETERS =====
    // Sampling range
    declare_parameter("x_dist_min", -0.5);
    declare_parameter("x_dist_max", 4.0);
    declare_parameter("y_dist_min", -2.0);
    declare_parameter("y_dist_max", 2.0);

    // Grid configuration
    declare_parameter("grid_resolution", 0.05);
    declare_parameter("grid_width", 162);
    declare_parameter("grid_height", 40);
    declare_parameter("grid_origin_x", -0.05);
    declare_parameter("grid_origin_y", -1.0);
    declare_parameter("inflation_radius", 0.40);

    // RRT algorithm
    declare_parameter("max_iterations", 300);
    declare_parameter("steer_step_size", 0.7);
    declare_parameter("collision_check_resolution", 0.05);
    declare_parameter("goal_tolerance", 0.3);
    declare_parameter("max_goal_distance", 4.0);

    // Timing
    declare_parameter("rrt_update_frequency_ms", 100);

    // Waypoint tracking
    declare_parameter("waypoint_threshold", 2.0);
    declare_parameter("min_dist_to_goal", 1.5);

    // Get parameter values
    param_x_dist_min_ = get_parameter("x_dist_min").as_double();
    param_x_dist_max_ = get_parameter("x_dist_max").as_double();
    param_y_dist_min_ = get_parameter("y_dist_min").as_double();
    param_y_dist_max_ = get_parameter("y_dist_max").as_double();

    param_grid_resolution_ = get_parameter("grid_resolution").as_double();
    param_grid_width_ = get_parameter("grid_width").as_int();
    param_grid_height_ = get_parameter("grid_height").as_int();
    param_grid_origin_x_ = get_parameter("grid_origin_x").as_double();
    param_grid_origin_y_ = get_parameter("grid_origin_y").as_double();
    param_inflation_radius_ = get_parameter("inflation_radius").as_double();

    param_max_iterations_ = get_parameter("max_iterations").as_int();
    param_steer_step_size_ = get_parameter("steer_step_size").as_double();
    param_collision_check_resolution_ = get_parameter("collision_check_resolution").as_double();
    param_goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    param_max_goal_distance_ = get_parameter("max_goal_distance").as_double();

    param_rrt_update_frequency_ms_ = get_parameter("rrt_update_frequency_ms").as_int();
    param_waypoint_threshold_ = get_parameter("waypoint_threshold").as_double();
    param_min_dist_to_goal_ = get_parameter("min_dist_to_goal").as_double();

    // Log parameters
    RCLCPP_INFO(this->get_logger(), "=== RRT Parameters Loaded ===");
    RCLCPP_INFO(this->get_logger(), "Sampling X: [%.2f, %.2f]", param_x_dist_min_, param_x_dist_max_);
    RCLCPP_INFO(this->get_logger(), "Sampling Y: [%.2f, %.2f]", param_y_dist_min_, param_y_dist_max_);
    RCLCPP_INFO(this->get_logger(), "Grid: %dx%d, resolution: %.3f", param_grid_width_, param_grid_height_, param_grid_resolution_);
    RCLCPP_INFO(this->get_logger(), "Max iterations: %d, Step size: %.2f", param_max_iterations_, param_steer_step_size_);

    // Initialize random distributions with parameters
    x_dist = std::uniform_real_distribution<>(param_x_dist_min_, param_x_dist_max_);
    y_dist = std::uniform_real_distribution<>(param_y_dist_min_, param_y_dist_max_);

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10,
        std::bind(&RRT::pose_callback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>("/rrt_path", 10);

    occupancy_grid_pub_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/rrt/grid", 10);

    occupancy_grid_ = std::make_unique<LocalOccupancyGrid>(
        param_grid_resolution_,
        param_grid_width_,
        param_grid_height_,
        param_grid_origin_x_,
        param_grid_origin_y_,
        param_inflation_radius_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(param_rrt_update_frequency_ms_),
        std::bind(&RRT::run_rrt, this));

    robot_map_x_ = 0.0;
    robot_map_y_ = 0.0;
    robot_map_yaw_ = 0.0;
    current_wp_idx_ = 0;

    // LOAD WAYPOINTS
    // load_waypoints("/home/etsa/f1/roboracer_ws/src/lab7_pkg/waypoints/levine_2nd_waypoints1.csv");
    load_waypoints("/home/avani/sim_ws/src/lab-6-motion-planning-team3/lab7_pkg/waypoints/levine_2nd_waypoints3.csv");
}

// ===================== WAYPOINT FUNCTIONS =====================

void RRT::load_waypoints(const std::string &file)
{
    std::ifstream f(file);
    std::string line;

    if (!f.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file");
        return;
    }

    std::getline(f, line); // skip header

    while (std::getline(f, line)) {
        std::stringstream ss(line);
        std::string val;

        Waypoint wp;

        std::getline(ss, val, ','); wp.x = std::stod(val);
        std::getline(ss, val, ','); wp.y = std::stod(val);

        waypoints_.push_back(wp);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %ld waypoints", waypoints_.size());
}

void RRT::update_goal()
{
    if (waypoints_.empty()) return;

    const double min_dist = 3.0;  // TUNE (minimum goal distance)

    int attempts = 0;
    int max_attempts = waypoints_.size();

    while (attempts < max_attempts) {
        auto &goal = waypoints_[current_wp_idx_];

        double dx = robot_map_x_ - goal.x;
        double dy = robot_map_y_ - goal.y;

        double dist = std::hypot(dx, dy);

        // If far enough, keep this goal
        if (dist >= min_dist) {
            return;
        }

        // Otherwise move to next waypoint
        current_wp_idx_ = (current_wp_idx_ + 1) % waypoints_.size();
        attempts++;
    }

    // Fallback: if none are far enough, keep current index
}


void RRT::map_to_base(double x_m, double y_m,
                      double &x_b, double &y_b)
{
    double dx = x_m - robot_map_x_;
    double dy = y_m - robot_map_y_;

    x_b =  std::cos(robot_map_yaw_) * dx +
           std::sin(robot_map_yaw_) * dy;

    y_b = -std::sin(robot_map_yaw_) * dx +
           std::cos(robot_map_yaw_) * dy;
}

// ===================== POSE CALLBACK =====================

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    robot_map_x_ = msg->pose.pose.position.x;
    robot_map_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    robot_map_yaw_ = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    );
}

// ===================== SCAN =====================

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
{
    occupancy_grid_->update_from_scan(*scan_msg);

    auto msg = occupancy_grid_->to_message("/ego_racecar/base_link", scan_msg->header.stamp);
    occupancy_grid_pub_->publish(msg);
}

// ===================== TRANSFORM =====================

void RRT::transform_to_map(double x_b, double y_b,
                          double &x_m, double &y_m)
{
    x_m = robot_map_x_ +
          std::cos(robot_map_yaw_) * x_b -     // TUNE (transform accuracy)
          std::sin(robot_map_yaw_) * y_b;

    y_m = robot_map_y_ +
          std::sin(robot_map_yaw_) * x_b +
          std::cos(robot_map_yaw_) * y_b;
}

// ===================== MAIN RRT =====================

void RRT::run_rrt()
{
    update_goal();
    if (waypoints_.empty()) return;

    Waypoint goal_wp = waypoints_[current_wp_idx_];

    double goal_bx, goal_by;
    map_to_base(goal_wp.x, goal_wp.y, goal_bx, goal_by);

    // Limit goal distance
    double max_dist = param_max_goal_distance_;
    double d = std::hypot(goal_bx, goal_by);
    if (d > max_dist) {
        goal_bx = goal_bx / d * max_dist;
        goal_by = goal_by / d * max_dist;
    }

    std::vector<RRT_Node> tree;

    RRT_Node start;
    start.x = 0.0;
    start.y = 0.0;
    start.parent = -1;

    tree.push_back(start);

    std::vector<RRT_Node> path;

    const int max_iter = param_max_iterations_;

    for (int i = 0; i < max_iter; i++) {

        auto sample_pt = sample();

        int nearest_idx = nearest(tree, sample_pt);
        RRT_Node nearest_node = tree[nearest_idx];

        RRT_Node new_node = steer(nearest_node, sample_pt);

        if (!check_collision(nearest_node, new_node)) {
            continue;
        }

        new_node.parent = nearest_idx;
        tree.push_back(new_node);

        if (is_goal(new_node, goal_bx, goal_by)) {
            path = find_path(tree, tree.size() - 1);
            break;
        }
    }

    if (!path.empty()) {
        // path = smooth_path(path);   // TUNE (smoothing iterations below)
        // path = resample_path(path, 0.2);

        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();

        for (auto &n : path) {

            double mx, my;
            transform_to_map(n.x, n.y, mx, my);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;

            pose.pose.position.x = mx;
            pose.pose.position.y = my;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;

            msg.poses.push_back(pose);
        }

        path_pub_->publish(msg);
    }
}

// ===================== RRT FUNCTIONS =====================

std::vector<double> RRT::sample()
{
    return {x_dist(gen), y_dist(gen)};   // TUNE (sampling distribution)
}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &pt)
{
    int best = 0;
    double best_d = 1e9;

    for (size_t i = 0; i < tree.size(); i++) {
        double dx = tree[i].x - pt[0];
        double dy = tree[i].y - pt[1];
        double d = dx * dx + dy * dy;

        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

RRT_Node RRT::steer(RRT_Node &from, std::vector<double> &to)
{
    double step = param_steer_step_size_;

    double dx = to[0] - from.x;
    double dy = to[1] - from.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    RRT_Node node;

    if (dist < step) {
        node.x = to[0];
        node.y = to[1];
    } else {
        node.x = from.x + dx / dist * step;
        node.y = from.y + dy / dist * step;
    }

    node.parent = -1;
    return node;
}

bool RRT::check_collision(RRT_Node &a, RRT_Node &b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;

    int steps = std::hypot(dx, dy) / param_collision_check_resolution_ + 1;

    for (int i = 0; i <= steps; i++) {
        double t = double(i) / steps;
        double x = a.x + t * dx;
        double y = a.y + t * dy;

        int gx, gy;

        if (!occupancy_grid_->world_to_grid(x, y, gx, gy)) return false;

        if (occupancy_grid_->value_at(gx, gy) == 100) return false;
    }

    return true;
}

bool RRT::is_goal(RRT_Node &node, double gx, double gy)
{
    double dx = node.x - gx;
    double dy = node.y - gy;
    return std::sqrt(dx * dx + dy * dy) < param_goal_tolerance_;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, int idx)
{
    std::vector<RRT_Node> path;

    while (idx != -1) {
        path.push_back(tree[idx]);
        idx = tree[idx].parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

// ===================== MAIN =====================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT>());
    rclcpp::shutdown();
    return 0;
}