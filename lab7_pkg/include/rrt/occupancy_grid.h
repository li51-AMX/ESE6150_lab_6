#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LocalOccupancyGrid {
public:
    LocalOccupancyGrid(
      double resolution,
      int width_cells,
      int height_cells,
      double origin_x,
      double origin_y,
      double inflation_radius_m,
      int8_t free_value = 0,
      int8_t occupied_value = 100,
      int8_t unknown_value = -1);

    void update_from_scan(const sensor_msgs::msg::LaserScan &scan);

    nav_msgs::msg::OccupancyGrid to_message(
      const std::string &frame_id,
      const builtin_interfaces::msg::Time &stamp) const;

    bool world_to_grid(double x, double y, int &gx, int &gy) const;
    bool in_bounds(int gx, int gy) const;
    int8_t value_at(int gx, int gy) const;

private:
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;

    int inflation_radius_cells_;
    int8_t free_value_;
    int8_t occupied_value_;
    int8_t unknown_value_;

    std::vector<int8_t> data_;

    int index(int gx, int gy) const;
    void reset_to_unknown();
    void set_free_if_unknown(int gx, int gy);
    void set_occupied(int gx, int gy);
    void raytrace_free(int x0, int y0, int x1, int y1);
    void inflate_obstacles();
};
