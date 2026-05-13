#include "rrt/occupancy_grid.h"

#include <algorithm>

LocalOccupancyGrid::LocalOccupancyGrid(
  double resolution,
  int width_cells,
  int height_cells,
  double origin_x,
  double origin_y,
  double inflation_radius_m,
  int8_t free_value,
  int8_t occupied_value,
  int8_t unknown_value)
: width_(width_cells),
  height_(height_cells),
  resolution_(resolution),
  origin_x_(origin_x),
  origin_y_(origin_y),
  inflation_radius_cells_(std::max(0, static_cast<int>(std::round(inflation_radius_m / resolution)))),
  free_value_(free_value),
  occupied_value_(occupied_value),
  unknown_value_(unknown_value),
  data_(static_cast<size_t>(width_cells * height_cells), unknown_value)
{
}

void LocalOccupancyGrid::update_from_scan(const sensor_msgs::msg::LaserScan &scan)
{
    reset_to_unknown();

    int sensor_gx = 0;
    int sensor_gy = 0;
    if (!world_to_grid(0.0, 0.0, sensor_gx, sensor_gy)) {
        return;
    }

    const float angle_min = scan.angle_min;
    const float angle_increment = scan.angle_increment;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        const float angle = angle_min + static_cast<float>(i) * angle_increment;

        if (!std::isfinite(range)) {
            continue;
        }
        if (range < scan.range_min) {
            continue;
        }

        const bool has_hit = range <= scan.range_max;
        const float ray_range = has_hit ? range : scan.range_max;

        const double end_x = static_cast<double>(ray_range) * std::cos(static_cast<double>(angle));
        const double end_y = static_cast<double>(ray_range) * std::sin(static_cast<double>(angle));

        int end_gx = 0;
        int end_gy = 0;
        if (!world_to_grid(end_x, end_y, end_gx, end_gy)) {
            continue;
        }

        raytrace_free(sensor_gx, sensor_gy, end_gx, end_gy);
        if (has_hit) {
            set_occupied(end_gx, end_gy);
        }
    }

    inflate_obstacles();
}

nav_msgs::msg::OccupancyGrid LocalOccupancyGrid::to_message(
  const std::string &frame_id,
  const builtin_interfaces::msg::Time &stamp) const
{
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;

    msg.info.resolution = static_cast<float>(resolution_);
    msg.info.width = static_cast<uint32_t>(width_);
    msg.info.height = static_cast<uint32_t>(height_);
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.data = data_;
    return msg;
}

bool LocalOccupancyGrid::world_to_grid(double x, double y, int &gx, int &gy) const
{
    gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    return in_bounds(gx, gy);
}

bool LocalOccupancyGrid::in_bounds(int gx, int gy) const
{
    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
}

int8_t LocalOccupancyGrid::value_at(int gx, int gy) const
{
    if (!in_bounds(gx, gy)) {
        return occupied_value_;
    }
    return data_[static_cast<size_t>(index(gx, gy))];
}

int LocalOccupancyGrid::index(int gx, int gy) const
{
    return gy * width_ + gx;
}

void LocalOccupancyGrid::reset_to_unknown()
{
    std::fill(data_.begin(), data_.end(), unknown_value_);
}

void LocalOccupancyGrid::set_free_if_unknown(int gx, int gy)
{
    if (!in_bounds(gx, gy)) {
        return;
    }
    int8_t &cell = data_[static_cast<size_t>(index(gx, gy))];
    if (cell == unknown_value_) {
        cell = free_value_;
    }
}

void LocalOccupancyGrid::set_occupied(int gx, int gy)
{
    if (!in_bounds(gx, gy)) {
        return;
    }
    data_[static_cast<size_t>(index(gx, gy))] = occupied_value_;
}

void LocalOccupancyGrid::raytrace_free(int x0, int y0, int x1, int y1)
{
    int x = x0;
    int y = y0;

    const int dx = std::abs(x1 - x0);
    const int dy = std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        set_free_if_unknown(x, y);
        if (x == x1 && y == y1) {
            break;
        }

        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }

        if (!in_bounds(x, y)) {
            break;
        }
    }
}

void LocalOccupancyGrid::inflate_obstacles()
{
    if (inflation_radius_cells_ <= 0) {
        return;
    }

    std::vector<int8_t> inflated = data_;
    const int radius_sq = inflation_radius_cells_ * inflation_radius_cells_;

    for (int gy = 0; gy < height_; ++gy) {
        for (int gx = 0; gx < width_; ++gx) {
            if (data_[static_cast<size_t>(index(gx, gy))] != occupied_value_) {
                continue;
            }

            for (int dy = -inflation_radius_cells_; dy <= inflation_radius_cells_; ++dy) {
                for (int dx = -inflation_radius_cells_; dx <= inflation_radius_cells_; ++dx) {
                    if (dx * dx + dy * dy > radius_sq) {
                        continue;
                    }

                    const int nx = gx + dx;
                    const int ny = gy + dy;
                    if (!in_bounds(nx, ny)) {
                        continue;
                    }

                    inflated[static_cast<size_t>(index(nx, ny))] = occupied_value_;
                }
            }
        }
    }

    data_.swap(inflated);
}
