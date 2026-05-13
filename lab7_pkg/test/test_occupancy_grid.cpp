#include <gtest/gtest.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rrt/occupancy_grid.h"

namespace {

sensor_msgs::msg::LaserScan make_scan(
  float angle_min,
  float angle_increment,
  float range_min,
  float range_max,
  const std::vector<float> &ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = angle_min;
  scan.angle_increment = angle_increment;
  scan.range_min = range_min;
  scan.range_max = range_max;
  scan.ranges = ranges;
  return scan;
}

TEST(LocalOccupancyGrid, WorldToGridAndBounds)
{
  LocalOccupancyGrid grid(0.1, 20, 20, -1.0, -1.0, 0.0);

  int gx = -1;
  int gy = -1;
  EXPECT_TRUE(grid.world_to_grid(0.0, 0.0, gx, gy));
  EXPECT_EQ(gx, 10);
  EXPECT_EQ(gy, 10);

  EXPECT_FALSE(grid.world_to_grid(2.5, 0.0, gx, gy));
}

TEST(LocalOccupancyGrid, UpdateFromScanMarksFreeAndOccupied)
{
  LocalOccupancyGrid grid(0.1, 40, 40, -1.0, -2.0, 0.0);
  auto scan = make_scan(0.0f, 1.0f, 0.0f, 5.0f, {1.0f});

  grid.update_from_scan(scan);

  int occupied_x = 0;
  int occupied_y = 0;
  ASSERT_TRUE(grid.world_to_grid(1.0, 0.0, occupied_x, occupied_y));
  EXPECT_EQ(grid.value_at(occupied_x, occupied_y), 100);

  int free_x = 0;
  int free_y = 0;
  ASSERT_TRUE(grid.world_to_grid(0.5, 0.0, free_x, free_y));
  EXPECT_EQ(grid.value_at(free_x, free_y), 0);

  int unknown_x = 0;
  int unknown_y = 0;
  ASSERT_TRUE(grid.world_to_grid(-0.9, 1.8, unknown_x, unknown_y));
  EXPECT_EQ(grid.value_at(unknown_x, unknown_y), -1);
}

TEST(LocalOccupancyGrid, InflationMarksNearbyCellsOccupied)
{
  LocalOccupancyGrid grid(0.1, 50, 50, -2.0, -2.0, 0.2);
  auto scan = make_scan(0.0f, 1.0f, 0.0f, 5.0f, {1.0f});

  grid.update_from_scan(scan);

  int hit_x = 0;
  int hit_y = 0;
  ASSERT_TRUE(grid.world_to_grid(1.0, 0.0, hit_x, hit_y));
  EXPECT_EQ(grid.value_at(hit_x, hit_y), 100);

  // One cell to the side is within inflation radius (2 cells at 0.1 m resolution).
  EXPECT_EQ(grid.value_at(hit_x, hit_y + 1), 100);
}

}  // namespace
