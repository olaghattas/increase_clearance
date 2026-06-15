#ifndef INCREASE_CLEARANCE__REPULSIVE_GEOMETRY_HPP_
#define INCREASE_CLEARANCE__REPULSIVE_GEOMETRY_HPP_

#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace increase_clearance
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct SegmentClosestResult
{
  double distance{std::numeric_limits<double>::infinity()};
  Point2D closest;
};

struct ForceContribution
{
  Point2D scan_point;
  Point2D boundary_point;
  double force_x{0.0};
  double force_y{0.0};
};

struct RepulsiveForceResult
{
  double net_force_x{0.0};
  double net_force_y{0.0};
  double min_clearance{std::numeric_limits<double>::infinity()};
  Point2D min_clearance_scan_point;
  Point2D min_clearance_boundary_point;
  std::vector<Point2D> influence_points;
  std::vector<ForceContribution> contributions;
};

SegmentClosestResult closestPointOnSegment(
  double px, double py,
  double x0, double y0,
  double x1, double y1);

SegmentClosestResult closestPointOnPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Point> & footprint);

std::vector<Point2D> laserScanToPoints(
  const sensor_msgs::msg::LaserScan & scan,
  tf2_ros::Buffer & tf,
  const std::string & target_frame,
  double transform_tolerance);

RepulsiveForceResult computeRepulsiveForce(
  const std::vector<Point2D> & points,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double influence_radius,
  double min_force_distance);

}  // namespace increase_clearance

#endif  // INCREASE_CLEARANCE__REPULSIVE_GEOMETRY_HPP_
