#include "increase_clearance/repulsive_geometry.hpp"

#include <algorithm>
#include <cmath>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace increase_clearance
{

SegmentClosestResult closestPointOnSegment(
  double px, double py,
  double x0, double y0,
  double x1, double y1)
{
  SegmentClosestResult result;
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double len_sq = dx * dx + dy * dy;

  if (len_sq < 1e-12) {
    result.closest.x = x0;
    result.closest.y = y0;
    result.distance = std::hypot(px - x0, py - y0);
    return result;
  }

  double t = ((px - x0) * dx + (py - y0) * dy) / len_sq;
  t = std::clamp(t, 0.0, 1.0);

  result.closest.x = x0 + t * dx;
  result.closest.y = y0 + t * dy;
  result.distance = std::hypot(px - result.closest.x, py - result.closest.y);
  return result;
}

SegmentClosestResult closestPointOnPolygon(
  double px, double py,
  const std::vector<geometry_msgs::msg::Point> & footprint)
{
  SegmentClosestResult best;
  if (footprint.size() < 2) {
    return best;
  }

  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & v0 = footprint[i];
    const auto & v1 = footprint[(i + 1) % footprint.size()];
    auto segment = closestPointOnSegment(px, py, v0.x, v0.y, v1.x, v1.y);
    if (segment.distance < best.distance) {
      best = segment;
    }
  }
  return best;
}

std::vector<Point2D> laserScanToPoints(
  const sensor_msgs::msg::LaserScan & scan,
  tf2_ros::Buffer & tf,
  const std::string & target_frame,
  double transform_tolerance)
{
  std::vector<Point2D> points;
  if (scan.ranges.empty()) {
    return points;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf.lookupTransform(
      target_frame, scan.header.frame_id, scan.header.stamp,
      rclcpp::Duration::from_seconds(transform_tolerance));
  } catch (const tf2::TransformException & ex) {
    return points;
  }

  points.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) ||
      range < scan.range_min || range > scan.range_max)
    {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    geometry_msgs::msg::PointStamped point_in;
    point_in.header = scan.header;
    point_in.point.x = range * std::cos(angle);
    point_in.point.y = range * std::sin(angle);
    point_in.point.z = 0.0;

    geometry_msgs::msg::PointStamped point_out;
    tf2::doTransform(point_in, point_out, transform);
    points.push_back({point_out.point.x, point_out.point.y});
  }

  return points;
}

RepulsiveForceResult computeRepulsiveForce(
  const std::vector<Point2D> & points,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double influence_radius,
  double min_force_distance)
{
  RepulsiveForceResult result;
  if (footprint.size() < 2 || points.empty()) {
    return result;
  }

  const double min_dist = std::max(min_force_distance, 1e-6);

  for (const auto & point : points) {
    const auto closest = closestPointOnPolygon(point.x, point.y, footprint);
    const double d = closest.distance;

    if (d < result.min_clearance) {
      result.min_clearance = d;
      result.min_clearance_scan_point = point;
      result.min_clearance_boundary_point = closest.closest;
    }

    if (d > influence_radius) {
      continue;
    }

    result.influence_points.push_back(point);

    const double dx = closest.closest.x - point.x;
    const double dy = closest.closest.y - point.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < 1e-12) {
      continue;
    }

    const double effective_d = std::max(d, min_dist);
    const double weight = 1.0 / (effective_d * effective_d);
    const double inv_dist = 1.0 / std::sqrt(dist_sq);
    const double force_x = dx * inv_dist * weight;
    const double force_y = dy * inv_dist * weight;

    result.net_force_x += force_x;
    result.net_force_y += force_y;

    ForceContribution contribution;
    contribution.scan_point = point;
    contribution.boundary_point = closest.closest;
    contribution.force_x = force_x;
    contribution.force_y = force_y;
    result.contributions.push_back(contribution);
  }

  return result;
}

}  // namespace increase_clearance
