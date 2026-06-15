#ifndef INCREASE_CLEARANCE__INCREASE_CLEARANCE_HPP_
#define INCREASE_CLEARANCE__INCREASE_CLEARANCE_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "increase_clearance/action/increase_clearance.hpp"
#include "increase_clearance/repulsive_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace increase_clearance
{
using namespace nav2_behaviors;  // NOLINT
using Action = increase_clearance::action::IncreaseClearance;

class IncreaseClearance : public TimedBehavior<Action>
{
public:
  IncreaseClearance();
  ~IncreaseClearance();

  ResultStatus onRun(const std::shared_ptr<const typename Action::Goal> command) override;
  ResultStatus onCycleUpdate() override;
  void onConfigure() override;
  void onCleanup() override;
  void onActionCompletion(std::shared_ptr<typename Action::Result> result) override;

  nav2_core::CostmapInfoType getResourceInfo() override
  {
    return nav2_core::CostmapInfoType::LOCAL;
  }

protected:
  enum class MotionModel
  {
    OMNI,
    DIFF,
    ACKERMANN,
    UNKNOWN
  };

  MotionModel parseMotionModel(const std::string & model) const;

  // Maps repulsive force to cmd_vel based on motion_model_ param.
  // TODO(motion-model): diff/ackermann stubs fall back to omni for now.
  geometry_msgs::msg::Twist mapForceToVelocity(double fx, double fy) const;
  geometry_msgs::msg::Twist mapForceOmni(double fx, double fy) const;
  geometry_msgs::msg::Twist mapForceDiff(double fx, double fy) const;
  geometry_msgs::msg::Twist mapForceAckermann(double fx, double fy) const;

  geometry_msgs::msg::Twist applyAccelerationLimit(
    const geometry_msgs::msg::Twist & target) const;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool getLatestScan(sensor_msgs::msg::LaserScan & scan) const;
  void publishDebugMarkers(
    const std::vector<geometry_msgs::msg::Point> & footprint,
    const RepulsiveForceResult & force_result,
    const geometry_msgs::msg::Twist & cmd_vel) const;

  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  typename Action::Feedback::SharedPtr feedback_;

  mutable std::mutex scan_mutex_;
  sensor_msgs::msg::LaserScan latest_scan_;
  bool scan_received_{false};

  MotionModel motion_model_{MotionModel::OMNI};
  std::string scan_topic_;
  std::string footprint_topic_;
  std::string marker_topic_;
  double influence_radius_{0.5};
  double safe_clearance_threshold_{0.35};
  double max_linear_vel_{0.25};
  double linear_accel_limit_{0.5};
  double force_gain_{0.1};
  double min_force_distance_{0.05};
  double time_allowance_{30.0};
  bool publish_debug_markers_{true};
  bool publish_individual_forces_{false};

  double min_clearance_{std::numeric_limits<double>::infinity()};
  double prev_cmd_vel_x_{0.0};
  double prev_cmd_vel_y_{0.0};
  rclcpp::Time end_time_{0, 0, RCL_ROS_TIME};
  mutable bool diff_warned_{false};
  mutable bool ackermann_warned_{false};
};

}  // namespace increase_clearance

#endif  // INCREASE_CLEARANCE__INCREASE_CLEARANCE_HPP_
