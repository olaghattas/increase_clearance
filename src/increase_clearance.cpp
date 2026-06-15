#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "increase_clearance/increase_clearance.hpp"
#include "nav2_util/node_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace increase_clearance
{

namespace
{
visualization_msgs::msg::Marker makeDeleteAllMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}
}  // namespace

IncreaseClearance::IncreaseClearance()
: TimedBehavior<Action>(),
  feedback_(std::make_shared<typename Action::Feedback>())
{
}

IncreaseClearance::~IncreaseClearance() = default;

IncreaseClearance::MotionModel IncreaseClearance::parseMotionModel(
  const std::string & model) const
{
  if (model == "omni") {
    return MotionModel::OMNI;
  }
  if (model == "diff") {
    return MotionModel::DIFF;
  }
  if (model == "ackermann") {
    return MotionModel::ACKERMANN;
  }
  RCLCPP_WARN(
    logger_, "Unknown motion_model '%s', falling back to omni.", model.c_str());
  return MotionModel::OMNI;
}

void IncreaseClearance::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  const auto param = [this](const std::string & name) {
      return behavior_name_ + "." + name;
    };

  nav2_util::declare_parameter_if_not_declared(
    node, param("scan_topic"), rclcpp::ParameterValue("/scan_filtered"));
  nav2_util::declare_parameter_if_not_declared(
    node, param("footprint_topic"), rclcpp::ParameterValue("local_costmap/published_footprint"));
  nav2_util::declare_parameter_if_not_declared(
    node, param("influence_radius"), rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, param("safe_clearance_threshold"), rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, param("max_linear_vel"), rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, param("linear_accel_limit"), rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, param("force_gain"), rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, param("min_force_distance"), rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, param("time_allowance"), rclcpp::ParameterValue(30.0));
  nav2_util::declare_parameter_if_not_declared(
    node, param("publish_debug_markers"), rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, param("publish_individual_forces"), rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, param("marker_topic"), rclcpp::ParameterValue("~/debug_markers"));
  nav2_util::declare_parameter_if_not_declared(
    node, param("motion_model"), rclcpp::ParameterValue("omni"));

  node->get_parameter(param("scan_topic"), scan_topic_);
  node->get_parameter(param("footprint_topic"), footprint_topic_);
  node->get_parameter(param("influence_radius"), influence_radius_);
  node->get_parameter(param("safe_clearance_threshold"), safe_clearance_threshold_);
  node->get_parameter(param("max_linear_vel"), max_linear_vel_);
  node->get_parameter(param("linear_accel_limit"), linear_accel_limit_);
  node->get_parameter(param("force_gain"), force_gain_);
  node->get_parameter(param("min_force_distance"), min_force_distance_);
  node->get_parameter(param("time_allowance"), time_allowance_);
  node->get_parameter(param("publish_debug_markers"), publish_debug_markers_);
  node->get_parameter(param("publish_individual_forces"), publish_individual_forces_);
  node->get_parameter(param("marker_topic"), marker_topic_);

  std::string motion_model_str;
  node->get_parameter(param("motion_model"), motion_model_str);
  motion_model_ = parseMotionModel(motion_model_str);

  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node, footprint_topic_, *tf_, robot_base_frame_, transform_tolerance_);

  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&IncreaseClearance::scanCallback, this, std::placeholders::_1));

  if (publish_debug_markers_) {
    marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, rclcpp::SystemDefaultsQoS());
  }

  RCLCPP_INFO(
    logger_,
    "Configured %s: scan=%s footprint=%s motion_model=%s influence=%.2f safe_clearance=%.2f",
    behavior_name_.c_str(), scan_topic_.c_str(), footprint_topic_.c_str(),
    motion_model_str.c_str(), influence_radius_, safe_clearance_threshold_);
}

void IncreaseClearance::onCleanup()
{
  scan_sub_.reset();
  marker_pub_.reset();
  footprint_sub_.reset();
  scan_received_ = false;
}

void IncreaseClearance::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = *msg;
  scan_received_ = true;
}

bool IncreaseClearance::getLatestScan(sensor_msgs::msg::LaserScan & scan) const
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (!scan_received_) {
    return false;
  }
  scan = latest_scan_;
  return true;
}

nav2_behaviors::ResultStatus IncreaseClearance::onRun(
  const std::shared_ptr<const typename Action::Goal>/*command*/)
{
  min_clearance_ = std::numeric_limits<double>::infinity();
  prev_cmd_vel_x_ = 0.0;
  prev_cmd_vel_y_ = 0.0;
  diff_warned_ = false;
  ackermann_warned_ = false;
  end_time_ = clock_->now() + rclcpp::Duration::from_seconds(time_allowance_);

  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header footprint_header;
  if (!footprint_sub_ || !footprint_sub_->getFootprintInRobotFrame(footprint, footprint_header)) {
    RCLCPP_ERROR(logger_, "Footprint not available on topic %s", footprint_topic_.c_str());
    return ResultStatus{Status::FAILED, Action::Result::NO_FOOTPRINT};
  }

  sensor_msgs::msg::LaserScan scan;
  if (!getLatestScan(scan)) {
    RCLCPP_ERROR(logger_, "LaserScan not available on topic %s", scan_topic_.c_str());
    return ResultStatus{Status::FAILED, Action::Result::NO_SCAN};
  }

  RCLCPP_INFO(logger_, "Starting repulsive escape behavior");
  return ResultStatus{Status::SUCCEEDED, Action::Result::NONE};
}

geometry_msgs::msg::Twist IncreaseClearance::mapForceOmni(double fx, double fy) const
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = fx * force_gain_;
  cmd.linear.y = fy * force_gain_;
  cmd.angular.z = 0.0;

  const double speed = std::hypot(cmd.linear.x, cmd.linear.y);
  if (speed > max_linear_vel_ && speed > 1e-6) {
    const double scale = max_linear_vel_ / speed;
    cmd.linear.x *= scale;
    cmd.linear.y *= scale;
  }
  return cmd;
}

geometry_msgs::msg::Twist IncreaseClearance::mapForceDiff(double fx, double fy) const
{
  // TODO(motion-model): project force onto forward axis + optional in-place rotate.
  if (!diff_warned_) {
    RCLCPP_WARN(
      logger_,
      "motion_model 'diff' is not implemented yet; falling back to omni.");
    diff_warned_ = true;
  }
  return mapForceOmni(fx, fy);
}

geometry_msgs::msg::Twist IncreaseClearance::mapForceAckermann(double fx, double fy) const
{
  // TODO(motion-model): add min turning radius and forward-only constraint.
  if (!ackermann_warned_) {
    RCLCPP_WARN(
      logger_,
      "motion_model 'ackermann' is not implemented yet; falling back to omni.");
    ackermann_warned_ = true;
  }
  return mapForceOmni(fx, fy);
}

geometry_msgs::msg::Twist IncreaseClearance::mapForceToVelocity(double fx, double fy) const
{
  switch (motion_model_) {
    case MotionModel::OMNI:
      return mapForceOmni(fx, fy);
    case MotionModel::DIFF:
      return mapForceDiff(fx, fy);
    case MotionModel::ACKERMANN:
      return mapForceAckermann(fx, fy);
    default:
      return mapForceOmni(fx, fy);
  }
}

geometry_msgs::msg::Twist IncreaseClearance::applyAccelerationLimit(
  const geometry_msgs::msg::Twist & target) const
{
  geometry_msgs::msg::Twist limited = target;
  const double dt = (cycle_frequency_ > 1e-6) ? (1.0 / cycle_frequency_) : 0.1;
  const double max_delta = linear_accel_limit_ * dt;

  const double dx = target.linear.x - prev_cmd_vel_x_;
  const double dy = target.linear.y - prev_cmd_vel_y_;
  limited.linear.x = prev_cmd_vel_x_ + std::clamp(dx, -max_delta, max_delta);
  limited.linear.y = prev_cmd_vel_y_ + std::clamp(dy, -max_delta, max_delta);
  return limited;
}

void IncreaseClearance::publishDebugMarkers(
  const std::vector<geometry_msgs::msg::Point> & footprint,
  const RepulsiveForceResult & force_result,
  const geometry_msgs::msg::Twist & cmd_vel) const
{
  if (!publish_debug_markers_ || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;
  array.markers.push_back(makeDeleteAllMarker());

  const auto stamp = clock_->now();
  const auto frame_id = robot_base_frame_;
  const rclcpp::Duration lifetime = rclcpp::Duration::from_seconds(0.2);

  auto set_header = [&](visualization_msgs::msg::Marker & marker, int32_t id) {
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id;
      marker.ns = "increase_clearance";
      marker.id = id;
      marker.lifetime = lifetime;
    };

  visualization_msgs::msg::Marker footprint_marker;
  set_header(footprint_marker, 0);
  footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  footprint_marker.action = visualization_msgs::msg::Marker::ADD;
  footprint_marker.scale.x = 0.02;
  footprint_marker.color.r = 0.0f;
  footprint_marker.color.g = 1.0f;
  footprint_marker.color.b = 0.0f;
  footprint_marker.color.a = 1.0f;
  for (const auto & vertex : footprint) {
    geometry_msgs::msg::Point p;
    p.x = vertex.x;
    p.y = vertex.y;
    p.z = 0.0;
    footprint_marker.points.push_back(p);
  }
  if (!footprint.empty()) {
    geometry_msgs::msg::Point p;
    p.x = footprint.front().x;
    p.y = footprint.front().y;
    p.z = 0.0;
    footprint_marker.points.push_back(p);
  }
  array.markers.push_back(footprint_marker);

  visualization_msgs::msg::Marker influence_points;
  set_header(influence_points, 1);
  influence_points.type = visualization_msgs::msg::Marker::POINTS;
  influence_points.action = visualization_msgs::msg::Marker::ADD;
  influence_points.scale.x = 0.04;
  influence_points.scale.y = 0.04;
  influence_points.color.r = 1.0f;
  influence_points.color.g = 1.0f;
  influence_points.color.b = 0.0f;
  influence_points.color.a = 1.0f;
  for (const auto & point : force_result.influence_points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    influence_points.points.push_back(p);
  }
  array.markers.push_back(influence_points);

  if (std::isfinite(force_result.min_clearance)) {
    visualization_msgs::msg::Marker min_point;
    set_header(min_point, 2);
    min_point.type = visualization_msgs::msg::Marker::POINTS;
    min_point.action = visualization_msgs::msg::Marker::ADD;
    min_point.scale.x = 0.08;
    min_point.scale.y = 0.08;
    min_point.color.r = 1.0f;
    min_point.color.g = 0.0f;
    min_point.color.b = 0.0f;
    min_point.color.a = 1.0f;
    geometry_msgs::msg::Point p;
    p.x = force_result.min_clearance_scan_point.x;
    p.y = force_result.min_clearance_scan_point.y;
    min_point.points.push_back(p);
    array.markers.push_back(min_point);

    visualization_msgs::msg::Marker boundary_point;
    set_header(boundary_point, 5);
    boundary_point.type = visualization_msgs::msg::Marker::SPHERE;
    boundary_point.action = visualization_msgs::msg::Marker::ADD;
    boundary_point.pose.position.x = force_result.min_clearance_boundary_point.x;
    boundary_point.pose.position.y = force_result.min_clearance_boundary_point.y;
    boundary_point.scale.x = 0.06;
    boundary_point.scale.y = 0.06;
    boundary_point.scale.z = 0.06;
    boundary_point.color.r = 1.0f;
    boundary_point.color.g = 0.5f;
    boundary_point.color.b = 0.0f;
    boundary_point.color.a = 1.0f;
    array.markers.push_back(boundary_point);
  }

  const double force_mag = std::hypot(force_result.net_force_x, force_result.net_force_y);
  if (force_mag > 1e-6) {
    visualization_msgs::msg::Marker force_arrow;
    set_header(force_arrow, 3);
    force_arrow.type = visualization_msgs::msg::Marker::ARROW;
    force_arrow.action = visualization_msgs::msg::Marker::ADD;
    force_arrow.points.resize(2);
    force_arrow.points[0].x = 0.0;
    force_arrow.points[0].y = 0.0;
    force_arrow.points[1].x = force_result.net_force_x * 0.1;
    force_arrow.points[1].y = force_result.net_force_y * 0.1;
    force_arrow.scale.x = 0.04;
    force_arrow.scale.y = 0.08;
    force_arrow.color.r = 0.0f;
    force_arrow.color.g = 0.0f;
    force_arrow.color.b = 1.0f;
    force_arrow.color.a = 1.0f;
    array.markers.push_back(force_arrow);
  }

  const double cmd_mag = std::hypot(cmd_vel.linear.x, cmd_vel.linear.y);
  if (cmd_mag > 1e-6) {
    visualization_msgs::msg::Marker cmd_arrow;
    set_header(cmd_arrow, 4);
    cmd_arrow.type = visualization_msgs::msg::Marker::ARROW;
    cmd_arrow.action = visualization_msgs::msg::Marker::ADD;
    cmd_arrow.points.resize(2);
    cmd_arrow.points[0].x = 0.0;
    cmd_arrow.points[0].y = 0.0;
    cmd_arrow.points[1].x = cmd_vel.linear.x;
    cmd_arrow.points[1].y = cmd_vel.linear.y;
    cmd_arrow.scale.x = 0.03;
    cmd_arrow.scale.y = 0.06;
    cmd_arrow.color.r = 0.0f;
    cmd_arrow.color.g = 1.0f;
    cmd_arrow.color.b = 1.0f;
    cmd_arrow.color.a = 1.0f;
    array.markers.push_back(cmd_arrow);
  }

  visualization_msgs::msg::Marker text_marker;
  set_header(text_marker, 6);
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position.x = 0.0;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 0.3;
  text_marker.scale.z = 0.08;
  text_marker.color.r = 1.0f;
  text_marker.color.g = 1.0f;
  text_marker.color.b = 1.0f;
  text_marker.color.a = 1.0f;
  std::ostringstream label;
  label << "d=" << force_result.min_clearance
        << " |F|=" << force_mag
        << " v=(" << cmd_vel.linear.x << "," << cmd_vel.linear.y << ")";
  text_marker.text = label.str();
  array.markers.push_back(text_marker);

  if (publish_individual_forces_) {
    int32_t id = 7;
    for (const auto & contribution : force_result.contributions) {
      visualization_msgs::msg::Marker contribution_arrow;
      set_header(contribution_arrow, id++);
      contribution_arrow.type = visualization_msgs::msg::Marker::ARROW;
      contribution_arrow.action = visualization_msgs::msg::Marker::ADD;
      contribution_arrow.points.resize(2);
      contribution_arrow.points[0].x = contribution.scan_point.x;
      contribution_arrow.points[0].y = contribution.scan_point.y;
      contribution_arrow.points[1].x = contribution.scan_point.x + contribution.force_x * 0.05;
      contribution_arrow.points[1].y = contribution.scan_point.y + contribution.force_y * 0.05;
      contribution_arrow.scale.x = 0.01;
      contribution_arrow.scale.y = 0.02;
      contribution_arrow.color.r = 0.8f;
      contribution_arrow.color.g = 0.2f;
      contribution_arrow.color.b = 0.8f;
      contribution_arrow.color.a = 0.8f;
      array.markers.push_back(contribution_arrow);
    }
  }

  marker_pub_->publish(array);
}

nav2_behaviors::ResultStatus IncreaseClearance::onCycleUpdate()
{
  if (clock_->now() > end_time_) {
    stopRobot();
    RCLCPP_WARN(logger_, "Repulsive escape timed out");
    return ResultStatus{Status::FAILED, Action::Result::TIMEOUT};
  }

  std::vector<geometry_msgs::msg::Point> footprint;
  std_msgs::msg::Header footprint_header;
  if (!footprint_sub_->getFootprintInRobotFrame(footprint, footprint_header)) {
    RCLCPP_ERROR(logger_, "Footprint not available");
    stopRobot();
    return ResultStatus{Status::FAILED, Action::Result::NO_FOOTPRINT};
  }

  sensor_msgs::msg::LaserScan scan;
  if (!getLatestScan(scan)) {
    RCLCPP_ERROR(logger_, "LaserScan not available");
    stopRobot();
    return ResultStatus{Status::FAILED, Action::Result::NO_SCAN};
  }

  const auto points = laserScanToPoints(
    scan, *tf_, robot_base_frame_, transform_tolerance_);
  if (points.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "No valid laser points after transform to %s", robot_base_frame_.c_str());
    return ResultStatus{Status::RUNNING};
  }

  const auto force_result = computeRepulsiveForce(
    points, footprint, influence_radius_, min_force_distance_);
  min_clearance_ = force_result.min_clearance;

  const auto target_cmd = mapForceToVelocity(
    force_result.net_force_x, force_result.net_force_y);
  const auto limited_cmd = applyAccelerationLimit(target_cmd);
  prev_cmd_vel_x_ = limited_cmd.linear.x;
  prev_cmd_vel_y_ = limited_cmd.linear.y;

  auto cmd_vel = std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel->header.stamp = clock_->now();
  cmd_vel->header.frame_id = robot_base_frame_;
  cmd_vel->twist = limited_cmd;
  vel_pub_->publish(std::move(cmd_vel));

  feedback_->min_clearance = static_cast<float>(min_clearance_);
  feedback_->net_force_x = static_cast<float>(force_result.net_force_x);
  feedback_->net_force_y = static_cast<float>(force_result.net_force_y);
  feedback_->cmd_vel_x = static_cast<float>(limited_cmd.linear.x);
  feedback_->cmd_vel_y = static_cast<float>(limited_cmd.linear.y);
  action_server_->publish_feedback(feedback_);

  publishDebugMarkers(footprint, force_result, limited_cmd);

  if (min_clearance_ > safe_clearance_threshold_) {
    stopRobot();
    prev_cmd_vel_x_ = 0.0;
    prev_cmd_vel_y_ = 0.0;
    RCLCPP_INFO(
      logger_, "Safe clearance reached: min_clearance=%.3f threshold=%.3f",
      min_clearance_, safe_clearance_threshold_);
    return ResultStatus{Status::SUCCEEDED, Action::Result::NONE};
  }

  return ResultStatus{Status::RUNNING};
}

void IncreaseClearance::onActionCompletion(std::shared_ptr<typename Action::Result> result)
{
  result->final_min_clearance = static_cast<float>(min_clearance_);
  RCLCPP_INFO(
    logger_, "Repulsive escape finished with final_min_clearance=%.3f",
    result->final_min_clearance);
}

}  // namespace increase_clearance

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(increase_clearance::IncreaseClearance, nav2_core::Behavior)
