#include "collision_restraint/visualization.hpp"

#include <cmath>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "collision_restraint/params.hpp"

namespace collision_restraint
{
const std_msgs::msg::ColorRGBA k_trajectory_color = []() {
  std_msgs::msg::ColorRGBA color;
  color.r = 0.0F;
  color.g = 0.0F;
  color.b = 1.0F;
  color.a = 1.0F;
  return color;
}();

Visualization::Visualization(const std::string & frame, Footprint footprint)
: frame_{frame}, footprint_{footprint}
{
}

visualization_msgs::msg::Marker Visualization::trajectoryMarker(
  const float linear_velocity, const float angular_velocity, const float stopping_distance) const
{
  if (std::abs(angular_velocity) <= g_straight_threshold) {
    return straightLineMarker(stopping_distance);
  }

  visualization_msgs::msg::Marker marker = baseTrajectoryMarker();
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.z = 0.01;
  marker.color = k_trajectory_color;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.z = 0.0;

  const double radius = std::abs(linear_velocity / angular_velocity);
  marker.scale.x = 2.0 * radius;
  marker.scale.y = 2.0 * radius;
  marker.pose.position.y = std::copysign(radius, angular_velocity);

  return marker;
}

visualization_msgs::msg::Marker Visualization::straightLineMarker(
  const float stopping_distance) const
{
  visualization_msgs::msg::Marker marker = baseTrajectoryMarker();
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.03;
  marker.color = k_trajectory_color;

  const auto make_point = [](const float x, const float y) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point point;
    point.z = 0.0;

    point.x = static_cast<double>(x);
    point.y = static_cast<double>(y);
    return point;
  };

  // left
  marker.points.push_back(make_point(0.0f, footprint_.halfWidth()));
  marker.points.push_back(make_point(stopping_distance, footprint_.halfWidth()));

  // right
  marker.points.push_back(make_point(0.0f, -footprint_.halfWidth()));
  marker.points.push_back(make_point(stopping_distance, -footprint_.halfWidth()));

  return marker;
}

visualization_msgs::msg::Marker Visualization::baseTrajectoryMarker() const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_;

  marker.ns = "trajectory";
  marker.id = 0;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  return marker;
}

visualization_msgs::msg::Marker Visualization::pointMarker(const float x, const float y) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_;

  marker.ns = "closest_point";
  marker.id = 0;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  const double size = 0.05;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;

  marker.color.r = 1.0F;
  marker.color.g = 1.0F;
  marker.color.b = 1.0F;
  marker.color.a = 1.0F;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = static_cast<double>(x);
  marker.pose.position.y = static_cast<double>(y);

  return marker;
}

}  // namespace collision_restraint
