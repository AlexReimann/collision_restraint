#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace collision_restraint
{

class CollisionRestraintNode : public rclcpp::Node
{
public:
  CollisionRestraintNode();

private:
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  void twistCallback(geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void twistStampedCallback(geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_stamped_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;

  sensor_msgs::msg::PointCloud2 latest_point_cloud_;
};

}  // namespace collision_restraint
