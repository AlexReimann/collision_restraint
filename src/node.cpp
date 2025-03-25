#include "collision_restraint/node.hpp"

#include <format>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdexcept>

#include "collision_restraint/collision_restraint.hpp"
#include "collision_restraint/footprint.hpp"
#include "collision_restraint/params.hpp"
#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

CollisionRestraintNode::CollisionRestraintNode() : Node("collision_restraint")
{
  pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("output", 1);
  pub_velocity_stamped_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("output_stamped", 1);

  sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "sub_point_cloud", rclcpp::SystemDefaultsQoS(),
    std::bind(&CollisionRestraintNode::pointCloudCallback, this, std::placeholders::_1));
  sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "sub_cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&CollisionRestraintNode::twistCallback, this, std::placeholders::_1));
  sub_twist_stamped_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "sub_cmd_vel_stamped", rclcpp::SystemDefaultsQoS(),
    std::bind(&CollisionRestraintNode::twistStampedCallback, this, std::placeholders::_1));
}

void CollisionRestraintNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  if (cloud_msg->header.frame_id != "base_link") {  // TODO(me): make this a param
    std::domain_error(
      source_prefix() +
      std::format(
        "Only point clouds in base_link frame are supported, got: {}", cloud_msg->header.frame_id));
  }
  latest_point_cloud_ = *cloud_msg;
}

void CollisionRestraintNode::twistCallback(geometry_msgs::msg::Twist::SharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped::SharedPtr stamped =
    std::make_shared<geometry_msgs::msg::TwistStamped>();
  stamped->header.stamp = rclcpp::Clock().now();

  // TODO(me): magic

  stamped->twist = *twist_msg;
  twistStampedCallback(stamped);
}

void CollisionRestraintNode::twistStampedCallback(
  geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped output;
  output.header = twist_msg->header;

  pub_velocity_stamped_->publish(output);
  pub_velocity_->publish(output.twist);
}

}  // namespace collision_restraint

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<collision_restraint::CollisionRestraintNode>());
  rclcpp::shutdown();
  return 0;
}
