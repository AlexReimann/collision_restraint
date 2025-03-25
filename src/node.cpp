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
  params_ = std::make_shared<Params>();
  parameter_callback_ = this->add_on_set_parameters_callback(
    std::bind(&CollisionRestraintNode::parametersCallback, this, std::placeholders::_1));

  this->declare_parameter("deceleration_", 1.0);
  this->declare_parameter("execution_delay_", 0.0);

  this->declare_parameter("min_obstacle_height_", -1.0);
  this->declare_parameter("max_obstacle_height_", 1.0);

  this->declare_parameter("ignore_inside_footprint_", false);
  this->declare_parameter("distance_buffer_", 0.0);

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

rcl_interfaces::msg::SetParametersResult CollisionRestraintNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto & parameter : parameters) {
    // TODO(me): Fix ugly prone to failure if-else
    if (parameter.get_name() == "deceleration") {
      params_->deceleration_ = static_cast<float>(parameter.as_double());
    } else if (parameter.get_name() == "execution_delay") {
      params_->execution_delay_ = static_cast<float>(parameter.as_double());

    } else if (parameter.get_name() == "min_obstacle_height") {
      params_->min_obstacle_height_ = static_cast<float>(parameter.as_double());
    } else if (parameter.get_name() == "max_obstacle_height") {
      params_->max_obstacle_height_ = static_cast<float>(parameter.as_double());

    } else if (parameter.get_name() == "ignore_inside_footprint_") {
      params_->ignore_inside_footprint_ = parameter.as_bool();
    } else if (parameter.get_name() == "distance_buffer") {
      params_->distance_buffer_ = static_cast<float>(parameter.as_double());
    }
  }
  return result;
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
