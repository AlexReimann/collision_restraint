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
#include "collision_restraint/polar_point.hpp"
#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

CollisionRestraintNode::CollisionRestraintNode() : Node("collision_restraint")
{
  params_ = std::make_shared<Params>();
  parameter_callback_ = this->add_post_set_parameters_callback(
    std::bind(&CollisionRestraintNode::parametersCallback, this, std::placeholders::_1));

  this->declare_parameter("base_link_frame", "base_link");
  base_link_frame_ = this->get_parameter("base_link_frame").as_string();

  this->declare_parameter("deceleration", 1.0);
  this->declare_parameter("execution_delay", 0.0);

  this->declare_parameter("min_obstacle_height", -1.0);
  this->declare_parameter("max_obstacle_height", 1.0);

  this->declare_parameter("ignore_inside_footprint", false);
  this->declare_parameter("distance_buffer", 0.0);

  this->declare_parameter("footprint_length_front", 1.0);
  this->declare_parameter("footprint_length_back", 0.0);
  this->declare_parameter("footprint_width", 1.0);

  const float footprint_front =
    static_cast<float>(this->get_parameter("footprint_length_front").as_double());
  const float footprint_back =
    static_cast<float>(this->get_parameter("footprint_length_back").as_double());
  const float footprint_width =
    static_cast<float>(this->get_parameter("footprint_width").as_double());

  collision_restraint_ = std::make_shared<CollisionRestraint>(
    Footprint(footprint_front, footprint_back, footprint_width), params_);
  visualization_ = std::make_shared<Visualization>(
    base_link_frame_, Footprint(footprint_front, footprint_back, footprint_width));

  pub_velocity_ = this->create_publisher<geometry_msgs::msg::Twist>("output", 1);
  pub_velocity_stamped_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("output_stamped", 1);

  pub_trajectory_visual_ =
    this->create_publisher<visualization_msgs::msg::Marker>("visual/trajectory", 1);

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

void CollisionRestraintNode::parametersCallback(const std::vector<rclcpp::Parameter> & parameters)
{
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

    } else if (parameter.get_name() == "ignore_inside_footprint") {
      params_->ignore_inside_footprint_ = parameter.as_bool();
    } else if (parameter.get_name() == "distance_buffer") {
      params_->distance_buffer_ = static_cast<float>(parameter.as_double());
    }
  }
}

void CollisionRestraintNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  if (cloud_msg->header.frame_id != base_link_frame_) {
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
  stamped->twist = *twist_msg;
  twistStampedCallback(stamped);
}

void CollisionRestraintNode::twistStampedCallback(
  geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped output(*twist_msg);
  const float input_linear = static_cast<float>(twist_msg->twist.linear.x);
  const float input_angular = static_cast<float>(twist_msg->twist.angular.z);

  if (input_linear != 0.0 || input_angular != 0.0) {
    const auto [restraint, velocities, closest_point] =
      collision_restraint_->restrain({input_linear, input_angular}, latest_point_cloud_);

    const bool full_brake = velocities.linear_ == 0.0F && velocities.angular_ == 0.0F;

    if (full_brake) {
      RCLCPP_INFO_STREAM_THROTTLE(
        this->get_logger(), *(this->get_clock()), 3000,
        std::format(
          "Full stop because of point at ({}, {})", closest_point.x(), closest_point.y()));
    }

    output.twist.linear.x = velocities.linear_;
    output.twist.angular.z = velocities.angular_;
  }

  pub_velocity_stamped_->publish(output);
  pub_velocity_->publish(output.twist);

  pub_trajectory_visual_->publish(visualization_->trajectoryMarker(input_linear, input_angular));
}

}  // namespace collision_restraint

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<collision_restraint::CollisionRestraintNode>());
  rclcpp::shutdown();
  return 0;
}
