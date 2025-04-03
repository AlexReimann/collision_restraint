#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "collision_restraint/collision_restraint.hpp"
#include "collision_restraint/params.hpp"
#include "collision_restraint/visualization.hpp"

namespace collision_restraint
{

class CollisionRestraintNode : public rclcpp::Node
{
public:
  CollisionRestraintNode();

private:
  void parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  void twistCallback(geometry_msgs::msg::Twist::SharedPtr twist_msg);
  void twistStampedCallback(geometry_msgs::msg::TwistStamped::SharedPtr twist_msg);

  std::shared_ptr<Params> params_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;

  std::string base_link_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<CollisionRestraint> collision_restraint_;
  std::shared_ptr<Visualization> visualization_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_velocity_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_stamped_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_trajectory_visual_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_point_visual_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_stamped_;

  sensor_msgs::msg::PointCloud2 latest_point_cloud_;
};

}  // namespace collision_restraint
