#include "my_components/readodom_component.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

namespace my_components
{
OdomSubscriber::OdomSubscriber(const rclcpp::NodeOptions & options) : Node("readodom", options)
{
  auto callback =
    [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
    {
      // Print X position of the robot
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->pose.pose.position.x);
      std::flush(std::cout);
    };
  
  sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, callback);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::OdomSubscriber)