#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber() : Node("odom_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "x_pos: '%f'", msg->pose.pose.position.x);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}