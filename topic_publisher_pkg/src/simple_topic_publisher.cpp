/*
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  auto message = std::make_shared<std_msgs::msg::Int32>();
  message->data = 0;
  rclcpp::WallRate loop_rate(2);

  while (rclcpp::ok()) {
    
    publisher->publish(*message);
    message->data++;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()
  : Node("simple_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}