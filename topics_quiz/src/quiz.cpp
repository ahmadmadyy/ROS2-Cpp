#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
auto message = geometry_msgs::msg::Twist();

class AvoidObs : public rclcpp::Node
{
public:
  AvoidObs() : Node("topics_quiz_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&AvoidObs::topic_callback, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&AvoidObs::motion_callback, this));
  }

private:
  void motion_callback()
  {
    publisher_->publish(message);
  }
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {

    auto sensor_front_1 = msg->ranges[360];
    auto sensor_front_2 = msg->ranges[390];
    auto sensor_front_3 = msg->ranges[330];

    auto sensor_left_1 = msg->ranges[0];
    auto sensor_left_2 = msg->ranges[30];

    auto sensor_right_1 = msg->ranges[719];
    auto sensor_right_2 = msg->ranges[690];

    //RCLCPP_INFO(this->get_logger(), "front: '%f'", sensor_front_1);
    //RCLCPP_INFO(this->get_logger(), "right: '%f'", sensor_right_1);
    //RCLCPP_INFO(this->get_logger(), "left: '%f'", sensor_left_1);
    //message.linear.x = 0.1;
    //message.angular.z = 0.1;

    if(sensor_front_1 > 1 || sensor_front_2 > 1 || sensor_front_3 > 1)
    {
      message.linear.x = 0.2;
      message.angular.z = 0;
      //RCLCPP_INFO(this->get_logger(), "front: '%f'", sensor_front);
    }
    if(sensor_front_1 <= 1 || sensor_front_2 <= 1 || sensor_front_3 <= 1)
    {
      message.linear.x = 0;
      message.angular.z = 0.5;
    }
    if(sensor_right_1 <= 1 || sensor_right_2 <= 1)
    {
      message.linear.x = 0;
      message.angular.z = 0.5;
    }
    if(sensor_left_1 <= 1 || sensor_left_2 <= 1)
    {
      message.linear.x = 0;
      message.angular.z = -0.5;
    }
    
    //RCLCPP_INFO(this->get_logger(), "ranges_0: '%f'", msg->ranges[0]);
    //RCLCPP_INFO(this->get_logger(), "ranges_180: '%f'", msg->ranges[180]);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AvoidObs>());
  rclcpp::shutdown();
  return 0;
}