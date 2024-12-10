#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode() : Node("rotation_server")
  {
    srv_ = create_service<Spin>("rotate", std::bind(&ServerNode::rotating_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void rotating_callback(
    const std::shared_ptr<Spin::Request> request, const std::shared_ptr<Spin::Response> response) 
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0;
    
        if(request->direction == "right")
        {
            auto dur = request->time;
            for(int i = 0; i < dur; i++) 
            {
                message.angular.z = - request->angular_velocity;
                publisher_->publish(message);
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Working Right");
            }
        }
        if(request->direction == "left")
        {
            auto dur = request->time;
            for(int i = 0; i < dur; i++) 
            {
                message.angular.z = request->angular_velocity;
                publisher_->publish(message);
                response->success = true;
                RCLCPP_INFO(this->get_logger(), "Working Left");
            }
        }
        else 
        {
            response->success = false;
        }   
        std::this_thread::sleep_for(request->time*1000ms);
        message.angular.z = 0;
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), " Not Working");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}