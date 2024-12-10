#include "rclcpp/init_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/age.hpp"
#include <chrono>

using namespace std::chrono_literals;

class AgeRobot : public rclcpp::Node
{
  public:
    AgeRobot() : Node("age_robot")
    {
      publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&AgeRobot::callback, this));
    }
  private:
    void callback(){
      auto message = custom_interfaces::msg::Age();
      message.years = 4;
      message.months = 11;
      message.days = 21;
      publisher_->publish(message);
    }
    rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv []){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AgeRobot>());
  rclcpp::shutdown();
  return 0;
}