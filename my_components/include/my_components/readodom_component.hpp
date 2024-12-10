#ifndef COMPOSITION__READODOM_COMPONENT_HPP_
#define COMPOSITION__READODOM_COMPONENT_HPP_

#include "my_components/visibility_control.h"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace my_components
{

    class OdomSubscriber : public rclcpp::Node
        {
        public:
            COMPOSITION_PUBLIC
            explicit OdomSubscriber(const rclcpp::NodeOptions & options);

        protected:
            void on_timer();

        private:
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    };

}  // namespace composition

#endif  // COMPOSITION__MOVEROBOT_COMPONENT_HPP_