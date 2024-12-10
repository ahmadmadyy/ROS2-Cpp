#include <functional>
#include <math.h>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/detail/float64__struct.hpp"
#include "std_msgs/msg/float64.hpp"

class MyActionServer : public rclcpp::Node
{
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("my_action_server", options)
  {
    using namespace std::placeholders;

    subscriber_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options_subs;
    options_subs.callback_group = subscriber_group_;

    this->action_server_ = rclcpp_action::create_server<Distance>(this, "distance_as",
            std::bind(&MyActionServer::handle_goal, this, _1, _2),
            std::bind(&MyActionServer::handle_cancel, this, _1),
            std::bind(&MyActionServer::handle_accepted, this, _1), rcl_action_server_get_default_options(), action_group_);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MyActionServer::topic_callback, this, _1), options_subs);

    //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    publisher2_ = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);

    distance_ = 0;
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr subscriber_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  double distance_;
  nav_msgs::msg::Odometry prev_pose_;
/*
  float dist_traveled()
  {
    return sqrt(pow(this->current_pos_x - this->start_pos_x,2) + pow(this->current_pos_y - this->start_pos_y,2));
  }
*/
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double dx = msg->pose.pose.position.x - prev_pose_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.pose.pose.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.pose.pose.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);
    distance_ += distance;
    prev_pose_.pose = msg->pose;
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Distance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &tracking_distance_FB = feedback->current_dist;
    tracking_distance_FB = distance_;
    auto result = std::make_shared<Distance::Result>();
    auto distance_msg = std_msgs::msg::Float64();
    distance_msg.data = tracking_distance_FB;
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;;
        result->total_dist = tracking_distance_FB;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Send feedback
      tracking_distance_FB = distance_;
      distance_msg.data = tracking_distance_FB;
      publisher2_->publish(distance_msg);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      RCLCPP_INFO(this->get_logger(), "current_dist:  %f", distance_msg.data);
      loop_rate.sleep();

      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = tracking_distance_FB;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "status: %s", result->status ? "true" : "false");
      RCLCPP_INFO(this->get_logger(), "total_dist: %f", tracking_distance_FB);
    }
  }
};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}