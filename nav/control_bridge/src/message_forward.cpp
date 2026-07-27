#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/bool.hpp>
#include "common.hpp"

class MessageForward : public rclcpp::Node {
public:
  MessageForward() : Node("message_forward") {
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      CMD_VEL_NAV, 10, std::bind(&MessageForward::twist_callback, this, std::placeholders::_1));
    sub_emergency_stop_ = this->create_subscription<std_msgs::msg::Bool>(
      EMERGENCY_STOP_TOPIC, 10, std::bind(&MessageForward::emergency_stop_callback, this, std::placeholders::_1));
    sub_stop_nav_task_ = this->create_subscription<std_msgs::msg::Bool>(
      STOP_NAV_TASK_TOPIC, 10, std::bind(&MessageForward::stop_nav_task_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(MX_BASE_VEL_COMMAND, 10);
    recv_emergency_stop_ = false;
    previous_time_ = this->now();
  }

  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const rclcpp::Time current_time = this->now();
    const rclcpp::Duration dt = current_time - previous_time_;
    geometry_msgs::msg::Vector3 cmd_vel_msg;
    if (recv_emergency_stop_ && dt.seconds() < 2.0) {
      cmd_vel_msg.x = 0;  
      cmd_vel_msg.y = 0;
      cmd_vel_msg.z = 0;
    } else {
      cmd_vel_msg.x = msg->linear.y;
      cmd_vel_msg.y = -msg->linear.x;
      cmd_vel_msg.z = msg->angular.z;
    }
    publisher_->publish(cmd_vel_msg);
  }

  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    (void)msg;
    recv_emergency_stop_ = true;
    previous_time_ = this->now();
  }

  void stop_nav_task_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    (void)msg;
    recv_emergency_stop_ = true;
    previous_time_ = this->now();
  }

private:
  bool recv_emergency_stop_;
  rclcpp::Time previous_time_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_emergency_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_nav_task_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MessageForward>());
  rclcpp::shutdown();
  return 0;
}