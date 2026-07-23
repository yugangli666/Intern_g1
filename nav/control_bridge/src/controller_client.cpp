#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "musen/musen.hpp"
#include "common.hpp"

class ControllerClient : public rclcpp::Node {
public:
  ControllerClient() : Node("controller_client"), broadcaster_(PORT), task_result_broadcaster_(5001), task_listener_(5002) {
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      CMD_VEL_NAV, 10, std::bind(&ControllerClient::twist_callback, this, std::placeholders::_1));
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ODOM_GLOBAL, 10, std::bind(&ControllerClient::odom_callback, this, std::placeholders::_1));
    sub_task_result_ = this->create_subscription<std_msgs::msg::Bool>("spirit_nav_task_result", 10, std::bind(&ControllerClient::result_callback, this, std::placeholders::_1));
    pub_task_ = this->create_publisher<std_msgs::msg::String>("spirit_nav_task", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerClient::task_callback, this));
    stop_flag_.store(false);
    seq_ = 0;
    task_seq_ = 0;
    recv_task_flag_ = false;
  }

private:
  void result_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    TaskData task_result;
    if (msg->data) {      
      task_result.result = true;      
    } else {
      task_result.result = false;
    }
    task_result_broadcaster_.send(task_result);
    RCLCPP_INFO(this->get_logger(), "Task result: %s", task_result.result ? "success" : "failed");
  }
  void task_callback() {
    auto msg = task_listener_.receive<TaskData>();
    if (msg.has_value()) {
      if (!recv_task_flag_) {
        task_seq_ = msg->seq;
        recv_task_flag_ = true;
      } else if (msg->seq == task_seq_) {
        return;
      }
      task_seq_ = msg->seq;
      std::string task_name = msg->task_name;
      std_msgs::msg::String task_msg;
      task_msg.data = task_name;
      pub_task_->publish(task_msg);
      
      RCLCPP_INFO(this->get_logger(), "Task name: %s, seq: %ld", task_name.c_str(), task_seq_);
    }
  }
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    CmdData cmd_vel_msg;
    cmd_vel_msg.seq = seq_++;
    cmd_vel_msg.x = msg->linear.y;
    cmd_vel_msg.y = -msg->linear.x;
    cmd_vel_msg.z = msg->angular.z;
    const rclcpp::Time current_time = this->now();
    const rclcpp::Time previous_time = prev_odom_time_;
    const rclcpp::Duration dt = current_time - previous_time;
    if (dt.seconds() > 0.5) {
      stop_flag_.store(true);
      // RCLCPP_ERROR(this->get_logger(), "Emergency stop!");
      // return;
    }
    broadcaster_.send(cmd_vel_msg);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    prev_odom_time_ = msg->header.stamp;
    stop_flag_.store(false);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_task_result_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_task_;
  rclcpp::Time prev_odom_time_;
  std::atomic<bool> stop_flag_;
  musen::Broadcaster broadcaster_;
  musen::Broadcaster task_result_broadcaster_;
  musen::Listener task_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t seq_;
  uint64_t task_seq_;
  bool recv_task_flag_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerClient>());
  rclcpp::shutdown();

  return 0;
}