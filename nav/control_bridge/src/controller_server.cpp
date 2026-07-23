#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "musen/musen.hpp"
#include "common.hpp"

class ControllerServer : public rclcpp::Node {
public:
  ControllerServer() : Node("controller_server"), listener_(PORT) {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(MX_BASE_VEL_COMMAND, 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ControllerServer::forward, this));
    recv_flag_ = false;
  }

private:
  void forward() {
      auto msg = listener_.receive<CmdData>();
      if (msg.has_value()) {
        if (!recv_flag_) {
          seq_ = msg->seq;
          recv_flag_ = true;
        }
        if (msg->seq == seq_) {
          return;
        }
        seq_ = msg->seq;
        geometry_msgs::msg::Vector3 cmd_vel_msg;
        cmd_vel_msg.x = msg->x;
        cmd_vel_msg.y = msg->y;
        cmd_vel_msg.z = msg->z;
        publisher_->publish(cmd_vel_msg);
      }
  }
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  bool recv_flag_;
  uint64_t seq_;
  musen::Listener listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerServer>());
  rclcpp::shutdown();

  return 0;
}