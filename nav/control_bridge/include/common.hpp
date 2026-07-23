#pragma once

constexpr int PORT = 5000;

const std::string MX_BASE_VEL_COMMAND = "mx_base_vel_command";
const std::string CMD_VEL_NAV = "cmd_vel";  // 订阅平滑后的指令（collision_monitor输出）
const std::string ODOM_GLOBAL = "/moz1/odom_global";
const std::string EMERGENCY_STOP_TOPIC = "emergency_stop";
const std::string STOP_NAV_TASK_TOPIC = "/robot_nav_stop_task";

struct CmdData {
    uint64_t seq;
    double x;
    double y;
    double z;
  };

struct TaskData {
  char task_name[50];
  bool result;
  uint64_t seq;
};

struct EmergencyStopData {
  bool emergency_stop;
  uint64_t seq;
};