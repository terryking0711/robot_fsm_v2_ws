#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robot_interfaces/action/navigate_to_named_pose.hpp"
#include "robot_interfaces/msg/mechanism_command.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"
#include "robot_interfaces/msg/stage_result.hpp"

// 場地座標（world frame，場地最左下角為 (0,0)），yaw 為 rad。
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// ctx 讓整個 FSM 系統共享全局資訊，例如State、Position、Sensor Data等，讓不同的FSM Stage都能存取和修改這些資訊。
struct RobotContext
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;
  using NavigateClient = rclcpp_action::Client<NavigateAction>;

  rclcpp::Node::SharedPtr node;

  // ---- 導航總開關 ----
  // 由 main.cpp 從 ROS parameter "enable_navigation" 載入（預設 true）。
  //   true  : 正常啟用導航與定位（需要 navigation_server / Nav2 /
  //           localization_manager 都有跑）
  //   false : 所有導航步驟直接視為已抵達、定位直接視為成功，
  //           FSM 只跑機構流程。用於單獨測試機構。
  // 只在啟動時讀一次，執行中不會變動。
  bool enable_navigation = true;

  // Navigation
  NavigateClient::SharedPtr nav_client;

  // Localization（對 tdk_slam_ws 的 localization_manager）
  //   /init_pose_cmd    : 發出初始化目標（world frame）
  //   /init_pose_status : localization_manager 回報結果
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr init_cmd_pub;
  bool init_status_received = false;  // 本輪是否已收到回報
  bool init_status_ok = false;        // 回報結果

  // 場外重置請求（/reset_cmd, std_msgs/UInt8, data = 1~4 對應第幾關的重置點）
  std::optional<uint8_t> pending_reset_stage;

  // 場地上的定位點（world frame）：
  //   "start"        比賽出發點（= map 原點）
  //   "reset_stage1" ~ "reset_stage4" 各關重置點
  // 由 main.cpp 從 ROS parameters 載入，可用 launch/yaml 覆蓋不需重編。
  std::map<std::string, Pose2D> field_poses;

  // Mechanism
  rclcpp::Publisher<robot_interfaces::msg::MechanismCommand>::SharedPtr mechanism_cmd_pub;

  // Cached latest messages
  std::mutex data_mutex;
  std::optional<robot_interfaces::msg::MechanismFeedback> latest_mechanism_feedback;
  std::optional<robot_interfaces::msg::VisionSceneState> latest_vision_state;
  std::optional<geometry_msgs::msg::PoseStamped> latest_final_pose;

  // Mission flags
  bool start_signal = false;
};
