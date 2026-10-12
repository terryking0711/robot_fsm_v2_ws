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
#include <geometry_msgs/msg/twist.hpp>

#include "robot_interfaces/action/navigate_to_named_pose.hpp"
#include "robot_interfaces/msg/mechanism_command.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"
#include "robot_interfaces/msg/stage_result.hpp"
#include "robot_interfaces/msg/straw_pick_status.hpp"
#include "straw_interfaces/msg/straw_target.hpp"

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

  // ---- 略過第二關 ----
  // 由 main.cpp 從 ROS parameter "skip_stage2" 載入（field_config.yaml 該 side 的
  // skip_stage2，DIT 自己場地測試用）。true 時 LEAVE_START_ZONE 之後直接
  // TRANSITION_TO_STAGE3（導航到 stage3_entry），整個 STAGE2_CLAM 不跑。
  bool skip_stage2 = false;

  // ---- 只跑第三關 ----
  // 由 main.cpp 從 ROS parameter "only_stage3" 載入（robot_bringup.launch.py only_stage3:=true）。
  // true 時 MECH_INIT 之後不做 LEAVE_START_ZONE，直接 TRANSITION_TO_STAGE3
  //（Nav2 到 stage3_entry）再跑 STAGE3_HAY。
  bool only_stage3 = false;

  // Navigation
  NavigateClient::SharedPtr nav_client;

  // ---- FSM 自己發的 /cmd_vel ----
  // 整個 robot_fsm_main 只用這一個 publisher（main.cpp 建立），各 stage 不要自己再建。
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  // 最近一次導航 goal 結束（成功 / 失敗 / 取消）的時間，由各處的 nav result_callback 更新。
  // Nav2 的 velocity_smoother 在導航結束後還會繼續發減速 / 0 速度直到 velocity_timeout
  //（tdk_nav2_params.yaml = 1.0 s），這段期間 FSM 如果也發 /cmd_vel，兩邊會交錯，
  // 底盤收到忽快忽停的低速指令。FSM 自己控速的 state 開始前要先等 nav_cmd_vel_settled()。
  rclcpp::Time last_nav_end_time{0, 0, RCL_ROS_TIME};
  static constexpr double kNavCmdVelSettleSec = 1.2;  // > velocity_smoother velocity_timeout

  bool nav_cmd_vel_settled() const
  {
    return (node->now() - last_nav_end_time).seconds() >= kNavCmdVelSettleSec;
  }

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

  // ---- 比賽場地（normal / mirror）----
  // 由 main.cpp 從 ROS parameter "field_side" 與 "fsm_side_file"
  // （field_config.yaml 的 sides.<side>.fsm_side_file）載入。
  std::string field_side = "normal";
  // 送 STM 前的 command_id 替換表（程式裡寫的代號 -> 該場地要送的代號）
  std::map<uint16_t, uint16_t> command_id_remap;
  // S2_PUSH_CLAM 的 linear.x 方向（fsm_side_<side>.yaml 的 stage2_push_clam_x_sign）
  //   normal = 1.0、mirror = -1.0
  double stage2_push_clam_x_sign = 1.0;

  // 所有送 STM 指令的地方都要經過這裡，鏡像場地的代號差異只改 fsm_side_<side>.yaml
  uint16_t side_command_id(uint16_t command_id) const
  {
    const auto it = command_id_remap.find(command_id);
    return it == command_id_remap.end() ? command_id : it->second;
  }

  // Mechanism
  rclcpp::Publisher<robot_interfaces::msg::MechanismCommand>::SharedPtr mechanism_cmd_pub;

  // Cached latest messages
  std::mutex data_mutex;
  std::optional<robot_interfaces::msg::MechanismFeedback> latest_mechanism_feedback;
  std::optional<robot_interfaces::msg::VisionSceneState> latest_vision_state;
  std::optional<geometry_msgs::msg::PoseStamped> latest_final_pose;

  // 第三關用：各自記錄「收到的時間」（node->now()），用來判斷資料是否過期。
  //   /robot_pose                    world frame 機器人位姿（tdk_slam_ws robot_pose_publisher）
  //   /straw/target                  相機看到的稻草捲相對機器人的偏移（TDK-straw straw_node）
  //   /mechanism/straw_pick_status   機構 STM 回報的自動夾取狀態
  std::optional<geometry_msgs::msg::PoseStamped> latest_robot_pose;
  rclcpp::Time latest_robot_pose_time{0, 0, RCL_ROS_TIME};
  std::optional<straw_interfaces::msg::StrawTarget> latest_straw_target;
  rclcpp::Time latest_straw_target_time{0, 0, RCL_ROS_TIME};
  std::optional<robot_interfaces::msg::StrawPickStatus> latest_straw_pick_status;
  rclcpp::Time latest_straw_pick_status_time{0, 0, RCL_ROS_TIME};

  // Mission flags
  bool start_signal = false;
};
