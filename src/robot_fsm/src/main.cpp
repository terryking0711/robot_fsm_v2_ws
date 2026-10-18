#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <yaml-cpp/yaml.h>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_controller.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/straw_pick_status.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"
#include "straw_interfaces/msg/straw_target.hpp"

namespace
{

// 從 ROS parameter 載入場地定位點（world frame，[x, y, yaw]）。
// 可用 launch/yaml 覆蓋，不需重編。
Pose2D load_pose_param(
  const rclcpp::Node::SharedPtr& node,
  const std::string& name,
  const std::vector<double>& default_xyyaw)
{
  node->declare_parameter(name, default_xyyaw);
  const auto v = node->get_parameter(name).as_double_array();

  Pose2D p;
  if (v.size() == 3) {
    p.x = v[0];
    p.y = v[1];
    p.yaw = v[2];
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "parameter '%s' must be [x, y, yaw], fallback to default", name.c_str());
    p.x = default_xyyaw[0];
    p.y = default_xyyaw[1];
    p.yaw = default_xyyaw[2];
  }

  RCLCPP_INFO(
    node->get_logger(),
    "field pose '%s' (world): [%.3f, %.3f, %.3f]", name.c_str(), p.x, p.y, p.yaw);
  return p;
}

Pose2D load_named_pose_from_yaml(
  const rclcpp::Node::SharedPtr& node,
  const std::string& named_poses_file,
  const std::string& pose_name,
  const std::vector<double>& fallback_xyyaw,
  const std::string& fallback_param_name)
{
  if (!named_poses_file.empty()) {
    try {
      const YAML::Node root = YAML::LoadFile(named_poses_file);
      const YAML::Node poses = root["named_poses"];

      if (poses && poses.IsMap() && poses[pose_name]) {
        const YAML::Node pose_node = poses[pose_name];

        if (pose_node["x"] && pose_node["y"] && pose_node["yaw"]) {
          Pose2D pose;
          pose.x = pose_node["x"].as<double>();
          pose.y = pose_node["y"].as<double>();
          pose.yaw = pose_node["yaw"].as<double>();

          RCLCPP_INFO(
            node->get_logger(),
            "field pose '%s' loaded from named_poses.yaml: [%.3f, %.3f, %.3f]",
            pose_name.c_str(), pose.x, pose.y, pose.yaw);
          return pose;
        }

        RCLCPP_WARN(
          node->get_logger(),
          "named pose '%s' in %s is missing x/y/yaw, fallback to parameter '%s'",
          pose_name.c_str(), named_poses_file.c_str(), fallback_param_name.c_str());
      } else {
        RCLCPP_WARN(
          node->get_logger(),
          "named pose '%s' not found in %s, fallback to parameter '%s'",
          pose_name.c_str(), named_poses_file.c_str(), fallback_param_name.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(
        node->get_logger(),
        "failed to load %s for pose '%s': %s, fallback to parameter '%s'",
        named_poses_file.c_str(), pose_name.c_str(), e.what(), fallback_param_name.c_str());
    }
  }

  return load_pose_param(node, fallback_param_name, fallback_xyyaw);
}

// 讀 fsm_side_<side>.yaml 的 command_id_remap（程式代號 -> 該場地代號）。
// 讀不到就維持空表 = 全部照程式代號送出。
void load_command_id_remap(
  const rclcpp::Node::SharedPtr& node,
  const std::string& fsm_side_file,
  std::map<uint16_t, uint16_t>& remap)
{
  if (fsm_side_file.empty()) {
    RCLCPP_WARN(node->get_logger(), "[Main] fsm_side_file is empty, no command_id remap");
    return;
  }

  try {
    const YAML::Node table = YAML::LoadFile(fsm_side_file)["command_id_remap"];
    if (table && table.IsMap()) {
      for (auto it = table.begin(); it != table.end(); ++it) {
        remap[it->first.as<uint16_t>()] = it->second.as<uint16_t>();
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      node->get_logger(),
      "[Main] failed to load command_id_remap from %s: %s", fsm_side_file.c_str(), e.what());
    return;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "[Main] %zu command_id remap(s) from %s", remap.size(), fsm_side_file.c_str());
  for (const auto& [from, to] : remap) {
    RCLCPP_INFO(node->get_logger(), "[Main]   command_id %u -> %u", from, to);
  }
}

// 讀 fsm_side_<side>.yaml 的單一數值（例如 stage2_push_clam_x_sign）。
// 讀不到就用 default_value。
double load_fsm_side_double(
  const rclcpp::Node::SharedPtr& node,
  const std::string& fsm_side_file,
  const std::string& key,
  double default_value)
{
  double value = default_value;
  if (!fsm_side_file.empty()) {
    try {
      const YAML::Node v = YAML::LoadFile(fsm_side_file)[key];
      if (v) {
        value = v.as<double>();
      } else {
        RCLCPP_WARN(
          node->get_logger(), "[Main] '%s' not found in %s, use default %.3f",
          key.c_str(), fsm_side_file.c_str(), default_value);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        node->get_logger(), "[Main] failed to load '%s' from %s: %s, use default %.3f",
        key.c_str(), fsm_side_file.c_str(), e.what(), default_value);
    }
  }

  RCLCPP_INFO(node->get_logger(), "[Main] %s = %.3f", key.c_str(), value);
  return value;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 建立 ROS node
  auto ros_node = rclcpp::Node::make_shared("robot_fsm_main");

  // 建立共享的 robot context(包含最新的感知和機構狀態)
  auto ctx = std::make_shared<RobotContext>();
  ctx->node = ros_node;

  // ------------------------------------------------------------------
  // 導航總開關
  //   ros2 run robot_fsm robot_fsm_main --ros-args -p enable_navigation:=false
  //   ros2 launch robot_fsm robot_bringup.launch.py enable_navigation:=false
  // false 時：不啟動 navigation_server、FSM 內所有導航/定位步驟直接跳過，
  //           只跑機構流程（等同先前 mission_test branch 的行為）。
  // ------------------------------------------------------------------
  ros_node->declare_parameter("enable_navigation", true);
  ctx->enable_navigation = ros_node->get_parameter("enable_navigation").as_bool();

  ros_node->declare_parameter<std::string>("named_poses_file", "");
  const auto named_poses_file = ros_node->get_parameter("named_poses_file").as_string();

  // ------------------------------------------------------------------
  // 比賽場地 normal / mirror（由 robot_bringup.launch.py field_side:=... 依
  // /workspaces/field_config.yaml 決定 named_poses_file 與 fsm_side_file）
  // ------------------------------------------------------------------
  ros_node->declare_parameter<std::string>("field_side", "normal");
  ros_node->declare_parameter<std::string>("fsm_side_file", "");
  ctx->field_side = ros_node->get_parameter("field_side").as_string();
  ros_node->declare_parameter("mirror_v2", false);
  ctx->mirror_v2 = ros_node->get_parameter("mirror_v2").as_bool();
  if (ctx->mirror_v2 && ctx->field_side != "mirror") {
    throw std::runtime_error("mirror_v2 requires field_side=mirror");
  }
  RCLCPP_WARN(
    ros_node->get_logger(),
    "[Main] FIELD SIDE = %s (named_poses_file: %s)",
    ctx->field_side.c_str(), named_poses_file.c_str());
  const auto fsm_side_file = ros_node->get_parameter("fsm_side_file").as_string();
  load_command_id_remap(ros_node, fsm_side_file, ctx->command_id_remap);
  ctx->stage2_push_clam_x_sign =
    load_fsm_side_double(ros_node, fsm_side_file, "stage2_push_clam_x_sign", 1.0);

  ctx->stage3_row_return_speed = load_fsm_side_double(
    ros_node, fsm_side_file, "stage3_row_return_speed", 0.42);
  ctx->stage3_row_return_slow_speed = load_fsm_side_double(
    ros_node, fsm_side_file, "stage3_row_return_slow_speed", 0.25);
  if (!std::isfinite(ctx->stage3_row_return_speed) ||
    !std::isfinite(ctx->stage3_row_return_slow_speed) ||
    ctx->stage3_row_return_slow_speed <= 0.0 ||
    ctx->stage3_row_return_speed < ctx->stage3_row_return_slow_speed)
  {
    throw std::runtime_error("ROW_RETURN requires 0 < slow_speed <= speed");
  }

  ctx->stage3_row_shift_max_speed = load_fsm_side_double(
    ros_node, fsm_side_file, "stage3_row_shift_max_speed", 0.16);
  if (!std::isfinite(ctx->stage3_row_shift_max_speed) || ctx->stage3_row_shift_max_speed <= 0.0) {
    throw std::runtime_error("stage3_row_shift_max_speed must be finite and positive");
  }

  // DIT runs stage 3 with the same motion settings as mirror.
  if (ctx->field_side == "mirror" || ctx->field_side == "dit") {
    ctx->stage2_shift_to_return_velocity_y = load_fsm_side_double(
      ros_node, fsm_side_file, "stage2_shift_to_return_velocity_y", -0.267);
    ctx->stage2_shift_to_return_duration_sec = load_fsm_side_double(
      ros_node, fsm_side_file, "stage2_shift_to_return_duration_sec", 1.7);
    ctx->stage3_cross_accel_distance = load_fsm_side_double(
      ros_node, fsm_side_file, "stage3_cross_accel_distance", 0.2);
    ctx->stage3_cross_max_speed = load_fsm_side_double(
      ros_node, fsm_side_file, "stage3_cross_max_speed", 0.6);
    ctx->stage3_cross_cruise_duration_sec = load_fsm_side_double(
      ros_node, fsm_side_file, "stage3_cross_cruise_duration_sec", 0.8);
    ctx->stage3_cross_timeout_sec = load_fsm_side_double(
      ros_node, fsm_side_file, "stage3_cross_timeout_sec", 10.0);
    if (!std::isfinite(ctx->stage3_cross_accel_distance) || ctx->stage3_cross_accel_distance <= 0.0 ||
      !std::isfinite(ctx->stage3_cross_max_speed) || ctx->stage3_cross_max_speed <= 0.05 ||
      !std::isfinite(ctx->stage3_cross_cruise_duration_sec) || ctx->stage3_cross_cruise_duration_sec <= 0.0 ||
      !std::isfinite(ctx->stage3_cross_timeout_sec) ||
      ctx->stage3_cross_timeout_sec <= ctx->stage3_cross_cruise_duration_sec)
    {
      throw std::runtime_error("invalid mirror/DIT CROSS settings");
    }
    if (!std::isfinite(ctx->stage2_shift_to_return_velocity_y) ||
      !std::isfinite(ctx->stage2_shift_to_return_duration_sec) ||
      ctx->stage2_shift_to_return_duration_sec <= 0.0)
    {
      throw std::runtime_error("invalid mirror/DIT SHIFT_TO_RETURN settings");
    }
  }

  ros_node->declare_parameter<double>("match_deadline_unix_sec", 0.0);
  const double deadline_unix = ros_node->get_parameter("match_deadline_unix_sec").as_double();
  if (deadline_unix > 0.0) {
    const double unix_now = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    ctx->match_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(deadline_unix - unix_now));
    RCLCPP_INFO(ros_node->get_logger(),
      "[Main] match timer: %.1f s remaining; stop collecting below %.0f s",
      ctx->match_remaining_sec(), RobotContext::kStage3ReturnReserveSec);
  }

  // 略過第二關（field_config.yaml 該 side 的 skip_stage2，由 robot_bringup.launch.py 傳入）
  ros_node->declare_parameter("skip_stage2", false);
  ctx->skip_stage2 = ros_node->get_parameter("skip_stage2").as_bool();
  if (ctx->skip_stage2) {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[Main] skip_stage2=true -- STAGE2_CLAM skipped, "
      "WAIT_START -> stage3_entry -> STAGE3_HAY");
  }

  // 第二關關閉手臂 state（robot_bringup.launch.py skip_stage2_arm:=true）
  ros_node->declare_parameter("skip_stage2_arm", false);
  ctx->skip_stage2_arm = ros_node->get_parameter("skip_stage2_arm").as_bool();
  if (ctx->skip_stage2_arm) {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[Main] skip_stage2_arm=true -- STAGE2 arm states "
      "(2000/2001/203/204/205/206/111/2021/207) skipped");
  }

  // 只跑第三關（robot_bringup.launch.py only_stage3:=true）
  ros_node->declare_parameter("only_stage3", false);
  ctx->only_stage3 = ros_node->get_parameter("only_stage3").as_bool();
  if (ctx->only_stage3) {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[Main] only_stage3=true -- STAGE2_CLAM skipped, "
      "stage3_entry -> STAGE3_HAY");
  }

  ros_node->declare_parameter("stage3_motion_test", false);
  ctx->stage3_motion_test = ros_node->get_parameter("stage3_motion_test").as_bool();
  if (ctx->stage3_motion_test && (ctx->field_side != "dit" || !ctx->enable_navigation)) {
    throw std::runtime_error("stage3_motion_test requires field_side=dit and enable_navigation=true");
  }

  ros_node->declare_parameter("reset_stage2", false);
  ros_node->declare_parameter("reset_stage3", false);
  ctx->reset_stage2 = ros_node->get_parameter("reset_stage2").as_bool();
  ctx->reset_stage3 = ros_node->get_parameter("reset_stage3").as_bool();
  if (ctx->stage3_motion_test && (ctx->reset_stage2 || ctx->reset_stage3)) {
    throw std::runtime_error("stage3_motion_test cannot be combined with S2/S3 RESET");
  }
  if (ctx->reset_stage2 && ctx->reset_stage3) {
    throw std::runtime_error("reset_stage2 and reset_stage3 are mutually exclusive");
  }
  if (ctx->reset_stage2) {
    ctx->only_stage3 = false;
    ctx->skip_stage2 = false;
    ctx->skip_stage2_arm = true;
    RCLCPP_WARN(ros_node->get_logger(),
      "[Main] S2 RESET: keep localization/deadline, rerun stage 2 -> stage 3");
  }
  if (ctx->reset_stage3) {
    RCLCPP_WARN(ros_node->get_logger(),
      "[Main] S3 RESET: keep localization/deadline, DirectTransition corner -> stage3_entry");
  }

  if (ctx->enable_navigation) {
    RCLCPP_INFO(ros_node->get_logger(), "[Main] navigation ENABLED");
  } else {
    RCLCPP_WARN(
      ros_node->get_logger(),
      "[Main] navigation DISABLED (enable_navigation=false) -- "
      "all nav goals will be skipped and localization treated as success. "
      "MECHANISM-ONLY TEST MODE, do not use in competition.");
  }

  // ------------------------------------------------------------------
  // 場地定位點（world frame，場地最左下角為 (0,0)）
  // Legacy localization tooling reads reset_pose_stage3; S2/S3 RESET do not use it.
  // 從該場地的 named_poses_<side>.yaml 讀 reset_pose_stage3，
  // 找不到才退回同名 ROS parameter（預設值如下）。
  // ------------------------------------------------------------------
  ctx->field_poses["reset_stage3"] = load_named_pose_from_yaml(
    ros_node, named_poses_file, "reset_pose_stage3", {0.425, 1.0, 0.0}, "reset_pose_stage3");

  // FSM 唯一的 /cmd_vel publisher（各 stage 共用，見 RobotContext::cmd_vel_pub）
  ctx->cmd_vel_pub = ros_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // topic = /vision/scene_state 還沒創立 publisher
  auto vision_sub =
    ros_node->create_subscription<robot_interfaces::msg::VisionSceneState>(
      "/vision/scene_state", 10,
      [ctx](const robot_interfaces::msg::VisionSceneState::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        ctx->latest_vision_state = *msg;
      });

  // topic name = /mechanism/feedback 還沒創立 publisher
  auto mech_sub =
    ros_node->create_subscription<robot_interfaces::msg::MechanismFeedback>(
      "/mechanism/feedback", 10,
      [ctx](const robot_interfaces::msg::MechanismFeedback::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        ctx->latest_mechanism_feedback = *msg;
      });

  // topic name = /final_pose 還沒創立 publisher
  auto pose_sub =
    ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/final_pose", 10,
      [ctx](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        ctx->latest_final_pose = *msg;
      });

  // ---- 第三關用 ----
  // world frame 機器人位姿（tdk_slam_ws robot_pose_publisher）
  auto robot_pose_sub =
    ros_node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/robot_pose", 10,
      [ctx](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        ctx->latest_robot_pose = *msg;
        ctx->latest_robot_pose_time = ctx->node->now();
      });

  // 相機稻草捲偵測（TDK-straw straw_node）。
  // 用 SensorDataQoS（best effort），不論對方是 reliable 或 best effort 都接得到。
  ros_node->declare_parameter<std::string>("straw_target_topic", "/straw/target");
  auto straw_target_sub =
    ros_node->create_subscription<straw_interfaces::msg::StrawTarget>(
      ros_node->get_parameter("straw_target_topic").as_string(),
      rclcpp::SensorDataQoS(),
      [ctx](const straw_interfaces::msg::StrawTarget::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        ctx->latest_straw_target = *msg;
        ctx->latest_straw_target_time = ctx->node->now();
      });

  // 機構 STM 的自動夾取狀態（micro-ROS，可能是 best effort，所以同樣用 SensorDataQoS）
  auto straw_pick_status_sub =
    ros_node->create_subscription<robot_interfaces::msg::StrawPickStatus>(
      "/mechanism/straw_pick_status",
      rclcpp::SensorDataQoS(),
      [ctx](const robot_interfaces::msg::StrawPickStatus::SharedPtr msg) {
        std::scoped_lock lock(ctx->data_mutex);
        // switch_pressed：true = 碰到稻草（STM 韌體端已改成按下 = true，這裡不再反轉）
        ctx->latest_straw_pick_status = *msg;
        ctx->latest_straw_pick_status_time = ctx->node->now();
      });

  // localization_manager 的初始化結果回報
  auto init_status_sub =
    ros_node->create_subscription<std_msgs::msg::Bool>(
      "/init_pose_status", 10,
      [ctx](const std_msgs::msg::Bool::SharedPtr msg) {
        ctx->init_status_received = true;
        ctx->init_status_ok = msg->data;
      });

  // 建立任務控制器，並將 robot context 傳入
  MissionController mission(ctx);

  // 先模擬比賽開始的signal，先用一秒代替
  std::thread starter([ctx]() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ctx->start_signal = true;
  });
  starter.detach();

  // 主循環，就跟 Arduino 的 loop() 一樣
  // Service timers/localization at 100 Hz, while preserving the FSM's 10 Hz
  // tick counts (all wait_ticks durations depend on that rate).
  rclcpp::WallRate loop_rate(100);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(ros_node);
  unsigned tick_divider = 0;
  while (rclcpp::ok()) {
    executor.spin_some();
    if (tick_divider == 0) { mission.tick(); }
    tick_divider = (tick_divider + 1) % 10;
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
