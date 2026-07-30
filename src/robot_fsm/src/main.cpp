#include <memory>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_controller.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"

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
  // "start" 預設 = map 原點在 world 的位置 (0.425, 1.0, 0.0)。
  // TODO: reset_stage1~4 為佔位值，實際重置點座標量測後用 launch/yaml 覆蓋。
  // ------------------------------------------------------------------
  ctx->field_poses["start"] = load_pose_param(ros_node, "start_pose", {0.425, 1.0, 0.0});
  ctx->field_poses["reset_stage1"] = load_pose_param(ros_node, "reset_pose_stage1", {0.425, 1.0, 0.0});
  ctx->field_poses["reset_stage2"] = load_pose_param(ros_node, "reset_pose_stage2", {0.425, 1.0, 0.0});
  ctx->field_poses["reset_stage3"] = load_pose_param(ros_node, "reset_pose_stage3", {0.425, 1.0, 0.0});
  ctx->field_poses["reset_stage4"] = load_pose_param(ros_node, "reset_pose_stage4", {0.425, 1.0, 0.0});

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

  // localization_manager 的初始化結果回報
  auto init_status_sub =
    ros_node->create_subscription<std_msgs::msg::Bool>(
      "/init_pose_status", 10,
      [ctx](const std_msgs::msg::Bool::SharedPtr msg) {
        ctx->init_status_received = true;
        ctx->init_status_ok = msg->data;
      });

  // 場外重置請求：隊員將機器人搬到第 N 關重置點後，發布：
  //   ros2 topic pub --once /reset_cmd std_msgs/msg/UInt8 "{data: 2}"
  auto reset_sub =
    ros_node->create_subscription<std_msgs::msg::UInt8>(
      "/reset_cmd", 10,
      [ctx](const std_msgs::msg::UInt8::SharedPtr msg) {
        RCLCPP_WARN(
          ctx->node->get_logger(),
          "[Main] reset request received: stage %u", msg->data);
        ctx->pending_reset_stage = msg->data;
      });

  // 建立任務控制器，並將 robot context 傳入
  MissionController mission(ctx);

  // 先模擬比賽開始的signal，先用五秒代替
  std::thread starter([ctx]() {
    std::this_thread::sleep_for(std::chrono::seconds(5));
    ctx->start_signal = true;
  });
  starter.detach();

  // 主循環，就跟 Arduino 的 loop() 一樣
  rclcpp::WallRate loop_rate(10);

  // 呼叫 mission.tick()
  while (rclcpp::ok()) {
    rclcpp::spin_some(ros_node);
    mission.tick();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
