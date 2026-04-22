#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_controller.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 建立 ROS node
  auto ros_node = rclcpp::Node::make_shared("robot_fsm_main");

  // 建立共享的 robot context(包含最新的感知和機構狀態)
  auto ctx = std::make_shared<RobotContext>();
  ctx->node = ros_node;

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
  
  // 建立任務控制器，並將 robot context 傳入
  MissionController mission(ctx);

  // 先模擬比賽開始的signal，ㄍ先用五秒代替
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