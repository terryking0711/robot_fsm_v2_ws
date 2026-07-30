#include "robot_fsm/stages/stage2_clam_fsm.hpp"

#include <chrono>

Stage2ClamFSM::Stage2ClamFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage2State::S2_ENTER),
  tick_count_(0),
  state_command_sent_(false),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_(""),
  nav_retry_count_(0),
  nav_backoff_ticks_(0),
  nav_arrived_(false)
{
}

bool Stage2ClamFSM::wait_ticks(int required_ticks)
{
  tick_count_++;
  if (tick_count_ >= required_ticks) {
    tick_count_ = 0;
    return true;
  }
  return false;
}

void Stage2ClamFSM::enter_state(Stage2State next_state)
{
  state_ = next_state;
  tick_count_ = 0;
  state_command_sent_ = false;
  nav_arrived_ = false;
}

bool Stage2ClamFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  // 導航功能已停用（僅保留任務機構流程），直接視為抵達並跳過。
  (void)timeout_sec;
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage2][Nav] navigation disabled, skip goal '%s'",
    target_name.c_str());
  return true;
}

void Stage2ClamFSM::publish_state_command(
  uint16_t command_id,
  const std::string& state_name,
  const std::string& action_name,
  const std::string& extra_json)
{
  if (state_command_sent_) {
    return;
  }

  robot_interfaces::msg::MechanismCommand msg;
  msg.command_id = command_id;
  msg.command_name = action_name;
  msg.arg_json =
    R"({"stage":"stage2_clam","state":")" + state_name +
    R"(","action":")" + action_name +
    R"(","extra":)" + extra_json + R"(})";

  ctx_->mechanism_cmd_pub->publish(msg);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage2][STM_CMD] id=%u name=%s json=%s",
    msg.command_id,
    msg.command_name.c_str(),
    msg.arg_json.c_str());

  state_command_sent_ = true;
}

bool Stage2ClamFSM::tick()
{
  switch (state_) {

    case Stage2State::S2_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] ENTER 第二關");
      wait_ticks(10);
      enter_state(Stage2State::S2_APPROACH);
      return false;

    case Stage2State::S2_APPROACH:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] APPROACH 靠近蛤蜊");
      enter_state(Stage2State::S2_EXTEND_ARM);
      return false;

    case Stage2State::S2_EXTEND_ARM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] EXTEND_ARM 伸出手臂");
      publish_state_command(101, "S2_EXTEND_ARM", "extend_arm");
      if (wait_ticks(30)) enter_state(Stage2State::S2_PUSH_CLAM);
      return false;

    case Stage2State::S2_PUSH_CLAM:
      if (!nav_arrived_) {
        if (!navigate_to_named_pose("stage2_push_clam", 15.0f)) {
          return false;
        }
        nav_arrived_ = true;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] PUSH_CLAM 將蛤蜊推入箱中");
      enter_state(Stage2State::S2_RETRACT_ARM);
      return false;

    case Stage2State::S2_RETRACT_ARM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] RETRACT_ARM 收回手臂");
      publish_state_command(102, "S2_RETRACT_ARM", "retract_arm");
      if (wait_ticks(30)) enter_state(Stage2State::S2_MOVE_FORWARD_ALIGN);
      return false;

    case Stage2State::S2_MOVE_FORWARD_ALIGN:
      if (!nav_arrived_) {
        if (!navigate_to_named_pose("stage2_push_clam", 15.0f)) {
          return false;
        }
        nav_arrived_ = true;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] MOVE_FORWARD_ALIGN 前進並對齊箱子");
      enter_state(Stage2State::S2_ROTATE_BOX);
      return false;

    case Stage2State::S2_ROTATE_BOX:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] ROTATE_BOX 翻轉箱子");
      publish_state_command(201, "S2_ROTATE_BOX", "rotate_box");
      if (wait_ticks(60)) enter_state(Stage2State::S2_MOVE_TO_RETURN);
      return false;

    case Stage2State::S2_MOVE_TO_RETURN:
      if (!nav_arrived_) {
        if (!navigate_to_named_pose("stage2_return_point", 15.0f)) {
          return false;
        }
        nav_arrived_ = true;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] MOVE_TO_RETURN 移動至返回點");
      enter_state(Stage2State::S2_DROP_BOX);
      return false;
    
    case Stage2State::S2_DROP_BOX:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] DROP_BOX 放下箱子");
      publish_state_command(202, "S2_DROP_BOX", "drop_box");
      if (wait_ticks(20)) enter_state(Stage2State::S2_DONE);
      return false;
    
    case Stage2State::S2_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] DONE 第二關完成");
      return true;
  }

  return false;
}