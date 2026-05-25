#include "robot_fsm/stages/stage2_clam_fsm.hpp"

Stage2ClamFSM::Stage2ClamFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage2State::S2_ENTER),
  tick_count_(0),
  state_command_sent_(false)
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
      publish_state_command(201, "S2_ENTER", "stage2_enter");
      enter_state(Stage2State::S2_APPROACH);
      return false;

    case Stage2State::S2_APPROACH:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] APPROACH 模擬靠近蛤蜊");
      publish_state_command(202, "S2_APPROACH", "approach_clam_area");
      if (wait_ticks(10)) enter_state(Stage2State::S2_PICK_CLAM);
      return false;

    case Stage2State::S2_PICK_CLAM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] PICK_CLAM 模擬抓取蛤蜊");
      publish_state_command(203, "S2_PICK_CLAM", "pick_clam");
      if (wait_ticks(10)) enter_state(Stage2State::S2_RETURN);
      return false;

    case Stage2State::S2_RETURN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] RETURN 模擬返回");
      publish_state_command(204, "S2_RETURN", "return_from_clam_area");
      if (wait_ticks(10)) enter_state(Stage2State::S2_DONE);
      return false;

    case Stage2State::S2_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] DONE 第二關完成");
      publish_state_command(205, "S2_DONE", "stage2_done");
      return true;

    case Stage2State::S2_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage2] FAILED");
      publish_state_command(299, "S2_FAILED", "stage2_failed");
      return false;
  }

  return false;
}