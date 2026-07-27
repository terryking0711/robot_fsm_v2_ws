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
  nav_backoff_ticks_(0)
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

bool Stage2ClamFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage2][Nav] nav_client not initialized");
    return false;
  }

  // 失敗後的退避期：先消化 backoff ticks，再允許重送
  if (nav_backoff_ticks_ > 0) {
    nav_backoff_ticks_--;
    return false;
  }

  if (!nav_goal_sent_) {
    if (!ctx_->nav_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        ctx_->node->get_logger(),
        "[Stage2][Nav] /navigate_to_named_pose server not ready, waiting...");
      return false;
    }

    NavigateAction::Goal goal;
    goal.target_name = target_name;
    goal.timeout_sec = timeout_sec;

    nav_goal_sent_ = true;
    nav_done_ = false;
    nav_success_ = false;
    nav_message_.clear();
    nav_active_target_ = target_name;

    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Stage2][Nav] send navigation goal: %s (attempt %d)",
      target_name.c_str(),
      nav_retry_count_ + 1);

    auto options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();

    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          nav_done_ = true;
          nav_success_ = false;
          nav_message_ = "goal rejected by navigation_server (unknown pose name?)";

          RCLCPP_ERROR(
            ctx_->node->get_logger(),
            "[Stage2][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage2][NavFeedback] state=%s progress=%.2f",
          feedback->current_state.c_str(),
          feedback->progress);
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
      {
        nav_done_ = true;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          nav_success_ = result.result->success;
          nav_message_ = result.result->message;
        } else {
          nav_success_ = false;
          nav_message_ = result.result
            ? result.result->message
            : (result.code == rclcpp_action::ResultCode::CANCELED
                ? "navigation goal canceled"
                : "navigation goal aborted");
        }

        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage2][NavResult] success=%s message=%s",
          nav_success_ ? "true" : "false",
          nav_message_.c_str());
      };

    ctx_->nav_client->async_send_goal(goal, options);

    return false;
  }

  if (!nav_done_) {
    return false;
  }

  const bool result = nav_success_;

  if (result) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Stage2][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage2][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage2][Nav] navigation to '%s' failed %d times in a row. "
        "Check: (1) Cartographer localization / initial pose, "
        "(2) named_poses.yaml target inside map & free space, "
        "(3) Nav2 lifecycle nodes all active.",
        nav_active_target_.c_str(),
        nav_retry_count_);
    }
  }

  nav_goal_sent_ = false;
  nav_done_ = false;
  nav_success_ = false;
  nav_message_.clear();
  nav_active_target_.clear();

  return result;
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
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] APPROACH 靠近蛤蜊");
      publish_state_command(202, "S2_APPROACH", "approach_clam_area");
      if (wait_ticks(10)) enter_state(Stage2State::S2_EXTEND_ARM);
      return false;

    case Stage2State::S2_EXTEND_ARM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] EXTEND_ARM 伸出手臂");
      publish_state_command(203, "S2_EXTEND_ARM", "extend_arm");
      if (wait_ticks(10)) enter_state(Stage2State::S2_PUSH_CLAM);
      return false;

    case Stage2State::S2_PUSH_CLAM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] PUSH_CLAM 將蛤蜊推入箱中");
      publish_state_command(204, "S2_PUSH_CLAM", "push_clam");
      if (wait_ticks(10)) enter_state(Stage2State::S2_RETRACT_ARM);
      return false;

    case Stage2State::S2_RETRACT_ARM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] RETRACT_ARM 收回手臂");
      publish_state_command(205, "S2_RETRACT_ARM", "retract_arm");
      if (wait_ticks(10)) enter_state(Stage2State::S2_MOVE_FORWARD_ALIGN);
      return false;

    case Stage2State::S2_MOVE_FORWARD_ALIGN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] MOVE_FORWARD_ALIGN 前進並對齊箱子");
      publish_state_command(206, "S2_MOVE_FORWARD_ALIGN", "move_forward_align");
      if (wait_ticks(10)) enter_state(Stage2State::S2_ROTATE_BOX);
      return false;

    case Stage2State::S2_ROTATE_BOX:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] ROTATE_BOX 翻轉箱子");
      publish_state_command(207, "S2_ROTATE_BOX", "rotate_box");
      if (wait_ticks(10)) enter_state(Stage2State::S2_MOVE_TO_RETURN);
      return false;

    case Stage2State::S2_MOVE_TO_RETURN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] MOVE_TO_RETURN 移動至返回點");
      publish_state_command(208, "S2_MOVE_TO_RETURN", "move_to_return");
      if (wait_ticks(10)) enter_state(Stage2State::S2_DONE);
      return false;

    case Stage2State::S2_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] DONE 第二關完成");
      publish_state_command(210, "S2_DONE", "stage2_done");
      return true;
  }

  return false;
}