#include "robot_fsm/stages/stage1_wetland_fsm.hpp"

#include <chrono>

Stage1WetlandFSM::Stage1WetlandFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage1State::S1_ENTER),
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

bool Stage1WetlandFSM::wait_ticks(int required_ticks)
{
  tick_count_++;
  if (tick_count_ >= required_ticks) {
    tick_count_ = 0;
    return true;
  }
  return false;
}

void Stage1WetlandFSM::enter_state(Stage1State next_state)
{
  state_ = next_state;
  tick_count_ = 0;
  state_command_sent_ = false;
}

bool Stage1WetlandFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage1][Nav] nav_client not initialized");
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
        "[Stage1][Nav] /navigate_to_named_pose server not ready, waiting...");
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
      "[Stage1][Nav] send navigation goal: %s (attempt %d)",
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
            "[Stage1][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage1][NavFeedback] state=%s progress=%.2f",
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
          "[Stage1][NavResult] success=%s message=%s",
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
      "[Stage1][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage1][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage1][Nav] navigation to '%s' failed %d times in a row. "
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

void Stage1WetlandFSM::publish_state_command(
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
    R"({"stage":"stage1_wetland","state":")" + state_name +
    R"(","action":")" + action_name +
    R"(","extra":)" + extra_json + R"(})";

  ctx_->mechanism_cmd_pub->publish(msg);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage1][STM_CMD] id=%u name=%s json=%s",
    msg.command_id,
    msg.command_name.c_str(),
    msg.arg_json.c_str());

  state_command_sent_ = true;
}

bool Stage1WetlandFSM::tick()
{
  switch (state_) {

    case Stage1State::S1_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] ENTER 第一關：濕地守護");
      publish_state_command(101, "S1_ENTER", "stage1_enter");
      enter_state(Stage1State::S1_LOCALIZE);
      return false;

    case Stage1State::S1_LOCALIZE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] LOCALIZE 模擬定位");
      publish_state_command(102, "S1_LOCALIZE", "localize");
      if (wait_ticks(10)) enter_state(Stage1State::S1_APPROACH_ART);
      return false;

    case Stage1State::S1_APPROACH_ART:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] APPROACH_ART 模擬靠近裝置藝術");
      publish_state_command(103, "S1_APPROACH_ART", "approach_art");
      if (wait_ticks(10)) enter_state(Stage1State::S1_FIX_ART);
      return false;

    case Stage1State::S1_FIX_ART:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] FIX_ART 模擬扶正裝置藝術");
      publish_state_command(104, "S1_FIX_ART", "fix_art");
      if (wait_ticks(15)) enter_state(Stage1State::S1_APPROACH_BIRD);
      return false;

    case Stage1State::S1_APPROACH_BIRD:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] APPROACH_BIRD 模擬靠近候鳥");
      publish_state_command(105, "S1_APPROACH_BIRD", "approach_bird");
      if (wait_ticks(10)) enter_state(Stage1State::S1_PICK_BIRD);
      return false;

    case Stage1State::S1_PICK_BIRD:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] PICK_BIRD 模擬抓取候鳥");
      publish_state_command(106, "S1_PICK_BIRD", "pick_bird", R"({"target":"bird_mock"})");
      if (wait_ticks(15)) enter_state(Stage1State::S1_MOVE_TO_RESCUE);
      return false;

    case Stage1State::S1_MOVE_TO_RESCUE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] MOVE_TO_RESCUE 模擬移動到救護站");
      publish_state_command(107, "S1_MOVE_TO_RESCUE", "move_to_rescue_station");
      if (wait_ticks(10)) enter_state(Stage1State::S1_DROP_BIRD);
      return false;

    case Stage1State::S1_DROP_BIRD:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] DROP_BIRD 模擬放置候鳥");
      publish_state_command(108, "S1_DROP_BIRD", "drop_bird");
      if (wait_ticks(15)) enter_state(Stage1State::S1_VERIFY);
      return false;

    case Stage1State::S1_VERIFY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] VERIFY 模擬確認第一關完成");
      publish_state_command(109, "S1_VERIFY", "verify_stage1");
      if (wait_ticks(8)) enter_state(Stage1State::S1_DONE);
      return false;

    case Stage1State::S1_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage1] DONE 第一關完成");
      publish_state_command(110, "S1_DONE", "stage1_done");
      return true;

    case Stage1State::S1_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage1] FAILED");
      publish_state_command(199, "S1_FAILED", "stage1_failed");
      return false;
  }

  return false;
}