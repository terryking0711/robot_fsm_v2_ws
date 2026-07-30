#include "robot_fsm/stages/stage3_hay_fsm.hpp"

#include <chrono>

Stage3HayFSM::Stage3HayFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage3State::S3_ENTER),
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

bool Stage3HayFSM::wait_ticks(int required_ticks)
{
  tick_count_++;
  if (tick_count_ >= required_ticks) {
    tick_count_ = 0;
    return true;
  }
  return false;
}

void Stage3HayFSM::enter_state(Stage3State next_state)
{
  state_ = next_state;
  tick_count_ = 0;
  state_command_sent_ = false;
  nav_arrived_ = false;
}

bool Stage3HayFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  // ---- 導航總開關（RobotContext::enable_navigation）----
  // false 時不送 goal，直接視為已抵達，讓機構流程可以在沒有
  // navigation_server / Nav2 / localization_manager 的情況下單獨測試。
  if (!ctx_->enable_navigation) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Stage3][Nav] navigation disabled, skip goal '%s'",
      target_name.c_str());
    return true;
  }

  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3][Nav] nav_client not initialized");
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
        "[Stage3][Nav] /navigate_to_named_pose server not ready, waiting...");
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
      "[Stage3][Nav] send navigation goal: %s (attempt %d)",
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
            "[Stage3][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3][NavFeedback] state=%s progress=%.2f",
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
          "[Stage3][NavResult] success=%s message=%s",
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
      "[Stage3][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage3][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage3][Nav] navigation to '%s' failed %d times in a row. "
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

void Stage3HayFSM::publish_state_command(
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
    R"({"stage":"stage3_hay","state":")" + state_name +
    R"(","action":")" + action_name +
    R"(","extra":)" + extra_json + R"(})";

  ctx_->mechanism_cmd_pub->publish(msg);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage3][STM_CMD] id=%u name=%s json=%s",
    msg.command_id,
    msg.command_name.c_str(),
    msg.arg_json.c_str());

  state_command_sent_ = true;
}

bool Stage3HayFSM::tick()
{
  switch (state_) {

    case Stage3State::S3_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] ENTER 第三關：稻草堆疊");
      publish_state_command(301, "S3_ENTER", "stage3_enter");
      enter_state(Stage3State::S3_LOCALIZE);
      return false;

    case Stage3State::S3_LOCALIZE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] LOCALIZE 模擬定位");
      publish_state_command(302, "S3_LOCALIZE", "localize");
      if (wait_ticks(5)) enter_state(Stage3State::S3_SCAN_HAY);
      return false;

    case Stage3State::S3_SCAN_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] SCAN_HAY 模擬視覺掃描");
      publish_state_command(303, "S3_SCAN_HAY", "scan_hay");
      if (wait_ticks(5)) enter_state(Stage3State::S3_PLAN_STACK);
      return false;

    case Stage3State::S3_PLAN_STACK:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLAN_STACK 模擬規劃堆疊");
      publish_state_command(304, "S3_PLAN_STACK", "plan_stack");
      if (wait_ticks(5)) enter_state(Stage3State::S3_SELECT_TARGET);
      return false;

    case Stage3State::S3_SELECT_TARGET:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] SELECT_TARGET 模擬選擇目標");
      publish_state_command(305, "S3_SELECT_TARGET", "select_target");
      enter_state(Stage3State::S3_NAV_TO_PICK);
      return false;

    case Stage3State::S3_NAV_TO_PICK:
      if (!nav_arrived_) {
        if (!navigate_to_named_pose("stage3_pick_pose", 30.0f)) {
          return false;
        }
        nav_arrived_ = true;
        tick_count_ = 0;  // 導航耗時不計入後面的 wait_ticks
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_PICK 抵達抓取點");
      publish_state_command(306, "S3_NAV_TO_PICK", "nav_to_pick");
      if (wait_ticks(10)) enter_state(Stage3State::S3_PICK_HAY);
      return false;

    case Stage3State::S3_PICK_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PICK_HAY 抓取稻草卷");
      publish_state_command(307, "S3_PICK_HAY", "pick_hay");
      if (wait_ticks(10)) enter_state(Stage3State::S3_NAV_TO_STACK);
      return false;

    case Stage3State::S3_NAV_TO_STACK:
      if (!nav_arrived_) {
        if (!navigate_to_named_pose("stage3_stack_pose", 30.0f)) {
          return false;
        }
        nav_arrived_ = true;
        tick_count_ = 0;  // 導航耗時不計入後面的 wait_ticks
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_STACK 抵達堆疊點");
      publish_state_command(308, "S3_NAV_TO_STACK", "nav_to_stack");
      if (wait_ticks(10)) enter_state(Stage3State::S3_PLACE_HAY);
      return false;

    case Stage3State::S3_PLACE_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLACE_HAY 放置稻草卷");
      publish_state_command(309, "S3_PLACE_HAY", "place_hay");
      if (wait_ticks(10)) enter_state(Stage3State::S3_VERIFY_STABLE);
      return false;

    case Stage3State::S3_VERIFY_STABLE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] VERIFY_STABLE 確認穩定");
      publish_state_command(310, "S3_VERIFY_STABLE", "verify_stable");
      if (wait_ticks(5)) enter_state(Stage3State::S3_DONE);
      return false;

    case Stage3State::S3_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] DONE 第三關完成");
      publish_state_command(311, "S3_DONE", "stage3_done");
      return true;

    case Stage3State::S3_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3] FAILED");
      publish_state_command(399, "S3_FAILED", "stage3_failed");
      return false;
  }

  return false;
}