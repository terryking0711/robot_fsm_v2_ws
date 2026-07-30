#include "robot_fsm/stages/stage5_new_fsm.hpp"

#include <chrono>

Stage5NewFSM::Stage5NewFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(State::ENTER),
  tick_count_(0),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_(""),
  nav_retry_count_(0),
  nav_backoff_ticks_(0)
{
}

void Stage5NewFSM::reset()
{
  state_ = State::ENTER;
  tick_count_ = 0;
}

bool Stage5NewFSM::wait_ticks(int required_ticks)
{
  tick_count_++;
  if (tick_count_ >= required_ticks) {
    tick_count_ = 0;
    return true;
  }
  return false;
}

bool Stage5NewFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  // ---- 導航總開關（RobotContext::enable_navigation）----
  // false 時不送 goal，直接視為已抵達，讓機構流程可以在沒有
  // navigation_server / Nav2 / localization_manager 的情況下單獨測試。
  if (!ctx_->enable_navigation) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Stage5][Nav] navigation disabled, skip goal '%s'",
      target_name.c_str());
    return true;
  }

  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage5][Nav] nav_client not initialized");
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
        "[Stage5][Nav] /navigate_to_named_pose server not ready, waiting...");
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
      "[Stage5][Nav] send navigation goal: %s (attempt %d)",
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
            "[Stage5][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage5][NavFeedback] state=%s progress=%.2f",
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
          "[Stage5][NavResult] success=%s message=%s",
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
      "[Stage5][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage5][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage5][Nav] navigation to '%s' failed %d times in a row. "
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

StageStatus Stage5NewFSM::tick()
{
  switch (state_) {

    case State::ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] ENTER 第五關");
      state_ = State::LOCALIZE;
      return StageStatus::RUNNING;

    case State::LOCALIZE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] LOCALIZE 模擬定位");
      if (wait_ticks(5)) state_ = State::SCAN;
      return StageStatus::RUNNING;

    case State::SCAN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] SCAN 模擬感測");
      if (wait_ticks(5)) state_ = State::PLAN;
      return StageStatus::RUNNING;

    case State::PLAN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] PLAN 模擬決策");
      if (wait_ticks(5)) state_ = State::EXECUTE;
      return StageStatus::RUNNING;

    case State::EXECUTE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] EXECUTE 模擬執行任務");
      if (wait_ticks(8)) state_ = State::VERIFY;
      return StageStatus::RUNNING;

    case State::VERIFY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] VERIFY 模擬確認");
      if (wait_ticks(3)) state_ = State::DONE;
      return StageStatus::RUNNING;

    case State::DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage5] DONE 第五關完成");
      return StageStatus::SUCCESS;
  }

  return StageStatus::FAILURE;
}