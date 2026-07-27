#include "robot_fsm/stages/stage4_mazu_fsm.hpp"

#include <chrono>

Stage4MazuFSM::Stage4MazuFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage4State::S4_ENTER),
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

bool Stage4MazuFSM::wait_ticks(int required_ticks)
{
  tick_count_++;
  if (tick_count_ >= required_ticks) {
    tick_count_ = 0;
    return true;
  }
  return false;
}

bool Stage4MazuFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage4][Nav] nav_client not initialized");
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
        "[Stage4][Nav] /navigate_to_named_pose server not ready, waiting...");
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
      "[Stage4][Nav] send navigation goal: %s (attempt %d)",
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
            "[Stage4][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage4][NavFeedback] state=%s progress=%.2f",
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
          "[Stage4][NavResult] success=%s message=%s",
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
      "[Stage4][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage4][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage4][Nav] navigation to '%s' failed %d times in a row. "
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

bool Stage4MazuFSM::tick()
{
  switch (state_) {

    case Stage4State::S4_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage4] ENTER 第四關");
      state_ = Stage4State::S4_APPROACH;
      return false;

    case Stage4State::S4_APPROACH:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage4] APPROACH 模擬靠近媽祖");
      if (wait_ticks(10)) state_ = Stage4State::S4_PERFORM_RITUAL;
      return false;

    case Stage4State::S4_PERFORM_RITUAL:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage4] PERFORM_RITUAL 模擬執行儀式");
      if (wait_ticks(10)) state_ = Stage4State::S4_COMPLETE;
      return false;

    case Stage4State::S4_COMPLETE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage4] COMPLETE 模擬收尾");
      if (wait_ticks(5)) state_ = Stage4State::S4_DONE;
      return false;

    case Stage4State::S4_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage4] DONE 第四關完成");
      return true;

    case Stage4State::S4_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage4] FAILED");
      return false;
  }

  return false;
}
