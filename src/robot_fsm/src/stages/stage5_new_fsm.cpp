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
  // 導航功能已停用（僅保留任務機構流程），直接視為抵達並跳過。
  (void)timeout_sec;
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage5][Nav] navigation disabled, skip goal '%s'",
    target_name.c_str());
  return true;
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