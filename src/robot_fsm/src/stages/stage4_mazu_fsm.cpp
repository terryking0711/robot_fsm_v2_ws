#include "robot_fsm/stages/stage4_mazu_fsm.hpp"

Stage4MazuFSM::Stage4MazuFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx), state_(Stage4State::S4_ENTER), tick_count_(0)
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
