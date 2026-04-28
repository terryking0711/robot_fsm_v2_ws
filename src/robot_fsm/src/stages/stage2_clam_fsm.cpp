#include "robot_fsm/stages/stage2_clam_fsm.hpp"

Stage2ClamFSM::Stage2ClamFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx), state_(Stage2State::S2_ENTER), tick_count_(0)
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

bool Stage2ClamFSM::tick()
{
  switch (state_) {

    case Stage2State::S2_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] ENTER 第二關");
      state_ = Stage2State::S2_APPROACH;
      return false;

    case Stage2State::S2_APPROACH:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] APPROACH 模擬靠近蛤蜊");
      if (wait_ticks(10)) state_ = Stage2State::S2_PICK_CLAM;
      return false;

    case Stage2State::S2_PICK_CLAM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] PICK_CLAM 模擬抓取蛤蜊");
      if (wait_ticks(10)) state_ = Stage2State::S2_RETURN;
      return false;

    case Stage2State::S2_RETURN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] RETURN 模擬返回");
      if (wait_ticks(10)) state_ = Stage2State::S2_DONE;
      return false;

    case Stage2State::S2_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage2] DONE 第二關完成");
      return true;

    case Stage2State::S2_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage2] FAILED");
      return false;
  }

  return false;
}
