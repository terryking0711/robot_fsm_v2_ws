#include "robot_fsm/stages/stage3_hay_fsm.hpp"

Stage3HayFSM::Stage3HayFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx), state_(Stage3State::S3_ENTER), tick_count_(0)
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

bool Stage3HayFSM::tick()
{
  switch (state_) {

    case Stage3State::S3_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] ENTER 第三關");
      state_ = Stage3State::S3_LOCALIZE;
      return false;

    case Stage3State::S3_LOCALIZE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] LOCALIZE 模擬定位");
      if (wait_ticks(5)) state_ = Stage3State::S3_SCAN_HAY;
      return false;

    case Stage3State::S3_SCAN_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] SCAN_HAY 模擬視覺掃描");
      if (wait_ticks(5)) state_ = Stage3State::S3_PLAN_STACK;
      return false;

    case Stage3State::S3_PLAN_STACK:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLAN_STACK 模擬規劃堆疊");
      if (wait_ticks(5)) state_ = Stage3State::S3_SELECT_TARGET;
      return false;

    case Stage3State::S3_SELECT_TARGET:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] SELECT_TARGET 模擬選擇目標");
      state_ = Stage3State::S3_NAV_TO_PICK;
      return false;

    case Stage3State::S3_NAV_TO_PICK:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_PICK 模擬導航到抓取點");
      if (wait_ticks(50)) state_ = Stage3State::S3_PICK_HAY;
      return false;

    case Stage3State::S3_PICK_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PICK_HAY 模擬抓取稻草卷");
      if (wait_ticks(10)) state_ = Stage3State::S3_NAV_TO_STACK;
      return false;

    case Stage3State::S3_NAV_TO_STACK:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_STACK 模擬導航到堆疊點");
      if (wait_ticks(50)) state_ = Stage3State::S3_PLACE_HAY;
      return false;

    case Stage3State::S3_PLACE_HAY:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLACE_HAY 模擬放置稻草卷");
      if (wait_ticks(10)) state_ = Stage3State::S3_VERIFY_STABLE;
      return false;

    case Stage3State::S3_VERIFY_STABLE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] VERIFY_STABLE 模擬確認穩定");
      if (wait_ticks(5)) state_ = Stage3State::S3_DONE;
      return false;

    case Stage3State::S3_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] DONE 第三關完成");
      return true;

    case Stage3State::S3_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3] FAILED");
      return false;
  }

  return false;
}
