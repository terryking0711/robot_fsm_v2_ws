#include "robot_fsm/mission/mission_controller.hpp"
#include "robot_fsm/stages/stage2_clam_fsm.hpp"
#include "robot_fsm/stages/stage3_hay_fsm.hpp"
#include "robot_fsm/stages/stage4_mazu_fsm.hpp"
#include "robot_fsm/stages/stage_common.hpp"

MissionController::MissionController(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(MissionState::BOOT),
  nav_goal_sent_(false),
  nav_start_time_(0, 0, RCL_ROS_TIME)
{
  stage2_fsm_ = std::make_shared<Stage2ClamFSM>(ctx_);
  stage3_fsm_ = std::make_shared<Stage3HayFSM>(ctx_);
  stage4_fsm_ = std::make_shared<Stage4MazuFSM>(ctx_);
  stage5_fsm_ = std::make_shared<Stage5NewFSM>(ctx_);
}

bool MissionController::init_system()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] INIT");

  ctx_->mechanism_cmd_pub =
    ctx_->node->create_publisher<robot_interfaces::msg::MechanismCommand>(
      "/mechanism/command", 10);

  return true;
}

bool MissionController::self_check()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] SELF_CHECK");
  return true;
}

bool MissionController::wait_start()
{
  if (ctx_->start_signal) {
    RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] start signal received");
    return true;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] waiting start signal...");
  return false;
}

bool MissionController::leave_start_zone()
{
  return transition_to_named_pose("leave_start_zone", 5.0);
}

// 模擬導航：第一次呼叫記錄起始時間，經過 NAV_SIM_DURATION_S 秒後回傳 true
bool MissionController::transition_to_named_pose(const std::string& target_name, float /*timeout_sec*/)
{
  if (!nav_goal_sent_) {
    RCLCPP_INFO(ctx_->node->get_logger(),
      "[Mission] navigating to '%s' (sim %.0fs)", target_name.c_str(), NAV_SIM_DURATION_S);
    nav_start_time_ = ctx_->node->now();
    nav_goal_sent_ = true;
    return false;
  }

  if ((ctx_->node->now() - nav_start_time_).seconds() >= NAV_SIM_DURATION_S) {
    RCLCPP_INFO(ctx_->node->get_logger(),
      "[Mission] nav to '%s' complete", target_name.c_str());
    nav_goal_sent_ = false;
    return true;
  }

  return false;
}

bool MissionController::finish_decision()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] FINISH_DECISION");
  return true;
}

void MissionController::tick()
{
  switch (state_) {

    case MissionState::BOOT:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] BOOT");
      state_ = MissionState::INIT;
      break;

    case MissionState::INIT:
      if (init_system()) {
        state_ = MissionState::SELF_CHECK;
      } else {
        state_ = MissionState::SAFE_STOP;
      }
      break;

    case MissionState::SELF_CHECK:
      if (self_check()) {
        state_ = MissionState::WAIT_START;
      } else {
        state_ = MissionState::SAFE_STOP;
      }
      break;

    case MissionState::WAIT_START:
      if (wait_start()) {
        state_ = MissionState::LEAVE_START_ZONE;
      }
      break;

    case MissionState::LEAVE_START_ZONE:
      if (leave_start_zone()) {
        state_ = MissionState::STAGE2_CLAM;
      }
      break;

    case MissionState::STAGE2_CLAM:
      if (stage2_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE3;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE3:
      if (transition_to_named_pose("stage3_entry", 5.0)) {
        state_ = MissionState::STAGE3_HAY;
      }
      break;

    case MissionState::STAGE3_HAY:
      if (stage3_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE4;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE4:
      if (transition_to_named_pose("stage4_entry", 5.0)) {
        state_ = MissionState::STAGE4_MAZU;
      }
      break;

    case MissionState::STAGE4_MAZU:
      if (stage4_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE5;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE5:
      if (transition_to_named_pose("stage5_entry", 5.0)) {
        state_ = MissionState::STAGE5_NEW;
      }
      break;

    case MissionState::STAGE5_NEW:
      if (stage5_fsm_->tick() == StageStatus::SUCCESS) {
        state_ = MissionState::FINISH_DECISION;
      }
      break;

    case MissionState::FINISH_DECISION:
      if (finish_decision()) {
        state_ = MissionState::END_RUN;
      } else {
        state_ = MissionState::EARLY_STOP;
      }
      break;

    case MissionState::EARLY_STOP:
      RCLCPP_WARN(ctx_->node->get_logger(), "[Mission] EARLY_STOP");
      state_ = MissionState::END_RUN;
      break;

    case MissionState::END_RUN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] END_RUN - 全部任務完成！");
      rclcpp::shutdown();
      break;

    case MissionState::SAFE_STOP:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] SAFE_STOP");
      break;
  }
}
