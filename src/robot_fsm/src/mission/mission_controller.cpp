#include "robot_fsm/mission/mission_controller.hpp"
#include "robot_fsm/stages/stage3_hay_fsm.hpp"

// 任務控制器的實作
MissionController::MissionController(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(MissionState::BOOT),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false)
{
  stage3_fsm_ = std::make_shared<Stage3HayFSM>(ctx_);
}

// 任務控制器的主循環，根據當前State執行對應行為
bool MissionController::init_system()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] INIT");

  ctx_->nav_client =
    rclcpp_action::create_client<robot_interfaces::action::NavigateToNamedPose>(
      ctx_->node, "navigate_to_named_pose");

  ctx_->mechanism_cmd_pub =
    ctx_->node->create_publisher<robot_interfaces::msg::MechanismCommand>(
      "/mechanism/command", 10);

  return true;
}

bool MissionController::self_check()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] SELF_CHECK");

  // 初版先簡化成永遠成功
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
  return transition_to_named_pose("leave_start_zone", 10.0);
}

bool MissionController::transition_to_named_pose(const std::string& target_name, float timeout_sec)
{
  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] nav client not ready");
    return false;
  }

  if (!nav_goal_sent_) {
    if (!ctx_->nav_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(ctx_->node->get_logger(), "[Mission] nav server not ready");
      return false;
    }

    robot_interfaces::action::NavigateToNamedPose::Goal goal;
    goal.target_name = target_name;
    goal.timeout_sec = timeout_sec;

    auto options =
      rclcpp_action::Client<robot_interfaces::action::NavigateToNamedPose>::SendGoalOptions();

    options.result_callback =
      [this](const auto& result) {
        nav_done_ = true;
        nav_success_ = result.result->success;
      };

    nav_goal_sent_ = true;
    nav_done_ = false;
    nav_success_ = false;
    ctx_->nav_client->async_send_goal(goal, options);

    return false;
  }

  if (!nav_done_) {
    return false;
  }

  bool result = nav_success_;
  nav_goal_sent_ = false;
  nav_done_ = false;
  nav_success_ = false;
  return result;
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
        state_ = MissionState::STAGE3_HAY;
      }
      break;

    case MissionState::STAGE3_HAY:
      if (stage3_fsm_->tick()) {
        state_ = MissionState::TRANSITION_3_TO_2;
      }
      break;

    case MissionState::TRANSITION_3_TO_2:
      if (transition_to_named_pose("stage2_entry", 15.0)) {
        state_ = MissionState::STAGE2_CLAM;
      }
      break;

    case MissionState::STAGE2_CLAM:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] STAGE2_CLAM placeholder");
      state_ = MissionState::TRANSITION_2_TO_4;
      break;

    case MissionState::TRANSITION_2_TO_4:
      if (transition_to_named_pose("stage4_entry", 15.0)) {
        state_ = MissionState::STAGE4_MAZU;
      }
      break;

    case MissionState::STAGE4_MAZU:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] STAGE4_MAZU placeholder");
      state_ = MissionState::TRANSITION_4_TO_1;
      break;

    case MissionState::TRANSITION_4_TO_1:
      if (transition_to_named_pose("stage1_entry", 15.0)) {
        state_ = MissionState::STAGE1_WETLAND;
      }
      break;

    case MissionState::STAGE1_WETLAND:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] STAGE1_WETLAND placeholder");
      state_ = MissionState::FINISH_DECISION;
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
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] END_RUN");
      break;

    case MissionState::SAFE_STOP:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] SAFE_STOP");
      break;
  }
}