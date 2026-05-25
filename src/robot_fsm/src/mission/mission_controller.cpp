#include "robot_fsm/mission/mission_controller.hpp"

#include "robot_fsm/stages/stage1_wetland_fsm.hpp"
#include "robot_fsm/stages/stage2_clam_fsm.hpp"
#include "robot_fsm/stages/stage3_hay_fsm.hpp"

MissionController::MissionController(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(MissionState::BOOT),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_("")
{
  stage1_fsm_ = std::make_shared<Stage1WetlandFSM>(ctx_);
  stage2_fsm_ = std::make_shared<Stage2ClamFSM>(ctx_);
  stage3_fsm_ = std::make_shared<Stage3HayFSM>(ctx_);
}

bool MissionController::init_system()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] INIT");

  ctx_->mechanism_cmd_pub =
    ctx_->node->create_publisher<robot_interfaces::msg::MechanismCommand>(
      "/mechanism/command", 10);

  ctx_->nav_client =
    rclcpp_action::create_client<robot_interfaces::action::NavigateToNamedPose>(
      ctx_->node,
      "navigate_to_named_pose");

  return true;
}

bool MissionController::self_check()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] SELF_CHECK");

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] nav_client is null");
    return false;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] self check passed");
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
  return transition_to_named_pose("leave_start_zone", 20.0);
}

bool MissionController::transition_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] nav_client not initialized");
    return false;
  }

  if (!nav_goal_sent_) {
    if (!ctx_->nav_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        ctx_->node->get_logger(),
        "[Mission] /navigate_to_named_pose server not ready, waiting...");
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
      "[Mission] send navigation goal: %s",
      target_name.c_str());

    auto options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][NavFeedback] state=%s progress=%.2f",
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
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          nav_success_ = false;
          nav_message_ = "navigation goal canceled";
        } else {
          nav_success_ = false;
          nav_message_ = "navigation goal aborted";
        }

        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][NavResult] success=%s message=%s",
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
      "[Mission] navigation to '%s' complete",
      nav_active_target_.c_str());
  } else {
    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Mission] navigation to '%s' failed: %s",
      nav_active_target_.c_str(),
      nav_message_.c_str());
  }

  nav_goal_sent_ = false;
  nav_done_ = false;
  nav_success_ = false;
  nav_message_.clear();
  nav_active_target_.clear();

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
        state_ = MissionState::STAGE1_WETLAND;
      }
      break;

    case MissionState::STAGE1_WETLAND:
      if (stage1_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE2;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE2:
      if (transition_to_named_pose("stage2_entry", 30.0)) {
        state_ = MissionState::STAGE2_CLAM;
      }
      break;

    case MissionState::STAGE2_CLAM:
      if (stage2_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE3;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE3:
      if (transition_to_named_pose("stage3_entry", 50.0)) {
        state_ = MissionState::STAGE3_HAY;
      }
      break;

    case MissionState::STAGE3_HAY:
      if (stage3_fsm_->tick()) {
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
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] END_RUN - 起點到第三關展示完成！");
      rclcpp::shutdown();
      break;

    case MissionState::SAFE_STOP:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] SAFE_STOP");
      break;
  }
}