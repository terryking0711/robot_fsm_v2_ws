#include "robot_fsm/stages/stage3_hay_fsm.hpp"

Stage3HayFSM::(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage3State::S3_ENTER),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  mechanism_waiting_(false)
{
}

void Stage3HayFSM::publish_mechanism_command(uint16_t id,
                                             const std::string& name,
                                             const std::string& arg_json)
{
  robot_interfaces::msg::MechanismCommand msg;
  msg.command_id = id;
  msg.command_name = name;
  msg.arg_json = arg_json;
  ctx_->mechanism_cmd_pub->publish(msg);
}

bool Stage3HayFSM::tick()
{
  switch (state_) {

    case Stage3State::S3_ENTER:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] ENTER");
      state_ = Stage3State::S3_LOCALIZE;
      return false;

    case Stage3State::S3_LOCALIZE:
      if (!ctx_->latest_final_pose.has_value()) {
        RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] waiting /final_pose");
        return false;
      }
      state_ = Stage3State::S3_SCAN_HAY;
      return false;

    case Stage3State::S3_SCAN_HAY:
      if (!ctx_->latest_vision_state.has_value()) {
        RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] waiting vision");
        return false;
      }
      state_ = Stage3State::S3_PLAN_STACK;
      return false;

    case Stage3State::S3_PLAN_STACK:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLAN_STACK");
      state_ = Stage3State::S3_SELECT_TARGET;
      return false;

    case Stage3State::S3_SELECT_TARGET:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] SELECT_TARGET");
      state_ = Stage3State::S3_NAV_TO_PICK;
      nav_goal_sent_ = false;
      nav_done_ = false;
      nav_success_ = false;
      return false;

    case Stage3State::S3_NAV_TO_PICK:
      if (!nav_goal_sent_) {
        if (!ctx_->nav_client->wait_for_action_server(std::chrono::seconds(1))) {
          RCLCPP_WARN(ctx_->node->get_logger(), "[Stage3] nav server not ready");
          return false;
        }

        robot_interfaces::action::NavigateToNamedPose::Goal goal;
        goal.target_name = "stage3_pick_pose";
        goal.timeout_sec = 10.0;

        auto options =
          rclcpp_action::Client<robot_interfaces::action::NavigateToNamedPose>::SendGoalOptions();

        options.result_callback =
          [this](const auto& result) {
            nav_done_ = true;
            nav_success_ = result.result->success;
          };

        ctx_->nav_client->async_send_goal(goal, options);
        nav_goal_sent_ = true;
        return false;
      }

      if (!nav_done_) {
        return false;
      }

      if (!nav_success_) {
        state_ = Stage3State::S3_FAILED;
        return false;
      }

      state_ = Stage3State::S3_PICK_HAY;
      mechanism_waiting_ = false;
      return false;

    case Stage3State::S3_PICK_HAY:
      if (!mechanism_waiting_) {
        publish_mechanism_command(1, "pick_hay", R"({"target":"hay_roll","index":0})");
        mechanism_waiting_ = true;
        return false;
      }

      if (!ctx_->latest_mechanism_feedback.has_value()) {
        return false;
      }

      if (ctx_->latest_mechanism_feedback->done &&
          ctx_->latest_mechanism_feedback->success) {
        state_ = Stage3State::S3_NAV_TO_STACK;
        nav_goal_sent_ = false;
        nav_done_ = false;
        nav_success_ = false;
        return false;
      }

      return false;

    case Stage3State::S3_NAV_TO_STACK:
      if (!nav_goal_sent_) {
        robot_interfaces::action::NavigateToNamedPose::Goal goal;
        goal.target_name = "stage3_stack_pose";
        goal.timeout_sec = 10.0;

        auto options =
          rclcpp_action::Client<robot_interfaces::action::NavigateToNamedPose>::SendGoalOptions();

        options.result_callback =
          [this](const auto& result) {
            nav_done_ = true;
            nav_success_ = result.result->success;
          };

        ctx_->nav_client->async_send_goal(goal, options);
        nav_goal_sent_ = true;
        return false;
      }

      if (!nav_done_) {
        return false;
      }

      if (!nav_success_) {
        state_ = Stage3State::S3_FAILED;
        return false;
      }

      state_ = Stage3State::S3_PLACE_HAY;
      mechanism_waiting_ = false;
      return false;

    case Stage3State::S3_PLACE_HAY:
      if (!mechanism_waiting_) {
        publish_mechanism_command(2, "place_hay", R"({"slot":"layer1_slot1"})");
        mechanism_waiting_ = true;
        return false;
      }

      if (!ctx_->latest_mechanism_feedback.has_value()) {
        return false;
      }

      if (ctx_->latest_mechanism_feedback->done &&
          ctx_->latest_mechanism_feedback->success) {
        state_ = Stage3State::S3_VERIFY_STABLE;
        return false;
      }

      return false;

    case Stage3State::S3_VERIFY_STABLE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] VERIFY_STABLE");
      state_ = Stage3State::S3_DONE;
      return false;

    case Stage3State::S3_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] DONE");
      return true;

    case Stage3State::S3_FAILED:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3] FAILED");
      return false;
  }

  return false;
}