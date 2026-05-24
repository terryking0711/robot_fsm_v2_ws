#include "robot_navigation/navigation_server.hpp"

#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

NavigationServer::NavigationServer()
: Node("navigation_server")
{
  this->declare_parameter<std::string>("named_poses_file", "");
  this->declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
  this->declare_parameter<std::string>("global_frame", "map");

  this->get_parameter("named_poses_file", named_poses_file_);
  this->get_parameter("nav2_action_name", nav2_action_name_);
  this->get_parameter("global_frame", global_frame_);

  load_named_poses();

  nav2_client_ =
    rclcpp_action::create_client<NavigateToPose>(
      this,
      nav2_action_name_);

  action_server_ =
    rclcpp_action::create_server<NavigateToNamedPose>(
      this,
      "navigate_to_named_pose",
      std::bind(&NavigationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigationServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigationServer::handle_accepted, this, std::placeholders::_1)
    );

  RCLCPP_INFO(this->get_logger(), "navigation_server started.");
  RCLCPP_INFO(this->get_logger(), "Nav2 action name: %s", nav2_action_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Global frame: %s", global_frame_.c_str());
}

void NavigationServer::load_named_poses()
{
  if (named_poses_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "named_poses_file is empty.");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading named poses from: %s", named_poses_file_.c_str());

  YAML::Node root = YAML::LoadFile(named_poses_file_);
  YAML::Node poses = root["named_poses"];

  if (!poses) {
    RCLCPP_ERROR(this->get_logger(), "YAML does not contain 'named_poses'.");
    return;
  }

  for (auto it = poses.begin(); it != poses.end(); ++it) {
    std::string name = it->first.as<std::string>();
    YAML::Node p = it->second;

    NamedPose2D pose;
    pose.x = p["x"].as<double>();
    pose.y = p["y"].as<double>();
    pose.yaw = p["yaw"].as<double>();

    named_poses_[name] = pose;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded pose [%s]: x=%.3f, y=%.3f, yaw=%.3f",
      name.c_str(), pose.x, pose.y, pose.yaw);
  }
}

geometry_msgs::msg::PoseStamped NavigationServer::make_pose_stamped(
  const NamedPose2D & pose)
{
  geometry_msgs::msg::PoseStamped msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = global_frame_;

  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  msg.pose.orientation = tf2::toMsg(q);

  return msg;
}

rclcpp_action::GoalResponse NavigationServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToNamedPose::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(
    this->get_logger(),
    "Received NavigateToNamedPose goal: target_name=%s, timeout=%.2f",
    goal->target_name.c_str(),
    goal->timeout_sec);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavigationServer::handle_cancel(
  const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(this->get_logger(), "Received cancel request.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavigationServer::handle_accepted(
  const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle)
{
  std::thread{
    std::bind(&NavigationServer::execute, this, std::placeholders::_1),
    goal_handle
  }.detach();
}

void NavigationServer::execute(
  const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();

  auto result = std::make_shared<NavigateToNamedPose::Result>();
  auto feedback = std::make_shared<NavigateToNamedPose::Feedback>();

  feedback->current_state = "lookup_named_pose";
  feedback->progress = 0.0;
  goal_handle->publish_feedback(feedback);

  if (named_poses_.find(goal->target_name) == named_poses_.end()) {
    result->success = false;
    result->message = "Unknown target_name: " + goal->target_name;
    goal_handle->succeed(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  if (!nav2_client_->wait_for_action_server(5s)) {
    result->success = false;
    result->message = "Nav2 action server not available: " + nav2_action_name_;
    goal_handle->succeed(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = make_pose_stamped(named_poses_[goal->target_name]);

  feedback->current_state = "send_goal_to_nav2";
  feedback->progress = 0.1;
  goal_handle->publish_feedback(feedback);

  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.feedback_callback =
    [goal_handle](
      NavigateToPoseGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> nav_feedback)
    {
      auto fb = std::make_shared<NavigateToNamedPose::Feedback>();

      fb->current_state =
        "navigating, distance_remaining=" +
        std::to_string(nav_feedback->distance_remaining);

      fb->progress = static_cast<float>(nav_feedback->distance_remaining);

      goal_handle->publish_feedback(fb);
    };

  auto nav_goal_handle_future =
    nav2_client_->async_send_goal(nav_goal, send_goal_options);

  if (nav_goal_handle_future.wait_for(30s) != std::future_status::ready) {
    result->success = false;
    result->message = "Timeout while sending goal to Nav2.";
    goal_handle->succeed(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  auto nav_goal_handle = nav_goal_handle_future.get();

  if (!nav_goal_handle) {
    result->success = false;
    result->message = "Nav2 rejected the goal.";
    goal_handle->succeed(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  feedback->current_state = "waiting_nav2_result";
  feedback->progress = 0.2;
  goal_handle->publish_feedback(feedback);

  auto nav_result_future =
    nav2_client_->async_get_result(nav_goal_handle);

  const auto start_time = this->now();
  const double timeout_sec = goal->timeout_sec > 0.0 ? goal->timeout_sec : 60.0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      nav2_client_->async_cancel_goal(nav_goal_handle);

      result->success = false;
      result->message = "NavigateToNamedPose canceled.";
      goal_handle->canceled(result);

      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      return;
    }

    if (nav_result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }

    const double elapsed = (this->now() - start_time).seconds();
    if (elapsed > timeout_sec) {
      nav2_client_->async_cancel_goal(nav_goal_handle);

      result->success = false;
      result->message = "NavigateToNamedPose timeout.";
      goal_handle->succeed(result);

      RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
      return;
    }
  }

  auto nav_result = nav_result_future.get();

  if (nav_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    result->success = true;
    result->message = "Navigation succeeded: " + goal->target_name;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
  } else if (nav_result.code == rclcpp_action::ResultCode::CANCELED) {
    result->success = false;
    result->message = "Navigation canceled by Nav2: " + goal->target_name;
    goal_handle->canceled(result);

    RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
  } else {
    result->success = false;
    result->message = "Navigation failed or aborted: " + goal->target_name;
    goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
  }
}