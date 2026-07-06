#include "robot_navigation/navigation_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// ============================================================================
// 對齊 tdk_slam_ws real-robot 的 Nav2 設定：
//   * global_frame = "map"（Cartographer pure localization 提供 map->odom）
//   * robot_base_frame = "base_footprint"
//   * Nav2 action = /navigate_to_pose（bt_navigator）
//   * use_sim_time = false（由 launch 傳入，本節點不 hardcode）
//   * goal tolerance 由 nav2 的 general_goal_checker 決定
//     （xy 0.04 m / yaw 0.07 rad），named pose 只需給出目標點即可。
// ============================================================================

NavigationServer::NavigationServer()
: Node("navigation_server")
{
  this->declare_parameter<std::string>("named_poses_file", "");
  this->declare_parameter<std::string>("nav2_action_name", "/navigate_to_pose");
  this->declare_parameter<std::string>("global_frame", "map");
  // Nav2 lifecycle 全部 active 需要數秒，開太短會誤判 server 不在。
  this->declare_parameter<double>("server_wait_sec", 10.0);
  // FSM 沒帶 timeout（<=0）時的預設導航逾時。
  this->declare_parameter<double>("default_timeout_sec", 60.0);

  this->get_parameter("named_poses_file", named_poses_file_);
  this->get_parameter("nav2_action_name", nav2_action_name_);
  this->get_parameter("global_frame", global_frame_);
  this->get_parameter("server_wait_sec", server_wait_sec_);
  this->get_parameter("default_timeout_sec", default_timeout_sec_);

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

  // 用 named_pose_recorder 更新 yaml 後，呼叫此 service 即可熱重載，
  // 不需要重啟 navigation_server：
  //   ros2 service call /reload_named_poses std_srvs/srv/Trigger
  reload_srv_ =
    this->create_service<std_srvs::srv::Trigger>(
      "reload_named_poses",
      std::bind(
        &NavigationServer::handle_reload_named_poses,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "navigation_server started.");
  RCLCPP_INFO(this->get_logger(), "Nav2 action name: %s", nav2_action_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Global frame: %s", global_frame_.c_str());
}

bool NavigationServer::load_named_poses()
{
  if (named_poses_file_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "named_poses_file is empty.");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading named poses from: %s", named_poses_file_.c_str());

  std::map<std::string, NamedPose2D> new_poses;

  try {
    YAML::Node root = YAML::LoadFile(named_poses_file_);
    YAML::Node poses = root["named_poses"];

    if (!poses || !poses.IsMap()) {
      RCLCPP_ERROR(this->get_logger(), "YAML does not contain a 'named_poses' map.");
      return false;
    }

    for (auto it = poses.begin(); it != poses.end(); ++it) {
      std::string name = it->first.as<std::string>();
      YAML::Node p = it->second;

      if (!p["x"] || !p["y"] || !p["yaw"]) {
        RCLCPP_WARN(
          this->get_logger(),
          "Pose [%s] missing x/y/yaw, skipped.", name.c_str());
        continue;
      }

      NamedPose2D pose;
      pose.x = p["x"].as<double>();
      pose.y = p["y"].as<double>();
      pose.yaw = p["yaw"].as<double>();

      new_poses[name] = pose;

      RCLCPP_INFO(
        this->get_logger(),
        "Loaded pose [%s]: x=%.3f, y=%.3f, yaw=%.3f",
        name.c_str(), pose.x, pose.y, pose.yaw);
    }
  } catch (const std::exception & e) {
    // 檔案不存在或格式錯誤時不要讓整個節點 crash。
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load named poses: %s", e.what());
    return false;
  }

  {
    std::scoped_lock lock(poses_mutex_);
    named_poses_ = std::move(new_poses);
  }

  return true;
}

std::optional<NamedPose2D> NavigationServer::find_named_pose(const std::string & name)
{
  std::scoped_lock lock(poses_mutex_);
  auto it = named_poses_.find(name);
  if (it == named_poses_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void NavigationServer::handle_reload_named_poses(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  const bool ok = load_named_poses();
  response->success = ok;
  response->message = ok
    ? "named poses reloaded from " + named_poses_file_
    : "failed to reload named poses, see navigation_server log";
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

  // 未知的 pose name 直接 REJECT，FSM 端會立刻收到 goal_response 失敗，
  // 不會浪費一輪 execute。
  if (!find_named_pose(goal->target_name)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unknown target_name '%s', rejecting goal. "
      "Check named_poses.yaml or call /reload_named_poses.",
      goal->target_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

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

  // handle_goal 已檢查過，但 reload 可能發生在 accept 與 execute 之間，
  // 這裡再取一次以保證一致性。
  auto named_pose = find_named_pose(goal->target_name);
  if (!named_pose) {
    result->success = false;
    result->message = "Unknown target_name: " + goal->target_name;
    goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  const auto server_wait =
    std::chrono::duration<double>(server_wait_sec_);
  if (!nav2_client_->wait_for_action_server(server_wait)) {
    result->success = false;
    result->message = "Nav2 action server not available: " + nav2_action_name_;
    goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = make_pose_stamped(*named_pose);
  // behavior_tree 留空 => 使用 bt_navigator 的
  // default_bt_xml_filename（navigate_w_replanning_and_recovery.xml）。

  feedback->current_state = "send_goal_to_nav2";
  feedback->progress = 0.05;
  goal_handle->publish_feedback(feedback);

  auto send_goal_options =
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // 用第一筆 distance_remaining 當作全程距離來估計 0~1 的 progress，
  // 對 FSM 的 log 比直接丟 distance 好讀。
  auto initial_distance = std::make_shared<float>(-1.0f);

  send_goal_options.feedback_callback =
    [goal_handle, initial_distance](
      NavigateToPoseGoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> nav_feedback)
    {
      auto fb = std::make_shared<NavigateToNamedPose::Feedback>();

      const float dist = nav_feedback->distance_remaining;

      if (*initial_distance <= 0.0f && dist > 0.0f) {
        *initial_distance = dist;
      }

      float progress = 0.1f;
      if (*initial_distance > 0.0f) {
        progress = 1.0f - dist / *initial_distance;
        progress = std::min(std::max(progress, 0.0f), 0.99f);
      }

      fb->current_state =
        "navigating dist=" + std::to_string(dist) +
        " recoveries=" + std::to_string(nav_feedback->number_of_recoveries);
      fb->progress = progress;

      goal_handle->publish_feedback(fb);
    };

  auto nav_goal_handle_future =
    nav2_client_->async_send_goal(nav_goal, send_goal_options);

  if (nav_goal_handle_future.wait_for(10s) != std::future_status::ready) {
    result->success = false;
    result->message = "Timeout while sending goal to Nav2.";
    goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  auto nav_goal_handle = nav_goal_handle_future.get();

  if (!nav_goal_handle) {
    result->success = false;
    result->message = "Nav2 rejected the goal.";
    goal_handle->abort(result);

    RCLCPP_ERROR(this->get_logger(), "%s", result->message.c_str());
    return;
  }

  feedback->current_state = "waiting_nav2_result";
  feedback->progress = 0.1;
  goal_handle->publish_feedback(feedback);

  auto nav_result_future =
    nav2_client_->async_get_result(nav_goal_handle);

  const auto start_time = this->now();
  const double timeout_sec =
    goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;

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
      result->message =
        "NavigateToNamedPose timeout after " +
        std::to_string(timeout_sec) + " s: " + goal->target_name;
      goal_handle->abort(result);

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