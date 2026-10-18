#include "robot_navigation/navigation_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace
{

// ---- fine_align（Nav2 到點後的精對位）----
// 底盤是 Omni（麥輪）：x / y / yaw 三軸各自 P 控制，同時修。
// 微調輸出發到 cmd_vel_nav（velocity_smoother 的輸入）。
// 取消／逾時時額外在 /cmd_vel 發零，等 Nav2 結束並排空 smoother 後才放行 FSM。
// 最低速度要高於 velocity_smoother 的 deadband_velocity [0.02, 0.02, 0.01]，否則會被吃成 0。
constexpr char kFineCmdTopic[] = "/cmd_vel_nav";
constexpr char kBaseFrame[] = "base_footprint";
constexpr double kFineXyTol = 0.01;        // m
constexpr double kFineYawTol = 0.015;      // rad（約 0.86°）
constexpr double kFineKpXy = 1.2;          // (m/s) / m
constexpr double kFineKpYaw = 1.5;         // (rad/s) / rad
constexpr double kFineMaxV = 0.10;         // m/s
constexpr double kFineMinV = 0.03;         // m/s
constexpr double kFineMaxW = 0.30;         // rad/s
constexpr double kFineMinW = 0.03;         // rad/s
constexpr int kFineSettleCycles = 4;       // 連續幾個週期都在容許內才算到（20 Hz => 0.2 s）
constexpr double kFineTfMaxAgeSec = 1.0;   // 過期定位停止精對位，保留 Nav2 結果
constexpr double kFineTimeoutSec = 3.0;    // 超時就停車，仍回報成功（Nav2 本身已到點）

// |err| <= tol 輸出 0；否則 kp*err，絕對值夾在 [min_v, max_v]。
double fine_axis_cmd(double err, double tol, double kp, double min_v, double max_v)
{
  if (std::abs(err) <= tol) {
    return 0.0;
  }
  const double v = std::clamp(std::abs(kp * err), min_v, max_v);
  return std::copysign(v, err);
}

// named pose 有兩種寫法：
//   絕對：{x, y, yaw}（world frame）
//   相對：{relative_to: <anchor>, dx, dy, dyaw}
//         dx / dy = world frame 的 x / y 偏移（m，不隨 anchor 的 yaw 旋轉），dyaw = 相對 anchor 的 yaw（rad，可省略 = 0）
// 兩種都寫時以 relative_to 為準。anchor 本身也可以是相對點，depth 限制擋循環引用。
// ⚠ robot_fsm/src/stages/stage3_hay_fsm.cpp 有同樣的解析，改格式要兩邊一起改。
bool resolve_named_pose(
  const YAML::Node & poses, const std::string & name,
  NamedPose2D & out, std::string & err, int depth = 0)
{
  if (depth > 8) {
    err = "relative_to chain too deep (circular?) at '" + name + "'";
    return false;
  }

  const YAML::Node p = poses[name];
  if (!p || !p.IsMap()) {
    err = "pose '" + name + "' not found";
    return false;
  }

  if (p["relative_to"]) {
    if (!p["dx"] || !p["dy"]) {
      err = "relative pose '" + name + "' missing dx/dy";
      return false;
    }
    NamedPose2D anchor;
    if (!resolve_named_pose(poses, p["relative_to"].as<std::string>(), anchor, err, depth + 1)) {
      return false;
    }
    const double dx = p["dx"].as<double>();
    const double dy = p["dy"].as<double>();
    const double dyaw = p["dyaw"] ? p["dyaw"].as<double>() : 0.0;
    // x_relative_to 可讓轉角的 x 跟隨另一個點；y/yaw 仍使用 relative_to。
    NamedPose2D x_anchor = anchor;
    if (p["x_relative_to"] &&
      !resolve_named_pose(poses, p["x_relative_to"].as<std::string>(), x_anchor, err, depth + 1))
    {
      return false;
    }
    out.x = x_anchor.x + dx;
    out.y = anchor.y + dy;
    out.yaw = std::atan2(std::sin(anchor.yaw + dyaw), std::cos(anchor.yaw + dyaw));
    return true;
  }

  if (!p["x"] || !p["y"] || !p["yaw"]) {
    err = "pose '" + name + "' missing x/y/yaw";
    return false;
  }
  out.x = p["x"].as<double>();
  out.y = p["y"].as<double>();
  out.yaw = p["yaw"].as<double>();
  return true;
}

}  // namespace

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

  stop_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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

  // 手動修改 yaml 後，呼叫此 service 即可熱重載，
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

  fine_cmd_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(kFineCmdTopic, 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

      NamedPose2D pose;
      std::string err;
      if (!resolve_named_pose(poses, name, pose, err)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Pose [%s] skipped: %s", name.c_str(), err.c_str());
        continue;
      }

      // precise 只看這個點自己，不從 relative_to 的 anchor 繼承。
      pose.precise = it->second["precise"] && it->second["precise"].as<bool>();

      new_poses[name] = pose;

      RCLCPP_INFO(
        this->get_logger(),
        "Loaded pose [%s]%s%s: x=%.3f, y=%.3f, yaw=%.3f",
        name.c_str(), it->second["relative_to"] ? " (relative)" : "",
        pose.precise ? " (precise)" : "",
        pose.x, pose.y, pose.yaw);
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

void NavigationServer::hold_stop()
{
  rclcpp::WallRate rate(20.0);
  const auto end = std::chrono::steady_clock::now() + 1200ms;
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    fine_cmd_pub_->publish(geometry_msgs::msg::Twist());
    stop_cmd_pub_->publish(geometry_msgs::msg::Twist());
    rate.sleep();
  }
}

void NavigationServer::cancel_nav2_and_stop(
  const NavigateToPoseGoalHandle::SharedPtr & nav_goal_handle)
{
  fine_cmd_pub_->publish(geometry_msgs::msg::Twist());
  stop_cmd_pub_->publish(geometry_msgs::msg::Twist());
  auto terminal = nav2_client_->async_get_result(nav_goal_handle);
  auto cancel = nav2_client_->async_cancel_goal(nav_goal_handle);
  rclcpp::WallRate rate(20.0);
  while (rclcpp::ok() && terminal.wait_for(0ms) != std::future_status::ready) {
    // Do not release the FSM while Nav2 can still command motion.
    fine_cmd_pub_->publish(geometry_msgs::msg::Twist());
    stop_cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for Nav2 to terminate after cancellation; holding zero velocity");
    if (cancel.valid() && cancel.wait_for(0ms) == std::future_status::ready) {
      if (cancel.get()->goals_canceling.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Nav2 did not accept cancellation; keeping stop active");
      }
    }
    rate.sleep();
  }
  hold_stop();
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

  const auto start_time = std::chrono::steady_clock::now();
  const double timeout_sec =
    goal->timeout_sec > 0.0 ? goal->timeout_sec : default_timeout_sec_;
  const auto deadline = start_time +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      cancel_nav2_and_stop(nav_goal_handle);

      result->success = false;
      result->message = "NavigateToNamedPose canceled.";
      goal_handle->canceled(result);

      RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
      return;
    }

    if (nav_result_future.wait_for(100ms) == std::future_status::ready) {
      break;
    }

    if (std::chrono::steady_clock::now() >= deadline) {
      cancel_nav2_and_stop(nav_goal_handle);

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
    bool fine_converged = true;
    if (named_pose->precise) {
      const bool converged = fine_align(goal_handle, *named_pose, deadline);
      fine_converged = converged;

      if (goal_handle->is_canceling()) {
        hold_stop();
        result->success = false;
        result->message = "NavigateToNamedPose canceled during fine_align.";
        goal_handle->canceled(result);

        RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
        return;
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        hold_stop();
        result->success = false;
        result->message = "NavigateToNamedPose timeout after " +
          std::to_string(timeout_sec) + " s: " + goal->target_name;
        goal_handle->abort(result);
        return;
      }

      if (!converged) {
        RCLCPP_WARN(
          this->get_logger(),
          "fine_align did not converge for '%s', keep Nav2 result.",
          goal->target_name.c_str());
      }
    }

    result->success = true;
    result->message = "Navigation succeeded: " + goal->target_name;
    if (named_pose->precise && !fine_converged) {
      result->message += " (Nav2 succeeded; fine_align not converged)";
    }
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

bool NavigationServer::fine_align(
  const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle,
  const NamedPose2D & target,
  const std::chrono::steady_clock::time_point & deadline)
{
  auto feedback = std::make_shared<NavigateToNamedPose::Feedback>();
  feedback->progress = 0.99;

  const auto start_time = this->now();
  int settled = 0;
  bool converged = false;
  double ex = 0.0, ey = 0.0, eyaw = 0.0;
  bool have_error = false;
  const char* stop_reason = "shutdown_or_cancel";

  rclcpp::Rate rate(20.0);
  while (rclcpp::ok() && !goal_handle->is_canceling()) {
    if (std::chrono::steady_clock::now() >= deadline ||
      (this->now() - start_time).seconds() > kFineTimeoutSec) {
      stop_reason = "timeout";
      break;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(global_frame_, kBaseFrame, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(), "fine_align TF lookup failed: %s", e.what());
      stop_reason = "tf_unavailable";
      break;
    }

    const double tf_age = (this->now() - rclcpp::Time(tf.header.stamp)).seconds();
    if (tf_age > kFineTfMaxAgeSec) {
      stop_reason = "tf_stale";
      RCLCPP_WARN(this->get_logger(), "fine_align TF stale: %.3f s", tf_age);
      break;
    }
    const double yaw = tf2::getYaw(tf.transform.rotation);
    ex = target.x - tf.transform.translation.x;
    ey = target.y - tf.transform.translation.y;
    eyaw = std::atan2(std::sin(target.yaw - yaw), std::cos(target.yaw - yaw));

    if (!std::isfinite(ex) || !std::isfinite(ey) || !std::isfinite(eyaw)) {
      stop_reason = "invalid_pose";
      break;
    }
    if (!have_error) {
      RCLCPP_INFO(this->get_logger(),
        "fine_align initial error: ex=%.4f ey=%.4f eyaw=%.4f", ex, ey, eyaw);
      have_error = true;
    }

    if (std::hypot(ex, ey) <= kFineXyTol && std::abs(eyaw) <= kFineYawTol) {
      fine_cmd_pub_->publish(geometry_msgs::msg::Twist());
      if (++settled >= kFineSettleCycles) {
        converged = true;
        stop_reason = "converged";
        break;
      }
      rate.sleep();
      continue;
    } else {
      settled = 0;
    }

    // world 誤差轉到車體座標
    const double bx = std::cos(yaw) * ex + std::sin(yaw) * ey;
    const double by = -std::sin(yaw) * ex + std::cos(yaw) * ey;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = fine_axis_cmd(bx, kFineXyTol, kFineKpXy, kFineMinV, kFineMaxV);
    cmd.linear.y = fine_axis_cmd(by, kFineXyTol, kFineKpXy, kFineMinV, kFineMaxV);
    cmd.angular.z = fine_axis_cmd(eyaw, kFineYawTol, kFineKpYaw, kFineMinW, kFineMaxW);
    fine_cmd_pub_->publish(cmd);

    feedback->current_state =
      "fine_align ex=" + std::to_string(ex) + " ey=" + std::to_string(ey) +
      " eyaw=" + std::to_string(eyaw);
    goal_handle->publish_feedback(feedback);

    rate.sleep();
  }

  fine_cmd_pub_->publish(geometry_msgs::msg::Twist());

  RCLCPP_INFO(
    this->get_logger(),
    "fine_align %s in %.2f s: ex=%.4f ey=%.4f eyaw=%.4f reason=%s settled=%d/%d error_valid=%s",
    converged ? "converged" : "stopped",
    (this->now() - start_time).seconds(), ex, ey, eyaw,
    stop_reason, settled, kFineSettleCycles, have_error ? "true" : "false");

  return converged;
}
