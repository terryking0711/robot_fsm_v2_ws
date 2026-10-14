#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_interfaces/action/navigate_to_named_pose.hpp"

struct NamedPose2D
{
  double x;
  double y;
  double yaw;
  bool precise = false;  // yaml 寫 precise: true => Nav2 到點後再跑 fine_align
};

class NavigationServer : public rclcpp::Node
{
public:
  using NavigateToNamedPose = robot_interfaces::action::NavigateToNamedPose;
  using NavigateToNamedPoseGoalHandle =
    rclcpp_action::ServerGoalHandle<NavigateToNamedPose>;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateToPoseGoalHandle =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigationServer();

private:
  // Returns true if the YAML file was loaded successfully.
  bool load_named_poses();

  // Thread-safe lookup. Returns std::nullopt when the name is unknown.
  std::optional<NamedPose2D> find_named_pose(const std::string & name);

  geometry_msgs::msg::PoseStamped make_pose_stamped(
    const NamedPose2D & pose);

  void handle_reload_named_poses(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToNamedPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  void execute(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  // Nav2 成功後的精對位（閉迴路 P 控制，直接發 cmd_vel_nav）。
  // 回傳 true = x / y / yaw 都收斂到容許誤差內。
  bool fine_align(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle,
    const NamedPose2D & target);

  rclcpp_action::Server<NavigateToNamedPose>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr fine_cmd_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string named_poses_file_;
  std::string nav2_action_name_;
  std::string global_frame_;
  double server_wait_sec_;
  double default_timeout_sec_;

  std::mutex poses_mutex_;
  std::map<std::string, NamedPose2D> named_poses_;
};