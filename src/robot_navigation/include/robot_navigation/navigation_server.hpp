#pragma once

#include <map>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "robot_interfaces/action/navigate_to_named_pose.hpp"

struct NamedPose2D
{
  double x;
  double y;
  double yaw;
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
  void load_named_poses();

  geometry_msgs::msg::PoseStamped make_pose_stamped(
    const NamedPose2D & pose);

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToNamedPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  void handle_accepted(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  void execute(
    const std::shared_ptr<NavigateToNamedPoseGoalHandle> goal_handle);

  rclcpp_action::Server<NavigateToNamedPose>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

  std::string named_poses_file_;
  std::string nav2_action_name_;
  std::string global_frame_;

  std::map<std::string, NamedPose2D> named_poses_;
};