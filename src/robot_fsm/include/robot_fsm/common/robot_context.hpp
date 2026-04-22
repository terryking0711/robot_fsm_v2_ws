#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robot_interfaces/action/navigate_to_named_pose.hpp"
#include "robot_interfaces/msg/mechanism_command.hpp"
#include "robot_interfaces/msg/mechanism_feedback.hpp"
#include "robot_interfaces/msg/vision_scene_state.hpp"
#include "robot_interfaces/msg/stage_result.hpp"

// ctx 讓整個 FSM 系統共享全局資訊，例如State、Position、Sensor Data等，讓不同的FSM Stage都能存取和修改這些資訊。
struct RobotContext
{
  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;
  using NavigateClient = rclcpp_action::Client<NavigateAction>;

  rclcpp::Node::SharedPtr node;

  // Navigation
  NavigateClient::SharedPtr nav_client;

  // Mechanism
  rclcpp::Publisher<robot_interfaces::msg::MechanismCommand>::SharedPtr mechanism_cmd_pub;

  // Cached latest messages
  std::mutex data_mutex;
  std::optional<robot_interfaces::msg::MechanismFeedback> latest_mechanism_feedback;
  std::optional<robot_interfaces::msg::VisionSceneState> latest_vision_state;
  std::optional<geometry_msgs::msg::PoseStamped> latest_final_pose;

  // Mission flags
  bool start_signal = false;
};