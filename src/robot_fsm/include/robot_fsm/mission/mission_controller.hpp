#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_states.hpp"

class Stage1WetlandFSM;
class Stage2ClamFSM;
class Stage3HayFSM;

class MissionController
{
public:
  explicit MissionController(std::shared_ptr<RobotContext> ctx);

  void tick();

private:
  bool init_system();
  bool self_check();
  bool wait_start();
  bool leave_start_zone();
  bool transition_to_named_pose(const std::string& target_name, float timeout_sec);
  bool finish_decision();

  std::shared_ptr<RobotContext> ctx_;
  MissionState state_;

  std::shared_ptr<Stage1WetlandFSM> stage1_fsm_;
  std::shared_ptr<Stage2ClamFSM> stage2_fsm_;
  std::shared_ptr<Stage3HayFSM> stage3_fsm_;

  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
  std::string nav_message_;
  std::string nav_active_target_;
};