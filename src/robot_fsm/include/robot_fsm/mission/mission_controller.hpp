#pragma once

#include <memory>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_states.hpp"
#include "robot_fsm/stages/stage5_new_fsm.hpp"

class Stage2ClamFSM;
class Stage3HayFSM;
class Stage4MazuFSM;

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

  std::shared_ptr<Stage2ClamFSM> stage2_fsm_;
  std::shared_ptr<Stage3HayFSM> stage3_fsm_;
  std::shared_ptr<Stage4MazuFSM> stage4_fsm_;
  std::shared_ptr<Stage5NewFSM> stage5_fsm_;

  std::shared_ptr<RobotContext> ctx_;
  MissionState state_;

  bool nav_goal_sent_;
  rclcpp::Time nav_start_time_;

  static constexpr double NAV_SIM_DURATION_S = 5.0;
};
