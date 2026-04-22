#pragma once

#include <memory>
#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage3_hay_states.hpp"

class Stage3HayFSM
{
public:
  explicit Stage3HayFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();  // true = 整關完成, false = 尚未完成

private:
  void publish_mechanism_command(uint16_t id,
                                 const std::string& name,
                                 const std::string& arg_json);

  std::shared_ptr<RobotContext> ctx_;
  Stage3State state_;

  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
  bool mechanism_waiting_;
};