#pragma once

#include <memory>
#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage4_mazu_states.hpp"

class Stage4MazuFSM
{
public:
  explicit Stage4MazuFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();  // true = 整關完成, false = 尚未完成

private:
  bool wait_ticks(int required_ticks);

  std::shared_ptr<RobotContext> ctx_;
  Stage4State state_;
  int tick_count_;
};
