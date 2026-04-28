#pragma once

#include <memory>
#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage2_clam_states.hpp"

class Stage2ClamFSM
{
public:
  explicit Stage2ClamFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();  // true = 整關完成, false = 尚未完成

private:
  bool wait_ticks(int required_ticks);

  std::shared_ptr<RobotContext> ctx_;
  Stage2State state_;
  int tick_count_;
};
