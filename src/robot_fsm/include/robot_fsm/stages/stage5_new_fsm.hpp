#pragma once

#include <memory>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage_common.hpp"

class Stage5NewFSM
{
public:
  explicit Stage5NewFSM(std::shared_ptr<RobotContext> ctx);

  StageStatus tick();
  void reset();

private:
  enum class State
  {
    ENTER,
    LOCALIZE,
    SCAN,
    PLAN,
    EXECUTE,
    VERIFY,
    DONE
  };

  bool wait_ticks(int required_ticks);

  std::shared_ptr<RobotContext> ctx_;
  State state_;
  int tick_count_;
};