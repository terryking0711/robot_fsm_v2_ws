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
  bool wait_ticks(int required_ticks);
  void enter_state(Stage3State next_state);
  void publish_state_command(
    uint16_t command_id,
    const std::string& state_name,
    const std::string& action_name,
    const std::string& extra_json = "{}");

  std::shared_ptr<RobotContext> ctx_;
  Stage3State state_;
  int tick_count_;
  bool state_command_sent_;
};
