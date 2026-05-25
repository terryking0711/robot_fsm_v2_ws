#pragma once

#include <memory>
#include <string>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage2_clam_states.hpp"

class Stage2ClamFSM
{
public:
  explicit Stage2ClamFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();

private:
  bool wait_ticks(int required_ticks);
  void enter_state(Stage2State next_state);

  void publish_state_command(
    uint16_t command_id,
    const std::string& state_name,
    const std::string& action_name,
    const std::string& extra_json = "{}");

  std::shared_ptr<RobotContext> ctx_;
  Stage2State state_;
  int tick_count_;
  bool state_command_sent_;
};