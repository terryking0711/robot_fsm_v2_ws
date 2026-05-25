#pragma once

#include <memory>
#include <string>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage1_wetland_states.hpp"

class Stage1WetlandFSM
{
public:
  explicit Stage1WetlandFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();  // true = 第一關完成, false = 尚未完成

private:
  bool wait_ticks(int required_ticks);
  void publish_state_command(
    uint16_t command_id,
    const std::string& state_name,
    const std::string& action_name,
    const std::string& extra_json = "{}");

  void enter_state(Stage1State next_state);

  std::shared_ptr<RobotContext> ctx_;
  Stage1State state_;
  int tick_count_;
  bool state_command_sent_;
};