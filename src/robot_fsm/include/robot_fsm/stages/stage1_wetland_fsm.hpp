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

  // 導航到指定 named pose（tick-based 非阻塞，失敗自動退避重送，做法同
  // MissionController::transition_to_named_pose）。給未來要在特定 state 內
  // 導航時直接呼叫用。
  bool navigate_to_named_pose(const std::string& target_name, float timeout_sec);

  std::shared_ptr<RobotContext> ctx_;
  Stage1State state_;
  int tick_count_;
  bool state_command_sent_;

  // ---- navigation bookkeeping ----
  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
  std::string nav_message_;
  std::string nav_active_target_;
  int nav_retry_count_;
  int nav_backoff_ticks_;
  static constexpr int kNavBackoffTicks = 20;
  static constexpr int kNavMaxRetryWarn = 3;
};