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

  // 導航到指定 named pose（tick-based 非阻塞，失敗自動退避重送，做法同
  // MissionController::transition_to_named_pose）。
  bool navigate_to_named_pose(const std::string& target_name, float timeout_sec);

  std::shared_ptr<RobotContext> ctx_;
  Stage2State state_;
  int tick_count_;
  bool state_command_sent_;

  // ---- navigation bookkeeping（給未來要在特定 state 內導航時直接呼叫用）----
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