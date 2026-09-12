#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>

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

  void publish_cmd_vel(double linear_x, double linear_y, double angular_z);

  bool run_timed_cmd_vel_state(
    const char* state_name,
    double duration_sec,
    double linear_x,
    double linear_y,
    double angular_z,
    Stage2State next_state);

  // 導航到指定 named pose（tick-based 非阻塞，失敗自動退避重送，做法同
  // MissionController::transition_to_named_pose）。
  bool navigate_to_named_pose(const std::string& target_name, float timeout_sec);

  std::shared_ptr<RobotContext> ctx_;
  Stage2State state_;
  int tick_count_;
  bool state_command_sent_;
  rclcpp::Time state_enter_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  static constexpr double kAllignDurationSec = 1.5;
  static constexpr double kAllignLinearX = 0.0;
  static constexpr double kAllignLinearY = -0.1;
  static constexpr double kAllignAngularZ = 0.0;

  static constexpr double kAllignLockDurationSec = 1.0;
  static constexpr double kAllignLockLinearX = 0.0;
  static constexpr double kAllignLockLinearY = -0.1;
  static constexpr double kAllignLockAngularZ = 0.0;

  // ---- navigation bookkeeping（給未來要在特定 state 內導航時直接呼叫用）----
  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
  std::string nav_message_;
  std::string nav_active_target_;
  int nav_retry_count_;
  int nav_backoff_ticks_;
  bool nav_arrived_;
  static constexpr int kNavBackoffTicks = 20;
  static constexpr int kNavMaxRetryWarn = 3;
};