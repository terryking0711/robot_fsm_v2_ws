#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_states.hpp"

class Stage1WetlandFSM;
class Stage2ClamFSM;
class Stage3HayFSM;

// localize_at() 的回傳值（tick-based 非阻塞流程）
enum class LocalizeResult
{
  RUNNING,   // 進行中（送出 cmd / 等待 status / 退避重送中）
  SUCCESS,   // localization_manager 回報成功
  FAILURE    // 重試超過上限，放棄
};

class MissionController
{
public:
  explicit MissionController(std::shared_ptr<RobotContext> ctx);

  void tick();

private:
  bool init_system();
  bool self_check();
  bool wait_start();
  bool leave_start_zone();
  bool transition_to_named_pose(const std::string& target_name, float timeout_sec);
  bool finish_decision();

  // ---- localization（對 localization_manager 的 tick-based client）----
  // 每個 tick 呼叫一次；內部處理送出 cmd、等待 /init_pose_status、
  // timeout 重送與重試上限。
  LocalizeResult localize_at(const Pose2D& target_world);
  void reset_localize_bookkeeping();

  // ---- 場外重置 ----
  // 檢查 ctx_->pending_reset_stage，若有且目前狀態允許重置，
  // 取消進行中的導航、重建該關 FSM、切到 RELOCALIZE。
  void handle_reset_request();
  bool is_resettable_state(MissionState s) const;
  void reset_nav_bookkeeping();

  std::shared_ptr<RobotContext> ctx_;
  MissionState state_;

  std::shared_ptr<Stage1WetlandFSM> stage1_fsm_;
  std::shared_ptr<Stage2ClamFSM> stage2_fsm_;
  std::shared_ptr<Stage3HayFSM> stage3_fsm_;

  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
  std::string nav_message_;
  std::string nav_active_target_;

  // 導航失敗後的重送退避：
  // 失敗後等 nav_backoff_ticks_ 個 tick（tick=10Hz，20 ticks = 2 秒）
  // 再重送 goal，避免 Nav2 還在 recovery 時被連續洗 goal。
  int nav_retry_count_;
  int nav_backoff_ticks_;
  static constexpr int kNavBackoffTicks = 20;
  static constexpr int kNavMaxRetryWarn = 3;

  // ---- localization bookkeeping ----
  bool loc_cmd_sent_;
  int loc_wait_ticks_;
  int loc_retry_count_;
  // localization_manager 內部最慢 5 秒（verify timeout）一定會回報，
  // 10 秒沒收到視為訊息掉了，重送。
  static constexpr int kLocTimeoutTicks = 100;  // 10Hz * 10s
  static constexpr int kLocMaxRetries = 5;

  // ---- reset bookkeeping ----
  MissionState reset_resume_state_;   // RELOCALIZE 成功後要回到的關卡
  std::string reset_pose_key_;        // field_poses 內的重置點 key
};
