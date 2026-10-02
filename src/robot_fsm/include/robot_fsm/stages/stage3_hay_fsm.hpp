#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage3_hay_states.hpp"

// 一排稻草的掃描線（world frame）。起點 / 終點都從 named_poses.yaml 讀：
//   stage3_row{i}_start : Nav2 導航目標，也是掃描線起點
//   stage3_row{i}_end   : 掃描線終點（不導航，只用來算方向與「到排尾了沒」）
// 蛇行掃描時第 2、4 排的 start 在場地另一端即可。
struct Stage3Row
{
  std::string start_pose;
  double start_x{0.0};
  double start_y{0.0};
  double dir_x{1.0};     // 起點 -> 終點的單位向量
  double dir_y{0.0};
  double length{0.0};    // 起點到終點距離（m）
  double heading{0.0};   // 掃描時機器人要保持的 yaw（rad）
};

class Stage3HayFSM
{
public:
  explicit Stage3HayFSM(std::shared_ptr<RobotContext> ctx);

  bool tick();  // true = 整關完成, false = 尚未完成

private:
  void enter_state(Stage3State next_state);

  // 每次呼叫都會發送（arm / disarm 重送用）
  void send_mechanism_command(
    uint16_t command_id,
    const std::string& state_name,
    const std::string& action_name,
    const std::string& extra_json = "{}");

  // 同一個 state 只會發送一次
  void publish_state_command(
    uint16_t command_id,
    const std::string& state_name,
    const std::string& action_name,
    const std::string& extra_json = "{}");

  void publish_cmd_vel(double linear_x, double linear_y, double angular_z);

  // 導航到指定 named pose（tick-based 非阻塞，失敗自動退避重送，做法同
  // MissionController::transition_to_named_pose）。
  bool navigate_to_named_pose(const std::string& target_name, float timeout_sec);

  // 從 named_poses.yaml 讀 stage3_row{i}_start / _end
  void load_rows();

  // 送 320 / 321 給機構 STM，直到 /mechanism/straw_pick_status 的 armed 符合為止。
  // STM 沒回報時等 kArmConfirmTimeoutTicks 後直接放行（避免韌體還沒寫好時卡死）。
  bool ensure_pick_armed(bool armed, const std::string& state_name);

  // 讀 ctx 的最新資料，過期則回傳 nullopt / false
  bool get_robot_pose(double& x, double& y, double& yaw);
  std::optional<straw_interfaces::msg::StrawTarget> fresh_straw_target();
  std::optional<robot_interfaces::msg::StrawPickStatus> fresh_pick_status();

  // 本關已夾取數量：有 STM 回報就用 STM 的 pick_count 差值，否則用本地觸發次數
  int picked_count();

  bool tick_sweep();
  bool tick_wait_pick();

  std::shared_ptr<RobotContext> ctx_;
  Stage3State state_;
  int tick_count_;
  bool state_command_sent_;
  rclcpp::Time state_enter_time_;

  // ---- 排 / 夾取進度 ----
  std::vector<Stage3Row> rows_;
  bool rows_valid_;
  size_t row_idx_;
  rclcpp::Time row_start_time_;
  int empty_ticks_;                          // 連續「畫面裡完全沒有稻草」的 tick 數
  int picked_local_;                         // 本地觸發次數（STM 沒回報時的備援）
  std::optional<uint8_t> pick_count_base_;   // 進入本關時 STM 的 pick_count

  // ---- 可在這裡直接調整 ----
  static constexpr int kRowCount = 4;
  static constexpr int kTargetPickCount = 10;

  // S3_CROSS：衝過田埂
  static constexpr double kCrossDurationSec = 2.0;
  static constexpr double kCrossLinearX = 0.4;
  static constexpr double kCrossLinearY = 0.0;
  static constexpr double kCrossAngularZ = 0.0;

  // S3_SWEEP：速度（m/s, rad/s）
  static constexpr double kSweepFastSpeed = 0.25;   // 前方沒有本排稻草 / 還很遠
  static constexpr double kSweepSlowSpeed = 0.08;   // 快碰到稻草時（決定停車前多推幾 cm）
  static constexpr double kSlowDownDistM = 0.35;    // distance_m 小於此值就降速
  static constexpr double kMaxLateralSpeed = 0.15;
  static constexpr double kMaxYawRate = 0.5;

  // S3_SWEEP：橫向 / 角度 P 控制
  //   相機：vy = -kCamLateralGain * lateral_error_m（lateral_error_m > 0 = 目標在右，往右 = vy < 0）
  //         實測方向相反的話把 kCamLateralGain 改成負值。
  //   沒看到本排稻草時：vy = -kLineLateralGain * 偏離排中線距離（> 0 = 機器在中線左側）
  static constexpr double kCamLateralGain = 1.5;
  static constexpr double kLineLateralGain = 1.5;
  static constexpr double kYawGain = 1.5;

  // S3_SWEEP：相機閘門。straw_node 只回報畫面中最大的那一捲，可能是隔壁排的，
  // 橫向超過閘門就當作「別排的」忽略（不能當成本排已經空了）。
  static constexpr double kCamLateralGateM = 0.15;
  static constexpr double kCamAheadMarginM = 0.3;   // distance_m 超過「本排剩餘長度 + margin」也忽略

  // S3_SWEEP：排結束條件
  static constexpr bool kEnableEmptyRowExit = true;  // 前方完全看不到稻草就提早換排
  static constexpr int kEmptyRowTicks = 10;          // 連續 1 秒 no_detection 才算空
  static constexpr double kRowTimeoutSec = 60.0;     // 單排上限（含夾取等待）

  // S3_WAIT_PICK：固定等待 kPickWaitSec；時間到但 STM 還在 PICKING / 開關還壓著就繼續等，
  // 最多等到 kPickWaitMaxSec。
  static constexpr double kPickWaitSec = 3.0;
  static constexpr double kPickWaitMaxSec = 10.0;

  // 資料過期門檻
  static constexpr double kPoseStaleSec = 0.5;
  static constexpr double kStrawTargetStaleSec = 0.3;
  static constexpr double kPickStatusStaleSec = 0.5;

  // 機構 STM /mechanism/command
  // kArmPick = false：人工夾取測試模式（手臂 / 夾取腳本還不能用時）。
  //   每排開始時送 321 保持 disarm，STM 不會跑夾取腳本；
  //   機器碰到稻草（switch_pressed）照樣停車，由隊友用手拿走稻草，
  //   開關放開且停滿 kPickWaitSec 後繼續。已撿數量改用本地觸發次數計算。
  static constexpr bool kArmPick = false;
  static constexpr uint16_t kCmdPickArm = 320;
  static constexpr uint16_t kCmdPickDisarm = 321;
  static constexpr int kArmResendTicks = 5;           // 0.5 s 重送一次
  static constexpr int kArmConfirmTimeoutTicks = 20;  // 2 s 沒確認就放行

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
