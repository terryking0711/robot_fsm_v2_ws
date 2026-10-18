#pragma once

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/common/direct_transition.hpp"
#include "robot_fsm/stages/stage3_hay_states.hpp"

// 一排稻草的掃描線（world frame）。起點 / 終點都從 named_poses.yaml 讀：
//   stage3_row{i}_start : 掃描線起點（第四排同時是 Nav2 導航目標）；掃完後沿中線倒車退回這裡
//   stage3_row{i}_end   : 掃描線終點，用來算方向與「到排尾了沒」。
//   stage3_place        : 放置點，依 /robot_pose 閉迴路控速到位並對準車頭
// 4 排都朝同一個方向掃（不是蛇行），順序 4 -> 3 -> 2 -> 1。rows_[0] = 第一排 ... rows_[3] = 第四排。
struct Stage3Row
{
  std::string start_pose;
  std::string end_pose;
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

  void start_motion_test();  // DIT-only movement test, bypass STM and mechanism states
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
  bool navigate_to_named_pose(
    const std::string& target_name, float timeout_sec, bool continue_on_timeout = false);

  // 從 named_poses.yaml 讀 stage3_row{i}_start / _end
  void load_rows();

  // 讀 ctx 的最新資料，過期則回傳 nullopt / false
  bool get_robot_pose(double& x, double& y, double& yaw);
  std::optional<straw_interfaces::msg::StrawTarget> fresh_straw_target();
  std::optional<robot_interfaces::msg::StrawPickStatus> fresh_pick_status();

  bool tick_sweep();
  bool tick_wait_pick();
  bool tick_backoff();
  bool tick_row_shift();
  bool tick_row_return();
  bool hold_return_for_pick();
  bool tick_cross_ramp();
  void update_cross_ramp();

  // 位置投影到這排的掃描線：progress = 沿排進度（m），offset = 偏離中線（m，> 0 = 在中線左側）
  static void row_frame(const Stage3Row& row, double x, double y, double& progress, double& offset);
  // 正在掃第四排（第一個掃的排）而且已經超過排尾 kRow4EndOvershootM
  bool past_row4_end();

  // STM 的 pick_count 已經 >= kPlaceAtPickCount（status 過期 = 不算）
  bool pick_target_reached();
  // 停車，等待最後一次夾取後經由轉向點去放置
  void start_go_to_place(const char* from_state);

  std::unique_ptr<DirectTransition> place_transition_;
  std::shared_ptr<RobotContext> ctx_;
  Stage3State state_;
  int tick_count_;
  bool state_command_sent_;
  rclcpp::Time state_enter_time_;

  // ---- 排 / 夾取進度 ----
  std::vector<Stage3Row> rows_;
  bool rows_valid_;
  size_t row_idx_;
  std::vector<size_t> row_order_;
  size_t row_order_pos_ = 0;
  rclcpp::Time row_start_time_;
  int empty_ticks_;                          // 連續「畫面裡完全沒有稻草」的 tick 數
  std::optional<rclcpp::Time> switch_release_time_;  // 人工模式：WAIT_PICK 中開關放開的時間
  bool pick_trigger_ready_;  // false = 上一捲還沒確認做完（STM 回 IDLE 且開關放開），不重複觸發 WAIT_PICK
  uint8_t wait_pick_start_count_;  // 進 WAIT_PICK 時 STM 的 pick_count
  bool pick_started_;              // 這次 WAIT_PICK 中 STM 有沒有開始夾（PICKING 或 pick_count 增加）
  int backoff_count_;              // 同一捲連續 BACKOFF 重撞次數，STM 開始夾就歸零
  std::optional<rclcpp::Time> backoff_release_time_;  // BACKOFF 中開關放開的時間
  bool return_after_pick_ = false;  // late pick at row end/while returning: resume return, not sweep
  bool timed_return_pending_ = false;  // <40 s: stop collecting, finish active pick, then place
  bool place_stm_idle_;            // 已經等到 STM 不再 PICKING
  std::optional<rclcpp::Time> shift_settle_start_;  // ROW_SHIFT：誤差進入容許範圍的時間
  // SWEEP：相機看到本排下一捲（valid + has_depth + 橫向在閘門內）就鎖住，
  // 直到碰到開關（WAIT_PICK）或換排。distance_m 含 z 不是沿排距離，不拿來判斷。
  bool straw_seen_;
  bool busy_hold_;                 // SWEEP：STM 還在夾時看到下一捲或已壓到開關 -> 鎖住停車直到 STM 不再 PICKING
  bool aligned_;                   // SWEEP：已對準前方稻草，可以往前靠（kAlignTolM / kAlignResumeTolM 遲滯）
  bool mech_init_second_sent_;     // MECH_INIT：20001 已送出

  // ---- 可在這裡直接調整 ----
  static constexpr int kRowCount = 4;  // 從第 4 排掃到第 1 排，中途夠 kPlaceAtPickCount 捲就去放置

  // STM 的 pick_count（10001 會歸 0）>= 這個值就停止夾取、直接去放置
  static constexpr uint8_t kPlaceAtPickCount = 6;

  // S3_MECH_INIT：cross 前的機構初始化，10001 -> 等 kMechInitWaitAfterFirstSec -> 20001 -> 等 kMechInitWaitAfterSecondSec
  static constexpr uint16_t kCmdMechInitFirst = 10001;
  static constexpr uint16_t kCmdMechInitSecond = 20001;
  static constexpr double kMechInitWaitAfterFirstSec = 5.0;
  static constexpr double kMechInitWaitAfterSecondSec = 6.0;

  // S3_CROSS：衝過田埂
  static constexpr double kCrossDurationSec = 2.0;
  static constexpr double kCrossLinearX = 0.6;
  static constexpr double kCrossLinearY = 0.0;
  static constexpr double kCrossAngularZ = 0.0;
  // 主迴圈只有 10 Hz，CROSS 期間另外用這個頻率連續發 /cmd_vel（= teleop / velocity_smoother 的 20 Hz）
  static constexpr double kCrossPublishHz = 20.0;

  // S3_SWEEP：速度（m/s, rad/s）
  static constexpr double kSweepFastSpeed = 0.4;   // 前方沒有本排稻草 / 還很遠
  static constexpr double kSweepSlowSpeed = 0.18;   // 對準後靠上去的速度
  static constexpr double kAlignTolM = 0.01;        // 對準完成
  static constexpr double kAlignResumeTolM = 0.04;  // 前進中誤差超過這個就停下來重新對
  static constexpr double kMinAlignLateralSpeed = 0.08;  // 對準時橫移最低速度，太小底盤不動
  static constexpr double kMaxLateralSpeed = 0.12;  // 對準時橫移最高速度
  static constexpr double kMaxYawRate = 0.5;

  // S3_SWEEP：橫向 / 角度 P 控制
  static constexpr double kCamLateralGain = 1.5;
  static constexpr double kLineLateralGain = 1.5;
  static constexpr double kYawGain = 1.5;

  static constexpr double kCamLateralGateM = 0.15;

  static constexpr double kCamToSwitchM = 0.0;  // 相機與極限開關前後幾乎持平
  static constexpr double kRowEndCamMarginM = 0.3;

  // S3_SWEEP：排結束條件
  static constexpr bool kEnableEmptyRowExit = false;  // 前方完全看不到稻草就提早換排（最後一排不適用）
  static constexpr int kEmptyRowTicks = 10;          // 連續 1 秒 no_detection 才算空
  static constexpr double kRowTimeoutSec = 60.0;     // 單排上限（含夾取等待）

  // S3_SWEEP / S3_WAIT_PICK：第四排超過排尾這麼多就立即返回（不管開關），留一點給最後一個格點的稻草
  static constexpr double kRow4EndOvershootM = 0.03;

  // S3_ROW_RETURN：沿中線倒車回排頭，速度由 YAML stage3_row_return_speed 控制；
  // 剩 kRowReturnSlowZoneM 內使用獨立的 stage3_row_return_slow_speed。
  static constexpr double kRowReturnSlowZoneM = 0.35;
  static constexpr double kRowReturnTimeoutSec = 20.0;

  // S3_ROW_SHIFT：保持車頭方向橫移到這排中線（速度上限由 YAML stage3_row_shift_max_speed 控制），
  // 停車後連續 kRowShiftSettleSec 都在 kRowShiftTolM 內才開始掃。超時也照樣開始掃
  static constexpr double kRowShiftTolM = 0.02;
  static constexpr double kRowShiftSettleSec = 0.3;
  static constexpr double kRowShiftTimeoutSec = 10.0;

  static constexpr double kPickWaitSec = 12.5;
  // S3_SWEEP：上一捲還在 PICKING（還沒放好）時，相機看到本排下一捲就停車等 STM 回 IDLE，
  // 避免 PICKING 期間壓到開關（STM 要開關先放開才會再夾）
  static constexpr double kManualResumeDelaySec = 1.5;

  // S3_BACKOFF：STM 只認「開關按下」那一下，送 320 前就壓著的話不會夾。
  static constexpr double kPickStartTimeoutSec = 1.0;
  static constexpr double kBackoffSpeed = 0.05;      // m/s（後退）
  static constexpr double kBackoffExtraSec = 0.5;    // 放開後再退這麼久（約 2.5 cm）
  static constexpr double kBackoffMaxSec = 3.0;      // 退這麼久開關還是壓著就放棄這次，回 SWEEP
  static constexpr int kBackoffMaxRetries = 3;       // 同一捲重撞幾次 STM 都沒夾就結束這排

  // 掃排與返回的縱向加減速限制（m/s²）；安全停車仍立即送零。
  static constexpr double kRowAccel = 0.50;
  static constexpr double kRowDecel = 0.70;
  double row_last_vx_ = 0.0;
  std::optional<std::chrono::steady_clock::time_point> row_last_update_;
  bool place_approach_reached_ = false;
  static constexpr const char* kPlaceApproachPose = "stage3_place_approach";

  // S3_NAV_TO_PLACE：先到轉向點，再到放置點。
  static constexpr const char* kPlacePose = "stage3_place";
  static constexpr uint16_t kCmdPlaceStraw = 9990;
  static constexpr double kPlaceWaitSec = 10.0;

  // 資料過期門檻
  static constexpr double kPoseStaleSec = 0.5;
  static constexpr double kStrawTargetStaleSec = 0.3;
  static constexpr double kPickStatusStaleSec = 0.5;

  // kAutoPick = true ：STM 自動夾取。碰到稻草停車，固定等 kPickWaitSec 再繼續（不等放好）。
  static constexpr bool kAutoPick = true;

  // S3_ROW_BEGIN（只有第一個掃的第 4 排）：kAutoPick = true 時，衝過田埂、Nav2 抵達 stage3_row4_start
  static constexpr uint16_t kCmdEnableAutoPick = 320;
  static constexpr int kEnablePickResendTicks = 5;   // 0.5 s
  static constexpr int kEnablePickSendTicks = 11;    // tick 0 / 5 / 10 各送一次

  // S3_AFTER_PlACE: 退回 60 cm，再送 kCmdAfterPlace
  static constexpr double MoveBackDuration = 2.0;
  static constexpr double MoveBackLinearX = 0.2;
  static constexpr uint16_t kCmdAfterPlace = 9991;

  // CROSS timer runs via spin_some at 20 Hz without blocking localization callbacks.
  rclcpp::TimerBase::SharedPtr cross_timer_;
  std::optional<Pose2D> cross_start_pose_;
  std::chrono::steady_clock::time_point cross_started_;
  std::chrono::steady_clock::time_point cross_last_update_;
  std::optional<std::chrono::steady_clock::time_point> cross_cruise_started_;
  double cross_last_speed_ = 0.0;
  bool cross_done_ = false;

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
