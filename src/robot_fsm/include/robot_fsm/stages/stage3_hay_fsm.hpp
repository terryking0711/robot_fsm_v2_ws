#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/stages/stage3_hay_states.hpp"

// 一排稻草的掃描線（world frame）。起點 / 終點都從 named_poses.yaml 讀：
//   stage3_row{i}_start : Nav2 導航目標，也是掃描線起點；掃完後也會 Nav2 退回這裡
//   stage3_row{i}_end   : 掃描線終點，用來算方向與「到排尾了沒」。
//                         最後一排的 end 同時是放置點（yaw 要填 = 掃描方向，沒開到排尾時 Nav2 導航過去）
//   stage3_place_rotate : 相對 stage3_row4_end 的 dyaw，到排尾後原地轉這麼多再放置
//                         （dyaw > 0 逆時針、< 0 順時針）
// 4 排都朝同一個方向掃（不是蛇行）。
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

  // 讀 ctx 的最新資料，過期則回傳 nullopt / false
  bool get_robot_pose(double& x, double& y, double& yaw);
  std::optional<straw_interfaces::msg::StrawTarget> fresh_straw_target();
  std::optional<robot_interfaces::msg::StrawPickStatus> fresh_pick_status();

  bool tick_sweep();
  bool tick_wait_pick();
  bool tick_backoff();
  bool tick_rotate_at_place();

  std::shared_ptr<RobotContext> ctx_;
  Stage3State state_;
  int tick_count_;
  bool state_command_sent_;
  rclcpp::Time state_enter_time_;

  // ---- 排 / 夾取進度 ----
  std::vector<Stage3Row> rows_;
  bool rows_valid_;
  double place_yaw_;      // 放置時的朝向（stage3_place_rotate 解析後的 yaw）
  bool place_rotate_ccw_; // 到放置點後原地轉的方向：stage3_place_rotate 的 dyaw > 0 = 逆時針（wz > 0）
  size_t row_idx_;
  rclcpp::Time row_start_time_;
  bool row_reached_end_;                     // 這排是不是真的開到排尾才結束
  int empty_ticks_;                          // 連續「畫面裡完全沒有稻草」的 tick 數
  std::optional<rclcpp::Time> switch_release_time_;  // 人工模式：WAIT_PICK 中開關放開的時間
  bool pick_trigger_ready_;  // false = 上一捲還沒確認做完（STM 回 IDLE 且開關放開），不重複觸發 WAIT_PICK
  uint8_t wait_pick_start_count_;  // 進 WAIT_PICK 時 STM 的 pick_count
  bool pick_started_;              // 這次 WAIT_PICK 中 STM 有沒有開始夾（PICKING 或 pick_count 增加）
  int backoff_count_;              // 同一捲連續 BACKOFF 重撞次數，STM 開始夾就歸零
  std::optional<rclcpp::Time> backoff_release_time_;  // BACKOFF 中開關放開的時間
  bool place_stm_idle_;            // ROTATE_AT_PLACE：已經等到 STM 回 IDLE
  std::optional<rclcpp::Time> rotate_settle_start_;  // ROTATE_AT_PLACE：誤差進入容許範圍的時間
  // SWEEP：相機看到本排下一捲（valid + has_depth + 橫向在閘門內）就鎖住，
  // 直到碰到開關（WAIT_PICK）或換排。distance_m 含 z 不是沿排距離，不拿來判斷。
  bool straw_seen_;
  bool busy_hold_;                 // SWEEP：STM 還在夾時看到下一捲或已壓到開關 -> 鎖住停車直到 STM 不再 PICKING
  bool aligned_;                   // SWEEP：已對準前方稻草，可以往前靠（kAlignTolM / kAlignResumeTolM 遲滯）

  // ---- 可在這裡直接調整 ----
  static constexpr int kRowCount = 4;  // 不計數，4 排一定都掃完

  // S3_CROSS：衝過田埂
  static constexpr double kCrossDurationSec = 1.2;
  static constexpr double kCrossLinearX = 0.6;
  static constexpr double kCrossLinearY = 0.0;
  static constexpr double kCrossAngularZ = 0.0;
  // 主迴圈只有 10 Hz，CROSS 期間另外用這個頻率連續發 /cmd_vel（= teleop / velocity_smoother 的 20 Hz）
  static constexpr double kCrossPublishHz = 20.0;

  // S3_SWEEP：速度（m/s, rad/s）
  static constexpr double kSweepFastSpeed = 0.24;   // 前方沒有本排稻草 / 還很遠
  static constexpr double kSweepSlowSpeed = 0.08;   // 對準後靠上去的速度
  static constexpr double kAlignTolM = 0.01;        // 對準完成
  static constexpr double kAlignResumeTolM = 0.04;  // 前進中誤差超過這個就停下來重新對
  static constexpr double kMinAlignLateralSpeed = 0.08;  // 對準時橫移最低速度，太小底盤不動
  static constexpr double kMaxLateralSpeed = 0.15;
  static constexpr double kMaxYawRate = 0.5;

  // S3_SWEEP：橫向 / 角度 P 控制
  static constexpr double kCamLateralGain = 1.5;
  static constexpr double kLineLateralGain = 1.5;
  static constexpr double kYawGain = 1.5;

  static constexpr double kCamLateralGateM = 0.15;

  // S3_SWEEP：排結束條件
  static constexpr bool kEnableEmptyRowExit = false;  // 前方完全看不到稻草就提早換排（最後一排不適用）
  static constexpr int kEmptyRowTicks = 10;          // 連續 1 秒 no_detection 才算空
  static constexpr double kRowTimeoutSec = 60.0;     // 單排上限（含夾取等待）

  static constexpr double kPickWaitSec = 14.0;
  // S3_SWEEP：上一捲還在 PICKING（還沒放好）時，相機看到本排下一捲就停車等 STM 回 IDLE，
  // 避免 PICKING 期間壓到開關（STM 要開關先放開才會再夾）
  static constexpr double kManualResumeDelaySec = 1.5;

  // S3_BACKOFF：STM 只認「開關按下」那一下，送 320 前就壓著的話不會夾。
  static constexpr double kPickStartTimeoutSec = 1.0;
  static constexpr double kBackoffSpeed = 0.05;      // m/s（後退）
  static constexpr double kBackoffExtraSec = 0.5;    // 放開後再退這麼久（約 2.5 cm）
  static constexpr double kBackoffMaxSec = 3.0;      // 退這麼久開關還是壓著就放棄這次，回 SWEEP
  static constexpr int kBackoffMaxRetries = 3;       // 同一捲重撞幾次 STM 都沒夾就結束這排

  // S3_PLACE：到放置點（最後一排排尾）後送放置指令，固定等待後結束本關。
  static constexpr uint16_t kCmdPlaceStraw = 9990;
  static constexpr double kPlaceWaitSec = 7.0;
  static constexpr double kAtPlaceTolM = 0.05;  // 離放置點這麼近就不用再 Nav2 過去
  static constexpr const char* kPlaceRotatePose = "stage3_place_rotate";

  // S3_ROTATE_AT_PLACE：誤差大於 kRotateSlowRad 時照 dyaw 的正負號方向以最大速度轉，
  // 進入 kRotateSlowRad 內改用 P 控制（走最短方向，overshoot 會自己修回來）。
  // 定位 / 底盤有延遲，看到到位時其實已經轉過頭：停車後要連續 kRotateSettleSec 都在容許內才放置
  static constexpr double kRotateMaxSpeed = 0.5;    // rad/s
  static constexpr double kRotateMinSpeed = 0.08;   // rad/s，太小底盤轉不動
  static constexpr double kRotateSlowRad = 0.6;
  static constexpr double kRotateGain = 0.8;
  static constexpr double kRotateTolRad = 0.03;
  static constexpr double kRotateSettleSec = 0.5;
  static constexpr double kRotateTimeoutSec = 15.0; // 超時就直接放置
  // ROTATE_AT_PLACE 開始轉之前一定等 STM 回 IDLE（最後一捲可能還在夾），不設上限

  // 資料過期門檻
  static constexpr double kPoseStaleSec = 0.5;
  static constexpr double kStrawTargetStaleSec = 0.3;
  static constexpr double kPickStatusStaleSec = 0.5;

  // kAutoPick = true ：STM 自動夾取。碰到稻草停車，固定等 kPickWaitSec 再繼續（不等放好）。
  static constexpr bool kAutoPick = true;

  // S3_ROW_BEGIN（只有第 1 排）：kAutoPick = true 時，衝過田埂、Nav2 抵達 stage3_row1_start
  static constexpr uint16_t kCmdEnableAutoPick = 320;
  static constexpr int kEnablePickResendTicks = 5;   // 0.5 s
  static constexpr int kEnablePickSendTicks = 11;    // tick 0 / 5 / 10 各送一次

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
