#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_states.hpp"

class Stage1WetlandFSM;
class Stage2ClamFSM;
class Stage3HayFSM;

// localize_at() 的回傳值（tick-based 非阻塞流程）
enum class LocalizeResult
{
  RUNNING,   // 進行中（health check / 停穩 / 等待 status / 驗收收斂中）
  SUCCESS,   // 定位已就緒（本來就健康，或重置後驗收通過）
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
  //
  // 對 cartographer 而言「重新初始化」= /finish_trajectory + /start_trajectory，
  // 是破壞性操作：已收斂的估計會被丟掉，換成 YAML 裡寫死的名目 pose，
  // 還要重新累積 submap 才會穩。所以流程是：
  //   IDLE  -> 若 force=false 且目前定位已健康，直接 SUCCESS，完全不碰 cartographer
  //   SETTLE-> 停穩後才送 /init_pose_cmd（新 trajectory 的 extrapolator 最脆弱，
  //            同時閃掉 STM32 動態下的 NaN odom）
  //   WAIT_STATUS -> 等 localization_manager 回報
  //   VERIFY-> manager 回 ok 後再等 pose graph 收斂，自行查 TF 驗收
  //
  // force=false：開機定位（LOCALIZE），已經定位好就跳過
  // force=true ：場外重置（RELOCALIZE），機器人被搬過，估計必錯，強制重建
  enum class LocPhase
  {
    IDLE,
    SETTLE,
    WAIT_STATUS,
    VERIFY
  };

  LocalizeResult localize_at(const Pose2D& target_world, bool force);
  void reset_localize_bookkeeping();

  bool is_localization_healthy(
    const Pose2D& expect_world, double tol_xy, double tol_yaw) const;
  void send_init_pose_cmd(const Pose2D& target_world);
  LocalizeResult handle_localize_retry(const char* reason);

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

  // 定位健康度檢查用（TransformListener 自帶 spin thread，不影響 mission tick）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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
  LocPhase loc_phase_;
  int loc_settle_ticks_;
  int loc_wait_ticks_;
  int loc_verify_ticks_;
  int loc_retry_count_;
  Pose2D loc_target_;

  // localization_manager 內部最慢 5 秒（verify timeout）一定會回報，
  // 10 秒沒收到視為訊息掉了，重送。
  static constexpr int kLocTimeoutTicks = 100;  // 10Hz * 10s
  static constexpr int kLocMaxRetries = 5;

  // tick = 10Hz
  static constexpr int kLocSettleTicks = 10;   // 1.0 s：送 init cmd 前停穩
  static constexpr int kLocVerifyTicks = 20;   // 2.0 s：等 pose graph 收斂再驗收

  // TF frame（若 URDF 用 base_link，改 kBaseFrame）
  static constexpr const char* kWorldFrame = "world";
  static constexpr const char* kBaseFrame = "base_footprint";

  // health check 容差：判斷「是否已經定位好、可以不用重置」，嚴
  static constexpr double kLocTolXy = 0.15;         // m
  static constexpr double kLocTolYaw = 0.15;        // rad (~8.6 deg)

  // 重置後的收斂驗收容差：寬，因為 scan match 會把 pose 修到真實位置，
  // 跟我們給的名目 pose 本來就會有落差
  static constexpr double kLocVerifyTolXy = 0.30;   // m
  static constexpr double kLocVerifyTolYaw = 0.25;  // rad (~14 deg)

  // TF 新鮮度上限：停更代表 cartographer 掛了或還沒收斂
  static constexpr double kLocTfMaxAgeSec = 1.0;    // s

  // ---- reset bookkeeping ----
  MissionState reset_resume_state_;   // RELOCALIZE 成功後要回到的關卡
  std::string reset_pose_key_;        // field_poses 內的重置點 key
};