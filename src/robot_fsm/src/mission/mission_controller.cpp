#include "robot_fsm/mission/mission_controller.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_fsm/stages/stage1_wetland_fsm.hpp"
#include "robot_fsm/stages/stage2_clam_fsm.hpp"
#include "robot_fsm/stages/stage3_hay_fsm.hpp"

namespace
{
// 角度差包到 [-pi, pi]，避免引入 angles 套件依賴
inline double wrap_angle_diff(double a, double b)
{
  return std::remainder(a - b, 2.0 * M_PI);
}
}  // namespace

MissionController::MissionController(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(MissionState::BOOT),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_(""),
  nav_retry_count_(0),
  nav_backoff_ticks_(0),
  loc_phase_(LocPhase::IDLE),
  loc_settle_ticks_(0),
  loc_wait_ticks_(0),
  loc_verify_ticks_(0),
  loc_retry_count_(0),
  loc_target_(),
  reset_resume_state_(MissionState::SAFE_STOP),
  reset_pose_key_("")
{
  stage1_fsm_ = std::make_shared<Stage1WetlandFSM>(ctx_);
  stage2_fsm_ = std::make_shared<Stage2ClamFSM>(ctx_);
  stage3_fsm_ = std::make_shared<Stage3HayFSM>(ctx_);
}

bool MissionController::init_system()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] INIT");

  ctx_->mechanism_cmd_pub =
    ctx_->node->create_publisher<robot_interfaces::msg::MechanismCommand>(
      "/mechanism/command", 10);

  // 只建立一次 client / publisher，之後全程重用
  //（重建 client 會造成 goal 遺失/失敗迴圈）
  ctx_->nav_client =
    rclcpp_action::create_client<robot_interfaces::action::NavigateToNamedPose>(
      ctx_->node,
      "navigate_to_named_pose");

  ctx_->init_cmd_pub =
    ctx_->node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/init_pose_cmd", 10);

  // TF：用來判斷 cartographer 目前定位是否已經健康。
  // TransformListener 預設會自己開一條 spin thread 處理 /tf，
  // 不會跟 mission tick 的 executor 打架。
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ctx_->node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, ctx_->node);

  return true;
}

bool MissionController::self_check()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] SELF_CHECK");

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] nav_client is null");
    return false;
  }

  if (!ctx_->init_cmd_pub) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] init_cmd_pub is null");
    return false;
  }

  if (!tf_buffer_) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] tf_buffer is null");
    return false;
  }

  if (ctx_->field_poses.find("start") == ctx_->field_poses.end()) {
    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Mission] field_poses missing 'start', check main.cpp parameters");
    return false;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] self check passed");
  return true;
}

bool MissionController::wait_start()
{
  if (ctx_->start_signal) {
    RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] start signal received");
    return true;
  }

  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] waiting start signal...");
  return false;
}

bool MissionController::leave_start_zone()
{
  return transition_to_named_pose("leave_start_zone", 20.0);
}

// ============================================================================
// Localization：tick-based，非阻塞
//
// 設計原則：對 cartographer 而言「重新初始化」= /finish_trajectory +
// /start_trajectory，是破壞性操作 —— 已經收斂的估計會被丟掉，換成 YAML 裡
// 寫死的名目 pose，還要重新累積 submap 才會穩。所以本檔的策略是：
//   1. 已經定位好就完全不動它（health check → 直接 SUCCESS）
//   2. 真的要重置時，先停穩再送指令（新 trajectory 的 extrapolator 最脆弱，
//      而且可以閃掉 STM32 動態下的 NaN odom）
//   3. manager 回報 ok 之後還要自己驗一次收斂，才算數
// ============================================================================

bool MissionController::is_localization_healthy(
  const Pose2D& expect_world,
  double tol_xy,
  double tol_yaw) const
{
  if (!tf_buffer_) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      kWorldFrame, kBaseFrame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException& e) {
    // 還沒有 trajectory / cartographer 尚未起來 → 視為不健康
    RCLCPP_DEBUG(
      ctx_->node->get_logger(),
      "[Mission][Localize] tf %s->%s unavailable: %s",
      kWorldFrame, kBaseFrame, e.what());
    return false;
  }

  // 新鮮度：TF 停更代表 cartographer 掛了或還沒收斂
  const auto age = ctx_->node->now() - rclcpp::Time(tf.header.stamp);
  if (age > rclcpp::Duration::from_seconds(kLocTfMaxAgeSec)) {
    RCLCPP_WARN(
      ctx_->node->get_logger(),
      "[Mission][Localize] tf stale (%.2f s old)", age.seconds());
    return false;
  }

  const double dx = tf.transform.translation.x - expect_world.x;
  const double dy = tf.transform.translation.y - expect_world.y;
  const double dist = std::hypot(dx, dy);

  const double yaw = tf2::getYaw(tf.transform.rotation);
  const double dyaw = std::fabs(wrap_angle_diff(yaw, expect_world.yaw));

  const bool ok = (dist <= tol_xy) && (dyaw <= tol_yaw);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Mission][Localize] health check: current=[%.3f, %.3f, %.3f] "
    "expect=[%.3f, %.3f, %.3f] err=[%.3f m, %.3f rad] tol=[%.3f, %.3f] -> %s",
    tf.transform.translation.x, tf.transform.translation.y, yaw,
    expect_world.x, expect_world.y, expect_world.yaw,
    dist, dyaw, tol_xy, tol_yaw,
    ok ? "HEALTHY" : "UNHEALTHY");

  return ok;
}

void MissionController::reset_localize_bookkeeping()
{
  loc_phase_ = LocPhase::IDLE;
  loc_settle_ticks_ = 0;
  loc_wait_ticks_ = 0;
  loc_verify_ticks_ = 0;
  loc_retry_count_ = 0;
  ctx_->init_status_received = false;
  ctx_->init_status_ok = false;
}

void MissionController::send_init_pose_cmd(const Pose2D& target_world)
{
  // 清掉舊的 status，避免吃到上一輪殘留的回報
  ctx_->init_status_received = false;
  ctx_->init_status_ok = false;

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = ctx_->node->get_clock()->now();
  msg.header.frame_id = kWorldFrame;
  msg.pose.position.x = target_world.x;
  msg.pose.position.y = target_world.y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, target_world.yaw);
  msg.pose.orientation = tf2::toMsg(q);

  ctx_->init_cmd_pub->publish(msg);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Mission][Localize] send init cmd (world): [%.3f, %.3f, %.3f] (attempt %d/%d)",
    target_world.x, target_world.y, target_world.yaw,
    loc_retry_count_ + 1, kLocMaxRetries);
}

LocalizeResult MissionController::handle_localize_retry(const char* reason)
{
  loc_retry_count_++;

  RCLCPP_WARN(
    ctx_->node->get_logger(),
    "[Mission][Localize] %s (attempt %d/%d)",
    reason, loc_retry_count_, kLocMaxRetries);

  if (loc_retry_count_ >= kLocMaxRetries) {
    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Mission][Localize] exceeded max retries (%d). "
      "Check: (1) cartographer_node up? (2) robot placed near target pose? "
      "(3) world->map static TF offset consistent with named_poses.yaml? "
      "(4) /odometry/filtered healthy (no NaN)?",
      kLocMaxRetries);
    loc_phase_ = LocPhase::IDLE;
    loc_retry_count_ = 0;
    return LocalizeResult::FAILURE;
  }

  // 回到 SETTLE：重送之前再停穩一次
  loc_phase_ = LocPhase::SETTLE;
  loc_settle_ticks_ = 0;
  loc_wait_ticks_ = 0;
  loc_verify_ticks_ = 0;
  return LocalizeResult::RUNNING;
}

LocalizeResult MissionController::localize_at(const Pose2D& target_world, bool force)
{
  // ---- 導航總開關（RobotContext::enable_navigation）----
  // 與導航共用同一個開關：false 時不對 localization_manager 送
  // /init_pose_cmd，直接視為定位成功。
  if (!ctx_->enable_navigation) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Mission][Localize] navigation disabled, skip localization");
    return LocalizeResult::SUCCESS;
  }

  loc_target_ = target_world;

  switch (loc_phase_) {

    // ------------------------------------------------------------------
    // 進入點：先問「有沒有必要重置」
    // ------------------------------------------------------------------
    case LocPhase::IDLE: {
      if (!force && is_localization_healthy(target_world, kLocTolXy, kLocTolYaw)) {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][Localize] already localized near target, "
          "skip trajectory restart");
        loc_retry_count_ = 0;
        return LocalizeResult::SUCCESS;
      }

      if (force) {
        RCLCPP_WARN(
          ctx_->node->get_logger(),
          "[Mission][Localize] forced relocalization requested");
      }

      loc_phase_ = LocPhase::SETTLE;
      loc_settle_ticks_ = 0;
      return LocalizeResult::RUNNING;
    }

    // ------------------------------------------------------------------
    // 停穩：新 trajectory 的 extrapolator 沒有歷史資料，動態下重置必爛。
    // 順便閃掉 STM32 在運動時噴 NaN odom 的路徑。
    // ------------------------------------------------------------------
    case LocPhase::SETTLE: {
      loc_settle_ticks_++;
      if (loc_settle_ticks_ < kLocSettleTicks) {
        return LocalizeResult::RUNNING;
      }

      send_init_pose_cmd(target_world);

      loc_phase_ = LocPhase::WAIT_STATUS;
      loc_wait_ticks_ = 0;
      return LocalizeResult::RUNNING;
    }

    // ------------------------------------------------------------------
    // 等 localization_manager 回報
    // ------------------------------------------------------------------
    case LocPhase::WAIT_STATUS: {
      if (ctx_->init_status_received) {
        const bool ok = ctx_->init_status_ok;
        ctx_->init_status_received = false;

        if (!ok) {
          return handle_localize_retry("localization_manager reported failure, resend");
        }

        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][Localize] manager reported OK, waiting %.1f s to verify convergence",
          kLocVerifyTicks / 10.0);

        loc_phase_ = LocPhase::VERIFY;
        loc_verify_ticks_ = 0;
        return LocalizeResult::RUNNING;
      }

      // 保險 timeout：manager 最慢 5 秒一定回，超過視為掉包
      loc_wait_ticks_++;
      if (loc_wait_ticks_ > kLocTimeoutTicks) {
        return handle_localize_retry("no status from localization_manager, resend");
      }

      return LocalizeResult::RUNNING;
    }

    // ------------------------------------------------------------------
    // 驗收：manager 說 ok 不代表 pose graph 已經收斂。
    // 等一段時間後自己查 TF，確認估計真的落在目標附近才放行。
    // ------------------------------------------------------------------
    case LocPhase::VERIFY: {
      loc_verify_ticks_++;
      if (loc_verify_ticks_ < kLocVerifyTicks) {
        return LocalizeResult::RUNNING;
      }

      // 容差比 health check 寬：重置後 cartographer 會 scan match 到真實位置，
      // 跟我們給的名目 pose 本來就會有落差。
      if (is_localization_healthy(target_world, kLocVerifyTolXy, kLocVerifyTolYaw)) {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][Localize] localization SUCCESS (converged)");
        loc_phase_ = LocPhase::IDLE;
        loc_retry_count_ = 0;
        return LocalizeResult::SUCCESS;
      }

      return handle_localize_retry("pose did not converge after restart, resend");
    }
  }

  return LocalizeResult::RUNNING;
}

// ============================================================================
// 場外重置
// ============================================================================

bool MissionController::is_resettable_state(MissionState s) const
{
  switch (s) {
    case MissionState::LEAVE_START_ZONE:
    case MissionState::STAGE1_WETLAND:
    case MissionState::TRANSITION_TO_STAGE2:
    case MissionState::STAGE2_CLAM:
    case MissionState::TRANSITION_TO_STAGE3:
    case MissionState::STAGE3_HAY:
    case MissionState::RELOCALIZE:   // 重置中再重置：允許，取新的重置點
      return true;
    default:
      return false;
  }
}

void MissionController::reset_nav_bookkeeping()
{
  nav_goal_sent_ = false;
  nav_done_ = false;
  nav_success_ = false;
  nav_message_.clear();
  nav_active_target_.clear();
  nav_retry_count_ = 0;
  nav_backoff_ticks_ = 0;
}

void MissionController::handle_reset_request()
{
  const uint8_t stage = *ctx_->pending_reset_stage;
  ctx_->pending_reset_stage.reset();

  std::string pose_key;
  MissionState resume;

  switch (stage) {
    case 1:
      pose_key = "reset_stage1";
      resume = MissionState::STAGE1_WETLAND;
      stage1_fsm_ = std::make_shared<Stage1WetlandFSM>(ctx_);  // 該關 FSM 重來
      break;
    case 2:
      pose_key = "reset_stage2";
      resume = MissionState::STAGE2_CLAM;
      stage2_fsm_ = std::make_shared<Stage2ClamFSM>(ctx_);
      break;
    case 3:
      pose_key = "reset_stage3";
      resume = MissionState::STAGE3_HAY;
      stage3_fsm_ = std::make_shared<Stage3HayFSM>(ctx_);
      break;
    case 4:
      // TODO: mission 目前只跑到第三關（STAGE3_HAY），
      // 第四關加入 MissionState 後再補上 resume state 與 stage4 FSM 重建。
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Mission][Reset] stage 4 not yet wired into mission flow, ignore");
      return;
    default:
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Mission][Reset] invalid reset stage %u (expect 1~4), ignore", stage);
      return;
  }

  if (ctx_->field_poses.find(pose_key) == ctx_->field_poses.end()) {
    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Mission][Reset] field_poses missing '%s', ignore reset", pose_key.c_str());
    return;
  }

  RCLCPP_WARN(
    ctx_->node->get_logger(),
    "[Mission][Reset] reset to stage %u requested, cancel nav and relocalize at '%s'",
    stage, pose_key.c_str());

  // 取消進行中的導航 goal，並清空導航/定位流程狀態
  if (ctx_->nav_client) {
    ctx_->nav_client->async_cancel_all_goals();
  }
  reset_nav_bookkeeping();
  reset_localize_bookkeeping();

  reset_pose_key_ = pose_key;
  reset_resume_state_ = resume;
  state_ = MissionState::RELOCALIZE;
}

// ============================================================================
// Navigation（原邏輯不變）
// ============================================================================

bool MissionController::transition_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  // ---- 導航總開關（RobotContext::enable_navigation）----
  // false 時不送 goal，直接視為已抵達，讓機構流程可以在沒有
  // navigation_server / Nav2 / localization_manager 的情況下單獨測試。
  if (!ctx_->enable_navigation) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Mission][Nav] navigation disabled, skip goal '%s'",
      target_name.c_str());
    return true;
  }

  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] nav_client not initialized");
    return false;
  }

  // 失敗後的退避期：先消化 backoff ticks，再允許重送
  if (nav_backoff_ticks_ > 0) {
    nav_backoff_ticks_--;
    return false;
  }

  if (!nav_goal_sent_) {
    if (!ctx_->nav_client->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(
        ctx_->node->get_logger(),
        "[Mission] /navigate_to_named_pose server not ready, waiting...");
      return false;
    }

    NavigateAction::Goal goal;
    goal.target_name = target_name;
    goal.timeout_sec = timeout_sec;

    nav_goal_sent_ = true;
    nav_done_ = false;
    nav_success_ = false;
    nav_message_.clear();
    nav_active_target_ = target_name;

    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Mission] send navigation goal: %s (attempt %d)",
      target_name.c_str(),
      nav_retry_count_ + 1);

    auto options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();

    // navigation_server 對未知 pose name 會直接 REJECT goal，
    // 這裡要接住 goal_response，否則 FSM 會永遠卡在 nav_done_ = false。
    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          nav_done_ = true;
          nav_success_ = false;
          nav_message_ = "goal rejected by navigation_server (unknown pose name?)";

          RCLCPP_ERROR(
            ctx_->node->get_logger(),
            "[Mission][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][NavFeedback] state=%s progress=%.2f",
          feedback->current_state.c_str(),
          feedback->progress);
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
      {
        nav_done_ = true;

        // navigation_server 現在使用正確的 action 語意：
        //   成功 -> SUCCEEDED、失敗/逾時 -> ABORTED、取消 -> CANCELED。
        // 三種終態的 result->message 都有內容，一律讀出來方便除錯。
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          nav_success_ = result.result->success;
          nav_message_ = result.result->message;
        } else {
          nav_success_ = false;
          nav_message_ = result.result
            ? result.result->message
            : (result.code == rclcpp_action::ResultCode::CANCELED
                ? "navigation goal canceled"
                : "navigation goal aborted");
        }

        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Mission][NavResult] success=%s message=%s",
          nav_success_ ? "true" : "false",
          nav_message_.c_str());
      };

    ctx_->nav_client->async_send_goal(goal, options);

    return false;
  }

  if (!nav_done_) {
    return false;
  }

  const bool result = nav_success_;

  if (result) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Mission] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Mission] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Mission] navigation to '%s' failed %d times in a row. "
        "Check: (1) Cartographer localization / initial pose, "
        "(2) named_poses.yaml target inside map & free space, "
        "(3) Nav2 lifecycle nodes all active.",
        nav_active_target_.c_str(),
        nav_retry_count_);
    }
  }

  nav_goal_sent_ = false;
  nav_done_ = false;
  nav_success_ = false;
  nav_message_.clear();
  nav_active_target_.clear();

  return result;
}

bool MissionController::finish_decision()
{
  RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] FINISH_DECISION");
  return true;
}

void MissionController::tick()
{
  // ---- 場外重置請求：優先權最高 ----
  if (ctx_->pending_reset_stage.has_value()) {
    if (is_resettable_state(state_)) {
      handle_reset_request();
    } else {
      // 開機流程（BOOT~WAIT_START）或收尾流程收到重置：丟棄並警告
      RCLCPP_WARN(
        ctx_->node->get_logger(),
        "[Mission][Reset] reset requested but current state not resettable, ignore");
      ctx_->pending_reset_stage.reset();
    }
  }

  switch (state_) {

    case MissionState::BOOT:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] BOOT");
      state_ = MissionState::INIT;
      break;

    case MissionState::INIT:
      if (init_system()) {
        state_ = MissionState::SELF_CHECK;
      } else {
        state_ = MissionState::SAFE_STOP;
      }
      break;

    case MissionState::SELF_CHECK:
      if (self_check()) {
        reset_localize_bookkeeping();
        state_ = MissionState::LOCALIZE;
      } else {
        state_ = MissionState::SAFE_STOP;
      }
      break;

    case MissionState::LOCALIZE: {
      // 開機定位：cartographer 若已經在起點附近收斂，就完全不要碰它。
      // force = false → 先做 health check，健康就直接放行。
      switch (localize_at(ctx_->field_poses.at("start"), /*force=*/false)) {
        case LocalizeResult::SUCCESS:
          RCLCPP_INFO(ctx_->node->get_logger(),
            "[Mission] LOCALIZE done, localization ready");
          state_ = MissionState::WAIT_START;
          break;
        case LocalizeResult::FAILURE:
          state_ = MissionState::SAFE_STOP;
          break;
        case LocalizeResult::RUNNING:
        default:
          break;
      }
      break;
    }

    case MissionState::WAIT_START:
      if (wait_start()) {
        state_ = MissionState::LEAVE_START_ZONE;
      }
      break;

    case MissionState::LEAVE_START_ZONE:
      if (leave_start_zone()) {
        state_ = MissionState::STAGE1_WETLAND;
      }
      break;

    case MissionState::STAGE1_WETLAND:
      if (stage1_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE2;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE2:
      if (transition_to_named_pose("stage2_entry", 30.0)) {
        state_ = MissionState::STAGE2_CLAM;
      }
      break;

    case MissionState::STAGE2_CLAM:
      if (stage2_fsm_->tick()) {
        state_ = MissionState::TRANSITION_TO_STAGE3;
      }
      break;

    case MissionState::TRANSITION_TO_STAGE3:
      if (transition_to_named_pose("stage3_entry", 50.0)) {
        state_ = MissionState::STAGE3_HAY;
      }
      break;

    case MissionState::STAGE3_HAY:
      if (stage3_fsm_->tick()) {
        state_ = MissionState::FINISH_DECISION;
      }
      break;

    case MissionState::RELOCALIZE: {
      // 機器人已被隊員搬到重置點，目前的估計必定是錯的：
      // force = true → 跳過 health check，強制重建 trajectory。
      switch (localize_at(ctx_->field_poses.at(reset_pose_key_), /*force=*/true)) {
        case LocalizeResult::SUCCESS:
          RCLCPP_INFO(
            ctx_->node->get_logger(),
            "[Mission] RELOCALIZE done, resume stage");
          state_ = reset_resume_state_;
          break;
        case LocalizeResult::FAILURE:
          state_ = MissionState::SAFE_STOP;
          break;
        case LocalizeResult::RUNNING:
        default:
          break;
      }
      break;
    }

    case MissionState::FINISH_DECISION:
      if (finish_decision()) {
        state_ = MissionState::END_RUN;
      } else {
        state_ = MissionState::EARLY_STOP;
      }
      break;

    case MissionState::EARLY_STOP:
      RCLCPP_WARN(ctx_->node->get_logger(), "[Mission] EARLY_STOP");
      state_ = MissionState::END_RUN;
      break;

    case MissionState::END_RUN:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission] END_RUN - 起點到第三關展示完成！");
      rclcpp::shutdown();
      break;

    case MissionState::SAFE_STOP:
      RCLCPP_ERROR(ctx_->node->get_logger(), "[Mission] SAFE_STOP");
      break;
  }
}