#include "robot_fsm/mission/mission_controller.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_fsm/stages/stage1_wetland_fsm.hpp"
#include "robot_fsm/stages/stage2_clam_fsm.hpp"
#include "robot_fsm/stages/stage3_hay_fsm.hpp"

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
  loc_cmd_sent_(false),
  loc_wait_ticks_(0),
  loc_retry_count_(0),
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
// ============================================================================

void MissionController::reset_localize_bookkeeping()
{
  loc_cmd_sent_ = false;
  loc_wait_ticks_ = 0;
  loc_retry_count_ = 0;
  ctx_->init_status_received = false;
  ctx_->init_status_ok = false;
}

LocalizeResult MissionController::localize_at(const Pose2D& target_world)
{
  // 1) 尚未送出指令：送出 /init_pose_cmd（world frame）
  if (!loc_cmd_sent_) {
    // 清掉舊的 status，避免吃到上一輪殘留的回報
    ctx_->init_status_received = false;
    ctx_->init_status_ok = false;

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = ctx_->node->get_clock()->now();
    msg.header.frame_id = "world";
    msg.pose.position.x = target_world.x;
    msg.pose.position.y = target_world.y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, target_world.yaw);
    msg.pose.orientation = tf2::toMsg(q);

    ctx_->init_cmd_pub->publish(msg);

    loc_cmd_sent_ = true;
    loc_wait_ticks_ = 0;

    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Mission][Localize] send init cmd (world): [%.3f, %.3f, %.3f] (attempt %d)",
      target_world.x, target_world.y, target_world.yaw,
      loc_retry_count_ + 1);

    return LocalizeResult::RUNNING;
  }

  // 2) 收到 localization_manager 回報
  if (ctx_->init_status_received) {
    const bool ok = ctx_->init_status_ok;
    ctx_->init_status_received = false;
    loc_cmd_sent_ = false;

    if (ok) {
      RCLCPP_INFO(ctx_->node->get_logger(), "[Mission][Localize] localization SUCCESS");
      loc_retry_count_ = 0;
      return LocalizeResult::SUCCESS;
    }

    loc_retry_count_++;
    RCLCPP_WARN(
      ctx_->node->get_logger(),
      "[Mission][Localize] localization failed (attempt %d/%d), resend",
      loc_retry_count_, kLocMaxRetries);

    if (loc_retry_count_ >= kLocMaxRetries) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Mission][Localize] exceeded max retries (%d). "
        "Check: (1) cartographer_node up? (2) robot placed near target pose? "
        "(3) world->map offset consistent?",
        kLocMaxRetries);
      loc_retry_count_ = 0;
      return LocalizeResult::FAILURE;
    }
    return LocalizeResult::RUNNING;  // 下個 tick 重送
  }

  // 3) 等待回報（保險 timeout：manager 最慢 5 秒一定回，10 秒沒回視為掉包）
  loc_wait_ticks_++;
  if (loc_wait_ticks_ > kLocTimeoutTicks) {
    loc_retry_count_++;
    loc_cmd_sent_ = false;

    RCLCPP_WARN(
      ctx_->node->get_logger(),
      "[Mission][Localize] no status after %.1f s, resend (attempt %d/%d)",
      kLocTimeoutTicks / 10.0, loc_retry_count_, kLocMaxRetries);

    if (loc_retry_count_ >= kLocMaxRetries) {
      loc_retry_count_ = 0;
      return LocalizeResult::FAILURE;
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
      // 對出發點初始化定位，成功前不准進入任何導航相關狀態
      switch (localize_at(ctx_->field_poses.at("start"))) {
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
      // 機器人已被隊員搬到重置點，對重置點重新初始化定位
      switch (localize_at(ctx_->field_poses.at(reset_pose_key_))) {
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
