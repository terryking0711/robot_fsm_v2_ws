#include "robot_fsm/mission/mission_controller.hpp"

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
  // 定位功能已停用（僅保留任務機構流程），直接視為成功並跳過。
  (void)target_world;
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Mission][Localize] localization disabled, skip");
  return LocalizeResult::SUCCESS;
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
  // 導航功能已停用（僅保留任務機構流程），直接視為抵達並跳過。
  (void)timeout_sec;
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Mission] navigation disabled, skip goal '%s'",
    target_name.c_str());
  return true;
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
          state_ = MissionState::WAIT_START;  
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
