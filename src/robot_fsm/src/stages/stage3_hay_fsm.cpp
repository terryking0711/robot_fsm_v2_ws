#include "robot_fsm/stages/stage3_hay_fsm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <yaml-cpp/yaml.h>

namespace
{

double normalize_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace

Stage3HayFSM::Stage3HayFSM(std::shared_ptr<RobotContext> ctx)
: ctx_(ctx),
  state_(Stage3State::S3_ENTER),
  tick_count_(0),
  state_command_sent_(false),
  state_enter_time_(ctx_->node->now()),
  rows_valid_(false),
  row_idx_(0),
  row_start_time_(ctx_->node->now()),
  empty_ticks_(0),
  picked_local_(0),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_(""),
  nav_retry_count_(0),
  nav_backoff_ticks_(0)
{
  load_rows();
}

void Stage3HayFSM::enter_state(Stage3State next_state)
{
  state_ = next_state;
  tick_count_ = 0;
  state_command_sent_ = false;
  state_enter_time_ = ctx_->node->now();
}

void Stage3HayFSM::publish_cmd_vel(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.linear.y = linear_y;
  msg.angular.z = angular_z;
  ctx_->cmd_vel_pub->publish(msg);
}

void Stage3HayFSM::load_rows()
{
  rows_.clear();
  rows_valid_ = false;

  const auto& node = ctx_->node;
  std::string file;
  if (node->has_parameter("named_poses_file")) {
    file = node->get_parameter("named_poses_file").as_string();
  }
  if (file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "[Stage3] named_poses_file is empty, cannot load rows");
    return;
  }

  YAML::Node poses;
  try {
    poses = YAML::LoadFile(file)["named_poses"];
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "[Stage3] failed to load %s: %s", file.c_str(), e.what());
    return;
  }

  for (int i = 1; i <= kRowCount; ++i) {
    const std::string start_name = "stage3_row" + std::to_string(i) + "_start";
    const std::string end_name = "stage3_row" + std::to_string(i) + "_end";
    const YAML::Node start = poses[start_name];
    const YAML::Node end = poses[end_name];

    if (!start || !end || !start["x"] || !start["y"] || !start["yaw"] || !end["x"] || !end["y"]) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[Stage3] named pose '%s' / '%s' missing or incomplete in %s",
        start_name.c_str(), end_name.c_str(), file.c_str());
      return;
    }

    Stage3Row row;
    row.start_pose = start_name;
    row.start_x = start["x"].as<double>();
    row.start_y = start["y"].as<double>();
    const double dx = end["x"].as<double>() - row.start_x;
    const double dy = end["y"].as<double>() - row.start_y;
    row.length = std::hypot(dx, dy);

    if (row.length < 0.1) {
      RCLCPP_ERROR(
        node->get_logger(),
        "[Stage3] row %d length %.3f m too short, measure '%s' / '%s' first",
        i, row.length, start_name.c_str(), end_name.c_str());
      return;
    }

    row.dir_x = dx / row.length;
    row.dir_y = dy / row.length;
    row.heading = std::atan2(dy, dx);

    // 起點的 yaw 應該就是掃描方向，差太多代表點位量錯
    const double yaw_diff = normalize_angle(start["yaw"].as<double>() - row.heading);
    if (std::fabs(yaw_diff) > 0.2) {
      RCLCPP_WARN(
        node->get_logger(),
        "[Stage3] row %d: '%s' yaw differs from start->end heading by %.2f rad",
        i, start_name.c_str(), yaw_diff);
    }

    RCLCPP_INFO(
      node->get_logger(),
      "[Stage3] row %d: start (%.3f, %.3f) heading %.3f length %.3f",
      i, row.start_x, row.start_y, row.heading, row.length);
    rows_.push_back(row);
  }

  rows_valid_ = true;
}

bool Stage3HayFSM::get_robot_pose(double& x, double& y, double& yaw)
{
  std::scoped_lock lock(ctx_->data_mutex);
  if (!ctx_->latest_robot_pose ||
    (ctx_->node->now() - ctx_->latest_robot_pose_time).seconds() > kPoseStaleSec)
  {
    return false;
  }

  const auto& p = ctx_->latest_robot_pose->pose;
  x = p.position.x;
  y = p.position.y;
  const auto& q = p.orientation;
  yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return true;
}

std::optional<straw_interfaces::msg::StrawTarget> Stage3HayFSM::fresh_straw_target()
{
  std::scoped_lock lock(ctx_->data_mutex);
  if (!ctx_->latest_straw_target ||
    (ctx_->node->now() - ctx_->latest_straw_target_time).seconds() > kStrawTargetStaleSec)
  {
    return std::nullopt;
  }
  return ctx_->latest_straw_target;
}

std::optional<robot_interfaces::msg::StrawPickStatus> Stage3HayFSM::fresh_pick_status()
{
  std::scoped_lock lock(ctx_->data_mutex);
  if (!ctx_->latest_straw_pick_status ||
    (ctx_->node->now() - ctx_->latest_straw_pick_status_time).seconds() > kPickStatusStaleSec)
  {
    return std::nullopt;
  }
  return ctx_->latest_straw_pick_status;
}

int Stage3HayFSM::picked_count()
{
  std::scoped_lock lock(ctx_->data_mutex);
  if (!kArmPick || !ctx_->latest_straw_pick_status) {
    return picked_local_;  // 人工夾取模式下 STM 不會累加 pick_count
  }

  const uint8_t count = ctx_->latest_straw_pick_status->pick_count;
  if (!pick_count_base_) {
    // 進入本關時還沒收到 STM 回報，以第一次收到的值為基準
    pick_count_base_ = count;
  }
  return static_cast<uint8_t>(count - *pick_count_base_);  // uint8 自然處理溢位
}

bool Stage3HayFSM::ensure_pick_armed(bool armed, const std::string& state_name)
{
  const auto status = fresh_pick_status();
  if (status && status->armed == armed) {
    return true;
  }

  if (tick_count_ % kArmResendTicks == 0) {
    send_mechanism_command(
      armed ? kCmdPickArm : kCmdPickDisarm,
      state_name,
      armed ? "pick_arm" : "pick_disarm");
  }

  tick_count_++;
  if (tick_count_ >= kArmConfirmTimeoutTicks) {
    RCLCPP_WARN(
      ctx_->node->get_logger(),
      "[Stage3] STM did not confirm %s (no /mechanism/straw_pick_status or armed mismatch), continue anyway",
      armed ? "arm" : "disarm");
    tick_count_ = 0;
    return true;
  }
  return false;
}

bool Stage3HayFSM::navigate_to_named_pose(
  const std::string& target_name,
  float timeout_sec)
{
  // ---- 導航總開關（RobotContext::enable_navigation）----
  // false 時不送 goal，直接視為已抵達，讓機構流程可以在沒有
  // navigation_server / Nav2 / localization_manager 的情況下單獨測試。
  if (!ctx_->enable_navigation) {
    RCLCPP_INFO(
      ctx_->node->get_logger(),
      "[Stage3][Nav] navigation disabled, skip goal '%s'",
      target_name.c_str());
    return true;
  }

  using NavigateAction = robot_interfaces::action::NavigateToNamedPose;

  if (!ctx_->nav_client) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3][Nav] nav_client not initialized");
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
        "[Stage3][Nav] /navigate_to_named_pose server not ready, waiting...");
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
      "[Stage3][Nav] send navigation goal: %s (attempt %d)",
      target_name.c_str(),
      nav_retry_count_ + 1);

    auto options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();

    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          nav_done_ = true;
          nav_success_ = false;
          nav_message_ = "goal rejected by navigation_server (unknown pose name?)";

          RCLCPP_ERROR(
            ctx_->node->get_logger(),
            "[Stage3][NavResponse] %s", nav_message_.c_str());
        }
      };

    options.feedback_callback =
      [this](
        rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
        const std::shared_ptr<const NavigateAction::Feedback> feedback)
      {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3][NavFeedback] state=%s progress=%.2f",
          feedback->current_state.c_str(),
          feedback->progress);
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
      {
        nav_done_ = true;
        ctx_->last_nav_end_time = ctx_->node->now();

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
          "[Stage3][NavResult] success=%s message=%s",
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
      "[Stage3][Nav] navigation to '%s' complete",
      nav_active_target_.c_str());

    nav_retry_count_ = 0;
    nav_backoff_ticks_ = 0;
  } else {
    nav_retry_count_++;
    nav_backoff_ticks_ = kNavBackoffTicks;

    RCLCPP_ERROR(
      ctx_->node->get_logger(),
      "[Stage3][Nav] navigation to '%s' failed (attempt %d): %s -- retry in %.1f s",
      nav_active_target_.c_str(),
      nav_retry_count_,
      nav_message_.c_str(),
      kNavBackoffTicks / 10.0);

    if (nav_retry_count_ >= kNavMaxRetryWarn) {
      RCLCPP_ERROR(
        ctx_->node->get_logger(),
        "[Stage3][Nav] navigation to '%s' failed %d times in a row. "
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

void Stage3HayFSM::send_mechanism_command(
  uint16_t command_id,
  const std::string& state_name,
  const std::string& action_name,
  const std::string& extra_json)
{
  robot_interfaces::msg::MechanismCommand msg;
  msg.command_id = ctx_->side_command_id(command_id);
  msg.command_name = action_name;
  msg.arg_json =
    R"({"stage":"stage3_hay","state":")" + state_name +
    R"(","action":")" + action_name +
    R"(","extra":)" + extra_json + R"(})";

  ctx_->mechanism_cmd_pub->publish(msg);

  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage3][STM_CMD] id=%u name=%s json=%s",
    msg.command_id,
    msg.command_name.c_str(),
    msg.arg_json.c_str());
}

void Stage3HayFSM::publish_state_command(
  uint16_t command_id,
  const std::string& state_name,
  const std::string& action_name,
  const std::string& extra_json)
{
  if (state_command_sent_) {
    return;
  }
  send_mechanism_command(command_id, state_name, action_name, extra_json);
  state_command_sent_ = true;
}

bool Stage3HayFSM::tick_sweep()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();
  const Stage3Row& row = rows_[row_idx_];

  // 1. 機構已經碰到稻草捲 -> 立刻停車等夾取
  const auto status = fresh_pick_status();
  if (status &&
    (status->phase == robot_interfaces::msg::StrawPickStatus::PHASE_PICKING ||
    status->switch_pressed))
  {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO(logger, "[Stage3] SWEEP row %zu 碰到稻草捲，停車等待夾取", row_idx_ + 1);
    enter_state(Stage3State::S3_WAIT_PICK);
    return false;
  }

  // 2. 單排超時
  if ((ctx_->node->now() - row_start_time_).seconds() > kRowTimeoutSec) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN(logger, "[Stage3] SWEEP row %zu timeout (%.0f s)", row_idx_ + 1, kRowTimeoutSec);
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  // 3. 位姿：算沿排進度與偏離中線距離
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!get_robot_pose(x, y, yaw)) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "[Stage3] SWEEP no fresh /robot_pose, hold still");
    return false;
  }

  const double rel_x = x - row.start_x;
  const double rel_y = y - row.start_y;
  const double progress = rel_x * row.dir_x + rel_y * row.dir_y;
  const double line_offset = row.dir_x * rel_y - row.dir_y * rel_x;  // > 0 = 在中線左側
  const double remaining = row.length - progress;

  if (remaining <= 0.0) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO(logger, "[Stage3] SWEEP row %zu 到排尾", row_idx_ + 1);
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  // 4. 相機
  const auto target = fresh_straw_target();
  const bool cam_ok =
    target && target->valid && target->has_depth &&
    std::fabs(target->lateral_error_m) < kCamLateralGateM &&
    target->distance_m < remaining + kCamAheadMarginM;

  // 空排判斷：只有「畫面裡完全沒有稻草」才算。看到別排的（valid 但被閘門擋掉）、
  // low_confidence、相機沒資料都不算，寧可掃到排尾也不要漏撿。
  if (target && !target->valid && target->reason == "no_detection") {
    empty_ticks_++;
  } else {
    empty_ticks_ = 0;
  }

  if (kEnableEmptyRowExit && empty_ticks_ >= kEmptyRowTicks) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO(
      logger, "[Stage3] SWEEP row %zu 前方沒有稻草，提早換排（剩 %.2f m）",
      row_idx_ + 1, remaining);
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  // 5. 速度
  double vx = kSweepFastSpeed;
  double vy = 0.0;
  if (cam_ok) {
    vy = -kCamLateralGain * target->lateral_error_m;
    if (target->distance_m < kSlowDownDistM) {
      vx = kSweepSlowSpeed;
    }
  } else {
    vy = -kLineLateralGain * line_offset;
  }
  vy = std::clamp(vy, -kMaxLateralSpeed, kMaxLateralSpeed);

  const double wz = std::clamp(
    kYawGain * normalize_angle(row.heading - yaw), -kMaxYawRate, kMaxYawRate);

  publish_cmd_vel(vx, vy, wz);

  RCLCPP_INFO_THROTTLE(
    logger, clock, 500,
    "[Stage3] SWEEP row %zu progress %.2f/%.2f offset %+.3f cam=%s dist=%.2f lat=%+.3f -> vx %.2f vy %+.2f wz %+.2f",
    row_idx_ + 1, progress, row.length, line_offset,
    cam_ok ? "on" : "off",
    target ? target->distance_m : 0.0f,
    target ? target->lateral_error_m : 0.0f,
    vx, vy, wz);
  return false;
}

bool Stage3HayFSM::tick_wait_pick()
{
  publish_cmd_vel(0.0, 0.0, 0.0);

  const double elapsed = (ctx_->node->now() - state_enter_time_).seconds();
  const auto status = fresh_pick_status();
  const bool busy = status &&
    (status->phase == robot_interfaces::msg::StrawPickStatus::PHASE_PICKING ||
    status->switch_pressed);

  if (elapsed < kPickWaitSec) {
    return false;
  }
  if (busy && elapsed < kPickWaitMaxSec) {
    return false;
  }
  if (busy) {
    RCLCPP_WARN(
      ctx_->node->get_logger(),
      "[Stage3] WAIT_PICK STM still busy after %.0f s, continue anyway", kPickWaitMaxSec);
  }
  if (status && status->phase == robot_interfaces::msg::StrawPickStatus::PHASE_ERROR) {
    RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3] WAIT_PICK STM reports PHASE_ERROR");
  }

  picked_local_++;
  empty_ticks_ = 0;
  const int picked = picked_count();
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage3] WAIT_PICK 完成，本關已撿 %d / %d", picked, kTargetPickCount);

  enter_state(picked >= kTargetPickCount ? Stage3State::S3_ROW_DONE : Stage3State::S3_SWEEP);
  return false;
}

bool Stage3HayFSM::tick()
{
  switch (state_) {

    case Stage3State::S3_ENTER:
      if (!rows_valid_) {
        RCLCPP_ERROR(
          ctx_->node->get_logger(),
          "[Stage3] rows not loaded, check stage3_row{1..%d}_start/_end in named_poses.yaml",
          kRowCount);
        enter_state(Stage3State::S3_FAILED);
        return false;
      }
      if (!ensure_pick_armed(false, "S3_ENTER")) {
        return false;
      }
      {
        std::scoped_lock lock(ctx_->data_mutex);
        if (ctx_->latest_straw_pick_status) {
          pick_count_base_ = ctx_->latest_straw_pick_status->pick_count;
        }
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] ENTER 第三關：稻草捲收集");
      enter_state(Stage3State::S3_CROSS);
      return false;

    case Stage3State::S3_CROSS: {
      // 剛結束導航時 velocity_smoother 還在發速度，等它停下來再開始計時
      if (!ctx_->nav_cmd_vel_settled()) {
        state_enter_time_ = ctx_->node->now();
        return false;
      }
      const double elapsed = (ctx_->node->now() - state_enter_time_).seconds();
      if (elapsed < kCrossDurationSec) {
        publish_cmd_vel(kCrossLinearX, kCrossLinearY, kCrossAngularZ);
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] CROSS 衝過田埂完成");
      row_idx_ = 0;
      enter_state(Stage3State::S3_NAV_ROW_START);
      return false;
    }

    case Stage3State::S3_NAV_ROW_START:
      if (!navigate_to_named_pose(rows_[row_idx_].start_pose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_ROW_START 抵達第 %zu 排起點", row_idx_ + 1);
      enter_state(Stage3State::S3_ARM_PICK);
      return false;

    case Stage3State::S3_ARM_PICK:
      // 等 velocity_smoother 停下來才進 SWEEP，避免兩邊的 /cmd_vel 交錯
      if (!ctx_->nav_cmd_vel_settled()) {
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      if (!ensure_pick_armed(kArmPick, "S3_ARM_PICK")) {
        return false;
      }
      row_start_time_ = ctx_->node->now();
      empty_ticks_ = 0;
      RCLCPP_INFO(
        ctx_->node->get_logger(), "[Stage3] ARM_PICK 開始掃描第 %zu 排%s",
        row_idx_ + 1, kArmPick ? "" : "（人工夾取模式：不 arm，碰到後請用手拿走稻草）");
      enter_state(Stage3State::S3_SWEEP);
      return false;

    case Stage3State::S3_SWEEP:
      return tick_sweep();

    case Stage3State::S3_WAIT_PICK:
      return tick_wait_pick();

    case Stage3State::S3_ROW_DONE: {
      publish_cmd_vel(0.0, 0.0, 0.0);
      if (!ensure_pick_armed(false, "S3_ROW_DONE")) {
        return false;
      }
      const int picked = picked_count();
      row_idx_++;
      if (picked >= kTargetPickCount || row_idx_ >= rows_.size()) {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3] ROW_DONE 結束掃描：已撿 %d / %d，掃過 %zu 排",
          picked, kTargetPickCount, row_idx_);
        enter_state(Stage3State::S3_DONE);
      } else {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3] ROW_DONE 已撿 %d / %d，前往第 %zu 排",
          picked, kTargetPickCount, row_idx_ + 1);
        enter_state(Stage3State::S3_NAV_ROW_START);
      }
      return false;
    }

    case Stage3State::S3_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] DONE 第三關完成");
      publish_state_command(311, "S3_DONE", "stage3_done");
      return true;

    case Stage3State::S3_FAILED:
      publish_cmd_vel(0.0, 0.0, 0.0);
      RCLCPP_ERROR_THROTTLE(
        ctx_->node->get_logger(), *ctx_->node->get_clock(), 2000, "[Stage3] FAILED");
      publish_state_command(399, "S3_FAILED", "stage3_failed");
      return false;
  }

  return false;
}
