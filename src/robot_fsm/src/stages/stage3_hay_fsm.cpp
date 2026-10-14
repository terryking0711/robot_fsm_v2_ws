#include "robot_fsm/stages/stage3_hay_fsm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include <yaml-cpp/yaml.h>

namespace
{

double normalize_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// 解析 named pose（絕對 {x, y, yaw} 或相對 {relative_to, dx, dy, dyaw}），
// 規則與 robot_navigation/src/navigation_server.cpp 的 resolve_named_pose 相同，改格式要兩邊一起改。
// require_yaw = false 時絕對點可以不填 yaw（row_end 的 yaw 不使用），缺的當 0。
bool resolve_named_pose(
  const YAML::Node& poses, const std::string& name, bool require_yaw,
  double& x, double& y, double& yaw, std::string& err, int depth = 0)
{
  if (depth > 8) {
    err = "relative_to chain too deep (circular?) at '" + name + "'";
    return false;
  }

  const YAML::Node p = poses[name];
  if (!p || !p.IsMap()) {
    err = "named pose '" + name + "' not found";
    return false;
  }

  if (p["relative_to"]) {
    if (!p["dx"] || !p["dy"]) {
      err = "relative pose '" + name + "' missing dx/dy";
      return false;
    }
    double ax, ay, ayaw;
    if (!resolve_named_pose(
        poses, p["relative_to"].as<std::string>(), true, ax, ay, ayaw, err, depth + 1))
    {
      return false;
    }
    const double dx = p["dx"].as<double>();
    const double dy = p["dy"].as<double>();
    const double dyaw = p["dyaw"] ? p["dyaw"].as<double>() : 0.0;
    x = ax + dx;
    y = ay + dy;
    yaw = normalize_angle(ayaw + dyaw);
    return true;
  }

  if (!p["x"] || !p["y"] || (require_yaw && !p["yaw"])) {
    err = "named pose '" + name + (require_yaw ? "' missing x/y/yaw" : "' missing x/y");
    return false;
  }
  x = p["x"].as<double>();
  y = p["y"].as<double>();
  yaw = p["yaw"] ? p["yaw"].as<double>() : 0.0;
  return true;
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
  pick_trigger_ready_(true),
  wait_pick_start_count_(0),
  pick_started_(false),
  backoff_count_(0),
  place_stm_idle_(false),
  straw_seen_(false),
  busy_hold_(false),
  aligned_(false),
  mech_init_second_sent_(false),
  nav_goal_sent_(false),
  nav_done_(false),
  nav_success_(false),
  nav_message_(""),
  nav_active_target_(""),
  nav_retry_count_(0),
  nav_backoff_ticks_(0),
  spin_goal_sent_(false),
  spin_done_(false),
  spin_success_(false),
  spin_fail_count_(0),
  spin_backoff_ticks_(0)
{
  spin_client_ = rclcpp_action::create_client<SpinAction>(ctx_->node, "spin");
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
    double start_x, start_y, start_yaw, end_x, end_y, end_yaw;
    std::string err;
    if (!resolve_named_pose(poses, start_name, true, start_x, start_y, start_yaw, err) ||
      !resolve_named_pose(poses, end_name, false, end_x, end_y, end_yaw, err))
    {
      RCLCPP_ERROR(node->get_logger(), "[Stage3] %s in %s", err.c_str(), file.c_str());
      return;
    }

    Stage3Row row;
    row.start_pose = start_name;
    row.end_pose = end_name;
    row.start_x = start_x;
    row.start_y = start_y;
    const double dx = end_x - row.start_x;
    const double dy = end_y - row.start_y;
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
    const double yaw_diff = normalize_angle(start_yaw - row.heading);
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

  // 0. 夠 6 捲了（例如 WAIT_PICK 等完後 STM 才夾完第 6 捲）-> 不再掃，去放置
  if (pick_target_reached()) {
    start_go_to_place("SWEEP");
    return false;
  }

  // 0.5 第四排超過排尾 -> 不管開關，立即返回（排尾後面是不能夾的 base）
  if (past_row4_end()) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN(logger, "[Stage3] SWEEP row %zu 超過排尾 %.2f m，立即返回", row_idx_ + 1, kRow4EndOvershootM);
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  // 1. 機構已經碰到稻草捲 -> 立刻停車等夾取
  //    邊緣觸發：上一捲要等 STM 回 IDLE 且開關放開才重新允許觸發，
  //    不然 WAIT_PICK 結束時 STM 還在夾 / 開關還壓著，會馬上又進 WAIT_PICK
  const auto status = fresh_pick_status();
  const bool stm_picking =
    status && status->phase == robot_interfaces::msg::StrawPickStatus::PHASE_PICKING;
  if (!pick_trigger_ready_ && status && !stm_picking && !status->switch_pressed) {
    pick_trigger_ready_ = true;
    RCLCPP_INFO(logger, "[Stage3] SWEEP STM 已放好、開關已放開，可以觸發下一捲");
  }
  if (pick_trigger_ready_ && (stm_picking || (status && status->switch_pressed))) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO(logger, "[Stage3] SWEEP row %zu 碰到稻草捲，停車等待夾取", row_idx_ + 1);
    wait_pick_start_count_ = status->pick_count;
    pick_started_ = stm_picking;
    straw_seen_ = false;  // 這捲已經碰到了
    busy_hold_ = false;
    aligned_ = false;
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

  // 沒有 STM 狀態就不知道有沒有碰到稻草，繼續開會推著稻草走 -> 停車等（單排超時兜底）
  if (!status) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(
      logger, clock, 1000, "[Stage3] SWEEP no fresh /mechanism/straw_pick_status, hold still");
    return false;
  }

  // 還不允許觸發（上一捲還沒確認做完）時開關被壓著：可能是 STM 還在夾就撞到下一捲。
  // STM 夾的時候停車等（鎖住，不然稻草被推開、開關一放開又往前推）；
  // STM 已回 IDLE 開關還壓著 = 這捲沒被夾到 -> 退後重撞
  if (!pick_trigger_ready_ && status->switch_pressed) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    if (stm_picking) {
      if (!busy_hold_) {
        RCLCPP_INFO(logger, "[Stage3] SWEEP row %zu 開關壓著但 STM 還在夾上一捲，停車等", row_idx_ + 1);
      }
      busy_hold_ = true;
      return false;
    }
    RCLCPP_WARN(logger, "[Stage3] SWEEP row %zu STM 已 IDLE 但開關還壓著，後退重撞", row_idx_ + 1);
    pick_trigger_ready_ = true;
    backoff_release_time_.reset();
    enter_state(Stage3State::S3_BACKOFF);
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
  // 看到本排下一捲：valid + has_depth + 橫向在閘門內（超過閘門的當成隔壁排）
  //               + 在排尾之前（第四排最後面平放當基底的 5 捲不能夾，不能拿來對準）。
  // touch_progress = 開關碰到這捲時的沿排進度；排尾 row.length 就是開關碰到最後一個格點稻草的位置
  const bool cam_depth = target && target->valid && target->has_depth;
  const double touch_progress =
    cam_depth ? progress + target->position_m.z - kCamToSwitchM : 0.0;
  const bool cam_beyond_end = cam_depth && touch_progress > row.length + kRowEndCamMarginM;
  const bool cam_ok =
    cam_depth && !cam_beyond_end && std::fabs(target->lateral_error_m) < kCamLateralGateM;

  // log 用：相機資料為什麼不能用
  char cam_state[64] = "on";
  if (!target) {
    std::scoped_lock lock(ctx_->data_mutex);
    if (ctx_->latest_straw_target) {
      std::snprintf(
        cam_state, sizeof(cam_state), "off(stale %.2fs)",
        (ctx_->node->now() - ctx_->latest_straw_target_time).seconds());
    } else {
      std::snprintf(cam_state, sizeof(cam_state), "off(no_msg)");
    }
  } else if (!target->valid) {
    std::snprintf(cam_state, sizeof(cam_state), "off(%s)", target->reason.c_str());
  } else if (!target->has_depth) {
    std::snprintf(cam_state, sizeof(cam_state), "off(no_depth)");
  } else if (cam_beyond_end) {
    std::snprintf(
      cam_state, sizeof(cam_state), "off(beyond_end +%.2f)", touch_progress - row.length);
  } else if (!cam_ok) {
    std::snprintf(cam_state, sizeof(cam_state), "off(gate lat=%+.2f)", target->lateral_error_m);
  } else {
    // end = 碰到這捲時離排尾多遠（< 0 = 在排尾之前），校正 kCamToSwitchM 用
    std::snprintf(cam_state, sizeof(cam_state), "on(end%+.2f)", touch_progress - row.length);
  }

  // 空排判斷：只有「畫面裡完全沒有稻草」才算。看到別排的（valid 但被閘門擋掉）、
  // low_confidence、相機沒資料都不算，寧可掃到排尾也不要漏撿。
  if (target && !target->valid && target->reason == "no_detection") {
    empty_ticks_++;
  } else {
    empty_ticks_ = 0;
  }

  // 看到一次就鎖住到碰到開關，相機一閃或太近看不到時才不會當成前面沒東西而加速衝上去
  if (cam_ok && !straw_seen_) {
    straw_seen_ = true;
    RCLCPP_INFO(
      logger, "[Stage3] SWEEP row %zu 看到下一捲（lat %+.3f）", row_idx_ + 1, target->lateral_error_m);
  }

  // 上一捲還沒放好就看到下一捲 -> 停在原地等，放好後再對準、往前推去碰開關。
  // 鎖住直到 STM 不再 PICKING（不看相機有沒有閃）
  const bool stm_busy = !pick_trigger_ready_ && stm_picking;
  if (!stm_busy) {
    busy_hold_ = false;
  } else if (straw_seen_) {
    busy_hold_ = true;
  }
  if (busy_hold_) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO_THROTTLE(
      logger, clock, 2000, "[Stage3] SWEEP row %zu 前方有下一捲，等 STM 放好上一捲", row_idx_ + 1);
    return false;
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
  const bool in_align_zone = straw_seen_;
  if (!in_align_zone) {
    aligned_ = false;
    vy = cam_ok ? -kCamLateralGain * target->lateral_error_m : -kLineLateralGain * line_offset;
  } else if (cam_ok) {
    // 先橫移對準、再慢慢靠上去
    const double lat_err = std::fabs(target->lateral_error_m);
    if (aligned_ && lat_err > kAlignResumeTolM) {
      aligned_ = false;
    } else if (!aligned_ && lat_err < kAlignTolM) {
      aligned_ = true;
    }
    vy = -kCamLateralGain * target->lateral_error_m;
    if (aligned_) {
      vx = kSweepSlowSpeed;
    } else {
      vx = 0.0;
      if (std::fabs(vy) < kMinAlignLateralSpeed) {
        vy = std::copysign(kMinAlignLateralSpeed, vy);
      }
    }
  } else {
    // 太近看不到 / 偵測閃掉：維持現在的橫向位置，直線慢速前進到碰到開關
    vx = kSweepSlowSpeed;
    vy = 0.0;
  }
  vy = std::clamp(vy, -kMaxLateralSpeed, kMaxLateralSpeed);

  const double wz = std::clamp(
    kYawGain * normalize_angle(row.heading - yaw), -kMaxYawRate, kMaxYawRate);

  publish_cmd_vel(vx, vy, wz);

  RCLCPP_INFO_THROTTLE(
    logger, clock, 500,
    "[Stage3] SWEEP row %zu progress %.2f/%.2f offset %+.3f cam=%s lat=%+.3f%s -> vx %.2f vy %+.2f wz %+.2f",
    row_idx_ + 1, progress, row.length, line_offset,
    cam_state,
    target ? target->lateral_error_m : 0.0f,
    in_align_zone ? (aligned_ ? " [aligned]" : (cam_ok ? " [aligning]" : " [blind]")) : "",
    vx, vy, wz);
  return false;
}

bool Stage3HayFSM::tick_wait_pick()
{
  publish_cmd_vel(0.0, 0.0, 0.0);

  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();
  const auto now = ctx_->node->now();
  const auto status = fresh_pick_status();

  // 第四排超過排尾才碰到的不是格點上的稻草 -> 不等夾，立即返回。
  // STM 可能已經開始夾，下一排要等它做完、開關放開才重新允許觸發
  if (past_row4_end()) {
    RCLCPP_WARN(
      logger, "[Stage3] WAIT_PICK row %zu 超過排尾 %.2f m，不等夾取，立即返回",
      row_idx_ + 1, kRow4EndOvershootM);
    pick_trigger_ready_ = false;
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  if (!kAutoPick) {
    // 人工夾取模式：等隊友拿走稻草（開關放開），放開後再停 kManualResumeDelaySec
    if (!status || status->switch_pressed) {
      switch_release_time_.reset();
      RCLCPP_INFO_THROTTLE(
        logger, clock, 3000, "[Stage3] WAIT_PICK %s",
        status ? "等待人工拿走稻草（開關仍壓著）" : "no fresh /mechanism/straw_pick_status, hold still");
      return false;
    }
    if (!switch_release_time_) {
      switch_release_time_ = now;
      RCLCPP_INFO(
        logger, "[Stage3] WAIT_PICK 開關已放開，%.1f s 後繼續", kManualResumeDelaySec);
    }
    if ((now - *switch_release_time_).seconds() < kManualResumeDelaySec) {
      return false;
    }
    switch_release_time_.reset();
    // 人工等待時間不算進單排超時
    row_start_time_ = row_start_time_ + (now - state_enter_time_);
  } else {
    // 自動夾取：夾取 / 收進機器完全交給 STM，這裡只固定停車等待
    if (status &&
      (status->phase == robot_interfaces::msg::StrawPickStatus::PHASE_PICKING ||
      status->pick_count != wait_pick_start_count_))
    {
      pick_started_ = true;
      backoff_count_ = 0;
    }

    // 夾完第 6 捲（STM 已回 IDLE）就不用等滿 kPickWaitSec，直接去放置
    if (status && status->phase != robot_interfaces::msg::StrawPickStatus::PHASE_PICKING &&
      status->pick_count >= kPlaceAtPickCount)
    {
      start_go_to_place("WAIT_PICK");
      return false;
    }

    // 開關壓著但 STM 一直沒開始夾（例如送 320 前就壓著，STM 只認按下那一下）-> 退後重撞。
    // 不處理的話等完 kPickWaitSec 後 pick_trigger_ready_ = false，SWEEP 會推著這捲往前開。
    if (!pick_started_ && status && status->switch_pressed &&
      (now - state_enter_time_).seconds() >= kPickStartTimeoutSec)
    {
      row_start_time_ = row_start_time_ + (now - state_enter_time_);
      if (backoff_count_ >= kBackoffMaxRetries) {
        RCLCPP_WARN(
          logger, "[Stage3] WAIT_PICK 重撞 %d 次 STM 都沒夾，結束第 %zu 排",
          kBackoffMaxRetries, row_idx_ + 1);
        backoff_count_ = 0;
        enter_state(Stage3State::S3_ROW_DONE);
        return false;
      }
      backoff_count_++;
      RCLCPP_WARN(
        logger, "[Stage3] WAIT_PICK 開關壓著但 STM 沒開始夾（armed=%s），後退重撞（第 %d 次）",
        status->armed ? "true" : "false", backoff_count_);
      if (!status->armed) {
        // 320 可能掉了，重送一次
        send_mechanism_command(kCmdEnableAutoPick, "S3_WAIT_PICK", "enable_auto_pick");
      }
      backoff_release_time_.reset();
      enter_state(Stage3State::S3_BACKOFF);
      return false;
    }

    if ((now - state_enter_time_).seconds() < kPickWaitSec) {
      RCLCPP_INFO_THROTTLE(
        logger, clock, 5000, "[Stage3] WAIT_PICK 停車等待 STM 夾取（%.0f s）", kPickWaitSec);
      return false;
    }
    // 等待時間不算進單排超時
    row_start_time_ = row_start_time_ + (now - state_enter_time_);
    pick_trigger_ready_ = false;
  }

  empty_ticks_ = 0;
  RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] WAIT_PICK 完成，繼續掃第 %zu 排", row_idx_ + 1);

  enter_state(Stage3State::S3_SWEEP);
  return false;
}

bool Stage3HayFSM::tick_backoff()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();
  const auto now = ctx_->node->now();
  const auto status = fresh_pick_status();

  // pick_trigger_ready_ 維持 true：回 SWEEP 往前碰到開關就會再進 WAIT_PICK
  if ((now - state_enter_time_).seconds() > kBackoffMaxSec) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN(logger, "[Stage3] BACKOFF %.1f s 開關還是沒放開，回 SWEEP", kBackoffMaxSec);
    enter_state(Stage3State::S3_SWEEP);
    return false;
  }

  if (!status) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(
      logger, clock, 1000, "[Stage3] BACKOFF no fresh /mechanism/straw_pick_status, hold still");
    return false;
  }

  if (!status->switch_pressed && !backoff_release_time_) {
    backoff_release_time_ = now;
  }
  if (backoff_release_time_ && (now - *backoff_release_time_).seconds() >= kBackoffExtraSec) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_INFO(logger, "[Stage3] BACKOFF 開關已放開，回 SWEEP 重撞");
    enter_state(Stage3State::S3_SWEEP);
    return false;
  }

  publish_cmd_vel(-kBackoffSpeed, 0.0, 0.0);
  return false;
}

void Stage3HayFSM::row_frame(
  const Stage3Row& row, double x, double y, double& progress, double& offset)
{
  const double rel_x = x - row.start_x;
  const double rel_y = y - row.start_y;
  progress = rel_x * row.dir_x + rel_y * row.dir_y;
  offset = row.dir_x * rel_y - row.dir_y * rel_x;  // > 0 = 在中線左側
}

bool Stage3HayFSM::past_row4_end()
{
  if (row_idx_ + 1 != rows_.size()) {
    return false;
  }
  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!get_robot_pose(x, y, yaw)) {
    return false;
  }
  const Stage3Row& row = rows_[row_idx_];
  double progress = 0.0, offset = 0.0;
  row_frame(row, x, y, progress, offset);
  return progress > row.length + kRow4EndOvershootM;
}

bool Stage3HayFSM::tick_row_return()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();
  const Stage3Row& row = rows_[row_idx_];

  bool done = false;
  if ((ctx_->node->now() - state_enter_time_).seconds() > kRowReturnTimeoutSec) {
    RCLCPP_WARN(logger, "[Stage3] ROW_RETURN timeout (%.0f s)，直接換排", kRowReturnTimeoutSec);
    done = true;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!done && !get_robot_pose(x, y, yaw)) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "[Stage3] ROW_RETURN no fresh /robot_pose, hold still");
    return false;
  }

  double progress = 0.0, offset = 0.0;
  if (!done) {
    row_frame(row, x, y, progress, offset);
    if (progress <= 0.0) {
      RCLCPP_INFO(
        logger, "[Stage3] ROW_RETURN 退回第 %zu 排起點（progress %+.3f offset %+.3f）",
        row_idx_ + 1, progress, offset);
      done = true;
    }
  }

  if (done) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    row_idx_--;
    RCLCPP_INFO(logger, "[Stage3] ROW_RETURN 橫移到第 %zu 排", row_idx_ + 1);
    shift_settle_start_.reset();
    enter_state(Stage3State::S3_ROW_SHIFT);
    return false;
  }

  // 沿中線直線倒車：橫向 / 角度控制跟 SWEEP 沒看到稻草時一樣
  const double vx = -(progress > kRowReturnSlowZoneM ? kSweepFastSpeed : kSweepSlowSpeed);
  const double vy = std::clamp(-kLineLateralGain * offset, -kMaxLateralSpeed, kMaxLateralSpeed);
  const double wz = std::clamp(
    kYawGain * normalize_angle(row.heading - yaw), -kMaxYawRate, kMaxYawRate);
  publish_cmd_vel(vx, vy, wz);

  RCLCPP_INFO_THROTTLE(
    logger, clock, 500, "[Stage3] ROW_RETURN row %zu progress %.2f offset %+.3f -> vx %.2f vy %+.2f wz %+.2f",
    row_idx_ + 1, progress, offset, vx, vy, wz);
  return false;
}

bool Stage3HayFSM::tick_row_shift()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();
  const auto now = ctx_->node->now();
  const Stage3Row& row = rows_[row_idx_];

  // 第四排是 Nav2 剛到起點：等 velocity_smoother 停下來再開始計時
  if (!ctx_->nav_cmd_vel_settled()) {
    state_enter_time_ = now;
    return false;
  }

  bool done = false;
  if ((now - state_enter_time_).seconds() > kRowShiftTimeoutSec) {
    RCLCPP_WARN(logger, "[Stage3] ROW_SHIFT timeout (%.0f s)，直接開始掃", kRowShiftTimeoutSec);
    done = true;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!done && !get_robot_pose(x, y, yaw)) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "[Stage3] ROW_SHIFT no fresh /robot_pose, hold still");
    return false;
  }

  if (!done) {
    double progress = 0.0, offset = 0.0;
    row_frame(row, x, y, progress, offset);
    if (std::fabs(offset) < kRowShiftTolM) {
      // 停車後連續 kRowShiftSettleSec 都在容許內才算到；滑出去的話下面會修回來
      publish_cmd_vel(0.0, 0.0, 0.0);
      if (!shift_settle_start_) {
        shift_settle_start_ = now;
      }
      if ((now - *shift_settle_start_).seconds() < kRowShiftSettleSec) {
        return false;
      }
      RCLCPP_INFO(
        logger, "[Stage3] ROW_SHIFT 已到第 %zu 排中線（offset %+.3f progress %+.3f）",
        row_idx_ + 1, offset, progress);
      done = true;
    } else {
      shift_settle_start_.reset();
      double vy = std::clamp(-kLineLateralGain * offset, -kMaxLateralSpeed, kMaxLateralSpeed);
      if (std::fabs(vy) < kMinAlignLateralSpeed) {
        vy = std::copysign(kMinAlignLateralSpeed, vy);
      }
      const double wz = std::clamp(
        kYawGain * normalize_angle(row.heading - yaw), -kMaxYawRate, kMaxYawRate);
      publish_cmd_vel(0.0, vy, wz);
      RCLCPP_INFO_THROTTLE(
        logger, clock, 500, "[Stage3] ROW_SHIFT row %zu offset %+.3f -> vy %+.2f wz %+.2f",
        row_idx_ + 1, offset, vy, wz);
      return false;
    }
  }

  publish_cmd_vel(0.0, 0.0, 0.0);
  enter_state(Stage3State::S3_ROW_BEGIN);
  return false;
}

bool Stage3HayFSM::pick_target_reached()
{
  const auto status = fresh_pick_status();
  return status && status->pick_count >= kPlaceAtPickCount;
}

void Stage3HayFSM::start_go_to_place(const char* from_state)
{
  publish_cmd_vel(0.0, 0.0, 0.0);
  const auto status = fresh_pick_status();
  RCLCPP_INFO(
    ctx_->node->get_logger(),
    "[Stage3] %s 第 %zu 排結束（pick_count %d / %u），原地轉 180° 去放置",
    from_state, row_idx_ + 1, status ? static_cast<int>(status->pick_count) : -1,
    static_cast<unsigned>(kPlaceAtPickCount));
  place_stm_idle_ = false;
  spin_goal_sent_ = false;
  spin_done_ = false;
  spin_fail_count_ = 0;
  spin_backoff_ticks_ = 0;
  enter_state(Stage3State::S3_SPIN_180);
}

bool Stage3HayFSM::tick_spin_180()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();

  // 剛結束導航時 velocity_smoother 還在發速度，等它停下來
  if (!ctx_->nav_cmd_vel_settled()) {
    return false;
  }

  // 最後一捲可能還在夾：一定等 STM 不再 PICKING 再轉（status 斷線也一直等）
  if (!place_stm_idle_) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    const auto status = fresh_pick_status();
    const bool idle =
      status && status->phase != robot_interfaces::msg::StrawPickStatus::PHASE_PICKING;
    if (!idle) {
      RCLCPP_INFO_THROTTLE(
        logger, clock, 2000, "[Stage3] SPIN_180 等 STM 夾完最後一捲（%.0f s）%s",
        (ctx_->node->now() - state_enter_time_).seconds(),
        status ? "" : "（no fresh /mechanism/straw_pick_status）");
      return false;
    }
    place_stm_idle_ = true;
  }

  // 失敗後的退避期
  if (spin_backoff_ticks_ > 0) {
    spin_backoff_ticks_--;
    return false;
  }

  if (!spin_goal_sent_) {
    if (!spin_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(logger, "[Stage3] SPIN_180 /spin server not ready, waiting...");
      return false;
    }

    SpinAction::Goal goal;
    goal.target_yaw = kSpinTargetYaw;
    goal.time_allowance = rclcpp::Duration::from_seconds(kSpinTimeAllowanceSec);

    spin_goal_sent_ = true;
    spin_done_ = false;
    spin_success_ = false;

    RCLCPP_INFO(
      logger, "[Stage3] SPIN_180 send spin goal %.3f rad (attempt %d)",
      kSpinTargetYaw, spin_fail_count_ + 1);

    auto options = rclcpp_action::Client<SpinAction>::SendGoalOptions();
    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<SpinAction>::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          spin_done_ = true;
          spin_success_ = false;
          RCLCPP_ERROR(ctx_->node->get_logger(), "[Stage3] SPIN_180 goal rejected");
        }
      };
    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<SpinAction>::WrappedResult& result)
      {
        spin_done_ = true;
        spin_success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        ctx_->last_nav_end_time = ctx_->node->now();
        RCLCPP_INFO(
          ctx_->node->get_logger(), "[Stage3] SPIN_180 result success=%s",
          spin_success_ ? "true" : "false");
      };
    spin_client_->async_send_goal(goal, options);
    return false;
  }

  if (!spin_done_) {
    return false;
  }

  spin_goal_sent_ = false;
  spin_done_ = false;
  if (!spin_success_) {
    spin_fail_count_++;
    if (spin_fail_count_ < kSpinMaxRetries) {
      RCLCPP_ERROR(
        logger, "[Stage3] SPIN_180 failed (%d/%d) -- retry in %.1f s",
        spin_fail_count_, kSpinMaxRetries, kNavBackoffTicks / 10.0);
      spin_backoff_ticks_ = kNavBackoffTicks;
      return false;
    }
    RCLCPP_ERROR(
      logger, "[Stage3] SPIN_180 failed %d times，不轉了，直接往下走", kSpinMaxRetries);
  } else {
    RCLCPP_INFO(logger, "[Stage3] SPIN_180 原地轉 180° 完成");
  }
  enter_state(Stage3State::S3_FORWARD);
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
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] ENTER 第三關：稻草捲收集");
      pick_trigger_ready_ = true;
      enter_state(Stage3State::S3_MECH_INIT);
      return false;

    case Stage3State::S3_MECH_INIT: {
      // 10001 -> 等 3 秒 -> 20001 -> 等 5 秒 -> CROSS
      publish_state_command(kCmdMechInitFirst, "S3_MECH_INIT", "mech_init_1");
      const double elapsed = (ctx_->node->now() - state_enter_time_).seconds();
      if (elapsed < kMechInitWaitAfterFirstSec) {
        return false;
      }
      if (!mech_init_second_sent_) {
        send_mechanism_command(kCmdMechInitSecond, "S3_MECH_INIT", "mech_init_2");
        mech_init_second_sent_ = true;
      }
      if (elapsed < kMechInitWaitAfterFirstSec + kMechInitWaitAfterSecondSec) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] MECH_INIT 完成");
      enter_state(Stage3State::S3_CROSS);
      return false;
    }

    case Stage3State::S3_CROSS: {
      // 剛結束導航時 velocity_smoother 還在發速度，等它停下來再開始衝
      if (!ctx_->nav_cmd_vel_settled()) {
        return false;
      }
      {
        using Clock = std::chrono::steady_clock;
        const auto period = std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(1.0 / kCrossPublishHz));
        const auto end = Clock::now() + std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(kCrossDurationSec));
        auto next = Clock::now();
        while (rclcpp::ok() && Clock::now() < end) {
          publish_cmd_vel(kCrossLinearX, kCrossLinearY, kCrossAngularZ);
          next += period;
          std::this_thread::sleep_until(std::min(next, end));
        }
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] CROSS 衝過田埂完成");
      row_idx_ = rows_.size() - 1;  // 從第四排開始掃
      enter_state(Stage3State::S3_NAV_ROW_START);
      return false;
    }

    case Stage3State::S3_NAV_ROW_START:
      if (!navigate_to_named_pose(rows_[row_idx_].start_pose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_ROW_START 抵達第 %zu 排起點", row_idx_ + 1);
      shift_settle_start_.reset();
      enter_state(Stage3State::S3_ROW_SHIFT);
      return false;

    case Stage3State::S3_ROW_SHIFT:
      return tick_row_shift();

    case Stage3State::S3_ROW_BEGIN:
      // 等 velocity_smoother 停下來才進 SWEEP，避免兩邊的 /cmd_vel 交錯
      if (!ctx_->nav_cmd_vel_settled()) {
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      // 換排途中 STM 才夾完第 6 捲：這排不用掃了
      if (pick_target_reached()) {
        start_go_to_place("ROW_BEGIN");
        return false;
      }
      // 第一個掃的排（第四排）先送 320 開啟自動夾取
      if (kAutoPick && row_idx_ + 1 == rows_.size() && tick_count_ < kEnablePickSendTicks) {
        if (tick_count_ % kEnablePickResendTicks == 0) {
          send_mechanism_command(kCmdEnableAutoPick, "S3_ROW_BEGIN", "enable_auto_pick");
        }
        tick_count_++;
        return false;
      }
      row_start_time_ = ctx_->node->now();
      empty_ticks_ = 0;
      backoff_count_ = 0;
      straw_seen_ = false;
      busy_hold_ = false;
      aligned_ = false;
      RCLCPP_INFO(
        ctx_->node->get_logger(), "[Stage3] ROW_BEGIN 開始掃描第 %zu 排%s",
        row_idx_ + 1, kAutoPick ? "" : "（人工夾取模式：碰到後請用手拿走稻草）");
      enter_state(Stage3State::S3_SWEEP);
      return false;

    case Stage3State::S3_SWEEP:
      return tick_sweep();

    case Stage3State::S3_WAIT_PICK:
      return tick_wait_pick();

    case Stage3State::S3_BACKOFF:
      return tick_backoff();

    case Stage3State::S3_ROW_DONE: {
      publish_cmd_vel(0.0, 0.0, 0.0);

      if (pick_target_reached()) {
        start_go_to_place("ROW_DONE");
      } else if (row_idx_ == 0) {
        RCLCPP_WARN(
          ctx_->node->get_logger(), "[Stage3] ROW_DONE 4 排都掃完還不到 %u 捲，照樣去放置",
          static_cast<unsigned>(kPlaceAtPickCount));
        start_go_to_place("ROW_DONE");
      } else {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3] ROW_DONE 沿中線倒車退回第 %zu 排起點", row_idx_ + 1);
        enter_state(Stage3State::S3_ROW_RETURN);
      }
      return false;
    }

    case Stage3State::S3_ROW_RETURN:
      return tick_row_return();

    case Stage3State::S3_SPIN_180:
      return tick_spin_180();

    case Stage3State::S3_FORWARD:
      // 等 spin 的速度指令停下來再開始計時
      if (!ctx_->nav_cmd_vel_settled()) {
        state_enter_time_ = ctx_->node->now();
        return false;
      }
      if ((ctx_->node->now() - state_enter_time_).seconds() < kForwardDurationSec) {
        publish_cmd_vel(kForwardLinearX, 0.0, 0.0);
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      RCLCPP_INFO(
        ctx_->node->get_logger(), "[Stage3] FORWARD 前進 %.2f m 完成",
        kForwardLinearX * kForwardDurationSec);
      enter_state(Stage3State::S3_NAV_TO_PLACE);
      return false;

    case Stage3State::S3_NAV_TO_PLACE:
      if (!navigate_to_named_pose(kPlacePose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_PLACE 抵達放置點");
      enter_state(Stage3State::S3_PLACE);
      return false;

    case Stage3State::S3_PLACE:
      if (kAutoPick) {
        publish_state_command(kCmdPlaceStraw, "S3_PLACE", "place_straw");
      } else if (!state_command_sent_) {
        // 人工夾取模式：手臂不能用，不送放置指令
        RCLCPP_WARN(ctx_->node->get_logger(), "[Stage3] PLACE 人工夾取模式，跳過放置指令");
        state_command_sent_ = true;
      }
      if ((ctx_->node->now() - state_enter_time_).seconds() < (kAutoPick ? kPlaceWaitSec : 0.0)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] PLACE 完成");
      enter_state(Stage3State::S3_AFTER_PLACE);
      return false;

    case Stage3State::S3_AFTER_PLACE:
      // 先後退 MoveBackDuration，停車後送 kCmdAfterPlace
      if ((ctx_->node->now() - state_enter_time_).seconds() < MoveBackDuration) {
        publish_cmd_vel(MoveBackLinearX, 0.0, 0.0);
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      publish_state_command(kCmdAfterPlace, "S3_AFTER_PLACE", "after_place");
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] AFTER_PLACE 後退完成，已送 %u", static_cast<unsigned>(kCmdAfterPlace));
      enter_state(Stage3State::S3_DONE);
      return false;

    case Stage3State::S3_DONE:
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] DONE 第三關完成");
      return true;

    case Stage3State::S3_FAILED:
      publish_cmd_vel(0.0, 0.0, 0.0);
      RCLCPP_ERROR_THROTTLE(
        ctx_->node->get_logger(), *ctx_->node->get_clock(), 2000, "[Stage3] FAILED");
      return false;
  }

  return false;
}
