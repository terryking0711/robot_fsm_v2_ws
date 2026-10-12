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
  place_yaw_(0.0),
  place_rotate_ccw_(true),
  row_idx_(0),
  row_start_time_(ctx_->node->now()),
  row_reached_end_(false),
  empty_ticks_(0),
  pick_trigger_ready_(true),
  wait_pick_start_count_(0),
  pick_started_(false),
  backoff_count_(0),
  place_stm_idle_(false),
  straw_seen_(false),
  busy_hold_(false),
  aligned_(false),
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
    const bool last_row = i == kRowCount;
    double start_x, start_y, start_yaw, end_x, end_y, end_yaw;
    std::string err;
    if (!resolve_named_pose(poses, start_name, true, start_x, start_y, start_yaw, err) ||
      !resolve_named_pose(poses, end_name, last_row, end_x, end_y, end_yaw, err))
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

    if (last_row) {
      // 最後一排的 end 是放置點（yaw = 掃描方向）。放置前原地轉 stage3_place_rotate 的 dyaw，
      // dyaw 的正負號決定方向：> 0 逆時針、< 0 順時針
      const YAML::Node rot = poses[kPlaceRotatePose];
      const double dyaw = rot && rot["dyaw"] ? rot["dyaw"].as<double>() : 0.0;
      if (!rot || !rot["relative_to"] || rot["relative_to"].as<std::string>() != end_name ||
        std::fabs(dyaw) < 1e-3)
      {
        RCLCPP_ERROR(
          node->get_logger(),
          "[Stage3] '%s' must be relative_to '%s' with non-zero dyaw (> 0 ccw, < 0 cw) in %s",
          kPlaceRotatePose, end_name.c_str(), file.c_str());
        return;
      }
      double rx, ry, ryaw;
      if (!resolve_named_pose(poses, kPlaceRotatePose, true, rx, ry, ryaw, err)) {
        RCLCPP_ERROR(node->get_logger(), "[Stage3] %s in %s", err.c_str(), file.c_str());
        return;
      }
      place_yaw_ = ryaw;
      place_rotate_ccw_ = dyaw > 0.0;

      RCLCPP_INFO(
        node->get_logger(), "[Stage3] place rotate %+.3f rad (%s) -> yaw %.3f",
        dyaw, place_rotate_ccw_ ? "ccw" : "cw", place_yaw_);
    }
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
    row_reached_end_ = true;
    enter_state(Stage3State::S3_ROW_DONE);
    return false;
  }

  // 4. 相機
  const auto target = fresh_straw_target();
  // 看到本排下一捲：valid + has_depth + 橫向在閘門內（超過閘門的當成隔壁排）
  const bool cam_ok =
    target && target->valid && target->has_depth &&
    std::fabs(target->lateral_error_m) < kCamLateralGateM;

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
  } else if (!cam_ok) {
    std::snprintf(cam_state, sizeof(cam_state), "off(gate lat=%+.2f)", target->lateral_error_m);
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

  // 最後一排一定要開到排尾（= 放置點），不提早結束
  const bool last_row = row_idx_ + 1 >= rows_.size();
  if (kEnableEmptyRowExit && !last_row && empty_ticks_ >= kEmptyRowTicks) {
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

bool Stage3HayFSM::tick_rotate_at_place()
{
  const auto logger = ctx_->node->get_logger();
  auto& clock = *ctx_->node->get_clock();

  // 剛結束導航時 velocity_smoother 還在發速度，等它停下來再開始計時
  if (!ctx_->nav_cmd_vel_settled()) {
    state_enter_time_ = ctx_->node->now();
    return false;
  }

  // 最後一捲可能還在夾：一定等 STM 回 IDLE 再轉向放置（status 斷線也一直等）
  if (!place_stm_idle_) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    const auto status = fresh_pick_status();
    const bool idle =
      status && status->phase != robot_interfaces::msg::StrawPickStatus::PHASE_PICKING;
    if (!idle) {
      RCLCPP_INFO_THROTTLE(
        logger, clock, 2000, "[Stage3] ROTATE_AT_PLACE 等 STM 夾完最後一捲（%.0f s）%s",
        (ctx_->node->now() - state_enter_time_).seconds(),
        status ? "" : "（no fresh /mechanism/straw_pick_status）");
      return false;
    }
    place_stm_idle_ = true;
    rotate_settle_start_.reset();
    state_enter_time_ = ctx_->node->now();  // 轉向超時從這裡開始算
  }

  if ((ctx_->node->now() - state_enter_time_).seconds() > kRotateTimeoutSec) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN(logger, "[Stage3] ROTATE_AT_PLACE timeout (%.0f s)，直接放置", kRotateTimeoutSec);
    enter_state(Stage3State::S3_PLACE);
    return false;
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  if (!get_robot_pose(x, y, yaw)) {
    publish_cmd_vel(0.0, 0.0, 0.0);
    RCLCPP_WARN_THROTTLE(logger, clock, 1000, "[Stage3] ROTATE_AT_PLACE no fresh /robot_pose, hold still");
    return false;
  }

  const double err = normalize_angle(place_yaw_ - yaw);
  if (std::fabs(err) < kRotateTolRad) {
    // 停車後連續 kRotateSettleSec 都在容許內才放置；滑出去的話下面會修回來
    publish_cmd_vel(0.0, 0.0, 0.0);
    const auto now = ctx_->node->now();
    if (!rotate_settle_start_) {
      rotate_settle_start_ = now;
    }
    if ((now - *rotate_settle_start_).seconds() < kRotateSettleSec) {
      return false;
    }
    RCLCPP_INFO(
      logger, "[Stage3] ROTATE_AT_PLACE 轉到放置朝向 yaw %.3f（目標 %.3f，誤差 %+.3f）",
      yaw, place_yaw_, err);
    enter_state(Stage3State::S3_PLACE);
    return false;
  }
  rotate_settle_start_.reset();

  double wz = 0.0;
  if (std::fabs(err) > kRotateSlowRad) {
    // 還很遠：照指定方向轉（就算反方向比較近也一樣）
    wz = place_rotate_ccw_ ? kRotateMaxSpeed : -kRotateMaxSpeed;
  } else {
    wz = std::clamp(kRotateGain * err, -kRotateMaxSpeed, kRotateMaxSpeed);
    if (std::fabs(wz) < kRotateMinSpeed) {
      wz = std::copysign(kRotateMinSpeed, err);
    }
  }
  publish_cmd_vel(0.0, 0.0, wz);

  RCLCPP_INFO_THROTTLE(
    logger, clock, 500, "[Stage3] ROTATE_AT_PLACE yaw %.3f -> %.3f (%s) err %+.3f wz %+.2f",
    yaw, place_yaw_, place_rotate_ccw_ ? "ccw" : "cw", err, wz);
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
      enter_state(Stage3State::S3_CROSS);
      return false;

    case Stage3State::S3_CROSS: {
      // 剛結束導航時 velocity_smoother 還在發速度，等它停下來再開始衝
      if (!ctx_->nav_cmd_vel_settled()) {
        return false;
      }
      // 主迴圈 10 Hz、每 tick 才發一次的話，STM 收到的指令間隔太稀，底盤會一頓一頓衝不到速。
      // 衝田埂很短，直接在這裡以 kCrossPublishHz 固定間隔連續發完
      // （會擋住主迴圈 kCrossDurationSec；面板 STOP 的 SIGINT 會讓 rclcpp::ok() 變 false 跳出）。
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
      row_idx_ = 0;
      enter_state(Stage3State::S3_NAV_ROW_START);
      return false;
    }

    case Stage3State::S3_NAV_ROW_START:
      if (!navigate_to_named_pose(rows_[row_idx_].start_pose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_ROW_START 抵達第 %zu 排起點", row_idx_ + 1);
      enter_state(Stage3State::S3_ROW_BEGIN);
      return false;

    case Stage3State::S3_ROW_BEGIN:
      // 等 velocity_smoother 停下來才進 SWEEP，避免兩邊的 /cmd_vel 交錯
      if (!ctx_->nav_cmd_vel_settled()) {
        return false;
      }
      publish_cmd_vel(0.0, 0.0, 0.0);
      // 第 1 排：已經衝過田埂、Nav2 也抵達 stage3_row1_start 並停穩，這時才通知 STM
      // 開始「碰到開關就自動夾取」（STM 收到 320 才 armed），避免 CROSS / 導航途中誤碰觸發。
      // 不等 STM 確認，在 kEnablePickSendTicks 個 tick 內重送幾次，避免 micro-ROS 掉封包。
      if (kAutoPick && row_idx_ == 0 && tick_count_ < kEnablePickSendTicks) {
        if (tick_count_ % kEnablePickResendTicks == 0) {
          send_mechanism_command(kCmdEnableAutoPick, "S3_ROW_BEGIN", "enable_auto_pick");
        }
        tick_count_++;
        return false;
      }
      row_start_time_ = ctx_->node->now();
      row_reached_end_ = false;
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
      const bool last_row = row_idx_ + 1 >= rows_.size();

      if (last_row) {
        // 最後一排開到排尾時，機器已經在放置點，不用再導航
        bool at_place = row_reached_end_;
        double x = 0.0, y = 0.0, yaw = 0.0;
        if (!at_place && get_robot_pose(x, y, yaw)) {
          const Stage3Row& row = rows_.back();
          const double place_x = row.start_x + row.dir_x * row.length;
          const double place_y = row.start_y + row.dir_y * row.length;
          at_place = std::hypot(x - place_x, y - place_y) < kAtPlaceTolM;
        }
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3] ROW_DONE 4 排掃描結束，%s",
          at_place ? "已在放置點，原地轉向" : "導航到放置點");
        enter_state(at_place ? Stage3State::S3_ROTATE_AT_PLACE : Stage3State::S3_NAV_TO_PLACE);
      } else {
        RCLCPP_INFO(
          ctx_->node->get_logger(),
          "[Stage3] ROW_DONE 退回第 %zu 排起點", row_idx_ + 1);
        enter_state(Stage3State::S3_NAV_ROW_RETURN);
      }
      return false;
    }

    case Stage3State::S3_NAV_ROW_RETURN:
      if (!navigate_to_named_pose(rows_[row_idx_].start_pose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(
        ctx_->node->get_logger(), "[Stage3] NAV_ROW_RETURN 退回第 %zu 排起點，側移到第 %zu 排",
        row_idx_ + 1, row_idx_ + 2);
      row_idx_++;
      enter_state(Stage3State::S3_NAV_ROW_START);
      return false;

    case Stage3State::S3_NAV_TO_PLACE:
      // Nav2 到排尾點（yaw = 掃描方向），轉向交給 ROTATE_AT_PLACE 才能指定方向
      if (!navigate_to_named_pose(rows_.back().end_pose, 30.0f)) {
        return false;
      }
      RCLCPP_INFO(ctx_->node->get_logger(), "[Stage3] NAV_TO_PLACE 抵達放置點");
      enter_state(Stage3State::S3_ROTATE_AT_PLACE);
      return false;

    case Stage3State::S3_ROTATE_AT_PLACE:
      return tick_rotate_at_place();

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
