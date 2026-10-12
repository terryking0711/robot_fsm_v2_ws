// straw_align_test：單獨測試「底盤 + 相機」的稻草捲對準，不需要 FSM / Nav2 / 定位。
//
// 控制律與 Stage3HayFSM::tick_sweep() 看到本排稻草時相同：
//   vy = -cam_lateral_gain * lateral_error_m（限幅 max_lateral_speed）
//   vx = distance_m < slow_down_dist_m ? slow_speed : fast_speed
// 預設參數與 stage3_hay_fsm.hpp 的常數一致；現場調好後記得把數值抄回 hpp。
//
// 模式：
//   mode:=lateral   只橫移對準（vx = 0），確認方向與收斂，對準後原地停住
//   mode:=approach  邊前進邊對準，碰到開關（switch_pressed）或 distance_m <= stop_dist_m 就停
//
//   ros2 run robot_fsm straw_align_test --ros-args -p mode:=lateral
//   ros2 run robot_fsm straw_align_test --ros-args -p mode:=approach -p fast_speed:=0.15
//
// 安全：看不到目標 / 目標超出閘門 / 相機資料過期 / 超過 max_run_sec 一律停車；
// Ctrl-C 時送一次 0 速度。測試時不要讓 Nav2 同時有 goal（會搶 /cmd_vel）。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_interfaces/msg/straw_pick_status.hpp"
#include "straw_interfaces/msg/straw_target.hpp"

using namespace std::chrono_literals;

class StrawAlignTest : public rclcpp::Node
{
public:
  StrawAlignTest()
  : Node("straw_align_test")
  {
    mode_ = declare_parameter<std::string>("mode", "lateral");
    fast_speed_ = declare_parameter<double>("fast_speed", 0.25);
    slow_speed_ = declare_parameter<double>("slow_speed", 0.08);
    slow_down_dist_m_ = declare_parameter<double>("slow_down_dist_m", 0.35);
    stop_dist_m_ = declare_parameter<double>("stop_dist_m", 0.15);
    cam_lateral_gain_ = declare_parameter<double>("cam_lateral_gain", 1.5);
    max_lateral_speed_ = declare_parameter<double>("max_lateral_speed", 0.15);
    lateral_gate_m_ = declare_parameter<double>("lateral_gate_m", 0.15);
    aligned_tol_m_ = declare_parameter<double>("aligned_tol_m", 0.01);
    target_stale_sec_ = declare_parameter<double>("target_stale_sec", 0.3);
    max_run_sec_ = declare_parameter<double>("max_run_sec", 20.0);
    const auto target_topic = declare_parameter<std::string>("straw_target_topic", "/straw/target");

    if (mode_ != "lateral" && mode_ != "approach") {
      RCLCPP_FATAL(get_logger(), "mode must be 'lateral' or 'approach', got '%s'", mode_.c_str());
      throw std::runtime_error("invalid mode");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    target_sub_ = create_subscription<straw_interfaces::msg::StrawTarget>(
      target_topic, rclcpp::SensorDataQoS(),
      [this](const straw_interfaces::msg::StrawTarget::SharedPtr msg) {
        target_ = *msg;
        target_time_ = now();
      });

    status_sub_ = create_subscription<robot_interfaces::msg::StrawPickStatus>(
      "/mechanism/straw_pick_status", rclcpp::SensorDataQoS(),
      [this](const robot_interfaces::msg::StrawPickStatus::SharedPtr msg) {
        switch_pressed_ = msg->switch_pressed;  // true = 按下（STM 韌體已改正，不再反轉）
      });

    start_time_ = now();
    timer_ = create_wall_timer(100ms, [this]() { tick(); });

    RCLCPP_WARN(
      get_logger(),
      "mode=%s fast=%.2f slow=%.2f slow_dist=%.2f stop_dist=%.2f gain=%.2f max_vy=%.2f gate=%.2f",
      mode_.c_str(), fast_speed_, slow_speed_, slow_down_dist_m_, stop_dist_m_,
      cam_lateral_gain_, max_lateral_speed_, lateral_gate_m_);
  }

  void stop()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  void publish(double vx, double vy)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    cmd_pub_->publish(msg);
  }

  void finish(const char* reason)
  {
    stop();
    RCLCPP_WARN(get_logger(), "STOP: %s", reason);
    finished_ = true;
  }

  void tick()
  {
    if (finished_) {
      stop();
      return;
    }

    if ((now() - start_time_).seconds() > max_run_sec_) {
      finish("max_run_sec reached");
      return;
    }

    if (mode_ == "approach" && switch_pressed_) {
      finish("switch_pressed = true（碰到稻草捲）");
      return;
    }

    if (!target_ || (now() - target_time_).seconds() > target_stale_sec_) {
      stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "no fresh /straw/target, hold still");
      return;
    }

    const auto& t = *target_;
    if (!t.valid || !t.has_depth) {
      stop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "target invalid (%s) or no depth, hold still",
        t.reason.c_str());
      return;
    }

    if (std::fabs(t.lateral_error_m) >= lateral_gate_m_) {
      stop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "lateral_error_m %+.3f outside gate %.2f (FSM 會當成別排的稻草), hold still",
        t.lateral_error_m, lateral_gate_m_);
      return;
    }

    if (mode_ == "approach" && t.distance_m <= stop_dist_m_) {
      finish("distance_m <= stop_dist_m");
      RCLCPP_WARN(get_logger(), "final lateral_error_m %+.3f", t.lateral_error_m);
      return;
    }

    const double vy = std::clamp(
      -cam_lateral_gain_ * t.lateral_error_m, -max_lateral_speed_, max_lateral_speed_);
    double vx = 0.0;
    if (mode_ == "approach") {
      vx = t.distance_m < slow_down_dist_m_ ? slow_speed_ : fast_speed_;
    }

    const bool aligned = std::fabs(t.lateral_error_m) < aligned_tol_m_;
    if (mode_ == "lateral" && aligned) {
      stop();
    } else {
      publish(vx, vy);
    }

    RCLCPP_INFO(
      get_logger(), "dist %.3f lat %+.3f -> vx %.2f vy %+.3f %s",
      t.distance_m, t.lateral_error_m, vx, (mode_ == "lateral" && aligned) ? 0.0 : vy,
      aligned ? "[aligned]" : "");
  }

  std::string mode_;
  double fast_speed_, slow_speed_, slow_down_dist_m_, stop_dist_m_;
  double cam_lateral_gain_, max_lateral_speed_, lateral_gate_m_, aligned_tol_m_;
  double target_stale_sec_, max_run_sec_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<straw_interfaces::msg::StrawTarget>::SharedPtr target_sub_;
  rclcpp::Subscription<robot_interfaces::msg::StrawPickStatus>::SharedPtr status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<straw_interfaces::msg::StrawTarget> target_;
  rclcpp::Time target_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  bool switch_pressed_{false};
  bool finished_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StrawAlignTest>();
  // Ctrl-C 後 spin() 返回時 context 已經 shutdown、不能再 publish，
  // 所以停車訊號要在 shutdown 之前的 callback 裡送
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
    [node]() { node->stop(); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
