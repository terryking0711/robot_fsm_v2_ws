#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "robot_interfaces/msg/mechanism_command.hpp"

using namespace std::chrono_literals;

class StmCommunicationNode : public rclcpp::Node
{
public:
  StmCommunicationNode()
  : Node("stm_communication_node")
  {
    // ===== Parameters =====
    this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("output_cmd_vel_topic", "/mecanum/cmd_vel");
    this->declare_parameter<double>("cmd_vel_rate_hz", 20.0);
    this->declare_parameter<double>("cmd_vel_stale_timeout_sec", 0.5);

    this->declare_parameter<bool>("enable_test_twist", false);
    this->declare_parameter<double>("test_linear_x", 0.05);
    this->declare_parameter<double>("test_linear_y", 0.0);
    this->declare_parameter<double>("test_angular_z", 0.0);
    this->declare_parameter<double>("test_duration_sec", 3.0);

    this->declare_parameter<std::string>("mechanism_topic", "/mechanism/command");
    this->declare_parameter<bool>("enable_mechanism_test_publish", false);
    this->declare_parameter<double>("mechanism_rate_hz", 1.0);
    this->declare_parameter<int>("mechanism_command_id", 1);
    this->declare_parameter<std::string>("mechanism_command_name", "test_mechanism");
    this->declare_parameter<std::string>(
      "mechanism_arg_json",
      R"({"source":"stm_communication_node","mode":"test"})"
    );

    this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
    this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
    this->get_parameter("cmd_vel_rate_hz", cmd_vel_rate_hz_);
    this->get_parameter("cmd_vel_stale_timeout_sec", cmd_vel_stale_timeout_sec_);

    this->get_parameter("enable_test_twist", enable_test_twist_);
    this->get_parameter("test_linear_x", test_linear_x_);
    this->get_parameter("test_linear_y", test_linear_y_);
    this->get_parameter("test_angular_z", test_angular_z_);
    this->get_parameter("test_duration_sec", test_duration_sec_);

    this->get_parameter("mechanism_topic", mechanism_topic_);
    this->get_parameter("enable_mechanism_test_publish", enable_mechanism_test_publish_);
    this->get_parameter("mechanism_rate_hz", mechanism_rate_hz_);
    this->get_parameter("mechanism_command_id", mechanism_command_id_);
    this->get_parameter("mechanism_command_name", mechanism_command_name_);
    this->get_parameter("mechanism_arg_json", mechanism_arg_json_);

    // ===== Publishers =====
    mecanum_cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        output_cmd_vel_topic_,
        10
      );

    mechanism_command_pub_ =
      this->create_publisher<robot_interfaces::msg::MechanismCommand>(
        mechanism_topic_,
        10
      );

    // ===== Subscriber: Nav2 cmd_vel input =====
    cmd_vel_sub_ =
      this->create_subscription<geometry_msgs::msg::Twist>(
        input_cmd_vel_topic_,
        10,
        std::bind(&StmCommunicationNode::cmd_vel_callback, this, std::placeholders::_1)
      );

    // ===== Timers =====
    const auto cmd_vel_period =
      std::chrono::duration<double>(1.0 / cmd_vel_rate_hz_);

    cmd_vel_timer_ =
      this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(cmd_vel_period),
        std::bind(&StmCommunicationNode::publish_mecanum_cmd_vel, this)
      );

    if (enable_mechanism_test_publish_) {
      const auto mechanism_period =
        std::chrono::duration<double>(1.0 / mechanism_rate_hz_);

      mechanism_timer_ =
        this->create_wall_timer(
          std::chrono::duration_cast<std::chrono::nanoseconds>(mechanism_period),
          std::bind(&StmCommunicationNode::publish_test_mechanism_command, this)
        );
    }

    start_time_ = this->now();
    last_cmd_vel_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "stm_communication_node started.");
    RCLCPP_INFO(this->get_logger(), "Subscribe cmd_vel from: %s", input_cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish mecanum cmd_vel to: %s", output_cmd_vel_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel publish rate: %.2f Hz", cmd_vel_rate_hz_);
    RCLCPP_INFO(this->get_logger(), "Mechanism topic: %s", mechanism_topic_.c_str());

    if (enable_mechanism_test_publish_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Mechanism test publish ENABLED. Make sure FSM is not publishing /mechanism/command at the same time."
      );
    }
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    latest_cmd_vel_ = *msg;
    last_cmd_vel_time_ = this->now();
    has_cmd_vel_ = true;
  }

  void publish_mecanum_cmd_vel()
  {
    geometry_msgs::msg::Twist output_msg;

    if (enable_test_twist_) {
      // ===== Manual slow test mode =====
      // 用於不啟動 Nav2 時，單獨測試 STM32 是否能接收 /mecanum/cmd_vel。
      // 預設只發 test_duration_sec 秒，之後自動送 0 速度，避免真機暴衝。
      const double elapsed = (this->now() - start_time_).seconds();

      if (elapsed <= test_duration_sec_) {
        output_msg.linear.x = test_linear_x_;
        output_msg.linear.y = test_linear_y_;
        output_msg.angular.z = test_angular_z_;
      } else {
        output_msg.linear.x = 0.0;
        output_msg.linear.y = 0.0;
        output_msg.angular.z = 0.0;
      }

      mecanum_cmd_vel_pub_->publish(output_msg);
      return;
    }

    // ===== Normal mode =====
    // 正式模式：接收 Nav2 發出的 /cmd_vel，固定頻率轉發到 /mecanum/cmd_vel。
    if (has_cmd_vel_) {
      const double age = (this->now() - last_cmd_vel_time_).seconds();

      if (age <= cmd_vel_stale_timeout_sec_) {
        output_msg = latest_cmd_vel_;
      } else {
        // 超過 timeout 沒收到新 /cmd_vel，就送 0 速度。
        output_msg.linear.x = 0.0;
        output_msg.linear.y = 0.0;
        output_msg.angular.z = 0.0;
      }
    } else {
      // 尚未收到任何 /cmd_vel，先送 0 速度。
      output_msg.linear.x = 0.0;
      output_msg.linear.y = 0.0;
      output_msg.angular.z = 0.0;
    }

    mecanum_cmd_vel_pub_->publish(output_msg);
  }

  void publish_test_mechanism_command()
  {
    robot_interfaces::msg::MechanismCommand msg;

    msg.command_id = static_cast<uint16_t>(mechanism_command_id_);
    msg.command_name = mechanism_command_name_;
    msg.arg_json = mechanism_arg_json_;

    mechanism_command_pub_->publish(msg);

    RCLCPP_INFO(
      this->get_logger(),
      "[Mechanism Test] id=%u, name=%s, arg_json=%s",
      msg.command_id,
      msg.command_name.c_str(),
      msg.arg_json.c_str()
    );
  }

  // Parameters
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  double cmd_vel_rate_hz_;
  double cmd_vel_stale_timeout_sec_;

  bool enable_test_twist_;
  double test_linear_x_;
  double test_linear_y_;
  double test_angular_z_;
  double test_duration_sec_;

  std::string mechanism_topic_;
  bool enable_mechanism_test_publish_;
  double mechanism_rate_hz_;
  int mechanism_command_id_;
  std::string mechanism_command_name_;
  std::string mechanism_arg_json_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mecanum_cmd_vel_pub_;
  rclcpp::Publisher<robot_interfaces::msg::MechanismCommand>::SharedPtr mechanism_command_pub_;

  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
  rclcpp::TimerBase::SharedPtr mechanism_timer_;

  // State
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time start_time_;
  bool has_cmd_vel_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<StmCommunicationNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}