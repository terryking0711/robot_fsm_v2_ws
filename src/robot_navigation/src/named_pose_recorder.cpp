#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include "robot_interfaces/srv/save_named_pose.hpp"

class NamedPoseRecorder : public rclcpp::Node
{
public:
  using SaveNamedPose = robot_interfaces::srv::SaveNamedPose;

  NamedPoseRecorder()
  : Node("named_pose_recorder"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("named_poses_file", "");
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("lookup_timeout_sec", 1.0);
    this->declare_parameter<int>("decimal_places", 4);

    this->declare_parameter<std::vector<std::string>>(
      "allowed_pose_names",
      std::vector<std::string>{
        "leave_start_zone",
        "stage1_entry",
        "stage2_entry"
      }
    );

    this->get_parameter("named_poses_file", named_poses_file_);
    this->get_parameter("global_frame", global_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("lookup_timeout_sec", lookup_timeout_sec_);
    this->get_parameter("decimal_places", decimal_places_);
    this->get_parameter("allowed_pose_names", allowed_pose_names_);

    save_named_pose_srv_ =
      this->create_service<SaveNamedPose>(
        "save_named_pose",
        std::bind(
          &NamedPoseRecorder::handle_save_named_pose,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );

    RCLCPP_INFO(this->get_logger(), "named_pose_recorder started.");
    RCLCPP_INFO(this->get_logger(), "named_poses_file: %s", named_poses_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "TF query: %s -> %s",
      global_frame_.c_str(),
      base_frame_.c_str()
    );
  }

private:
  void handle_save_named_pose(
    const std::shared_ptr<SaveNamedPose::Request> request,
    std::shared_ptr<SaveNamedPose::Response> response)
  {
    const std::string name = request->name;

    if (name.empty()) {
      response->success = false;
      response->message = "Pose name is empty.";
      return;
    }

    if (!allowed_pose_names_.empty()) {
      const auto it =
        std::find(allowed_pose_names_.begin(), allowed_pose_names_.end(), name);

      if (it == allowed_pose_names_.end()) {
        response->success = false;
        response->message =
          "Pose name is not allowed: " + name +
          ". Check allowed_pose_names parameter.";
        return;
      }
    }

    if (named_poses_file_.empty()) {
      response->success = false;
      response->message = "named_poses_file parameter is empty.";
      return;
    }

    geometry_msgs::msg::TransformStamped transform;

    try {
      transform =
        tf_buffer_.lookupTransform(
          global_frame_,
          base_frame_,
          tf2::TimePointZero,
          tf2::durationFromSec(lookup_timeout_sec_)
        );
    } catch (const tf2::TransformException& ex) {
      response->success = false;
      response->message =
        "Failed to lookup transform " + global_frame_ + " -> " + base_frame_ +
        ": " + std::string(ex.what());

      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    const double x = round_value(transform.transform.translation.x);
    const double y = round_value(transform.transform.translation.y);
    const double yaw = round_value(tf2::getYaw(transform.transform.rotation));

    if (!write_pose_to_yaml(name, x, y, yaw, response->message)) {
      response->success = false;
      return;
    }

    response->success = true;
    response->x = x;
    response->y = y;
    response->yaw = yaw;
    response->message =
      "Saved pose [" + name + "] x=" + std::to_string(x) +
      ", y=" + std::to_string(y) +
      ", yaw=" + std::to_string(yaw);

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  double round_value(double value) const
  {
    if (decimal_places_ < 0) {
      return value;
    }

    const double scale = std::pow(10.0, static_cast<double>(decimal_places_));
    return std::round(value * scale) / scale;
  }

  bool write_pose_to_yaml(
    const std::string& name,
    double x,
    double y,
    double yaw,
    std::string& error_message)
  {
    YAML::Node root;

    try {
      root = YAML::LoadFile(named_poses_file_);
    } catch (const YAML::Exception& ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to load YAML file. A new one will be created. Error: %s",
        ex.what()
      );

      root = YAML::Node(YAML::NodeType::Map);
    }

    if (!root["named_poses"] || !root["named_poses"].IsMap()) {
      root["named_poses"] = YAML::Node(YAML::NodeType::Map);
    }

    root["named_poses"][name]["x"] = x;
    root["named_poses"][name]["y"] = y;
    root["named_poses"][name]["yaw"] = yaw;

    YAML::Emitter emitter;
    emitter << root;

    if (!emitter.good()) {
      error_message = "Failed to serialize YAML.";
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return false;
    }

    std::ofstream fout(named_poses_file_);

    if (!fout.is_open()) {
      error_message = "Failed to open named_poses_file for writing: " + named_poses_file_;
      RCLCPP_ERROR(this->get_logger(), "%s", error_message.c_str());
      return false;
    }

    fout << emitter.c_str() << "\n";
    fout.close();

    return true;
  }

  std::string named_poses_file_;
  std::string global_frame_;
  std::string base_frame_;
  double lookup_timeout_sec_;
  int decimal_places_;
  std::vector<std::string> allowed_pose_names_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Service<SaveNamedPose>::SharedPtr save_named_pose_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NamedPoseRecorder>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}