// hpp 的目的是告訴電腦「有什麼東西可以用」
// 功能：定義類別（Class）結構、函式原型（Function Prototype）、全域變數宣告以及結構體。
// 用途：讓其他的 .cpp 檔案只要 #include 它，就能知道該類別有哪些方法可以呼叫，而不需要知道背後的程式碼怎麼寫。
#pragma once

#include <memory>

#include "robot_fsm/common/robot_context.hpp"
#include "robot_fsm/mission/mission_states.hpp"

class Stage3HayFSM;

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

  std::shared_ptr<RobotContext> ctx_;
  MissionState state_;

  std::shared_ptr<Stage3HayFSM> stage3_fsm_;

  bool nav_goal_sent_;
  bool nav_done_;
  bool nav_success_;
};