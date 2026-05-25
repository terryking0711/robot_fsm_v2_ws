# TDK TOAST Robot — FSM Mission Controller

ROS2 競賽機器人的任務控制主程式，採用有限狀態機（FSM）架構管理完整的比賽流程。

---

## 專案簡介

本專案為 TDK TOAST 機器人競賽的 ROS2 主控程式，負責協調機器人在比賽中依序完成各關卡任務。

整體架構以 **非阻塞式 FSM（Finite State Machine）** 為核心：主循環每 100ms 呼叫一次 `tick()`，每個 state 只執行一小步後立即返回，不阻塞 ROS 的訊息接收。感測資料（視覺、機構回饋、定位）透過 ROS topic 非同步更新至共享的 `RobotContext`，各關卡 FSM 從中讀取，實現感知與決策的乾淨分離。

---
## 如何使用
```
1.開啟所有tdk_slam_ws需要的terminal(4個)
2.開啟navigation_server
3.ros2 run robot_fsm robot_fsm main  # 開啟主程式fsm
4.ros2 topic echo /mechanism/command # 看到topic內容
```
## 比賽流程

```
起始區 → 第二關（蛤蜊）→ 第三關（稻草堆疊）→ 第四關（媽祖）→ 第五關 → 終點
```

對應到程式的頂層狀態機：

```
BOOT → INIT → SELF_CHECK → WAIT_START
  → LEAVE_START_ZONE
  → STAGE2_CLAM  → TRANSITION_TO_STAGE3
  → STAGE3_HAY   → TRANSITION_TO_STAGE4
  → STAGE4_MAZU  → TRANSITION_TO_STAGE5
  → STAGE5_NEW
  → FINISH_DECISION → END_RUN
```

---

## 快速開始

### 環境需求

- ROS2 Humble（或更新版本）
- `rclcpp`, `rclcpp_action`, `geometry_msgs`
- Docker 環境請參考 [docker/](docker/)

### Build

```bash
cd ~/robot_fsm_v2_ws

# Step 1: 先 build 自訂介面
colcon build --packages-select robot_interfaces
source install/setup.bash

# Step 2: build 主程式
colcon build --packages-select robot_fsm
source install/setup.bash
```

### Run

```bash
ros2 run robot_fsm robot_fsm_main
```

程式啟動後會等待 5 秒模擬開始訊號，接著依序跑完所有關卡，最後印出 `END_RUN - 全部任務完成！` 後自動關閉。

### 預期 Log 輸出

```
[Mission] BOOT
[Mission] INIT
[Mission] SELF_CHECK
[Mission] waiting start signal...   ← 重複 5 秒
[Mission] start signal received
[Mission] navigating to 'leave_start_zone' (sim 5s)
[Mission] nav to 'leave_start_zone' complete
[Stage2] ENTER 第二關
[Stage2] APPROACH 模擬靠近蛤蜊
...
[Stage2] DONE 第二關完成
[Mission] navigating to 'stage3_entry' (sim 5s)
...
[Stage3] DONE 第三關完成
...
[Mission] END_RUN - 全部任務完成！
```
---
## 測試和其他workspace通訊
### Terminal(robot_fsm_v2_ws) 5 - 開啟 Navigation server
```bash
source install/setup.bash

ros2 launch robot_navigation navigation_server.launch.py
```

### 測試兩個navigation_server
```
ros2 action send_goal /navigate_to_named_pose robot_interfaces/action/NavigateToNamedPose "{target_name: 'stage1_entry', timeout_sec: 60.0}" --feedback
```
### 測試基本通訊
```
# 1. 確認 ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# 2. 確認 RMW
echo $RMW_IMPLEMENTATION

# 3. 確認網路模式（在 host 機器跑）
docker inspect tdk_slam --format='{{.HostConfig.NetworkMode}}'
docker inspect ros2_projects --format='{{.HostConfig.NetworkMode}}'

# 4. 測試 topic 互通（tdk_slam 發）
ros2 topic pub /test std_msgs/msg/String "data: 'hello'"

# 5. 測試 topic 互通（ros2_projects 收）
ros2 topic echo /test

# 6. 確認 Nav2 action server 存在
ros2 action list

# 7. 確認 bt_navigator 正常
ros2 action info /navigate_to_pose

# 8. 直接 call Nav2 測試
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```
---

## 專案結構

```
robot_fsm_v2_ws/
├── src/
│   ├── robot_interfaces/          # 自訂 ROS2 訊息與 Action 定義
│   │   ├── action/
│   │   │   └── NavigateToNamedPose.action
│   │   └── msg/
│   │       ├── MechanismCommand.msg
│   │       ├── MechanismFeedback.msg
│   │       ├── VisionSceneState.msg
│   │       └── StageResult.msg
│   │
│   └── robot_fsm/                 # 主程式
│       ├── src/
│       │   ├── main.cpp                      # ROS node、subscribers、主循環
│       │   ├── mission/
│       │   │   └── mission_controller.cpp    # 頂層 FSM，協調各關卡
│       │   └── stages/
│       │       ├── stage2_clam_fsm.cpp       # 第二關：蛤蜊
│       │       ├── stage3_hay_fsm.cpp        # 第三關：稻草堆疊
│       │       ├── stage4_mazu_fsm.cpp       # 第四關：媽祖
│       │       └── stage5_new_fsm.cpp        # 第五關
│       └── include/robot_fsm/
│           ├── common/
│           │   └── robot_context.hpp         # 全域共享資料結構
│           ├── mission/
│           │   ├── mission_controller.hpp
│           │   └── mission_states.hpp        # 頂層 MissionState enum
│           └── stages/
│               ├── stage_common.hpp          # StageStatus enum（RUNNING/SUCCESS/FAILURE）
│               ├── stage2_clam_fsm.hpp / stage2_clam_states.hpp
│               ├── stage3_hay_fsm.hpp  / stage3_hay_states.hpp
│               ├── stage4_mazu_fsm.hpp / stage4_mazu_states.hpp
│               └── stage5_new_fsm.hpp
│
├── docker/                        # 開發環境容器設定
├── ARCHITECTURE.md                # 詳細架構流程圖
└── README.md
```

---

## 核心設計說明

### RobotContext — 共享資料中心

```cpp
struct RobotContext {
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<MechanismCommand>::SharedPtr mechanism_cmd_pub;

    std::optional<VisionSceneState>   latest_vision_state;
    std::optional<MechanismFeedback>  latest_mechanism_feedback;
    std::optional<PoseStamped>        latest_final_pose;
    bool start_signal = false;
};
```

所有 FSM 都持有同一個 `ctx` 指標。`main.cpp` 的 subscriber callback 負責寫入，各 FSM 的 `tick()` 負責讀取，兩者透過 `data_mutex` 保護。

### 非阻塞 FSM Pattern

```cpp
// ✅ 正確：每次只做一小步，立即返回
bool Stage3HayFSM::tick() {
    switch (state_) {
        case S3_NAV_TO_PICK:
            if (wait_ticks(50)) state_ = S3_PICK_HAY;
            return false;  // 下一個 tick 再來
        case S3_DONE:
            return true;   // 通知上層完成
    }
}

// ❌ 錯誤：阻塞式等待會凍結整個 ROS 主循環
void bad_example() {
    std::this_thread::sleep_for(5s);  // 不要這樣做
}
```

### 導航模擬 → 真實替換

目前 `transition_to_named_pose` 用計時器模擬 5 秒導航，替換成真實 Nav2 時只需修改此一函式：

```cpp
// 目前（模擬）
bool MissionController::transition_to_named_pose(...) {
    if (!nav_goal_sent_) {
        nav_start_time_ = ctx_->node->now();
        nav_goal_sent_ = true;
        return false;
    }
    return (ctx_->node->now() - nav_start_time_).seconds() >= 5.0;
}

// 未來（真實 Nav2 Action Client）
// → 改為 ctx_->nav_client->async_send_goal(goal, options)
//   並在 result_callback 中設 nav_done_ = true
```

---

## ROS2 介面定義

### Topics

| 方向 | Topic | 訊息類型 | 說明 |
|------|-------|----------|------|
| Subscribe | `/vision/scene_state` | `VisionSceneState` | 視覺系統偵測結果 |
| Subscribe | `/mechanism/feedback` | `MechanismFeedback` | 機構執行回饋 |
| Subscribe | `/final_pose` | `PoseStamped` | 精確定位結果 |
| Publish | `/mechanism/command` | `MechanismCommand` | 發送機構控制指令 |

### Action（預留，導航模擬期間未使用）

| Action | 類型 | 說明 |
|--------|------|------|
| `navigate_to_named_pose` | `NavigateToNamedPose` | 導航至指定位置名稱 |

---

## 開發狀態

| 關卡 | 狀態 | 說明 |
|------|------|------|
| MissionController 頂層 FSM | 完成 | 完整狀態切換邏輯 |
| Stage2 蛤蜊 | Stub | 子狀態架構已建立，待填入實際邏輯 |
| Stage3 稻草堆疊 | Stub | 子狀態架構已建立，待填入實際邏輯 |
| Stage4 媽祖 | Stub | 子狀態架構已建立，待填入實際邏輯 |
| Stage5 | Stub | 子狀態架構已建立，待填入實際邏輯 |
| 真實導航（Nav2） | 未接入 | `transition_to_named_pose` 預留替換介面 |
| 視覺系統 | 未接入 | ctx.latest_vision_state 預留 |
| 機構控制 | 未接入 | ctx.latest_mechanism_feedback 預留 |

詳細架構流程圖請見 [ARCHITECTURE.md](ARCHITECTURE.md)。
