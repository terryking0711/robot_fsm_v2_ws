# TDK 30th — Robot FSM

TDK 30 屆競賽機器人的任務主程式，以 **ROS 2 Humble、C++ 與非阻塞有限狀態機（FSM）**，協調定位導航、視覺辨識及 STM32 機構控制，完成關卡移動、稻草收集與放置。

本專案保留作為個人作品與後續開發參考。設計重點是把「任務順序」「移動控制」「機構與感知回授」分開管理，再由 FSM 決定何時執行、等待、切換或停車。以下以實機使用的第二、三關流程為主，其他 workspace 僅介紹與 FSM 的整合關係。

## 1. 整體系統架構

```mermaid
flowchart TB
    UI["robot_panel_ui<br/>監控、START / STOP / RESET"]
    subgraph THIS["robot_fsm_v2_ws"]
        PANEL["robot_panel_bridge<br/>程序管理、比賽倒數"]
        FSM["robot_fsm<br/>MissionController → 關卡子 FSM"]
        NAV["robot_navigation<br/>命名點位 → Nav2 action"]
        DIRECT["閉迴路直接控速<br/>目標誤差 → 車體速度"]
    end
    SLAM["tdk_slam_ws<br/>Cartographer、TF、Nav2"]
    VISION["TDK-straw<br/>稻草目標與對準誤差"]
    BRIDGE["速度橋接 + micro-ROS Agent"]
    STM["STM32<br/>底盤與機構韌體"]

    UI <-->|"rosbridge：操作與狀態"| PANEL
    PANEL -->|"啟動 / 重置 / 停止"| FSM
    FSM -->|"命名目標"| NAV
    NAV <-->|"導航 action / 結果"| SLAM
    SLAM -->|"位姿與定位初始化結果"| FSM
    FSM -->|"定位初始化請求"| SLAM
    FSM -->|"目標與控制條件"| DIRECT
    SLAM -->|"/robot_pose"| DIRECT
    VISION -->|"/straw/target"| FSM
    SLAM -->|"Nav2 速度"| BRIDGE
    DIRECT -->|"直接控制速度"| BRIDGE
    FSM <-->|"機構指令 / 夾取狀態，經 micro-ROS"| STM
    BRIDGE <-->|"速度命令 / 里程計"| STM
```

FSM 是決策中心：根據目前關卡、導航結果、定位、視覺與機構狀態決定下一步。Nav2 與直接控速用於不同移動段落，依任務狀態切換；面板透過後端操作 FSM，底層定位、感測器與 micro-ROS 則由配套系統啟動。

## 2. 通訊、套件與跨 workspace 整合

### FSM 與套件分工

主程式以 **10 Hz tick** 推進狀態，每次只執行一小步；等待導航或機構時仍處理 ROS callbacks。`RobotContext` 保存最新回授與共用設定，`MissionController` 管理整體流程，第二、三關子 FSM 管理關卡內動作。

| 套件 | 職責與主要技術 |
| --- | --- |
| `robot_fsm` | C++／rclcpp 非阻塞 FSM、機構指令、視覺與夾取回授、直接控速、速度橋接 |
| `robot_navigation` | rclcpp_action 封裝 Nav2、named poses YAML、逾時與精對位、Python 點位視覺化工具 |
| `robot_interfaces` | 自訂機構訊息、夾取狀態及 `NavigateToNamedPose.action`，定義模組間介面 |
| `robot_panel_bridge` | Python／rclpy services、FSM 子程序管理、比賽倒數與主機電池監控 |
| `straw_interfaces` | 本 workspace 內另附的稻草訊息套件，供 FSM 接收 TDK-straw 偵測結果 |

### 跨 repo 關係

| Repo／workspace | 與 FSM 的關係 |
| --- | --- |
| [tdk_slam_ws](https://github.com/terryking0711/tdk_slam_ws/tree/tmep_ver) | 提供 Cartographer 定位、TF、`/robot_pose`、定位初始化與 Nav2（Smac2D + MPPI Omni） |
| `TDK-straw` | 使用 OpenCV 的 HSV 色彩與幾何分析偵測稻草，透過 `/straw/target` 提供第三關對準資訊 |
| `microros_ws` | 提供 micro-ROS Agent 與相關容器服務，銜接 ROS 2 和 STM32；韌體負責實際底盤及機構執行 |
| [robot_panel_ui](https://github.com/terryking0711/robot_panel_ui) | 透過 rosbridge 顯示狀態、呼叫 launcher services，不直接實作關卡邏輯 |
| [tdk_robot_docker](https://github.com/terryking0711/tdk_robot_docker) | 統整 Docker、桌面啟動腳本、`field_config.yaml` 與跨 workspace 執行環境 |

<!-- 在此加入 TDK-straw 辨識圖片／實機影片 -->

通訊依用途選擇：**topics** 傳送位姿、視覺、速度與機構狀態；**actions** 處理導航目標與結果；**services** 處理啟停、重置與點位重載；**TF** 統一場地、地圖、里程計與車體座標。STM32 透過 micro-ROS 參與 ROS 通訊。

### 移動控制：Nav2 與自行實作的閉迴路控速

開發初期使用 Nav2 執行點位移動，但在本麥輪機器人與競賽場地上，移動表現不易調整到符合需求，調參成本高。後來針對固定點位與關卡移動實作 **`DirectTransition` 閉迴路控速**，在實機使用上更容易取得需要的移動表現。

#### 使用 nav2 的實機效果

https://github.com/user-attachments/assets/20d47f27-70ef-4f53-84d4-dda88f34d5d7

#### 使用控速的實際效果

https://github.com/user-attachments/assets/02b318e7-a96c-47ff-a192-95f5787f165d


控制器持續讀取 `/robot_pose`，計算 world 座標下的位置與朝向誤差，再轉換成車體速度送到 `/cmd_vel`。可依目標設定速度上限、位置／角度容差、穩定週期與逾時策略，並依設定限制加減速、接近目標時減速。位姿過期時停車；達成條件後由 FSM 接續機構或下一段移動。

| 控制方式 | 在 FSM 中的用途 |
| --- | --- |
| Nav2 | 需要路徑規劃的導航段落，由 `navigation_server` 管理目標與結果 |
| `DirectTransition` | 固定入口、轉角與放置點等段落，利用定位回授直接控制；mirror／DIT 多個段落採此方式 |
| 關卡內控速 | 第三關掃排、換排與視覺對準，結合位姿、目標與機構回授決定速度及停車時機 |

直接控速仍依賴定位品質與場地路徑安排，沒有 Nav2 同等的路徑規劃與動態避障功能。兩種方式由 FSM 依段落使用，這也是本專案從通用導航轉向任務導向控制的重要設計經驗。

<!-- 在此加入 Nav2 與閉迴路控速的實機對比影片，可附場地與測試條件 -->

## 3. 開發與視覺化工具

### 地圖、點位與移動疊圖

`plot_named_poses.py` 不需要啟動 ROS，讀取場地地圖、world/map 轉換、named poses 與部分 FSM 移動常數，輸出 PNG。需有 Python 的 PyYAML、Matplotlib 與 Pillow。

```bash
cd /workspaces/robot_fsm_v2_ws
# 產生所有場地疊圖，或指定場地與輸出目錄
python3 src/robot_navigation/scripts/plot_named_poses.py
python3 src/robot_navigation/scripts/plot_named_poses.py mirror dit -o /tmp/fsm_maps
# 互動檢視；--full-map 可顯示整張地圖
python3 src/robot_navigation/scripts/plot_named_poses.py mirror --show
```

### mirror 場地的 named_pose 圖片
![image](https://hackmd.io/_uploads/B1ZBA8M3Gx.png)

### normal 場地的 named_pose 圖片
![image](https://hackmd.io/_uploads/Syz6RIG3Ge.png)

圖中的定時移動是依控制常數估算的**名目軌跡**，掃排線為參考線，並非實機量測軌跡，也不保證涵蓋所有路線變體。適合檢查座標、朝向與動作方向，實際路徑仍需錄製定位資料驗證。

<!-- 在此加入地圖＋座標點＋移動疊圖 -->

### Foxglove／Lichtblick 即時開發

在 Foxglove 或 Lichtblick 選擇 Foxglove WebSocket，連線至 `ws://<機器人IP>:8767`。既有環境另提供 Lichtblick 網頁入口 `http://<機器人IP>:8080`；bridge 由配套 micro-ROS 容器堆疊啟動。
#### 測試時的 foxglove 畫面


https://github.com/user-attachments/assets/8f405bbf-3dc4-4d92-a229-3eca00532e06


3D panel 的 display frame 設為 `world`，加入 `/map`、TF、`/robot_pose` 與 `/named_poses_markers`，即可對照地圖、機器人位置與目標點。點位 markers 可手動啟動：

```bash
python3 /workspaces/robot_fsm_v2_ws/src/robot_navigation/scripts/named_poses_markers.py \
  --ros-args -p field_side:=mirror
```

此工具偵測 YAML 存檔後自動更新圖示；`/workspaces/start.sh` 已包含它，整合啟動時不必重複執行。**圖示更新不等於導航點位已重載**，`navigation_server` 另提供 `/reload_named_poses`。

<!-- 在此加入 Foxglove／Lichtblick 開發畫面 -->

## 4. 快速啟動

以下以既有 Docker 掛載路徑 `/workspaces` 為例，需要相關 workspace、地圖、場地設定與硬體環境。系統使用 `ROS_DOMAIN_ID=30`。

```bash
# ROS 容器內：重新建置 FSM（會清除本 workspace 的 build/install/log）
cd /workspaces/robot_fsm_v2_ws
./build_fsm.sh
source ./use_fsm.sh
```

整合使用時先啟動 `TDK MicroROS` 與視覺，再啟動 `TDK Loc_Normal`／`TDK Loc_Mirror`／`TDK Loc_DIT`，最後開啟 robot_panel 按對應 START。定位啟動腳本會帶起定位、Nav2、launcher 與點位 markers；START 再啟動 FSM 及 navigation_server。

已有定位導航與通訊後端時，也可在容器內直接啟動：

```bash
export ROS_DOMAIN_ID=30
source /workspaces/robot_fsm_v2_ws/use_fsm.sh
ros2 launch robot_fsm robot_bringup.launch.py field_side:=mirror skip_stage2_arm:=true
```

直接 launch 不會建立面板 launcher 的比賽倒數；比賽計時與 RESET 操作請使用面板整合入口。操作說明另見 [Robot Panel UI](https://github.com/terryking0711/robot_panel_ui)。

## 5. FSM 與整體實作細節

### 任務流程與回授同步

```mermaid
flowchart LR
    INIT["初始化／檢查"] --> LOC["沿用現有定位"]
    LOC --> WAIT["等待開始"]
    WAIT --> S2["第二關準備與蛤蜊流程"]
    S2 --> MOVE["轉角／第三關入口"]
    WAIT -->|"only_stage3 / skip_stage2"| MOVE
    MOVE --> S3["第三關初始化、跨越與收集"]
    S3 --> PLACE["approach → place → 放置"]
    PLACE --> END["任務結束"]
```

- **頂層控制**：處理關卡切換、導航成功／失敗與停止條件，讓關卡子 FSM 專注於動作順序。目前啟動流程沿用既有 Cartographer 定位，`LOCALIZE` 狀態直接放行；程式保留定位初始化請求與結果介面。
- **第二關**：機構準備、入口移動、推蛤蜊、抬臂／收臂與返回；實機面板使用 `skip_stage2_arm` 分支略過部分手臂流程。
- **第三關**：初始化、跨越、進排、掃描對準、等待夾取、退回與換排，最後返回放置。移動與夾取回授互相約束，STM 狀態過期或仍在 PICKING 時停車等待。
- **放置與時間管理**：收集期間剩餘時間小於 40 秒時停止找下一捲，等當次夾取完成，經 `stage3_place_approach` 再到 `stage3_place`。倒數歸零不會直接中斷機構動作。

第三關程式位於 `src/robot_fsm/src/stages/stage3_hay_fsm.cpp`，相關專門說明見 [STAGE3_README.md](STAGE3_README.md)；主流程與條件以目前程式為準。

### 場地與控制設定

| 設定 | 位置／用途 |
| --- | --- |
| 場地入口 | `/workspaces/field_config.yaml`：選擇地圖、TF、點位與 FSM 設定檔 |
| 命名點位 | `src/robot_navigation/config/named_poses_{normal,mirror,dit}.yaml` |
| 控速與指令映射 | `src/robot_fsm/config/fsm_side_{normal,mirror}.yaml`；DIT 共用 mirror 控制設定 |
| 直接控速 | `direct_transition.<目標>`：速度、容差、穩定週期及逾時策略，支援段落的加減速設定 |
| 掃排／換排 | `stage3_row_return_speed`、`stage3_row_return_slow_speed`、`stage3_row_shift_max_speed` 等；部分常數仍在 C++ header |

normal／mirror 使用各自地圖與點位；DIT 為實驗室第三關測試場地。MIRROR V2 使用 mirror 地圖，改走第三關 row3 → row1 → row2，原 MIRROR 為 row4 → row3 → row2 → row1。

### RESET 與比賽倒數

launcher 保存截止時間與路線選擇。STOP／RESET 持續扣除原倒數，RESET 不重建定位；S2 從第二關準備開始，S3 經轉角及第三關入口重新進關。重新 START 才設定新期限，重啟 launcher 則會遺失原有記憶體狀態。

目前 normal／mirror 設為 240 秒，DIT 保留 120 秒測試設定，定義於 `fsm_launcher.py`；移植時可依需求修改。

### TF tree 與座標約定

命名點位與 FSM 定位目標使用 **world frame**，以場地左下角為原點。`world → map` 的平移與旋轉由場地設定提供；目前設定為 x=0.425 m、y=1.0 m、yaw=0。

```mermaid
graph TD
    world["world：場地座標"] -->|"static_transform_publisher：依場地設定"| map["map：定位地圖座標"]
    map -->|"Cartographer pure localization"| odom["odom：里程計座標"]
    odom -->|"odom_tf_broadcaster：/odom pose"| base["base_footprint：底盤中心"]
    base -->|"robot_state_publisher／URDF"| front["laser_front"]
    base -->|"robot_state_publisher／URDF"| rear["laser_rear"]
```

定位與 TF 由 `tdk_slam_ws` 維護，FSM 利用轉換後的 `/robot_pose` 做控制。圖中保留實機使用的 odom TF 轉發方式。

### 主要 topics

方向以 FSM 系統為基準；速度橋接與監控節點另於用途中標示。

| Topic | 型別 | 方向與用途 |
| --- | --- | --- |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 輸入：world 位姿，供直接控速與關卡判斷 |
| `/init_pose_cmd` | `geometry_msgs/msg/PoseStamped` | 輸出：保留的定位初始化請求介面 |
| `/init_pose_status` | `std_msgs/msg/Bool` | 輸入：保留的定位初始化結果介面 |
| `/straw/target` | `straw_interfaces/msg/StrawTarget` | 輸入：TDK-straw 視覺目標與誤差 |
| `/mechanism/command` | `robot_interfaces/msg/MechanismCommand` | 輸出：機構動作指令 |
| `/mechanism/straw_pick_status` | `robot_interfaces/msg/StrawPickStatus` | 輸入：夾取與機構狀態 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 底盤速度輸出，來自當前使用的導航／直接控制流程 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Nav2 速度鏈與 navigation_server 精對位使用 |
| `/mecanum/cmd_vel` | `geometry_msgs/msg/Twist` | 標準速度橋接送往底盤，20 Hz；輸入過期自動送零 |
| `/odom` | `nav_msgs/msg/Odometry` | STM32 里程計，供定位導航系統使用 |
| `/fsm/match_remaining` | `std_msgs/msg/Float64` | launcher 發布剩餘秒數，供面板顯示 |
| `/host/battery` | `sensor_msgs/msg/BatteryState` | battery_monitor 發布主機電量 |
| `/named_poses_markers` | `visualization_msgs/msg/MarkerArray` | 點位工具發布的視覺化標記 |

程式另外保留 `/vision/scene_state`、`/mechanism/feedback` 與 `/final_pose` 訂閱介面；目前主要視覺與夾取整合使用上表的 `/straw/target` 和 `/mechanism/straw_pick_status`。

### 主要 actions 與 services

| 介面 | 型別 | 用途 |
| --- | --- | --- |
| `/navigate_to_named_pose` | `robot_interfaces/action/NavigateToNamedPose` | 接收 `target_name` 與 `timeout_sec`，回報成功與訊息 |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | navigation_server 呼叫 Nav2 導航 |
| `/spin` | `nav2_msgs/action/Spin` | 指定旋轉／測試流程使用的 Nav2 behavior |
| `/reload_named_poses` | `std_srvs/srv/Trigger` | 重載 navigation_server 的命名點位 |
| `/fsm/start_normal`、`/fsm/start_mirror` | `std_srvs/srv/Trigger` | 啟動對應場地 FSM |
| `/fsm/start_mirror_v2`、`/fsm/start_dit` | `std_srvs/srv/Trigger` | 啟動 V2 路線或 DIT 測試 |
| `/fsm/stop` | `std_srvs/srv/Trigger` | 停止 FSM、取消導航並發送零速度 |
| `/fsm/reset_stage2`、`/fsm/reset_stage3` | `std_srvs/srv/Trigger` | 保留定位、倒數與路線選擇，重啟指定關卡 |

修改 YAML 後需讓對應節點重新載入；named poses 可使用重載 service，FSM 控制參數通常需重啟 FSM。`mirror_spin_speed_scale` 由 Nav2 啟動時讀取，因此修改後需重啟導航。修改 C++ 則需重新建置。

## 6. 整合架構與資料流

以下將實機 Cartographer 定位模式拆成三張圖：跨 repo 分工、移動回授及面板操作。詳細 topic 型別與服務名稱見前節介面表。

### 跨 repo 整合

```mermaid
flowchart TB
    PANEL["robot_panel_ui<br/>狀態監控與操作"]
    subgraph FSMWS["robot_fsm_v2_ws"]
        LAUNCH["robot_panel_bridge<br/>啟停、重置、倒數"]
        FSM["MissionController + 關卡 FSM<br/>任務決策與回授同步"]
        MOVE["robot_navigation / DirectTransition<br/>Nav2 目標或閉迴路控速"]
    end
    SLAM["tdk_slam_ws<br/>LiDAR 前處理、定位、TF、Nav2"]
    VISION["TDK-straw<br/>視覺目標與對準誤差"]
    STM["STM32 底盤與機構<br/>經 micro-ROS 通訊"]

    PANEL <-->|"rosbridge：services / 狀態"| LAUNCH
    LAUNCH -->|"啟動／停止／重置"| FSM
    FSM -->|"移動目標"| MOVE
    MOVE <-->|"導航 action / 結果"| SLAM
    SLAM -->|"/robot_pose"| FSM
    VISION -->|"/straw/target"| FSM
    FSM <-->|"機構指令 / 夾取回授"| STM
    MOVE -->|"直接速度，經速度橋接"| STM
    SLAM <-->|"Nav2 速度 / 里程計"| STM
```

共用的 `field_config.yaml` 決定地圖、TF、點位與場地控制設定；Docker 與啟動腳本負責載入配套服務。圖中的速度與硬體回授鏈於下圖展開。

### 移動控制與回授

```mermaid
flowchart TB
    FSM["FSM：選擇移動段落"]
    NAV["navigation_server → Nav2<br/>路徑規劃與追蹤"]
    DIRECT["DirectTransition / 關卡控速<br/>位置誤差、視覺對準與停止條件"]
    SMOOTH["velocity_smoother"]
    CMD["/cmd_vel → 速度橋接<br/>/mecanum/cmd_vel → micro-ROS"]
    STM["STM32：執行底盤速度"]
    LOC["tdk_slam_ws<br/>LiDAR + odom → 定位與 TF"]
    POSE["/robot_pose：world 位姿"]

    FSM -->|"命名點位 action"| NAV
    FSM -->|"直接控制段落"| DIRECT
    NAV -->|"/cmd_vel_nav"| SMOOTH
    SMOOTH --> CMD
    DIRECT -->|"直接發布 /cmd_vel"| CMD
    CMD --> STM
    STM -->|"/odom"| LOC
    LOC --> POSE
    POSE --> DIRECT
    LOC -->|"TF / 地圖 / scan"| NAV
    NAV -->|"成功／失敗／取消結果"| FSM
    DIRECT -->|"到位／逾時／等待"| FSM
```

兩條控制路徑依 FSM 狀態切換；直接控速前等待導航結束及速度沉降，避免與 smoother 的剩餘輸出交錯。定位過期時直接控制停車；標準速度橋接也會在輸入過期時送零。

### 面板操作與開發監控

```mermaid
flowchart TB
    UI["robot_panel_ui<br/>START / STOP / RESET、燈號、電量、倒數"]
    RB["rosbridge :9090<br/>ROSLIB.js 通訊"]
    LAUNCH["fsm_launcher<br/>場地檢查、程序管理、deadline"]
    FSM["robot_fsm_main + navigation_server"]
    STATUS["ROS 狀態來源<br/>odom、視覺、機構、位姿、節點、電量"]
    FOX["foxglove_bridge :8767"]
    VIEW["Foxglove / Lichtblick<br/>地圖、TF、位姿、named pose markers"]

    UI -->|"Trigger services"| RB
    RB -->|"/fsm/start_*、stop、reset_stage*"| LAUNCH
    LAUNCH -->|"啟動／停止／重置"| FSM
    LAUNCH -->|"操作結果、/fsm/match_remaining"| RB
    STATUS -->|"topics / graph 查詢"| RB
    RB -->|"更新畫面"| UI
    STATUS --> FOX
    FOX -->|"WebSocket"| VIEW
    UI -. "ROBOT PANEL 連結" .-> VIEW
```

START 設定新的比賽期限，STOP／RESET 保留原倒數與路線選擇。燈號依訊息新鮮度及 topic／節點存在判斷；Foxglove／Lichtblick 則供開發時檢查地圖、座標與即時資料。


https://github.com/user-attachments/assets/f84f0316-34dd-41fd-ae32-2e04222bebea

