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
| [tdk_slam_ws](https://github.com/terryking0711/tdk_slam_ws) | 提供 Cartographer 定位、TF、`/robot_pose`、定位初始化與 Nav2（Smac2D + MPPI Omni） |
| `TDK-straw` | 使用 OpenCV 的 HSV 色彩與幾何分析偵測稻草，透過 `/straw/target` 提供第三關對準資訊 |
| `microros_ws` | 提供 micro-ROS Agent 與相關容器服務，銜接 ROS 2 和 STM32；韌體負責實際底盤及機構執行 |
| [robot_panel_ui](https://github.com/terryking0711/robot_panel_ui) | 透過 rosbridge 顯示狀態、呼叫 launcher services，不直接實作關卡邏輯 |
| [tdk_robot_docker](https://github.com/terryking0711/tdk_robot_docker) | 統整 Docker、桌面啟動腳本、`field_config.yaml` 與跨 workspace 執行環境 |

<!-- 在此加入 TDK-straw 辨識圖片／實機影片 -->

通訊依用途選擇：**topics** 傳送位姿、視覺、速度與機構狀態；**actions** 處理導航目標與結果；**services** 處理啟停、重置與點位重載；**TF** 統一場地、地圖、里程計與車體座標。STM32 透過 micro-ROS 參與 ROS 通訊。

### 移動控制：Nav2 與自行實作的閉迴路控速

開發初期使用 Nav2 執行點位移動，但在本麥輪機器人與競賽場地上，移動表現不易調整到符合需求，調參成本高。後來針對固定點位與關卡移動實作 **`DirectTransition` 閉迴路控速**，在實機使用上更容易取得需要的移動表現。

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

圖中的定時移動是依控制常數估算的**名目軌跡**，掃排線為參考線，並非實機量測軌跡，也不保證涵蓋所有路線變體。適合檢查座標、朝向與動作方向，實際路徑仍需錄製定位資料驗證。

<!-- 在此加入地圖＋座標點＋移動疊圖 -->

### Foxglove／Lichtblick 即時開發

在 Foxglove 或 Lichtblick 選擇 Foxglove WebSocket，連線至 `ws://<機器人IP>:8767`。既有環境另提供 Lichtblick 網頁入口 `http://<機器人IP>:8080`；bridge 由配套 micro-ROS 容器堆疊啟動。

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

## 6. 完整整合架構與資料流

以下展開實機使用的 **Cartographer 定位模式**，涵蓋 `tdk_slam_ws`、`robot_fsm_v2_ws`、`robot_panel_ui`，以及必要的視覺、通訊與硬體。實線為資料／操作流，虛線為設定載入、程序管理或保留介面；節點代表邏輯功能，不一定是獨立程序。建圖與其他定位模式不會與圖中定位流程全部同時執行。

```mermaid
flowchart TB
    subgraph CONFIG["共用設定與啟動：tdk_robot_docker"]
        FIELD["field_config.yaml<br/>場地、地圖、world→map、點位與控制設定路徑"]
        START["start.sh / 桌面 Loc 啟動入口<br/>ROS_DOMAIN_ID=30"]
        MAPFILES["地圖檔<br/>pbstream + YAML / PGM"]
        POSES["named_poses_normal / mirror / dit.yaml"]
        CTRL["fsm_side_normal / mirror.yaml<br/>控速、容差、指令映射與 spin 倍率"]
    end

    subgraph WEB["robot_panel_ui：瀏覽器"]
        PAGE["index.html / style.css / app.js<br/>本機 file 或 HTTP :8000"]
        BUTTON["START NORMAL / MIRROR / MIRROR V2 / DIT<br/>STOP / RESET S2 / RESET S3"]
        LIGHT["SYSTEM / DEVICE / ROBOT STATE<br/>訊息新鮮度、topic / node 存在、組合條件"]
        DISPLAY["主機電量、功率、剩餘時間<br/>比賽倒數"]
        LINK["ROBOT PANEL 外連"]
        PAGE --> BUTTON & LIGHT & DISPLAY & LINK
    end

    subgraph ACCESS["通訊與開發服務：配套容器"]
        ROSBR["rosbridge_websocket :9090<br/>topics / services / rosapi graph 查詢"]
        AGENT["micro-ROS Agents<br/>上位機 ROS 2 ↔ STM32"]
        FOX["foxglove_bridge :8767"]
        VIEW["Foxglove / Lichtblick :8080<br/>3D 地圖、TF、位姿、點位與訊息檢視"]
    end

    subgraph FSMWS["robot_fsm_v2_ws"]
        subgraph PANELPKG["robot_panel_bridge"]
            LAUNCHER["fsm_launcher<br/>場地檢查、啟停程序、保留路線與 deadline"]
            TIMER["比賽剩餘秒數<br/>START 設定期限；STOP / RESET 保留"]
            BAT["battery_monitor<br/>讀取主機 /sys/class/power_supply"]
        end
        subgraph FSMPKG["robot_fsm：robot_fsm_main"]
            CALLBACK["ROS callbacks<br/>接收位姿、視覺與機構回授"]
            CTX["RobotContext<br/>最新資料、時間戳、共享設定與 deadline"]
            MISSION["MissionController：10 Hz tick<br/>檢查 → 等待 → 關卡切換 → 結束 / SAFE_STOP"]
            S2["Stage2ClamFSM<br/>準備、推動、抬臂／收臂、返回"]
            S3["Stage3HayFSM<br/>跨越、進排、掃描、夾取等待、退回、換排、放置"]
            DIRECT["DirectTransition<br/>目標位置與 yaw 誤差 → 車體速度<br/>容差、穩定週期、逾時、資料過期停車"]
            ROWCTRL["關卡內速度控制<br/>位姿／視覺對準、加減速、機構同步停車"]
            MECH["機構命令發布<br/>MechanismCommand / 場地指令映射"]
            HANDOFF["移動控制切換<br/>等待導航結束及速度沉降，再接手直接控速"]
        end
        subgraph NAVPKG["robot_navigation"]
            NAVSERVER["navigation_server<br/>named pose 解析、action 結果、逾時／取消"]
            ALIGN["fine_align<br/>Nav2 後的位姿精對位"]
            RELOAD["/reload_named_poses<br/>Trigger service"]
            MARKERS["named_poses_markers.py<br/>監看 YAML 更新"]
            PLOT["plot_named_poses.py<br/>離線地圖＋點位＋名目移動 PNG"]
        end
        CMD["/cmd_vel<br/>當前控制流程的底盤速度"]
        STMBRIDGE["stm_communication_node<br/>標準橋接：20 Hz 轉發，輸入過期送零"]
        INTERFACES["robot_interfaces / straw_interfaces<br/>自訂 msg 與 NavigateToNamedPose.action"]
    end

    subgraph SLAMWS["tdk_slam_ws"]
        DRIVERS["rplidar_ros：前／後 LiDAR 驅動"]
        FILTERF["filter_front：角度與自身範圍過濾"]
        FILTERR["filter_rear：角度與自身範圍過濾"]
        MERGER["ira_laser_tools：laser_merger<br/>統一座標與合併掃描"]
        CARTO["Cartographer pure localization<br/>pbstream + scan + odom"]
        ODOMTF["odom_tf_broadcaster<br/>odom → base_footprint"]
        STATIC["static_transform_publisher<br/>world → map"]
        URDF["robot_state_publisher<br/>base_footprint → laser_front / laser_rear"]
        TF["TF tree<br/>world → map → odom → base_footprint → sensors"]
        POSEPUB["robot_pose_publisher<br/>查 TF，輸出 world 位姿"]
        LOC["localization_manager<br/>軌跡重設、位姿驗證、清 costmap"]
        MAPSERVER["map_server → /map<br/>lifecycle_manager 管理啟用"]
        COSTMAP["Nav2 global / local costmaps<br/>地圖、掃描與 TF"]
        BT["bt_navigator<br/>NavigateToPose 任務與回復流程"]
        PLANNER["planner_server<br/>Smac2D 路徑規劃"]
        CONTROLLER["controller_server<br/>MPPI Omni 路徑追蹤"]
        BEHAVIOR["behavior_server<br/>Spin 等回復動作"]
        SMOOTHER["velocity_smoother<br/>速度／加減速限制"]
    end

    subgraph VISIONWS["TDK-straw"]
        DETECT["相機影像 → OpenCV HSV / 幾何分析<br/>目標有效性、角度與橫向誤差"]
    end
    subgraph HW["硬體與韌體"]
        LIDAR["前／後 RPLiDAR"]
        CAMERA["相機"]
        CHASSIS["STM32 底盤<br/>速度執行與里程計"]
        MECHANISM["STM32 機構<br/>動作執行與夾取回授"]
    end

    FIELD -.-> START & STATIC & LAUNCHER & NAVSERVER & MARKERS
    FIELD -.-> MAPFILES & POSES & CTRL
    START -.-> CARTO & LOC & MAPSERVER & BT & LAUNCHER & MARKERS
    CTRL -.-> CTX
    CTRL -. "mirror / DIT spin 倍率於 Nav2 啟動時載入" .-> BEHAVIOR & SMOOTHER
    POSES -.-> NAVSERVER & DIRECT & MARKERS & PLOT
    MAPFILES -. "pbstream" .-> CARTO
    MAPFILES -. "YAML / PGM" .-> MAPSERVER & PLOT
    INTERFACES -. "型別定義" .-> CALLBACK & NAVSERVER & MECH

    BUTTON -->|"ROSLIB.js / Trigger service"| ROSBR
    ROSBR -->|"/fsm/start_*、/fsm/stop、/fsm/reset_stage*"| LAUNCHER
    LAUNCHER -->|"service success / message，經 rosbridge"| BUTTON
    LAUNCHER -. "launch 參數：場地、路線、reset、deadline" .-> MISSION
    LAUNCHER --> TIMER
    TIMER -->|"/fsm/match_remaining"| ROSBR
    BAT -->|"/host/battery"| ROSBR
    ROSBR -->|"電量與剩餘秒數"| DISPLAY
    ROSBR -->|"topic 訊息及 graph 查詢結果"| LIGHT
    LAUNCHER -->|"STOP：零速度"| CMD
    LAUNCHER -->|"CancelGoal：navigate_to_pose / navigate_through_poses"| BT

    LIDAR --> DRIVERS
    DRIVERS -->|"/front/scan"| FILTERF
    DRIVERS -->|"/rear/scan"| FILTERR
    FILTERF -->|"/front/scan_filtered"| MERGER
    FILTERR -->|"/rear/scan_filtered"| MERGER
    MERGER -->|"/scan"| CARTO & COSTMAP
    CHASSIS -->|"里程計，經 Agent"| AGENT
    AGENT -->|"/odom"| ODOMTF & CARTO & CONTROLLER & ROSBR
    CARTO -->|"map → odom"| TF
    ODOMTF --> TF
    STATIC --> TF
    URDF --> TF
    TF --> POSEPUB & COSTMAP & MERGER & ALIGN
    POSEPUB -->|"/robot_pose"| CALLBACK & LOC & ROSBR
    MAPSERVER -->|"/map"| COSTMAP

    CAMERA --> DETECT
    DETECT -->|"/straw/target"| CALLBACK & ROSBR
    MECHANISM -->|"夾取狀態，經 Agent"| AGENT
    AGENT -->|"/mechanism/straw_pick_status"| CALLBACK & ROSBR
    CALLBACK --> CTX
    CTX --> MISSION & S2 & S3 & DIRECT & ROWCTRL
    MISSION --> S2 & S3
    S2 & S3 -->|"關卡完成／等待條件"| MISSION
    MISSION & S2 & S3 -->|"命名導航目標"| NAVSERVER
    MISSION & S2 & S3 --> HANDOFF
    HANDOFF --> DIRECT & ROWCTRL
    MISSION & S2 & S3 --> MECH
    MECH -->|"/mechanism/command"| AGENT & ROSBR
    AGENT -->|"機構指令"| MECHANISM
    DIRECT & ROWCTRL --> CMD

    RELOAD -->|"重新讀取 YAML"| NAVSERVER
    NAVSERVER <-->|"/navigate_to_named_pose：goal / result"| MISSION
    NAVSERVER -->|"/navigate_to_pose：goal / cancel"| BT
    BT --> PLANNER & CONTROLLER & BEHAVIOR
    COSTMAP --> PLANNER & CONTROLLER
    PLANNER -->|"規劃路徑，經 BT"| CONTROLLER
    CONTROLLER & BEHAVIOR -->|"/cmd_vel_nav"| SMOOTHER
    BT -->|"導航結果"| NAVSERVER
    NAVSERVER --> ALIGN
    ALIGN -->|"/cmd_vel_nav"| SMOOTHER
    NAVSERVER -->|"取消／逾時停車"| CMD
    SMOOTHER -->|"cmd_vel_smoothed remap 至 /cmd_vel"| CMD
    CMD --> STMBRIDGE
    STMBRIDGE -->|"/mecanum/cmd_vel"| AGENT
    AGENT -->|"底盤速度命令"| CHASSIS

    MISSION -. "保留 /init_pose_cmd；目前啟動略過重定位" .-> LOC
    LOC -. "/finish_trajectory、/start_trajectory" .-> CARTO
    LOC -. "清除 costmap services" .-> COSTMAP
    LOC -. "/init_pose_status" .-> CALLBACK

    DRIVERS -->|"/front/scan、/rear/scan：燈號監控"| ROSBR
    BT & MISSION -->|"rosapi：節點存在查詢"| ROSBR
    LINK --> VIEW
    TF & POSEPUB & MAPSERVER & MARKERS & DETECT -->|"ROS topics / TF"| FOX
    MARKERS -->|"/named_poses_markers"| FOX
    FOX -->|"Foxglove WebSocket"| VIEW
```

閱讀時可沿三條主線追蹤：**感測器 → 定位／視覺 → RobotContext → FSM 決策**、**FSM → Nav2 或直接控速 → 底盤 → 里程計回授**、**面板 → launcher → FSM 程序與倒數 → 面板監控**。圖中 Nav2 內部流程為功能摘要，實際節點間的路徑追蹤由 Nav2 actions 與 BT 協調。

速度來源的切換由程式流程管理，並非額外的硬體仲裁器；FSM 接手前等待導航結束與 1.2 秒沉降，避免與 smoother 的剩餘輸出交錯。圖中的速度橋接為 `nav_cmd_bridge.launch.py` 標準配置，其他測試 launch 可以覆寫輸入與輸出 topic。
