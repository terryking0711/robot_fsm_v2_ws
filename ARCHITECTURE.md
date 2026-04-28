    # Robot FSM v2 架構流程圖

    ---

    ## 1. 系統元件總覽 (Component Diagram)

    ```mermaid
    graph TB
        subgraph main_cpp ["main.cpp  (程式入口)"]
            NODE[ros_node\nrobot_fsm_main]
            CTX["RobotContext  ctx\n共享資料中心\ndata_mutex 保護"]
            MC[MissionController]
        end

        subgraph sub_topics ["訂閱的 Topics  (感知輸入，只讀)"]
            V["/vision/scene_state\nVisionSceneState"]
            MF["/mechanism/feedback\nMechanismFeedback"]
            FP["/final_pose\nPoseStamped"]
        end

        subgraph pub_topics ["發布的 Topic  (機構控制，只寫)"]
            MC_PUB["/mechanism/command\nMechanismCommand"]
        end

        subgraph action_layer ["ROS2 Action  (導航)"]
            NAV["navigate_to_named_pose\nNavigateToNamedPose\nAction Server (外部)"]
        end

        subgraph stage_fsms ["各關卡子 FSM"]
            S2[Stage2ClamFSM]
            S3[Stage3HayFSM]
            S4[Stage4MazuFSM]
            S5[Stage5NewFSM]
        end

        V  -->|"callback → ctx.latest_vision_state"| CTX
        MF -->|"callback → ctx.latest_mechanism_feedback"| CTX
        FP -->|"callback → ctx.latest_final_pose"| CTX

        CTX --> MC
        MC --> S2 & S3 & S4 & S5

        MC -->|"async_send_goal\n(navigate_to_named_pose)"| NAV
        S3 -->|"async_send_goal\n(navigate_to_named_pose)"| NAV
        NAV -->|"result_callback → nav_done_ / nav_success_"| MC
        NAV -->|"result_callback → nav_done_ / nav_success_"| S3

        MC -->|"publish MechanismCommand"| MC_PUB
        S3 -->|"publish_mechanism_command()"| MC_PUB
    ```

    ---

    ## 2. 主循環流程 (Main Loop — 每 100 ms)

    ```mermaid
    flowchart TD
        A([程式啟動]) --> B["初始化 ROS Node\n建立 RobotContext ctx\n建立 3 個 Subscribers\n  /vision/scene_state\n  /mechanism/feedback\n  /final_pose"]
        B --> C["建立 MissionController(ctx)\n  初始 state_ = BOOT\n  建立 Stage2/3/4/5 FSM 物件\n  nav_goal_sent_=false, nav_done_=false"]
        C --> D["背景執行緒 detach()\n sleep(5s) → ctx.start_signal = true"]
        D --> E[["WallRate 10 Hz  主循環"]]

        E --> F["rclcpp::spin_some()\n處理佇列中所有 ROS 訊息\n  ↳ vision_sub callback  → ctx.latest_vision_state\n  ↳ mech_sub callback   → ctx.latest_mechanism_feedback\n  ↳ pose_sub callback   → ctx.latest_final_pose\n  ↳ action callbacks   → nav_done_ / nav_success_"]
        F --> G["mission.tick()\n根據 state_ 執行一步 FSM\n  可能：保持原 state（回傳 false）\n  可能：切換到下一個 state（回傳 true）"]
        G --> H["loop_rate.sleep()\n等到下個 100 ms"]
        H --> E
    ```

    ---

    ## 3. MissionController 頂層狀態機

    > 每個 tick 只執行一個 `case`；需要等待的 state 會重複留在原地直到條件滿足。

    ```mermaid
    stateDiagram-v2
        direction TB
        [*] --> BOOT : 程式啟動

        BOOT --> INIT : 立即切換（同一 tick）

        INIT --> SELF_CHECK : init_system() = true\n建立 nav_client\n建立 mechanism_cmd_pub
        INIT --> SAFE_STOP   : init_system() = false

        SELF_CHECK --> WAIT_START : self_check() = true\n（目前永遠成功）
        SELF_CHECK --> SAFE_STOP  : self_check() = false

        WAIT_START --> WAIT_START        : ctx.start_signal == false\n每 tick 輪詢，等待背景執行緒
        WAIT_START --> LEAVE_START_ZONE  : ctx.start_signal == true

        LEAVE_START_ZONE --> LEAVE_START_ZONE : transition_to_named_pose\n"leave_start_zone" 進行中
        LEAVE_START_ZONE --> STAGE2_CLAM     : nav 完成且成功

        STAGE2_CLAM --> STAGE2_CLAM          : stage2_fsm.tick() = false
        STAGE2_CLAM --> TRANSITION_TO_STAGE3 : stage2_fsm.tick() = true ⚠️ enum 未定義

        TRANSITION_TO_STAGE3 --> TRANSITION_TO_STAGE3 : nav "stage3_entry" 進行中
        TRANSITION_TO_STAGE3 --> STAGE3_HAY            : nav 完成

        STAGE3_HAY --> STAGE3_HAY            : stage3_fsm.tick() = false
        STAGE3_HAY --> TRANSITION_TO_STAGE4  : stage3_fsm.tick() = true ⚠️ enum 未定義

        TRANSITION_TO_STAGE4 --> TRANSITION_TO_STAGE4 : nav "stage4_entry" 進行中
        TRANSITION_TO_STAGE4 --> STAGE4_MAZU           : nav 完成

        STAGE4_MAZU --> STAGE4_MAZU           : stage4_fsm.tick() = false
        STAGE4_MAZU --> TRANSITION_TO_STAGE5  : stage4_fsm.tick() = true ⚠️ enum 未定義

        TRANSITION_TO_STAGE5 --> TRANSITION_TO_STAGE5 : nav "stage5_entry" 進行中
        TRANSITION_TO_STAGE5 --> STAGE5_NEW            : nav 完成

        STAGE5_NEW --> STAGE5_NEW         : stage5_fsm.tick() ≠ SUCCESS\n⚠️ .cpp 缺少此 case
        STAGE5_NEW --> FINISH_DECISION    : stage5_fsm.tick() = SUCCESS

        FINISH_DECISION --> END_RUN    : finish_decision() = true
        FINISH_DECISION --> EARLY_STOP : finish_decision() = false

        EARLY_STOP --> END_RUN
        END_RUN   --> END_RUN   : 任務結束，永遠停留
        SAFE_STOP --> SAFE_STOP : 錯誤停止，永遠停留
    ```

    ---

    ## 4. Stage3HayFSM 子狀態機（最完整的範例關卡）

    ```mermaid
    stateDiagram-v2
        direction TB
        [*] --> S3_ENTER

        S3_ENTER --> S3_LOCALIZE : 下一個 tick

        S3_LOCALIZE --> S3_LOCALIZE : ctx.latest_final_pose 無值\n繼續等待 /final_pose topic
        S3_LOCALIZE --> S3_SCAN_HAY : ctx.latest_final_pose 有值\n定位資訊已到達

        S3_SCAN_HAY --> S3_SCAN_HAY  : ctx.latest_vision_state 無值\n繼續等待 /vision/scene_state topic
        S3_SCAN_HAY --> S3_PLAN_STACK : ctx.latest_vision_state 有值

        S3_PLAN_STACK   --> S3_SELECT_TARGET : 規劃完成（同 tick）
        S3_SELECT_TARGET --> S3_NAV_TO_PICK  : 重置 nav flags

        S3_NAV_TO_PICK --> S3_NAV_TO_PICK : 第一次 tick：發送 Action Goal\ntarget="stage3_pick_pose"\n之後每 tick：等待 result_callback
        S3_NAV_TO_PICK --> S3_PICK_HAY   : nav_success == true
        S3_NAV_TO_PICK --> S3_FAILED     : nav_success == false

        S3_PICK_HAY --> S3_PICK_HAY    : 第一次：publish MechanismCommand\n  command_name="pick_hay"\n  arg_json={target:hay_roll,index:0}\n之後：等待 /mechanism/feedback\n  done==true && success==true
        S3_PICK_HAY --> S3_NAV_TO_STACK : feedback.done && feedback.success

        S3_NAV_TO_STACK --> S3_NAV_TO_STACK : 發送 Action Goal\ntarget="stage3_stack_pose"\n等待 result_callback
        S3_NAV_TO_STACK --> S3_PLACE_HAY   : nav_success == true
        S3_NAV_TO_STACK --> S3_FAILED      : nav_success == false

        S3_PLACE_HAY --> S3_PLACE_HAY      : publish MechanismCommand\n  command_name="place_hay"\n  arg_json={slot:layer1_slot1}\n等待 feedback
        S3_PLACE_HAY --> S3_VERIFY_STABLE  : feedback.done && feedback.success

        S3_VERIFY_STABLE --> S3_DONE : 確認完成（同 tick）

        S3_DONE   --> [*]         : return true → MissionController 推進
        S3_FAILED --> S3_FAILED   : return false，永遠停留
    ```

    ---

    ## 5. 非同步導航每一 tick 的細節（pattern 說明）

    > 同樣的模式出現在 `MissionController::transition_to_named_pose()` 和 `Stage3HayFSM::S3_NAV_TO_PICK`

    ```mermaid
    flowchart TD
        A["tick() 被呼叫\n目前 state = S3_NAV_TO_PICK"] --> B{nav_goal_sent_?}

        B -- false --> C["wait_for_action_server(1s)\n確認 Action Server 存在"]
        C --> D["建立 Goal\ntarget_name = stage3_pick_pose\ntimeout_sec = 10.0"]
        D --> E["設定 result_callback lambda\n當導航完成時執行：\n  nav_done_ = true\n  nav_success_ = result.success"]
        E --> F["ctx.nav_client->async_send_goal()\n非同步發出，立刻返回（不阻塞）"]
        F --> G["nav_goal_sent_ = true\nreturn false（這個 tick 結束）"]

        B -- true --> H{nav_done_?}
        H -- false --> I["return false\n繼續等待下一個 tick"]
        H -- true  --> J{nav_success_?}
        J -- false --> K["state = S3_FAILED\nreturn false"]
        J -- true  --> L["state = S3_PICK_HAY\nreturn false"]

        subgraph bg ["ROS2 後台（不在主迴圈，由 spin_some 處理）"]
            NAV_SRV["Action Server\n導航完成"] -->|"result_callback 被觸發"| CB["nav_done_ = true\nnav_success_ = result.result->success"]
        end

        style bg fill:#f0f4ff,stroke:#99aaff
    ```

    ---

    ## 6. Stage5NewFSM — tick 計數器模式

    > 用 `wait_ticks(N)` 模擬需要多個 tick 的等待行為（N ticks × 100ms）

    ```mermaid
    stateDiagram-v2
        [*] --> ENTER
        ENTER --> LOCALIZE : return RUNNING

        LOCALIZE --> LOCALIZE : tick_count < 5  → return RUNNING
        LOCALIZE --> SCAN    : tick_count ≥ 5  (≈ 500 ms)

        SCAN --> SCAN  : tick_count < 5  → return RUNNING
        SCAN --> PLAN  : tick_count ≥ 5

        PLAN --> PLAN    : tick_count < 5  → return RUNNING
        PLAN --> EXECUTE : tick_count ≥ 5

        EXECUTE --> EXECUTE : tick_count < 8  → return RUNNING
        EXECUTE --> VERIFY  : tick_count ≥ 8  (≈ 800 ms)

        VERIFY --> VERIFY : tick_count < 3  → return RUNNING
        VERIFY --> DONE   : tick_count ≥ 3  (≈ 300 ms)

        DONE --> [*] : return StageStatus::SUCCESS\n通知 MissionController 完成
    ```

    ---

    ## 7. RobotContext 資料流

    ```mermaid
    graph LR
        subgraph 外部系統
            VS_PUB[視覺系統\n發布者]
            MECH_PUB[機構控制器\n發布者]
            NAV_PUB[導航系統\n發布者]
        end

        subgraph ctx ["RobotContext（共享記憶體，mutex 保護）"]
            LVS["latest_vision_state\nstd::optional<VisionSceneState>"]
            LMF["latest_mechanism_feedback\nstd::optional<MechanismFeedback>"]
            LFP["latest_final_pose\nstd::optional<PoseStamped>"]
            SS["start_signal  bool"]
            NAV_C["nav_client\nAction Client"]
            MECH_P["mechanism_cmd_pub\nPublisher"]
        end

        VS_PUB  -->|"/vision/scene_state"| LVS
        MECH_PUB -->|"/mechanism/feedback"| LMF
        NAV_PUB  -->|"/final_pose"| LFP

        LVS -->|"has_value() 檢查\nS3_SCAN_HAY"| S3[Stage3HayFSM]
        LMF -->|"done && success 檢查\nS3_PICK_HAY / S3_PLACE_HAY"| S3
        LFP -->|"has_value() 檢查\nS3_LOCALIZE"| S3
        SS  -->|"== true 檢查\nWAIT_START"| MC[MissionController]
        NAV_C -->|"async_send_goal"| MC
        NAV_C -->|"async_send_goal"| S3
        MECH_P -->|"publish"| MC
        MECH_P -->|"publish_mechanism_command"| S3
    ```

    ---

    ## 設計概念整理

    | 概念 | 說明 |
    |------|------|
    | **非阻塞 FSM** | 每個 state 函式每次只執行「一小步」，`false`/`RUNNING` = 繼續等，`true`/`SUCCESS` = 完成 |
    | **spin_some + tick 分離** | `spin_some` 更新 ctx 資料 → `tick` 讀取 ctx 做決策，兩者乾淨分離 |
    | **RobotContext 共享** | 所有 FSM 都拿同一個 ctx 指標，感知資料只需寫一次，所有人都能讀 |
    | **非同步導航** | `async_send_goal` 立刻返回，結果透過 `result_callback` lambda 寫入 `nav_done_/nav_success_` |
    | **機構控制** | Publish `MechanismCommand`（fire-and-forget），下一 tick 開始輪詢 `MechanismFeedback` |
    | **optional 感知等待** | `std::optional::has_value()` 判斷是否收到第一筆資料 |

    ---

    ## ⚠️ 程式碼現有問題

    | # | 位置 | 問題 |
    |---|------|------|
    | 1 | `mission_states.hpp` | 缺少 `TRANSITION_TO_STAGE3/4/5` 三個 enum 值，但 `.cpp` 使用了它們 → 編譯錯誤 |
    | 2 | `mission_controller.cpp` | `tick()` 缺少 `STAGE5_NEW` 的 case |
    | 3 | `stage3_hay_fsm.cpp:3` | `Stage3HayFSM::(...)` 應改為 `Stage3HayFSM::Stage3HayFSM(...)` → 語法錯誤 |
    | 4 | `mission_controller.hpp:40` | `stage3_fsm_` 宣告了兩次 → 重複宣告錯誤 |
    | 5 | `stage5_new_fsm` | `tick()` 回傳 `StageStatus`，但 MissionController 用 `bool` 接收（其他 stage 回傳 `bool`） |
