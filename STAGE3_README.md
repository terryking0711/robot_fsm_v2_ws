# 第三關（稻草捲收集）測試與調參手冊

這份文件寫給**要測試、調整第三關，但不熟 ROS2** 的隊員。照著做就能：

1. 看懂第三關機器人在做什麼
2. 設定目標點、調速度、重新編譯、上場測試

> 所有說明以 **DIT 實驗室場地**為準（桌面「TDK Loc_DIT」、面板「START DIT」）。

---

## 目錄

1. [第三關在做什麼](#1-第三關在做什麼)
2. [先懂這幾個名詞](#2-先懂這幾個名詞)
3. [開機到開始跑](#3-開機到開始跑)
4. [改了東西之後要做什麼](#4-改了東西之後要做什麼)
5. [設定目標點](#5-設定目標點)
6. [調整參數（速度等）](#6-調整參數速度等)
7. [建議的測試順序](#7-建議的測試順序)
8. [看懂 log](#8-看懂-log)
9. [常見問題](#9-常見問題)
10. [指令速查](#10-指令速查)

---

## 1. 第三關在做什麼

### 場地

- 場上有 **4 排**，每排 **6 個格點**，共 24 個格點，其中隨機 **10 個**放了稻草捲。
- 機器前方有一個**極限開關**。開關碰到稻草捲時，**機構 STM 自己**跑腳本把稻草夾起來，收進 storage。主程式只負責停車等待，第三關除了最後的放置指令以外不送任何 `/mechanism/command`。
- 最後一排的排尾就是**放置點**。

### 流程

```
衝過田埂
   │
   ▼
導航到第 1 排起點
   │
   ▼
沿著這排往前走 ◄──────────────┐
   │  碰到稻草捲 → 停車等夾取 ─┘
   │
   ▼ 這排走完
 ┌─ 還有下一排？ ─ 是 → 導航退回這排起點 → 導航側移到下一排起點 → 回到「沿著這排往前走」
 │
 └─ 否（第 4 排走完；不計數，4 排一定都掃）
   │
   ▼
在放置點送「放置」指令 → 第三關結束
```

每一步的細節：

| 步驟 | 程式裡的名字 | 做什麼 |
|---|---|---|
| 進入 | `S3_ENTER` | 檢查 4 排的點位都設好了（沒設好會直接 `S3_FAILED`） |
| 衝田埂 | `S3_CROSS` | 不靠導航，直接以固定速度往前衝固定秒數 |
| 到排起點 | `S3_NAV_ROW_START` | 用 Nav2（自動導航）開到 `stage3_rowN_start` |
| 開始這排 | `S3_ROW_BEGIN` | 等導航的速度指令停下來，開始這排的計時；第 1 排會先送 320 通知 STM 開始自動夾取（1 秒內送 3 次） |
| 沿排前進 | `S3_SWEEP` | **不用 Nav2**，程式自己控制輪子往前走（見下方說明） |
| 等夾取 | `S3_WAIT_PICK` | 碰到稻草後停車，固定等 20 秒（STM 自己夾）再繼續走；人工模式是等開關放開 |
| 退後重撞 | `S3_BACKOFF` | 等夾取時開關壓著、但 STM 1 秒內都沒開始夾（例如送 320 之前就已經壓著，STM 只認「按下」那一下）：慢慢後退到開關放開，再回「沿排前進」往前重撞。同一捲重撞 3 次都沒夾就結束這排 |
| 這排結束 | `S3_ROW_DONE` | 停車，決定下一步要去哪 |
| 退回 | `S3_NAV_ROW_RETURN` | 用 Nav2 退回這排的起點，再去下一排 |
| 去放置點 | `S3_NAV_TO_PLACE` | 最後一排沒開到排尾（例如超時）時，用 Nav2 開到 `stage3_row4_end`（車頭朝掃描方向） |
| 放置點轉向 | `S3_ROTATE_AT_PLACE` | 原地轉 `stage3_place_rotate` 的 `dyaw`（正 = 逆時針、負 = 順時針），轉完才放置 |
| 放置 | `S3_PLACE` | 送放置指令，等幾秒 |
| 完成 | `S3_DONE` | 第三關結束 |

### 「沿排前進」怎麼對準稻草

機器每 0.1 秒做一次判斷：

- **相機有看到這排的稻草** → 依相機算出的左右偏移**橫移對準**。距離小於 35 cm 時降到慢速，避免撞太大力。
- **相機沒看到** → 依定位座標**沿著這排的中心線**走。
- 看到的稻草如果**偏太多（超過 15 cm）**，就當成**隔壁排的**，直接忽略。

這排什麼時候算結束？以下任一個成立就結束：

1. 開到排尾（`stage3_rowN_end`）
2. 連續 1 秒畫面裡完全看不到稻草，代表前面空了，提早換排（**第 4 排不會提早，一定開到排尾**）
3. 這排花了超過 60 秒（停車等夾取的時間不算）

---

## 2. 先懂這幾個名詞

| 名詞 | 白話解釋 |
|---|---|
| **topic** | 程式之間傳資料的「頻道」。例如 `/robot_pose` 頻道一直在廣播機器人的位置 |
| **`/cmd_vel`** | 叫底盤移動的頻道。`linear.x` 是前後，`linear.y` 是左右（**正 = 左**），`angular.z` 是轉向 |
| **`/robot_pose`** | 機器人目前在場地上的位置（x, y）和朝向 |
| **`/straw/target`** | 相機看到的稻草捲：`distance_m` 是距離，`lateral_error_m` 是左右偏移（**正 = 在右邊**） |
| **`/mechanism/straw_pick_status`** | 機構 STM 回報的狀態：開關有沒有被按、是否正在夾取 |
| **named pose（目標點）** | 有名字的場地座標，例如 `stage3_row1_start`，都寫在 yaml 檔裡 |
| **world 座標** | 場地座標系，單位是公尺，**場地左下角是 (0, 0)** |
| **yaw** | 機器人的朝向，單位是弧度（見[第 5 節的表](#yaw-怎麼填)） |
| **build（編譯）** | 改了 C++ 程式（`.hpp` / `.cpp`）之後，要重新編譯才會生效 |
| **容器（container）** | 我們的 ROS 程式都跑在 Docker 容器裡，指令要在容器裡下 |

---

## 3. 開機到開始跑

### 3.1 依序點桌面圖示

| 順序 | 圖示 | 作用 |
|---|---|---|
| 1 | **TDK micro_ros** | 連線到 STM32（底盤、機構） |
| 2 | **TDK Loc_DIT** | 定位 + 導航（DIT 場地地圖） |
| 3 | **TDK vision** | 相機 + 稻草捲偵測 |
| 4 | **TDK robot panel** | 打開控制面板網頁 |

### 3.2 確認面板狀態

面板上的 **CHASSIS、LOCALIZATION、NAVIGATION、VISION**，以及機構狀態，都應該亮正常。哪一個不亮，就先處理哪一個，不要直接開跑。

### 3.3 開跑

1. 把機器放在 DIT 場地的**出發點**（yaml 裡的 `start_pose`）。
2. 按面板的 **START DIT**。
3. 機器會：定位 → 約 5 秒後出發 → 離開出發區 → **跳過第一、二關**，導航到 `stage3_entry` → 開始第三關。

### 3.4 停止

- 按面板的 **STOP ROBOT_FSM_MAIN**。這會讓機器停車，並停掉主程式和導航目標。
- **緊急狀況請直接按實體急停。**

### 3.5 開一個容器裡的 terminal（下指令用）

後面的指令都要在容器裡執行。在電腦上開一個 terminal，貼上：

```bash
docker exec -it $(docker ps --filter label=com.docker.compose.service=tdk --format '{{.Names}}' | head -1) bash
```

進去之後，**每次都要先執行**：

```bash
source /workspaces/robot_fsm_v2_ws/use_fsm.sh
```

---

## 4. 改了東西之後要做什麼

| 你改了什麼 | 要做什麼 |
|---|---|
| **目標點**（`named_poses_dit.yaml`） | **不用 build**。面板 STOP → START DIT 就生效 |
| **速度、參數**（`stage3_hay_fsm.hpp`） | 面板 STOP → **build** → START DIT |
| 其他 `.cpp` / `.hpp` 程式 | 同上，要 build |

### 怎麼 build

1. **先按面板 STOP**。build 時會清掉舊的程式，跑到一半會出事。
2. 在容器的 terminal 執行（[3.5 節](#35-開一個容器裡的-terminal下指令用)）：

```bash
cd /workspaces/robot_fsm_v2_ws
./build_fsm.sh
```

3. 大約 1～2 分鐘。最後看到 `Build verified:` 就是成功了。
4. 如果出現紅色的 `error:`，代表程式改壞了。看錯誤訊息指的是哪個檔案、第幾行，通常是少了分號或打錯字。
5. 回面板按 **START DIT**。

---

## 5. 設定目標點

### 檔案位置

```
robot_fsm_v2_ws/src/robot_navigation/config/named_poses_dit.yaml
```

格式長這樣（單位：公尺 / 弧度）：

```yaml
  stage3_row1_start:
    x: 6.20
    y: 1.35
    yaw: 1.5708
```

> 改的時候注意**縮排要用空格、對齊上下行**，yaml 對縮排很敏感。

### 第三關要設的點

| 點名 | 位置 | yaw |
|---|---|---|
| `stage3_entry` | **田埂前方**，車頭對著田埂。機器到這裡後會直接往前衝 | 朝田埂 |
| `stage3_row1_start` ~ `stage3_row4_start` | 每排**第一個格點的前方**，車頭朝掃描方向。4 個點在同一側 | 掃描方向 |
| `stage3_row1_end` ~ `stage3_row3_end` | 每排**最後一個格點上放稻草時，開關剛好碰到稻草的那個機器位置** | 不會用到（填跟掃描方向一樣即可） |
| `stage3_row4_end` | 同上，**同時也是放置點**，沒開到排尾時 Nav2 會導航過去 | **一定要填掃描方向** |
| `stage3_place_rotate` | 相對 `stage3_row4_end`、`dx` / `dy` 填 0，只填 `dyaw`：放置前原地轉多少 | **`dyaw` 正 = 逆時針（往左）、負 = 順時針（往右）**，例如 `3.1416` / `-3.1416` |

> ⚠ 目前 `stage3_row*` 都是 0。只要有一個點是 0，第三關一開始就會進 `S3_FAILED`（這是保護機制）。

### 怎麼量

1. 面板按 **STOP**，確定主程式沒在跑。
2. 把機器移到要量的位置：用手推，或在容器 terminal 用鍵盤遙控：
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. 在容器 terminal 讀目前位置：
   ```bash
   ros2 topic echo /robot_pose --once
   ```
4. 把 `position` 底下的 **x、y** 抄進 yaml。**yaw 不要抄 orientation**（那是另一種格式），依照下表填。

### yaw 怎麼填

| 車頭朝向（world 座標） | yaw |
|---|---|
| +x 方向（往右） | `0.0` |
| +y 方向（往上） | `1.5708` |
| −x 方向（往左） | `3.1416` |
| −y 方向（往下） | `-1.5708` |

### 量完檢查

START DIT 後，log 開頭會印出每一排的資訊：

```
[Stage3] row 1: start (6.200, 1.350) heading 1.571 length 1.650
```

- `length` 應該約等於「一排的長度」（公尺）。
- 如果出現 `yaw differs from start->end heading`，代表 start 的 yaw 跟「start→end 的方向」差太多，回頭檢查點位。

---

## 6. 調整參數（速度等）

### 檔案位置

```
robot_fsm_v2_ws/src/robot_fsm/include/robot_fsm/stages/stage3_hay_fsm.hpp
```

找「`---- 可在這裡直接調整 ----`」那一段，只改 `=` 後面的數字。**改完一定要 build**（[第 4 節](#4-改了東西之後要做什麼)）。

### 最常調的

| 參數 | 目前值 | 單位 | 作用 | 怎麼調 |
|---|---|---|---|---|
| `kAutoPick` | `true` | — | `true` = 機構 STM 自動夾取（衝過田埂、抵達第 1 排起點後才送 320）；`false` = **人工夾取模式**（手臂不能用時） | 手臂有問題時改回 `false` |
| `kCrossDurationSec` | 2.0 | 秒 | 衝田埂衝多久 | 過不去就加長；衝太遠就減短 |
| `kCrossLinearX` | 0.4 | m/s | 衝田埂的速度 | 爬不上去就加大 |
| `kSweepFastSpeed` | 0.25 | m/s | 沿排前進的一般速度 | **第一次測請先改成 0.15** |
| `kSweepSlowSpeed` | 0.08 | m/s | 對準後靠上去的速度 | 碰到後把稻草推太遠 → 調小 |
| `kAlignTolM` / `kAlignResumeTolM` | 0.01 / 0.02 | m | 相機看到本排下一捲（`has_depth` 且橫向 < 0.15 m）就停止前進先橫移對準，誤差 < 1 cm 才前進、> 2 cm 又停下來對；看到後鎖住直到碰到開關 | 對很久對不好 → 放寬 |
| `kPickWaitSec` | 20.0 | 秒 | 自動夾取時，碰到稻草後停多久（不看 STM 狀態） | 夾取還沒完成就走了 → 調大 |
| `kPlaceWaitSec` | 5.0 | 秒 | 送放置指令後等多久 | 依放置腳本的時間調整 |

### 對準相關

| 參數 | 目前值 | 作用 | 怎麼調 |
|---|---|---|---|
| `kCamLateralGain` | 1.5 | 看到稻草時，橫移修正的力道 | 左右晃 → 調小；修太慢 → 調大。**越修越偏 → 改成負數**（方向反了） |
| `kLineLateralGain` | 1.5 | 沒看到稻草時，回到排中線的力道 | 同上 |
| `kYawGain` | 1.5 | 保持車頭方向的力道 | 車頭左右擺 → 調小 |
| `kMaxLateralSpeed` | 0.15 m/s | 橫移速度上限 | — |
| `kCamLateralGateM` | 0.15 m | 稻草左右偏超過多少就當成隔壁排的 | 本排的稻草常被忽略 → 調大；被隔壁排的拉走 → 調小 |

### 結束條件

| 參數 | 目前值 | 作用 |
|---|---|---|
| `kEnableEmptyRowExit` | `true` | 前面看不到稻草就提早換排（第 4 排除外）。**空場測試時改成 `false`**，不然每排開 1 秒就換排 |
| `kEmptyRowTicks` | 10 | 連續幾次（每次 0.1 秒）看不到才算空 |
| `kRowTimeoutSec` | 60 | 一排最多花幾秒 |
| `kCmdPlaceStraw` | 330 | 放置指令代號（**佔位值**，跟機構組確認後再改） |

> 不在表上的參數，沒把握就不要動。

### 人工夾取模式（`kAutoPick = false`）

手臂和夾取腳本還沒好的時候用：

- 機器碰到稻草時一樣會**停車**。
- **隊友用手把稻草拿走**，開關放開、再過 1.5 秒後，機器繼續走。
- 不會送放置指令。
- 前提：機構 STM 要正常發 `/mechanism/straw_pick_status`，不然機器不知道自己碰到了稻草。

---

## 7. 建議的測試順序

**一步過了再做下一步。**

### Step 1：資料都有在傳

在容器 terminal 執行：

```bash
ros2 topic hz /robot_pose                      # 要有穩定頻率
ros2 topic hz /straw/target                    # 要有穩定頻率
ros2 topic echo /mechanism/straw_pick_status   # 用手按開關，switch_pressed 要變 true
```

（每個都用 `Ctrl+C` 結束）

### Step 2：底盤左右方向

FSM 要停著。執行下面指令，機器應該往**右**移一點：

```bash
timeout 1 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.05}}"; \
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

如果往**左**移，代表底盤方向是反的，**先找底盤組修，不要繼續測**。

### Step 3：只測相機對準（不用跑整個流程）

FSM 要停著。把稻草放在機器前方約 50 cm、偏右約 10 cm：

```bash
# 只橫移：機器應往右移，對準後停住（log 顯示 [aligned]）
ros2 run robot_fsm straw_align_test --ros-args -p mode:=lateral

# 邊前進邊對準：碰到開關或距離 < 15 cm 就停，最後會印出剩下的偏差
ros2 run robot_fsm straw_align_test --ros-args -p mode:=approach -p fast_speed:=0.15
```

- 這個工具的參數可以直接加 `-p 名稱:=數值` 來試，**不用 build**。例如 `-p cam_lateral_gain:=1.0`。
- 試出好的數值後，再抄回 `stage3_hay_fsm.hpp` 對應的參數，然後 build。

| 工具參數 | 對應 hpp 參數 |
|---|---|
| `cam_lateral_gain` | `kCamLateralGain` |
| `fast_speed` | `kSweepFastSpeed` |
| `slow_speed` | `kSweepSlowSpeed` |
| `slow_down_dist_m` | `kSlowDownDistM` |
| `lateral_gate_m` | `kCamLateralGateM` |

隨時可以按 `Ctrl+C` 停止，機器會停車。

### Step 4：量點位

依照[第 5 節](#5-設定目標點)量好 `stage3_entry` 和 8 個 row 點。

### Step 5：衝田埂

FSM 停著，把機器放在 `stage3_entry`，手動衝一次（0.4 m/s 衝 2 秒）：

```bash
timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"; \
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

- 過不去 → 調 `kCrossLinearX` / `kCrossDurationSec`，再用上面的指令把數字換掉重試。
- 過去之後，執行 `ros2 topic echo /robot_pose --once`，看位置有沒有亂跳（定位有沒有跑掉）。

### Step 6：空場跑完整流程

- **場上不放稻草**，`kEnableEmptyRowExit` 先改成 `false`，再 build。
- START DIT，觀察：
  - [ ] 4 排的起點都導航得到嗎？
  - [ ] 沿排走的時候，有沒有偏離中線？（看 log 的 `offset`）
  - [ ] 每排有開到排尾嗎？退回起點、側移到下一排順不順？
  - [ ] Nav2 退回起點時是直接倒退，還是先原地轉 180°？（後者要回報，需要調 Nav2）
- 測完把 `kEnableEmptyRowExit` 改回 `true`。

### Step 7：放稻草，人工夾取

- `kAutoPick = false`，在格點上放幾捲稻草。
- **安排一個人**：機器碰到稻草停下後，就把稻草拿走。
- 觀察：
  - [ ] 碰到之後把稻草多推了幾 cm？
  - [ ] 開關有打在稻草正中間嗎？（夾子容許大約 ±3.5 cm）
  - [ ] 有沒有被隔壁排的稻草拉偏？

### Step 8：錄資料帶回來

有問題的時候錄下來，方便事後分析：

```bash
ros2 bag record /robot_pose /straw/target /cmd_vel /mechanism/straw_pick_status /mechanism/command
```

`Ctrl+C` 結束，會在目前資料夾產生一個 `rosbag2_日期時間` 的資料夾。

---

## 8. 看懂 log

沿排前進時，大約每 0.5 秒會印一行：

```
[Stage3] SWEEP row 2 progress 0.85/1.65 offset +0.012 cam=on dist=0.42 lat=+0.031 -> vx 0.25 vy -0.05 wz +0.01
```

| 欄位 | 意思 |
|---|---|
| `row 2` | 第幾排 |
| `progress 0.85/1.65` | 這排走了 0.85 m，總長 1.65 m |
| `offset +0.012` | 偏離這排中心線多少 m（正 = 偏左） |
| `cam=on` | 相機正在用（看到本排的稻草）；`off` = 沒看到或被當成隔壁排的 |
| `dist=0.42` | 相機看到的稻草距離（m） |
| `lat=+0.031` | 稻草左右偏多少（正 = 在右邊） |
| `vx / vy / wz` | 這一刻送給底盤的前進 / 橫移 / 轉向速度 |

其他常見訊息：

| 訊息 | 意思 |
|---|---|
| `碰到稻草捲，停車等待夾取` | 開關被按下，進入等待 |
| `WAIT_PICK 完成，繼續掃第 2 排` | 等待結束，繼續往前 |
| `前方沒有稻草，提早換排` | 連續 1 秒看不到稻草 |
| `到排尾` | 開到 `stage3_rowN_end` |
| `no fresh /robot_pose, hold still` | 定位資料斷了，機器停住等待 |

---

## 9. 常見問題

| 狀況 | 可能原因 | 怎麼辦 |
|---|---|---|
| 一開始就 `S3_FAILED`，log 寫 `rows not loaded` / `too short` | yaml 的 row 點還是 0，或某排的 start 和 end 太近 | 量好點位（[第 5 節](#5-設定目標點)） |
| 碰到稻草**不會停**，一直推著走 | 機構狀態沒傳上來 | `ros2 topic echo /mechanism/straw_pick_status`，按開關看 `switch_pressed` 有沒有變 |
| 停下來後**一直不走** | 開關一直被壓著（稻草沒拿走） | 把稻草拿走；最多 10 秒後也會自己繼續 |
| 對準時**越修越偏** | 方向反了 | 先做 [Step 2](#step-2底盤左右方向)；底盤正確的話，`kCamLateralGain` 改成負數 |
| 左右**一直晃** | 修正力道太大 | 調小 `kCamLateralGain` / `kLineLateralGain` |
| log 一直是 `cam=off` | 相機沒開，或稻草偏移超過閘門 | 看 `/straw/target` 有沒有資料；調大 `kCamLateralGateM` |
| 每排開一下就換排 | 「前面沒稻草就換排」生效了 | 空場測試時把 `kEnableEmptyRowExit` 設成 `false` |
| 一直停住、log 寫 `no fresh /robot_pose` | 定位掛了 | 檢查 TDK Loc_DIT 視窗有沒有錯誤 |
| 退回起點時**原地轉 180°** | Nav2 的設定 | 回報負責導航的人 |
| 衝田埂後**定位亂掉** | 撞擊造成里程計誤差 | 回報；可能要降低衝的速度，或在過田埂後重新定位 |
| build 出現 `error:` | 程式改壞了 | 看錯誤指出的檔案和行號，檢查剛剛改的地方（分號、數字格式） |

---

## 10. 指令速查

> 都在容器 terminal 裡執行，而且要先 `source /workspaces/robot_fsm_v2_ws/use_fsm.sh`。

```bash
# 進容器（在電腦的 terminal）
docker exec -it $(docker ps --filter label=com.docker.compose.service=tdk --format '{{.Names}}' | head -1) bash

# build（記得先按面板 STOP）
cd /workspaces/robot_fsm_v2_ws && ./build_fsm.sh

# 看目前位置（量點位用）
ros2 topic echo /robot_pose --once

# 看相機偵測結果
ros2 topic echo /straw/target

# 看機構狀態（開關、夾取）
ros2 topic echo /mechanism/straw_pick_status

# 鍵盤遙控（FSM 要停著）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 讓機器停下來
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# 相機對準測試工具
ros2 run robot_fsm straw_align_test --ros-args -p mode:=lateral
ros2 run robot_fsm straw_align_test --ros-args -p mode:=approach -p fast_speed:=0.15

# 錄資料
ros2 bag record /robot_pose /straw/target /cmd_vel /mechanism/straw_pick_status /mechanism/command
```

### 相關檔案

| 檔案 | 內容 |
|---|---|
| `src/robot_navigation/config/named_poses_dit.yaml` | DIT 場地的所有目標點 |
| `src/robot_fsm/include/robot_fsm/stages/stage3_hay_fsm.hpp` | 第三關的參數 |
| `src/robot_fsm/src/stages/stage3_hay_fsm.cpp` | 第三關的程式邏輯 |
| `src/robot_fsm/src/tools/straw_align_test.cpp` | 相機對準測試工具 |
| `/workspaces/field_config.yaml` | 場地設定（DIT 會跳過第二關） |
