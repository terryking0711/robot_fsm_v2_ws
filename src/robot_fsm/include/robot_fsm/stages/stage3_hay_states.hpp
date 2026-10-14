#pragma once

// 第三關流程（4 排都朝同一方向掃，每排 6 個格點，共 10 捲稻草）：
// 時間不夠夾完 10 捲，從第四排開始往回掃 4 -> 3 -> 2 -> 1，
// STM 的 pick_count >= kPlaceAtPickCount（6）就馬上去放置（stage3_place）。
//
//   ENTER -> MECH_INIT -> CROSS -> NAV_ROW_START（只有第四排）-> ROW_SHIFT -> ROW_BEGIN -> SWEEP <-> WAIT_PICK -> BACKOFF -> SWEEP
//                                                                ^                           |
//                                                                |                       ROW_DONE
//                                                                |                           |
//               （還沒夠 6 捲）       row_idx_-- <- ROW_RETURN <--+---------------------------┤  沿中線直線倒車回本排起點，
//                                                                                            |  再橫移到下一排中線（不用 Nav2）
//               （夠 6 捲 / 第一排也掃完）        SPIN_180 <---------------------------------┘  等 STM 不再 PICKING，Nav2 spin 原地轉 180°
//                                                   |                （ROW_BEGIN / SWEEP / WAIT_PICK 中夠 6 捲也直接進來）
//                                                FORWARD             發 /cmd_vel 往機器 +x 前進 20 cm
//                                                   |
//                                             NAV_TO_PLACE           Nav2 到 stage3_place
//                                                   |
//                                  PLACE -> AFTER_PLACE -> DONE
//
// 第四排（第一個掃的排）超過排尾 kRow4EndOvershootM 就立即 ROW_DONE 返回，不管開關（排尾後面是不能夾的 base）。
// 夾取完全交給機構 STM（碰到開關就自己跑腳本、收進機器），FSM 只負責停車等待。
// WAIT_PICK 中開關壓著但 STM 沒開始夾（例如送 320 前就壓著）時才走 BACKOFF 退後重撞。
enum class Stage3State
{
  S3_ENTER,          // 進入第三關，檢查排的點位
  S3_MECH_INIT,      // 機構初始化：10001 -> 等 3 秒 -> 20001 -> 等 5 秒
  S3_CROSS,          // 開環直接發速度衝過田埂
  S3_NAV_ROW_START,  // Nav2 導航到第四排起點（stage3_row4_start，只有第一排用 Nav2）
  S3_ROW_SHIFT,      // FSM 發 /cmd_vel 保持車頭方向橫移到這排中線，停穩才開始掃
  S3_ROW_BEGIN,      // 等 Nav2 速度指令停下來（第一個掃的第 4 排先送 320 開啟自動夾取），開始這排的計時
  S3_SWEEP,          // FSM 直接發 /cmd_vel 沿排前進（相機 + /robot_pose 對準）
  S3_WAIT_PICK,      // 碰到稻草捲，停車等待機構夾取完成
  S3_BACKOFF,        // 開關壓著但 STM 沒開始夾（例如送 320 前就壓著）：後退到開關放開，再回 SWEEP 重撞
  S3_ROW_DONE,       // 這排結束：停車，決定下一步
  S3_ROW_RETURN,     // FSM 發 /cmd_vel 沿這排中線直線倒車回起點，之後橫移到下一排（4 -> 3 -> 2 -> 1）
  S3_SPIN_180,       // 夠 6 捲（或 4 排都掃完）：等 STM 不再 PICKING，Nav2 spin 原地轉 180°
  S3_FORWARD,        // 發 /cmd_vel 往機器 +x 前進 20 cm
  S3_NAV_TO_PLACE,   // Nav2 導航到放置點 stage3_place
  S3_PLACE,          // 送放置指令，固定等待
  S3_AFTER_PLACE,    // 發 /cmd_vel 後退，再送 9991
  S3_DONE,           // 第三關完成
  S3_FAILED          // 第三關失敗
};
