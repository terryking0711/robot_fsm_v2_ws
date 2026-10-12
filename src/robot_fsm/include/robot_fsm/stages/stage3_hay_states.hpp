#pragma once

// 第三關流程（4 排都朝同一方向掃，每排 6 個格點，共 10 捲稻草）：
//
//   ENTER -> CROSS -> NAV_ROW_START -> ROW_BEGIN -> SWEEP <-> WAIT_PICK -> BACKOFF -> SWEEP
//                          ^                         |
//                          |                     ROW_DONE
//                          |                         |
//     （第 1~3 排）  row_idx_++ <- NAV_ROW_RETURN <---┤  Nav2 退回本排起點，再 Nav2 側移到下一排起點
//                                                    |
//     （最後一排）  PLACE <- ROTATE_AT_PLACE <--------┤  開到排尾：直接原地轉
//                     |             ^                 |
//                    DONE           └- NAV_TO_PLACE <-┘  沒到排尾（例如超時）：Nav2 先到排尾點
//
// 夾取完全交給機構 STM（碰到開關就自己跑腳本、收進機器），FSM 只負責停車等待。
// WAIT_PICK 中開關壓著但 STM 沒開始夾（例如送 320 前就壓著）時才走 BACKOFF 退後重撞。
enum class Stage3State
{
  S3_ENTER,          // 進入第三關，檢查排的點位
  S3_CROSS,          // 開環直接發速度衝過田埂
  S3_NAV_ROW_START,  // Nav2 導航到目前這排的起點（stage3_row{i}_start）
  S3_ROW_BEGIN,      // 等 Nav2 速度指令停下來（第 1 排先送 320 開啟自動夾取），開始這排的計時
  S3_SWEEP,          // FSM 直接發 /cmd_vel 沿排前進（相機 + /robot_pose 對準）
  S3_WAIT_PICK,      // 碰到稻草捲，停車等待機構夾取完成
  S3_BACKOFF,        // 開關壓著但 STM 沒開始夾（例如送 320 前就壓著）：後退到開關放開，再回 SWEEP 重撞
  S3_ROW_DONE,       // 這排結束：停車，決定下一步
  S3_NAV_ROW_RETURN, // Nav2 退回剛掃完這排的起點，之後換下一排
  S3_NAV_TO_PLACE,   // Nav2 導航到放置點（stage3_row{N}_end，yaw = 掃描方向）
  S3_ROTATE_AT_PLACE, // FSM 發 /cmd_vel 原地轉 stage3_place_rotate 的 dyaw（正 = 逆時針）
  S3_PLACE,          // 送放置指令，固定等待
  S3_DONE,           // 第三關完成
  S3_FAILED          // 第三關失敗
};
