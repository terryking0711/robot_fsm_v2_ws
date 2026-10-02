#pragma once

// 第三關流程（蛇行掃描 4 排，每排 6 個格點，共 10 捲稻草）：
//
//   ENTER -> CROSS -> ┌-> NAV_ROW_START -> ARM_PICK -> SWEEP <-> WAIT_PICK
//                     |                                 |
//                     └────────── ROW_DONE <────────────┘
//                                    └-> DONE（4 排掃完或已撿滿 10 捲）
enum class Stage3State
{
  S3_ENTER,          // 進入第三關，確認機構自動夾取為 disarm
  S3_CROSS,          // 開環直接發速度衝過田埂
  S3_NAV_ROW_START,  // Nav2 導航到目前這排的起點（stage3_row{i}_start）
  S3_ARM_PICK,       // 允許機構「碰到開關就自動夾取」
  S3_SWEEP,          // FSM 直接發 /cmd_vel 沿排前進（相機 + /robot_pose 對準）
  S3_WAIT_PICK,      // 碰到稻草捲，停車等待機構夾取完成
  S3_ROW_DONE,       // 這排結束：停車、disarm，換下一排或結束
  S3_DONE,           // 第三關完成
  S3_FAILED          // 第三關失敗
};
