#pragma once

// 全域State, 使 MissionController和各個Stage的FSM都能共用
// 使用時打 MissionState::BOOT
enum class MissionState
{
  BOOT,               // 上電 / 開機
  INIT,               // 初始化 ROS、client、publisher
  SELF_CHECK,         // 自我檢查
  WAIT_START,         // 等待起始信號
  LEAVE_START_ZONE,   // 離開出發區

  STAGE3_HAY,         // 第三關：稻草卷堆放
  TRANSITION_3_TO_2,  // 第三關轉第二關

  STAGE2_CLAM,        // 第二關：文蛤分級
  TRANSITION_2_TO_4,  // 第二關轉第四關

  STAGE4_MAZU,        // 第四關：北港迎媽祖
  TRANSITION_4_TO_1,  // 第四關轉第一關

  STAGE1_WETLAND,     // 第一關：濕地生態守護

  FINISH_DECISION,    // 結束判斷
  EARLY_STOP,         // 提前結束
  END_RUN,            // 正常結束
  SAFE_STOP           // 安全停止
};