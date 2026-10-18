#pragma once

enum class MissionState
{
  BOOT,
  INIT,
  SELF_CHECK,
  LOCALIZE,          // 開機定位：對出發點初始化，成功才准進 WAIT_START
  WAIT_START,
  STAGE2_PREPARE,    // 第二關初始化：2101 等1秒、2000 等4秒、2001 等1秒

  STAGE1_WETLAND,
  TRANSITION_TO_STAGE2,

  STAGE2_CLAM,
  TRANSITION_TO_STAGE3_CORNER, // 第二關結束先到 L 形轉角
  TRANSITION_TO_STAGE3,

  STAGE3_HAY,

  RELOCALIZE,        // Legacy explicit localization path; S2/S3 RESET do not enter it

  FINISH_DECISION,
  EARLY_STOP,
  END_RUN,
  SAFE_STOP
};
