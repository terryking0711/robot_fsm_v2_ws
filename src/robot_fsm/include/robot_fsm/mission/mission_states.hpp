#pragma once

enum class MissionState
{
  BOOT,
  INIT,
  SELF_CHECK,
  LOCALIZE,          // 開機定位：對出發點初始化，成功才准進 WAIT_START
  WAIT_START,
  LEAVE_START_ZONE,

  STAGE1_WETLAND,
  TRANSITION_TO_STAGE2,

  STAGE2_CLAM,
  TRANSITION_TO_STAGE3,

  STAGE3_HAY,

  RELOCALIZE,        // 場外重置：在該關重置點重新定位，成功後跳回該關起點

  FINISH_DECISION,
  EARLY_STOP,
  END_RUN,
  SAFE_STOP
};
