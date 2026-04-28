#pragma once

// 全域State, 使 MissionController和各個Stage的FSM都能共用
// 使用時打 MissionState::BOOT
enum class MissionState
{
  BOOT,
  INIT,
  SELF_CHECK,
  WAIT_START,
  LEAVE_START_ZONE,

  STAGE2_CLAM,
  TRANSITION_TO_STAGE3,
  STAGE3_HAY,
  TRANSITION_TO_STAGE4,
  STAGE4_MAZU,
  TRANSITION_TO_STAGE5,
  STAGE5_NEW,

  FINISH_DECISION,
  EARLY_STOP,
  END_RUN,
  SAFE_STOP
};