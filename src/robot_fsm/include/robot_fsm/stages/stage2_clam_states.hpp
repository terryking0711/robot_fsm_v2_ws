#pragma once

enum class Stage2State
{
  S2_EXTEND_ARM_2,    // extend arm
  S2_PUSH_CLAM,     // push clam into the box
  S2_RETRACT_ARM_2,   // retract arm
  S2_MOVE_FORWARD_ALIGN,  // move forward and align to the box
  S2_ALLIGN_LOCK,   // keep publishing cmd_vel before locking the box
  S2_LIFT_ARM,        // lift arm
  S2_SHIFT_TO_RETURN, // 直接橫移 1 秒後導航返回點
  S2_LOCK_BOX,        // lock the box
  S2_ROTATE_BOX_1,     // rotate box upward and downward
  S2_MOVE_BACK,
  S2_MOVE_TO_RETURN,  // move to return point
  S2_PRE_DROP_BOX,    // send 206, wait 1s before move back
  S2_RETRACT_ARM_1,
  S2_DONE
  // S2_FAILED
};
