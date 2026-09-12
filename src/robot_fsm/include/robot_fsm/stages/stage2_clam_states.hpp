#pragma once

enum class Stage2State
{
  S2_ENTER,
  S2_EXTEND_ARM,    // extend arm
  S2_PUSH_CLAM,     // push clam into the box
  S2_RETRACT_ARM_2,   // retract arm
  S2_LOCKER_DOWN,  // lock the locker down
  S2_MOVE_FORWARD_ALIGN,  // move forward and align to the box
  S2_ALLIGN,        // keep publishing cmd_vel for a fixed duration
  S2_ALLIGN_LOCK,   // keep publishing cmd_vel before locking the box
  S2_LIFT_ARM,        // lift arm
  S2_LOCK_BOX,        // lock the box
  S2_ROTATE_BOX_1,     // rotate box upward and downward
  S2_ROTATE_BOX_2,     // rotate box upward and downward
  S2_MOVE_TO_RETURN,  // move to return point
  S2_DROP_BOX,
  S2_RECOVERY,
  S2_RETRACT_ARM_1,
  S2_DONE
  // S2_FAILED
};
