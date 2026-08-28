#pragma once

enum class Stage2State
{
  S2_ENTER,
  S2_APPROACH,      // docking to clam
  S2_EXTEND_ARM,    // extend arm
  S2_PUSH_CLAM,     // push clam into the box
  S2_RETRACT_ARM,   // retract arm
  S2_LOCKER_DOWN,  // lock the locker down
  S2_MOVE_FORWARD_ALIGN,  // move forward and align to the box
  S2_LOCK_BOX,        // lock the box
  S2_ROTATE_BOX,     // rotate box upward and downward
  S2_MOVE_TO_RETURN,  // move to return point
  S2_DROP_BOX,
  S2_DONE
  // S2_FAILED
};
