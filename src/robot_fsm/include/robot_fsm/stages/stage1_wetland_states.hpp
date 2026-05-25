#pragma once

enum class Stage1State
{
  S1_ENTER,
  S1_LOCALIZE,
  S1_APPROACH_ART,
  S1_FIX_ART,
  S1_APPROACH_BIRD,
  S1_PICK_BIRD,
  S1_MOVE_TO_RESCUE,
  S1_DROP_BIRD,
  S1_VERIFY,
  S1_DONE,
  S1_FAILED
};