#pragma once

// Eye-animation types — kept in a separate header so the Arduino IDE
// auto-prototype generator (which prepends prototypes at the very top
// of the .ino file) sees the type definition before it sees any
// function signature that uses it. With this declaration tucked into a
// .h, the IDE leaves it untouched and the include order works as expected.

#include <stdint.h>

struct EyeState {
  int8_t  dx;          // pupil offset x (-3..+3)
  int8_t  dy;          // pupil offset y (-2..+2)
  uint8_t lid_top;     // 0 (open) … EYE_H/2 (fully closed)
  uint8_t lid_bot;     // mirror — symmetric blink
  int8_t  brow_dy;     // -2 angry, 0 neutral, +2 happy
};
