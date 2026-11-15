#pragma once
// 4-pin qualification auton (src/fourpin.cpp). One route, four starts:
//   RED BOTTOM  back on the bottom wall toggle, facing up. Red goal (-23.55, -47.09). This is the route as drawn.
//   BLUE TOP    the same route turned 180 degrees (back on the top wall toggle). Blue goal (23.55, 47.09).
//   RED LEFT    the route flipped across the diagonal the field pins sit on (back on the left wall toggle).
//   BLUE RIGHT  RED LEFT turned 180 degrees (back on the right wall toggle).
// Add them to the selector with addAuton() in src/rw-user.cpp.
void fourPinRedBottom();
void fourPinBlueTop();
void fourPinRedLeft();
void fourPinBlueRight();
