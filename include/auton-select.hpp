#pragma once
// =====================================================================
//   AUTON SELECTOR ON THE CONTROLLER (src/auton-select.cpp)
// =====================================================================
// LEFT / RIGHT arrows pick the routine, shown on the controller screen. Default: Skills. The list (match autons
// first, then the tuning tools) is ROUTINES in src/auton-select.cpp.
//
// When the arrows work: only with field control / a competition switch, and only while the robot is DISABLED
// (before the match or skills run). Never during autonomous and never during driver control.
// Autons and tools ONLY run from field control / a competition switch, in the autonomous period. A plain program
// run (nothing connected) goes straight to driver control and never runs an auton (the Y "bench mode" was removed
// on 2026-11-25). The pick is saved on the SD card if there is one, so it survives a restart.
//
// While a match auton runs nothing on the controller does anything. A tuning tool runs until autonomous ends
// =====================================================================

void selectorStart();         // runPreAutonomous(): loads the saved pick, starts the selector task
void runSelectedRoutine();    // runAutonomous(): runs the pick
void driverStarting();        // opcontrol(), first thing: driver control owns lines 1 and 2 from now
bool driverScreenOn();        // true while driver control owns controller lines 1 and 2
// Tuning tools check this to leave early (always false now: they end with autonomous)
bool tuneExitWanted();
