#pragma once
// =====================================================================
//   AUTON SELECTOR ON THE CONTROLLER (src/auton-select.cpp)
// =====================================================================
// LEFT / RIGHT arrows pick the routine, shown on the controller screen. Default: Skills. The list (match autons
// first, then the tuning tools) is ROUTINES in src/auton-select.cpp.
//
// When the arrows work: only with field control / a competition switch, and only while the robot is DISABLED
// (before the match or skills run). Never during autonomous and never during driver control.
// BENCH MODE, a plain program run (nothing connected): it goes straight to driver control. Line 0 shows the pick;
// LEFT / RIGHT change it, Y runs it right there (driving stops while it runs). Hold B to end a tuning tool; a match
// auton runs to its end. Results stay on the controller until an arrow is pressed. The pick is saved on the SD card
// if there is one, so it survives a restart (a Timed Run uses it). Once field control / a competition switch has been
// connected even once since the program started, bench mode is OFF until the program is restarted: a loose field
// cable mid-match can never let Y start an auton.
//
// While a match auton runs nothing on the controller does anything. A tuning tool runs until autonomous ends
// (or B is held, in bench mode).
// =====================================================================

void selectorStart();         // runPreAutonomous(): loads the saved pick, starts the selector task
void runSelectedRoutine();    // runAutonomous(): runs the pick
void driverStarting();        // opcontrol(), first thing: driver control owns lines 1 and 2 from now
bool driverScreenOn();        // true while driver control owns controller lines 1 and 2
// Tuning tools check this to leave early: B held during a bench-mode run (with field control they end with autonomous)
bool tuneExitWanted();
bool benchRunWanted();        // opcontrol() loop, nothing connected: LEFT / RIGHT pick, true = Y pressed, run it now
void benchFinished();         // opcontrol(), after that run
