#pragma once
// =====================================================================
//   CONTROLLER SCREEN (src/pad.cpp)
// =====================================================================
// The controller's 3 lines of 19 characters, written by ONE task, so the auton selector, the autons, the tuning
// tools and the driver readouts never fight over it. Anyone sets a line with pad::line(); the task sends the lines
// that changed, one every 55 ms (the controller drops prints that come faster than every 50 ms), and re-sends all
// three every 2 s in case the controller reconnected. Rumbles go through the same task.
// =====================================================================

namespace pad {
constexpr int WIDTH = 19;   // characters per controller line

void start();   // starts the writer task (runPreAutonomous() calls it first thing)
// Sets row 0, 1 or 2 (printf-style). Longer than WIDTH is cut off.
void line(int row, const char* format, ...) __attribute__((format(printf, 2, 3)));
void lines(const char* row0, const char* row1, const char* row2);
void rumble(const char* pattern);   // "." short, "-" long, " " pause
}  // namespace pad
