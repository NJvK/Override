#include "robot.hpp"          // rw-config.h + the robot's devices and lift functions
#include "auton-select.hpp"   // the auton selector on the controller (src/auton-select.cpp)
#include "pad.hpp"            // the controller screen (src/pad.cpp)

// Modify autonomous, driver, or pre-auton code below. src/main.cpp calls these.

// Runs whatever is picked on the controller with LEFT / RIGHT (default: Skills). The list of match autons and
// tuning tools, and when the arrows work, are in src/auton-select.cpp / include/auton-select.hpp.
// Simulator: sh sim/run-sim.sh "<name>" ... picks by the same names (no name = Skills).
void runAutonomous() {
  runSelectedRoutine();
}

// Controller sticks read -127 to 127; RW's driver functions take -100 to 100
double stick(pros::controller_analog_e_t axis) {
  return controller_1.get_analog(axis) * 100.0 / 127.0;
}

void runDriver() {
  // Not used: opcontrol() in src/main.cpp is the driver code (drive, lift, claw, toggle, auto stack),
  // and it drives with the same feel as before. Kept because RW declares it.
}

void runPreAutonomous() {
  // The controller screen and the auton selector first, so the controller shows "IMU CAL" while it calibrates
  pad::start();
  selectorStart();

  // Calibrate the inertial sensor and wait for it (about 2 seconds). Keep the robot still.
  inertial_sensor.reset(true);

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();

  // MCL: uses the distance sensors to keep odometry from drifting while driving.
  // Turn it on with use_mcl in src/rw-config.cpp (LOCALIZATION block) once the sensors are mounted.
  // Every auton starts with setPose() at its real field spot, which is what MCL needs.
  // Field-aware version of RW's MCL (src/field-mcl.cpp): skips readings that hit goals / field pins / loaders
  if (use_mcl) {
    startFieldMcl();
  }

  // Brain screen: X / Y / heading, MCL, and what the AI Vision sensors see (src/brain-screen.cpp). Everything
  // that matters is on the controller too.
  startBrainScreen();
}
