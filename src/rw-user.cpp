#include "robot.hpp"   // rw-config.h + the robot's autons (twoPinAlliance, manyPinAlliance, ...)
#include "rw/hardware.h"   // hw::screenClear

extern bool selector_running;   // RW's auton selector: false stops it (what runSelectedAuton() does first)

// Modify autonomous, driver, or pre-auton code below. src/main.cpp calls these.

// Test that autonomous always runs, whatever is picked on the selector (for tuning with the controller's Timed Run):
//   exampleAuton = "Tuning Test", squareTestAuton = "Square Test", exampleAuton2 = "Odom Test",
//   odomCheckAuton = "Odom Check", sensorCheckAuton = "Sensor Check", manyPinAlliance = "Skills",
//   qualLeftAlliance = "Qual Left", qualRightAlliance = "Qual Right", feedforwardAuton = "Feedforward", ...
// SET IT BACK TO nullptr BEFORE A MATCH, or Skills / the qual autons won't run.
void (*const TIMED_RUN_TEST)() = manyPinAlliance;

void runAutonomous() {
  if (TIMED_RUN_TEST != nullptr) {
    // Stop the selector first, like runSelectedAuton(), or it keeps drawing over the test's PID graphs
    selector_running = false;
    hw::screenClear(0x000000);
    TIMED_RUN_TEST();
    return;
  }
  // Runs the routine picked on the Brain screen (or with the controller arrows while disabled).
  // Add routines to the list in runPreAutonomous() below.
  runSelectedAuton();
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

  // Autonomous routines for the selector, in the order they appear on the Brain screen (up to 12).
  // The first one is picked by default; the choice is saved on the SD card if one is in.
  addAuton("Skills", manyPinAlliance);               // skillsauto.cpp: 1-minute skills run
  addAuton("Qual 2-Pin R", twoPinAlliance);          // auton.cpp: preload + 2 pins, right wall
  addAuton("Qual Mirror R", mirrorAllianceRight);    // autonmirror.cpp: 3-pin route, right wall
  addAuton("Qual Mirror Top", mirrorAllianceTop);    // autonmirror.cpp: 3-pin route, top wall
  addAuton("Tuning Test", exampleAuton);             // drive + turn PID
  addAuton("Qual Right", qualRightAlliance);         // quals.cpp: Qual Left mirrored, from the right wall
  // (the selector holds 12: "Odom Test" (exampleAuton2) made room for "Qual Right"; run it with TIMED_RUN_TEST)
  addAuton("Odom Check", odomCheckAuton);            // push by hand, read X / Y / heading
  addAuton("Square Test", squareTestAuton);          // 24" square, should end at 0, 0, 0
  addAuton("Sensor Check", sensorCheckAuton);        // distance sensors, resets, MCL
  addAuton("IMU Scale", imuScaleAuton);              // measure imu_scale
  addAuton("Drive Calibration", calibrateDrive);     // for the simulator (see the docs: Simulator > Calibrating)
  addAuton("Qual Left", qualLeftAlliance);           // quals.cpp: skills opening from the left wall (preload + 2 field pins)
  // (the selector holds 12, so "Feedforward" isn't listed: run it with TIMED_RUN_TEST = feedforwardAuton above)
  startAutonSelector();
}