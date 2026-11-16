#include "robot.hpp"   // rw-config.h + the robot's devices and lift functions
#include "fourpin.hpp"      // the 4-pin qual auton (src/fourpin.cpp)
#include "skills.hpp"       // the 1-minute skills run (src/skills.cpp)

// Modify autonomous, driver, or pre-auton code below. src/main.cpp calls these.

// There's no auton selector: autonomous runs whatever is called here. Uncomment exactly ONE line.
void runAutonomous() {
  // ---- Skills (src/skills.cpp): 1 minute, back on the LEFT wall toggle (middle of the left wall), facing right ----
  skillsAuton();

  // ---- 4-pin qual auton (src/fourpin.cpp): one route, four starts ----
  // fourPinRedBottom();    // back on the bottom wall toggle (red)
  // fourPinBlueTop();      // the same route turned 180 degrees, back on the top wall toggle (blue)
  // fourPinRedLeft();      // flipped across the pin diagonal, back on the left wall toggle (red)
  // fourPinBlueRight();    // Red Left turned 180 degrees, back on the right wall toggle (blue)

  // ---- RW tuning / calibration tools (src/rw-autons.cpp), not match autons ----
  // exampleAuton();        // "Tuning Test": drive and turn PID
  // exampleAuton2();       // "Odom Test": drives a few points and back to the start
  // odomCheckAuton();      // "Odom Check": nothing moves, push the robot and read X / Y / heading
  // squareTestAuton();     // "Square Test": 24" square, should end at 0, 0, 0
  // sensorCheckAuton();    // "Sensor Check": distance sensor readings, resets and MCL from the skills start
  // imuScaleAuton();       // "IMU Scale": spin the robot 10 turns by hand to measure imu_scale
  // feedforwardAuton();    // "Feedforward": measures drive_kS / kV / kA for profiled moves and path following
  // calibrateDrive();      // "Drive Calibration": for the simulator (see the docs: Simulator > Calibrating)
  // aiVisionTuningAuton(); // "AI Vision Tuning": white balance for this light, live pin / cup / stack readout + rumble on the controller
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

  // No auton selector: pick the auton in runAutonomous() above.

  // Brain screen: X / Y / heading, MCL, and what the AI Vision sensor sees (src/brain-screen.cpp)
  startBrainScreen();
}
