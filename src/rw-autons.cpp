#include "rw-chassis.hpp"   // rw-config.h + localizeWithWalls()
#include <cstdio>          // snprintf
#include "rw/hardware.h"   // hw::screenRow (Brain screen)

// Tuning and test routines. The robot's real autons are in auton.cpp, autonmirror.cpp and skillsauto.cpp.
// IMPORTANT: Remember to add respective function declarations to include/rw-config.h
// Then add each routine to the auton selector with addAuton() in runPreAutonomous() (src/rw-user.cpp)

namespace {
// Prints on the controller's three lines, one line every 60 ms (the controller only takes one print every 50 ms)
void controllerLines(const char* line1, const char* line2, const char* line3) {
  controller_1.print(0, 0, "%-19s", line1);
  pros::delay(60);
  controller_1.print(1, 0, "%-19s", line2);
  pros::delay(60);
  controller_1.print(2, 0, "%-19s", line3);
  pros::delay(60);
}
}  // namespace

/*
 * "Tuning Test": drive and turn PID (distance_kp/kd and turn_kp/kd in src/rw-config.cpp).
 * Drives 60" and turns through a few angles. Each turn draws its PID graph on the Brain screen:
 * green line = target, red = the robot. Tune until it gets there fast with no wobble.
 * At the end the Brain screen and controller show where it thinks it ended (should be about 0, 0, 0).
 */
void exampleAuton() {
  // Drive motor encoder distance (what driveTo counts), to compare with the tracking wheels (y_pos)
  auto encoder_in = [] {
    return (getLeftRotationDegree() + getRightRotationDegree()) / 2.0 / 360.0 * wheel_distance_in;
  };
  setPose(0, 0, 0);
  double enc_start = encoder_in();
  driveTo(60, 3000);
  pros::delay(300);                 // let it finish rolling, so this is where it really stopped
  double first_drive_y = y_pos;     // should be 60.0: shown at the end
  double first_drive_enc = encoder_in() - enc_start;
  turnToAngle(90, 2000);
  turnToAngle(135, 2000);
  turnToAngle(150, 2000);
  turnToAngle(160, 2000);
  turnToAngle(165, 2000);
  turnToAngle(0, 2000);
  double before_back_y = y_pos;     // where it starts the drive back (the turns can move it a little)
  double before_back_enc = encoder_in();
  driveTo(-60, 3000);
  pros::delay(300);
  double back_enc = before_back_enc - encoder_in();
  char line1[32], line2[32];
  snprintf(line1, sizeof(line1), "X %.2f Y %.2f", x_pos, y_pos);
  snprintf(line2, sizeof(line2), "H %.1f", getInertialHeading());
  hw::screenRow(1, "TUNING DONE (should be about 0, 0, 0)");
  hw::screenRow(2, "%s  %s", line1, line2);
  hw::screenRow(3, "Forward 60: went %.2f   Back 60: went %.2f", first_drive_y, before_back_y - y_pos);
  hw::screenRow(4, "Turns moved Y by %.2f", before_back_y - first_drive_y);
  hw::screenRow(5, "Drive motors counted: fwd %.2f  back %.2f", first_drive_enc, back_enc);
  while (true) controllerLines("TUNING DONE", line1, line2);
}

/*
 * "Odom Test": drives a few points and comes back to where it started. X / Y should read about 0, 0 at the end.
 */
void exampleAuton2() {
  setPose(0, 0, 0);
  moveToPoint(24, 24, 1, 2000, false);
  moveToPoint(48, 48, 1, 2000, true);
  moveToPoint(24, 24, -1, 2000, true);
  moveToPoint(0, 0, 1, 2000, true);
  correct_angle = 0;
  driveTo(24, 2000, false, 8);
  turnToAngle(90, 800, false);
  turnToAngle(180, 800, true);
}

/*
 * "Odom Check": nothing moves. Push the robot by hand and read the controller:
 *   forward -> Y goes up, to its right -> X goes up, clockwise -> heading goes up.
 *   L / R = inches the drive motors have counted (what driveTo uses). Push it forward 24": both should read
 *   about 24. One stuck at 0 = the first motor in that group (port 1 left, 19 right) isn't reporting.
 *   Top line = each left motor's inches, in left_chassis order (a half motor reads about 1/3 of a blue one).
 * If X or Y goes the wrong way, flip that tracking wheel's port sign in src/rw-config.cpp.
 * Push it 96" along a wall to check the tracking wheel diameters (Y should read 96.0).
 */
void odomCheckAuton() {
  setPose(0, 0, 0);
  stopChassis(pros::MotorBrake::coast);   // easy to push
  const double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  // Each left motor on its own (in left_chassis order): a half motor reads about 1/3 of a blue one
  double start_each[3] = {0, 0, 0};
  for (int i = 0; i < 3; i++) start_each[i] = left_chassis.get_position(i);
  char line0[32], line1[32], line2[32];
  while (true) {
    double each[3];
    for (int i = 0; i < 3; i++) each[i] = (left_chassis.get_position(i) - start_each[i]) / 360.0 * wheel_distance_in;
    snprintf(line0, sizeof(line0), "%.0f %.0f %.0f", each[0], each[1], each[2]);
    snprintf(line1, sizeof(line1), "X %.2f Y %.2f", x_pos, y_pos);
    snprintf(line2, sizeof(line2), "H%.0f L%.1f R%.1f", getInertialHeading(),
             (getLeftRotationDegree() - start_left) / 360.0 * wheel_distance_in,
             (getRightRotationDegree() - start_right) / 360.0 * wheel_distance_in);
    controllerLines(line0, line1, line2);
  }
}

/*
 * "Square Test": a 24" square that comes back to the start. The controller and the Brain screen show where the
 * robot thinks it ended (should be about 0, 0, 0). The Brain screen keeps it after a Timed Run switches to
 * driver control (the driver readout takes over the controller). Run it 3 times: the same error every time = PID or
 * offsets, a different error every time = wheel slip or a loose tracking wheel.
 */
void squareTestAuton() {
  setPose(0, 0, 0);
  const double square[4][2] = {{0, 24}, {24, 24}, {24, 0}, {0, 0}};
  for (const auto& point : square) {
    turnToPoint(point[0], point[1], 1, 1500);
    moveToPoint(point[0], point[1], 1, 2500);
  }
  turnToAngle(0, 1500);
  char line1[32], line2[32];
  snprintf(line1, sizeof(line1), "X %.2f Y %.2f", x_pos, y_pos);
  snprintf(line2, sizeof(line2), "H %.1f", getInertialHeading());
  hw::screenRow(1, "SQUARE DONE (should be about 0, 0, 0)");
  hw::screenRow(2, "%s  %s", line1, line2);
  while (true) controllerLines("SQUARE DONE", line1, line2);
}

/*
 * "Sensor Check": for setting up the distance sensors, resets and MCL (LOCALIZATION block in src/rw-config.cpp).
 * Put the robot at the skills start (back on the left wall, centered, facing right). Nothing moves.
 * The controller shows each sensor's reading in inches (front / right / back / left, "--" = sees nothing),
 * the robot's X / Y / heading, how many distance resets have worked ("rst", if use_distance_resets is on),
 * and the MCL spread (if use_mcl is on). You can push the robot around and watch X / Y follow.
 */
void sensorCheckAuton() {
  setPose(-61.95, 0, 90);   // skills start: center 8.3" off the left wall (walls at +-70.25), facing right
  stopChassis(pros::MotorBrake::coast);
  pros::Distance* sensors[4] = {&front_sensor, &right_sensor, &back_sensor, &left_sensor};
  int resets = 0;
  char line1[32], line2[32], line3[32];
  while (true) {
    resets += localizeWithWalls();
    char reading[4][8];
    for (int i = 0; i < 4; i++) {
      int32_t mm = sensors[i]->get();
      if (mm > 0 && mm < 9999) {
        snprintf(reading[i], sizeof(reading[i]), "%.1f", mm / 25.4 * distance_sensor_scale);
      } else {
        snprintf(reading[i], sizeof(reading[i]), "--");
      }
    }
    snprintf(line1, sizeof(line1), "F%s R%s", reading[0], reading[1]);
    snprintf(line2, sizeof(line2), "B%s L%s", reading[2], reading[3]);
    if (mcl_running) {
      snprintf(line3, sizeof(line3), "%.1f %.1f s%.1f", x_pos, y_pos, mcl_spread);
    } else {
      snprintf(line3, sizeof(line3), "%.1f %.1f r%d", x_pos, y_pos, resets);
    }
    controllerLines(line1, line2, line3);
    pros::delay(300);
  }
}

/*
 * "Feedforward": measures drive_kS / drive_kV / drive_kA (RW's motion profiles and path following need them).
 * Give the robot about 4 feet of open space in front and behind. Copy the numbers on the Brain screen into
 * src/rw-config.cpp. Never ends.
 */
void feedforwardAuton() {
  measureDriveFeedforward();
}

/*
 * "IMU Scale": line the robot up with a tile edge, spin it by hand exactly 10 full turns clockwise and line
 * it up again. The Brain screen shows the imu_scale to put in src/rw-config.cpp. Never ends.
 */
void imuScaleAuton() {
  measureImuScale(10);
}