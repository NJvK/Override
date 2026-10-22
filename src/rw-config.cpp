#include "rw-config.h"

// ============================================================================
// DEVICES
// RW uses the devices below by name (see rw/devices.h), so keep the names and set the ports.
// Declare your own devices in include/rw-config.h too, so other files can use them.
// ============================================================================

pros::Controller controller_1(pros::E_CONTROLLER_MASTER);

// Drive motors: a negative port reverses a motor. Reverse them so driveChassis(12, 12) drives forward.
// Override (2026-27) rule R11 caps drivetrain motors at 55W total, e.g. 4 x 11W, 5 x 11W, or 4 x 11W + 2 x 5.5W.
// Drive encoder readings come from the FIRST motor in each group, so with mixed motors
// list an 11W motor first and set wheel_distance_in for that motor's gearing.
// (same ports and directions as leftMotors / rightMotors in main.cpp)
pros::MotorGroup left_chassis({-1, -11, -12}, pros::MotorGearset::blue);
pros::MotorGroup right_chassis({19, 10, 20}, pros::MotorGearset::blue);
// X-drive, mecanum or H-drive: see the docs (Guide > Drive Types) for the motors to make here and
// the line to add to runPreAutonomous() in src/rw-user.cpp

pros::Imu inertial_sensor(17);   // IMU_PORT in main.cpp

// Tracking wheels (a negative port reverses one). Give them any free ports if you don't use tracking wheels
// Same ports/directions as ODOM_HORIZONTAL_PORT / ODOM_VERTICAL_PORT in main.cpp.
// RW may count directions differently from LemLib: run "Odom Test" (or push the robot by hand) and check
// that pushing forward raises Y and pushing to the robot's right raises X. If one goes the wrong way, flip that sign.
pros::Rotation horizontal_tracker(13);
pros::Rotation vertical_tracker(-14);

// Distance sensors for MCL and distance resets, one facing each way. TODO: set the real ports when they're
// mounted (see LOCALIZATION below). For now they're on FREE ports so they can't clash with anything.
// (Ports in use: 1, 2, 3, 4, 5, 7, 9, 10, 11, 12, 13, 14, 17, 19, 20. Free: 6, 8, 15, 16, 18, 21.)
pros::Distance front_sensor(6);
pros::Distance left_sensor(8);
pros::Distance right_sensor(15);
pros::Distance back_sensor(16);

// The rest of the robot (lift, claw, toggle, optical sensor) is in src/main.cpp, same as before.

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
// (mecanum: plus the distance between the front and back wheels, see the docs: Guide > Drive Types)
double distance_between_wheels = 12.625;   // DRIVE_TRACK_WIDTH in main.cpp (12 5/8", measured)

// motor to wheel gear ratio * wheel diameter (in inches) * pi
// (X-drive: also * 0.707, see the docs: Guide > Drive Types)
// 600 rpm (blue) motors geared to 450 rpm at the wheel (DRIVE_WHEEL_RPM = 450) = 36:48, 2.75" drive wheels
double wheel_distance_in = (36.0 / 48.0) * 2.75 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
// Converted from the tuned LemLib values in main.cpp. LemLib's PID outputs -127..127; RW's outputs volts (-12..12),
// so each gain is multiplied by 12/127 (about 0.0945):
//   drive: kP 5.78 -> 0.55, kI 0 -> 0, kD 6.0 -> 0.57      turn: kP 3.7 -> 0.35, kI 0 -> 0, kD 25.5 -> 2.41
// They're a starting point, not final: RW's loop isn't identical to LemLib's, so check them with "Tuning Test".
// (RW's own defaults were drive 1.1 / 0.1 / 7 and turn 0.3 / 0 / 2.5 if these feel too soft.)
double distance_kp = 0.55, distance_ki = 0, distance_kd = 0.57;
double turn_kp = 0.35, turn_ki = 0, turn_kd = 2.41;
// (no LemLib equivalent: RW's default, keeps the robot straight while driving)
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = true;   // horizontal tracking wheel on port 13
bool using_vertical_tracker = true;     // vertical tracking wheel on port 14

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// These comments are in the perspective of a top down view of the robot when the robot is facing vertical
// Vertical distance from the center of the bot to the horizontal tracking wheel (in inches, positive is when the wheel is behind the center)
// main.cpp ODOM_HORIZONTAL_OFFSET = -2.25 (LemLib: behind = negative). RW: behind = POSITIVE, so +2.25
double horizontal_tracker_dist_from_center = 2.25;
// Horizontal distance from the center of the bot to the vertical tracking wheel (in inches, positive is when the wheel is to the right of the center)
// main.cpp ODOM_VERTICAL_OFFSET = -0.1 (left of center = negative in both LemLib and RW)
double vertical_tracker_dist_from_center = -0.1;
double horizontal_tracker_diameter = 2.00836820084; // ODOM_HORIZONTAL_DIAMETER (tuned with the push test)
double vertical_tracker_diameter = 2.01258746176;   // ODOM_VERTICAL_DIAMETER (tuned with the 96" push test)

// ============================================================================
// LOCALIZATION: MCL + DISTANCE RESETS (everything for "the robot knows where it is" is in this block)
// ============================================================================
// MCL (Monte Carlo localization) runs all the time while the robot drives: it compares what the 4 distance
// sensors see with where the walls should be, and keeps nudging x / y back to the truth (it fixes drift
// from bumps, wheel slip and rocking). Distance resets are one-shot fixes: the autons call them while the
// robot is standing still (at the start, at the matchloader, before the toggle sweep).
// RW's note: MCL is new and has mostly been tested in its simulator, so check it on the field (step 5).
//
// Turn them on in this order, one step at a time:
//   1. Mount the 4 sensors on the drive base, 5-9" off the floor, flat, each facing straight out
//      (front / right / back / left). Set their ports above.
//   2. Fill in each sensor's offset below: where the FACE of the sensor is, measured from the robot's center
//      (9" from the back, centered side to side). X: + = right of center, - = left. Y: + = in front, - = behind.
//   3. Run "Sensor Check" on the auton selector with the robot a tape-measured distance (2-4 ft) from a wall.
//      If the reading is a little off, set distance_sensor_scale = (tape measure) / (reading).
//   4. use_distance_resets = true. Run "Sensor Check" again from the skills start spot: X / Y should match
//      the real spot, and "rst" should count up.
//   5. use_mcl = true. The selector screen shows "MCL <spread>in <time>ms": spread should stay small (under ~2")
//      and time under a few ms (lower mcl_particles if it isn't). If the position lags while driving toward
//      or away from a wall, adjust distance_sensor_delay_msec (further down).
// Both are off until the sensors are on: with nothing plugged in they'd just do nothing.
bool use_mcl = false;              // continuous correction while driving (startMcl() in src/rw-user.cpp)
bool use_distance_resets = false;  // one-shot resets the autons do while stopped (localizeWithWalls())

// Distance sensor offsets (TODO once mounted, step 2). Unused sensors can stay 0.
// If you are not using all four distance sensors, just set the unused ones to 0
// If you are not using distance resets these values will be ignored
// Add your sensor offsets here (where the sensor's face is, relative to the robot's center)
// If a sensor is dead-center but 6.5" forward relative to the robot's center:
// double front_sensor_offsetX = 0.0;
// double front_sensor_offsetY = 6.5;
// For the X offset, a negative value indicates that the sensor is located on the left side of the robot, while a positive value indicates that it is on the right side.

double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 0.0;

double left_sensor_offsetX = 0.0;
double left_sensor_offsetY = 0.0;

double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;

double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start of movement
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end of movement

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

// ============================================================================
// NEWER SETTINGS (OPTIONAL)
// Every setting here works at its default. Change one only when you use the feature it's for.
// ============================================================================

// Multiplies the IMU heading. Many V5 IMUs read a little off over full turns (e.g. 357 for 360).
// Find yours with measureImuScale(), 1.0 = no correction
// TODO: measure with measureImuScale() once it's running on RW (an earlier spin test hinted yours may read off)
double imu_scale = 1.0;

// End a motion early once the robot has been stuck (not moving or turning) for this long, in msec.
// Saves time if the robot gets caught on something. 0 = off.
// Leave it off for moves meant to push into something until they time out. 350 or more avoids false exits.
double stall_exit_msec = 0;

// Distance resets: skip readings where the beam hits the wall at a steeper angle than this (degrees).
// The sensor's beam is a cone about 12 degrees wide on each side, so steep readings get less exact
double distance_reset_max_angle = 30;
// Distance resets: skip readings that would move the position more than this (inches, 0 = off).
// 6-10 helps ignore other robots and game elements in front of the wall
// 6 = a reading that would move the robot more than 6" is probably seeing a robot or a pin, so it's skipped.
double distance_reset_max_jump = 6;
// Distance resets: skip readings that hit the wall this close to a corner (inches, 0 = off).
// For Override, about 18 keeps the corner loaders out of the readings
// RW's value for Override: 18 keeps the corner loaders out of the readings.
double distance_reset_corner_margin = 18;

// MCL (see startMcl() in src/rw-user.cpp): particle count (more = smoother but slower, up to 1000),
// longest distance reading it trusts (inches), and how far behind the distance sensor readings are (msec).
// If the position lags while driving toward or away from a wall, adjust the delay.
// The selector screen shows how long each MCL update takes; lower the particle count if it's more than a few msec
int mcl_particles = 200;
double mcl_max_range = 60;
double distance_sensor_delay_msec = 40;

// Multiplies every distance sensor reading (resets and MCL). V5 distance sensors often read 1-3% long.
// To measure yours: put the robot a tape-measured distance from a wall (2-4 ft), then use
// (measured distance) / (what the sensor reads). 1.0 = no correction
double distance_sensor_scale = 1.0;

// Motion profiles and path following (driveProfiled, turnProfiled, followPath, followPathRamsete).
// These use a model of your drive: drive_kS = volts to start moving, drive_kV = volts per in/s,
// drive_kA = volts per in/s^2. Leave kV and kA at 0 to estimate them from drive_motor_rpm and
// wheel_distance_in, or measure all three with measureDriveFeedforward()
double drive_kS = 0.5;
double drive_kV = 0;
double drive_kA = 0;
double drive_motor_rpm = 600;          // Drive motor cartridge: 600 (blue), 200 (green), 100 (red)  (yours: blue 600, geared to 450 above)
double profile_max_speed = 0;          // Top speed (in/s, 0 = 80% of the drive's top speed)
double profile_max_accel = 100;        // Speed up / slow down rate (in/s^2): lower if the wheels slip
double profile_turn_speed = 360;       // turnProfiled top speed (degrees/s)
double profile_turn_accel = 1200;      // turnProfiled acceleration (degrees/s^2)
double path_lookahead = 12;            // Pure pursuit lookahead (inches): bigger = smoother, smaller = tighter
double path_max_lateral_accel = 120;   // Slows down in curves so the robot doesn't slide (in/s^2)

// Driver control feel, used by driverTank(), driverArcade() and driverHolonomic()
// main.cpp used a LemLib expo curve with gain 1.019 (basically linear) and a deadband of 5 out of 127 (about 4%)
double driver_curve = 0;     // 0 = linear, higher = finer control at low stick values (2-6 is typical)
double driver_deadband = 4;  // Stick values below this (percent) count as zero

// X-drives, mecanum drives and H-drives (see the docs: Guide > Drive Types): how far the robot moves
// sideways per revolution of a strafing motor (inches). 0 = the same as wheel_distance_in, right for an X-drive
double strafe_distance_in = 0;