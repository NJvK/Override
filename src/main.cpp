#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
// #include "pros/optical.hpp" // optical sensor disabled
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iterator>
#include <cmath>
#include <cstdint> // only needed by disabled anti-tip/optical timing code
#include "pros/screen.hpp"

// ============================================================
// CONTROLS
//   L1 / L2   DR4B up / down
//   R1 / R2   intake in / out
//   A         toggle claw open / closed
//   B         toggle claw orientation piston
//   UP / DOWN manual roller spin
//
// ANTI-TIP and OPTICAL SENSOR code are kept below but commented out.
// ============================================================
//
// PORT MAP
//   left drive    17, 10 (both reversed)
//   right drive   2, 1
//   IMU           4
//   DR4B          20, 13 (20 reversed)
//   toggle roller 6, 8
//   optical       3
//   intake        NOT SET  <-- see INTAKE_PORT below
//   claw          ADI B
//   orientation   ADI H
//
// Valid V5 smart ports are 1 through 21. Port 0 does not exist.
// ============================================================

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-17, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({2, 1}, pros::MotorGearset::blue);

// Inertial Sensor on port 4
pros::Imu imu(4);

// tracking wheels
// These were on port 0, which is not a real port, AND the OdomSensors block
// below passes nullptr for every tracking wheel, so LemLib never reads them.
// Odometry currently runs off the IMU plus the drive motor encoders.
// If you add tracking wheels back, uncomment these and put real ports in,
// then put the pointers back into the OdomSensors block.
//
pros::Rotation horizontalEnc(19);
pros::Rotation verticalEnc(-16);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

const double PI = 3.14159265358979323846;

int deadband(int value) {
    if (abs(value) < 5) { return 0; }
    return value;
}

// // ============================================================
// // ANTI-TIP
// // ============================================================
// //
// // How it works: the inertial sensor reports how far the robot is leaning.
// // Once that lean passes TIP_ANGLE_ON, the driver's joystick is ignored and
// // the drivetrain is driven toward the side the robot is falling, which pulls
// // the wheels back under the center of gravity. Control goes back to the
// // driver once the lean drops below TIP_ANGLE_OFF.
// //
// // TRIGGERING SOONER. Two things do this, and they stack:
// //   1. TIP_ANGLE_ON lowered from 5.5 to 4.5 degrees.
// //   2. TIP_LOOKAHEAD_S, which adds how fast the lean is growing on top of
// //      how far it has already gone. A robot actually going over accelerates,
// //      while a robot squatting under hard acceleration does not, so this
// //      catches real tips early without lowering the bar for ordinary rocking.
// //      Set it to 0.0 to turn the prediction off and go back to plain angle.
// //
// // AXIS: set to X_ROLL because the inertial sensor is mounted sideways.
// // Line 5 of the brain screen shows the raw roll value.
// //
// // RESTING ANGLE: the sensor reads about 5 degrees with the robot level, so
// // that gets subtracted out. Sit the robot level, read line 5, and put the
// // real number here. If this is off by a degree, you lose a degree of margin.
// //
// // DIRECTION: put the robot on blocks so the wheels spin free, tilt it past
// // the trigger angle, and watch the wheels. They should spin toward the low
// // side. If they spin the wrong way, set TIP_INVERT to true.
// //
// enum class TipAxis { Y_PITCH, X_ROLL };
// constexpr TipAxis TIP_AXIS         = TipAxis::X_ROLL; // inertial is mounted sideways
// constexpr double TIP_RESTING_ANGLE = 5.0;   // raw IMU reading when robot is level
// constexpr double TIP_ANGLE_ON      = 4.5;   // activate here (was 5.5)
// constexpr double TIP_ANGLE_OFF     = 2.0;   // give control back when nearly level
// constexpr double TIP_LOOKAHEAD_S   = 0.15;  // seconds of lean rate to look ahead, 0 disables
// constexpr double TIP_KP            = 9.0;   // correction strength per degree
// constexpr double TIP_MIN_POWER     = 35.0;  // minimum correction once active
// constexpr double TIP_MAX_POWER     = 110.0; // maximum correction power
// constexpr bool   TIP_INVERT        = false; // flip if the robot pushes the wrong way
//
// bool antiTipActive = false;
//
// // Returns the lean angle on whichever axis is configured above.
// // Positive is treated as "nose up" (falling backward).
// double tipAngle() {
//     double rawAngle = (TIP_AXIS == TipAxis::Y_PITCH) ? imu.get_pitch() : imu.get_roll();
//
//     if (!std::isfinite(rawAngle)) {
//         return 0.0;
//     }
//
//     // The IMU rests at about 5 degrees, so treat 5 degrees as level.
//     double correctedAngle = rawAngle - TIP_RESTING_ANGLE;
//
//     return TIP_INVERT ? -correctedAngle : correctedAngle;
// }
//
// // --- lean rate tracking, for the lookahead ---
// double lastTipAngle = 0.0;
// std::uint32_t lastTipTime = 0;
//
// // Current lean plus where it will be TIP_LOOKAHEAD_S from now at the rate it
// // is currently moving. Leaning 4 degrees and holding stays under the
// // threshold. Leaning 4 degrees while moving 30 deg/sec reads as 8.5 and
// // trips immediately.
// double predictedLean() {
//     double angle = tipAngle();
//     std::uint32_t now = pros::millis();
//     double dt = (now - lastTipTime) / 1000.0;
//
//     double rate = 0.0;
//     // ignore the first reading and any gap long enough to be a stale sample
//     if (lastTipTime != 0 && dt > 0.001 && dt < 0.5) {
//         rate = (angle - lastTipAngle) / dt; // degrees per second
//     }
//
//     lastTipAngle = angle;
//     lastTipTime = now;
//
//     return angle + rate * TIP_LOOKAHEAD_S;
// }
//
// // Overwrites throttle and turn if a correction is needed.
// // Returns true if anti-tip took control away from the driver.
// bool antiTip(int& throttle, int& turn) {
//     double angle = tipAngle();          // drives which way to correct
//     double lean = std::fabs(predictedLean()); // drives whether to correct at all
//
//     // hysteresis: turn on at the high threshold, off at the low one, so the
//     // code does not flicker on and off right at the trigger point
//     if (!antiTipActive && lean > TIP_ANGLE_ON) {
//         antiTipActive = true;
//     } else if (antiTipActive && lean < TIP_ANGLE_OFF) {
//         antiTipActive = false;
//     }
//
//     if (!antiTipActive) { return false; }
//
//     // the further past the threshold, the harder the correction
//     double power = TIP_KP * (lean - TIP_ANGLE_OFF);
//     if (power < TIP_MIN_POWER) { power = TIP_MIN_POWER; }
//     if (power > TIP_MAX_POWER) { power = TIP_MAX_POWER; }
//
//     // nose up means the robot is falling backward, so drive backward to
//     // catch it, and the other way around for nose down
//     throttle = static_cast<int>(angle > 0 ? -power : power);
//     turn = 0;
//     return true;
// }
//
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5.78, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            6, // derivative gain (kD)
                                            .5, // anti windup
                                            1, // small error range, in inches
                                            75, // small error range timeout, in milliseconds
                                            2, // large error range, in inches
                                            150, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             25.5, // derivative gain (kD)
                                             0, // anti windup
                                             1, // small error range, in degrees
                                             50, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
// no tracking wheels connected, so odometry runs on the IMU plus the drive
// motor encoders
lemlib::OdomSensors sensors = {
    &vertical, // vertical tracking wheel
    nullptr, // no second vertical tracking wheel
    &horizontal, // horizontal tracking wheel
    nullptr, // no second horizontal tracking wheel
    &imu // inertial sensor
};

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

pros::Motor DR4B1(-20);
pros::Motor DR4B2(13);

// STILL NEEDS A REAL PORT. 0 is not a valid V5 port, so the intake will not
// move until this is set to something between 1 and 21. Ports already taken:
// 1, 2, 3, 4, 6, 8, 10, 13, 17, 20.
constexpr int INTAKE_PORT = 0;
pros::Motor intake(INTAKE_PORT);

pros::MotorGroup Toggle({6, 8});

pros::adi::DigitalOut tClaw('H'); // claw orientation piston
pros::adi::DigitalOut claw('B');  // claw open/close

// // ============================================================
// // TOGGLE ROLLER COLOR ALIGNMENT
// // ============================================================
// //
// // A starts automatic roller alignment.
// // Both half motors on ports 6 and 8 spin until the optical sensor sees
// // the selected alliance color. UP and DOWN still manually spin the rollers.
// //
// // Change OPTICAL_PORT to the actual smart port used by your optical sensor.
// constexpr int OPTICAL_PORT = 3;
// pros::Optical colorSensor(OPTICAL_PORT);
//
// enum class Alliance { RED, BLUE };
// Alliance alliance = Alliance::BLUE;
//
// constexpr double RED_HUE_MAX  = 30.0;
// constexpr double RED_HUE_WRAP = 330.0;
// constexpr double BLUE_HUE_MIN = 180.0;
// constexpr double BLUE_HUE_MAX = 250.0;
//
// constexpr int MIN_COLOR_PROXIMITY = 120;
// constexpr int TOGGLE_ROLLER_SPEED = -200;
// constexpr int COLOR_CONFIRM_MS = 50;
// constexpr int COLOR_TIMEOUT_MS = 1500;
//
// bool toggleColorActive = false;
// std::uint32_t toggleColorStart = 0;
// std::uint32_t targetColorSeenStart = 0;
//
// bool hueIsRed(double hue) {
//     return hue <= RED_HUE_MAX || hue >= RED_HUE_WRAP;
// }
//
// bool hueIsBlue(double hue) {
//     return hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX;
// }
//
// bool seesAllianceColor() {
//     int proximity = colorSensor.get_proximity();
//     if (proximity < MIN_COLOR_PROXIMITY || proximity > 255) {
//         return false;
//     }
//
//     double hue = colorSensor.get_hue();
//     if (!std::isfinite(hue)) {
//         return false;
//     }
//
//     return alliance == Alliance::RED ? hueIsRed(hue) : hueIsBlue(hue);
// }
//
// void stopToggleColorAlign() {
//     toggleColorActive = false;
//     targetColorSeenStart = 0;
//     Toggle.move_velocity(0);
// }
//
// void startToggleColorAlign() {
//     toggleColorActive = true;
//     toggleColorStart = pros::millis();
//     targetColorSeenStart = 0;
// }
//
// void updateToggleColorAlign() {
//     if (!toggleColorActive) {
//         return;
//     }
//
//     std::uint32_t now = pros::millis();
//
//     if (now - toggleColorStart >= COLOR_TIMEOUT_MS) {
//         stopToggleColorAlign();
//         return;
//     }
//
//     if (seesAllianceColor()) {
//         if (targetColorSeenStart == 0) {
//             targetColorSeenStart = now;
//         }
//
//         if (now - targetColorSeenStart >= COLOR_CONFIRM_MS) {
//             stopToggleColorAlign();
//             controller.rumble(".");
//             return;
//         }
//     } else {
//         targetColorSeenStart = 0;
//     }
//
//     Toggle.move_velocity(TOGGLE_ROLLER_SPEED);
// }
//
// void runToggleToAllianceColor() {
//     startToggleColorAlign();
//
//     while (toggleColorActive) {
//         updateToggleColorAlign();
//         pros::delay(10);
//     }
// }
//
// ============================================================
// CLAW
// ============================================================

bool clawOn = false;
bool tclawOn = true; // starts down, so starts activated

constexpr std::uint32_t CLAW_DELAY_MS = 250;
std::uint32_t lastClawChange = 0;

void toggleClaw() {
    std::uint32_t now = pros::millis();

    // Don't allow the claw to switch again until 150 ms has passed.
    if (lastClawChange != 0 &&
        now - lastClawChange < CLAW_DELAY_MS) {
        return;
    }

    clawOn = !clawOn;
    claw.set_value(clawOn);

    lastClawChange = now;
}

void toggleClawO() {
    tclawOn = !tclawOn;
    tClaw.set_value(tclawOn);
}

// --- orientation piston, automatic [ DISABLED ] ---
//
// The piston was driven off the DR4B motor encoder: ON when the lift sits at
// its starting position, OFF once raised. Now on manual toggle with B.
// To go back to automatic, uncomment this block, the screen line in
// initialize(), and the updateClawOrientation() calls in DR4B() and the
// opcontrol loop.
//
// constexpr double DR4B_DOWN_POS = 25;  // below this, the lift counts as down
// constexpr double DR4B_UP_POS   = 60;  // above this, the lift counts as up
//
// double dr4bPosition() {
//     double pos = DR4B1.get_position();
//     if (!std::isfinite(pos)) { return 0.0; } // motor unplugged
//     return pos;
// }
//
// void updateClawOrientation() {
//     double pos = dr4bPosition();
//     if (!tclawOn && pos < DR4B_DOWN_POS) {
//         tclawOn = true;
//         tClaw.set_value(true);
//     } else if (tclawOn && pos > DR4B_UP_POS) {
//         tclawOn = false;
//         tClaw.set_value(false);
//     }
// }

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // OPTICAL SENSOR DISABLED
    // colorSensor.set_led_pwm(100);
    // colorSensor.set_integration_time(20);

    // zero the lift encoders with the DR4B physically all the way down
    DR4B1.tare_position();
    DR4B2.tare_position();
    // hold, so the lift does not sag under its own weight
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // starting pneumatic states
    tclawOn = true;
    tClaw.set_value(true);
    claw.set_value(clawOn);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // // ANTI-TIP tuning. "Lean" is the corrected angle, "Pred" is that
            // // plus the lookahead. Pred is what the trigger actually compares
            // // against, so watch the gap between them while driving.
            // pros::lcd::print(3, "Lean: %.1f  Pred: %.1f",
            // tipAngle(), predictedLean());
            // pros::lcd::print(4, "Y/Pitch: %.1f", imu.get_pitch());
            // pros::lcd::print(5, "X/Roll: %.1f  Tip: %s",
            // imu.get_roll(), antiTipActive ? "ACT" : "off");

            // pros::lcd::print(6, "Hue: %.0f  Prox: %d",
            // colorSensor.get_hue(), colorSensor.get_proximity());
            // pros::lcd::print(7, "Alliance: %s Roller: %s",
            // alliance == Alliance::RED ? "RED" : "BLUE",
            // toggleColorActive ? "AUTO" : "off");


            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void exit_condition(lemlib::Pose target, double exitDist) {
    chassis.waitUntil(fabs(chassis.getPose().distance(target)) - exitDist);
    chassis.cancelMotion();
}

void DR4B(float speed) {
    DR4B2.move_velocity(speed);
    DR4B1.move_velocity(speed);
}

void DR4BStop() {
    DR4B2.move_velocity(0);
    DR4B1.move_velocity(0);
}
void redLeft() {
    // alliance = Alliance::RED; // optical sensor disabled
    // set the starting position for the robot
    chassis.setPose(60.757, -2.321, 335.39);

    // moves back to toggle for roller using flex wheel mech
    chassis.moveToPose(63.483, -6.733, 0, 1000, {.forwards = false}); // moves back
    chassis.moveToPose(60.757, -2.321, 335.39, 1000);
    chassis.moveToPose(63.483, -6.733, 0, 1000, {.forwards = false}); // moves back
    // gets toggle ^^^^^^^
    chassis.moveToPose(60.176, 2.705, 304.768, 1000);
    chassis.moveToPose(47.646, 8.123, 0, 1000);
    // lifts up DR4B to score preloads
    DR4B(60);
    chassis.moveToPose(46.981, 18.713, 358.449, 1000);
    toggleClaw();// score preloads/toggle claw

    chassis.moveToPose(44.728, 8.305, 41.698, 1000, {.forwards = false});// moves back
    // move DR4B down to pick up more pins
    DR4B(-30);
    chassis.moveToPose(56.366, 23.274, 90.777, 1000);
    DR4BStop();
    chassis.moveToPose(65.356, 23.171, 89.119, 1000, {.maxSpeed = 50, });// moves slower
    // picks up pins
    toggleClaw();
    chassis.moveToPose(60.473, 23.059, 91.173, 1000, {.forwards = false});// moves back
    chassis.moveToPose(50.656, 22.985, 271.853, 1000);
    // score pins
    toggleClaw();
    chassis.moveToPose(58.561, 22.914, 270.113, 1000, {.forwards = false});// moves back
    chassis.moveToPose(47.105, -6.802, 113.795, 1000, {.maxSpeed = 90, .minSpeed = 50,});// moves to allign with pick up more pins
    chassis.moveToPose(57.865, -23.834, 90.346, 1000);
    chassis.moveToPose(67.075, -23.726, 87.604, 1000, {.maxSpeed = 40});// moves slow
    chassis.moveToPose(61.021, -23.978, 86.23, 1000, {.forwards = false});// moves back 
    chassis.moveToPose(56.812, -24.014, 272.699, 1000);
    // lift up DR4B to score pins
    DR4B(60);
    chassis.moveToPose(51.164, -24.063, 276.657, 1000);
    // score pins
    toggleClaw();
}

void redRight() {
    // alliance = Alliance::RED; // optical sensor disabled
    chassis.setPose(9.712, 62.005, 143.063);

    chassis.moveToPose(5.772, 67.051, 141.808, 1000, {.forwards = false}); // reverse
    // toggle
    toggleClaw();
    chassis.moveToPose(14.788, 55.158, 135.534, 1000);
    // move DR4B up
    DR4B(40);
    chassis.moveToPose(20.144, 49.583, 134.468, 1000, {.maxSpeed = 50}); // move slow
    chassis.moveToPose(14.65, 55.277, 182.499, 1000, {.forwards = false}); // reverse
    // move DR4B down
    DR4B(-50);
    chassis.moveToPose(12.777, 24.208, 93.799, 1000);
    DR4BStop();
    chassis.moveToPose(20.289, 23.374, 90.975, 1000, {.maxSpeed = 50}); // slow
    // toggle claw
    toggleClaw();
    chassis.moveToPose(6.577, 25.931, 45.035, 1000, {.maxSpeed = 50});// slow
    // move DR4B up
    DR4B(50);
    chassis.moveToPose(20.743, 44.036, 42.701, 1000);
    // toggle claw
    toggleClaw();
    chassis.moveToPose(14.762, 36.52, 38.802, 1000, {.forwards = false}); // reverse
    // move DR4B down
    DR4B(-50);
    chassis.moveToPose(-23.857, 54.275, 0, 1000);
    DR4BStop();
    chassis.moveToPose(-23.482, 63.729, 0, 1000, {.maxSpeed = 60}); // slow
    chassis.moveToPose(-23.449, 58.258, 0, 1000, {.forwards = false}); // reverse
    // move DR4B up
    DR4B(50);
    chassis.moveToPose(-26.737, 49.893, 134.928, 1000);
    DR4B(-70);
    pros::delay(300);
    // toggle claw
    toggleClaw();

}

void blueRight() {
    // alliance = Alliance::BLUE; // optical sensor disabled
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // set the starting position for the robot
    chassis.setPose(0, 0, 0);

    DR4B(80);
    pros::delay(500);
    DR4B(-80);
    chassis.moveToPoint(0, -5, 1000, {.forwards = false});
    pros::delay(450);
    DR4BStop();
    chassis.moveToPoint(0, 14, 2000, {.minSpeed = 90, .earlyExitRange = 2});
    chassis.turnToHeading(-89, 1000);
    chassis.moveToPose(-38.93, 17.8, -91.89, 1500, {.minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(300);
    toggleClaw();
    pros::delay(500);
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // chassis.moveToPose(-13.78, 17.7, -88.16, 1000, {.forwards = false, .minSpeed = 80, .earlyExitRange = 2});
    // chassis.turnToHeading(-43.66, 1000);
    // chassis.moveToPose(-32.59, 46.67, -46.55, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    // pros::delay(1500);
    // toggleClaw();
    // pros::delay(500);
    // DR4B(200);
    // pros::delay(400);
    // DR4BStop();
    // pros::delay(200);
    // chassis.turnToHeading(-161.87, 1000);
    // chassis.moveToPose(-48.74, 27.706, -167.48, 1000, {.minSpeed = 70, .earlyExitRange = 2});
    // pros::delay(1000);
    // DR4B(-200);
    // pros::delay(200);
    // DR4BStop();
    // pros::delay(1000);
    // toggleClaw();
}

void blueLeft() {
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // set the starting position for the robot
    chassis.setPose(0, 0, 0);

    DR4B(80);
    pros::delay(500);
    DR4B(-80);
    chassis.moveToPoint(0, -5, 1000, {.forwards = false});
    pros::delay(450);
    DR4BStop();
    chassis.moveToPoint(0, 12, 2000, {.minSpeed = 90, .earlyExitRange = 2});
    chassis.turnToHeading(95, 1000);
    chassis.moveToPose(26.406, -2.20, 100.94, 1500, {.minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(300);
    toggleClaw();
    pros::delay(500);
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void skills() {
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // set the starting position for the robot
    chassis.setPose(0, 0, 0);

    DR4B(80);
    pros::delay(500);
    DR4B(-80);
    chassis.moveToPoint(0, -5, 1000, {.forwards = false});
    pros::delay(450);
    DR4BStop();
    chassis.moveToPoint(0, 12, 2000, {.minSpeed = 90, .earlyExitRange = 2});
    chassis.turnToHeading(95, 1000);
    chassis.moveToPose(26.406, -2.20, 100.94, 1500, {.minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(300);
    toggleClaw();
    pros::delay(500);
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.moveToPoint(0, 14, 2000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 2});
    chassis.turnToHeading(0, 1000);
    chassis.moveToPoint(0, 100, 2000);
}

void autonomous() {
    // redLeft();
    // redRight();
    // blueLeft();
    // blueRight();
    // skills();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

    // ANTI-TIP DISABLED
    // bool tipWasActive = false;

    while (true) {
        // ---- drive ----
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // Normal driver control with no anti-tip override.
        chassis.arcade(leftY, 0.9 * rightX);

        // ---- L1 / L2: DR4B ----
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            DR4B1.move_velocity(90);
            DR4B2.move_velocity(90);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            DR4B1.move_velocity(-90);
            DR4B2.move_velocity(-90);
        } else {
            DR4B1.move_velocity(0);
            DR4B2.move_velocity(0);
        }

        // ---- R1 / R2: intake ----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            toggleClaw();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

        } else {

        }

        // ---- A: claw open / closed ----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            
        }

        // ---- B: claw orientation piston ----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            
        }

        // ---- UP / DOWN: manual toggle roller ----
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {

        } else {

        }

        // OPTICAL SENSOR AUTO-ALIGN DISABLED
        // To bring it back later, uncomment the optical section above
        // and restore the automatic roller calls here.

        // delay to save resources
        pros::delay(10);
    }
}