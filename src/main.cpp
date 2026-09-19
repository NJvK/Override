#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iterator>
#include <cmath>
#include <cstdint>
#include "pros/screen.hpp"

// ============================================================
// CONTROLS
//   L1 / L2   DR4B up / down
//   R1 / R2   intake in / out
//   A         toggle claw open / closed
//   automatic orientation piston follows DR4B height
//
// ANTI-TIP and COLOR SORT are commented out right now. Search for
// "ANTI-TIP" and "COLOR SORT" to find every block that has to be
// uncommented to turn them back on. Each one is marked.
// ============================================================
//
// PORT MAP
//   left drive    17, 10 (both reversed)
//   right drive   2, 1
//   IMU           4
//   DR4B          20, 13 (20 reversed)
//   intake        NOT SET  <-- see INTAKE_PORT below
//   claw          ADI A
//   orientation   ADI B
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
// pros::Rotation horizontalEnc(0);
// pros::Rotation verticalEnc(-0);
// lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

const double PI = 3.14159265358979323846;

int deadband(int value) {
    if (abs(value) < 5) { return 0; }
    return value;
}

// ============================================================
// ANTI-TIP  [ DISABLED - uncomment this whole block to re-enable ]
// ============================================================
//
// How it works: the inertial sensor reports how far the robot is leaning.
// Once that lean passes TIP_ANGLE_ON, the driver's joystick is ignored and
// the drivetrain is driven toward the side the robot is falling, which pulls
// the wheels back under the center of gravity. Control goes back to the
// driver once the lean drops below TIP_ANGLE_OFF.
//
// SETUP STEP 1: which axis.
//   Set to the y-axis, which is pitch, meaning nose up and nose down.
//   The x-axis is roll, meaning side to side. Lift the front of the robot by
//   hand and confirm the y/pitch number is the one that moves. If it is the
//   other one, change TIP_AXIS to TipAxis::X_ROLL.
//
// SETUP STEP 2: direction.
//   Put the robot on blocks so the wheels spin free, tilt it past the
//   trigger angle, and watch the wheels. They should spin toward the low
//   side. If they spin the wrong way, set TIP_INVERT to true.
//
// OTHER PLACES TO UNCOMMENT: the pitch/roll screen lines in initialize(),
// and the anti-tip section at the top of the opcontrol loop.
//
enum class TipAxis { Y_PITCH, X_ROLL };
constexpr TipAxis TIP_AXIS     = TipAxis::X_ROLL;
constexpr double TIP_RESTING_ANGLE = 5.0; // raw IMU reading when robot is level // y-axis, nose up / nose down //x-axis since inertial is sideways
constexpr double TIP_ANGLE_ON  = 5.5;   // activate sooner
constexpr double TIP_ANGLE_OFF = 2.0;   // give control back when nearly level
constexpr double TIP_KP        = 9.0;   // stronger correction
constexpr double TIP_MIN_POWER = 35.0;  // minimum correction once active
constexpr double TIP_MAX_POWER = 110.0; // maximum correction power
constexpr bool   TIP_INVERT    = false;  // flip if the robot pushes the wrong way

bool antiTipActive = false;

// Returns the lean angle on whichever axis is configured above.
// Positive is treated as "nose up" (falling backward).
double tipAngle() {
    double rawAngle = (TIP_AXIS == TipAxis::Y_PITCH) ? imu.get_pitch() : imu.get_roll();

    if (!std::isfinite(rawAngle)) {
        return 0.0;
    }

    // The IMU rests at about 5 degrees, so treat 5 degrees as level.
    double correctedAngle = rawAngle - TIP_RESTING_ANGLE;

    return TIP_INVERT ? -correctedAngle : correctedAngle;
}

// Overwrites throttle and turn if a correction is needed.
// Returns true if anti-tip took control away from the driver.
bool antiTip(int& throttle, int& turn) {
    double angle = tipAngle();
    double lean = std::fabs(angle);

    // hysteresis: turn on at the high threshold, off at the low one, so the
    // code does not flicker on and off right at the trigger point
    if (!antiTipActive && lean > TIP_ANGLE_ON) {
        antiTipActive = true;
    } else if (antiTipActive && lean < TIP_ANGLE_OFF) {
        antiTipActive = false;
    }

    if (!antiTipActive) { return false; }

    // the further past the threshold, the harder the correction
    double power = TIP_KP * (lean - TIP_ANGLE_OFF);
    if (power < TIP_MIN_POWER) { power = TIP_MIN_POWER; }
    if (power > TIP_MAX_POWER) { power = TIP_MAX_POWER; }

    // nose up means the robot is falling backward, so drive backward to
    // catch it, and the other way around for nose down
    throttle = static_cast<int>(angle > 0 ? -power : power);
    turn = 0;
    return true;
}

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
lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel
    nullptr, // no second vertical tracking wheel
    nullptr, // horizontal tracking wheel
    nullptr, // no second horizontal tracking wheel
    &imu // inertial sensor
);

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
// 1, 2, 4, 10, 13, 17, 20.
constexpr int INTAKE_PORT = 0;
pros::Motor intake(INTAKE_PORT);

pros::MotorGroup Toggle({6, 8});

pros::adi::DigitalOut tClaw('H'); // claw orientation piston, driven automatically
// pros::adi::DigitalOut tClaw2('C');
pros::adi::DigitalOut claw('B');  // claw open/close, driven by button A

// ============================================================
// TOGGLE ROLLER COLOR ALIGNMENT
// ============================================================
//
// A starts automatic roller alignment.
// Both half motors on ports 6 and 8 spin until the optical sensor sees
// the selected alliance color. UP and DOWN still manually spin the rollers.
//
// Change OPTICAL_PORT to the actual smart port used by your optical sensor.
constexpr int OPTICAL_PORT = 3;
pros::Optical colorSensor(OPTICAL_PORT);

enum class Alliance { RED, BLUE };
Alliance alliance = Alliance::BLUE;

constexpr double RED_HUE_MAX  = 30.0;
constexpr double RED_HUE_WRAP = 330.0;
constexpr double BLUE_HUE_MIN = 180.0;
constexpr double BLUE_HUE_MAX = 250.0;

constexpr int MIN_COLOR_PROXIMITY = 120;
constexpr int TOGGLE_ROLLER_SPEED = -200;
constexpr int COLOR_CONFIRM_MS = 50;
constexpr int COLOR_TIMEOUT_MS = 1500;

bool toggleColorActive = false;
std::uint32_t toggleColorStart = 0;
std::uint32_t targetColorSeenStart = 0;

bool hueIsRed(double hue) {
    return hue <= RED_HUE_MAX || hue >= RED_HUE_WRAP;
}

bool hueIsBlue(double hue) {
    return hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX;
}

bool seesAllianceColor() {
    int proximity = colorSensor.get_proximity();
    if (proximity < MIN_COLOR_PROXIMITY || proximity > 255) {
        return false;
    }

    double hue = colorSensor.get_hue();
    if (!std::isfinite(hue)) {
        return false;
    }

    return alliance == Alliance::RED ? hueIsRed(hue) : hueIsBlue(hue);
}

void stopToggleColorAlign() {
    toggleColorActive = false;
    targetColorSeenStart = 0;
    Toggle.move_velocity(0);
}

void startToggleColorAlign() {
    toggleColorActive = true;
    toggleColorStart = pros::millis();
    targetColorSeenStart = 0;
}

void updateToggleColorAlign() {
    if (!toggleColorActive) {
        return;
    }

    std::uint32_t now = pros::millis();

    if (now - toggleColorStart >= COLOR_TIMEOUT_MS) {
        stopToggleColorAlign();
        return;
    }

    if (seesAllianceColor()) {
        if (targetColorSeenStart == 0) {
            targetColorSeenStart = now;
        }

        if (now - targetColorSeenStart >= COLOR_CONFIRM_MS) {
            stopToggleColorAlign();
            controller.rumble(".");
            return;
        }
    } else {
        targetColorSeenStart = 0;
    }

    Toggle.move_velocity(TOGGLE_ROLLER_SPEED);
}

void runToggleToAllianceColor() {
    startToggleColorAlign();

    while (toggleColorActive) {
        updateToggleColorAlign();
        pros::delay(10);
    }
}

// ============================================================
// CLAW
// ============================================================

// --- open / close, toggled by button A ---
bool clawOn = false;
bool tclawOn = true; // starts down, so starts activated

void toggleClaw() {
    clawOn = !clawOn;
    claw.set_value(clawOn);
}

void toggleClawO() {
    tclawOn = !tclawOn;
    tClaw.set_value(tclawOn);
}

// --- orientation piston, automatic ---
//
// The piston is ON whenever the DR4B is sitting at its starting position and
// OFF once the lift is raised. There is no sensor on the lift, so this uses
// the DR4B motor encoder, which gets zeroed in initialize(). THE LIFT MUST BE
// ALL THE WAY DOWN WHEN THE PROGRAM STARTS or every reading will be off.
//
// Two thresholds instead of one so the piston does not chatter when the lift
// hovers right at the boundary. Read live lift position off line 3 of the
// brain screen to pick your numbers.
// constexpr double DR4B_DOWN_POS = 25;  // below this, the lift counts as down
// constexpr double DR4B_UP_POS   = 60;  // above this, the lift counts as up

// bool tclawOn = true; // starts down, so starts activated

// double dr4bPosition() {
//     double pos = DR4B1.get_position();
//     if (!std::isfinite(pos)) { return 0.0; } // motor unplugged
//     return pos;
// }

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

    colorSensor.set_led_pwm(100);
    colorSensor.set_integration_time(20);

    // zero the lift encoders. the DR4B must be physically all the way down
    // right now for the orientation piston logic to work.
    DR4B1.tare_position();
    DR4B2.tare_position();
    // hold, so the lift does not sag back down past the piston threshold
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // lift starts down, so the orientation piston starts activated
    tclawOn = true;
    tClaw.set_value(true);
    claw.set_value(clawOn);

    // COLOR SORT: uncomment to start the sorting task
    // pros::Task sortTask(colorSortTask);

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
            // lift position, for setting DR4B_DOWN_POS and DR4B_UP_POS
            // pros::lcd::print(3, "Lift: %.0f  Ort: %s",
            //                  dr4bPosition(),
            //                  tclawOn ? "on" : "off");

            // ANTI-TIP: uncomment these to pick the axis and check tuning
            pros::lcd::print(4, "Y/Pitch: %.1f", imu.get_pitch());
            pros::lcd::print(5, "X/Roll: %.1f  Tip: %s",
                             imu.get_roll(), antiTipActive ? "ACT" : "off");

            pros::lcd::print(6, "Hue: %.0f  Prox: %d",
                             colorSensor.get_hue(), colorSensor.get_proximity());
            pros::lcd::print(7, "Alliance: %s Roller: %s",
                             alliance == Alliance::RED ? "RED" : "BLUE",
                             toggleColorActive ? "AUTO" : "off");

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

void DR4B(float speed, int time) {
    DR4B2.move_velocity(speed);
    DR4B1.move_velocity(speed);
    pros::delay(time);
    DR4B2.move_velocity(0);
    DR4B1.move_velocity(0);
    // keep the orientation piston in step with the new lift height
    // updateClawOrientation();
}

void redLeft() {
    alliance = Alliance::RED;
}

void redRight() {
    alliance = Alliance::RED;
}

void blueLeft() {
    alliance = Alliance::BLUE;

    // set the starting position for the robot
    chassis.setPose(60.757, -2.321, 335.39);

    // moves back to toggle for roller using flex wheel mech
    chassis.moveToPose(63.483, -6.733, 0, 1000, {.forwards = false}); // moves back
    // color sort stuff with flex wheel toggle
    chassis.moveToPose(60.176, 2.705, 304.768, 1000);
    chassis.moveToPose(47.646, 8.123, 0, 1000);
    // lifts up DR4B to score preloads
    chassis.moveToPose(46.981, 18.713, 358.449, 1000);
    // score preloads/toggle claw

    chassis.moveToPose(44.728, 8.305, 41.698, 1000, {.forwards = false});// moves back
    // move DR4B down to pick up more pins
    chassis.moveToPose(56.366, 23.274, 90.777, 1000);
    chassis.moveToPose(65.356, 23.171, 89.119, 1000, {.maxSpeed = 50, });// moves slower
    // picks up pins
    chassis.moveToPose(60.473, 23.059, 91.173, 1000, {.forwards = false});// moves back
    chassis.moveToPose(50.656, 22.985, 271.853, 1000);
    // score pins

    chassis.moveToPose(58.561, 22.914, 270.113, 1000, {.forwards = false});// moves back
    chassis.moveToPose(47.105, -6.802, 113.795, 1000, {.maxSpeed = 90, .minSpeed = 50,});// moves to allign with pick up more pins
    chassis.moveToPose(57.865, -23.834, 90.346, 1000);
    chassis.moveToPose(67.075, -23.726, 87.604, 1000, {.maxSpeed = 40});// moves slow
    chassis.moveToPose(61.021, -23.978, 86.23, 1000, {.forwards = false});// moves back 
    chassis.moveToPose(56.812, -24.014, 272.699, 1000);
    // lift up DR4B to score pins
    chassis.moveToPose(51.164, -24.063, 276.657, 1000);
    // score pins
}

void blueRight() {
    alliance = Alliance::BLUE;
}

void skills() {
    // COLOR SORT: uncomment and set whichever color you run skills with
    // alliance = Alliance::RED;
}

void autonomous() {
    // redLeft();
    // redRight();
    // blueLeft();
    // blueRight();
    // does one time
    // skills();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

    // ANTI-TIP: uncomment
    bool tipWasActive = false;

    while (true) {
        // ---- drive ----
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        // move the chassis with curvature drive
        // chassis.arcade(leftY, 0.9 * rightX);

        // ANTI-TIP: to re-enable, comment out the single arcade line above
        // and uncomment everything from here down to the end of this block.
        
        // anti-tip check. if this returns true it has already overwritten
        // leftY and rightX with the correction it wants
        bool tipping = antiTip(leftY, rightX);
        
        if (tipping) {
            // brake mode holds the wheels once the robot settles back down
            if (!tipWasActive) {
                chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
                controller.rumble("."); // one short buzz so the driver knows
            }
            // drive straight, no turning, while recovering
            chassis.arcade(leftY, rightX);
        } else {
            if (tipWasActive) {
                chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
            }
            chassis.arcade(leftY, 0.9 * rightX);
        }
        
        tipWasActive = tipping;

        // ---- L1 / L2: DR4B ----
        // ANTI-TIP: when re-enabling, change the L1 line to
        //   if (controller.get_digital(...L1) && !tipping)
        // so the lift cannot be raised mid-tip, which makes tipping worse.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !tipping) {
            DR4B1.move_velocity(200);
            DR4B2.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            DR4B1.move_velocity(-200);
            DR4B2.move_velocity(-200);
        } else {
            DR4B1.move_velocity(0);
            DR4B2.move_velocity(0);
        }

        // ---- R1 / R2: intake ----
        // COLOR SORT: with sorting off, the intake is driven straight from
        // here. When re-enabling, swap these three intake.move() calls back
        // to intakeCommand = 127 / -127 / 0, because the sorting task then
        // owns the motor and the two would fight over it.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(-200);
        } else {
            intake.move_velocity(0);
        }

        // A starts automatic toggle roller color alignment.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            startToggleColorAlign();
        }

        // X now opens/closes the claw because A is used for the roller.
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            toggleClaw();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleClawO();
        }

        // Manual roller control cancels automatic color alignment.
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            toggleColorActive = false;
            Toggle.move_velocity(TOGGLE_ROLLER_SPEED);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            toggleColorActive = false;
            Toggle.move_velocity(-TOGGLE_ROLLER_SPEED);
        } else if (toggleColorActive) {
            updateToggleColorAlign();
        } else {
            Toggle.move_velocity(0);
        }

        // ---- orientation piston follows the lift, no button needed ----
        // updateClawOrientation();

        // delay to save resources
        pros::delay(10);
    }
}