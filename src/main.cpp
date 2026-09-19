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
constexpr TipAxis TIP_AXIS     = TipAxis::X_ROLL; // y-axis, nose up / nose down //x-axis since inertial is sideways
constexpr double TIP_ANGLE_ON  = 7.0;   // degrees of lean before taking over
constexpr double TIP_ANGLE_OFF = 3.0;    // degrees of lean before giving control back
constexpr double TIP_KP        = 6.0;    // motor power per degree past the threshold
constexpr double TIP_MAX_POWER = 100.0;  // cap on correction power, out of 127
constexpr bool   TIP_INVERT    = true;  // flip if the robot pushes the wrong way

bool antiTipActive = false;

// Returns the lean angle on whichever axis is configured above.
// Positive is treated as "nose up" (falling backward).
double tipAngle() {
    double angle = (TIP_AXIS == TipAxis::Y_PITCH) ? imu.get_pitch() : imu.get_roll();
    // the IMU returns infinity while calibrating or if the port is unplugged
    if (!std::isfinite(angle)) { return 0.0; }
    return TIP_INVERT ? -angle : angle;
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
// COLOR SORT  [ DISABLED - uncomment this whole block to re-enable ]
// ============================================================
//
// The optical sensor watches what goes through the intake. If it sees a
// piece belonging to the other alliance, it waits a moment for that piece to
// reach the eject point, then reverses the intake to throw it back out.
//
// OTHER PLACES TO UNCOMMENT when turning this back on:
//   - the sortTask line in initialize()
//   - the hue and alliance screen lines in initialize()
//   - the B button and DOWN button blocks in opcontrol()
//   - the alliance lines inside the autonomous routines
//   - AND the R1/R2 intake block in opcontrol() has to go back to setting
//     intakeCommand instead of calling intake.move() directly, or the loop
//     and the task will fight over the intake motor
//
// constexpr int OPTICAL_PORT = 0; // needs a real port, 1 to 21
// pros::Optical colorSensor(OPTICAL_PORT);
//
// enum class Alliance { RED, BLUE };
// Alliance alliance = Alliance::RED; // <-- SET YOUR ALLIANCE HERE
//
// // Hue is a 0-360 color wheel. Red sits at both ends of it, which is why it
// // needs two checks. Watch the hue readout with a real game piece in front
// // of the sensor and widen these if your readings do not match.
// constexpr double RED_HUE_MAX    = 25;   // 0 up to here counts as red
// constexpr double RED_HUE_WRAP   = 340;  // and this up to 360 also counts as red
// constexpr double BLUE_HUE_MIN   = 190;
// constexpr double BLUE_HUE_MAX   = 240;
// constexpr int    MIN_PROXIMITY  = 150;  // 0-255, ignore anything not right up close
// constexpr int    MAX_PROXIMITY  = 255;  // real readings never exceed this
// constexpr int    EJECT_DELAY_MS = 60;   // travel time from sensor to eject point
// constexpr int    EJECT_TIME_MS  = 250;  // how long to run the intake backward
// constexpr int    EJECT_SPEED    = -127; // use 0 instead if you want it to just stop
//
// bool sortingEnabled = true;
// int intakeCommand = 0;  // what the driver wants the intake to do, -127 to 127
// bool ejecting = false;  // true while a piece is being spat back out
//
// bool hueIsRed(double hue) { return hue <= RED_HUE_MAX || hue >= RED_HUE_WRAP; }
//
// bool hueIsBlue(double hue) { return hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX; }
//
// // True if the thing at the sensor belongs to the other alliance.
// bool isOpposingPiece() {
//     int prox = colorSensor.get_proximity();
//     // an unplugged sensor returns PROS_ERR, a huge number that would sail
//     // past the minimum check and look like a piece jammed against the lens
//     if (prox < MIN_PROXIMITY || prox > MAX_PROXIMITY) { return false; }
//     double hue = colorSensor.get_hue();
//     if (!std::isfinite(hue)) { return false; }
//     return (alliance == Alliance::RED) ? hueIsBlue(hue) : hueIsRed(hue);
// }
//
// // Runs in the background so the ejection wait does not freeze driver
// // control. This task is the only thing that talks to the intake motor.
// // Everything else just sets intakeCommand.
// void colorSortTask() {
//     colorSensor.set_led_pwm(100);         // sensor needs its own light to read color
//     colorSensor.set_integration_time(20); // faster sampling for moving pieces
//
//     while (true) {
//         // only sort while the intake is actually pulling something in
//         if (sortingEnabled && intakeCommand > 0 && isOpposingPiece()) {
//             ejecting = true;
//             pros::delay(EJECT_DELAY_MS); // let the piece reach the eject point
//             intake.move(EJECT_SPEED);
//             pros::delay(EJECT_TIME_MS);
//             ejecting = false;
//         } else {
//             intake.move(intakeCommand);
//         }
//         pros::delay(10);
//     }
// }

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

            // COLOR SORT: uncomment these to check hue ranges and alliance
            // pros::lcd::print(6, "Hue: %.0f  Prox: %d",
            //                  colorSensor.get_hue(), colorSensor.get_proximity());
            // pros::lcd::print(7, "Alliance: %s  Sort: %s",
            //                  alliance == Alliance::RED ? "RED" : "BLUE",
            //                  sortingEnabled ? "on" : "OFF");

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
    // COLOR SORT: uncomment
    // alliance = Alliance::RED;
}

void redRight() {
    // COLOR SORT: uncomment
    // alliance = Alliance::RED;
}

void blueLeft() {
    // COLOR SORT: uncomment
    // alliance = Alliance::BLUE;
}

void blueRight() {
    // COLOR SORT: uncomment
    // alliance = Alliance::BLUE;
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

        // ---- A: claw open / closed ----
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            toggleClaw();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            toggleClawO();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            Toggle.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            Toggle.move_velocity(-200);
        } else {
            Toggle.move_velocity(0);
        
        }

        // COLOR SORT: uncomment for the alliance color toggle
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        //     alliance = (alliance == Alliance::RED) ? Alliance::BLUE : Alliance::RED;
        //     controller.rumble(alliance == Alliance::RED ? "-" : "--");
        // }

        // COLOR SORT: uncomment to toggle sorting on and off
        // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //     sortingEnabled = !sortingEnabled;
        // }

        // ---- orientation piston follows the lift, no button needed ----
        // updateClawOrientation();

        // delay to save resources
        pros::delay(10);
    }
}