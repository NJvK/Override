#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iterator>
#include <cmath>
#include "pros/screen.hpp"
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// motor groups
pros::MotorGroup leftMotors({-1, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({17, 10}, pros::MotorGearset::blue);
// Inertial Sensor on port 18
pros::Imu imu(18);
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 8, not reversed
pros::Rotation horizontalEnc(8);
// vertical tracking wheel encoder. Rotation sensor, port 15, reversed
pros::Rotation verticalEnc(-15);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);
const double PI = 3.14159265358979323846;
int deadband(int value) {
    if (abs(value) < 5) {
        return 0;
    }
    return value;
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
lemlib::OdomSensors sensors(
    &vertical,      // vertical tracking wheel
    nullptr,        // no second vertical tracking wheel
    &horizontal,    // horizontal tracking wheel
    nullptr,        // no second horizontal tracking wheel
    &imu            // inertial sensor
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
pros::Motor cascade1(20);
pros::Motor cascade2(-13);
pros::adi::DigitalOut claw('A');

bool claw_state = false;
void toggle_claw() {
    claw_state = !claw_state;
    claw.set_value(claw_state);
    pros::delay(300); // Add a small delay to prevent rapid toggling
}
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
void cascade(float speed, int time) {
    cascade1.move_velocity(speed);
    cascade2.move_velocity(speed);
    pros::delay(time);
    cascade1.move_velocity(0);
    cascade2.move_velocity(0);
}
void redLeft() {
    // chassis.setPose(0, 0, 0);
    // pros::delay(50);
    // cascade(200, 70);
    // pros::delay(100);
    // toggle_claw();
    // pros::delay(50);
    // cascade(200, 300);
    // chassis.moveToPoint(0, 2.5, 1000, {.minSpeed = 30});
    // chassis.turnToHeading(-75, 1000, {.earlyExitRange = 1});
    // // moves to goal
    // chassis.moveToPoint(-17, 5.4, 1000, {.minSpeed = 30});
    // pros::delay(800);
    // cascade(-200, 130);
    // toggle_claw();
    // chassis.turnToHeading(-55, 500);
    // // // scores preload pin
    // chassis.moveToPoint(-4.8, 14.7, 1000);
    // chassis.turnToHeading(-90, 1000);
    // chassis.moveToPoint(-45, 17, 1000);
    // chassis.turnToHeading(0, 1000);
    // chassis.moveToPoint(-45, -2, 1000,  {.forwards = false});

    chassis.setPose(0, 0, 0);
    pros::delay(50);
    pros::delay(50);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});
    pros::delay(50);
    chassis.moveToPoint(0, 5, 1000);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});

    // chassis.moveToPoint(0, 10, 1000);
    // chassis.turnToHeading(55, 700);
}
void redRight() {
    // toggle
    chassis.setPose(0, 0, 0);
    pros::delay(50);
    toggle_claw();
    pros::delay(50);
    chassis.moveToPoint(0, -3, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});
    pros::delay(50);
    // chassis.moveToPoint(0, 5, 1000);
    // chassis.moveToPoint(0, -3, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 1});
}
void skills(){
    chassis.setPose(0, 0, 0);
    pros::delay(50);
    chassis.moveToPoint(0, 72, 3000, {.maxSpeed = 80, .minSpeed = 50, .earlyExitRange = 5});
}

void autonomous() {
    // redLeft();
    // redRight(); // does one time
    skills();
}
/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    while (true) {
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.setPose(0, 0, 0);
        
            chassis.moveToPoint(0, 6, 3000);
            chassis.waitUntilDone();
        }
        // get joystick positions
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        // move the chassis with curvature drive
        chassis.arcade(leftY, 0.9 * rightX);
        // buttons for controller
        // Control Intake using shoulder buttons (L1/L2)
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade1.move_velocity(200);
            cascade2.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            toggle_claw();

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            toggle_claw();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            cascade1.move_velocity(-200);
            cascade2.move_velocity(-200);
        } else {
            cascade1.move_velocity(0);
            cascade2.move_velocity(0);
        }
        // delay to save resources
        pros::delay(10);
    }
}
