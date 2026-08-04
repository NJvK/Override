#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <iterator>
#include <cmath>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, 17}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9, 15}, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(1);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
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
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            6, // derivative gain (kD)
                                            0, // anti windup
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
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
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

pros::Motor cascade(13);
pros::Motor cascade2(-4);

// pros::ADIDigitalOut descore('B');
// pros::ADIDigitalOut matchload('A');
pros::ADIDigitalOut claw('A');

// pros::Distance distancer(0);
// pros::Distance distanceb(0);
// pros::Distance distancef(0);
// pros::Distance distancel(0);

bool claw_state = false;

void toggle_claw() {
    claw_state = !claw_state;
    claw.set_value(claw_state);
}

// DISTANCE CONTROL:

const double FIELD_WIDTH = 144.0;
const double FIELD_HEIGHT = 144.0;

// to be edited
const double FRONT_OFFSET = 4.5;
const double BACK_OFFSET = 5.0;
const double LEFT_OFFSET = 6;
const double RIGHT_OFFSET = 6;

double mmToIn(double mm) { return mm / 25.4; }

double safeRead(pros::Distance& sensor) {
    int raw = sensor.get();

    // Reject invalid mm readings
    if (raw <= 0 || raw > 2000) return -1;

    double inches = mmToIn(raw);

    // Reject readings under 8 inches
    if (inches < 1) return -1;

    return inches;
}

void resetcoord(int quadrant, int angle) {
    double front = safeRead(distancef) + FRONT_OFFSET;
    double back = safeRead(distanceb) + BACK_OFFSET;
    double left = safeRead(distancel) + LEFT_OFFSET;
    double right = safeRead(distancer) + RIGHT_OFFSET;

    // Default to current pose if a reading is invalid
    lemlib::Pose current = chassis.getPose();
    double xPos = current.x;
    double yPos = current.y;

    double HALF_FIELD = 72;

    bool red = false;
    bool blue = false;
    bool leftd = false;
    bool rightd = false;

    switch (angle) {
        case 0: blue = true; break;

        case 90: rightd = true; break;
        case 180: red = true; break;
        case 270: leftd = true; break;
        default: break;
    }

    // QUAD 1

    if (quadrant == 1) {
        if (red) {
            xPos = (HALF_FIELD - left);
            yPos = (HALF_FIELD - back);

        } else if (blue) {
            xPos = (HALF_FIELD - right);
            yPos = (HALF_FIELD - front);
        } else if (rightd) {
            xPos = (HALF_FIELD - front);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd) {
            xPos = (HALF_FIELD - back);
            yPos = (HALF_FIELD - right);
        }
    }

    // QUAD 2

    if (quadrant == 2) {
        if (red) {
            xPos = (HALF_FIELD - right);
            yPos = (HALF_FIELD - back);
        } else if (blue) {
            xPos = -(HALF_FIELD - left);
            yPos = (HALF_FIELD - front);
        } else if (rightd) {
            xPos = -(HALF_FIELD - back);
            yPos = (HALF_FIELD - left);
        }

        else if (leftd) {
            xPos = -(HALF_FIELD - front);
            yPos = (HALF_FIELD - right);
        }
    }

    // QUAD 3

    if (quadrant == 3) {
        if (red) {
            xPos = -(HALF_FIELD - right);
            yPos = -(HALF_FIELD - front);
        } else if (blue) {
            xPos = -(HALF_FIELD - left);
            yPos = -(HALF_FIELD - back);
        } else if (rightd) {
            xPos = -(HALF_FIELD - back);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd) {
            xPos = -(HALF_FIELD - front);
            yPos = -(HALF_FIELD - left);
        }
    }

    // QUAD 4

    if (quadrant == 4) {
        if (red) {
            xPos = (HALF_FIELD - left);
            yPos = -(HALF_FIELD - front);

        } else if (blue) {
            xPos = (HALF_FIELD - right);
            yPos = -(HALF_FIELD - back);
        } else if (rightd) {
            xPos = (HALF_FIELD - front);
            yPos = -(HALF_FIELD - right);
        }

        else if (leftd) {
            xPos = (HALF_FIELD - back);
            yPos = -(HALF_FIELD - left);
        }
    }

    chassis.setPose(xPos, yPos, chassis.getPose().theta);
}

// direction: "x" or "y"
// distance: positive or negative inches
// timeout: ms
// settings: lemlib::MoveToPointSettings (same struct used in moveToPoint)

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    imu.tare_rotation();
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
            pros::lcd::print(2, "Angle: %f", chassis.getPose().theta); // heading
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


void autonomous() {

}

/**
 * Runs in driver control
 */

void opcontrol() {
    // controller
    // loop to continuously update motors

    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the chassis with curvature drive
        chassis.arcade(leftY, 0.9 * rightX);

        // buttons for controller

        // Control Intake using shoulder buttons (L1/L2)

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade.move_velocity(200);
            cascade2.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            toggle_claw();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            cascade.move_velocity(-200);
            cascade2.move_velocity(-200);
        } else {
            cascade.move_velocity(0);
            cascade2.move_velocity(0);
        }

        // delay to save resources
        pros::delay(10);
    }
}
