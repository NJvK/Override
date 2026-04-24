#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <iterator>

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, 12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(1);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

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

pros::Motor intakeb(-19);
pros::Motor intakef(20);

pros::ADIDigitalOut descore('B');
pros::ADIDigitalOut matchload('A');

pros::Distance distancer(9);
pros::Distance distanceb(10);
pros::Distance distancef(7);
pros::Distance distancel(4);

bool descore_state = false;
bool matchload_state = false;

void set_both(int32_t voltage) {
    intakef.move_voltage(voltage);
    intakeb.move_voltage(voltage);
}

void set_intakeb(int32_t voltage) { intakeb.move_voltage(voltage); }

void set_intakef(int32_t voltage) { intakef.move_voltage(voltage); }

void toggle_matchload() {
    matchload_state = !matchload_state;
    matchload.set_value(matchload_state);
}

void toggle_descore() {
    descore_state = !descore_state;
    descore.set_value(descore_state);
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

void right_quals() {
    chassis.setPose(0, 0, 0);
    resetcoord(4, 0);
    toggle_descore();

    chassis.moveToPoint(20, -34, 1000, {.minSpeed = 30, .earlyExitRange = 3});
    set_intakef(12000);

    chassis.moveToPoint(38, -21, 2000, {.maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(21);
    chassis.turnToHeading(80, 500, {.minSpeed = 10, .earlyExitRange = 5});
    chassis.moveToPoint(21, -41, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    chassis.turnToHeading(125, 1000, {.minSpeed = 10, .earlyExitRange = 5});
    chassis.moveToPoint(42, -48, 1000, {.minSpeed = 10, .earlyExitRange = 1});
    set_intakef(0);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    resetcoord(4, 180);
    pros::delay(50);
    chassis.moveToPoint(48, -31.5, 1000, {.forwards = false});
    chassis.waitUntil(15);
    set_both(12000);
    pros::delay(1300);
    toggle_matchload();

    chassis.moveToPoint(48, -63, 1850, {.maxSpeed = 45});
    set_intakeb(0);
    chassis.moveToPoint(48, -50, 1000, {.forwards = false});
    toggle_matchload();
    chassis.turnToHeading(315, 1000);
    chassis.moveToPoint(21, -21.5, 2000, {.maxSpeed = 80});
    intakeb.move_absolute((intakeb.get_position() + 250), 12000);
    chassis.waitUntil(22);
    set_both(-12000);
    pros::delay(1200);
    set_both(-10000);
    pros::delay(1400);
    chassis.moveToPoint(34, -31, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntil(50);
    toggle_descore();
    chassis.waitUntilDone();
}

void solo_awp_right() {
    chassis.setPose(0, 0, 90);
    resetcoord(4, 90);
    pros::delay(50);

    chassis.moveToPoint(40.5, -50, 1000, {.minSpeed = 1, .earlyExitRange = 3});

    chassis.turnToHeading(180, 1000, {.minSpeed = 1, .earlyExitRange = 10});
    toggle_matchload();
    // toggle amtchlod

    chassis.moveToPoint(42, -61.5, 1100, {.maxSpeed = 50, .minSpeed = 20});
    set_intakef(12000);
    chassis.waitUntilDone();
    chassis.moveToPoint(42, -29, 1800, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntil(22);
    set_intakeb(12000);
    // score
    pros::delay(400);

    chassis.moveToPose(42, -37, 180, 1000, {.maxSpeed = 70, .minSpeed = 30});

    toggle_matchload();

    chassis.waitUntilDone();
    resetcoord(4, 180);
    pros::delay(10);
    chassis.turnToHeading(300, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    set_both(0);
    chassis.moveToPoint(29.5, -27, 1500, {.minSpeed = 20, .earlyExitRange = 2});
    set_intakef(12000);
    chassis.waitUntil(14);
    toggle_matchload();
    chassis.turnToHeading(270, 1000, {.minSpeed = 30, .earlyExitRange = 5});

    chassis.moveToPoint(-8, -27, 1500, {.maxSpeed = 100, .minSpeed = 30});
    chassis.waitUntil(1);
    toggle_matchload();
    chassis.waitUntil(29);
    toggle_matchload();
    chassis.waitUntilDone();
    pros::delay(50);

    chassis.turnToHeading(225, 1000, {.minSpeed = 25, .earlyExitRange = 5});
    chassis.waitUntil(10);
    set_intakef(0);

    chassis.moveToPose(9, -11, 225, 2000, {.forwards = false, .maxSpeed = 100, .minSpeed = 70});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntil(15);
    set_both(-12000);
    pros::delay(80);
    set_intakeb(0);
    set_intakef(12000);
    pros::delay(900);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    chassis.moveToPoint(-26, -46, 2000, {.minSpeed = 30, .earlyExitRange = 2});
    chassis.turnToHeading(180, 600);
    chassis.waitUntilDone();
    resetcoord(3, 180);

    chassis.moveToPoint(-48, -63, 1150, {.maxSpeed = 50, .minSpeed = 20});
    pros::delay(400);
    chassis.moveToPoint(-47.5, -28, 1000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(18);
    set_both(12000);
}

void ball7_left() {
    chassis.setPose(-14, -50, 0);
    pros::delay(50);
    //-15, -50
    chassis.moveToPose(-19, -29, -33.0, 1300, {.lead = 0.13, .maxSpeed = 80, .minSpeed = 30});
    set_intakef(12000);
    chassis.waitUntil(15);
    toggle_matchload();
    exit_condition({-21, -29, -33}, 3);

    // -39, -40
    chassis.moveToPose(-43, -57, 180, 1500, {.lead = .3, .maxSpeed = 110, .minSpeed = 30});
    exit_condition({-40.5, -57, 130}, 1);
    chassis.moveToPose(-39.5, -80, 180.0, 1600, {.lead = 0.1, .maxSpeed = 55, .minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPose(-38.5, -40, 180.0, 1500, {.forwards = false, .lead = 0.1, .maxSpeed = 50, .minSpeed = 30});
    chassis.waitUntil(16);
    // set_both(12000);
    // pros::delay(2000);
    // chassis.waitUntilDone();
    // chassis.moveToPose(chassis.getPose().x - 11, chassis.getPose().y - 9.5, 200, 1000,
    //                    {.lead = .1, .maxSpeed = 80, .minSpeed = 1, .earlyExitRange = 2});
    // toggle_matchload();
    // toggle_descore();
    // chassis.moveToPose(-42, -37, 180, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange =
    // 2}); chassis.waitUntil(3); toggle_descore(); chassis.waitUntilDone(); chassis.turnToHeading(-230, 1000,
    // {.maxSpeed = 20}); chassis.waitUntil(30); toggle_matchload(); chassis.waitUntilDone();
    // chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void ball7_wing_right() {
    chassis.moveToPoint(5, 16, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    set_intakef(12000);

    chassis.moveToPoint(24, 32, 2000, {.maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(21);
    toggle_matchload();

    chassis.moveToPoint(7, 15, 1000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    chassis.turnToHeading(125, 1000);

    chassis.moveToPoint(25, -2, 1000);
    chassis.turnToHeading(180, 1000);

    chassis.moveToPoint(25.5, 15, 1500, {.forwards = false});
    chassis.waitUntil(15);
    set_both(12000);
    chassis.waitUntilDone();
    pros::delay(1000);

    chassis.moveToPoint(25.5, -13, 1500);
    set_intakeb(0);
    chassis.waitUntilDone();
    pros::delay(1000);

    chassis.moveToPoint(25.5, 15, 1000, {.forwards = false});
    chassis.waitUntil(20);
    set_both(12000);
    pros::delay(1000);

    chassis.moveToPoint(25.5, 4, 1000);
    chassis.turnToHeading(145, 1000);

    chassis.moveToPoint(17, 15, 1000, {.forwards = false});
    chassis.turnToHeading(180, 1000);

    chassis.moveToPoint(16, 33, 1000, {.forwards = false, .maxSpeed = 100});
    chassis.turnToHeading(145, 1000);
}

void ball7_right() {
    resetcoord(4, 0);
    //-15, -50
    chassis.moveToPose(19.5, -33, 33.0, 1300, {.lead = 0.13, .maxSpeed = 100, .minSpeed = 40});
    set_intakef(12000);
    chassis.waitUntil(14);
    toggle_matchload();
    exit_condition({22.5, -35, -33}, 3);

    chassis.moveToPose(39.5, -56, 180, 1400, {.lead = .3, .maxSpeed = 110, .minSpeed = 60});
    exit_condition({40.5, -56, 130}, 1);
    chassis.moveToPose(38, -80, 180.0, 1350, {.lead = 0.1, .maxSpeed = 50, .minSpeed = 50});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000, {.maxSpeed = 60, .minSpeed = 60, .earlyExitRange = 1});
    chassis.waitUntilDone();
    resetcoord(4, 180);
    pros::delay(50);
    chassis.moveToPose(46.8, -26, 180.0, 1800, {.forwards = false, .lead = 0.1, .maxSpeed = 80, .minSpeed = 60});
    chassis.waitUntil(18);
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    set_both(12000);
    pros::delay(1800);
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    chassis.moveToPose(chassis.getPose().x - 10.5, chassis.getPose().y - 10, 220, 1000,
                       {.lead = .1, .maxSpeed = 120, .minSpeed = 30, .earlyExitRange = 2});
    toggle_matchload();
    chassis.moveToPose(43.7, -22, 180, 2000, {.forwards = false, .maxSpeed = 90, .minSpeed = 80, .earlyExitRange = 1});
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    chassis.waitUntilDone();
    chassis.turnToHeading(-230, 1500, {.maxSpeed = 20});
    chassis.waitUntil(30);
    toggle_matchload();
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void left_quals() {
    chassis.setPose(-14, -50, 0);

    chassis.moveToPoint(-19, -34, 1000, {.minSpeed = 20, .earlyExitRange = 3});
    set_intakef(12000);

    chassis.moveToPoint(-38, -17, 2000, {.maxSpeed = 70, .minSpeed = 10, .earlyExitRange = 2});
    chassis.waitUntil(21);
    chassis.turnToHeading(-80, 500, {.minSpeed = 10, .earlyExitRange = 5});

    chassis.moveToPoint(-21, -29, 1000, {.forwards = false, .maxSpeed = 90});
    chassis.turnToHeading(-130, 1000, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.moveToPoint(-9, -18, 2500, {.forwards = false, .maxSpeed = 80, .minSpeed = 70});
    chassis.waitUntil(13);
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    set_both(-12000);
    pros::delay(200);
    set_intakef(12000);
    set_intakeb(0);
    pros::delay(700);
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);

    chassis.moveToPoint(-42.7, -49, 2000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.turnToHeading(-180, 1000, {.minSpeed = 10, .earlyExitRange = 5});
    toggle_matchload();
    chassis.waitUntilDone();

    chassis.moveToPoint(-42, -63, 1100);
    chassis.waitUntilDone();
    pros::delay(10);
    chassis.moveToPoint(-42, -30, 1000, {.forwards = false});
    chassis.waitUntil(20);
    set_both(12000);
    chassis.waitUntilDone();
    pros::delay(500);
    set_intakeb(12000);
    pros::delay(100);
    set_both(12000);
    pros::delay(1300);
    // chassis.moveToPose(chassis.getPose().x - 10, chassis.getPose().y - 15, 200, 1000,
    //                    {.lead = .2, .maxSpeed = 80, .minSpeed = 1, .earlyExitRange = 2});
    // robot.toggle_matchload();
    // chassis.moveToPose(-51, -23, 180, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange =
    // 2}); chassis.waitUntilDone(); chassis.turnToHeading(-230, 1000, {.maxSpeed = 20}); chassis.waitUntil(30);
    // robot.toggle_matchload();
    // chassis.waitUntilDone();
    // chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void ball4_left() {
    chassis.setPose(0, 0, 0);
    resetcoord(3, 0);
    pros::delay(50);
    chassis.moveToPose(-21, -31, -33.0, 1300, {.lead = 0.13, .maxSpeed = 80, .minSpeed = 80});
    set_intakef(12000);
    chassis.waitUntil(15);
    toggle_matchload();
    exit_condition({-23, -31, -33}, 3);

    chassis.moveToPose(-39.5, -61, 180, 1500, {.lead = .3, .maxSpeed = 110, .minSpeed = 60});
    exit_condition({-39.5, -60, -130}, 1);
    chassis.moveToPose(-39.5, -39, 180.0, 1150, {.forwards = false, .lead = 0.1, .maxSpeed = 70, .minSpeed = 50});
    chassis.waitUntil(9);
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
    set_both(12000), pros::delay(900);
    // chassis.waitUntilDone();
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
    chassis.moveToPose(chassis.getPose().x - 10, chassis.getPose().y - 13.5, 200, 1000,
                       {.lead = .1, .maxSpeed = 100, .minSpeed = 1, .earlyExitRange = 2});
    toggle_matchload();
    chassis.moveToPose(-38, -45, 180, 2000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 2});
    chassis.waitUntilDone();
    chassis.turnToHeading(-230, 1000, {.maxSpeed = 20});
    chassis.waitUntil(30);
    toggle_matchload();
    chassis.waitUntilDone();
    chassis.setBrakeMode(pros::motor_brake_mode_e::E_MOTOR_BRAKE_BRAKE);
}

void skills_auto() {
    chassis.setPose(0, 0, 0);
    resetcoord(3, 0);
    toggle_descore();
    pros::delay(100);
    chassis.moveToPoint(-17.3, -37, 1000, {.maxSpeed = 90, .earlyExitRange = 2});
    chassis.waitUntil(5);
    set_intakef(12000);
    chassis.moveToPoint(-23.5, -30, 1500, {.maxSpeed = 70});
    chassis.waitUntil(5);
    set_intakef(0);
    chassis.turnToHeading(-135, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPose(-5.5, -15.5, -135, 4000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.waitUntil(13);
    set_intakef(10000);
    set_intakeb(0);
    pros::delay(1700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-43, -53, 2000, {.maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 1});
    set_intakeb(12000);
    chassis.turnToHeading(180, 500);
    toggle_matchload();
    chassis.waitUntilDone();
    resetcoord(3, 180);
    chassis.moveToPoint(-46.5, -65, 2600, {.maxSpeed = 35});
    set_intakef(12000);
    chassis.waitUntil(4);
    set_intakeb(0);
    pros::delay(600);
    chassis.moveToPose(-47, -50, 180, 1000, {.forwards = false, .maxSpeed = 60});
    pros::delay(100);
    chassis.moveToPoint(-56, -39.2, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntil(10);
    toggle_matchload();
    chassis.turnToHeading(180, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    set_both(0);
    chassis.moveToPoint(-55, 15, 3500, {.forwards = false, .maxSpeed = 110, .minSpeed = 30, .earlyExitRange = 3});
    // chassis.turnToHeading(-90, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    // // for all the 45.5 you could change that to be 44.5. it was originally 43.5
    chassis.moveToPoint(-41, 25.5, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1});
    chassis.turnToHeading(0, 1000);
    chassis.waitUntilDone();
    resetcoord(2, 0);
    pros::delay(50);


    chassis.moveToPoint(-47, 20, 1700, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(6);
    set_both(12000);
    pros::delay(2500);
    chassis.moveToPoint(-47, 65, 3000, {.maxSpeed = 35});
    toggle_matchload();
    set_intakeb(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 20, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntil(21);
    set_both(12000);
    toggle_matchload();
    pros::delay(1000);
    set_intakeb(12000);
    pros::delay(500);
    chassis.moveToPose(-47, 36, 0, 1000, {.maxSpeed = 100});
    chassis.turnToHeading(0, 300);
    chassis.waitUntilDone();
    resetcoord(2, 0);
    pros::delay(50);
    // //-47, 41

    chassis.moveToPose(-18, 60.5, 90, 1500, {.lead = .33, .maxSpeed = 100});
    chassis.waitUntilDone();
    toggle_matchload();
    chassis.moveToPose(28, 60, 90, 4000, {.maxSpeed = 100});
    chassis.waitUntil(13);
    toggle_matchload();
    set_intakeb(0);
    set_intakef(12000);
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 1500);
    chassis.waitUntilDone();
    pros::delay(50);
    resetcoord(1, 180);
    pros::delay(50);

    chassis.moveToPoint(22, 37.5, 1000);
    chassis.moveToPoint(21, 24.5, 1000);
    chassis.waitUntil(6);
    set_intakef(0);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPose(10, 13.5, 45, 1300, {.forwards = false, .maxSpeed = 75, .minSpeed = 50});
    chassis.waitUntil(10);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    set_both(-12000);
    pros::delay(250);
    set_intakef(12000);
    set_intakeb(0);
    pros::delay(2000);
    set_intakef(8000);
    pros::delay(1200);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(15, 18.2, 1000, {.maxSpeed = 30, .minSpeed = 10});
    chassis.moveToPoint(47, 50, 2500, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3});
    set_both(12000);
    chassis.turnToHeading(0, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    toggle_matchload();
    chassis.moveToPoint(45.5, 67, 3000, {.maxSpeed = 60});
    set_intakef(12000);
    set_intakeb(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(45.5, 55, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(55, 44, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntil(10);
    toggle_matchload();
    chassis.turnToHeading(0, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    set_both(0);
    chassis.moveToPoint(51.5, -10, 4000, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 1});
    chassis.moveToPoint(41, -25, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1});
    chassis.turnToHeading(180, 700);
    chassis.waitUntilDone();
    pros::delay(50);
    resetcoord(4, 180);
    pros::delay(50);

    // chassis.moveToPoint(47, -36, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1});
    // chassis.turnToHeading(180, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(48.5, -20, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(6);
    set_both(12000);
    pros::delay(2000);

    pros::delay(50);
    toggle_matchload();
    set_intakeb(0);
    chassis.moveToPoint(48.5, -65, 3000, {.maxSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(48.5, -20, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(20);
    set_both(12000);
    pros::delay(2800);
    toggle_matchload();
    chassis.moveToPose(48.5, -37, 180, 1000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    resetcoord(4, 180);
    chassis.moveToPose(22, -60, 270, 2000, {.lead = 0.3, .maxSpeed = 120});
    chassis.waitUntilDone();
    toggle_matchload();
    pros::delay(30);
    chassis.moveToPose(-2, -60, 270, 6000, {.maxSpeed = 100, .minSpeed = 100});
    chassis.waitUntil(13);
    toggle_matchload();
}
void skills_auto1() {
    chassis.setPose(0, 0, 0);
    resetcoord(3, 0);
    toggle_descore();
    pros::delay(100);
    chassis.moveToPoint(-17.3, -37, 1000, {.maxSpeed = 90, .earlyExitRange = 2});
    chassis.waitUntil(5);
    set_intakef(12000);
    chassis.moveToPoint(-23.5, -30, 1500, {.maxSpeed = 70});
    chassis.waitUntil(5);
    set_intakef(0);
    chassis.turnToHeading(-135, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPose(-5.5, -15.5, -135, 4000, {.forwards = false, .maxSpeed = 70, .minSpeed = 50});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.waitUntil(13);
    set_intakef(10000);
    set_intakeb(0);
    pros::delay(1700);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(-43, -53, 2000, {.maxSpeed = 100, .minSpeed = 10, .earlyExitRange = 1});
    set_intakeb(12000);
    chassis.turnToHeading(180, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    toggle_matchload();
    chassis.moveToPoint(-42.5, -64, 2600, {.maxSpeed = 60});
    set_intakef(12000);
    chassis.waitUntil(4);
    set_intakeb(0);
    pros::delay(600);
    chassis.moveToPose(-42.5, -50, 180, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(100);
    resetcoord(3, 180);
    pros::delay(100);
    chassis.moveToPoint(-56, -39.2, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntil(10);
    toggle_matchload();
    chassis.turnToHeading(180, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    set_both(0);
    chassis.moveToPoint(-55, 23, 3500, {.forwards = false, .maxSpeed = 110, .minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(-90, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(-43.5, 20, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1});
    chassis.turnToHeading(0, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(-43.5, 1.5, 1700, {.forwards = false, .maxSpeed = 110});
    chassis.waitUntil(6);
    set_both(12000);
    pros::delay(2500);
    chassis.moveToPoint(-43.3, 40, 3000, {.maxSpeed = 50});
    toggle_matchload();
    set_intakeb(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(-43.5, 5, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntil(21);
    set_both(12000);
    toggle_matchload();
    pros::delay(1500);
    set_intakeb(12000);
    pros::delay(500);
    chassis.moveToPose(-43.5, 22, 0, 1000, {.maxSpeed = 100});
    chassis.turnToHeading(0, 300);
    chassis.waitUntilDone();
    resetcoord(2, 0);
    pros::delay(50);
    // //-47, 41

    chassis.moveToPose(-18, 60.5, 90, 1500, {.lead = .33, .maxSpeed = 100});
    chassis.waitUntilDone();
    toggle_matchload();
    chassis.moveToPose(26, 60.5, 90, 4000, {.maxSpeed = 100});
    chassis.waitUntil(13);
    toggle_matchload();
    set_intakeb(0);
    set_intakef(12000);
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 1500);
    chassis.waitUntilDone();
    pros::delay(50);
    resetcoord(1, 180);
    pros::delay(50);
    // 26, 60
    chassis.moveToPoint(21.5, 37.5, 1000);
    chassis.moveToPoint(21, 25.5, 1000);
    chassis.waitUntil(6);
    set_intakef(0);
    chassis.turnToHeading(45, 1000);
    chassis.moveToPose(7, 16, 45, 1500,  {.forwards = false, .maxSpeed = 75, .minSpeed = 50});
    chassis.waitUntil(10);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    set_both(-12000);
    pros::delay(150);
    set_intakef(9000);
    set_intakeb(0);
    pros::delay(3000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(15, 18.2, 1000, {.maxSpeed = 30, .minSpeed = 10});
    chassis.moveToPoint(46, 50, 2500, {.maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3});
    set_both(12000);
    chassis.turnToHeading(0, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    toggle_matchload();
    chassis.moveToPoint(45.5, 67, 3000, {.maxSpeed = 60});
    set_intakef(12000);
    set_intakeb(0);
    chassis.waitUntilDone();
    chassis.moveToPoint(45.5, 55, 2000, {.forwards = false, .minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(55, 44, 1000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntil(10);
    toggle_matchload();
    chassis.turnToHeading(0, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    set_both(0);
    chassis.moveToPoint(51.5, -20, 4000, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange =1 });
    chassis.turnToHeading(90, 1500);
    chassis.waitUntilDone();
    pros::delay(50);
    resetcoord(4, 90);
    pros::delay(50);


    chassis.moveToPoint(50.5, -36, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 20, .earlyExitRange = 1});
    chassis.turnToHeading(180, 1000, {.minSpeed = 10, .earlyExitRange = 2});
    chassis.moveToPoint(50.5, -22, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(4);
    set_both(12000);
    pros::delay(2000);

    pros::delay(50);
    toggle_matchload();
    set_intakeb(0);
    chassis.moveToPoint(50.5, -57, 3000, {.maxSpeed = 55});
    chassis.waitUntilDone();
    chassis.moveToPoint(50.5, -22, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(20);
    set_both(12000);
    pros::delay(2800);
    toggle_matchload();
    chassis.moveToPose(50.5,-40, 180, 1000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    resetcoord(4, 180);
    chassis.moveToPose(22, -60, 270, 2000, {.lead = 0.3, .maxSpeed = 120});
    chassis.waitUntilDone();   
    toggle_matchload();
    pros::delay(30);
    chassis.moveToPose(0, -60, 270, 6000, {.maxSpeed = 100, .minSpeed = 100});
    chassis.waitUntil(13);
    toggle_matchload();
}
void inch() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 5, 0, 1000);
}

void autonomous() {
    // cant really do skills auto but its not important
    // only run top four autons
    // ball4_left(); // kinda works
    solo_awp_right(); // this works
    // right_quals(); // run this
    // left_quals();
    // skills_auto();

    // left_quals(); // this works
    //ball7_left();
    // solo_awp_right();
    // ball7_right();
    // inch();
    // moveForward();
    // left_quals();
    // ball7_wing_left();
    // skills_auto1();
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
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, 0.9 * rightX);

        // buttons for controller

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { toggle_descore(); }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) { toggle_matchload(); }

        // Control Intake using shoulder buttons (L1/L2)

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            set_intakef(12000);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakef.move_velocity(-200);
            intakeb.move_velocity(-200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            set_both(12000);
        } else {
            set_both(0);
        }

        // delay to save resources
        pros::delay(10);
    }
}