#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cmath>
#include <cstdint>
#include <iterator>

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors

pros::MotorGroup leftMotors({-17, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({2, 1}, pros::MotorGearset::blue);


// IMU

pros::Imu imu(4);


// Odometry rotation sensors

pros::Rotation horizontalEnc(19);
pros::Rotation verticalEnc(-16);

lemlib::TrackingWheel horizontal(
    &horizontalEnc,
    lemlib::Omniwheel::NEW_275,
    -5.75
);

lemlib::TrackingWheel vertical(
    &verticalEnc,
    lemlib::Omniwheel::NEW_275,
    -2.5
);


// DR4B motors

pros::Motor DR4B1(-20);
pros::Motor DR4B2(13);


// DR4B rotation sensor

constexpr int DR4B_ROTATION_PORT = 5;

pros::Rotation dr4bRotation(DR4B_ROTATION_PORT);


// Other mechanisms

pros::MotorGroup Toggle({6, 8});

pros::adi::DigitalOut tClaw('H');
pros::adi::DigitalOut claw('B');


// Basic constants

const double PI = 3.14159265358979323846;


// Joystick deadband

int deadband(int value) {
    if (std::abs(value) < 5) {
        return 0;
    }

    return value;
}

// Drivetrain

lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    10,
    lemlib::Omniwheel::NEW_275,
    360,
    2
);

// Linear PID

lemlib::ControllerSettings linearController(
    5.78,
    0,
    6,
    0.5,
    1,
    75,
    2,
    150,
    0
);

// Angular PID

lemlib::ControllerSettings angularController(
    3.7,
    0,
    25.5,
    0,
    1,
    50,
    2,
    200,
    0
);

// Odometry sensors

lemlib::OdomSensors sensors = {
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
};

// Driver curves

lemlib::ExpoDriveCurve throttleCurve(
    3,
    10,
    1.019
);

lemlib::ExpoDriveCurve steerCurve(
    3,
    10,
    1.019
);
// Chassis
lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

// DR4B PID
// Rotation sensor position is converted from centidegrees to degrees.
double dr4bTarget = 0.0;
// Start with these values.
// They will need to be tuned on the actual robot.
constexpr double DR4B_KP = 250.0;
constexpr double DR4B_KI = 0.0;
constexpr double DR4B_KD = 12.0;
// Maximum motor voltage
constexpr double DR4B_MAX_VOLTAGE = 12000.0;
// Software position limits.
//
// These are temporary values.
// Measure your actual fully-down and fully-up positions
// and change these values.
constexpr double DR4B_MIN_POSITION = 0.0;
constexpr double DR4B_MAX_POSITION = 120.0;
// Manual mode lets your existing autonomous code still use
// DR4B(80), DR4B(-80), etc.
bool dr4bManualMode = false;
double dr4bManualVelocity = 0.0;
// Get rotation sensor position in normal degrees

double getDR4BPosition() {
    return dr4bRotation.get_position() / 100.0;
}

// Set PID target

void setDR4BTarget(double target) {
    if (target < DR4B_MIN_POSITION) {
        target = DR4B_MIN_POSITION;
    }

    if (target > DR4B_MAX_POSITION) {target = DR4B_MAX_POSITION;}
    dr4bTarget = target;
    dr4bManualMode = false;
}

// DR4B PID loop

void runDR4BPID() {
    double previousError = 0.0;
    double integral = 0.0;
    constexpr double DT = 0.020;

    while (true) {
        double position = getDR4BPosition();

        // Existing autonomous functions can temporarily
        // control the lift using velocity.

        if (dr4bManualMode) {
            DR4B1.move_velocity(dr4bManualVelocity);
            DR4B2.move_velocity(dr4bManualVelocity);
            // Keep the PID target following the actual lift
            // while manually moving it.
            dr4bTarget = position;
            previousError = 0.0;
            integral = 0.0;
            pros::delay(20);
            continue;
        }

        double error = dr4bTarget - position;

        // Only build integral when reasonably close.

        if (std::fabs(error) < 10.0) {
            integral += error * DT;
        } else {
            integral = 0.0;
        }

        // Integral protection

        if (integral > 20.0) {integral = 20.0;}

        if (integral < -20.0) {integral = -20.0;}

        double derivative = (error - previousError) / DT;
        double output = DR4B_KP * error + DR4B_KI * integral + DR4B_KD * derivative;
        // Limit voltage

        if (output > DR4B_MAX_VOLTAGE) {output = DR4B_MAX_VOLTAGE;}
        if (output < -DR4B_MAX_VOLTAGE) {output = -DR4B_MAX_VOLTAGE;}

        DR4B1.move_voltage(static_cast<int>(output));
        DR4B2.move_voltage(static_cast<int>(output));
        previousError = error;
        pros::delay(20);
    }
}

// Existing autonomous compatibility functions

void DR4B(float speed) {
    dr4bManualVelocity = speed;
    dr4bManualMode = true;
}


void DR4BStop() {
    // Switch back to PID and hold wherever the lift currently is.
    dr4bTarget = getDR4BPosition();
    dr4bManualVelocity = 0;
    dr4bManualMode = false;
}

// Optional wait function for autonomous

void waitForDR4B(double tolerance = 2.0,int timeout = 1500) {
    std::uint32_t start = pros::millis();

    while (std::fabs(dr4bTarget - getDR4BPosition()) > tolerance) {
        if (pros::millis() - start >static_cast<std::uint32_t>(timeout)) {
            break;
        }
        pros::delay(10);
    }
}

// Claw
bool clawOn = false;
bool tclawOn = true;

void toggleClaw() {
    clawOn = !clawOn;
    claw.set_value(clawOn);
}


void toggleClawO() {
    tclawOn = !tclawOn;
    tClaw.set_value(tclawOn);
}


// Initialize

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // Zero normal motor encoders

    DR4B1.tare_position();
    DR4B2.tare_position();

    // IMPORTANT:
    //
    // The DR4B should physically be in its fully-down
    // starting position when the robot turns on.

    dr4bRotation.reset_position();
    dr4bTarget = 0.0;
    DR4B1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Start DR4B PID

    static pros::Task dr4bPIDTask([]() {runDR4BPID();});

    // Starting pneumatic states

    tclawOn = true;
    tClaw.set_value(true);
    claw.set_value(clawOn);

    // Brain screen

    static pros::Task screenTask([]() {

        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            pros::lcd::print(3, "Lift: %.2f", getDR4BPosition());
            pros::lcd::print(4, "Target: %.2f", dr4bTarget);

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }

    });
}


// Disabled

void disabled() {

}


// Competition initialize

void competition_initialize() {

}


// Pure pursuit asset

ASSET(example_txt);


// Exit condition

void exit_condition(
    lemlib::Pose target,
    double exitDist) {
    chassis.waitUntil(fabs(chassis.getPose().distance(target)) - exitDist);
    chassis.cancelMotion();
}


// Red left autonomous

void redLeft() {

}


// Red right autonomous
void redRight() {

}

// Blue right autonomous

void blueRight() {

    DR4B1.set_brake_mode(
        pros::E_MOTOR_BRAKE_COAST
    );

    DR4B2.set_brake_mode(
        pros::E_MOTOR_BRAKE_COAST
    );


    chassis.setPose(0, 0, 0);


    DR4B(80);

    pros::delay(500);


    DR4B(-80);


    chassis.moveToPoint(
        0,
        -5,
        1000,
        {.forwards = false}
    );


    pros::delay(450);


    DR4BStop();


    chassis.moveToPoint(
        0,
        14,
        2000,
        {
            .minSpeed = 90,
            .earlyExitRange = 2
        }
    );


    chassis.turnToHeading(
        -89,
        1000
    );


    chassis.moveToPose(
        -38.93,
        17.8,
        -91.89,
        1500,
        {.minSpeed = 80}
    );


    chassis.waitUntilDone();


    pros::delay(300);


    toggleClaw();


    pros::delay(500);


    DR4B1.set_brake_mode(
        pros::E_MOTOR_BRAKE_HOLD
    );

    DR4B2.set_brake_mode(
        pros::E_MOTOR_BRAKE_HOLD
    );
}


// Blue left autonomous

void blueLeft() {

}


// Skills autonomous

void skills() {

}


// Autonomous selector

void autonomous() {
    // redLeft();
    // redRight();
    // blueLeft();
    // blueRight();
    // skills();
}


// Driver control

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    // Start driver control holding the lift
    // wherever it currently is.

    setDR4BTarget(getDR4BPosition());

    while (true) {
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
        chassis.arcade(leftY, 0.9 * rightX);


        // DR4B PID target control
        //
        // Holding L1 raises the desired lift position.
        // Holding L2 lowers the desired lift position.
        //
        // Releasing the button leaves the target where it is,
        // so the PID holds the lift there.

        if (
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) 
        {
            setDR4BTarget(dr4bTarget + 1.5);
        }else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            setDR4BTarget(dr4bTarget - 1.5);
        }
        // R1 toggles claw
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            toggleClaw();
        }

        pros::delay(20);
    }
}