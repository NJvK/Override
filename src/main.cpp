#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cmath>
#include <cstdint>
#include <iterator>

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::MotorGroup leftMotors({-1, -11, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({19, 10, 20}, pros::MotorGearset::blue);

// IMU
pros::Imu imu(4);

// Odometry rotation sensors
pros::Rotation horizontalEnc(18);
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

// DR4B motor
pros::Motor DR4B(7);

// DR4B rotation sensor
constexpr int DR4B_ROTATION_PORT = 8;
pros::Rotation dr4bRotation(DR4B_ROTATION_PORT);

// Other mechanisms
pros::Motor claw(9);
bool clawHolding = false;

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

// Position where the driver last left the lift
double dr4bTarget = 0.0;

// True while L1 or L2 is manually controlling the lift
bool dr4bManualMode = false;

// True when the lift is gently returning to its original bottom position
bool dr4bReturningToBottom = false;

// Manual velocity
double dr4bManualVelocity = 0.0;

// PID values
constexpr double DR4B_KP = 350.0;
constexpr double DR4B_KD = 10.0;

// Maximum voltage PID can use to push the lift back up
constexpr double DR4B_MAX_HOLD_VOLTAGE = 5000.0;

// Amount the lift can fall before PID starts correcting
constexpr double DR4B_HOLD_TOLERANCE = 0.75;

// Original starting position
constexpr double DR4B_BOTTOM_POSITION = 0.0;

// If the driver releases L2 below this height,
// the lift automatically finishes returning to 0
constexpr double DR4B_BOTTOM_RETURN_ZONE = 10.0;

// How close to 0 counts as fully down
constexpr double DR4B_BOTTOM_TOLERANCE = 0.5;

// Manual lift speeds
constexpr double DR4B_UP_SPEED = 100.0;
constexpr double DR4B_DOWN_SPEED = -100.0;

// Slower speed when approaching the bottom
constexpr double DR4B_SLOW_DOWN_SPEED = -40.0;

// Voltage used to gently finish returning to the bottom
constexpr double DR4B_BOTTOM_RETURN_VOLTAGE = -2500.0;

// Get rotation sensor position in normal degrees
double getDR4BPosition() {
    return dr4bRotation.get_position() / 100.0;
}

// Manually move DR4B
void moveDR4B(double velocity) {
    dr4bManualVelocity = velocity;
    dr4bManualMode = true;
    dr4bReturningToBottom = false;
}

// Stop manual movement
void holdDR4B() {
    double position = getDR4BPosition();

    dr4bManualVelocity = 0;
    dr4bManualMode = false;

    // If the lift is close to the original bottom,
    // finish returning all the way to 0
    if (position <= DR4B_BOTTOM_RETURN_ZONE) {
        dr4bTarget = DR4B_BOTTOM_POSITION;
        dr4bReturningToBottom = true;
    } else {
        // Anywhere else, hold exactly where the driver left it
        dr4bTarget = position;
        dr4bReturningToBottom = false;
    }
}

// DR4B PID loop
void runDR4BPID() {
    double previousError = 0.0;
    constexpr double DT = 0.020;

    while (true) {
        double position = getDR4BPosition();

        // Manual control gets priority
        if (dr4bManualMode) {

            // Raising
            if (dr4bManualVelocity > 0) {
                DR4B.move_velocity(dr4bManualVelocity);
            }

            // Lowering
            else if (dr4bManualVelocity < 0) {

                // Stop once it reaches the original starting position
                if (position <= DR4B_BOTTOM_POSITION + DR4B_BOTTOM_TOLERANCE) {
                    DR4B.move_voltage(0);
                }

                // Slow down when approaching the original bottom
                else if (position <= DR4B_BOTTOM_RETURN_ZONE) {
                    DR4B.move_velocity(DR4B_SLOW_DOWN_SPEED);
                }

                // Normal manual lowering
                else {
                    DR4B.move_velocity(dr4bManualVelocity);
                }
            }

            // Keep target following the lift while manually moving
            dr4bTarget = position;
            previousError = 0.0;

            pros::delay(20);
            continue;
        }

        // Finish returning to the original bottom position
        if (dr4bReturningToBottom) {

            if (position > DR4B_BOTTOM_POSITION + DR4B_BOTTOM_TOLERANCE) {

                // Gently continue downward toward 0
                DR4B.move_voltage(
                    static_cast<int>(DR4B_BOTTOM_RETURN_VOLTAGE)
                );

            } else {

                // Original starting position reached
                DR4B.move_voltage(0);

                dr4bTarget = DR4B_BOTTOM_POSITION;
                dr4bReturningToBottom = false;
            }

            previousError = 0.0;

            pros::delay(20);
            continue;
        }

        // Positive error means the lift has fallen below
        // where the driver left it
        double error = dr4bTarget - position;

        // Only use PID if the lift has fallen
        if (error > DR4B_HOLD_TOLERANCE) {

            double derivative =
                (error - previousError) / DT;

            double output =
                DR4B_KP * error +
                DR4B_KD * derivative;

            // PID is only allowed to push upward
            if (output < 0) {
                output = 0;
            }

            // Limit correction power
            if (output > DR4B_MAX_HOLD_VOLTAGE) {
                output = DR4B_MAX_HOLD_VOLTAGE;
            }

            DR4B.move_voltage(
                static_cast<int>(output)
            );

        } else {

            // At the correct height
            DR4B.move_voltage(0);
        }

        previousError = error;

        pros::delay(20);
    }
}

// Initialize
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // DR4B must physically be at its true lowest position
    // when the robot is turned on
    DR4B.tare_position();

    // This makes the original starting position permanently 0 degrees
    dr4bRotation.reset_position();

    dr4bTarget = DR4B_BOTTOM_POSITION;
    dr4bManualMode = false;
    dr4bReturningToBottom = false;

    // PID handles holding instead of motor brake hold
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Start DR4B PID
    static pros::Task dr4bPIDTask([]() {
        runDR4BPID();
    });

    // Brain screen
    static pros::Task screenTask([]() {

        while (true) {

            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);

            pros::lcd::print(
                3,
                "Lift: %.2f",
                getDR4BPosition()
            );

            pros::lcd::print(
                4,
                "Hold: %.2f",
                dr4bTarget
            );

            pros::lcd::print(
                5,
                "Bottom: %s",
                dr4bReturningToBottom ? "RETURN" : "OFF"
            );

            lemlib::telemetrySink()->info(
                "Chassis pose: {}",
                chassis.getPose()
            );

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
void exit_condition(lemlib::Pose target, double exitDist) {
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
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Hold wherever the lift is when driver control starts
    dr4bTarget = getDR4BPosition();
    dr4bManualMode = false;
    dr4bReturningToBottom = false;

    while (true) {

        // Drive
        int leftY = deadband(
            controller.get_analog(
                pros::E_CONTROLLER_ANALOG_LEFT_Y
            )
        );

        int rightX = deadband(
            controller.get_analog(
                pros::E_CONTROLLER_ANALOG_RIGHT_X
            )
        );

        chassis.arcade(leftY, 0.9 * rightX);

        // DR4B manual control

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

            // Manually raise DR4B
            moveDR4B(DR4B_UP_SPEED);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

            // Manually lower DR4B
            moveDR4B(DR4B_DOWN_SPEED);

        } else {

            // When L1/L2 are released
            if (dr4bManualMode) {
                holdDR4B();
            }
        }

        // Claw
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Spin forward and keep spinning after R1 is released
            clawHolding = true;
            claw.move_velocity(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Spin backward and cancel the holding state
            clawHolding = false;
            claw.move_velocity(-200);
        } else {
            // Keep spinning forward only if R1 was the last command
            if (clawHolding) {
                claw.move_velocity(200);
            } else {
                claw.move_velocity(0);
            }
        }

        pros::delay(20);
    }
}