#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
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
pros::MotorGroup DR4B({-7, -8}, pros::MotorGearset::green);

// DR4B rotation sensor
constexpr int DR4B_ROTATION_PORT = 0;
pros::Rotation dr4bRotation(DR4B_ROTATION_PORT);

// Other mechanisms
pros::Motor claw(9);

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

// True while L1 or L2 manually controls the lift
bool dr4bManualMode = false;

// PID values
constexpr double DR4B_KP = 350.0;
constexpr double DR4B_KD = 10.0;

// Maximum voltage PID can use to correct the lift
constexpr double DR4B_MAX_HOLD_VOLTAGE = 5000.0;

// Amount the lift can fall before PID corrects it
constexpr double DR4B_HOLD_TOLERANCE = 0.75;

// No PID correction between 0 and 5 degrees
constexpr double DR4B_BOTTOM_DEADZONE = 10.0;

// Manual DR4B speeds
constexpr double DR4B_UP_SPEED = 100.0;
constexpr double DR4B_DOWN_SPEED = -100.0;

// Get rotation sensor position in degrees
double getDR4BPosition() {
    return dr4bRotation.get_position() / 100.0;
}

// Save current lift position
void holdDR4B() {
    dr4bTarget = getDR4BPosition();
    dr4bManualMode = false;
}

// DR4B PID loop
void runDR4BPID() {
    double previousError = 0.0;
    constexpr double DT = 0.020;

    while (true) {

        // Driver manually controls the lift
        if (dr4bManualMode) {
            previousError = 0.0;
            pros::delay(20);
            continue;
        }

        double position = getDR4BPosition();

        // No PID correction between 0 and 5 degrees
        if (position >= 0.0 && position <= DR4B_BOTTOM_DEADZONE) {
            DR4B.move_voltage(0);
            previousError = 0.0;

            pros::delay(20);
            continue;
        }

        // Positive error means the lift fell below the hold position
        double error = dr4bTarget - position;

        // Only correct upward if the lift falls
        if (error > DR4B_HOLD_TOLERANCE) {
            double derivative = (error - previousError) / DT;
            double output = DR4B_KP * error + DR4B_KD * derivative;

            // Only allow upward correction
            if (output < 0) {
                output = 0;
            }

            // Limit correction power
            if (output > DR4B_MAX_HOLD_VOLTAGE) {
                output = DR4B_MAX_HOLD_VOLTAGE;
            }

            DR4B.move_voltage(static_cast<int>(output));

        } else {
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

    // DR4B should start fully down
    DR4B.tare_position();
    dr4bRotation.reset_position();

    dr4bTarget = getDR4BPosition();

    // PID controls holding instead of motor hold
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
            pros::lcd::print(3, "Lift: %.2f", getDR4BPosition());
            pros::lcd::print(4, "Hold: %.2f", dr4bTarget);

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
    holdDR4B();

    // Claw keeps spinning forward after R1 until R2 is used
    bool clawHolding = false;

    while (true) {

        // Drive
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        chassis.arcade(leftY, 0.9 * rightX);

        // DR4B manual control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {

            dr4bManualMode = true;

            // L1 directly raises the lift
            DR4B.move_velocity(DR4B_UP_SPEED);

            // Keep target following current position
            dr4bTarget = getDR4BPosition();

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {

            dr4bManualMode = true;

            double position = getDR4BPosition();

            // Lower until the lift reaches the bottom deadzone
            if (position > DR4B_BOTTOM_DEADZONE) {
                DR4B.move_velocity(DR4B_DOWN_SPEED);
            } else {
                DR4B.move_velocity(0);
            }

            dr4bTarget = position;

        } else {

            // Driver released L1/L2
            if (dr4bManualMode) {
                DR4B.move_velocity(0);
                holdDR4B();
            }
        }

        // Claw
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

            clawHolding = true;
            claw.move_velocity(200);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

            clawHolding = false;
            claw.move_velocity(-200);

        } else {

            if (clawHolding) {
                claw.move_velocity(60);
            } else {
                claw.move_velocity(0);
            }
        }

        pros::delay(20);
    }
}