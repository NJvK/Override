#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::MotorGroup leftMotors({-1, -11, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({19, 10, 20}, pros::MotorGearset::blue);

// IMU
pros::Imu imu(18);

// Odometry rotation sensors
pros::Rotation horizontalEnc(13);
pros::Rotation verticalEnc(-14);

lemlib::TrackingWheel horizontal(
    &horizontalEnc,
    1.976,      // was lemlib::Omniwheel::NEW_275 (2.75 * 24 / 33.4)
    -2.5
);

lemlib::TrackingWheel vertical(
    &verticalEnc,
    1.982,      // was lemlib::Omniwheel::NEW_275 (2.75 * 48 / 66.6)
    -0.1
);

// DR4B motor
pros::MotorGroup DR4B({-7, 2}, pros::MotorGearset::green);

// DR4B rotation sensor
constexpr int DR4B_ROTATION_PORT = 3;
pros::Rotation dr4bRotation(DR4B_ROTATION_PORT);

// Other mechanisms
pros::Motor claw(9);
pros::Motor toggleMechanism(-4);

// =====================================================================
//                     AUTO STACK TUNING (all in one place)
// =====================================================================
// All heights are in DEGREES of the DR4B rotation sensor, the same number
// shown on the brain screen as "Lift:". To find a height: lift manually to
// the right spot on the field and read the Lift value.
// NOTE: every height should be above DR4B_BOTTOM_DEADZONE (the PID is off
// below it). Values at or under it get bumped up to DR4B_BOTTOM_DEADZONE + 1.

// --- Optical sensor (the sensor at the bottom of the robot facing the goal) ---
constexpr int OPTICAL_PORT = 5;               // CHANGE to the real port
constexpr int OPTICAL_LED_BRIGHTNESS = 100;   // 0-100

// --- Alliance color: true = we are red, false = we are blue ---
constexpr bool ALLIANCE_IS_RED = true;

// --- Goal hue ranges (hue is 0-359) ---
// If MIN is bigger than MAX the range wraps past 0 (needed for red).
// The brain screen prints the live hue and proximity so you can tune these.
constexpr double RED_HUE_MIN = 300.01;
constexpr double RED_HUE_MAX = 360.0;
constexpr double BLUE_HUE_MIN = 220.0;
constexpr double BLUE_HUE_MAX = 239.0;
constexpr double NEUTRAL_HUE_MIN = 240000.0;      // PLACEHOLDER - tune for the dark neutral goal
constexpr double NEUTRAL_HUE_MAX = 3000000.0;     // PLACEHOLDER - tune for the dark neutral goal

// --- Goal detection ---
// Optical proximity is 0-255, HIGHER = CLOSER. The goal only counts as seen
// when proximity is at or above this value.
constexpr int GOAL_PROXIMITY_THRESHOLD = 100;
// The new goal must be seen this many loops in a row (1 loop = 20 ms)
// before it switches. Raise it if it flickers between goals.
constexpr int GOAL_CONFIRM_CYCLES = 5;
// A brief dropout (reading nothing for a moment, which happens when a hue sits near the edge of its
// range) does NOT reset that count. It only resets after this many missed loops in a row.
constexpr int GOAL_MISS_TOLERANCE = 4;

// --- Height lists (degrees), one value per stack position 1-5 ---
constexpr int NUM_STACK_HEIGHTS = 5;
constexpr double ALLIANCE_HEIGHTS[NUM_STACK_HEIGHTS] = {
    167.95,   // pin/cup 1
    256.11,   // pin/cup 2
    368.52,   // pin/cup 3
    490.69,   // pin/cup 4
    642.66    // pin/cup 5
};
constexpr double NEUTRAL_HEIGHTS[NUM_STACK_HEIGHTS] = {
    50.0,   // pin/cup 1   PLACEHOLDER
    85.0,   // pin/cup 2   PLACEHOLDER
    120.0,  // pin/cup 3   PLACEHOLDER
    155.0,  // pin/cup 4   PLACEHOLDER
    190.0   // pin/cup 5   PLACEHOLDER
};

// --- Auto height behavior ---
// Auto height feature starts ON (true) or OFF (false) when driver control starts
constexpr bool AUTO_HEIGHT_ON_AT_START = false;
// A lift press counts as "still going up" until the lift is within this many
// degrees of its target. If a new goal is detected while still going up, that
// press is treated as a misfire and the old goal's counter is rolled back.
constexpr double AUTO_ARRIVE_TOLERANCE = 3.0;

// --- Lift motion (how the PID travels to a height) ---
// Voltage cap (mV, 12000 = full power) while the lift is travelling to a new height.
// Holding a height still uses DR4B_MAX_HOLD_VOLTAGE further down.
constexpr double DR4B_MAX_MOVE_VOLTAGE = 12000.0;
// How fast the PID target slides toward a new height (degrees per second).
// This is what stops the bouncing: the lift is eased there instead of being
// slammed at the full height. Lower = gentler, higher = faster. If the lift
// can't keep up with it, lower it; if it feels slow, raise it.
constexpr double DR4B_SLEW_UP = 600.0;     // top speed of the target going up (degrees/second)
constexpr double DR4B_SLEW_DOWN = 700.0;   // top speed of the target going down
// How fast the target speeds up and slows down (degrees/second per second).
// The lift only RESTS on the standoff, so if the standoff slows down too hard the lift
// flies off it (the rubber bands throw it upward) and then drops back: that is the bounce.
// The slow-down rate matters most. LOWER DECEL = softer stop = less bounce, but a longer finish.
// The bounce is worst near the bottom because that's where the bands pull hardest, so
// if it only bounces low, lower DR4B_DECEL_UP.
constexpr double DR4B_ACCEL = 4000.0;
constexpr double DR4B_DECEL_UP = 2000.0;
constexpr double DR4B_DECEL_DOWN = 3000.0;
// Error (degrees) treated as "close enough". No voltage is added inside this.
constexpr double DR4B_PID_DEADBAND = 0.5;   // keep this small: with rubber bands, a big deadband makes the lift drift to its edge and bounce
// Speed feedforward: millivolts per degree/second of target speed. This gives the lift the
// power to follow the moving target (so low gains don't make the trip weak), and it drops
// automatically as the target slows down, so it never fights the slow-down.
// A green motor free-spins about 1200 deg/s at 12000 mV (about 10 mV per deg/s with no load).
// Raise UP if the lift lags behind the target going up; lower it if the lift runs ahead of it.
constexpr double DR4B_KV_UP = 11.0;
constexpr double DR4B_KV_DOWN = 4.0;     // lower because gravity helps going down
// Degrees from the real target where the voltage cap is full; it shrinks to the hold cap at the target
constexpr double DR4B_TRAVEL_BOOST_FADE = 60.0;
// Extra damping near the bottom of the lift, where the rubber bands bounce it the most.
// Below DR4B_LOW_ZONE degrees the gains blend toward these scales: full effect at 0 degrees,
// no effect at the top of the zone. KD scale above 1 = more damping, KP scale below 1 = softer push.
// If the bottom gets worse, set KP scale back to 1.0 and only use the KD scale.
constexpr double DR4B_LOW_ZONE = 300.0;
constexpr double DR4B_LOW_KP_SCALE = 1.0;   // 1.0 = no change (this was 0.7 and didn't help)
constexpr double DR4B_LOW_KD_SCALE = 1.0;   // 1.0 = no change (this was 1.6 and didn't help)
// Optional integral term to stop the lift slowly sagging below its height.
// 0 = off. If you turn it on, start small (like 20).
constexpr double DR4B_KI = 0.0;
constexpr double DR4B_I_ZONE = 5.0;     // degrees: integral only builds this close to the target
constexpr double DR4B_I_MAX = 3000.0;   // mV: most voltage the integral can add

// Simple PID switch. true  = the original-style loop: DR4B_KP / DR4B_KD (350 / 10) straight on the real
//                             target, no sliding setpoint, no speed feedforward, no motor hold.
//                    false = the smooth loop (SMOOTH_KP / SMOOTH_KD, sliding setpoint, feedforward, motor hold).
// Swap this to compare the two on the robot. Heights, auto presses and the lower-limit re-zero work in both.
constexpr bool DR4B_USE_SIMPLE_PID = false;

// Motor hold: once the lift has arrived and stopped, lock it with the motors' built-in hold
// instead of holding it with the voltage PID. The lift is geared 12:84 (7:1), so the motors
// hold it very firmly, and the built-in hold reacts much faster than the PID loop can.
// This is what stops the leftover bounce. Set to false to go back to the PID-only hold.
constexpr bool DR4B_USE_BRAKE_HOLD = true;
constexpr double DR4B_HOLD_ENTER_ERROR = 4.0;    // degrees: must be this close to the target
constexpr double DR4B_HOLD_ENTER_SPEED = 60.0;   // degrees/second: and moving slower than this
constexpr int DR4B_HOLD_ENTER_CYCLES = 12;       // loops in a row (10 ms each) before it locks
constexpr double DR4B_HOLD_EXIT_ERROR = 12.0;    // degrees: unlocks if the target moves or the lift is pushed this far

// --- Lift lower limit re-zero ---
// If the lift reads lower than this (degrees) the sensor is re-zeroed to 0
// at that spot. Checked constantly, in every mode.
constexpr double DR4B_LOWER_LIMIT = -1.0;
// Must read below the limit this many loops in a row (10 ms each) to re-zero
constexpr int DR4B_LOWER_LIMIT_CONFIRM_CYCLES = 6;

// --- Manual lift release ---
// After releasing L1/L2 the lift brakes this long before PID takes over
constexpr int DR4B_SETTLE_TIME_MS = 250;

// =====================================================================
//              PIN PICKUP + LIFT ZEROING TUNING (all in one place)
// =====================================================================
// --- Pin pickup: when the claw grabs a pin while the lift is low, the lift goes to PIN_LIFT_HEIGHT ---
// "Grabbed" = R1 is held (claw driving forward) but the claw has slowed down to a stall.
// The brain screen prints the live claw speed ("Claw V:") so you can tune the stall values.
constexpr bool PIN_LIFT_ENABLED = true;
constexpr double PIN_LIFT_HEIGHT = 10.0;        // lift height (degrees) it goes to after a pin is grabbed
// The lift always rises at least this many degrees from where it is when the pin is grabbed,
// so it still lifts even if the lift reads something like 5-15 when it is all the way down
constexpr double PIN_LIFT_MIN_RISE = 10.0;
constexpr double PIN_LIFT_ZONE_MIN = -5.0;      // only works when the lift is between these two heights
constexpr double PIN_LIFT_ZONE_MAX = 30.0;
// The pin lift drives the motors directly at this speed (rpm) until it gets there, then locks.
constexpr double PIN_LIFT_SPEED = 200.0;        // full speed so it gets off the ground fast
constexpr int PIN_LIFT_VOLTAGE = 12000;         // mV: full power for the pin hop (instant, no ramp-up). Lower to ~10000 if it overshoots
constexpr int PIN_LIFT_TIMEOUT_MS = 750;        // gives up pushing after this long (in case something blocks it)
// Minimum push (mV) the PID uses for a pin lift target (backup, the direct drive above does the real work)
constexpr double PIN_LIFT_MIN_VOLTAGE = 4000.0;
// Controller buzzes this when a grab is detected, so you can tell detection from lifting problems
constexpr const char* PIN_LIFT_RUMBLE = "..";
constexpr double CLAW_STALL_SPEED = 50.0;       // rpm: the claw counts as "slow / stalled" below this (while R1 is held) (was 30 - higher = detects sooner)
constexpr double CLAW_HOLD_STALL_SPEED = 15.0;  // rpm: same, after R1 is released and the claw keeps squeezing at 60 rpm
// Stall checks start once the claw reaches CLAW_SPINUP_SPEED, or after CLAW_STALL_GRACE_MS at the latest
constexpr double CLAW_SPINUP_SPEED = 80.0;      // rpm: the claw counts as "got moving" above this
constexpr int CLAW_STALL_GRACE_MS = 120;        // ms: start checking anyway after this (element already in the claw)
constexpr int CLAW_STALL_CONFIRM_MS = 20;       // ms: must stay slow this long before it counts as a pin (1 loop) (was 40)
                                                // raise to 40-80 if it fires when the claw hasn't grabbed anything

// --- Lift zeroing: hold L2 at the bottom and the lift is pushed gently down until it stalls, then zeroed ---
// This fixes the lift reading something like 5 when it is all the way down.
// It only runs once the lift is below DR4B_BOTTOM_DEADZONE while L2 is held.
constexpr bool LIFT_ZERO_ON_STALL = true;
constexpr double LIFT_ZERO_DRIVE_SPEED = -50.0; // rpm: gentle push down while looking for the stall
constexpr double LIFT_ZERO_STALL_SPEED = 20.0;  // degrees/second: slower than this counts as stopped
constexpr int LIFT_ZERO_GRACE_MS = 150;         // wait this long after starting the push before checking
constexpr int LIFT_ZERO_STALL_MS = 200;         // must stay stopped this long before it zeroes
// =====================================================================

// =====================================================================
//                      TIP PROTECTION TUNING (all in one place)
// =====================================================================
// If the robot tips forward or backward more than TIP_ROLL_LIMIT degrees (roll axis of the inertial
// sensor), the lift lowers by itself and ignores the lift buttons until the robot levels out again.
// The lift is lowered until it reaches the bottom OR stops moving (its hard stop), then the motors are
// switched off, so it never keeps pushing against the stop. If it stopped on the hard stop, the lift
// sensor is also set to 0 there.
// The brain screen prints the live roll ("Roll:"). Tip the robot by hand to check the sign and size.
constexpr bool TIP_PROTECTION_ENABLED = true;
constexpr double TIP_ROLL_LIMIT = 13.0;          // degrees: lowers the lift when |roll| is more than this
constexpr double TIP_ROLL_HYSTERESIS = 3.0;      // buttons work again once |roll| is below (limit - this)
constexpr int TIP_CONFIRM_CYCLES = 3;            // must stay over the limit this many loops (20 ms each)
constexpr double TIP_LOWER_SPEED = -150.0;       // rpm while the lift is above the bottom deadzone
constexpr double TIP_LOWER_SLOW_SPEED = -50.0;   // rpm for the last stretch near the hard stop
constexpr double TIP_LOWER_STOP_POSITION = 1.0;  // degrees: counts as "all the way down" at or below this
constexpr double TIP_STALL_SPEED = 20.0;         // degrees/second: slower than this counts as stopped
constexpr int TIP_STALL_GRACE_MS = 150;          // wait this long after the slow push starts before checking
constexpr int TIP_STALL_MS = 150;                // must stay stopped this long before the motors switch off
// =====================================================================

// --- Controller vibration patterns ("." = short, "-" = long) ---
constexpr const char* GOAL_CHANGE_RUMBLE = ".";   // goal change detected
constexpr const char* AUTO_TOGGLE_RUMBLE = "-";   // B pressed (auto on/off)
// =====================================================================

// Optical sensor
pros::Optical optical(OPTICAL_PORT);

// Goal types we can detect (middle goal is the same color as neutral, so it is treated as neutral)
enum GoalType {
    GOAL_ALLIANCE = 0,
    GOAL_NEUTRAL = 1
};

// Auto stack state
bool autoHeightEnabled = AUTO_HEIGHT_ON_AT_START;
GoalType currentGoal = GOAL_ALLIANCE;          // goal list currently in use
int stackCount[2] = {0, 0};                    // how many presses we've used on each goal (0-5)
bool autoLiftInProgress = false;               // true while the lift is still going to an auto target
GoalType lastPressGoal = GOAL_ALLIANCE;        // goal used by the most recent auto press
int lastPressPrevCount = 0;                    // that goal's counter before the most recent press
int goalCandidate = -1;                        // goal being confirmed by the sensor
int goalCandidateSeen = 0;                     // loops the candidate has been seen
int goalCandidateMisses = 0;                   // loops in a row the candidate has been missed
int pinLiftCount = 0;                          // times the pin pickup moved the lift (shown on the brain screen)
int liftZeroCount = 0;                         // times the lift was zeroed by a stall (shown on the brain screen)

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
    0       // slew: gradual speed-up so it doesn't wheelie (was 0). Higher = quicker start, lower = gentler.
            // Only affects auton moves (driver control doesn't use this)
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

// True when the target was set on purpose inside the bottom deadzone (the pin pickup height),
// so the PID must actually go there instead of treating it as "stay at the bottom"
bool dr4bTargetForced = false;

// True while L1 or L2 manually controls the lift (also true while it settles after release)
bool dr4bManualMode = false;

// True while the lift is braking after L1/L2 was released, waiting to capture its position
bool dr4bSettling = false;
uint32_t dr4bSettleStartMs = 0;

// PID values (used when DR4B_USE_SIMPLE_PID is true; the smooth loop uses SMOOTH_KP / SMOOTH_KD instead)
constexpr double DR4B_KP = 350.0;
constexpr double DR4B_KD = 10.0;

// Maximum voltage PID can use to HOLD the lift in place
// (while travelling to a new height it uses DR4B_MAX_MOVE_VOLTAGE instead)
constexpr double DR4B_MAX_HOLD_VOLTAGE = 5000.0;

// Amount the lift can fall before PID corrects it
constexpr double DR4B_HOLD_TOLERANCE = 0.75;

// No PID correction between 0 and 10 degrees, but only when the target is also down there
// (otherwise the lift could never start moving from the bottom)
constexpr double DR4B_BOTTOM_DEADZONE = 10.0;

// Manual DR4B speeds
constexpr double DR4B_UP_SPEED = 200.0;
constexpr double DR4B_DOWN_SPEED = -100.0;

// Get rotation sensor position in degrees
double getDR4BPosition() {
    return dr4bRotation.get_position() / 100.0;
}

// Save current lift position
void holdDR4B() {
    dr4bTarget = getDR4BPosition();
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
}

// DR4B PID loop - Smooth Hold & Motion
void runDR4BPID() {
    constexpr double DT = 0.010;  // loop time in seconds (loop runs every 10 ms, matches the pros::delay calls below)

    double previousError = 0.0;
    double filteredDerivative = 0.0;
    double integral = 0.0;
    int belowLimitCount = 0;

    // Motor hold state
    bool brakeHolding = false;
    int settledCycles = 0;
    double lockedTarget = 0.0;    // the target the motor hold locked onto
    double lastPosition = getDR4BPosition();
    double positionSpeed = 0.0;   // smoothed lift speed in degrees/second

    // Setpoint that slides toward dr4bTarget at a limited speed so the lift
    // glides to each height instead of being slammed at it
    double profiledTarget = getDR4BPosition();
    double profileVelocity = 0.0;   // how fast that setpoint is moving (degrees/second, + = up)

    // Gains (low on purpose for a steady hold; travel power comes from the travel boost)
    constexpr double SMOOTH_KP = 70.0;
    constexpr double SMOOTH_KD = 20.0;  // Damps overshoot

    while (true) {

        // Track the lift's speed (used to decide when it has stopped)
        double nowPosition = getDR4BPosition();
        positionSpeed = 0.7 * positionSpeed + 0.3 * ((nowPosition - lastPosition) / DT);
        lastPosition = nowPosition;

        // Constantly check if the lift has gone lower than the lower limit.
        // If so, the sensor's zero is too high, so re-zero it at this spot.
        // Runs in every mode (manual, PID, auto).
        if (getDR4BPosition() < DR4B_LOWER_LIMIT) {
            belowLimitCount++;
        } else {
            belowLimitCount = 0;
        }

        if (belowLimitCount >= DR4B_LOWER_LIMIT_CONFIRM_CYCLES) {
            dr4bRotation.reset_position();
            belowLimitCount = 0;
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = 0.0;
            profileVelocity = 0.0;
            lastPosition = 0.0;
            positionSpeed = 0.0;

            // Don't leave a target below the new zero
            if (dr4bTarget < 0.0) {
                dr4bTarget = 0.0;
            }
        }

        // Driver manually controls the lift
        if (dr4bManualMode) {
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = getDR4BPosition();
            profileVelocity = 0.0;
            brakeHolding = false;
            settledCycles = 0;
            pros::delay(10);
            continue;
        }

        double position = getDR4BPosition();

        // No correction in bottom deadzone, as long as the target is down there too.
        // (If the target is higher, e.g. an auto lift press, PID must run to get it moving.
        // The same goes for the pin pickup height, which is inside the deadzone on purpose.)
        if (position >= 0.0 && position <= DR4B_BOTTOM_DEADZONE && dr4bTarget <= DR4B_BOTTOM_DEADZONE && !dr4bTargetForced) {
            if (brakeHolding) {
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                brakeHolding = false;
            }
            settledCycles = 0;
            DR4B.move_voltage(0);
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = position; profileVelocity = 0.0;
            pros::delay(10);
            continue;
        }

        // Simple PID (original style): DR4B_KP / DR4B_KD straight on the real target.
        // The lift is left alone until it is off its target by DR4B_HOLD_TOLERANCE degrees.
        if (DR4B_USE_SIMPLE_PID) {
            double simpleError = dr4bTarget - position;

            if (std::abs(simpleError) < DR4B_HOLD_TOLERANCE) {
                DR4B.move_voltage(0);
            } else {
                double simpleDerivative = (simpleError - previousError) / DT;
                double simpleOutput = (DR4B_KP * simpleError) + (DR4B_KD * simpleDerivative);

                if (simpleOutput > DR4B_MAX_HOLD_VOLTAGE) simpleOutput = DR4B_MAX_HOLD_VOLTAGE;
                if (simpleOutput < -DR4B_MAX_HOLD_VOLTAGE) simpleOutput = -DR4B_MAX_HOLD_VOLTAGE;

                // Pin pickup lift: make sure the short hop has enough power to get moving
                if (dr4bTargetForced && simpleError > 2.0 && simpleOutput < PIN_LIFT_MIN_VOLTAGE) {
                    simpleOutput = PIN_LIFT_MIN_VOLTAGE;
                }

                DR4B.move_voltage(static_cast<int>(simpleOutput));
            }

            previousError = simpleError;
            profiledTarget = position; profileVelocity = 0.0;
            pros::delay(10);
            continue;
        }

        // Motor hold: lock the lift once it has arrived and stopped (see DR4B_USE_BRAKE_HOLD)
        if (DR4B_USE_BRAKE_HOLD) {
            if (brakeHolding) {
                if (std::abs(dr4bTarget - position) > DR4B_HOLD_EXIT_ERROR
                    || std::abs(dr4bTarget - lockedTarget) > 0.5) {
                    // Target moved (new lift press) or the lift was pushed away: go back to PID
                    brakeHolding = false;
                    settledCycles = 0;
                    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                    previousError = 0.0;
                    filteredDerivative = 0.0;
                    integral = 0.0;
                    profiledTarget = position; profileVelocity = 0.0;   // start the slide from where the lift is now
                } else {
                    // Stay locked, nothing for the PID to do
                    previousError = 0.0;
                    filteredDerivative = 0.0;
                    integral = 0.0;
                    profiledTarget = position; profileVelocity = 0.0;
                    pros::delay(10);
                    continue;
                }
            } else if (std::abs(dr4bTarget - position) < DR4B_HOLD_ENTER_ERROR
                       && std::abs(positionSpeed) < DR4B_HOLD_ENTER_SPEED) {
                settledCycles++;
                if (settledCycles >= DR4B_HOLD_ENTER_CYCLES) {
                    brakeHolding = true;
                    lockedTarget = dr4bTarget;
                    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    DR4B.brake();
                    profiledTarget = position; profileVelocity = 0.0;
                    pros::delay(10);
                    continue;
                }
            } else {
                settledCycles = 0;
            }
        }

        // Move the setpoint toward the real target with a speed-up rate, a top speed and a
        // slow-down rate. It starts slowing down early enough to stop exactly at the target.
        double remaining = dr4bTarget - profiledTarget;
        double direction = (remaining >= 0.0) ? 1.0 : -1.0;
        double topSpeed = (direction > 0.0) ? DR4B_SLEW_UP : DR4B_SLEW_DOWN;
        double decelRate = (direction > 0.0) ? DR4B_DECEL_UP : DR4B_DECEL_DOWN;
        double speedToward = profileVelocity * direction;   // negative = moving away from the target

        if (speedToward < 0.0) {
            // Moving the wrong way: slow down and turn around first
            speedToward += DR4B_ACCEL * DT;
            if (speedToward > 0.0) speedToward = 0.0;
        } else {
            double stopDistance = speedToward * speedToward / (2.0 * decelRate);
            if (std::abs(remaining) <= stopDistance) {
                speedToward -= decelRate * DT;
                if (speedToward < 0.0) speedToward = 0.0;
            } else {
                speedToward += DR4B_ACCEL * DT;
                if (speedToward > topSpeed) speedToward = topSpeed;
            }
        }

        double profileStep = speedToward * DT;
        if (std::abs(remaining) <= 1.0 || profileStep >= std::abs(remaining)) {
            profiledTarget = dr4bTarget;
            profileVelocity = 0.0;
        } else {
            profiledTarget += direction * profileStep;
            profileVelocity = direction * speedToward;
        }

        double error = profiledTarget - position;

        // Continuous derivative calculation with low-pass filter to prevent voltage spikes
        double rawDerivative = (error - previousError) / DT;
        filteredDerivative = 0.7 * filteredDerivative + 0.3 * rawDerivative;

        // Optional integral (off when DR4B_KI is 0), only builds close to the target
        if (DR4B_KI != 0.0 && std::abs(error) < DR4B_I_ZONE) {
            integral += error * DT;
            double integralLimit = DR4B_I_MAX / DR4B_KI;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;
        } else {
            integral = 0.0;
        }

        // Deadband: errors smaller than the deadband add no push, and the push grows
        // smoothly from zero outside it (a hard on/off cutoff made the lift hunt up and down)
        double pushError = 0.0;
        if (error > DR4B_PID_DEADBAND) {
            pushError = error - DR4B_PID_DEADBAND;
        } else if (error < -DR4B_PID_DEADBAND) {
            pushError = error + DR4B_PID_DEADBAND;
        }

        // Gains blend toward the low-height scales near the bottom (full at 0 degrees, none at DR4B_LOW_ZONE)
        double lowBlend = 1.0 - position / DR4B_LOW_ZONE;
        if (lowBlend < 0.0) lowBlend = 0.0;
        if (lowBlend > 1.0) lowBlend = 1.0;
        double kp = SMOOTH_KP * (1.0 + (DR4B_LOW_KP_SCALE - 1.0) * lowBlend);
        double kd = SMOOTH_KD * (1.0 + (DR4B_LOW_KD_SCALE - 1.0) * lowBlend);

        double output = (kp * pushError) + (DR4B_KI * integral);

        // Only damp (D term) when we are actually outside the deadband
        if (pushError != 0.0) {
            output += kd * filteredDerivative;
        }

        // Speed feedforward: the power to follow the moving target (drops to zero as the target stops)
        output += profileVelocity * (profileVelocity > 0.0 ? DR4B_KV_UP : DR4B_KV_DOWN);

        // How far the lift is from its real target (raises the voltage cap on long moves)
        double distToTarget = dr4bTarget - position;
        double fade = std::abs(distToTarget) / DR4B_TRAVEL_BOOST_FADE;
        if (fade > 1.0) fade = 1.0;

        // Allow bidirectional holding (both up and down correction); the voltage cap also
        // rises smoothly toward the move cap the farther the lift is from its target
        double maxVoltage = DR4B_MAX_HOLD_VOLTAGE + (DR4B_MAX_MOVE_VOLTAGE - DR4B_MAX_HOLD_VOLTAGE) * fade;
        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        // Pin pickup lift: the move is too short for the PID/feedforward to build enough power,
        // so push with at least PIN_LIFT_MIN_VOLTAGE until it is almost there
        if (dr4bTargetForced && distToTarget > 2.0 && output < PIN_LIFT_MIN_VOLTAGE) {
            output = PIN_LIFT_MIN_VOLTAGE;
        }

        DR4B.move_voltage(static_cast<int>(output));

        previousError = error;
        pros::delay(10);
    }
}

// ---------------------------------------------------------------------
// Auto stack helpers
// ---------------------------------------------------------------------

// True if hue is inside [lo, hi]. If lo > hi the range wraps past 0 (red).
bool hueInRange(double hue, double lo, double hi) {
    if (lo <= hi) {
        return hue >= lo && hue <= hi;
    }
    return hue >= lo || hue <= hi;
}

// Returns the goal the optical sensor currently sees, or -1 if nothing
// close enough (or a color we don't care about, like the opposing alliance goal)
int detectGoal() {
    if (optical.get_proximity() < GOAL_PROXIMITY_THRESHOLD) {
        return -1;
    }

    double hue = optical.get_hue();

    bool isAllianceColor = ALLIANCE_IS_RED
        ? hueInRange(hue, RED_HUE_MIN, RED_HUE_MAX)
        : hueInRange(hue, BLUE_HUE_MIN, BLUE_HUE_MAX);

    if (isAllianceColor) {
        return GOAL_ALLIANCE;
    }

    if (hueInRange(hue, NEUTRAL_HUE_MIN, NEUTRAL_HUE_MAX)) {
        return GOAL_NEUTRAL;
    }

    return -1;
}

// Height (degrees) for a goal and stack position (count is 1-5)
double getStackHeight(GoalType goal, int count) {
    const double* list = (goal == GOAL_ALLIANCE) ? ALLIANCE_HEIGHTS : NEUTRAL_HEIGHTS;
    double height = list[count - 1];

    // PID is off in the bottom deadzone, so keep targets above it
    if (height <= DR4B_BOTTOM_DEADZONE) {
        height = DR4B_BOTTOM_DEADZONE + 1.0;
    }

    return height;
}

// Called every loop: switches the active goal when the sensor confirms a different one.
// Runs even with auto height off so the last goal is always remembered.
void updateGoalDetection() {
    int detected = detectGoal();

    // Seeing the goal we're already on: nothing to switch to
    if (detected == static_cast<int>(currentGoal)) {
        goalCandidate = -1;
        goalCandidateSeen = 0;
        goalCandidateMisses = 0;
        return;
    }

    // Nothing recognised this loop: forgive a short dropout, give up after several in a row
    if (detected < 0) {
        if (goalCandidate >= 0) {
            goalCandidateMisses++;
            if (goalCandidateMisses > GOAL_MISS_TOLERANCE) {
                goalCandidate = -1;
                goalCandidateSeen = 0;
                goalCandidateMisses = 0;
            }
        }
        return;
    }

    if (detected == goalCandidate) {
        goalCandidateSeen++;
        goalCandidateMisses = 0;
    } else {
        goalCandidate = detected;
        goalCandidateSeen = 1;
        goalCandidateMisses = 0;
    }

    if (goalCandidateSeen < GOAL_CONFIRM_CYCLES) {
        return;
    }

    // Goal change confirmed
    GoalType oldGoal = currentGoal;
    currentGoal = static_cast<GoalType>(detected);
    goalCandidate = -1;
    goalCandidateSeen = 0;
    goalCandidateMisses = 0;

    if (autoHeightEnabled) {
        // If the lift was still going up for the old goal, that press was a misfire
        // (we were at a different goal), so give the old goal its count back.
        if (autoLiftInProgress
            && lastPressGoal == oldGoal
            && std::fabs(dr4bTarget - getDR4BPosition()) > AUTO_ARRIVE_TOLERANCE) {
            stackCount[oldGoal] = lastPressPrevCount;
        }

        autoLiftInProgress = false;
        controller.rumble(GOAL_CHANGE_RUMBLE);
    }
}

// L1 press in auto mode: advance this goal's counter and send the lift to that height.
// After pin/cup 5 the next press wraps back to pin/cup 1 of the same goal.
void autoLiftPress() {
    GoalType goal = currentGoal;

    lastPressGoal = goal;
    lastPressPrevCount = stackCount[goal];

    stackCount[goal]++;
    if (stackCount[goal] > NUM_STACK_HEIGHTS) {
        stackCount[goal] = 1;
    }

    dr4bTarget = getStackHeight(goal, stackCount[goal]);
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    autoLiftInProgress = true;
}

// Initialize
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // Optical sensor LED
    optical.set_led_pwm(OPTICAL_LED_BRIGHTNESS);

    // Make the lift rotation sensor update every 5 ms (default is 20 ms) so the PID
    // gets fresh readings every loop and its damping (D term) isn't fed jumpy data
    dr4bRotation.set_data_rate(5);

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
            {
                lemlib::Pose pose = chassis.getPose();
                pros::lcd::print(0, "X:%.1f Y:%.1f Th:%.0f", pose.x, pose.y, pose.theta);
            }
            // Claw speed (for tuning the pin stall values) and how many times pin pickup / zeroing have fired
            pros::lcd::print(1, "Claw V:%.0f  Pin:%d", claw.get_actual_velocity(), pinLiftCount);
            pros::lcd::print(2, "Zeroed:%d Roll:%.1f", liftZeroCount, imu.get_roll());
            pros::lcd::print(3, "Lift: %.2f", getDR4BPosition());
            pros::lcd::print(4, "Hold: %.2f", dr4bTarget);
            pros::lcd::print(5, "Hue: %.0f  Prox: %d", optical.get_hue(), static_cast<int>(optical.get_proximity()));
            {
                // See: what the optical sensor sees right now (A = alliance, N = neutral, - = nothing/other)
                int seen = detectGoal();
                pros::lcd::print(6, "Cnt A:%d N:%d See:%c Cf:%d", stackCount[GOAL_ALLIANCE], stackCount[GOAL_NEUTRAL],
                                 seen == GOAL_ALLIANCE ? 'A' : (seen == GOAL_NEUTRAL ? 'N' : '-'),
                                 goalCandidateSeen);
            }
            pros::lcd::print(7, "%s  Auto:%s",
                             currentGoal == GOAL_ALLIANCE ? "ALLIANCE" : "NEUTRAL",
                             autoHeightEnabled ? "ON" : "OFF");

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

// =====================================================================
//              AUTON: 2 PINS ON THE ALLIANCE GOAL (tuning)
// =====================================================================
// Field coordinates are in inches, with (0, 0) at the center of the field.
// +X = toward the right wall (the wall the robot starts on), +Y = toward the top of the field map.
// Headings: 0 = facing the top of the map, 90 = facing right, 180 = facing down, 270 = facing left.
// One tile = 24 inches.

// Start: back of the robot against the right wall, in the middle of the wall, claw facing into the field.
// The robot is 18" long, so its center is 9" off the wall (wall is at X = 72).
constexpr double AUTON_START_X = 62.0;
constexpr double AUTON_START_Y = 0;
constexpr double AUTON_START_HEADING = 270.0;

// Alliance goal (the one being scored on)
constexpr double AUTON_GOAL_X = 48.0;
constexpr double AUTON_GOAL_Y = 24.0;

// Pin + cup 1: one tile left of the goal
constexpr double AUTON_PIN1_X = 24.0;
constexpr double AUTON_PIN1_Y = 24.0;
constexpr double AUTON_PIN1_APPROACH_HEADING = 315.0;  // drives in from the lower right (stays clear of the small pin below it)

// Pin + cup 2: one tile above the goal
constexpr double AUTON_PIN2_X = 48.0;
constexpr double AUTON_PIN2_Y = 48.0;
constexpr double AUTON_PIN2_APPROACH_HEADING = 45.0;   // drives in along the diagonal line from the lower left (stays clear of the goal)

// Lift
constexpr double AUTON_LOW_HEIGHT = 30.0;       // degrees: lift height for driving and grabbing. It drops here after the start lift and after each score
constexpr double AUTON_LIFT_TOLERANCE = 5.0;    // degrees: the lift counts as "there" when this close
// Grounding: while driving to each pin the lift is pushed all the way down to 0 (the PID alone stops
// around 10 because of its bottom deadzone). It pushes harder until it reaches the bottom, then holds lightly.
constexpr int AUTON_GROUND_PUSH_MV = 4000;      // mV pushing down until it reaches the bottom
constexpr int AUTON_GROUND_HOLD_MV = 1200;      // mV holding it down once it's there (low so the motors don't heat up)
constexpr double AUTON_GROUND_DONE_DEG = 1.0;   // counts as at the bottom at or below this

// Claw / grabbing
constexpr double AUTON_CLAW_REACH = 8.0;        // inches from the robot's center (middle of the drive wheels) to the pin when the claw closes on it. TUNE THIS FIRST
constexpr double AUTON_CLAW_OPEN_DIST = 5.0;    // starts spinning the claw when this many inches from the grab spot
constexpr int AUTON_APPROACH_CLAW_SPEED = 200;  // rpm the claw spins at while driving into a pin (keeps spinning, never stops to open).
                                                // + = same way as R1 (closing), - = same way as R2 (opening)
constexpr double AUTON_SLOW_DIST = 3.0;         // the last this-many inches to the pin are driven slowly
constexpr int AUTON_SLOW_SPEED = 30;            // speed driving straight into a pin to pick it up (out of 127) (was 40)
constexpr int AUTON_CLAW_HOLD_SPEED = 60;       // slow grabbing speed (same as driver control)
constexpr int AUTON_CLAW_OPEN_MS = 300;         // how long the claw spins backward to open
constexpr int AUTON_CLAW_FAST_MS = 1000;        // claw squeezes at full speed this long after closing, then drops to the slow speed
constexpr int AUTON_GRAB_TIMEOUT_MS = 300;      // if no claw stall is seen by then, it does the small lift anyway (was 400)

// Scoring
constexpr double AUTON_PRE_SCORE_GAP = 4.0;     // stops this many inches short of the goal and waits for the lift before going over it
constexpr int AUTON_SCORE_SPEED = 80;           // speed for those last inches over the goal (out of 127) (was 50)
// Pins only (the preload keeps the two settings above):
constexpr double AUTON_PIN_PRE_SCORE_GAP = 2.0; // inches of slower driving at the end when scoring a pin (was 4)
constexpr int AUTON_PIN_SCORE_SPEED = 110;      // speed for that last bit over the goal when scoring a pin (was 80)
constexpr double AUTON_BACKUP_DIST = 4.0;       // backs up this far once the lift is halfway down
constexpr double AUTON_CLEAR_DIST = 6.0;        // then backs up this much more so the robot's corners don't hit the goal when it turns (was 4, +2" more)

// Start: toggle mechanism (back of the robot)
constexpr double AUTON_TOGGLE_LIFT_HEIGHT = 30.0;   // degrees: lift goes up this much first so the toggles clear
constexpr double AUTON_TOGGLE_ROTATIONS = 2.0;      // full turns of the toggle (output side). Never goes past this
constexpr double AUTON_TOGGLE_GEAR_RATIO = 2.0;     // motor turns per toggle turn (1:2 = motor spins 2x for 1 toggle turn)
constexpr int AUTON_TOGGLE_SPEED = 200;             // motor rpm (same direction as holding A in driver control)
constexpr int AUTON_TOGGLE_TIMEOUT_MS = 2500;
constexpr double AUTON_TOGGLE_START_LIFT = 15.0;    // degrees: toggle starts spinning once the lift is this high (doesn't wait for the full lift)
constexpr double AUTON_TOGGLE_DONE_DEG = 90.0;      // motor degrees: robot starts moving when the toggle is this close to done.
                                                    // The toggle still finishes its 2 turns exactly on its own and stops there.       // stops trying after this, in case it gets stuck

// Preload (scored first, right after the toggles)
constexpr double AUTON_PRELOAD_CARRY_HEIGHT = 100.0; // degrees: lift rises to this (from 30) while driving to the goal with the preload
constexpr double AUTON_PRELOAD_DROP_HEIGHT = 10.0;  // degrees: lift lowers to this over the goal before the claw opens
constexpr int AUTON_PRELOAD_CLAW_OPEN_MS = 500;     // claw opens longer for the preload than for pins (AUTON_CLAW_OPEN_MS)
// Turns this many degrees LESS than straight at the goal for the preload, then drives straight.
// Raise it if it still turns too far, lower it (or go negative) if it doesn't turn enough.
constexpr double AUTON_PRELOAD_EXTRA_DIST = -2.0;      // inches. + = drives further toward the goal, - = stops shorter (was -3, now 1" further)
constexpr double AUTON_PRELOAD_TURN_LESS_DEG = -4.8;   // negative = turns 3 degrees MORE than straight at the goal
constexpr double AUTON_PRELOAD_BACKUP = 10.0;       // inches it backs away from the goal after letting go (was 8, +2" more)

// Which stack spot each pin goes on (picks the height from ALLIANCE_HEIGHTS).
// The preload is already on the goal, so the pins go on spots 2 and 3.
// If the preload doesn't count as a spot in your height list, change these to 1 and 2.
constexpr int AUTON_PIN1_STACK = 2;
constexpr int AUTON_PIN2_STACK = 3;

// Per-pin tuning offsets
// GRAB_TURN_OFFSET: degrees. Turns the robot this much extra right before the slow drive into the pin.
//                   + = more to the RIGHT (clockwise), - = more to the LEFT
// GRAB_EXTRA_DIST:  inches. + = drives FURTHER forward into the pin when grabbing, - = stops shorter
// SCORE_TURN_LESS:  degrees. + = turns LESS toward the goal than straight at it, - = turns MORE (same idea as the preload)
// SCORE_EXTRA_DIST: inches. + = drives FURTHER toward the goal when scoring (it was stopping short), - = stops shorter
constexpr double AUTON_PIN1_GRAB_TURN_OFFSET = 3.0;   // was 5, turned about 2 degrees too far right
constexpr double AUTON_PIN1_GRAB_EXTRA_DIST = 2.0;    // was 5: the straight slow drive now actually reaches its spot, so 3" less
constexpr double AUTON_PIN1_SCORE_TURN_LESS = 2.0;    // was turning about 2 degrees too far
constexpr double AUTON_PIN1_SCORE_EXTRA_DIST = -1.0;  // was 3, now 4" less into the goal

constexpr double AUTON_PIN2_GRAB_TURN_OFFSET = 8.0;   // was 5, now 3 degrees more right
constexpr double AUTON_PIN1_SLOW_DIST = 6.0;          // inches before the grab spot where it switches to slow (was 3, same as pin 2 now)
constexpr double AUTON_PIN2_SLOW_DIST = 6.0;          // starts slow earlier for pin 2 (it was still knocking it over)
constexpr double AUTON_PIN2_GRAB_EXTRA_DIST = 0.0;    // was 3: 3" less, same reason as pin 1
constexpr double AUTON_PIN2_SCORE_TURN_LESS = 0.0;    // not tested yet
constexpr double AUTON_PIN2_SCORE_EXTRA_DIST = 0.0;   // not tested yet

// Speed-ups: instead of coming to a full stop between moves that go straight into each other
// (fast drive -> slow drive, turn -> drive), the first move hands off early while still rolling.
constexpr int AUTON_CHAIN_MIN_SPEED = 60;        // drive speed kept at the hand-off (out of 127) (was 40)
constexpr double AUTON_CHAIN_EXIT_DIST = 1.5;    // inches early the fast part hands off to the slow part
constexpr int AUTON_TURN_MIN_SPEED = 40;         // turn speed kept at the hand-off (was 30)
constexpr double AUTON_TURN_EXIT_DEG = 5.0;      // degrees early a turn hands off to the next drive
// true = the lift starts going up to scoring height while the robot turns toward the goal
// (it was only starting once the turn finished). false = old behavior.
constexpr bool AUTON_LIFT_DURING_TURN = true;
// false = never stops in front of the goal to wait for the lift; it keeps driving while the lift finishes.
// Set to true if the pin/cup starts hitting the goal because the lift isn't up in time.
constexpr bool AUTON_WAIT_FOR_LIFT_AT_GOAL = false;

// Aiming turns (the turn right before driving into a pin or the goal) always finish completely.
// This is the most time they're allowed. They only use it if they need it, so a turn is never
// cut off partway just because the battery is a bit low.
constexpr int AUTON_AIM_TURN_TIMEOUT_MS = 1500;
// Aiming turns count as done once they're this close (degrees), instead of waiting to settle
// perfectly. Waiting for the last degree or two was eating most of the 15 seconds.
// Lower = more exact but slower. Raise a little if turns still hang at the end.
constexpr double AUTON_AIM_EXIT_DEG = 1.5;
constexpr int AUTON_AIM_MIN_SPEED = 10;
// Same idea for drives that end in a stop (final drive into a pin/goal, backing up): done once this close (inches)
constexpr double AUTON_STOP_EXIT_DIST = 0.5;
constexpr int AUTON_STOP_MIN_SPEED = 10;
constexpr double AUTON_BACKUP_EXIT_DIST = 1.0;   // backing up doesn't need to be exact
// =====================================================================

// Lift grounding state (see AUTON_GROUND_PUSH_MV)
bool autonLiftGrounding = false;
uint32_t autonGroundStartMs = 0;

// Auton claw timers (0 = nothing scheduled)
uint32_t autonClawSlowAtMs = 0;   // when the claw drops from full speed to the slow grabbing speed
uint32_t autonClawStopAtMs = 0;   // when the claw stops spinning backward after opening
bool autonClawOpen = false;

// Runs the claw timers. Called every 10 ms while auton waits on anything.
void autonTick() {
    uint32_t now = pros::millis();

    if (autonClawSlowAtMs != 0 && now >= autonClawSlowAtMs) {
        claw.move_velocity(AUTON_CLAW_HOLD_SPEED);
        autonClawSlowAtMs = 0;
    }

    if (autonClawStopAtMs != 0 && now >= autonClawStopAtMs) {
        claw.move_velocity(0);
        autonClawStopAtMs = 0;
    }

    // Lift grounding: push down hard until at the bottom (or stopped on the hard stop), then hold lightly
    if (autonLiftGrounding) {
        double pos = getDR4BPosition();
        double speed = std::fabs(dr4bRotation.get_velocity() / 100.0);   // degrees per second
        bool atBottom = pos <= AUTON_GROUND_DONE_DEG
                        || (pos < DR4B_BOTTOM_DEADZONE && now - autonGroundStartMs > 250 && speed < 20.0);
        DR4B.move_voltage(atBottom ? -AUTON_GROUND_HOLD_MV : -AUTON_GROUND_PUSH_MV);
    }
}

// Starts pushing the lift down to 0. Keeps going in the background (autonTick) until the claw grabs.
void autonStartGrounding() {
    dr4bSettling = false;
    dr4bManualMode = true;   // keeps the lift PID out of the way
    dr4bTargetForced = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DR4B.move_voltage(-AUTON_GROUND_PUSH_MV);
    autonGroundStartMs = pros::millis();
    autonLiftGrounding = true;
}

// Waits for the current drive motion to finish
void autonWaitMotion() {
    while (chassis.isInMotion()) {
        autonTick();
        pros::delay(10);
    }
}

// Sends the lift to a height (the DR4B PID task does the moving)
void autonSetLift(double degrees) {
    dr4bTarget = degrees;
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

// Waits until the lift reaches a height (or the timeout runs out)
void autonWaitLift(double target, uint32_t timeoutMs) {
    uint32_t start = pros::millis();
    while (std::fabs(getDR4BPosition() - target) > AUTON_LIFT_TOLERANCE
           && pros::millis() - start < timeoutMs) {
        autonTick();
        pros::delay(10);
    }
}

// Opens the claw (spins backward for openMs, then stops). Doesn't wait.
void autonOpenClaw(int openMs = AUTON_CLAW_OPEN_MS) {
    autonClawSlowAtMs = 0;
    claw.move_velocity(-200);
    autonClawStopAtMs = pros::millis() + openMs;
    autonClawOpen = true;
}

// Closes the claw at full speed, waits for the stall (it has the pin), then does the
// same small lift as the driver pin pickup. The robot is stopped the whole time.
void autonCloseClawAndLift() {
    uint32_t start = pros::millis();

    autonClawStopAtMs = 0;
    claw.move_velocity(200);
    autonClawOpen = false;
    autonClawSlowAtMs = start + AUTON_CLAW_FAST_MS;   // drops to the slow speed 1 s from now (during the turn)

    // Same stall check as driver control
    bool spunUp = false;
    bool slow = false;
    uint32_t slowStart = 0;

    while (pros::millis() - start < AUTON_GRAB_TIMEOUT_MS) {
        uint32_t now = pros::millis();
        double clawSpeed = std::fabs(claw.get_actual_velocity());

        if (clawSpeed >= CLAW_SPINUP_SPEED || now - start >= CLAW_STALL_GRACE_MS) {
            spunUp = true;
        }

        if (spunUp && clawSpeed < CLAW_STALL_SPEED) {
            if (!slow) {
                slow = true;
                slowStart = now;
            }
            if (now - slowStart >= CLAW_STALL_CONFIRM_MS) {
                break;   // grabbed
            }
        } else {
            slow = false;
        }

        autonTick();
        pros::delay(10);
    }

    // Stop holding the lift down; the small lift takes over from here
    autonLiftGrounding = false;

    // Small lift: same height rule as the driver pin pickup (PIN_LIFT_HEIGHT / PIN_LIFT_MIN_RISE)
    double liftGoal = std::max(PIN_LIFT_HEIGHT, getDR4BPosition() + PIN_LIFT_MIN_RISE);
    pinLiftCount++;

    dr4bSettling = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.move_voltage(PIN_LIFT_VOLTAGE);

    uint32_t liftStart = pros::millis();
    while (getDR4BPosition() < liftGoal && pros::millis() - liftStart < PIN_LIFT_TIMEOUT_MS) {
        autonTick();
        pros::delay(10);
    }

    dr4bTarget = getDR4BPosition();
    dr4bTargetForced = true;   // hold it here even if it's inside the bottom deadzone
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.brake();
    dr4bManualMode = false;
}

// Drives straight forward along `heading` with both sides at the same speed (no turning at all)
// until it has gone far enough to reach (targetX, targetY), then stops.
void autonDriveStraight(double targetX, double targetY, double heading, int speed, uint32_t timeoutMs) {
    lemlib::Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    double total = (targetX - start.x) * hx + (targetY - start.y) * hy;   // distance to go along the heading

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        lemlib::Pose p = chassis.getPose();
        double done = (p.x - start.x) * hx + (p.y - start.y) * hy;
        if (done >= total - 0.25) break;
        chassis.tank(speed, speed, true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

// Drives to a pin + cup, coming in along approachHeading, and grabs it
// turnOffset: + = turns that many degrees more right before the slow drive in, - = more left
// extraDist:  + = drives that many inches further forward into the pin, - = stops shorter
void autonGrabPin(double pinX, double pinY, double approachHeading, double turnOffset, double extraDist,
                  double slowDist) {
    double rad = approachHeading * PI / 180.0;
    double ux = std::sin(rad);
    double uy = std::cos(rad);

    // Where the slow part starts (same spot as before, lined up with the pin)
    double slowX = pinX - (AUTON_CLAW_REACH + AUTON_SLOW_DIST) * ux;
    double slowY = pinY - (AUTON_CLAW_REACH + AUTON_SLOW_DIST) * uy;

    // Heading for the slow drive in, with the turn offset added
    double grabHeading = approachHeading + turnOffset;
    double gx = std::sin(grabHeading * PI / 180.0);
    double gy = std::cos(grabHeading * PI / 180.0);

    // Where the robot's center ends up: straight ahead from the slow spot, plus extraDist
    double grabX = slowX + (AUTON_SLOW_DIST + extraDist) * gx;
    double grabY = slowY + (AUTON_SLOW_DIST + extraDist) * gy;

    // The fast drive covers the extra distance too, so the slow part is always just the last slowDist inches
    slowX = grabX - slowDist * gx;
    slowY = grabY - slowDist * gy;

    // Lift goes all the way down to 0 while driving to the pin
    autonStartGrounding();

    // Face the approach point, then drive there and line up with the pin
    chassis.turnToPoint(slowX, slowY, 700,
                        {.minSpeed = AUTON_TURN_MIN_SPEED, .earlyExitRange = AUTON_TURN_EXIT_DEG});
    autonWaitMotion();

    // Fast drive to slowDist before the grab spot. Comes to a real stop there
    // (no rolling hand-off) so it can't coast into the pin before the slow part starts.
    chassis.moveToPose(slowX, slowY, grabHeading, 1800, {.lead = 0.3});
    // Start spinning the claw about 5" before reaching the pin (instead of opening it)
    bool clawSpinning = false;
    while (chassis.isInMotion()) {
        if (!clawSpinning
            && chassis.getPose().distance(lemlib::Pose(grabX, grabY)) <= AUTON_CLAW_OPEN_DIST) {
            autonClawStopAtMs = 0;   // cancel any leftover open/slow timers so nothing stops the spin
            autonClawSlowAtMs = 0;
            claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
            clawSpinning = true;
        }
        autonTick();
        pros::delay(10);
    }
    if (!clawSpinning) {
        autonClawStopAtMs = 0;
        autonClawSlowAtMs = 0;
        claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
    }

    // Lift has to be all the way down before going in (only waits if it isn't there yet)
    {
        uint32_t groundWait = pros::millis();
        while (getDR4BPosition() > AUTON_GROUND_DONE_DEG + 2.0 && pros::millis() - groundWait < 500) {
            autonTick();
            pros::delay(10);
        }
    }

    // Square up exactly to the grab heading (the drive above hands off early, so it may not be fully turned yet)
    chassis.turnToHeading(grabHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Last few inches: slow and dead straight (both sides same speed, no turning) with the claw spinning
    autonDriveStraight(grabX, grabY, grabHeading, AUTON_SLOW_SPEED, 1500);

    // Stopped: close, wait for the grab, small lift
    autonCloseClawAndLift();
}

// Turns to the alliance goal, lifts while driving to it, scores, then lowers and backs off
// stackPosition = which pin/cup on the goal (1 = first, 2 = second...), uses ALLIANCE_HEIGHTS
// turnLessDeg: + = turns that many degrees less than straight at the goal, - = more
// extraDist:   + = drives that many inches further toward the goal, - = stops shorter
void autonScore(int stackPosition, double turnLessDeg, double extraDist) {
    double height = getStackHeight(GOAL_ALLIANCE, stackPosition);

    if (AUTON_LIFT_DURING_TURN) {
        autonSetLift(height);   // start lifting now so it's up sooner
    }

    // Turn toward the goal, adjusted by turnLessDeg (the claw drops to the slow speed during this turn)
    lemlib::Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(AUTON_GOAL_X - startPose.x, AUTON_GOAL_Y - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    turn -= (turn >= 0 ? turnLessDeg : -turnLessDeg);
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along the aim heading
    lemlib::Pose pose = chassis.getPose();
    double dist = std::hypot(AUTON_GOAL_X - pose.x, AUTON_GOAL_Y - pose.y);
    double ux = std::sin(aimHeading * PI / 180.0);
    double uy = std::cos(aimHeading * PI / 180.0);

    // How far the robot center travels: until the claw reaches the goal, plus extraDist
    double travel = dist - AUTON_CLAW_REACH + extraDist;
    double scoreX = pose.x + travel * ux;
    double scoreY = pose.y + travel * uy;

    // Lift up while driving toward the goal
    autonSetLift(height);

    if (travel > AUTON_PIN_PRE_SCORE_GAP + 0.5) {
        // Hands off early to the slower last part
        chassis.moveToPoint(scoreX - AUTON_PIN_PRE_SCORE_GAP * ux, scoreY - AUTON_PIN_PRE_SCORE_GAP * uy, 1000,
                            {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = AUTON_CHAIN_EXIT_DIST});
        autonWaitMotion();
    }

    // No stop here anymore (see AUTON_WAIT_FOR_LIFT_AT_GOAL): the lift keeps rising while it drives in
    if (AUTON_WAIT_FOR_LIFT_AT_GOAL) {
        autonWaitLift(height, 1200);
    }

    chassis.moveToPose(scoreX, scoreY, aimHeading, 900,   // holds the aim heading
                       {.lead = 0.1, .maxSpeed = AUTON_PIN_SCORE_SPEED,
                        .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_STOP_EXIT_DIST});
    autonWaitMotion();

    // Lower the lift and open the claw at the same time
    autonSetLift(AUTON_LOW_HEIGHT);
    autonOpenClaw();

    // Halfway down: back up
    double halfway = (height + AUTON_LOW_HEIGHT) / 2.0;
    uint32_t lowerStart = pros::millis();
    while (getDR4BPosition() > halfway && pros::millis() - lowerStart < 600) {
        autonTick();
        pros::delay(10);
    }

    // Back up AUTON_BACKUP_DIST + AUTON_CLEAR_DIST in one move (it used to stop in the middle)
    double backDist = AUTON_BACKUP_DIST + AUTON_CLEAR_DIST;
    chassis.moveToPoint(scoreX - backDist * ux, scoreY - backDist * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();
}

// Drives the preload to the alliance goal, lowers it onto the goal, lets go and backs off
void autonScorePreload() {
    // Turn toward the goal, but AUTON_PRELOAD_TURN_LESS_DEG less than straight at it
    lemlib::Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(AUTON_GOAL_X - startPose.x, AUTON_GOAL_Y - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    if (std::fabs(turn) > AUTON_PRELOAD_TURN_LESS_DEG) {
        turn -= (turn > 0 ? AUTON_PRELOAD_TURN_LESS_DEG : -AUTON_PRELOAD_TURN_LESS_DEG);
    } else {
        turn = 0.0;
    }
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along that heading, stopping when the claw is as far out as the goal
    lemlib::Pose pose = chassis.getPose();
    double dist = std::hypot(AUTON_GOAL_X - pose.x, AUTON_GOAL_Y - pose.y);
    double ux = std::sin(aimHeading * PI / 180.0);
    double uy = std::cos(aimHeading * PI / 180.0);

    // Robot center at the drop spot (adjusted by AUTON_PRELOAD_EXTRA_DIST)
    double travel = dist - AUTON_CLAW_REACH + AUTON_PRELOAD_EXTRA_DIST;
    double scoreX = pose.x + travel * ux;
    double scoreY = pose.y + travel * uy;

    // Lift goes from 30 up to the carry height while driving to the goal
    autonSetLift(AUTON_PRELOAD_CARRY_HEIGHT);

    // Drive most of the way at full speed, the last few inches slower (no stop in between)
    if (travel > AUTON_PRE_SCORE_GAP + 0.5) {
        chassis.moveToPoint(scoreX - AUTON_PRE_SCORE_GAP * ux, scoreY - AUTON_PRE_SCORE_GAP * uy, 1000,
                            {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = AUTON_CHAIN_EXIT_DIST});
        autonWaitMotion();
    }

    // Lift should be at the carry height before going over the goal (only waits if it isn't there yet)
    if (AUTON_WAIT_FOR_LIFT_AT_GOAL) {
        autonWaitLift(AUTON_PRELOAD_CARRY_HEIGHT, 600);
    }

    chassis.moveToPose(scoreX, scoreY, aimHeading, 900,   // holds the aim heading
                       {.lead = 0.1, .maxSpeed = AUTON_SCORE_SPEED,
                        .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_STOP_EXIT_DIST});
    autonWaitMotion();

    // Lower onto the goal, then open the claw
    autonSetLift(AUTON_PRELOAD_DROP_HEIGHT);
    autonWaitLift(AUTON_PRELOAD_DROP_HEIGHT, 700);

    autonOpenClaw(AUTON_PRELOAD_CLAW_OPEN_MS);   // opens a little longer for the preload
    while (autonClawStopAtMs != 0) {   // wait until it has finished opening
        autonTick();
        pros::delay(10);
    }

    // Back straight away from the goal
    chassis.moveToPoint(scoreX - AUTON_PRELOAD_BACKUP * ux, scoreY - AUTON_PRELOAD_BACKUP * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();

    // Back up to grab height for the pins (happens while it drives to pin 1)
    autonSetLift(AUTON_LOW_HEIGHT);
}

// Scores the preload + 2 pins and cups on the alliance goal
void twoPinAlliance() {
    chassis.setPose(AUTON_START_X, AUTON_START_Y, AUTON_START_HEADING);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);   // driver control sets it back to coast

    autonClawSlowAtMs = 0;
    autonClawStopAtMs = 0;
    autonClawOpen = false;

    // 1. Claw at the slow grabbing speed
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);

    // 2. Lift up 20 degrees so the toggles clear, then spin the toggle exactly 2 turns.
    //    move_absolute stops the motor right at the target, so it can't go past 2 turns.
    autonSetLift(AUTON_TOGGLE_LIFT_HEIGHT);
    {
        // Start the toggle as soon as the lift is high enough, instead of waiting for the full lift
        uint32_t liftStart = pros::millis();
        while (getDR4BPosition() < AUTON_TOGGLE_START_LIFT && pros::millis() - liftStart < 1000) {
            autonTick();
            pros::delay(10);
        }
    }

    double toggleTargetDeg = AUTON_TOGGLE_ROTATIONS * AUTON_TOGGLE_GEAR_RATIO * 180.0;   // 2 * 2 * 180 = 720 motor degrees
    toggleMechanism.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    toggleMechanism.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    toggleMechanism.tare_position();
    toggleMechanism.move_absolute(toggleTargetDeg, AUTON_TOGGLE_SPEED);

    uint32_t toggleStart = pros::millis();
    while (std::fabs(toggleMechanism.get_position() - toggleTargetDeg) > AUTON_TOGGLE_DONE_DEG
           && pros::millis() - toggleStart < AUTON_TOGGLE_TIMEOUT_MS) {
        autonTick();
        pros::delay(10);
    }
    // No stop command here: move_absolute keeps going and stops itself exactly at 2 turns
    // while the robot starts turning toward the goal. (If it got stuck and timed out, stop it.)
    if (pros::millis() - toggleStart >= AUTON_TOGGLE_TIMEOUT_MS) {
        toggleMechanism.move_velocity(0);
    }

    // Lift to grab height (AUTON_LOW_HEIGHT) for the preload drive
    // autonSetLift(AUTON_LOW_HEIGHT);

    // 3. Preload -> goal (lower to 10, open claw, back off)
    autonScorePreload();

    // 4. Pin + cup 1 (one tile left of the goal) -> goal
    autonGrabPin(AUTON_PIN1_X, AUTON_PIN1_Y, AUTON_PIN1_APPROACH_HEADING,
                 AUTON_PIN1_GRAB_TURN_OFFSET, AUTON_PIN1_GRAB_EXTRA_DIST, AUTON_PIN1_SLOW_DIST);
    autonScore(AUTON_PIN1_STACK, AUTON_PIN1_SCORE_TURN_LESS, AUTON_PIN1_SCORE_EXTRA_DIST);

    // 5. Pin + cup 2 (one tile above the goal) -> goal
    autonGrabPin(AUTON_PIN2_X, AUTON_PIN2_Y, AUTON_PIN2_APPROACH_HEADING,
                 AUTON_PIN2_GRAB_TURN_OFFSET, AUTON_PIN2_GRAB_EXTRA_DIST, AUTON_PIN2_SLOW_DIST);
    autonScore(AUTON_PIN2_STACK, AUTON_PIN2_SCORE_TURN_LESS, AUTON_PIN2_SCORE_EXTRA_DIST);

    claw.move_velocity(0);
}

// Autonomous selector (only runs when field control / a competition switch starts autonomous)
void autonomous() {
    twoPinAlliance();
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

    // Match start: assume alliance goal, all counters at zero
    currentGoal = GOAL_ALLIANCE;
    stackCount[GOAL_ALLIANCE] = 0;
    stackCount[GOAL_NEUTRAL] = 0;
    autoLiftInProgress = false;
    goalCandidate = -1;
    goalCandidateSeen = 0;
    autoHeightEnabled = AUTO_HEIGHT_ON_AT_START;

    // Claw keeps spinning forward after R1 until R2 is used
    bool clawHolding = false;

    // Pin pickup state
    uint32_t r1PressStartMs = 0;
    uint32_t clawSlowStartMs = 0;
    bool clawSlow = false;
    bool clawSpunUp = false;         // true once the claw has sped up after an R1 press
    bool clawStallLatched = false;   // true once a stall was seen, so one grab only triggers once
    bool pinLiftActive = false;      // true while the lift is being driven up after a grab
    uint32_t pinLiftStartMs = 0;
    double pinLiftGoal = 0.0;

    // Lift zeroing state (only used while L2 is held)
    bool zeroPushing = false;
    uint32_t zeroPushStartMs = 0;
    bool zeroStalling = false;
    uint32_t zeroStallStartMs = 0;
    bool liftZeroedThisHold = false;

    // Tip protection state
    bool tipActive = false;
    int tipOverCount = 0;
    bool tipPushing = false;
    uint32_t tipPushStartMs = 0;
    bool tipStalling = false;
    uint32_t tipStallStartMs = 0;
    bool tipLiftDone = false;

    while (true) {

        // Drive
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        chassis.arcade(leftY, 0.9 * rightX);

        // B toggles the auto height feature.
        // Turning it back ON resets all counters, so the next L1 press goes to
        // height 1 of the last remembered goal (alliance if none yet).
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            autoHeightEnabled = !autoHeightEnabled;

            if (autoHeightEnabled) {
                stackCount[GOAL_ALLIANCE] = 0;
                stackCount[GOAL_NEUTRAL] = 0;
                autoLiftInProgress = false;
                goalCandidate = -1;
                goalCandidateSeen = 0;
            }

            controller.rumble(AUTO_TOGGLE_RUMBLE);
        }

        // Constantly watch the optical sensor for a goal change
        updateGoalDetection();

        // The auto lift press is finished once the lift reaches its target
        if (autoLiftInProgress && std::fabs(dr4bTarget - getDR4BPosition()) <= AUTO_ARRIVE_TOLERANCE) {
            autoLiftInProgress = false;
        }

        // Lift buttons
        // Auto on:  L1 = single press goes to next stack height, L2 = manual down (unchanged)
        // Auto off: L1 = manual up, L2 = manual down (original behavior)
        bool l1Held = !autoHeightEnabled && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l1Pressed = autoHeightEnabled && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // Lift zeroing only lasts while L2 is held
        if (!l2Held) {
            zeroPushing = false;
            zeroStalling = false;
            liftZeroedThisHold = false;
        }

        // Tip protection: watch the roll angle (inertial sensor)
        {
            double roll = imu.get_roll();

            if (!TIP_PROTECTION_ENABLED || !std::isfinite(roll)) {
                // Off, or the sensor isn't giving a reading (still calibrating / unplugged): do nothing
                tipOverCount = 0;
                tipActive = false;
            } else {
                if (std::fabs(roll) > TIP_ROLL_LIMIT) {
                    if (tipOverCount < 1000) tipOverCount++;
                } else {
                    tipOverCount = 0;
                }

                if (!tipActive && tipOverCount >= TIP_CONFIRM_CYCLES) {
                    tipActive = true;
                } else if (tipActive && std::fabs(roll) < TIP_ROLL_LIMIT - TIP_ROLL_HYSTERESIS) {
                    tipActive = false;
                }
            }

            if (!tipActive) {
                tipPushing = false;
                tipStalling = false;
                tipLiftDone = false;
            }
        }

        // Tipping or a lift button always cancels a pin lift in progress
        if (tipActive || l1Held || l1Pressed || l2Held) {
            pinLiftActive = false;
        }

        // DR4B manual control
        if (tipActive) {

            // Tipping: lower the lift by itself (the lift buttons are ignored until the robot levels out).
            // It stops at the bottom or when it stops moving, and never keeps pushing against the hard stop.
            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            double position = getDR4BPosition();

            if (position > DR4B_BOTTOM_DEADZONE) {
                DR4B.move_velocity(TIP_LOWER_SPEED);
                tipPushing = false;
                tipStalling = false;
            } else if (position <= TIP_LOWER_STOP_POSITION || tipLiftDone) {
                DR4B.move_voltage(0);
            } else {
                uint32_t now = pros::millis();

                if (!tipPushing) {
                    tipPushing = true;
                    tipPushStartMs = now;
                    tipStalling = false;
                }

                DR4B.move_velocity(TIP_LOWER_SLOW_SPEED);

                double liftSpeed = dr4bRotation.get_velocity() / 100.0;   // degrees per second

                if (now - tipPushStartMs >= TIP_STALL_GRACE_MS && std::fabs(liftSpeed) < TIP_STALL_SPEED) {
                    if (!tipStalling) {
                        tipStalling = true;
                        tipStallStartMs = now;
                    }

                    if (now - tipStallStartMs >= TIP_STALL_MS) {
                        // The lift is on its hard stop: set the sensor to 0 right here
                        dr4bRotation.reset_position();
                        liftZeroCount++;
                        tipLiftDone = true;
                        DR4B.move_voltage(0);
                    }
                } else {
                    tipStalling = false;
                }
            }

        } else if (l1Held) {

            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            DR4B.move_velocity(DR4B_UP_SPEED);

        } else if (l2Held) {

            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            double position = getDR4BPosition();

            if (position > DR4B_BOTTOM_DEADZONE) {
                DR4B.move_velocity(DR4B_DOWN_SPEED);
                zeroPushing = false;
                zeroStalling = false;
            } else if (!LIFT_ZERO_ON_STALL || liftZeroedThisHold) {
                DR4B.move_velocity(0);
            } else {
                // At the bottom with L2 held: keep pushing down gently, and when the lift
                // stops moving (it is on its hard stop) set the sensor to 0 right there
                uint32_t now = pros::millis();

                if (!zeroPushing) {
                    zeroPushing = true;
                    zeroPushStartMs = now;
                    zeroStalling = false;
                }

                DR4B.move_velocity(LIFT_ZERO_DRIVE_SPEED);

                double liftSpeed = dr4bRotation.get_velocity() / 100.0;   // degrees per second

                if (now - zeroPushStartMs >= LIFT_ZERO_GRACE_MS && std::fabs(liftSpeed) < LIFT_ZERO_STALL_SPEED) {
                    if (!zeroStalling) {
                        zeroStalling = true;
                        zeroStallStartMs = now;
                    }

                    if (now - zeroStallStartMs >= LIFT_ZERO_STALL_MS) {
                        dr4bRotation.reset_position();
                        liftZeroCount++;
                        liftZeroedThisHold = true;
                        DR4B.move_velocity(0);
                    }
                } else {
                    zeroStalling = false;
                }
            }

        } else if (pinLiftActive) {

            // Pin pickup: drive the lift straight up with the motors (not the PID) until it reaches
            // the goal, then lock it there. The PID is kept out of the way while this runs.
            dr4bManualMode = true;
            dr4bSettling = false;

            double position = getDR4BPosition();

            if (position >= pinLiftGoal || pros::millis() - pinLiftStartMs >= PIN_LIFT_TIMEOUT_MS) {
                pinLiftActive = false;
                dr4bTarget = position;
                dr4bTargetForced = true;   // hold it here even though it's inside the bottom deadzone
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                DR4B.brake();
                dr4bManualMode = false;
            } else {
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                DR4B.move_voltage(PIN_LIFT_VOLTAGE);   // full power, no ramp-up (was move_velocity(PIN_LIFT_SPEED))
            }

        } else {

            // Driver released L1/L2 (non-blocking so driving stays responsive)
            if (dr4bManualMode && !dr4bSettling) {
                // 1. Actively brake the motors to kill momentum
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                DR4B.move_velocity(0);

                // 2. Start the settle timer so the DR4B structure and rubber bands can come to rest
                dr4bSettling = true;
                dr4bSettleStartMs = pros::millis();

            } else if (dr4bSettling && (pros::millis() - dr4bSettleStartMs) >= DR4B_SETTLE_TIME_MS) {
                // 3. Capture exact settled position
                dr4bTarget = getDR4BPosition();
                dr4bManualMode = false;
                dr4bSettling = false;

                // 4. Return brake mode back to coast so custom PID handles holding
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            }

            // Auto stack press (overrides any settling in progress)
            if (l1Pressed) {
                autoLiftPress();
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
        // Pin pickup: R1 is held (claw driving forward) but the claw has slowed to a stall, so it has a pin.
        // If the lift is low, move it to PIN_LIFT_HEIGHT. This runs once per R1 press.
        // It watches the claw both while R1 is held AND after R1 is released (the claw keeps squeezing at
        // 60 rpm), so a quick tap of R1 still counts. Pressing R1 again or using R2 re-arms it.
        {
            bool r1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            bool r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            uint32_t now = pros::millis();

            double clawSpeed = std::fabs(claw.get_actual_velocity());

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                r1PressStartMs = now;
                clawSlow = false;
                clawStallLatched = false;
                clawSpunUp = false;
            }

            // Claw opened: the next grab can trigger again
            if (r2Held) {
                clawStallLatched = false;
            }

            // Stall checks start as soon as the claw has actually got moving (or after a short
            // grace time, in case the element is already in the claw and it never speeds up)
            if (clawSpeed >= CLAW_SPINUP_SPEED || now - r1PressStartMs >= CLAW_STALL_GRACE_MS) {
                clawSpunUp = true;
            }

            bool clawSqueezing = r1Held || (clawHolding && !r2Held);
            double stallSpeed = r1Held ? CLAW_STALL_SPEED : CLAW_HOLD_STALL_SPEED;

            if (!PIN_LIFT_ENABLED || !clawSqueezing || clawStallLatched || !clawSpunUp) {
                clawSlow = false;
            } else if (clawSpeed < stallSpeed) {
                if (!clawSlow) {
                    clawSlow = true;
                    clawSlowStartMs = now;
                }

                if (now - clawSlowStartMs >= CLAW_STALL_CONFIRM_MS) {
                    clawStallLatched = true;

                    double liftPos = getDR4BPosition();

                    // Only when the driver isn't moving the lift (the short settle after
                    // releasing L1/L2 is fine) and the lift is in the low zone
                    if ((!dr4bManualMode || dr4bSettling) && !autoLiftInProgress && !tipActive
                        && !l1Held && !l2Held
                        && liftPos >= PIN_LIFT_ZONE_MIN && liftPos <= PIN_LIFT_ZONE_MAX) {
                        pinLiftGoal = std::max(PIN_LIFT_HEIGHT, liftPos + PIN_LIFT_MIN_RISE);
                        pinLiftActive = true;
                        pinLiftStartMs = now;
                        dr4bSettling = false;
                        pinLiftCount++;

                        // Start lifting right now instead of waiting for the next loop
                        dr4bManualMode = true;
                        DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        DR4B.move_voltage(PIN_LIFT_VOLTAGE);   // full power, no ramp-up (was move_velocity(PIN_LIFT_SPEED))
                    }

                    controller.rumble(PIN_LIFT_RUMBLE);   // grab detected
                }
            } else {
                clawSlow = false;
            }
        }
        //code for the toggle mechanism, spins one way while holding A and stops when A is released
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            toggleMechanism.move_velocity(200); // Spins one way while holding A (change speed if needed)
        } else {
            toggleMechanism.move_velocity(0);   // Stops when A is released
        }
        // -----------------------------------------------
        pros::delay(20);
    }
}