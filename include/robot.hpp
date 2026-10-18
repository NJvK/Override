#pragma once
// Shared by every file: robot devices, lift / sensor tuning, and the lift + goal-detection functions.
// Devices are created in robot.cpp; this file only tells the other files they exist.

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

// Devices (ports are set in robot.cpp)
extern pros::Controller controller;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Imu imu;
extern pros::Rotation horizontalEnc;
extern pros::Rotation verticalEnc;
extern pros::MotorGroup DR4B;
extern pros::Rotation dr4bRotation;
extern pros::Motor claw;
extern pros::Motor toggleMechanism;
extern pros::Optical optical;
extern lemlib::Chassis chassis;

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
constexpr bool AUTO_HEIGHT_ON_AT_START = true;
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
constexpr int PIN_LIFT_TIMEOUT_MS = 750;        // gives up pushing after this long (in case something blocks it)
// Minimum push (mV) the PID uses for a pin lift target (backup, the direct drive above does the real work)
constexpr double PIN_LIFT_MIN_VOLTAGE = 4000.0;
// Controller buzzes this when a grab is detected, so you can tell detection from lifting problems
constexpr const char* PIN_LIFT_RUMBLE = "..";
constexpr double CLAW_STALL_SPEED = 30.0;       // rpm: the claw counts as "slow / stalled" below this (while R1 is held)
constexpr double CLAW_HOLD_STALL_SPEED = 15.0;  // rpm: same, after R1 is released and the claw keeps squeezing at 60 rpm
// Stall checks start once the claw reaches CLAW_SPINUP_SPEED, or after CLAW_STALL_GRACE_MS at the latest
constexpr double CLAW_SPINUP_SPEED = 80.0;      // rpm: the claw counts as "got moving" above this
constexpr int CLAW_STALL_GRACE_MS = 120;        // ms: start checking anyway after this (element already in the claw)
constexpr int CLAW_STALL_CONFIRM_MS = 40;       // ms: must stay slow this long before it counts as a pin (2 loops)
                                                // raise to 60-80 if it fires when the claw hasn't grabbed anything

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

// Goal types we can detect (middle goal is the same color as neutral, so it is treated as neutral)
enum GoalType {
    GOAL_ALLIANCE = 0,
    GOAL_NEUTRAL = 1
};

// Auto stack state (defined in robot.cpp)
extern bool autoHeightEnabled;
extern GoalType currentGoal;
extern int stackCount[2];
extern bool autoLiftInProgress;
extern GoalType lastPressGoal;
extern int lastPressPrevCount;
extern int goalCandidate;
extern int goalCandidateSeen;
extern int goalCandidateMisses;
extern int pinLiftCount;
extern int liftZeroCount;

// Lift state (defined in robot.cpp)
extern double dr4bTarget;
extern bool dr4bTargetForced;
extern bool dr4bManualMode;
extern bool dr4bSettling;
extern uint32_t dr4bSettleStartMs;

// Basic constants
constexpr double PI = 3.14159265358979323846;

// Functions in robot.cpp
int deadband(int value);
double getDR4BPosition();
void holdDR4B();
int detectGoal();
double getStackHeight(GoalType goal, int count);
void updateGoalDetection();
void autoLiftPress();
