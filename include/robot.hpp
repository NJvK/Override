#pragma once
// Shared by main.cpp (robot setup + driver control) and auton.cpp (the auton):
// the devices, settings and lift functions the auton uses. The devices themselves are created
// in main.cpp; this file only tells auton.cpp they exist.

#include "main.h"            // all the PROS devices
#include "rw-chassis.hpp"     // RW-Template + the chassis the autons drive with (replaces lemlib/api.hpp)

#include <algorithm>          // std::max / std::min
#include <cmath>
#include <cstdint>

// Devices (ports are set in main.cpp; the drive, IMU and tracking wheels are in src/rw-config.cpp)
// (chassis is declared in rw-chassis.hpp)
extern pros::Motor claw;
extern pros::Motor toggleMechanism;
extern pros::MotorGroup DR4B;
extern pros::Rotation dr4bRotation;

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

// No PID correction between 0 and 10 degrees, but only when the target is also down there
// (otherwise the lift could never start moving from the bottom)
constexpr double DR4B_BOTTOM_DEADZONE = 10.0;

// Basic constants
const double PI = 3.14159265358979323846;

// Goal types we can detect (middle goal is the same color as neutral, so it is treated as neutral)
enum GoalType {
    GOAL_ALLIANCE = 0,
    GOAL_NEUTRAL = 1
};

// Lift state (in main.cpp)
extern double dr4bTarget;
extern bool dr4bTargetForced;
extern bool dr4bManualMode;
extern bool dr4bSettling;
extern int pinLiftCount;
extern double dr4bSlewDown;   // top speed (degrees/second) the lift PID lowers at right now

// Lift functions (in main.cpp)
double getDR4BPosition();
double getStackHeight(GoalType goal, int count);

// Autonomous routines. They're on the brain screen's auton selector (runPreAutonomous() in src/rw-user.cpp).
void twoPinAlliance();       // auton.cpp:       preload + 2 pins (right wall)
void mirrorAlliance();       // autonmirror.cpp: 3-pin route, right wall or top wall (AUTON_START_ON_TOP in that file)
void mirrorAllianceRight();  // autonmirror.cpp: 3-pin route from the right wall
void mirrorAllianceTop();    // autonmirror.cpp: 3-pin route from the top wall
void manyPinAlliance();      // skillsauto.cpp:  the 1-minute skills run