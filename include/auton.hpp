#pragma once
// Shared by every autonomous file: claw/timeout settings and the movement helpers in auton.cpp.

#include "robot.hpp"

// Claw / toggle motor commands and timing. Change signs if a mechanism runs backward.
constexpr int AUTO_CLAW_OPEN_RPM = -200;                  // Same direction as driver R2 (open)
constexpr int AUTO_CLAW_GRAB_RPM = 200;                   // Same direction as driver R1 (grab)
constexpr int AUTO_CLAW_HOLD_RPM = 60;                     // Same slow clamp speed as driver hold
constexpr int AUTO_SECOND_PICKUP_GRAB_MS = 500;            // Stationary clamp duration for unsupported second pickup
constexpr int AUTO_TOGGLE_RPM = 200;                       // Same direction/speed as driver A
constexpr int AUTO_TOGGLE_SPIN_MS = 1000;                  // Requested toggle spin duration: 1 second

// Per-step timeout caps. These are not target travel times: increase a timeout if
// a correctly tuned movement aborts early; reduce it if the robot can get stuck too long.
// If a movement reaches its timeout too far from the target, the route aborts rather
// than continuing with a bad pose. AUTO_DRIVE_ARRIVE_TOLERANCE_IN and
// AUTO_TURN_ARRIVE_TOLERANCE_DEG control how close counts as successful.
constexpr int AUTO_TURN_TIMEOUT_MS = 700;
constexpr int AUTO_DRIVE_TIMEOUT_MS = 1300;
constexpr int AUTO_LIFT_TIMEOUT_MS = 1800;
constexpr double AUTO_LIFT_ARRIVE_TOLERANCE_DEG = 4.0;
constexpr double AUTO_TURN_ARRIVE_TOLERANCE_DEG = 8.0;   // Increase slightly if the IMU settles just outside tolerance
constexpr double AUTO_DRIVE_ARRIVE_TOLERANCE_IN = 3.0;   // Increase if odometry consistently stops short of its target

// Auton timer (set in autonomous() and by each routine)
extern uint32_t autoStartMs;
extern uint32_t autoBudgetMs;

// Movement helpers (auton.cpp)
bool autoTimeExpired();
int autoBoundedTimeout(int requestedMs);
bool autoDelay(int durationMs);
void autoSetLiftTarget(double heightDeg);
bool autoWaitForLift(double heightDeg, int timeoutMs);
bool autoTurnRelative(double deltaDeg);
bool autoDriveDistance(double distanceIn, bool forwards, int requestedTimeoutMs = AUTO_DRIVE_TIMEOUT_MS,
                       float maxSpeed = 127);
bool autoScoreAndRetreat(double retreatIn);

// Routines
bool runTwoPinToggleAutoRoute();   // auton_quals.cpp
bool skills();                     // auton_skills.cpp
