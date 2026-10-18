// Qualification match auton: two pins + toggle (15 s).
#include "auton.hpp"

// =====================================================================
//                 TWO-PIN + TOGGLE AUTONOMOUS TUNING
// =====================================================================
// The routine is written relative to the robot's current odometry pose, so you
// can tune distances without calculating field X/Y coordinates. Distances are
// inches; turn amounts are degrees relative to the robot's heading at that step.
//
// IMPORTANT TUNING ORDER:
//  1. Put the robot in its exact starting spot and set AUTO_START_HEADING_DEG.
//  2. Test each turn with the wheels off the ground. Positive turn = right / CW
//     in LemLib's usual heading convention. Flip the sign if your setup differs.
//  3. Tune the *_IN distances a little at a time. They are intended as starting
//     guesses only; tune them on the actual field with the robot's 17 x 16 in base.
//  4. Tune the pickup creep speed/time separately so the claw pulls the element
//     in without driving past it. Stop and re-check the route after every change.
//
// This auton uses the existing alliance heights from the auto-lift feature list:
// pin/cup 1 = 167.95 degrees, pin/cup 2 = 256.11 degrees.
constexpr uint32_t TWO_PIN_AUTO_BUDGET_MS = 15000;
constexpr double AUTO_START_X = 0.0;                // odometry origin at start
constexpr double AUTO_START_Y = 0.0;                // odometry origin at start
constexpr double AUTO_START_HEADING_DEG = 0.0;      // TUNE to the robot's actual starting heading

// Turns: tune each independently. Positive is a right turn, negative is a left turn.
constexpr double AUTO_TURN_START_TO_GOAL_DEG = 45.0;       // First turn right toward the alliance goal
constexpr double AUTO_TURN_GOAL_TO_PICKUP1_DEG = 90.0;     // Turn right toward arrow 2 / first pin + cup
constexpr double AUTO_TURN_PICKUP1_TO_GOAL_DEG = -90.0;    // Turn toward arrow 3 / back to the alliance goal
constexpr double AUTO_TURN_GOAL_TO_PICKUP2_DEG = -90.0;    // Turn toward arrow 4 / second pin + cup
constexpr double AUTO_TURN_PICKUP2_TO_GOAL_DEG = 90.0;     // Turn back to the alliance goal for pin/cup 2

// Relative travel distances. Increase/decrease each value to stop at the right spot.
constexpr double AUTO_PRELOAD_DRIVE_TO_GOAL_IN = 8.0;      // Start -> goal while holding the preload
constexpr double AUTO_PRELOAD_BACKOFF_IN = 4.0;           // Back away after releasing the preload
constexpr double AUTO_PICKUP1_OPEN_APPROACH_IN = 8.0;      // Arrow 2: drive toward pin/cup with claw open
constexpr double AUTO_PICKUP1_BACKOFF_IN = 4.0;            // Back away after grabbing first pin/cup
constexpr double AUTO_DRIVE_TO_GOAL_PIN1_IN = 12.0;        // Arrow 3: pickup 1 -> goal
constexpr double AUTO_PICKUP2_OPEN_APPROACH_IN = 10.0;     // Arrow 4: drive toward second pin/cup with claw open
constexpr double AUTO_DRIVE_TO_GOAL_PIN2_IN = 12.0;        // Second pickup -> goal
constexpr double AUTO_SCORE_BACKOFF_IN = 5.0;              // Reverse distance after each scored pin/cup

// The first pickup is collected with a short, tunable forward creep while the claw
// is at full speed. Tune both values to pull the element in without overshooting.
constexpr int AUTO_PICKUP1_CREEP_SPEED = 28;                // Chassis command, -127..127; positive = forward
constexpr int AUTO_PICKUP1_CREEP_MS = 1800;                // How long it creeps while claw closes at full speed

bool runTwoPinToggleAutoRoute() {
    // 15 s limit, and tell LemLib where the robot starts and which way it faces.
    // Tune AUTO_START_HEADING_DEG to match the direction the robot actually faces.
    autoBudgetMs = TWO_PIN_AUTO_BUDGET_MS;
    chassis.setPose(AUTO_START_X, AUTO_START_Y, AUTO_START_HEADING_DEG);

    // Match starts with the preload already clamped in the claw. Hold it gently
    // while the lift/toggle sequence runs, just like driver-control claw hold.
    claw.move_velocity(AUTO_CLAW_HOLD_RPM);

    // 1) Lift to alliance pin/cup 2 height, then return to the pin-grab lift height.
    const double alliancePin2Height = getStackHeight(GOAL_ALLIANCE, 2); // 256.11 deg from the existing list
    autoSetLiftTarget(alliancePin2Height);
    if (!autoWaitForLift(alliancePin2Height, AUTO_LIFT_TIMEOUT_MS)) return false;

    autoSetLiftTarget(PIN_LIFT_HEIGHT); // Existing claw-stall pickup height (10 deg)
    if (!autoWaitForLift(PIN_LIFT_HEIGHT, AUTO_LIFT_TIMEOUT_MS)) return false;

    // 2) Spin the toggle mechanism for the requested 1 second.
    toggleMechanism.move_velocity(AUTO_TOGGLE_RPM);
    bool toggleFinished = autoDelay(AUTO_TOGGLE_SPIN_MS);
    toggleMechanism.move_velocity(0);
    if (!toggleFinished) return false;

    // 3) Turn right to the alliance goal, drive to it, release the preload, and retreat.
    if (!autoTurnRelative(AUTO_TURN_START_TO_GOAL_DEG)) return false;
    if (!autoDriveDistance(AUTO_PRELOAD_DRIVE_TO_GOAL_IN, true)) return false;
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    autoSetLiftTarget(0.0);
    bool preloadRetreatOk = autoDriveDistance(AUTO_PRELOAD_BACKOFF_IN, false);
    claw.move_velocity(0);
    if (!preloadRetreatOk) return false;

    // 4) Arrow 2 / first pin + cup. Lower to zero while turning, open while
    // driving the approach, then clamp at full speed during the short forward creep.
    autoSetLiftTarget(0.0);
    if (!autoTurnRelative(AUTO_TURN_GOAL_TO_PICKUP1_DEG)) return false;
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    if (!autoDriveDistance(AUTO_PICKUP1_OPEN_APPROACH_IN, true)) return false;

    claw.move_velocity(AUTO_CLAW_GRAB_RPM);
    chassis.arcade(AUTO_PICKUP1_CREEP_SPEED, 0);
    bool creepFinished = autoDelay(AUTO_PICKUP1_CREEP_MS);
    chassis.arcade(0, 0);
    if (!creepFinished) return false;
    claw.move_velocity(AUTO_CLAW_HOLD_RPM);

    // Lift the first pin/cup slightly off the ground.
    // Uses the existing 10-degree claw-stall pickup height.
    autoSetLiftTarget(PIN_LIFT_HEIGHT);

    if (!autoWaitForLift(PIN_LIFT_HEIGHT, AUTO_LIFT_TIMEOUT_MS)) {
        return false;
    }
    
    // Back away from the pickup, turn back toward the goal, raise to pin/cup 1,
    // then wait for both the drivetrain move and lift target before releasing.
    if (!autoDriveDistance(AUTO_PICKUP1_BACKOFF_IN, false)) return false;
    const double alliancePin1Height = getStackHeight(GOAL_ALLIANCE, 1); // 167.95 deg from the existing list
    autoSetLiftTarget(alliancePin1Height);
    if (!autoTurnRelative(AUTO_TURN_PICKUP1_TO_GOAL_DEG)) return false;
    if (!autoDriveDistance(AUTO_DRIVE_TO_GOAL_PIN1_IN, true)) return false;
    if (!autoWaitForLift(alliancePin1Height, AUTO_LIFT_TIMEOUT_MS)) return false;
    if (!autoScoreAndRetreat(AUTO_SCORE_BACKOFF_IN)) return false;

    // 5) Arrow 4 / second pin + cup. Keep the lift at zero while turning and
    // driving with the claw open. At the pickup, stop the drivetrain first, then
    // clamp in place for 500 ms (this element is not supported by a wall/surface).
    autoSetLiftTarget(0.0);
    if (!autoTurnRelative(AUTO_TURN_GOAL_TO_PICKUP2_DEG)) return false;
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    if (!autoDriveDistance(AUTO_PICKUP2_OPEN_APPROACH_IN, true)) return false;
    chassis.arcade(0, 0);

    claw.move_velocity(AUTO_CLAW_GRAB_RPM);
    if (!autoDelay(AUTO_SECOND_PICKUP_GRAB_MS)) return false;
    claw.move_velocity(AUTO_CLAW_HOLD_RPM);

    // Lift the second pin/cup slightly off the ground.
    autoSetLiftTarget(PIN_LIFT_HEIGHT);

    if (!autoWaitForLift(PIN_LIFT_HEIGHT, AUTO_LIFT_TIMEOUT_MS)) {
        return false;
    }

    // 6) Turn back to the goal while raising to alliance pin/cup 2 height, drive
    // to the scoring position, then open + lower + retreat concurrently.
    autoSetLiftTarget(alliancePin2Height);
    if (!autoTurnRelative(AUTO_TURN_PICKUP2_TO_GOAL_DEG)) return false;
    if (!autoDriveDistance(AUTO_DRIVE_TO_GOAL_PIN2_IN, true)) return false;
    if (!autoWaitForLift(alliancePin2Height, AUTO_LIFT_TIMEOUT_MS)) return false;
    if (!autoScoreAndRetreat(AUTO_SCORE_BACKOFF_IN)) return false;

    return !autoTimeExpired();
}
