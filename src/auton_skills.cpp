// Skills auton: 5 pins from the match loader stacked on the alliance goal (60 s, red left).
#include "auton.hpp"

// =====================================================================
//          SKILLS AUTONOMOUS: 5 PINS FROM THE MATCH LOADER (RED LEFT)
// =====================================================================
constexpr uint32_t SKILLS_AUTO_BUDGET_MS = 60000;

// The robot runs 5 trips: match loader -> alliance goal, stacking one pin each trip at
// alliance heights 1, 2, 3, 4, 5 (ALLIANCE_HEIGHTS in include/robot.hpp).
//
// The two spots below are FIELD points in the odometry frame, where the starting
// spot (red left tile) is X = 0, Y = 0. +Y = the way the robot faces at the start,
// +X = to its right. Heading is in degrees, 0 = the starting direction, + = clockwise.
//
// HOW TO MEASURE THEM (no math needed):
//  1. Place the robot on its exact starting spot facing SKILLS_START_HEADING_DEG, turn it on.
//  2. Push the robot by hand to the spot where it should be lined up in front of the
//     match loader, square to it, with the open claw just short of touching the pin.
//  3. Read X / Y / Th from line 0 of the brain screen and type them in below.
//  4. Do the same for the spot lined up in front of the alliance goal, a few inches back.
// The values below are PLACEHOLDERS, not real field positions.
constexpr double SKILLS_START_X = 0.0;
constexpr double SKILLS_START_Y = 0.0;
constexpr double SKILLS_START_HEADING_DEG = 0.0;

constexpr double SKILLS_LOADER_X = -48.0;          // PLACEHOLDER: lined up in front of the match loader
constexpr double SKILLS_LOADER_Y = 0.0;            // PLACEHOLDER
constexpr double SKILLS_LOADER_HEADING_DEG = 180.0; // PLACEHOLDER: facing the match loader

constexpr double SKILLS_GOAL_X = 24.0;              // PLACEHOLDER: lined up in front of the alliance goal
constexpr double SKILLS_GOAL_Y = 24.0;             // PLACEHOLDER
constexpr double SKILLS_GOAL_HEADING_DEG = 0.0;     // PLACEHOLDER: facing the alliance goal

// --- At the match loader ---
// The claw grabs at full speed while the robot creeps gently into the loader (the loader
// holds the pin still, like pickup 1 against the wall in the two-pin auton).
constexpr int SKILLS_LOADER_CREEP_SPEED = 28;       // -127..127, positive = forward
constexpr int SKILLS_LOADER_CREEP_MS = 900;         // how long it creeps while the claw closes
constexpr double SKILLS_LOADER_BACKOFF_IN = 6.0;    // back away from the loader after grabbing

// --- At the alliance goal ---
// The lift only goes this high while driving across the field (a tall lift can tip the robot).
// It finishes rising once the robot is lined up in front of the goal.
constexpr double SKILLS_TRAVEL_LIFT_MAX_DEG = 300.0;
constexpr double SKILLS_GOAL_APPROACH_IN = 5.0;     // drive in from the lined-up spot to the scoring spot
constexpr float SKILLS_GOAL_APPROACH_SPEED = 50;    // slow so the stack isn't bumped (0-127)
constexpr int SKILLS_RELEASE_MS = 250;              // claw opens this long before backing away
constexpr double SKILLS_GOAL_BACKOFF_IN = 6.0;      // back away from the stack after each pin

// --- Timeouts / tolerances for the long drives between loader and goal ---
constexpr int SKILLS_TRAVEL_TIMEOUT_MS = 2500;
constexpr double SKILLS_TRAVEL_ARRIVE_TOLERANCE_IN = 3.0;
constexpr float SKILLS_TRAVEL_LEAD = 0.4;           // moveToPose curviness (0-1, lower = straighter)
// =====================================================================

// Drives to a field point and ends facing the given heading. Waits until done.
bool skillsMoveToPose(double x, double y, double headingDeg) {
    if (autoTimeExpired()) return false;
    int timeoutMs = autoBoundedTimeout(SKILLS_TRAVEL_TIMEOUT_MS);
    if (timeoutMs <= 0) return false;

    chassis.moveToPose(x, y, headingDeg, timeoutMs, {.lead = SKILLS_TRAVEL_LEAD}, false);
    if (autoTimeExpired()) return false;

    // If this is false, raise SKILLS_TRAVEL_TIMEOUT_MS or check the measured X / Y.
    lemlib::Pose after = chassis.getPose();
    return std::hypot(x - after.x, y - after.y) <= SKILLS_TRAVEL_ARRIVE_TOLERANCE_IN;
}

// One trip: grab a pin at the match loader and stack it at alliance height stackNumber (1-5).
bool skillsLoaderToGoalCycle(int stackNumber) {
    // Drive to the loader with the lift down and the claw open
    autoSetLiftTarget(0.0);
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    if (!skillsMoveToPose(SKILLS_LOADER_X, SKILLS_LOADER_Y, SKILLS_LOADER_HEADING_DEG)) return false;

    // Grab: claw full speed while creeping into the loader, then gentle hold
    claw.move_velocity(AUTO_CLAW_GRAB_RPM);
    chassis.arcade(SKILLS_LOADER_CREEP_SPEED, 0);
    bool creepFinished = autoDelay(SKILLS_LOADER_CREEP_MS);
    chassis.arcade(0, 0);
    if (!creepFinished) return false;
    claw.move_velocity(AUTO_CLAW_HOLD_RPM);

    // Lift the pin off the loader, then back away
    autoSetLiftTarget(PIN_LIFT_HEIGHT);
    if (!autoWaitForLift(PIN_LIFT_HEIGHT, AUTO_LIFT_TIMEOUT_MS)) return false;
    if (!autoDriveDistance(SKILLS_LOADER_BACKOFF_IN, false)) return false;

    // Start raising while crossing the field (capped so the robot doesn't drive top-heavy)
    const double stackHeight = getStackHeight(GOAL_ALLIANCE, stackNumber);
    autoSetLiftTarget(std::min(stackHeight, SKILLS_TRAVEL_LIFT_MAX_DEG));
    if (!skillsMoveToPose(SKILLS_GOAL_X, SKILLS_GOAL_Y, SKILLS_GOAL_HEADING_DEG)) return false;

    // Finish raising, then ease in over the stack
    autoSetLiftTarget(stackHeight);
    if (!autoWaitForLift(stackHeight, AUTO_LIFT_TIMEOUT_MS)) return false;
    if (!autoDriveDistance(SKILLS_GOAL_APPROACH_IN, true, AUTO_DRIVE_TIMEOUT_MS,
                                 SKILLS_GOAL_APPROACH_SPEED)) return false;

    // Release, wait for the pin to settle, back off, and only then lower the lift
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    if (!autoDelay(SKILLS_RELEASE_MS)) return false;
    bool backedOff = autoDriveDistance(SKILLS_GOAL_BACKOFF_IN, false);
    claw.move_velocity(0);
    autoSetLiftTarget(0.0);
    return backedOff;
}

// Skills autonomous: 5 pins from the match loader stacked on the alliance goal
bool skills() {
    // 60 s limit, and tell LemLib where the robot starts and which way it faces
    autoBudgetMs = SKILLS_AUTO_BUDGET_MS;
    chassis.setPose(SKILLS_START_X, SKILLS_START_Y, SKILLS_START_HEADING_DEG);

    for (int stackNumber = 1; stackNumber <= NUM_STACK_HEIGHTS; stackNumber++) {
        if (!skillsLoaderToGoalCycle(stackNumber)) return false;
    }
    return !autoTimeExpired();
}
