// Autonomous selector and the movement helpers shared by the quals and skills autons.
#include "auton.hpp"

// Which routine autonomous() runs. true = skills (60 s, auton_skills.cpp),
// false = the two-pin match auton (15 s, auton_quals.cpp).
// SET THIS BACK TO false BEFORE A MATCH.
constexpr bool AUTO_RUN_SKILLS = true;

uint32_t autoStartMs = 0;
uint32_t autoBudgetMs = 15000;   // 15 s for a match auton, 60 s for skills (each routine sets it)

bool autoTimeExpired() {
    return (pros::millis() - autoStartMs) >= autoBudgetMs;
}

int autoBoundedTimeout(int requestedMs) {
    uint32_t elapsed = pros::millis() - autoStartMs;
    if (elapsed >= autoBudgetMs) return 0;
    uint32_t remaining = autoBudgetMs - elapsed;
    return std::min(requestedMs, static_cast<int>(remaining));
}

// Delays are cut short at the autonomous time limit.
bool autoDelay(int durationMs) {
    int waited = 0;
    while (waited < durationMs && !autoTimeExpired()) {
        int slice = std::min(10, durationMs - waited);
        pros::delay(slice);
        waited += slice;
    }
    return !autoTimeExpired();
}

void autoSetLiftTarget(double heightDeg) {
    dr4bTarget = heightDeg;
    // The normal PID intentionally ignores targets in the bottom deadzone. Force
    // the 10-degree pin-pickup target to be held; a true zero is allowed to rest low.
    dr4bTargetForced = (heightDeg > 0.0 && heightDeg <= DR4B_BOTTOM_DEADZONE);
    dr4bManualMode = false;
    dr4bSettling = false;
    autoLiftInProgress = heightDeg > DR4B_BOTTOM_DEADZONE;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

bool autoWaitForLift(double heightDeg, int timeoutMs) {
    uint32_t startMs = pros::millis();
    while (!autoTimeExpired() && (pros::millis() - startMs) < static_cast<uint32_t>(timeoutMs)) {
        double position = getDR4BPosition();
        // A zero target is satisfied anywhere in the existing bottom deadzone;
        // the normal lift PID intentionally does not drive below that zone.
        bool atTarget = (heightDeg <= DR4B_BOTTOM_DEADZONE && !dr4bTargetForced)
            ? position <= DR4B_BOTTOM_DEADZONE + AUTO_LIFT_ARRIVE_TOLERANCE_DEG
            : std::fabs(heightDeg - position) <= AUTO_LIFT_ARRIVE_TOLERANCE_DEG;
        if (atTarget) {
            autoLiftInProgress = false;
            return true;
        }
        pros::delay(10);
    }
    return !autoTimeExpired() &&
        ((heightDeg <= DR4B_BOTTOM_DEADZONE && !dr4bTargetForced)
            ? getDR4BPosition() <= DR4B_BOTTOM_DEADZONE + AUTO_LIFT_ARRIVE_TOLERANCE_DEG
            : std::fabs(heightDeg - getDR4BPosition()) <= AUTO_LIFT_ARRIVE_TOLERANCE_DEG);
}

bool autoTurnRelative(double deltaDeg) {
    if (autoTimeExpired()) return false;
    int timeoutMs = autoBoundedTimeout(AUTO_TURN_TIMEOUT_MS);
    if (timeoutMs <= 0) return false;

    lemlib::Pose pose = chassis.getPose();
    double targetHeading = std::fmod(pose.theta + deltaDeg + 360.0, 360.0);
    chassis.turnToHeading(targetHeading, timeoutMs, {}, false);  // false = wait until the turn is done
    if (autoTimeExpired()) return false;
    // If this is false, raise AUTO_TURN_TIMEOUT_MS or retune LemLib's angular PID.
    double headingError = std::fabs(std::remainder(targetHeading - chassis.getPose().theta, 360.0));
    return headingError <= AUTO_TURN_ARRIVE_TOLERANCE_DEG;
}

bool autoDriveDistance(double distanceIn, bool forwards, int requestedTimeoutMs, float maxSpeed) {
    if (autoTimeExpired()) return false;
    int timeoutMs = autoBoundedTimeout(requestedTimeoutMs);
    if (timeoutMs <= 0) return false;

    // Convert a relative distance into a temporary field target using the current
    // odometry heading. This avoids hard-coded field coordinates. Reverse movement
    // targets a point behind the robot and tells LemLib to drive backward to it.
    lemlib::Pose pose = chassis.getPose();
    double headingRad = pose.theta * PI / 180.0;
    double signedDistance = forwards ? distanceIn : -distanceIn;
    double targetX = pose.x + signedDistance * std::sin(headingRad);
    double targetY = pose.y + signedDistance * std::cos(headingRad);

    // In this LemLib version the 5th argument is "async", so direction goes in the params
    // and async is false so this waits until the move is done.
    chassis.moveToPoint(targetX, targetY, timeoutMs, {.forwards = forwards, .maxSpeed = maxSpeed}, false);
    if (autoTimeExpired()) return false;
    // If this is false, raise AUTO_DRIVE_TIMEOUT_MS or tune LemLib's linear PID.
    lemlib::Pose after = chassis.getPose();
    double remainingDistance = std::hypot(targetX - after.x, targetY - after.y);
    return remainingDistance <= AUTO_DRIVE_ARRIVE_TOLERANCE_IN;
}

// Opens the claw, starts lowering the lift, and backs away all at once. The lift
// PID runs in its existing task, while the blocking drivetrain move is in progress.
bool autoScoreAndRetreat(double retreatIn) {
    claw.move_velocity(AUTO_CLAW_OPEN_RPM);
    autoSetLiftTarget(0.0);
    bool moved = autoDriveDistance(retreatIn, false, AUTO_DRIVE_TIMEOUT_MS);
    claw.move_velocity(0);
    return moved;
}

// Autonomous selector
void autonomous() {
    // Each routine sets its own time limit and starting pose (in its own file)
    autoStartMs = pros::millis();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Start from known lift/autolift state. The autonomous routine drives the lift
    // directly by target but uses the same existing PID task and alliance height list.
    currentGoal = GOAL_ALLIANCE;
    stackCount[GOAL_ALLIANCE] = 0;
    stackCount[GOAL_NEUTRAL] = 0;
    autoLiftInProgress = false;
    dr4bManualMode = false;
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bTarget = getDR4BPosition();

    if (AUTO_RUN_SKILLS) {
        skills();
    } else {
        runTwoPinToggleAutoRoute();
    }

    // Always stop open-loop mechanisms/drivetrain if the route finishes or reaches
    // the 15 s budget. Capture the lift's present position so it doesn't chase a
    // stale autonomous target when the driver period starts.
    chassis.cancelMotion();
    chassis.arcade(0, 0);
    claw.move_velocity(0);
    toggleMechanism.move_velocity(0);
    holdDR4B();
}


// Pure pursuit asset
ASSET(example_txt);

// Exit condition
void exit_condition(lemlib::Pose target, double exitDist) {
    chassis.waitUntil(fabs(chassis.getPose().distance(target)) - exitDist);
    chassis.cancelMotion();
}
