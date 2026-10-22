// RW chassis bridge: LemLib-style chassis calls for the autons, run on RW-Template. See include/rw-chassis.hpp.
#include "rw-chassis.hpp"
#include <algorithm>

RWChassis chassis;

namespace {

enum class MoveKind { none, turn, point, pose };

// The move the background motion task runs. RW's runAsync() takes a plain function, so the settings
// for the move are kept here (a new move always waits for the last one first, so they can't mix).
struct PendingMove {
    MoveKind kind = MoveKind::none;
    double x = 0, y = 0, theta = 0, lead = 0;
    int dir = 1;
    double timeoutMs = 0;
    double maxVolts = 12;
};
PendingMove pendingMove;

// The running move's target and early exit (checked in isInMotion())
MoveKind activeKind = MoveKind::none;
double activeX = 0, activeY = 0, activeTheta = 0, activeEarlyExit = 0;
bool earlyExitSent = false;

// What the drive does when it's told to stop (setBrakeMode); LemLib's default was coast
pros::motor_brake_mode_e stopMode = pros::E_MOTOR_BRAKE_COAST;

// LemLib speed (0-127) -> RW volts (0-12)
double toVolts(double speed) {
    return std::clamp(std::fabs(speed), 0.0, 127.0) / 127.0 * 12.0;
}

// -180 to 180
double wrap180(double deg) {
    return std::fmod(std::fmod(deg + 180.0, 360.0) + 360.0, 360.0) - 180.0;
}

void stopDrive() {
    stopChassis(static_cast<pros::MotorBrake>(stopMode));
}

// Runs in RW's background motion task
void runPendingMove() {
    const PendingMove move = pendingMove;
    switch (move.kind) {
        case MoveKind::turn:
            ::turnToAngle(move.theta, move.timeoutMs, true, move.maxVolts);
            break;
        case MoveKind::point:
            ::moveToPoint(move.x, move.y, move.dir, move.timeoutMs, true, move.maxVolts);
            break;
        case MoveKind::pose:
            // exit = false: RW's boomerang with exit = true stops when it's within 5" of the target,
            // so it runs to the end here and the drive is stopped right after
            ::boomerang(move.x, move.y, move.dir, move.theta, move.lead, move.timeoutMs, false, move.maxVolts);
            stopDrive();
            break;
        default:
            break;
    }
}

void launch(const PendingMove& move, double targetX, double targetY, double targetTheta, double earlyExit) {
    ::waitForMotion();   // like LemLib: a new move waits for the one before it to finish
    pendingMove = move;
    activeKind = move.kind;
    activeX = targetX;
    activeY = targetY;
    activeTheta = targetTheta;
    activeEarlyExit = earlyExit;
    earlyExitSent = false;
    ::runAsync(runPendingMove);
}

}  // namespace

Pose RWChassis::getPose() const {
    return Pose(x_pos, y_pos, getInertialHeading());
}

void RWChassis::setPose(double x, double y, double theta) {
    ::setPose(x, y, theta);
}

void RWChassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    stopMode = mode;
    left_chassis.set_brake_mode_all(mode);
    right_chassis.set_brake_mode_all(mode);
}

void RWChassis::tank(int left, int right, bool disableDriveCurve) {
    (void)disableDriveCurve;   // the autons always drive raw
    if (left == 0 && right == 0) {
        stopDrive();
    } else {
        driveChassis(left / 127.0 * 12.0, right / 127.0 * 12.0);
    }
}

void RWChassis::turnToHeading(double theta, int timeout, TurnToHeadingParams params) {
    PendingMove move;
    move.kind = MoveKind::turn;
    move.theta = theta;
    move.timeoutMs = timeout;
    move.maxVolts = toVolts(params.maxSpeed);
    launch(move, 0, 0, theta, params.earlyExitRange);
}

void RWChassis::turnToPoint(double x, double y, int timeout, TurnToHeadingParams params) {
    ::waitForMotion();   // aim from where the robot is once the last move is done
    double heading = std::atan2(x - x_pos, y - y_pos) * 180.0 / M_PI;
    turnToHeading(heading, timeout, params);
}

void RWChassis::moveToPoint(double x, double y, int timeout, MoveToPointParams params) {
    PendingMove move;
    move.kind = MoveKind::point;
    move.x = x;
    move.y = y;
    move.dir = params.forwards ? 1 : -1;
    move.timeoutMs = timeout;
    move.maxVolts = toVolts(params.maxSpeed);
    launch(move, x, y, 0, params.earlyExitRange);
}

void RWChassis::moveToPose(double x, double y, double theta, int timeout, MoveToPoseParams params) {
    PendingMove move;
    move.kind = MoveKind::pose;
    move.x = x;
    move.y = y;
    move.theta = theta;
    move.lead = std::clamp(static_cast<double>(params.lead), 0.0, 0.6);   // RW: don't set above 0.6
    move.dir = params.forwards ? 1 : -1;
    move.timeoutMs = timeout;
    move.maxVolts = toVolts(params.maxSpeed);
    launch(move, x, y, theta, params.earlyExitRange);
}

bool RWChassis::isInMotion() {
    if (!::isMotionRunning()) return false;

    // Early exit: end the move once it's within earlyExitRange of its target
    if (activeEarlyExit > 0 && !earlyExitSent) {
        Pose pose = getPose();
        bool close = activeKind == MoveKind::turn
                         ? std::fabs(wrap180(activeTheta - pose.theta)) <= activeEarlyExit
                         : std::hypot(activeX - pose.x, activeY - pose.y) <= activeEarlyExit;
        if (close) {
            ::cancelMotion();
            earlyExitSent = true;
        }
    }
    return true;
}

void RWChassis::waitUntilDone() {
    while (isInMotion()) pros::delay(10);
}

void RWChassis::cancelMotion() {
    ::cancelMotion();
}

int localizeWithWalls() {
    if (!use_distance_resets) return 0;
    int resets = 0;
    if (resetPositionFront()) resets++;
    if (resetPositionBack()) resets++;
    if (resetPositionLeft()) resets++;
    if (resetPositionRight()) resets++;
    return resets;
}