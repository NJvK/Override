// =====================================================================
//   AUTON CORE (see include/auton-core.hpp)
// =====================================================================
// The curve follower, the lift / claw / flipper / intake service, the AI Vision aiming, the tip guard and the timer
// that the 4-pin qual auton (src/fourpin.cpp) and the skills run (src/skills.cpp) are written with. This used to be
// the top of fourpin.cpp; it moved here so both routes drive the same way.
// =====================================================================
#include "auton-core.hpp"
#include "pad.hpp"   // the controller screen

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"   // the simulator has no AI Vision sensor
#endif

// Devices that live in main.cpp and aren't in robot.hpp
extern pros::Motor flipper;
extern pros::Motor intake;
void fieldMclPinPickedUp(double x, double y);   // src/field-mcl.cpp
int fieldAwareReset(double maxJump);            // src/field-mcl.cpp: one-shot distance sensor reset while stopped

namespace ac {

// =====================================================================
//   SMALL MATH
// =====================================================================
double rad(double deg) { return deg * FP_PI / 180.0; }
double wrap180(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}
double clampTo(double value, double low, double high) { return value < low ? low : (value > high ? high : value); }
Vec forwardOf(double headingDeg) { return {std::sin(rad(headingDeg)), std::cos(rad(headingDeg))}; }
Vec rightOf(double headingDeg) { return {std::cos(rad(headingDeg)), -std::sin(rad(headingDeg))}; }

// =====================================================================
//   ROUTE FRAME
// =====================================================================
namespace {
bool fpTurned = false;     // true = turned 180 degrees (blue)
bool fpMirrored = false;   // true = flipped across the diagonal the field pins sit on (left / right wall starts)
}

void setRouteFrame(bool turned, bool mirrored) {
    fpTurned = turned;
    fpMirrored = mirrored;
}
Vec toField(Vec p) {
    if (fpMirrored) std::swap(p.x, p.y);
    if (fpTurned) { p.x = -p.x; p.y = -p.y; }
    return p;
}
double headingToField(double heading) {
    if (fpMirrored) heading = 90.0 - heading;
    if (fpTurned) heading += 180.0;
    return heading;
}
Vec toRoute(Vec p) {
    if (fpTurned) { p.x = -p.x; p.y = -p.y; }
    if (fpMirrored) std::swap(p.x, p.y);
    return p;
}
double headingToRoute(double heading) {
    if (fpTurned) heading -= 180.0;
    if (fpMirrored) heading = 90.0 - heading;
    return heading;
}

Node node(double x, double y, double heading, double tangentIn, double tangentOut, double speed) {
    Node n;
    n.x = x; n.y = y; n.heading = heading; n.tangentIn = tangentIn; n.tangentOut = tangentOut; n.speed = speed;
    return n;
}
Node along(const Node& from, double inches, double tangentIn, double tangentOut, double speed) {
    Vec f = forwardOf(from.heading);
    return node(from.x + f.x * inches, from.y + f.y * inches, from.heading, tangentIn, tangentOut, speed);
}
Node clawOn(double x, double y, double heading, double reach) {
    Vec f = forwardOf(heading);
    return node(x - f.x * reach, y - f.y * reach, heading);
}
Node hereNode() {
    Vec p = toRoute({x_pos, y_pos});
    return node(p.x, p.y, headingToRoute(getInertialHeading()));
}

// =====================================================================
//   CLOCK
// =====================================================================
namespace {
const char* fpRouteName = "auton";
uint32_t fpStartMs = 0;
int fpLimitMs = 15000;
int fpEndReserveMs = FP_COAST_BEFORE_END_MS;   // everything stops this long before the limit
bool fpTimeUp = false;
}
int elapsed() { return static_cast<int>(pros::millis() - fpStartMs); }
int timeLeft() { return fpLimitMs - elapsed(); }
bool timeUp() { return fpTimeUp; }
void setEndReserve(int ms) { fpEndReserveMs = ms; }
namespace {
bool fpStatusOnPad = false;   // the route's time / pose / last step on the controller (coreStart turns it on;
                              // the tuning tools turn it off again)
uint32_t fpStatusLastMs = 0;
double heading360() { return std::fmod(std::fmod(getInertialHeading(), 360.0) + 360.0, 360.0); }
// Controller lines 0 and 1: route + time, and the pose
void statusToPad() {
    pad::line(0, "%-10.10s %5.1fs", fpRouteName, elapsed() / 1000.0);
    pad::line(1, "X%.1f Y%.1f H%.0f", x_pos, y_pos, heading360());
}
}  // namespace
void showStatusOnController(bool on) { fpStatusOnPad = on; }
void note(const char* what) {
    std::printf("%s %5.2f s  %-28s x %6.1f  y %6.1f  heading %5.0f  lift %4.0f\n", fpRouteName, elapsed() / 1000.0, what, x_pos, y_pos,
                heading360(), getDR4BPosition());
    if (fpStatusOnPad) pad::line(2, "%s", what);   // controller line 2: the last step
}

// =====================================================================
//   LIFT
// =====================================================================
namespace {
enum class LiftMode { idle, pid, ground, coast, direct, hop };
LiftMode fpLiftMode = LiftMode::idle;
double fpLiftGoal = 0.0;          // height the route wants (pid mode)
uint32_t fpGroundStartMs = 0;
bool fpGroundPushing = false;
uint32_t fpGroundPushStartMs = 0;
bool fpLiftPress = false;         // picking up: push down harder than the light hold
bool fpLiftPending = false;       // a pickup hop waiting for its start time
uint32_t fpLiftPendingAtMs = 0;
double fpHopThen = -1.0;          // height to go to after the hop (-1 = stay at the hop height)
double fpHopGoal = 0.0;
uint32_t fpHopStartMs = 0;
// Tip guard (the checks are in service()). While the robot is tipping the lift stays down whatever the route asks
// for; what the route asked for last is remembered and done once it's level again.
bool fpTipping = false;
int fpTipTicks = 0;
double fpTipRestoreGoal = 0.0;    // height to go back to when it's level again...
bool fpTipRestoreDown = false;    // ...unless the route has sent the lift down since (it let go and is driving away)
// Is the lift still climbing? (liftStuck: service() moves the mark up each time it has risen FP_LIFT_RISE_DEG)
uint32_t fpLiftSentMs = 0;        // when liftTo last sent it
double fpLiftRiseMark = 0.0;
uint32_t fpLiftRiseMs = 0;
}

void liftTo(double height) {
    if (fpTipping) {
        // Tipping: not now. It goes to this height when the robot is level again.
        fpLiftPending = false;
        fpTipRestoreGoal = height;
        fpTipRestoreDown = false;
        return;
    }
    fpLiftPending = false;
    fpLiftPress = false;
    fpLiftMode = LiftMode::pid;
    fpLiftGoal = height;
    fpLiftSentMs = fpLiftRiseMs = pros::millis();
    fpLiftRiseMark = getDR4BPosition();
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bTarget = height;
    dr4bManualMode = false;
}
void liftHopThen(double thenHeight) {
    fpLiftPending = true;
    fpLiftPendingAtMs = pros::millis() + FP_PICKUP_LIFT_DELAY_MS;
    fpHopThen = thenHeight;
}
namespace {
// Full power up to max(PIN_LIFT_HEIGHT, here + PIN_LIFT_MIN_RISE) like driver control, then on to fpHopThen
void startHop() {
    fpLiftPending = false;
    fpLiftPress = false;
    fpLiftMode = LiftMode::hop;
    fpHopGoal = std::max(PIN_LIFT_HEIGHT, getDR4BPosition() + PIN_LIFT_MIN_RISE);
    fpHopStartMs = pros::millis();
    dr4bManualMode = true;
    dr4bSettling = false;
    dr4bTargetForced = false;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.move_voltage(PIN_LIFT_VOLTAGE);
}
// Hop finished: on to the next height, or lock it where it is (same as driver control: forced target, so the PID
// holds it even inside the bottom deadzone)
void endHop() {
    if (fpHopThen >= 0.0) {
        liftTo(fpHopThen);
        return;
    }
    double position = getDR4BPosition();
    fpLiftMode = LiftMode::pid;
    fpLiftGoal = position;
    dr4bTarget = position;
    dr4bTargetForced = true;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.brake();
    dr4bManualMode = false;
}
}
void liftAfterHop(double height) {
    if (fpLiftPending || fpLiftMode == LiftMode::hop) fpHopThen = height;   // the hop carries on up to it
    else liftTo(height);
}
double liftSeconds(double height) { return FP_LIFT_START_S + std::max(height, 0.0) / FP_LIFT_UP_DPS; }
void liftDown() {
    fpLiftPending = false;
    if (fpTipping) fpTipRestoreDown = true;   // the route wants it down: it stays down when the robot is level again
    if (fpLiftMode == LiftMode::ground) return;
    fpLiftMode = LiftMode::ground;
    fpGroundStartMs = pros::millis();
    fpGroundPushing = false;
    dr4bManualMode = true;
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bTarget = 0.0;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}
void liftPress(bool on) { fpLiftPress = on; }
void liftCoast() {
    fpLiftPending = false;
    fpLiftPress = false;
    fpLiftMode = LiftMode::coast;
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    DR4B.move_voltage(0);
}
void liftRaiseDirect(double height, int maxMs) {
    fpLiftPending = false;
    fpLiftPress = false;
    fpLiftMode = LiftMode::direct;
    dr4bManualMode = true;
    dr4bSettling = false;
    dr4bTargetForced = false;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    uint32_t t0 = pros::millis();
    while (getDR4BPosition() < height && pros::millis() - t0 < static_cast<uint32_t>(maxMs)) {
        DR4B.move_voltage(12000);
        service();
        if (fpTimeUp) break;
        pros::delay(10);
    }
}
bool liftReady() {
    return fpLiftMode == LiftMode::pid && getDR4BPosition() >= fpLiftGoal - FP_LIFT_READY_TOLERANCE;
}
bool liftStuck() {
    if (fpLiftMode != LiftMode::pid || liftReady()) return false;
    const uint32_t now = pros::millis();
    return now - fpLiftSentMs >= static_cast<uint32_t>(FP_LIFT_STUCK_MIN_MS) && now - fpLiftRiseMs >= static_cast<uint32_t>(FP_LIFT_STUCK_MS);
}
bool liftUpOrStuck() { return liftReady() || liftStuck(); }
bool liftIsDown() { return getDR4BPosition() <= 5.0; }
bool liftClimbingPast(double deg) { return fpLiftMode == LiftMode::pid && getDR4BPosition() >= deg; }
double liftGoal() { return fpLiftGoal; }

// =====================================================================
//   CLAW
// =====================================================================
namespace {
enum class ClawMode { off, opening, closing, hold };
ClawMode fpClawMode = ClawMode::off;
uint32_t fpClawSinceMs = 0;
void clawSet(ClawMode mode) {
    if (fpClawMode == mode) return;
    if (mode == ClawMode::closing && fpClawMode == ClawMode::hold) return;       // already holding
    fpClawMode = mode;
    fpClawSinceMs = pros::millis();
}
}
void clawOpen() { clawSet(ClawMode::opening); }
void clawClose() { clawSet(ClawMode::closing); }
void clawCloseAgain() { fpClawMode = ClawMode::closing; fpClawSinceMs = pros::millis(); }
void clawOff() { clawSet(ClawMode::off); }
bool clawHolding() { return fpClawMode == ClawMode::closing || fpClawMode == ClawMode::hold; }
// The claw camera is only read with the claw open (FP_CLAW_CAM_OPEN_MS): closed, its arms fill the picture
static bool clawOpenForCamera() {
    return fpClawMode == ClawMode::opening && pros::millis() - fpClawSinceMs >= static_cast<uint32_t>(FP_CLAW_CAM_OPEN_MS);
}

// =====================================================================
//   FLIPPER + INTAKE
// =====================================================================
namespace {
enum class FlipMode { off, up, down };
FlipMode fpFlipMode = FlipMode::off;
uint32_t fpFlipSinceMs = 0;
double fpFlipCheckPos = 0.0;
uint32_t fpFlipCheckMs = 0;
bool fpIntakeOn = false;
uint32_t fpIntakeOffAtMs = 0;   // 0 = no off time set
}

// Flipping: the intake spins forward the whole time the flipper is flipping (same as holding A in driver control),
// and turns off when the flipper goes back down (same as letting go of A)
void flipperUp() {
    if (fpFlipMode == FlipMode::up) return;
    fpFlipMode = FlipMode::up;
    fpFlipSinceMs = pros::millis();
    fpIntakeOn = true;
    fpIntakeOffAtMs = 0;   // no timer: it stays on until the flipper comes back down
}
void flipperDown() {
    if (fpFlipMode != FlipMode::up) return;   // only from the flipped position
    fpFlipMode = FlipMode::down;
    fpFlipSinceMs = pros::millis();
    fpFlipCheckPos = flipper.get_position();
    fpFlipCheckMs = fpFlipSinceMs;
    fpIntakeOn = false;
}
void intakeOn() { fpIntakeOn = true; }
void intakeOffAfter(int ms) { fpIntakeOffAtMs = pros::millis() + ms; }
void intakeOff() { fpIntakeOn = false; }

// =====================================================================
//   TIP GUARD
// =====================================================================
bool tipping() { return fpTipping; }

// =====================================================================
//   AI VISION AIM
// =====================================================================
// While aiming, the curve is nudged by fpAimShift (field inches) so it ends on what the camera sees.
namespace {
AimKind fpAimKind = AimKind::none;
Vec fpAimSpot = {0, 0};      // where the pin / goal should be (field)
Vec fpAimShift = {0, 0};     // current nudge (field)
uint32_t fpAimLastMs = 0;
int fpAimTagId = -1;         // goal tag aiming: the tag it's looking for (-1 = any)
double fpAimWindow = 0.0;    // this aim only trusts a sighting within this many inches sideways (0 = the usual window)
int fpAimSightings = 0;      // camera frames that found it during this aim (for the terminal log)
/// Pixels of focal length: how far off-center something looks = FOCAL * tan(its angle)
const double FP_CAM_FOCAL_PX = (FP_CAM_WIDTH_PX / 2.0) / std::tan(rad(FP_CAM_FOV_DEG / 2.0));
uint32_t fpAimSeenMs = 0;    // when a camera last found what's being aimed at

// Where the robot was over the last 0.3 s (service() adds one every 10 ms). A camera's picture is FP_CAM_LATENCY_MS
// old, so what it shows is compared with where the robot was THEN, not where it is now.
struct PastPose { uint32_t ms; double x, y, heading; };
constexpr int FP_PAST_COUNT = 32;
PastPose fpPast[FP_PAST_COUNT];
int fpPastNext = 0;
int fpPastFilled = 0;
void rememberPose() {
    fpPast[fpPastNext] = {pros::millis(), x_pos, y_pos, getInertialHeading()};
    fpPastNext = (fpPastNext + 1) % FP_PAST_COUNT;
    if (fpPastFilled < FP_PAST_COUNT) fpPastFilled++;
}
PastPose poseAgo(int ms) {
    PastPose best = {pros::millis(), x_pos, y_pos, getInertialHeading()};
    const uint32_t want = best.ms - static_cast<uint32_t>(ms);
    for (int k = 1; k <= fpPastFilled; k++) {   // newest first: stops at the first one that old
        best = fpPast[(fpPastNext - k + FP_PAST_COUNT) % FP_PAST_COUNT];
        if (best.ms <= want) break;
    }
    return best;
}

// The two cameras
enum class Cam { claw, field };
#ifndef RW_SIM_PROS
pros::AIVision& camera(Cam cam) {
    static pros::AIVision clawCamera(FP_PIN_CAM_PORT);
    static pros::AIVision fieldCamera(FP_GOAL_CAM_PORT);
    return cam == Cam::claw ? clawCamera : fieldCamera;
}
#endif
double camYaw(Cam cam) { return cam == Cam::claw ? FP_PIN_CAM_YAW : FP_GOAL_CAM_YAW; }

// Where a camera's lens was on the field with the robot at `at`
Vec cameraSpot(Cam cam, const PastPose& at) {
    Vec f = forwardOf(at.heading), r = rightOf(at.heading);
    if (cam == Cam::field) {
        return {at.x + f.x * FP_GOAL_CAM_FORWARD + r.x * FP_GOAL_CAM_RIGHT, at.y + f.y * FP_GOAL_CAM_FORWARD + r.y * FP_GOAL_CAM_RIGHT};
    }
    return {at.x + f.x * FP_PIN_CAM_FORWARD, at.y + f.y * FP_PIN_CAM_FORWARD};
}

// What one camera sees of the thing being aimed at, as inches SIDEWAYS: how far to the right of where it should be
// it is, looking from the camera (+ = further right than odometry expects).
//   then      where the robot was when the picture was taken
//   spot      where the camera's lens was then
//   expected  the bearing the thing should have from the camera (degrees right of the ROBOT's straight ahead)
//   dist      inches from the camera to where it should be
// Pickups: CUP detections only (isCupClass), within FP_PIN_WINDOW_IN of where the cup should be. Exactly one cup there
// (boxes within FP_AIM_SAME_IN of each other are one cup); none, or two that disagree = not sure, nothing this frame.
// Goal tags: the tag with the right id that faces the camera most squarely (FP_GOAL_TAG_MIN_SQUARENESS), within
// FP_GOAL_TAG_WINDOW_IN. Only its direction is used, never its size (see the header).
// false = nothing believable in view.
bool cameraSighting(Cam cam, AimKind kind, const PastPose& then, const Vec& spot, double expected, double dist, double& sideways) {
    const bool tags = kind == AimKind::goalTag;
    const double yaw = camYaw(cam);
    const double window = fpAimWindow > 0.0 ? fpAimWindow : (tags ? FP_GOAL_TAG_WINDOW_IN : FP_PIN_WINDOW_IN);
    const double reach = tags ? std::max(dist - FP_GOAL_TAG_TO_CENTER, 1.0) : dist;   // the tag is on the goal's near side
#ifdef RW_SIM_PROS
    // Simulator: there's no camera. SIM_PIN_SHIFT="dx,dy" / SIM_GOAL_SHIFT="dx,dy" pretend every pin / goal sits
    // that far (route inches) from its spot and that the camera sees it perfectly, to test the lining up.
    const char* text = std::getenv(kind == AimKind::pin ? "SIM_PIN_SHIFT" : "SIM_GOAL_SHIFT");
    double dx = 0, dy = 0;
    if (kind == AimKind::none || text == nullptr || std::sscanf(text, "%lf,%lf", &dx, &dy) != 2) return false;
    Vec moved = toField({dx, dy});
    double tx = fpAimSpot.x + moved.x - spot.x, ty = fpAimSpot.y + moved.y - spot.y;
    double seen = wrap180(std::atan2(tx, ty) * 180.0 / FP_PI - then.heading);
    if (std::fabs(seen - yaw) > FP_CAM_FOV_DEG / 2.0) return false;   // outside the picture
    if (!tags && cam == Cam::claw && std::fabs(seen) > FP_AIM_AHEAD_DEG) return false;   // claw camera: nearly straight ahead
    if (std::fabs(seen - expected) > 80.0) return false;
    sideways = reach * std::tan(rad(seen - expected));
    return std::fabs(sideways) <= window;
#else
    (void)then;
    (void)spot;
    int32_t count = camera(cam).get_object_count();
    if (count <= 0 || count > 64) return false;   // nothing, or the sensor isn't answering (PROS_ERR)
    bool found = false;
    double bestRank = -1e9;
    double cupLow = 1e9, cupHigh = -1e9, cupSum = 0;   // pickups: the cups in the window
    int cups = 0;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera(cam).get_object(i);
        double centerPx = 0, rank = 0;
        if (tags) {
            if (!(seen.type & pros::E_AIVISION_DETECTED_TAG)) continue;
            if (fpAimTagId >= 0 && seen.id != fpAimTagId) continue;
            const auto& t = seen.object.tag;
            centerPx = (t.x0 + t.x1 + t.x2 + t.x3) / 4.0;
            // The four edges, as the pair that runs across the picture and the pair that runs up it
            const double e01 = std::hypot(t.x1 - t.x0, t.y1 - t.y0), e12 = std::hypot(t.x2 - t.x1, t.y2 - t.y1);
            const double e23 = std::hypot(t.x3 - t.x2, t.y3 - t.y2), e30 = std::hypot(t.x0 - t.x3, t.y0 - t.y3);
            const bool firstPairAcross = std::abs(t.x1 - t.x0) + std::abs(t.x3 - t.x2) >= std::abs(t.y1 - t.y0) + std::abs(t.y3 - t.y2);
            const double wide = firstPairAcross ? (e01 + e23) / 2.0 : (e12 + e30) / 2.0;
            const double tall = firstPairAcross ? (e12 + e30) / 2.0 : (e01 + e23) / 2.0;
            if (tall < FP_GOAL_TAG_MIN_PX) continue;
            // A tag turned away from the camera looks narrower than it is tall, and it sits off to one side of the
            // goal's center: only a tag that faces the camera is lined up on
            const double squareness = std::min(wide / tall, 1.0);
            if (squareness < FP_GOAL_TAG_MIN_SQUARENESS) continue;
            rank = squareness;   // the one that faces the camera most
        } else {
            if (!(seen.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
            if (!isCupClass(seen.id)) continue;   // cups only: the model finds pins too unreliably
            if (seen.object.element.score < AI_VISION_MIN_SCORE) continue;   // too unsure (include/robot.hpp)
            // Claw camera: the gears fill the bottom of the picture. A box centered down there is a gear / the claw.
            if (cam == Cam::claw && seen.object.element.yoffset + seen.object.element.height / 2 >= FP_CLAW_CAM_MAX_CENTER_Y) continue;
            centerPx = seen.object.element.xoffset + seen.object.element.width / 2.0;
        }
        if (cam == Cam::field ? FP_GOAL_CAM_UPSIDE_DOWN : FP_PIN_CAM_UPSIDE_DOWN) centerPx = FP_CAM_WIDTH_PX - centerPx;
        double tanAcross = (centerPx - FP_CAM_WIDTH_PX / 2.0) / FP_CAM_FOCAL_PX;   // tan of the angle in the camera's picture
        if (cam == Cam::claw) tanAcross /= std::cos(rad(FP_PIN_CAM_TILT_DEG));     // the claw camera looks down: undo the squash
        const double seenBearing = std::atan(tanAcross) * 180.0 / FP_PI + yaw;
        if (!tags && cam == Cam::claw && std::fabs(seenBearing) > FP_AIM_AHEAD_DEG) continue;   // claw camera: nearly straight ahead
        if (std::fabs(seenBearing - expected) > 80.0) continue;
        const double off = reach * std::tan(rad(seenBearing - expected));
        if (std::fabs(off) > window) continue;   // not where the thing should be: something else
        if (!tags) {
            cupLow = std::min(cupLow, off);
            cupHigh = std::max(cupHigh, off);
            cupSum += off;
            cups++;
            continue;
        }
        if (rank > bestRank) {
            bestRank = rank;
            sideways = off;
            found = true;
        }
    }
    if (!tags) {
        if (cups == 0 || cupHigh - cupLow > FP_AIM_SAME_IN) return false;   // none, or not sure which: skip this frame
        sideways = cupSum / cups;
        return true;
    }
    return found;
#endif
}

// One camera's look at the thing being aimed at. true = it saw it, and the nudge was updated from it.
bool aimWith(Cam cam) {
    const bool tag = fpAimKind == AimKind::goalTag;
    if (std::fabs(odom_turn_rate_dps) > FP_AIM_MAX_TURN_DPS) return false;   // spinning too fast for a sharp picture
    if (cam == Cam::claw && !clawOpenForCamera()) return false;              // claw not open: its arms are in the picture
    const PastPose then = poseAgo(FP_CAM_LATENCY_MS);
    const Vec spot = cameraSpot(cam, then);
    const double dx = fpAimSpot.x - spot.x, dy = fpAimSpot.y - spot.y;
    const double dist = std::hypot(dx, dy);
    const double maxDist = tag ? FP_GOAL_AIM_MAX_DIST : (cam == Cam::field ? FP_FIELD_PIN_MAX_DIST : FP_AIM_MAX_DIST);
    if (dist > maxDist || dist < (tag ? FP_GOAL_AIM_MIN_DIST : FP_AIM_MIN_DIST)) return false;
    const double expected = wrap180(std::atan2(dx, dy) * 180.0 / FP_PI - then.heading);
    if (std::fabs(expected - camYaw(cam)) > FP_CAM_FOV_DEG / 2.0 + 8.0) return false;   // nowhere near this camera's picture
    double sideways = 0;
    if (!cameraSighting(cam, fpAimKind, then, spot, expected, dist, sideways)) return false;
    fpAimSightings++;
    fpAimSeenMs = pros::millis();

    // The thing is `sideways` inches to the right of where it should be, looking from the camera: the curve is moved
    // that way, ACROSS the line of sight (not along the robot's own side: in a curve those two aren't the same)
    const double most = tag ? FP_GOAL_MAX_SHIFT : FP_AIM_MAX_SHIFT;
    sideways = clampTo(sideways, -most, most);
    const Vec across = rightOf(then.heading + expected);
    const double smooth = tag ? FP_GOAL_AIM_SMOOTH : FP_AIM_SMOOTH;
    Vec step = {smooth * (across.x * sideways - fpAimShift.x), smooth * (across.y * sideways - fpAimShift.y)};
    const double size = std::hypot(step.x, step.y), mostStep = FP_AIM_SLEW_IPS * FP_AIM_PERIOD_MS / 1000.0;
    if (size > mostStep) {
        step.x *= mostStep / size;
        step.y *= mostStep / size;
    }
    fpAimShift.x += step.x;
    fpAimShift.y += step.y;
    return true;
}

void aimUpdate() {
    uint32_t now = pros::millis();
    if (fpAimKind == AimKind::none) {
        // Not aiming: the nudge fades away (steadily, no jump) so the rest of the curve is back on its drawn line
        const double size = std::hypot(fpAimShift.x, fpAimShift.y), fade = FP_AIM_FADE_IPS * 0.010;
        const double keep = size > fade ? (size - fade) / size : 0.0;
        fpAimShift.x *= keep;
        fpAimShift.y *= keep;
        return;
    }
    if (now - fpAimLastMs < static_cast<uint32_t>(FP_AIM_PERIOD_MS)) return;
    fpAimLastMs = now;
    if (fpAimKind == AimKind::goalTag) {
        if (FP_USE_GOAL_TAG) aimWith(Cam::field);
        return;
    }
    // A pin: the claw camera first (it lines the pin up with the claw), the field camera if it has nothing
    if (FP_USE_PIN_VISION && aimWith(Cam::claw)) return;
    if (FP_FIELD_CAM_SEES_PINS) aimWith(Cam::field);
}
}  // namespace

void visionStart() {
#ifndef RW_SIM_PROS
    if (FP_USE_PIN_VISION) camera(Cam::claw).enable_detection_types(pros::AivisionModeType::objects);
    if (FP_FIELD_CAM_SEES_PINS) camera(Cam::field).enable_detection_types(pros::AivisionModeType::objects, pros::AivisionModeType::tags);
    else if (FP_USE_GOAL_TAG) camera(Cam::field).enable_detection_types(pros::AivisionModeType::tags);
#endif
}
void aimAt(AimKind kind, double routeX, double routeY, int tagId, double windowIn) {
    fpAimKind = kind;
    fpAimWindow = windowIn;
    fpAimSpot = toField({routeX, routeY});
    fpAimLastMs = 0;
    fpAimTagId = tagId;
    fpAimSightings = 0;
    // Carrying a pin's nudge on into a goal approach: never more than a goal's own nudge is allowed to be
    const double size = std::hypot(fpAimShift.x, fpAimShift.y);
    if (kind == AimKind::goalTag && size > FP_GOAL_MAX_SHIFT) {
        fpAimShift.x *= FP_GOAL_MAX_SHIFT / size;
        fpAimShift.y *= FP_GOAL_MAX_SHIFT / size;
    }
}
void aimStop() {
    if (fpAimKind == AimKind::pin) {
        std::printf("%s %5.2f s  pin camera: seen %d times, curve moved %.1f in (x %+.1f  y %+.1f)\n", fpRouteName,
                    elapsed() / 1000.0, fpAimSightings, std::hypot(fpAimShift.x, fpAimShift.y), fpAimShift.x, fpAimShift.y);
    }
    if (fpAimKind == AimKind::goalTag) {
        if (fpAimSightings > 0) {
            std::printf("%s %5.2f s  goal tag: seen %d times, approach moved %.1f in (x %+.1f  y %+.1f)\n", fpRouteName,
                        elapsed() / 1000.0, fpAimSightings, std::hypot(fpAimShift.x, fpAimShift.y), fpAimShift.x, fpAimShift.y);
        } else {
            std::printf("%s %5.2f s  goal tag: never seen (drove the odometry route)\n", fpRouteName, elapsed() / 1000.0);
        }
    }
    fpAimKind = AimKind::none;
}
Vec aimShift() { return fpAimShift; }
int aimSightings() { return fpAimSightings; }

// =====================================================================
//   VISION PICKUP TOOL: find one cup with both cameras (src/tuning.cpp, tuneVisionPickup)
// =====================================================================
namespace {
#ifndef RW_SIM_PROS
// The bearing (degrees right of the robot's straight ahead, from the camera) of the ONE cup this camera sees now,
// with the autos' filters. false = no cup, or two that disagree by more than 6 degrees (about 1.5" at 14").
bool oneCupBearing(Cam cam, double& bearing) {
    const int32_t count = camera(cam).get_object_count();
    if (count <= 0 || count > 64) return false;
    double low = 1e9, high = -1e9, sum = 0;
    int n = 0;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera(cam).get_object(i);
        if (!(seen.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
        if (!isCupClass(seen.id) || seen.object.element.score < AI_VISION_MIN_SCORE) continue;
        const auto& box = seen.object.element;
        if (cam == Cam::claw && box.yoffset + box.height / 2 >= FP_CLAW_CAM_MAX_CENTER_Y) continue;   // the gears / claw
        double centerPx = box.xoffset + box.width / 2.0;
        if (cam == Cam::field ? FP_GOAL_CAM_UPSIDE_DOWN : FP_PIN_CAM_UPSIDE_DOWN) centerPx = FP_CAM_WIDTH_PX - centerPx;
        double tanAcross = (centerPx - FP_CAM_WIDTH_PX / 2.0) / FP_CAM_FOCAL_PX;
        if (cam == Cam::claw) tanAcross /= std::cos(rad(FP_PIN_CAM_TILT_DEG));
        const double b = std::atan(tanAcross) * 180.0 / FP_PI + camYaw(cam);
        if (cam == Cam::claw && std::fabs(b) > 30.0) continue;   // finding it: a wider look than the autos' FP_AIM_AHEAD_DEG
        low = std::min(low, b);
        high = std::max(high, b);
        sum += b;
        n++;
    }
    if (n == 0 || high - low > 6.0) return false;
    bearing = sum / n;
    return true;
}
#endif
}  // namespace

CupFix locateCup(int ms) {
    CupFix fix;
    std::vector<double> xs, ys;
    const uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<uint32_t>(ms)) {
        service();
        fix.frames++;
#ifdef RW_SIM_PROS
        // Simulator: no cameras. SIM_CUP="x,y" (robot frame) pretends both see a cup there.
        const char* text = std::getenv("SIM_CUP");
        double sx = 0, sy = 0;
        if (text != nullptr && std::sscanf(text, "%lf,%lf", &sx, &sy) == 2) {
            xs.push_back(sx);
            ys.push_back(sy);
        }
#else
        double clawDeg = 0, fieldDeg = 0;
        const bool clawSees = oneCupBearing(Cam::claw, clawDeg);
        const bool fieldSees = oneCupBearing(Cam::field, fieldDeg);
        if (clawSees && !fieldSees) fix.clawOnly++;
        if (fieldSees && !clawSees) fix.fieldOnly++;
        if (clawSees && fieldSees) {
            // Two lines of sight: from each lens along its bearing. Where they cross is the cup.
            const Vec c = {0.0, FP_PIN_CAM_FORWARD}, f = {FP_GOAL_CAM_RIGHT, FP_GOAL_CAM_FORWARD};
            const Vec dc = {std::sin(rad(clawDeg)), std::cos(rad(clawDeg))}, df = {std::sin(rad(fieldDeg)), std::cos(rad(fieldDeg))};
            const double cross = dc.x * df.y - dc.y * df.x;
            if (std::fabs(cross) > std::sin(rad(2.0))) {   // not (nearly) parallel
                const double t = ((f.x - c.x) * df.y - (f.y - c.y) * df.x) / cross;   // inches along the claw camera's line
                const double s = ((f.x - c.x) * dc.y - (f.y - c.y) * dc.x) / cross;   // ...and the field camera's
                const double x = c.x + dc.x * t, y = c.y + dc.y * t;
                if (t > 0 && s > 0 && y > 0 && y < 60) {
                    xs.push_back(x);
                    ys.push_back(y);
                }
            }
        }
#endif
        pros::delay(30);
    }
    fix.fixes = static_cast<int>(xs.size());
    if (fix.fixes >= 5) {
        std::vector<double> sx = xs, sy = ys;
        std::nth_element(sx.begin(), sx.begin() + sx.size() / 2, sx.end());
        std::nth_element(sy.begin(), sy.begin() + sy.size() / 2, sy.end());
        fix.x = sx[sx.size() / 2];
        fix.y = sy[sy.size() / 2];
        for (size_t i = 0; i < xs.size(); i++) fix.spread = std::max(fix.spread, std::hypot(xs[i] - fix.x, ys[i] - fix.y));
    }
    return fix;
}

// =====================================================================
//   SERVICE: runs every 10 ms while the robot drives or waits. Lift, claw, flipper, intake, tip guard, timer.
// =====================================================================
void service() {
    uint32_t now = pros::millis();

    // Timer: lift PID off and limp just before the end
    if (!fpTimeUp && elapsed() >= fpLimitMs - fpEndReserveMs) {
        fpTimeUp = true;
        liftCoast();
    }
    rememberPose();   // for the cameras (their pictures are a moment old)
    if (fpStatusOnPad && now - fpStatusLastMs >= 250) {
        fpStatusLastMs = now;
        statusToPad();
    }

    // Tip guard
    if (FP_TIP_GUARD && !fpTimeUp) {
        double roll = inertial_sensor.get_roll();
        bool rollOk = std::isfinite(roll) && std::fabs(roll) < 360.0;
        if (!fpTipping) {
            bool liftUp = fpLiftMode == LiftMode::pid && getDR4BPosition() > FP_TIP_MIN_LIFT;
            fpTipTicks = (rollOk && liftUp && std::fabs(roll) > FP_TIP_ROLL_LIMIT) ? fpTipTicks + 1 : 0;
            if (fpTipTicks >= FP_TIP_CONFIRM_TICKS) {
                fpTipRestoreGoal = fpLiftGoal;
                liftDown();
                fpTipRestoreDown = false;
                fpTipping = true;
            }
        } else if (!rollOk || std::fabs(roll) < FP_TIP_ROLL_CLEAR) {
            // Level again: back to the height the route wants now, and on with the route. If the route has let go and
            // sent the lift down in the meantime, it stays down. (It used to go back up to the old scoring height
            // even then, and the robot drove off with the lift up.)
            fpTipping = false;
            fpTipTicks = 0;
            if (!fpTipRestoreDown) liftTo(fpTipRestoreGoal);
        }
    }

    // Lift
    if (fpLiftMode == LiftMode::pid && getDR4BPosition() >= fpLiftRiseMark + FP_LIFT_RISE_DEG) {
        fpLiftRiseMark = getDR4BPosition();   // still climbing
        fpLiftRiseMs = now;
    }
    if (fpLiftPending && !fpTipping && !fpTimeUp && now >= fpLiftPendingAtMs) startHop();
    if (fpLiftMode == LiftMode::hop) {
        if (getDR4BPosition() >= fpHopGoal || now - fpHopStartMs >= static_cast<uint32_t>(PIN_LIFT_TIMEOUT_MS)) endHop();
        else DR4B.move_voltage(PIN_LIFT_VOLTAGE);
    }
    if (fpLiftMode == LiftMode::ground) {
        double position = getDR4BPosition();
        if (position > FP_GROUND_FAST_ABOVE) {
            DR4B.move_voltage(-12000);
            fpGroundPushing = false;
        } else {
            if (!fpGroundPushing) {
                fpGroundPushing = true;
                fpGroundPushStartMs = now;
            }
            bool settled = position <= 1.0 || now - fpGroundPushStartMs >= static_cast<uint32_t>(FP_GROUND_PUSH_MAX_MS);
            int hold = fpLiftPress ? FP_PICKUP_PRESS_MV : FP_GROUND_HOLD_MV;   // picking up: keeps pushing down
            DR4B.move_voltage(settled ? -hold : -std::max(FP_GROUND_PUSH_MV, hold));
        }
    }

    // Claw
    switch (fpClawMode) {
        case ClawMode::off:
            claw.move_voltage(0);
            break;
        case ClawMode::opening:
            claw.move_velocity(-FP_CLAW_RPM);   // same as holding R2
            break;
        case ClawMode::closing:
            claw.move_velocity(FP_CLAW_RPM);
            if (now - fpClawSinceMs >= static_cast<uint32_t>(FP_CLAW_FAST_MS)) fpClawMode = ClawMode::hold;
            break;
        case ClawMode::hold:
            claw.move_voltage(FP_CLAW_HOLD_MV);
            break;
    }

    // Flipper
    if (fpFlipMode == FlipMode::up) {
        flipper.move_velocity(FP_FLIP_RPM);   // keeps pushing while it's flipped (like holding A)
    } else if (fpFlipMode == FlipMode::down) {
        flipper.move_velocity(-FP_FLIP_DOWN_RPM);
        double position = flipper.get_position();
        if (std::fabs(position - fpFlipCheckPos) >= FP_FLIPPER_STALL_DEG) {
            fpFlipCheckPos = position;
            fpFlipCheckMs = now;
        } else if (now - fpFlipCheckMs >= static_cast<uint32_t>(FP_FLIPPER_STALL_MS)) {
            fpFlipMode = FlipMode::off;   // on its stop = home
        }
        if (now - fpFlipSinceMs >= static_cast<uint32_t>(FP_FLIPPER_DOWN_MAX_MS)) fpFlipMode = FlipMode::off;
    } else {
        flipper.move_voltage(0);
    }

    // Intake
    if (fpIntakeOn && fpIntakeOffAtMs != 0 && now >= fpIntakeOffAtMs) fpIntakeOn = false;
    intake.move_voltage(fpIntakeOn ? FP_INTAKE_MV : 0);

    aimUpdate();
}

bool waitMs(int ms) {
    uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<uint32_t>(ms)) {
        service();
        if (fpTimeUp) return false;
        pros::delay(10);
    }
    return true;
}

void coreStart(const char* routeName, int limitMs) {
    fpRouteName = routeName;
    fpStartMs = pros::millis();
    fpLimitMs = limitMs;
    fpEndReserveMs = FP_COAST_BEFORE_END_MS;
    fpTimeUp = false;
    fpPastFilled = 0;
    fpLiftMode = LiftMode::idle;
    fpLiftPending = false;
    fpLiftPress = false;
    fpClawMode = ClawMode::off;
    fpFlipMode = FlipMode::off;
    fpIntakeOn = false;
    fpIntakeOffAtMs = 0;
    fpTipping = false;
    fpTipTicks = 0;
    fpTipRestoreDown = false;
    fpAimKind = AimKind::none;
    fpAimWindow = 0.0;
    fpAimShift = {0, 0};
    left_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    flipper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);   // off = limp, never held against its stop
    visionStart();
    // A route always runs with MCL correcting (a tuning tool that paused it may have been cut off by field control)
    pauseFieldMcl(false);
    fpStatusOnPad = true;
    fpStatusLastMs = 0;
    pad::line(2, "%s", "");
}

void finishSafe(bool stopDrive) {
    if (stopDrive) stopChassis(pros::MotorBrake::brake);
    intake.move_voltage(0);
    flipper.move_voltage(0);
    if (clawHolding()) claw.move_voltage(FP_CLAW_HOLD_MV);
    else claw.move_voltage(0);
    if (fpTimeUp) note("time up: lift limp");
    if (fpStatusOnPad) {
        // Stays on the controller after the route: how long it took, where it ended, and its last step (line 2)
        pad::line(0, "%-7.7s END %.1fs%s", fpRouteName, elapsed() / 1000.0, fpTimeUp ? " T" : "");   // T = ran out of time
        pad::line(1, "X%.1f Y%.1f H%.0f", x_pos, y_pos, heading360());
        fpStatusOnPad = false;   // nothing updates it now
    }
}

// =====================================================================
//   CURVES
// =====================================================================
Curve makeCurve(const std::vector<Node>& nodes, bool reverse, double maxSpeed, double endSpeed) {
    Curve curve;
    curve.reverse = reverse;
    if (nodes.size() < 2) return curve;

    // Dense points (field coordinates)
    std::vector<Vec> dense;
    std::vector<int> nodeDense;
    for (size_t i = 0; i + 1 < nodes.size(); i++) {
        const Node& a = nodes[i];
        const Node& b = nodes[i + 1];
        Vec ta = forwardOf(a.heading + (reverse ? 180.0 : 0.0));   // direction of travel at each end
        Vec tb = forwardOf(b.heading + (reverse ? 180.0 : 0.0));
        double chord = std::hypot(b.x - a.x, b.y - a.y);
        // A tangent longer than the nodes are apart makes the curve loop back on itself (a spot the robot then
        // spins round at walking pace). The nodes are never exactly where they were drawn, since most curves start
        // from wherever the robot really is, so a set tangent is cut to 3/4 of the gap.
        double ka = std::min(a.tangentOut > 0 ? a.tangentOut : 0.4 * chord, 0.75 * chord);
        double kb = std::min(b.tangentIn > 0 ? b.tangentIn : 0.4 * chord, 0.75 * chord);
        Vec p0 = {a.x, a.y}, p1 = {a.x + ta.x * ka, a.y + ta.y * ka};
        Vec p3 = {b.x, b.y}, p2 = {b.x - tb.x * kb, b.y - tb.y * kb};
        int steps = std::max(static_cast<int>(chord * 4), 16);
        if (i == 0) nodeDense.push_back(0);
        for (int k = (i == 0 ? 0 : 1); k <= steps; k++) {
            double t = static_cast<double>(k) / steps, m = 1.0 - t;
            Vec p = {m * m * m * p0.x + 3 * m * m * t * p1.x + 3 * m * t * t * p2.x + t * t * t * p3.x,
                     m * m * m * p0.y + 3 * m * m * t * p1.y + 3 * m * t * t * p2.y + t * t * t * p3.y};
            dense.push_back(toField(p));
        }
        nodeDense.push_back(static_cast<int>(dense.size()) - 1);
    }

    // A point every FP_SAMPLE_STEP along it
    std::vector<Vec> even;
    std::vector<double> denseAt(dense.size(), 0.0);   // distance along the curve of each dense point
    even.push_back(dense[0]);
    double carried = 0.0, total = 0.0;
    for (size_t i = 1; i < dense.size(); i++) {
        double x0 = dense[i - 1].x, y0 = dense[i - 1].y;
        double seg = std::hypot(dense[i].x - x0, dense[i].y - y0);
        total += seg;
        denseAt[i] = total;
        if (seg < 1e-9) continue;
        double at = FP_SAMPLE_STEP - carried;
        while (at <= seg) {
            double f = at / seg;
            even.push_back({x0 + (dense[i].x - x0) * f, y0 + (dense[i].y - y0) * f});
            at += FP_SAMPLE_STEP;
        }
        carried = seg - (at - FP_SAMPLE_STEP);
    }
    if (std::hypot(even.back().x - dense.back().x, even.back().y - dense.back().y) > 0.05) even.push_back(dense.back());
    int n = static_cast<int>(even.size());
    for (int index : nodeDense) {
        curve.nodeIndex.push_back(std::min(n - 1, static_cast<int>(std::lround(denseAt[index] / FP_SAMPLE_STEP))));
    }

    // Direction and curvature at each point
    curve.points.resize(n);
    const int span = 3;   // curvature from points 1.5" either side
    for (int i = 0; i < n; i++) {
        Sample& s = curve.points[i];
        s.x = even[i].x;
        s.y = even[i].y;
        int a = std::max(i - 1, 0), b = std::min(i + 1, n - 1);
        s.heading = std::atan2(even[b].x - even[a].x, even[b].y - even[a].y) * 180.0 / FP_PI;
        s.curvature = 0.0;
        if (i >= span && i + span < n) {
            const Vec& p = even[i - span];
            const Vec& q = even[i];
            const Vec& r = even[i + span];
            double cross = (q.x - p.x) * (r.y - q.y) - (q.y - p.y) * (r.x - q.x);
            double lengths = std::hypot(q.x - p.x, q.y - p.y) * std::hypot(r.x - q.x, r.y - q.y) * std::hypot(r.x - p.x, r.y - p.y);
            s.curvature = lengths > 1e-9 ? -2.0 * cross / lengths : 0.0;
        }
    }
    for (int i = 0; i < std::min(span, n); i++) curve.points[i].curvature = curve.points[std::min(span, n - 1)].curvature;
    for (int i = std::max(n - span, 0); i < n; i++) curve.points[i].curvature = curve.points[std::max(n - span - 1, 0)].curvature;

    // Speed plan: slower in tight curves, and slowing down in time for every slow spot and the end
    for (int i = 0; i < n; i++) {
        double k = std::fabs(curve.points[i].curvature);
        double limit = maxSpeed;
        if (k > 1e-6) {
            limit = std::min(limit, std::sqrt(FP_LAT_ACCEL / k));                          // cornering grip
            limit = std::min(limit, FP_WHEEL_MAX / (1.0 + k * distance_between_wheels / 2.0));   // outside wheel at top speed
            limit = std::min(limit, rad(FP_MAX_TURN_DPS) / k);                             // spin rate
        }
        curve.points[i].speed = std::max(limit, FP_MIN_SPEED);
    }
    for (size_t j = 0; j < nodes.size(); j++) {
        if (nodes[j].speed <= 0) continue;
        int center = curve.nodeIndex[j], reach = static_cast<int>(3.0 / FP_SAMPLE_STEP);
        for (int i = std::max(center - reach, 0); i <= std::min(center + reach, n - 1); i++) {
            curve.points[i].speed = std::min(curve.points[i].speed, nodes[j].speed);
        }
    }
    if (endSpeed <= 0.0) {
        for (int i = 0; i < n; i++) {
            double leftToGo = (n - 1 - i) * FP_SAMPLE_STEP;
            curve.points[i].speed = std::min(curve.points[i].speed, std::max(FP_STOP_GAIN * leftToGo, FP_MIN_SPEED));
        }
    }
    curve.points[n - 1].speed = std::min(curve.points[n - 1].speed, std::max(endSpeed, 0.0));
    for (int i = n - 1; i > 0; i--) {
        double before = std::sqrt(curve.points[i].speed * curve.points[i].speed + 2.0 * FP_DECEL * FP_SAMPLE_STEP);
        curve.points[i - 1].speed = std::min(curve.points[i - 1].speed, before);
    }
    return curve;
}

// Slowdown: the stretch of the curve that ends endAt inches along it is capped at speed, long enough to take ms at
// that speed. The speed plan before it brakes down in time, so it's a smooth slowdown, never a stop. ms <= 0 = no change.
void addSlowdown(Curve& curve, double endAt, int ms, double speed) {
    int n = static_cast<int>(curve.points.size());
    if (ms <= 0 || n < 2) return;
    speed = std::max(speed, FP_MIN_SPEED);
    double length = speed * ms / 1000.0;
    int last = std::min(std::max(static_cast<int>(std::lround(endAt / FP_SAMPLE_STEP)), 0), n - 1);
    int first = std::max(static_cast<int>(std::lround((endAt - length) / FP_SAMPLE_STEP)), 0);
    for (int i = first; i <= last; i++) curve.points[i].speed = std::min(curve.points[i].speed, speed);
    for (int i = n - 1; i > 0; i--) {   // braking pass again, so it slows down before the slow stretch
        double before = std::sqrt(curve.points[i].speed * curve.points[i].speed + 2.0 * FP_DECEL * FP_SAMPLE_STEP);
        curve.points[i - 1].speed = std::min(curve.points[i - 1].speed, before);
    }
}
void addSlowdownAtEnd(Curve& curve, int ms, double speed) { addSlowdown(curve, curve.length(), ms, speed); }

// =====================================================================
//   FOLLOWER
// =====================================================================
namespace {
// Volts for one side of the drive to go a speed (in/s), same model RW uses for its paths (rw-config.cpp drive_k*)
double sideVolts(double speed, double accel, double measured) {
    double kV = drive_kV > 0 ? drive_kV : 12.0 / (drive_motor_rpm / 60.0 * wheel_distance_in * 0.92);
    double kA = drive_kA > 0 ? drive_kA : kV * 0.13;
    double direction = speed > 0.01 ? 1.0 : (speed < -0.01 ? -1.0 : 0.0);
    return drive_kS * direction + kV * speed + kA * accel + FP_SPEED_FEEDBACK * (speed - measured);
}

// Top speed right now: slower the higher the lift is (or is headed), crawling while it recovers from a tip.
// accelNow / decelNow: how fast the speed may rise / fall (in/s per second).
double speedCapNow(double& accelNow, double& decelNow) {
    // The higher of where the lift is and where it's been told to go: the cap is on from the moment it starts up
    // until it's back down after letting go
    double lift = getDR4BPosition();
    if (fpLiftMode == LiftMode::pid) lift = std::max(lift, fpLiftGoal);
    if (fpLiftPending || fpLiftMode == LiftMode::hop) lift = std::max(lift, fpHopThen);
    double tall = clampTo((lift - FP_TALL_LIFT_LO) / (FP_TALL_LIFT_HI - FP_TALL_LIFT_LO), 0.0, 1.0);
    accelNow = FP_ACCEL + (FP_TALL_ACCEL - FP_ACCEL) * tall;
    decelNow = 2.0 * FP_DECEL + (FP_TALL_DECEL - 2.0 * FP_DECEL) * tall;
    double cap = FP_MAX_SPEED + (FP_TALL_SPEED - FP_MAX_SPEED) * tall;
    if (fpTipping) cap = std::min(cap, FP_TIP_SPEED);
    return cap;
}
}  // namespace

namespace { bool fpGateHold = true; }
void setGateHold(bool hold) { fpGateHold = hold; }

MoveEnd follow(const Curve& curve, const MoveOptions& options) {
    int n = static_cast<int>(curve.points.size());
    if (n < 2) return MoveEnd::arrived;
    const double direction = curve.reverse ? -1.0 : 1.0;
    const double halfTrack = distance_between_wheels / 2.0;
    const double inchesPerRpm = wheel_distance_in / 60.0;

    // Time limit: twice the planned time plus a second
    int timeoutMs = options.timeoutMs;
    if (timeoutMs <= 0) {
        double planned = 0.0;
        for (int i = 1; i < n; i++) planned += FP_SAMPLE_STEP / std::max((curve.points[i].speed + curve.points[i - 1].speed) / 2.0, FP_MIN_SPEED);
        timeoutMs = static_cast<int>(planned * 2000.0) + 1000 + (options.gateBeforeEnd >= 0 ? FP_GATE_MAX_MS : 0);
    }

    double leftSpeed = left_chassis.get_actual_velocity() * inchesPerRpm;
    double rightSpeed = right_chassis.get_actual_velocity() * inchesPerRpm;
    double command = std::max(0.0, direction * (leftSpeed + rightSpeed) / 2.0);   // carries the speed it already has
    double lastWantLeft = leftSpeed, lastWantRight = rightSpeed;
    int closest = 0;
    int stalledMs = 0;
    int gateHeldMs = 0;
    bool gateOpen = options.gateBeforeEnd < 0;
    bool crept = false;        // it crept toward the gate (gateEta) instead of stopping
    double creepHold = 0.0;    // speed it keeps once the lift is up after creeping (the fastest it crept)
    uint32_t start = pros::millis();
    const Sample& last = curve.points[n - 1];
    Vec lastDir = forwardOf(last.heading);

    while (true) {
        service();
        if (fpTimeUp) {
            stopChassis(pros::MotorBrake::brake);
            return MoveEnd::timeUp;
        }
        if (pros::millis() - start > static_cast<uint32_t>(timeoutMs)) {
            stopChassis(pros::MotorBrake::brake);
            return MoveEnd::timedOut;
        }

        // Where it is on the curve (never searches backwards)
        double px = x_pos - fpAimShift.x, py = y_pos - fpAimShift.y;   // the nudged curve = the robot seen from the un-nudged one
        double best = 1e18;
        int stop = std::min(closest + 40, n);
        for (int i = closest; i < stop; i++) {
            double d = std::hypot(curve.points[i].x - px, curve.points[i].y - py);
            if (d < best) { best = d; closest = i; }
        }
        double done = closest * FP_SAMPLE_STEP, left = (n - 1 - closest) * FP_SAMPLE_STEP;
        double past = (px - last.x) * lastDir.x + (py - last.y) * lastDir.y;   // + = beyond the end
        if (closest >= n - 3 && (std::hypot(last.x - px, last.y - py) < FP_END_TOLERANCE || past > 0)) break;

        if (options.tick && !options.tick(done, left)) {
            if (options.brakeAtEnd) stopChassis(pros::MotorBrake::brake);
            return MoveEnd::cut;
        }

        // Speed: the plan, the lift / tip limit, and the gate
        double accelNow = FP_ACCEL, decelNow = 2.0 * FP_DECEL;
        const double capNow = speedCapNow(accelNow, decelNow);
        double target = std::min(curve.points[closest].speed, capNow);
        bool holding = false;
        if (!gateOpen && !(options.gateReady && options.gateReady())) {
            // Lift not up (yet, or it dropped for a tip): slow down to stop at the gate, or stop right here if it's past it
            double room = left - options.gateBeforeEnd;
            if (room <= 0.4 && !options.gateHold) {
                target = std::min(target, FP_GATE_CREEP_MIN);   // no stopping: keeps rolling in while the lift finishes
            } else if (room <= 0.4) {
                holding = true;
                gateHeldMs += 10;
                if (gateHeldMs > FP_GATE_MAX_MS) gateOpen = true;   // waited long enough: go anyway
            } else if (options.gateEta) {
                // Creep in: reach the gate about when the lift is up (no stop, no hard re-acceleration)
                double eta = options.gateEta() + FP_GATE_ETA_BUFFER_S;
                // Right at the end (room and time both near 0) the ratio gets jumpy, so it just keeps its speed
                double creep = (eta > FP_GATE_ETA_BUFFER_S + 0.2 || !crept) ? room / eta : creepHold;
                creep = std::max(creep, FP_GATE_CREEP_MIN);
                // Only counts as creeping when the lift really is what's holding it back. (It used to count from the
                // first moment of every scoring drive and capped the whole drive at 28 in/s, lift up or not.)
                if (creep < target) {
                    creepHold = creep;   // the steady speed it's creeping in at
                    target = creep;
                    crept = true;
                }
            } else {
                target = std::min(target, std::sqrt(2.0 * FP_TALL_ACCEL * room));
            }
        } else if (crept) {
            // Lift is up after creeping in: keep the creep speed the rest of the way (no speed-up with the lift high)
            target = std::min(target, std::max(creepHold, FP_GATE_CREEP_MIN));
        }
        if (holding) {
            // Waiting for the lift: stand still, and use the wait to point straight down the curve
            const Sample& gateSpot = curve.points[closest];
            double off = wrap180(gateSpot.heading - (getInertialHeading() + (curve.reverse ? 180.0 : 0.0)));
            if (std::fabs(off) < 1.5) {
                stopChassis(pros::MotorBrake::brake);
            } else {
                double spin = rad(clampTo(FP_HEADING_GAIN * off, -120.0, 120.0)) * halfTrack;
                driveChassis(sideVolts(spin, 0, 0), sideVolts(-spin, 0, 0));
            }
            command = 0.0;
            stalledMs = 0;
            lastWantLeft = 0.0;
            lastWantRight = 0.0;
            pros::delay(10);
            continue;
        }
        if (target > command) command = std::min(target, command + accelNow * 0.010);
        else command = std::max(target, command - decelNow * 0.010);
        command = std::max(command, FP_MIN_SPEED);

        // Steering: the curve's own bend a little ahead, plus a correction for being off it / pointing off it.
        // The pointing correction never gets weaker than FP_HEADING_GAIN, so it still squares up at low speed.
        int ahead = std::min(closest + static_cast<int>(command * FP_STEER_PREVIEW_S / FP_SAMPLE_STEP), n - 1);
        const Sample& here = curve.points[closest];
        const Sample& there = curve.points[ahead];
        double travelHeading = getInertialHeading() + (curve.reverse ? 180.0 : 0.0);
        Vec pathRight = rightOf(here.heading);
        double offRight = (px - here.x) * pathRight.x + (py - here.y) * pathRight.y;   // + = robot is right of the curve
        offRight = clampTo(offRight, -FP_STEER_MAX_OFF, FP_STEER_MAX_OFF);             // a position jump is never a swerve
        double pointOff = rad(wrap180(there.heading - travelHeading));                 // + = curve points right of the robot
        // Chases a point FP_STEER_TIME_S of driving ahead: further ahead the faster it goes
        const double steerDist = clampTo(command * FP_STEER_TIME_S, FP_STEER_DIST, FP_STEER_DIST_MAX);
        double turnRate = command * there.curvature                                     // rad/s, + = clockwise
                          - command * 2.0 * offRight / (steerDist * steerDist)
                          + std::max(2.0 * command / steerDist, FP_HEADING_GAIN) * pointOff;
        turnRate = clampTo(turnRate, -rad(FP_MAX_TURN_DPS) * 1.5, rad(FP_MAX_TURN_DPS) * 1.5);

        // Each side's speed (faster on the outside of the curve; backwards mirrors which side that is)
        double spin = turnRate * halfTrack * (curve.reverse ? -1.0 : 1.0);
        double v = direction * command;
        double wantLeft = v + direction * spin, wantRight = v - direction * spin;
        double biggest = std::max(std::fabs(wantLeft), std::fabs(wantRight));
        if (biggest > FP_WHEEL_MAX) {
            wantLeft *= FP_WHEEL_MAX / biggest;
            wantRight *= FP_WHEEL_MAX / biggest;
        }
        leftSpeed = 0.6 * leftSpeed + 0.4 * left_chassis.get_actual_velocity() * inchesPerRpm;
        rightSpeed = 0.6 * rightSpeed + 0.4 * right_chassis.get_actual_velocity() * inchesPerRpm;
        double leftAccel = clampTo((wantLeft - lastWantLeft) / 0.010, -FP_FF_ACCEL_MAX, FP_FF_ACCEL_MAX);
        double rightAccel = clampTo((wantRight - lastWantRight) / 0.010, -FP_FF_ACCEL_MAX, FP_FF_ACCEL_MAX);
        lastWantLeft = wantLeft;
        lastWantRight = wantRight;
        double leftVolts = sideVolts(wantLeft, leftAccel, leftSpeed);
        double rightVolts = sideVolts(wantRight, rightAccel, rightSpeed);
        double most = std::max(std::fabs(leftVolts), std::fabs(rightVolts));
        if (most > 12.0) {
            leftVolts *= 12.0 / most;
            rightVolts *= 12.0 / most;
        }
        driveChassis(leftVolts, rightVolts);

        // Against something: told to drive, not moving and not turning
        if (command >= FP_MIN_SPEED && odom_speed_ips < 1.0 && std::fabs(odom_turn_rate_dps) < 60.0 && pros::millis() - start > 200) stalledMs += 10;
        else stalledMs = 0;
        // Driving into something at the end (a goal, the wall): close to the end, touching it is enough
        const int stallLimit = (options.endStallMs > 0 && left <= options.endStallIn) ? options.endStallMs : options.stallMs;
        if (stalledMs >= stallLimit) {
            if (options.brakeAtEnd) stopChassis(pros::MotorBrake::brake);
            return MoveEnd::stalled;
        }
        pros::delay(10);
    }
    if (options.brakeAtEnd) stopChassis(pros::MotorBrake::brake);
    return MoveEnd::arrived;
}

bool spinTo(double heading, int maxMs) {
    const double halfTrack = distance_between_wheels / 2.0;
    const double target = headingToField(heading);
    const double inchesPerRpm = wheel_distance_in / 60.0;
    uint32_t start = pros::millis();
    double rate = 0.0;       // degrees/s it's being asked to turn at (+ = clockwise)
    double lastSpin = 0.0;   // in/s each side was last asked for
    const double firstOff = wrap180(target - getInertialHeading());
    while (pros::millis() - start < static_cast<uint32_t>(maxMs)) {
        service();
        if (fpTimeUp) {
            stopChassis(pros::MotorBrake::brake);
            return false;
        }
        double off = wrap180(target - getInertialHeading());   // + = turn right
        // There: close, and no longer swinging fast (a turn that overshoots is brought back, not left there)
        if (std::fabs(off) < FP_SPIN_DONE_DEG && std::fabs(odom_turn_rate_dps) < FP_SPIN_DONE_DPS) break;
        // As fast as it can still brake from: v = sqrt(2 * decel * degrees left)
        double want = std::min(FP_SPIN_MAX_DPS, std::sqrt(2.0 * FP_SPIN_DECEL_DPS2 * std::fabs(off)));
        want = std::max(want, FP_SPIN_MIN_DPS) * (off > 0 ? 1.0 : -1.0);
        const double step = FP_SPIN_ACCEL_DPS2 * 0.010;
        rate = std::fabs(want) < std::fabs(rate) ? want : clampTo(want, rate - step, rate + step);   // brakes at once, spins up steadily
        double spin = rad(rate) * halfTrack;
        double accel = clampTo((spin - lastSpin) / 0.010, -FP_FF_ACCEL_MAX, FP_FF_ACCEL_MAX);
        lastSpin = spin;
        driveChassis(sideVolts(spin, accel, left_chassis.get_actual_velocity() * inchesPerRpm),
                     sideVolts(-spin, -accel, right_chassis.get_actual_velocity() * inchesPerRpm));
        pros::delay(10);
    }
    stopChassis(pros::MotorBrake::brake);
    const double endOff = wrap180(target - getInertialHeading());
    if (std::fabs(endOff) > 2.0 * FP_SPIN_DONE_DEG) {
        std::printf("%s %5.2f s  turn of %.0f deg ended %.1f deg off after %d ms\n", fpRouteName, elapsed() / 1000.0, firstOff, endOff,
                    static_cast<int>(pros::millis() - start));
    }
    return true;
}

bool shoveForward(double inches, int maxMs) {
    const double startX = x_pos, startY = y_pos;
    uint32_t start = pros::millis();
    int stalledMs = 0;
    while (pros::millis() - start < static_cast<uint32_t>(maxMs)) {
        service();
        if (fpTimeUp) {
            stopChassis(pros::MotorBrake::brake);
            return false;
        }
        if (std::hypot(x_pos - startX, y_pos - startY) >= inches) break;
        driveChassis(12.0, 12.0);
        if (odom_speed_ips < 1.0 && pros::millis() - start > 200) stalledMs += 10;
        else stalledMs = 0;
        if (stalledMs >= FP_STALL_MS) break;   // against something
        pros::delay(10);
    }
    return true;   // no brake: the next move carries the speed on
}

// =====================================================================
//   ROUTE HELPERS
// =====================================================================
double backCornerLineGap() {
    Node now = hereNode();
    Vec f = forwardOf(now.heading), r = rightOf(now.heading);
    double gap = 1e9;
    for (double side : {-FP_HALF_WIDTH, FP_HALF_WIDTH}) {
        double cx = now.x - f.x * FP_CENTER_TO_BACK + r.x * side;
        double cy = now.y - f.y * FP_CENTER_TO_BACK + r.y * side;
        gap = std::min(gap, -(cx + cy) / std::sqrt(2.0));
    }
    return gap;
}

double frontCornerPastWall() {
    Node now = hereNode();
    Vec f = forwardOf(now.heading), r = rightOf(now.heading);
    double lowest = 1e9;
    for (double side : {-FP_HALF_WIDTH, FP_HALF_WIDTH}) {   // the body's front corners
        lowest = std::min(lowest, now.y + f.y * FP_CENTER_TO_FRONT + r.y * side);
    }
    for (double side : {-FP_CLAW_HALF_WIDTH, FP_CLAW_HALF_WIDTH}) {   // the claw's tips (further out, narrower)
        lowest = std::min(lowest, now.y + f.y * FP_CLAW_TIP_FORWARD + r.y * side);
    }
    return -FP_FIELD_HALF - lowest;
}

double clawGap(double routeX, double routeY) {
    Vec spot = toField({routeX, routeY});
    Vec f = forwardOf(getInertialHeading());
    double cx = x_pos + f.x * FP_CLAW_REACH, cy = y_pos + f.y * FP_CLAW_REACH;
    return std::hypot(spot.x + fpAimShift.x - cx, spot.y + fpAimShift.y - cy);
}

// At a goal, stopped: lets go (see the header). The distance sensors check the position while it's standing still.
bool letGo(int settleMs, int releaseMs, int lowerBeforeOpenMs) {
    if (!waitMs(settleMs)) return false;
    if (lowerBeforeOpenMs > 0) {
        note("at the goal: lower, let go");
        liftDown();   // sets the pin down on the goal; keeps coming down on the way out
        if (!waitMs(lowerBeforeOpenMs)) return false;
        clawOpen();
    } else {
        note("at the goal: let go");
        clawOpen();
    }
    // The reset (up to 0.12 s) runs inside the release wait, not on top of it: no extra time standing at the goal
    const uint32_t resetStart = pros::millis();
    if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);
    return waitMs(std::max(releaseMs - static_cast<int>(pros::millis() - resetStart), 0));
}

namespace {
// A pin picked up on the way to the goal
struct PinPlan {
    int node;
    double x, y, mclX, mclY;
    int slowMs;
};

// Caps the speed between two distances along the curve, and brakes down to it in time
void capSpeed(Curve& curve, double fromAt, double toAt, double speed) {
    const int n = static_cast<int>(curve.points.size());
    if (n < 2 || speed <= 0.0) return;
    speed = std::max(speed, FP_MIN_SPEED);
    const int first = std::min(std::max(static_cast<int>(std::lround(fromAt / FP_SAMPLE_STEP)), 0), n - 1);
    const int last = std::min(std::max(static_cast<int>(std::lround(toAt / FP_SAMPLE_STEP)), 0), n - 1);
    for (int i = first; i <= last; i++) curve.points[i].speed = std::min(curve.points[i].speed, speed);
    for (int i = n - 1; i > 0; i--) {
        double before = std::sqrt(curve.points[i].speed * curve.points[i].speed + 2.0 * FP_DECEL * FP_SAMPLE_STEP);
        curve.points[i - 1].speed = std::min(curve.points[i - 1].speed, before);
    }
}

// The scoring move: along the curve (through a pin first, if there is one) into the goal, lift up in time, let go.
bool runScore(std::vector<Node> nodes, const PinPlan* pin, const ScorePlan& plan) {
    if (nodes.size() < 2) return !fpTimeUp;
    // The curve carries on past the spot where the claw should be on the goal: the goal stops the robot
    const double push = std::max(plan.pushIn, 0.0);
    if (push > 0.0) nodes.push_back(along(nodes.back(), push));
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    const int n = static_cast<int>(curve.points.size());
    const double length = curve.length();
    const double seatAt = length - push;                         // inches along the curve where the claw is on the goal
    const double gateAt = seatAt - FP_SCORE_GATE_IN;             // ...and where the lift has to be up by
    const double grabAt = pin ? curve.nodeIndex[pin->node] * FP_SAMPLE_STEP : -1.0;
    if (pin) addSlowdown(curve, grabAt, pin->slowMs, FP_PICKUP_SLOW_SPEED);
    addSlowdown(curve, seatAt, plan.scoreSlowMs, FP_SCORE_SLOW_SPEED);
    if (plan.approachSpeed > 0.0) capSpeed(curve, seatAt - plan.approachIn, length, plan.approachSpeed);
    // Meeting the goal: gently, so the stack that's already on it isn't knocked
    if (push > 0.0) capSpeed(curve, seatAt - 1.5, length, FP_CONTACT_SPEED);

    // Seconds from each point of the curve to the gate, at the planned speeds (speeding up from how fast it's going now)
    std::vector<double> toGate(n, 0.0);
    {
        std::vector<double> speedAt(n);
        double speed = std::max(odom_speed_ips, FP_MIN_SPEED);
        for (int i = 0; i < n; i++) {
            speed = std::min(curve.points[i].speed, std::sqrt(speed * speed + 2.0 * FP_ACCEL * FP_SAMPLE_STEP));
            speedAt[i] = std::max(speed, FP_MIN_SPEED);
        }
        const int gateIndex = std::min(std::max(static_cast<int>(std::lround(gateAt / FP_SAMPLE_STEP)), 0), n - 1);
        for (int i = gateIndex - 1; i >= 0; i--) toGate[i] = toGate[i + 1] + FP_SAMPLE_STEP / ((speedAt[i] + speedAt[i + 1]) / 2.0);
    }
    const double liftLead = liftSeconds(plan.height) + FP_LIFT_LEAD_MARGIN_S;   // the lift starts up this long before the gate

    bool grabbed = pin == nullptr;
    bool liftStarted = false;
    auto aimGoal = [&]() {
        if (FP_USE_GOAL_TAG && plan.tagId != NO_GOAL_TAG) aimAt(AimKind::goalTag, plan.goalX, plan.goalY, plan.tagId);
    };
    if (pin) {
        aimAt(AimKind::pin, pin->x, pin->y);
    } else {
        aimGoal();
        if (!plan.liftLate) {
            liftTo(plan.height);
            liftStarted = true;
        }
    }

    MoveOptions options;
    options.gateBeforeEnd = FP_SCORE_GATE_IN + push;
    options.gateHold = fpGateHold;
    // Up, or as high as it gets (no creeping in at 8 in/s behind a lift that has stopped a little short)
    options.gateReady = [&]() { return liftStarted && liftUpOrStuck() && !fpTipping; };
    options.gateEta = [&]() {
        if (!liftStarted) return liftSeconds(plan.height) + std::max(plan.liftNotBeforeMs - elapsed(), 0) / 1000.0;
        double climb = plan.height - FP_LIFT_READY_TOLERANCE - std::max(getDR4BPosition(), 0.0);
        return std::max(climb, 0.0) / FP_LIFT_UP_DPS;
    };
    if (push > 0.0) {
        options.endStallMs = FP_CONTACT_STALL_MS;
        options.endStallIn = push + 1.5;
    }
    options.tick = [&](double done, double left) {
        (void)left;
        if (!grabbed) {
            if (grabAt - done <= FP_CLAW_OPEN_DIST) {
                clawOpen();
                liftDown();        // lift pushed down onto the floor while it goes in for the pin
                liftPress(true);
            }
            if (done >= grabAt || (grabAt - done < 4.0 && clawGap(pin->x, pin->y) <= FP_GRAB_TOLERANCE)) {
                // The pin is in the claw: close on it, full speed for FP_CLAW_FAST_MS then the hold. The lift hops
                // off the ground like driver control's pin pickup; it goes on up to the scoring height straight
                // away, or (liftLate) stays at the hop until the goal is close.
                grabbed = true;
                note("claw closes on a pin");
                clawClose();
                liftPress(false);
                liftHopThen(plan.liftLate ? -1.0 : plan.height);
                if (!plan.liftLate) liftStarted = true;
                aimStop();
                Vec gone = toField({pin->mclX, pin->mclY});
                fieldMclPinPickedUp(gone.x, gone.y);   // MCL stops expecting a pin there
                aimGoal();                             // line up on the goal's tag
            }
        }
        if (pin == nullptr && plan.clawOpenIn > 0.0 && seatAt - done <= plan.clawOpenIn) clawOpen();   // the goal goes into the open claw
        if (grabbed && !liftStarted) {
            const int index = std::min(std::max(static_cast<int>(done / FP_SAMPLE_STEP), 0), n - 1);
            if (toGate[index] <= liftLead && elapsed() >= plan.liftNotBeforeMs) {
                liftStarted = true;
                liftAfterHop(plan.height);
            }
        }
        return true;
    };
    MoveEnd end = follow(curve, options);
    aimStop();
    if (end == MoveEnd::timeUp) return false;

    if (!grabbed) {   // the curve ended before the grab spot (stalled / timed out): close anyway
        clawClose();
        liftPress(false);
        liftHopThen(plan.height);
        liftStarted = true;
    }
    // The move ended before the lift was up (it met the goal early, or the lift is slower than FP_LIFT_UP_DPS): wait
    // for it, so the pin is let go at its height and not into the side of the stack. Only while it's still climbing:
    // once it stops rising it's as high as it gets, and waiting longer changes nothing (it used to wait out the whole
    // FP_GATE_MAX_MS = 1.5 s at the goal for a lift that had topped out a little short, then let go anyway).
    if (plan.clawOpenIn > 0.0 && pin == nullptr) clawOpen();
    if (!liftStarted) {
        while (elapsed() < plan.liftNotBeforeMs) {   // (it got there first: the lift still may not start yet)
            if (!waitMs(10)) return false;
        }
        liftAfterHop(plan.height);
    }
    const uint32_t waitStart = pros::millis();
    while (!(liftUpOrStuck() && !fpTipping) && pros::millis() - waitStart < static_cast<uint32_t>(FP_GATE_MAX_MS)) {
        if (!waitMs(10)) return false;
    }
    const int waitedMs = static_cast<int>(pros::millis() - waitStart);
    if (liftStuck()) {
        std::printf("%s %5.2f s  lift stopped at %.0f (wanted %.0f): scoring there, waited %d ms\n", fpRouteName, elapsed() / 1000.0,
                    getDR4BPosition(), plan.height, waitedMs);
    } else if (waitedMs > 30) {
        std::printf("%s %5.2f s  at the goal before the lift: waited %d ms for it\n", fpRouteName, elapsed() / 1000.0, waitedMs);
    }
    if (!plan.release) return !fpTimeUp;
    return letGo(plan.settleMs, plan.releaseMs, plan.lowerBeforeOpenMs);
}
}  // namespace

bool scoreWith(const std::vector<Node>& nodes, const ScorePlan& plan) { return runScore(nodes, nullptr, plan); }

bool pickUpAndScoreWith(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY,
                        int pinSlowMs, const ScorePlan& plan) {
    PinPlan pin = {pinNode, pinX, pinY, mclPinX, mclPinY, pinSlowMs};
    return runScore(nodes, &pin, plan);
}

bool pickUpAndScore(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY,
                    double scoreHeight, double goalX, double goalY, int goalTagId, int pinSlowMs, int scoreSlowMs, int settleMs,
                    int releaseMs, int lowerBeforeOpenMs) {
    ScorePlan plan;
    plan.height = scoreHeight;
    plan.goalX = goalX;
    plan.goalY = goalY;
    plan.tagId = goalTagId;
    plan.scoreSlowMs = scoreSlowMs;
    plan.settleMs = settleMs;
    plan.releaseMs = releaseMs;
    plan.lowerBeforeOpenMs = lowerBeforeOpenMs;
    return pickUpAndScoreWith(nodes, pinNode, pinX, pinY, mclPinX, mclPinY, pinSlowMs, plan);
}

bool scoreOnGoal(const std::vector<Node>& nodes, double scoreHeight, double goalX, double goalY, int goalTagId, int scoreSlowMs,
                 int settleMs, int releaseMs, int lowerBeforeOpenMs) {
    ScorePlan plan;
    plan.height = scoreHeight;
    plan.goalX = goalX;
    plan.goalY = goalY;
    plan.tagId = goalTagId;
    plan.scoreSlowMs = scoreSlowMs;
    plan.settleMs = settleMs;
    plan.releaseMs = releaseMs;
    plan.lowerBeforeOpenMs = lowerBeforeOpenMs;
    return scoreWith(nodes, plan);
}

std::vector<Node> backArc(const Node& start, double straight, double arcIn, double noseTurnDeg) {
    Node off = along(start, -straight);
    if (arcIn <= 0.0 || std::fabs(noseTurnDeg) < 1.0) return {start, off};
    const double turn = rad(std::fabs(noseTurnDeg)), radius = arcIn / turn;
    const double back = radius * std::sin(turn), side = radius * (1.0 - std::cos(turn));
    Vec f = forwardOf(start.heading), r = rightOf(start.heading);
    // Backing up with the nose swinging clockwise, the robot ends up to its own LEFT of the line it started on
    const double way = noseTurnDeg > 0 ? -1.0 : 1.0;
    return {start, off, node(off.x - f.x * back + r.x * side * way, off.y - f.y * back + r.y * side * way, start.heading + noseTurnDeg)};
}

void endDash(double routeX, double routeY, double zoneRadius, int driveMs) {
    if (!fpTimeUp) return;
    const Vec zone = toField({routeX, routeY});
    left_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);   // so what speed it has carries it on after the end
    right_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    note("time up: dash for the zone");
    while (elapsed() < fpLimitMs) {
        const double dx = zone.x - x_pos, dy = zone.y - y_pos, dist = std::hypot(dx, dy);
        if (dist <= zoneRadius * 0.5) break;   // well inside already
        double off = wrap180(std::atan2(dx, dy) * 180.0 / FP_PI - getInertialHeading());   // + = the zone is right of the nose
        const bool backwards = std::fabs(off) > 90.0;   // the back end is nearer to facing it: drive backwards
        if (backwards) off = wrap180(off + 180.0);
        // "Facing the zone" = pointing anywhere inside it (it's wide: this many degrees either side of its middle)
        const double wide = dist > zoneRadius ? std::asin(zoneRadius / dist) * 180.0 / FP_PI : 90.0;
        if (std::fabs(off) > wide && timeLeft() > driveMs) {
            const double turn = off > 0 ? 12.0 : -12.0;
            driveChassis(turn, -turn);
        } else {
            driveChassis(backwards ? -12.0 : 12.0, backwards ? -12.0 : 12.0);
        }
        pros::delay(5);
    }
    driveChassis(0.0, 0.0);
}

bool pickUp(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY, double thenHeight,
            int pinSlowMs) {
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    double grabAt = curve.nodeIndex[pinNode] * FP_SAMPLE_STEP;
    addSlowdown(curve, grabAt, pinSlowMs, FP_PICKUP_SLOW_SPEED);
    bool grabbed = false;
    aimAt(AimKind::pin, pinX, pinY);
    MoveOptions options;
    options.tick = [&](double done, double left) {
        (void)left;
        if (!grabbed) {
            if (grabAt - done <= FP_CLAW_OPEN_DIST) {
                clawOpen();
                liftDown();
                liftPress(true);
            }
            if (done >= grabAt || (grabAt - done < 4.0 && clawGap(pinX, pinY) <= FP_GRAB_TOLERANCE)) {
                grabbed = true;
                note("claw closes on a pin");
                clawClose();
                liftPress(false);
                liftHopThen(thenHeight);
                aimStop();
                Vec gone = toField({mclPinX, mclPinY});
                fieldMclPinPickedUp(gone.x, gone.y);
            }
        }
        return true;
    };
    MoveEnd end = follow(curve, options);
    aimStop();
    if (!grabbed) {   // the curve ended before the grab spot (stalled / cut): close anyway
        clawClose();
        liftPress(false);
        liftHopThen(thenHeight);
    }
    return end != MoveEnd::timeUp;
}

void snapToGoal(double goalX, double goalY) {
    const double heading = getInertialHeading();
    Vec f = forwardOf(heading);
    Vec goal = toField({goalX, goalY});
    setPose(goal.x - f.x * FP_CLAW_REACH, goal.y - f.y * FP_CLAW_REACH, heading);
    note("position snapped to the goal");
}

bool backOutTo(double straight, const Node& turnSpot, const std::function<void(double done)>& onTick) {
    // Straight back the way it's really pointing first (the goal is still in the claw opening), then the swing
    Node start = hereNode();
    Curve curve = makeCurve({start, along(start, -straight), turnSpot}, true, FP_MAX_SPEED, FP_CUSP_SPEED);
    MoveOptions options;
    options.brakeAtEnd = false;
    options.tick = [&](double done, double left) {
        (void)left;
        if (done >= FP_LOWER_AFTER_BACK_IN) liftDown();
        if (done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
        if (onTick) onTick(done);
        return true;
    };
    MoveEnd end = follow(curve, options);
    note("backed out, nose swung");
    liftDown();
    clawOff();
    return end != MoveEnd::timeUp;
}

bool backStraight(double inches) {
    Node start = hereNode();
    Curve curve = makeCurve({start, along(start, -inches)}, true, FP_MAX_SPEED, 0.0);
    MoveOptions options;
    return follow(curve, options) != MoveEnd::timeUp;
}

}  // namespace ac
