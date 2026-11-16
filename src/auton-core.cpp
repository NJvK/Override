// =====================================================================
//   AUTON CORE (see include/auton-core.hpp)
// =====================================================================
// The curve follower, the lift / claw / flipper / intake service, the AI Vision aiming, the tip guard and the timer
// that the 4-pin qual auton (src/fourpin.cpp) and the skills run (src/skills.cpp) are written with. This used to be
// the top of fourpin.cpp; it moved here so both routes drive the same way.
// =====================================================================
#include "auton-core.hpp"

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
bool fpTimeUp = false;
}
int elapsed() { return static_cast<int>(pros::millis() - fpStartMs); }
bool timeUp() { return fpTimeUp; }
void note(const char* what) {
    std::printf("%s %5.2f s  %-28s x %6.1f  y %6.1f  heading %5.0f  lift %4.0f\n", fpRouteName, elapsed() / 1000.0, what, x_pos, y_pos,
                std::fmod(std::fmod(getInertialHeading(), 360.0) + 360.0, 360.0), getDR4BPosition());
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
}

void liftTo(double height) {
    fpLiftPending = false;
    fpLiftPress = false;
    fpLiftMode = LiftMode::pid;
    fpLiftGoal = height;
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
void liftDown() {
    fpLiftPending = false;
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
namespace {
bool fpTipping = false;
int fpTipTicks = 0;
double fpTipRestoreGoal = 0.0;
}
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
int fpAimSightings = 0;      // camera frames that found it during this aim (for the terminal log)
// Pixels of focal length: how big something looks = FOCAL * its size / its distance
const double FP_CAM_FOCAL_PX = (FP_CAM_WIDTH_PX / 2.0) / std::tan(rad(FP_CAM_FOV_DEG / 2.0));

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

// Where a camera's lens is on the field right now
Vec cameraSpot(Cam cam, double heading) {
    Vec f = forwardOf(heading), r = rightOf(heading);
    if (cam == Cam::field) {
        return {x_pos + f.x * FP_GOAL_CAM_FORWARD + r.x * FP_GOAL_CAM_RIGHT, y_pos + f.y * FP_GOAL_CAM_FORWARD + r.y * FP_GOAL_CAM_RIGHT};
    }
    return {x_pos + f.x * FP_PIN_CAM_FORWARD, y_pos + f.y * FP_PIN_CAM_FORWARD};
}

// Where a goal tag seen at (bearing, range) from the camera puts the goal's CENTER on the field: through the tag
// and FP_GOAL_TAG_TO_CENTER further in
Vec goalFromTag(const Vec& cam, double heading, double bearing, double range) {
    Vec ray = forwardOf(heading + bearing);
    double reach = range + FP_GOAL_TAG_TO_CENTER;
    return {cam.x + ray.x * reach, cam.y + ray.y * reach};
}

// What one camera sees of the thing it should be looking at.
//   Pins: the detection closest to expectedBearing (within FP_AIM_WINDOW_DEG). range = 0 (direction only).
//   Goal tags: the tag that puts the goal closest to where it should be (within FP_GOAL_TAG_WINDOW_IN).
//   bearing: degrees right of the ROBOT's straight ahead (the camera's own yaw is already added in)
//   range:   inches from the camera lens to the tag, from how big it looks (tags only)
// false = nothing believable in view.
bool cameraSighting(Cam cam, AimKind kind, double expectedBearing, double& bearing, double& range) {
    range = 0.0;
    const bool tags = kind == AimKind::goalTag;
    const double heading = getInertialHeading();
    const Vec spot = cameraSpot(cam, heading);
    const double yaw = camYaw(cam);
    // How far a tag sighting puts the goal from where it should be
    auto goalError = [&](double seenBearing, double seenRange) {
        Vec goal = goalFromTag(spot, heading, seenBearing, seenRange);
        return std::hypot(goal.x - fpAimSpot.x, goal.y - fpAimSpot.y);
    };
#ifdef RW_SIM_PROS
    // Simulator: there's no camera. SIM_PIN_SHIFT="dx,dy" / SIM_GOAL_SHIFT="dx,dy" pretend every pin / goal sits
    // that far (route inches) from its spot and that the camera sees it perfectly, to test the lining up.
    const char* text = std::getenv(kind == AimKind::pin ? "SIM_PIN_SHIFT" : "SIM_GOAL_SHIFT");
    double dx = 0, dy = 0;
    if (kind == AimKind::none || text == nullptr || std::sscanf(text, "%lf,%lf", &dx, &dy) != 2) return false;
    Vec moved = toField({dx, dy});
    double tx = fpAimSpot.x + moved.x - spot.x, ty = fpAimSpot.y + moved.y - spot.y;
    bearing = wrap180(std::atan2(tx, ty) * 180.0 / FP_PI - heading);
    if (std::fabs(bearing - yaw) > FP_CAM_FOV_DEG / 2.0) return false;   // outside the picture
    if (!tags) return std::fabs(bearing - expectedBearing) <= FP_AIM_WINDOW_DEG;
    range = std::max(std::hypot(tx, ty) - FP_GOAL_TAG_TO_CENTER, 0.0);   // the tag is on the goal's face
    return goalError(bearing, range) <= FP_GOAL_TAG_WINDOW_IN;
#else
    int32_t count = camera(cam).get_object_count();
    if (count <= 0 || count > 64) return false;   // nothing, or the sensor isn't answering (PROS_ERR)
    bool found = false;
    double bestOff = tags ? FP_GOAL_TAG_WINDOW_IN : FP_AIM_WINDOW_DEG;   // inches for tags, degrees for pins
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera(cam).get_object(i);
        double centerPx = 0, sizePx = 0;
        if (tags) {
            if (!(seen.type & pros::E_AIVISION_DETECTED_TAG)) continue;
            if (fpAimTagId >= 0 && seen.id != fpAimTagId) continue;
            const auto& t = seen.object.tag;
            centerPx = (t.x0 + t.x1 + t.x2 + t.x3) / 4.0;
            // The tag's size in pixels: its longest edge (the edges that face the camera square-on keep their
            // length when the tag is seen from an angle, the others shrink)
            sizePx = std::max({std::hypot(t.x1 - t.x0, t.y1 - t.y0), std::hypot(t.x2 - t.x1, t.y2 - t.y1),
                               std::hypot(t.x3 - t.x2, t.y3 - t.y2), std::hypot(t.x0 - t.x3, t.y0 - t.y3)});
            if (sizePx < FP_GOAL_TAG_MIN_PX) continue;
        } else {
            if (!(seen.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
            if (FP_PIN_CLASS_ID >= 0 && seen.id != FP_PIN_CLASS_ID) continue;
            if (seen.object.element.score < AI_VISION_MIN_SCORE) continue;   // too unsure (include/robot.hpp)
            // Claw camera: the gears fill the bottom of the picture. Something that starts down there is a gear.
            if (cam == Cam::claw && seen.object.element.yoffset >= FP_CLAW_CAM_BLOCKED_FROM_PX) continue;
            centerPx = seen.object.element.xoffset + seen.object.element.width / 2.0;
        }
        if (cam == Cam::field ? FP_GOAL_CAM_UPSIDE_DOWN : FP_PIN_CAM_UPSIDE_DOWN) centerPx = FP_CAM_WIDTH_PX - centerPx;
        double sideways = (centerPx - FP_CAM_WIDTH_PX / 2.0) / FP_CAM_FOCAL_PX;   // tan of the angle in the camera's picture
        if (cam == Cam::claw) sideways /= std::cos(rad(FP_PIN_CAM_TILT_DEG));     // the claw camera looks down: undo the squash
        double seenBearing = std::atan(sideways) * 180.0 / FP_PI + yaw;
        double seenRange = tags ? FP_CAM_FOCAL_PX * FP_GOAL_TAG_SIZE_IN / sizePx : 0.0;
        double off = tags ? goalError(seenBearing, seenRange) : std::fabs(seenBearing - expectedBearing);
        if (off <= bestOff) {
            bestOff = off;
            bearing = seenBearing;
            range = seenRange;
            found = true;
        }
    }
    return found;
#endif
}

// One camera's look at the thing being aimed at. true = it saw it, and the nudge was updated from it.
bool aimWith(Cam cam) {
    const bool tag = fpAimKind == AimKind::goalTag;
    double heading = getInertialHeading();
    Vec r = rightOf(heading);
    Vec spot = cameraSpot(cam, heading);
    double dx = fpAimSpot.x - spot.x, dy = fpAimSpot.y - spot.y;
    double dist = std::hypot(dx, dy);
    double maxDist = tag ? FP_GOAL_AIM_MAX_DIST : (cam == Cam::field ? FP_FIELD_PIN_MAX_DIST : FP_AIM_MAX_DIST);
    if (dist > maxDist || dist < (tag ? FP_GOAL_AIM_MIN_DIST : FP_AIM_MIN_DIST)) return false;
    double expected = wrap180(std::atan2(dx, dy) * 180.0 / FP_PI - heading);
    if (std::fabs(expected - camYaw(cam)) > FP_CAM_FOV_DEG / 2.0 - 4.0) return false;   // not in this camera's picture yet
    double seen = 0, range = 0;
    if (!cameraSighting(cam, fpAimKind, expected, seen, range)) return false;
    fpAimSightings++;

    Vec want;   // where the curve should be moved to (field inches)
    double smooth = FP_AIM_SMOOTH;
    if (tag && range > 0.0) {
        // The tag gives a direction and a distance: where the goal's center really is. The nudge is the difference
        // from where it should be, in any direction (sideways AND closer / further), capped at FP_GOAL_MAX_SHIFT.
        Vec goal = goalFromTag(spot, heading, seen, range);
        want = {goal.x - fpAimSpot.x, goal.y - fpAimSpot.y};
        double size = std::hypot(want.x, want.y);
        if (size > FP_GOAL_MAX_SHIFT) {
            want.x *= FP_GOAL_MAX_SHIFT / size;
            want.y *= FP_GOAL_MAX_SHIFT / size;
        }
        smooth = FP_GOAL_AIM_SMOOTH;
    } else {
        // Only a direction: it's this far to the side of where odometry says it should be
        double sideways = clampTo(dist * std::tan(rad(seen - expected)), -FP_AIM_MAX_SHIFT, FP_AIM_MAX_SHIFT);
        want = {r.x * sideways, r.y * sideways};
    }
    fpAimShift.x += smooth * (want.x - fpAimShift.x);
    fpAimShift.y += smooth * (want.y - fpAimShift.y);
    return true;
}

void aimUpdate() {
    uint32_t now = pros::millis();
    if (fpAimKind == AimKind::none) {
        // Not aiming: the nudge fades away so the rest of the curve is back on its drawn line
        fpAimShift.x *= 0.9;
        fpAimShift.y *= 0.9;
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
void aimAt(AimKind kind, double routeX, double routeY, int tagId) {
    fpAimKind = kind;
    fpAimSpot = toField({routeX, routeY});
    fpAimLastMs = 0;
    fpAimTagId = tagId;
    fpAimSightings = 0;
}
void aimStop() {
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

// =====================================================================
//   SERVICE: runs every 10 ms while the robot drives or waits. Lift, claw, flipper, intake, tip guard, timer.
// =====================================================================
void service() {
    uint32_t now = pros::millis();

    // Timer: lift PID off and limp just before the end
    if (!fpTimeUp && elapsed() >= fpLimitMs - FP_COAST_BEFORE_END_MS) {
        fpTimeUp = true;
        liftCoast();
    }

    // Tip guard
    if (FP_TIP_GUARD && !fpTimeUp) {
        double roll = inertial_sensor.get_roll();
        bool rollOk = std::isfinite(roll) && std::fabs(roll) < 360.0;
        if (!fpTipping) {
            bool liftUp = fpLiftMode == LiftMode::pid && getDR4BPosition() > FP_TIP_MIN_LIFT;
            fpTipTicks = (rollOk && liftUp && std::fabs(roll) > FP_TIP_ROLL_LIMIT) ? fpTipTicks + 1 : 0;
            if (fpTipTicks >= FP_TIP_CONFIRM_TICKS) {
                fpTipping = true;
                fpTipRestoreGoal = fpLiftGoal;
                liftDown();
            }
        } else if (!rollOk || std::fabs(roll) < FP_TIP_ROLL_CLEAR) {
            // Level again: back to the height it was going to, and on with the route
            fpTipping = false;
            fpTipTicks = 0;
            liftTo(fpTipRestoreGoal);
        }
    }

    // Lift
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
    fpTimeUp = false;
    fpLiftMode = LiftMode::idle;
    fpLiftPending = false;
    fpLiftPress = false;
    fpClawMode = ClawMode::off;
    fpFlipMode = FlipMode::off;
    fpIntakeOn = false;
    fpIntakeOffAtMs = 0;
    fpTipping = false;
    fpTipTicks = 0;
    fpAimKind = AimKind::none;
    fpAimShift = {0, 0};
    left_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    flipper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);   // off = limp, never held against its stop
    visionStart();
}

void finishSafe() {
    stopChassis(pros::MotorBrake::brake);
    intake.move_voltage(0);
    flipper.move_voltage(0);
    if (clawHolding()) claw.move_voltage(FP_CLAW_HOLD_MV);
    else claw.move_voltage(0);
    if (fpTimeUp) note("time up: lift limp");
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
        double ka = a.tangentOut > 0 ? a.tangentOut : 0.4 * chord;
        double kb = b.tangentIn > 0 ? b.tangentIn : 0.4 * chord;
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

// Top speed right now: slower with the lift up, crawling while it recovers from a tip
double speedCapNow(double& accelNow) {
    double lift = getDR4BPosition();
    double tall = clampTo((lift - FP_TALL_LIFT_LO) / (FP_TALL_LIFT_HI - FP_TALL_LIFT_LO), 0.0, 1.0);
    accelNow = FP_ACCEL + (FP_TALL_ACCEL - FP_ACCEL) * tall;
    double cap = FP_MAX_SPEED + (FP_TALL_SPEED - FP_MAX_SPEED) * tall;
    // Scoring drive: from the moment the lift is told to go up to a scoring height, until it's back down below
    // the slow height after letting go (covers lifting on the way in and lowering on the way out)
    const double slowAbove = getStackHeight(GOAL_ALLIANCE, 2) + FP_MOVING_SLOW_MARGIN;
    bool headedUp = (fpLiftMode == LiftMode::pid && fpLiftGoal > slowAbove) ||
                    ((fpLiftPending || fpLiftMode == LiftMode::hop) && fpHopThen > slowAbove);
    if (headedUp || lift > slowAbove) {
        cap = std::min(cap, FP_MOVING_SPEED);
        accelNow = std::min(accelNow, FP_MOVING_ACCEL);
    }
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
        double accelNow = FP_ACCEL;
        double target = std::min(curve.points[closest].speed, speedCapNow(accelNow));
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
                creepHold = clampTo(creep, FP_GATE_CREEP_MIN, FP_MOVING_SPEED);   // the steady speed it's creeping in at
                target = std::min(target, creepHold);
                crept = true;
            } else {
                target = std::min(target, std::sqrt(2.0 * FP_TALL_ACCEL * room));
            }
        } else if (crept) {
            // Lift is up after creeping in: keep the creep speed the rest of the way (no speed-up with the lift high)
            target = std::min(target, clampTo(creepHold, FP_GATE_CREEP_MIN, FP_MOVING_SPEED));
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
        else command = std::max(target, command - 2.0 * FP_DECEL * 0.010);
        command = std::max(command, FP_MIN_SPEED);

        // Steering: the curve's own bend a little ahead, plus a correction for being off it / pointing off it.
        // The pointing correction never gets weaker than FP_HEADING_GAIN, so it still squares up at low speed.
        int ahead = std::min(closest + static_cast<int>(command * FP_STEER_PREVIEW_S / FP_SAMPLE_STEP), n - 1);
        const Sample& here = curve.points[closest];
        const Sample& there = curve.points[ahead];
        double travelHeading = getInertialHeading() + (curve.reverse ? 180.0 : 0.0);
        Vec pathRight = rightOf(here.heading);
        double offRight = (px - here.x) * pathRight.x + (py - here.y) * pathRight.y;   // + = robot is right of the curve
        double pointOff = rad(wrap180(there.heading - travelHeading));                 // + = curve points right of the robot
        double turnRate = command * there.curvature                                     // rad/s, + = clockwise
                          - command * 2.0 * offRight / (FP_STEER_DIST * FP_STEER_DIST)
                          + std::max(2.0 * command / FP_STEER_DIST, FP_HEADING_GAIN) * pointOff;
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
        if (stalledMs >= options.stallMs) {
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
    uint32_t start = pros::millis();
    int doneMs = 0;
    while (pros::millis() - start < static_cast<uint32_t>(maxMs)) {
        service();
        if (fpTimeUp) {
            stopChassis(pros::MotorBrake::brake);
            return false;
        }
        double off = wrap180(target - getInertialHeading());   // + = turn right
        if (std::fabs(off) < FP_SPIN_DONE_DEG) {
            doneMs += 10;
            if (doneMs >= FP_SPIN_DONE_MS) break;
            stopChassis(pros::MotorBrake::brake);
        } else {
            doneMs = 0;
            double spin = rad(clampTo(FP_SPIN_GAIN * off, -FP_SPIN_MAX_DPS, FP_SPIN_MAX_DPS)) * halfTrack;
            driveChassis(sideVolts(spin, 0, 0), sideVolts(-spin, 0, 0));
        }
        pros::delay(10);
    }
    stopChassis(pros::MotorBrake::brake);
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
    if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);
    return waitMs(releaseMs);
}

bool pickUpAndScore(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY,
                    double scoreHeight, double goalX, double goalY, int goalTagId, int pinSlowMs, int scoreSlowMs, int settleMs,
                    int releaseMs, int lowerBeforeOpenMs) {
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    double grabAt = curve.nodeIndex[pinNode] * FP_SAMPLE_STEP;   // inches along the curve where the pin is in the claw
    addSlowdown(curve, grabAt, pinSlowMs, FP_PICKUP_SLOW_SPEED);
    addSlowdownAtEnd(curve, scoreSlowMs, FP_SCORE_SLOW_SPEED);
    bool grabbed = false;
    aimAt(AimKind::pin, pinX, pinY);

    MoveOptions options;
    options.gateBeforeEnd = FP_SCORE_GATE_IN;
    options.gateHold = fpGateHold;
    options.gateReady = []() { return liftReady() && !fpTipping; };
    options.tick = [&](double done, double left) {
        (void)left;
        if (!grabbed) {
            if (grabAt - done <= FP_CLAW_OPEN_DIST) {
                clawOpen();
                liftDown();        // lift pushed down onto the floor while it goes in for the pin
                liftPress(true);
            }
            if (done >= grabAt || (grabAt - done < 4.0 && clawGap(pinX, pinY) <= FP_GRAB_TOLERANCE)) {
                // The pin is in the claw: close on it, full speed for FP_CLAW_FAST_MS then the hold. The lift hops
                // off the ground like driver control's pin pickup, then carries on up to the scoring height.
                grabbed = true;
                note("claw closes on a pin");
                clawClose();
                liftPress(false);
                liftHopThen(scoreHeight);
                aimStop();
                Vec gone = toField({mclPinX, mclPinY});
                fieldMclPinPickedUp(gone.x, gone.y);   // MCL stops expecting a pin there
                if (FP_USE_GOAL_TAG && goalTagId != NO_GOAL_TAG) aimAt(AimKind::goalTag, goalX, goalY, goalTagId);   // line up on the goal's tag
            }
        }
        return true;
    };
    MoveEnd end = follow(curve, options);
    aimStop();
    if (end == MoveEnd::timeUp) return false;

    return letGo(settleMs, releaseMs, lowerBeforeOpenMs);
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

bool scoreOnGoal(const std::vector<Node>& nodes, double scoreHeight, double goalX, double goalY, int goalTagId, int scoreSlowMs,
                 int settleMs, int releaseMs, int lowerBeforeOpenMs) {
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    addSlowdownAtEnd(curve, scoreSlowMs, FP_SCORE_SLOW_SPEED);
    liftTo(scoreHeight);
    if (FP_USE_GOAL_TAG && goalTagId != NO_GOAL_TAG) aimAt(AimKind::goalTag, goalX, goalY, goalTagId);
    MoveOptions options;
    options.gateBeforeEnd = FP_SCORE_GATE_IN;
    options.gateHold = fpGateHold;
    options.gateReady = []() { return liftReady() && !fpTipping; };
    options.gateEta = [scoreHeight]() {
        double climb = scoreHeight - FP_LIFT_READY_TOLERANCE - std::max(getDR4BPosition(), 0.0);
        return std::max(climb, 0.0) / FP_LIFT_UP_DPS;
    };
    MoveEnd end = follow(curve, options);
    aimStop();
    if (end == MoveEnd::timeUp) return false;
    return letGo(settleMs, releaseMs, lowerBeforeOpenMs);
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
