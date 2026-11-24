// =====================================================================
//   4-PIN QUALIFICATION AUTON (15 s)
// =====================================================================
// The route, its spots and its waits are in this file. The driving (curve follower), the lift / claw / flipper /
// intake service, the AI Vision aiming, the tip guard and the timer are in src/auton-core.cpp (include/auton-core.hpp),
// shared with the skills run (src/skills.cpp): a change to how the robot drives or lifts applies to both.
//
// The route (drawn for RED, back on the BOTTOM wall toggle; the other three starts are the same route turned
// or flipped, see include/fourpin.hpp). It never holds two loose pins at once: the intake pin goes into the
// wall cup before the robot picks up anything else.
//   start  lift up FP_TOGGLE_LIFT_DEG and straight back down (drops the toggles), then drive off the toggle
//   1      curve left into the alliance goal, claw open, lift down; flip the preload onto the goal (alliance pin 1)
//   2      back out and curve BACKWARDS into the red pin of the 4-pin cluster (intake on the back of the robot)
//   3      forward, curving down past the left of the neutral goal into the wall cup below it, claw spinning
//          closed, into the wall for a good lock
//   4      back off the wall with the lift going up, flip the intake pin into the cup on the way, swing round
//          and drive into the NEUTRAL goal from the left, flipper back down, let go (neutral pin 2)
//   5      back out while the lift drops and swing the nose up-left, pick up the middle field pin on the way
//          through, curve left into the alliance goal from above, let go (alliance pin 2)
//   6      back out, swing the nose at the corner field pin, pick it up on the way through, curve left into
//          the alliance goal from the lower left, let go and lower (alliance pin 3). Lift limp just before 15 s.
//
// How it drives: every move is a smooth curve (no turn-in-place, no straight-line PID moves). A curve is a few
// "nodes" (a spot + the way the robot points there); the follower drives through them with the speed planned
// ahead (slower in tight curves, slowing down in time for the end). It only stops where it has to: at a goal
// (to let go), at the intake pin, against the wall at the cup, and where it switches between forwards and
// backwards (it doesn't settle there, it carries a little speed through the switch).
//
// LINING UP (the same as the skills run, src/skills.cpp): odometry + MCL drive the route, and
//   - every goal approach is lined up sideways on the goal's AprilTag (field camera) and carries on a few inches past
//     where odometry says the goal is, slowly, until the goal stops the robot (FP_GOAL_PUSH_IN)
//   - every pin pickup and the wall cup are lined up on the pin / cup (claw camera first, field camera if it has nothing)
//   - a distance-sensor reset runs at every goal while the robot is stopped there anyway
// A camera that sees nothing changes nothing: the robot just drives the odometry route. Settings: include/auton-core.hpp.
//
// LIFT (field pins, also the same as skills): it stays down after the pickup hop while the robot drives at full speed,
// goes up as late as it can, and the robot never stops to wait for it on the way in (it keeps rolling). At the goal the
// lift starts down first and the claw opens a moment later, so the pin is set down on the goal, not dropped.
//
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// All the spots below are for RED BOTTOM. The code turns / flips them for the other starts.
// =====================================================================
#include "auton-core.hpp"
#include "fourpin.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace ac;

int fieldAwareReset(double maxJump);   // src/field-mcl.cpp

// =====================================================================
//   FIELD SPOTS (red, bottom wall)
// =====================================================================
constexpr double FP_START_X = 0.0;
constexpr double FP_START_Y = -FP_FIELD_HALF + FP_CENTER_TO_BACK;   // -61.25: back on the wall
constexpr double FP_START_HEADING = 0.0;
constexpr double FP_GOAL_X = -23.55, FP_GOAL_Y = -47.09;       // alliance goal
constexpr double FP_NEUTRAL_X = 23.55, FP_NEUTRAL_Y = -47.09;  // neutral goal on our side (right of the start)
constexpr double FP_PIN2_TIP_X = 17.0, FP_PIN2_TIP_Y = -23.55; // tip of the red cluster pin that points at the goal side
constexpr double FP_PIN3_X = -23.55, FP_PIN3_Y = -23.55;       // field pin on the diagonal, near the middle
constexpr double FP_PIN4_X = -47.09, FP_PIN4_Y = -47.09;       // field pin on the diagonal, near the corner
// Wall cup: the left one of the pair below the neutral goal. It sits 2.65" left of the goal, the same as the
// cup the old route took from below the alliance goal, so the cup -> goal moves are the same shape as before.
constexpr double FP_CUP_X = 20.9, FP_CUP_Y = -68.7;
// AprilTags on the goals (-1 = any tag the goal camera sees near where the goal should be, or that goal's tag id;
// the brain screen shows each tag's id)
constexpr int FP_ALLIANCE_TAG_ID = -1;
constexpr int FP_NEUTRAL_TAG_ID = -1;

// =====================================================================
//   ROUTE SHAPE (the way the robot points at each spot, and the spots where it switches direction)
// =====================================================================
constexpr double FP_PRELOAD_HEADING = 265.0;    // pointing at the goal for the preload
// Backing into the cluster pin. 270 would be straight in line with the pin, but an 18" robot doesn't fit between
// the pin's tip and the small red pin at (0, -23.55), so it comes in at an angle with its nose below that pin.
constexpr double FP_PIN2_HEADING = 228.0;
constexpr double FP_CUP_HEADING = 140.0;        // driving into the wall cup (squarer than this and the back corner gets too close to the neutral goal)
constexpr double FP_NEUTRAL_SCORE_HEADING = 85.0;  // scoring the cup on the neutral goal (comes at it from the left)
// Inches of straight run into the neutral goal after the swing round from the wall (the lift finishes climbing
// on it). The neutral height is low, so this is much shorter than the old 16" run for the 4th alliance pin.
constexpr double FP_NEUTRAL_SCORE_RUN = 10.0;
// Backs up to here from the neutral goal, nose swung up and left toward the middle field pin
constexpr double FP_TURN3_X = 2.0, FP_TURN3_Y = -54.0, FP_TURN3_HEADING = 20.0;
constexpr double FP_PIN3_HEADING = 300.0;       // driving through pin 3 (the middle field pin)
constexpr double FP_PIN3_SCORE_HEADING = 165.0; // scoring pin 3 (comes at the goal from above)
constexpr double FP_TURN4_X = -25.4, FP_TURN4_Y = -27.1, FP_TURN4_HEADING = 222.0;   // backs up to here, nose swung at pin 4
constexpr double FP_PIN4_HEADING = 225.0;       // driving through pin 4, along the diagonal between the two red goals
constexpr double FP_PIN4_SCORE_HEADING = 65.0;  // scoring pin 4 (comes at the goal from the lower left)
constexpr double FP_CUP_LEAD_IN = 5.0;          // inches of straight run into the wall cup
// (FP_GOAL_LEAD_IN / FP_PIN_LEAD_IN / FP_PIN_LEAD_OUT: include/auton-core.hpp)
// Inches straight back before the nose starts to swing (so the front corners miss the goal)
constexpr double FP_PRELOAD_BACK_OUT = 5.0;     // from the goal after the preload
constexpr double FP_NEUTRAL_BACK_OUT = 5.0;     // from the neutral goal after the cup
constexpr double FP_PIN3_BACK_OUT = 6.0;        // from the goal after pin 3
constexpr double FP_WALL_BACK_OUT = 2.0;        // from the wall cup
constexpr double FP_START_STRAIGHT = 5.0;       // inches straight off the toggle before it starts to curve
// Inches straight away from the cluster pin before it curves toward the cup. At the cluster pin the small red pin
// at (0, -23.55) is ~2" off the robot's right side, and curving left swings the right side into it, so the robot
// drives on until the small pin is behind it first. Shorter than ~15 and it clips the small pin.
constexpr double FP_PIN2_STRAIGHT_OUT = 16.0;
constexpr double FP_PIN2_SWING_TANGENT = 4.0;   // how gently the curve leaves that straight (lower = turns sooner after it)

// Every goal approach drives on this far past where the goal should be, at FP_CONTACT_SPEED, until the goal stops
// the robot (covers odometry reading a little long: "didn't drive far enough in"). 0 = stops where odometry says.
constexpr double FP_PRELOAD_PUSH_IN = 3.0;      // preload into the alliance goal (the same as skills)
constexpr double FP_GOAL_PUSH_IN = 2.5;         // every other score (the same as skills)
// Speed over the last FP_APPROACH_IN inches into the alliance goal with a field pin, by how many things are on the
// goal after this one (the same numbers as skills: the taller the stack, the more a bump matters)
constexpr double FP_APPROACH_SPEED[] = {
    46.0,    // 1st thing on the goal (the preload: not used, it's flipped on)
    42.0,    // 2nd: middle field pin
    36.0,    // 3rd: corner field pin
};
constexpr double FP_APPROACH_IN = 10.0;
// The wall cup is lined up with the cameras on the way in, like the pin in the matchloader in skills. false = odometry only.
// There are two cups side by side there, 5.3" apart. Only a cup within FP_CUP_WINDOW_IN (sideways) of where ours
// should be is lined up on: less than half the gap, so the other cup can never pull the robot across to it.
constexpr bool FP_CUP_USE_CAMERA = true;
constexpr double FP_CUP_WINDOW_IN = 2.5;

// Fine tuning per spot (inches, + = further forward)
constexpr double FP_PRELOAD_EXTRA = 0.0;
constexpr double FP_PIN2_EXTRA = 4.0;           // + = backs further into the cluster pin (the auton line guard still stops it)
constexpr double FP_NEUTRAL_SCORE_EXTRA = 1.0;
constexpr double FP_PIN3_GRAB_EXTRA = 0.0;
constexpr double FP_PIN3_SCORE_EXTRA = 0.0;
constexpr double FP_PIN4_GRAB_EXTRA = 0.0;
constexpr double FP_PIN4_SCORE_EXTRA = 0.0;

// The auton line (the double diagonal line through the cluster). The back corners stay at least this far on
// our side of it while backing into the cluster pin; if one gets closer the robot stops backing right there.
constexpr double FP_LINE_MARGIN = 1.5;

// =====================================================================
//   WAITS AND SLOWDOWNS (all the timing knobs in one place)
// =====================================================================
// ---- STOPS: the robot sits still this long (ms) ----
constexpr int FP_PRELOAD_FLIP_WAIT_MS = 500;     // at the alliance goal while the preload flips, then backs away
constexpr int FP_INTAKE_DWELL_MS = 600;          // at the cluster pin while the intake takes it
constexpr int FP_WALL_PUSH_MS = 600;             // pushing into the wall on the cup while the claw locks on...
constexpr int FP_CUP_EXTRA_WAIT_MS = 150;        // ...then stopped this much longer (claw still squeezing) before backing off
// Letting go at a goal (the same as skills): the lift starts down, FP_LOWER_BEFORE_OPEN_MS later the claw opens (the
// pin is set down, not dropped), and the release time after that it backs away with the lift still coming down.
// (It used to open the claw with the lift held up and wait 300 ms.)
constexpr int FP_LOWER_BEFORE_OPEN_MS = 100;     // alliance goal, both field pins. 0 = claw opens with the lift held up
constexpr int FP_NEUTRAL_LOWER_BEFORE_OPEN_MS = 100;   // neutral goal (the cup). 0 = claw opens with the lift held up
constexpr int FP_NEUTRAL_RELEASE_MS = 150;       // at the neutral goal after the claw starts opening
constexpr int FP_ALLIANCE_RELEASE_MS = 150;      // at the alliance goal after the claw starts opening (both field pins)
// Settle: stopped at the goal this long BEFORE the claw opens (lets the lift finish / stop bouncing). 0 = lets go
// as soon as it arrives.
constexpr int FP_NEUTRAL_SETTLE_MS = 0;          // cup on the neutral goal
constexpr int FP_PIN3_SETTLE_MS = 0;             // middle field pin on the alliance goal
constexpr int FP_PIN4_SETTLE_MS = 0;             // corner field pin on the alliance goal

// ---- SLOWDOWNS: never a stop. The robot drives the last stretch before a spot at a slow speed instead. ----
// The number is how long (ms) it spends at that slow speed before the spot: 0 = off (normal speed, like now),
// higher = it drops to the slow speed earlier. The slow stretch is (slow speed x time) inches long, and the
// robot brakes down to the slow speed before it, the same as for any other slow part of a curve.
// (The slow speeds themselves, FP_PICKUP_SLOW_SPEED / FP_SCORE_SLOW_SPEED, are in include/auton-core.hpp.)
// Into pins / the cup (more time to line up and grab):
constexpr int FP_CLUSTER_PIN_SLOW_MS = 0;        // backing into the cluster pin (intake)
constexpr int FP_CUP_SLOW_MS = 0;                // into the wall cup
constexpr int FP_PIN3_SLOW_MS = 0;               // through the middle field pin
constexpr int FP_PIN4_SLOW_MS = 0;               // through the corner field pin
// Into a goal before scoring (e.g. the lift can't reach its height fast enough: this gives it more time while the
// robot keeps moving, instead of stopping at the lift gate):
constexpr int FP_PRELOAD_SCORE_SLOW_MS = 0;      // preload into the alliance goal
constexpr int FP_NEUTRAL_SCORE_SLOW_MS = 0;      // cup into the neutral goal (at most the 10" run-in after the swing round)
constexpr int FP_PIN3_SCORE_SLOW_MS = 0;         // middle field pin into the alliance goal
constexpr int FP_PIN4_SCORE_SLOW_MS = 0;         // corner field pin into the alliance goal

// =====================================================================
//   LIFT
// =====================================================================
constexpr double FP_TOGGLE_LIFT_DEG = 50.0;      // start: lift goes up this far to drop the toggles...
constexpr int FP_TOGGLE_UP_MAX_MS = 400;         // ...(gives up after this long)
constexpr double FP_TOGGLE_GO_DEG = 20.0;        // ...and the robot drives off once it's back down to this
constexpr int FP_TOGGLE_DOWN_MAX_MS = 400;
// ---- SCORING HEIGHTS (lift degrees). The same numbers on all four starts. ----
// The alliance and neutral goals start at different heights, so each has its own list, one entry per thing on
// the goal (entry 1 = the 1st thing on it). Read a height with the lift readout (CONTROLLER_READOUT 0, "L").
// Alliance: pin 1 is the preload, flipped on with the lift DOWN (no height used). Pins 2-3 = the two field pins.
constexpr double FP_ALLIANCE_HEIGHTS[] = {
    0.0,      // pin 1: preload (lift down, flipper puts it on)
    240.0,    // pin 2: middle field pin   (= ALLIANCE_HEIGHTS[1] in main.cpp)
    315.0,    // pin 3: corner field pin   (= ALLIANCE_HEIGHTS[2] in main.cpp)
};
// Neutral: the wall cup (with the intake pin flipped into it) goes on as the 2nd thing.
constexpr double FP_NEUTRAL_HEIGHTS[] = {
    0.0,      // pin 1: not scored by this auton
    85.0,     // pin 2: wall cup + intake pin   PLACEHOLDER: MEASURE THIS (old untested neutral guess)
};
constexpr int FP_ALLIANCE_FIELD_PIN3_SPOT = 2;   // middle field pin -> alliance pin 2
constexpr int FP_ALLIANCE_FIELD_PIN4_SPOT = 3;   // corner field pin -> alliance pin 3
constexpr int FP_NEUTRAL_CUP_SPOT = 2;           // wall cup -> neutral pin 2
constexpr double FP_CUP_FLIP_LIFT_DEG = 30.0;    // the pin flips into the cup once the lift has climbed this far (it keeps going up)
constexpr int FP_CUP_FLIP_MIN_MS = 450;          // the flipper stays up at least this long so the pin lands in the cup before it comes back down

// =====================================================================
//   CLAW, FLIPPER, INTAKE (route timing; the mechanisms themselves are tuned in include/auton-core.hpp)
// =====================================================================
constexpr double FP_PRELOAD_CLAW_OPEN_AFTER_IN = 6.0;   // claw starts opening once it has driven this far off the toggle
constexpr double FP_FLIPPER_DOWN_AFTER_IN = 5.0; // after the preload, flipper goes back down once backed this far (after lift down + claw off)
constexpr double FP_FLIPPER_BACK_BEFORE_IN = 7.0;   // neutral score: flipper goes back down this far before the goal (once the lift is up)
constexpr double FP_INTAKE_ON_DIST = 14.0;       // intake starts this many inches before the cluster pin
constexpr int FP_INTAKE_KEEP_MS = 6000;          // intake keeps running this long after it leaves the pin (it also stops when the pin is flipped into the cup)
constexpr double FP_CUP_CLAW_ON_DIST = 15.0;     // claw starts spinning closed this many inches before the wall cup
constexpr int FP_WALL_PUSH_MV = 6000;            // push into the wall on the cup (for FP_WALL_PUSH_MS)
// The drive into the wall ends when the robot stops moving (it's on the wall). Backup: it also ends once odometry
// says the front corner is this far PAST the wall, which can only mean it's there.
constexpr double FP_WALL_OVERDRIVE = 1.0;

// =====================================================================
//   15 SECOND TIMER
// =====================================================================
constexpr int FP_AUTON_MS = 15000;

namespace {

// Lift height for a spot on a goal, from the height lists at the top (1 = 1st thing on the goal). Kept above the
// lift PID's bottom deadzone, the same as getStackHeight() in main.cpp.
template <size_t N>
double heightFor(const double (&list)[N], int spot) {
    double height = list[std::min(std::max(spot, 1), static_cast<int>(N)) - 1];
    return std::max(height, DR4B_BOTTOM_DEADZONE + 1.0);
}
double allianceHeight(int spot) { return heightFor(FP_ALLIANCE_HEIGHTS, spot); }
double neutralHeight(int spot) { return heightFor(FP_NEUTRAL_HEIGHTS, spot); }

// How a field pin is scored as the spot-th thing on the alliance goal: the same plan the skills run uses
ScorePlan alliancePlan(int spot, int scoreSlowMs, int settleMs) {
    const int spots = static_cast<int>(sizeof(FP_APPROACH_SPEED) / sizeof(FP_APPROACH_SPEED[0]));
    ScorePlan plan;
    plan.height = allianceHeight(spot);
    plan.goalX = FP_GOAL_X;
    plan.goalY = FP_GOAL_Y;
    plan.tagId = FP_ALLIANCE_TAG_ID;
    plan.scoreSlowMs = scoreSlowMs;
    plan.settleMs = settleMs;
    plan.releaseMs = FP_ALLIANCE_RELEASE_MS;
    plan.lowerBeforeOpenMs = FP_LOWER_BEFORE_OPEN_MS;
    plan.liftLate = true;   // lift down while it drives, up as late as it can be
    plan.approachSpeed = FP_APPROACH_SPEED[std::min(std::max(spot, 1), spots) - 1];
    plan.approachIn = FP_APPROACH_IN;
    plan.pushIn = FP_GOAL_PUSH_IN;
    return plan;
}

// For a curve that carries on `push` inches past the spot where the claw is on a goal: any old-style slowdown ends at
// the goal (not at the end of the push), the robot meets the goal at FP_CONTACT_SPEED, and touching it ends the move.
void pushIntoGoal(Curve& curve, MoveOptions& options, double push, int scoreSlowMs) {
    addSlowdown(curve, curve.length() - push, scoreSlowMs, FP_SCORE_SLOW_SPEED);
    if (push <= 0.0) return;
    addSlowdownAtEnd(curve, static_cast<int>(std::lround((push + 1.5) / FP_CONTACT_SPEED * 1000.0)), FP_CONTACT_SPEED);
    options.endStallMs = FP_CONTACT_STALL_MS;   // on the goal = there
    options.endStallIn = push + 1.5;
}

// =====================================================================
//   THE ROUTE
// =====================================================================
void runFourPin(bool turned, bool mirrored) {
    setRouteFrame(turned, mirrored);
    coreStart("4pin", FP_AUTON_MS);
    setGateHold(false);   // field pins: never stops for the lift before the goal, it keeps rolling in while the lift finishes

    // Where the robot starts: back on the wall at the toggle
    Vec startSpot = toField({FP_START_X, FP_START_Y});
    setPose(startSpot.x, startSpot.y, headingToField(FP_START_HEADING));

    // The poses the route is built from (red bottom drawing)
    const Node preloadPose = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PRELOAD_HEADING, FP_CLAW_REACH + FP_PRELOAD_EXTRA);
    Vec pin2Back = forwardOf(FP_PIN2_HEADING);
    const Node pin2Pose = node(FP_PIN2_TIP_X + pin2Back.x * (FP_INTAKE_REACH - FP_PIN2_EXTRA),
                               FP_PIN2_TIP_Y + pin2Back.y * (FP_INTAKE_REACH - FP_PIN2_EXTRA), FP_PIN2_HEADING);
    const Node cupPose = clawOn(FP_CUP_X, FP_CUP_Y, FP_CUP_HEADING, FP_CLAW_REACH);
    const Node neutralScore = clawOn(FP_NEUTRAL_X, FP_NEUTRAL_Y, FP_NEUTRAL_SCORE_HEADING, FP_CLAW_REACH + FP_NEUTRAL_SCORE_EXTRA);
    const Node turn3 = node(FP_TURN3_X, FP_TURN3_Y, FP_TURN3_HEADING);
    const Node pin3Pose = clawOn(FP_PIN3_X, FP_PIN3_Y, FP_PIN3_HEADING, FP_CLAW_REACH + FP_PIN3_GRAB_EXTRA);
    const Node pin3Score = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PIN3_SCORE_HEADING, FP_CLAW_REACH + FP_PIN3_SCORE_EXTRA);
    const Node turn4 = node(FP_TURN4_X, FP_TURN4_Y, FP_TURN4_HEADING);
    const Node pin4Pose = clawOn(FP_PIN4_X, FP_PIN4_Y, FP_PIN4_HEADING, FP_CLAW_REACH + FP_PIN4_GRAB_EXTRA);
    const Node pin4Score = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PIN4_SCORE_HEADING, FP_CLAW_REACH + FP_PIN4_SCORE_EXTRA);

    // ---------------- Start: lift up and straight back down (the toggles drop onto the field toggle) ----------------
    liftRaiseDirect(FP_TOGGLE_LIFT_DEG, FP_TOGGLE_UP_MAX_MS);
    note("lift up for the toggles");
    liftDown();
    {
        uint32_t t0 = pros::millis();
        while (getDR4BPosition() > FP_TOGGLE_GO_DEG && pros::millis() - t0 < static_cast<uint32_t>(FP_TOGGLE_DOWN_MAX_MS)) {
            service();
            pros::delay(10);
        }
    }

    // ---------------- 1. Off the toggle and curve into the goal, claw open, lift down. Flip the preload on. ----------------
    {
        Node start = hereNode();
        start.heading = FP_START_HEADING;   // straight off the wall
        // Onto the goal and on until the goal stops it
        Curve curve = makeCurve({start, along(start, FP_START_STRAIGHT, 0, 6.0), along(preloadPose, -FP_GOAL_LEAD_IN, 8.0, 0), preloadPose,
                                 along(preloadPose, FP_PRELOAD_PUSH_IN)},
                                false, FP_MAX_SPEED, 0.0);
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, FP_GOAL_X, FP_GOAL_Y, FP_ALLIANCE_TAG_ID);   // line up on the goal's tag
        MoveOptions options;
        pushIntoGoal(curve, options, FP_PRELOAD_PUSH_IN, FP_PRELOAD_SCORE_SLOW_MS);
        options.tick = [&](double done, double left) {
            (void)left;
            if (done >= FP_PRELOAD_CLAW_OPEN_AFTER_IN) clawOpen();   // opens while it curves
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at the goal (preload)");
        aimStop();
        clawOpen();
        stopChassis(pros::MotorBrake::brake);
        flipperUp();   // stopped at the goal: flip, wait, then back away
        service();     // (the flipper and the intake get going before the reset below, which takes up to 0.12 s)
        const uint32_t arrived = pros::millis();
        if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);   // standing still: the distance sensors check the position
        if (!waitMs(std::max(FP_PRELOAD_FLIP_WAIT_MS - static_cast<int>(pros::millis() - arrived), 0))) return;
        note("preload flipped");
    }

    // ---------------- 2. Backwards into the red pin of the cluster (intake) ----------------
    {
        Node start = hereNode();
        Node clear = along(start, -FP_PRELOAD_BACK_OUT, 0, 7.0);   // straight back the way it's really pointing first
        Node end = pin2Pose;
        end.tangentIn = 9.0;
        end.speed = FP_INTAKE_SPEED;
        Curve curve = makeCurve({start, clear, end}, true, FP_MAX_SPEED, 0.0);
        addSlowdownAtEnd(curve, FP_CLUSTER_PIN_SLOW_MS, FP_PICKUP_SLOW_SPEED);
        MoveOptions options;
        options.tick = [&](double done, double left) {
            if (done >= 1.0) clawOff();                                    // claw motor off: it closes by itself
            if (done >= 3.0) liftDown();                                   // lift held down for the intake
            if (done >= FP_FLIPPER_DOWN_AFTER_IN) flipperDown();           // then the flipper back to the intake position
            if (left <= FP_INTAKE_ON_DIST) {
                intakeOn();
                liftPress(true);   // lift pushed down while the intake takes the pin
            }
            if (left <= 8.0 && backCornerLineGap() < FP_LINE_MARGIN) return false;   // never over the auton line
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at the cluster pin");
        flipperDown();
        intakeOn();
        if (!waitMs(FP_INTAKE_DWELL_MS)) return;
        intakeOffAfter(FP_INTAKE_KEEP_MS);
        liftPress(false);
    }

    // ---------------- 3. Forward, curving down past the left of the neutral goal into the wall cup ----------------
    // One curve from the cluster pin: straight away from the small red pin, swing left round the neutral goal, and
    // into the cup and the wall with the claw spinning closed. The intake still has the cluster pin.
    {
        Node end = cupPose;
        end.speed = FP_WALL_SPEED;
        std::vector<Node> nodes = {
            hereNode(),
            along(pin2Pose, FP_PIN2_STRAIGHT_OUT, 0, FP_PIN2_SWING_TANGENT),
            along(cupPose, -FP_CUP_LEAD_IN),
            end,
        };
        Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, FP_WALL_SPEED);
        addSlowdownAtEnd(curve, FP_CUP_SLOW_MS, FP_PICKUP_SLOW_SPEED);
        if (FP_CUP_USE_CAMERA) aimAt(AimKind::pin, FP_CUP_X, FP_CUP_Y, -1, FP_CUP_WINDOW_IN);   // the cameras line the cup up with the claw
        MoveOptions options;
        options.brakeAtEnd = false;
        options.stallMs = 100;   // against the wall = there
        options.tick = [&](double done, double left) {
            (void)done;
            if (left <= FP_CUP_CLAW_ON_DIST) {
                clawClose();       // spinning closed the rest of the way in (not opened for the cup)
                liftPress(true);   // lift pushed down onto the cup
            }
            liftDown();
#ifdef RW_SIM_PROS
            // The simulator's wall only stops the middle of the robot, so a front corner would drive through it.
            // This ends the move where a real wall would stop the robot.
            if (frontCornerPastWall() >= 0.0) return false;
#endif
            return frontCornerPastWall() < FP_WALL_OVERDRIVE;
        };
        MoveEnd ended = follow(curve, options);   // arrived or stalled on the wall: both are fine
        aimStop();
        if (ended == MoveEnd::timeUp) return;
        // Into the wall for a good lock on the cup
        note("at the wall cup");
        clawCloseAgain();
        liftHopThen(-1.0);   // hop it off the ground like driver control's pin pickup (holds there until the flip height)
        uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(FP_WALL_PUSH_MS)) {
#ifdef RW_SIM_PROS
            bool wallThere = frontCornerPastWall() >= 0.0;   // simulator stand-in for the wall (see above)
#else
            bool wallThere = frontCornerPastWall() >= FP_WALL_OVERDRIVE;
#endif
            if (!wallThere) driveChassis(FP_WALL_PUSH_MV / 1000.0, FP_WALL_PUSH_MV / 1000.0);
            else stopChassis(pros::MotorBrake::brake);
            service();
            if (timeUp()) { stopChassis(pros::MotorBrake::brake); return; }
            pros::delay(10);
        }
        stopChassis(pros::MotorBrake::brake);
        if (!waitMs(FP_CUP_EXTRA_WAIT_MS)) return;   // a little longer on the cup, claw still squeezing
    }

    // ---------------- 4. Back off the wall with the lift going up, flip the pin into the cup, swing round into the neutral goal ----------------
    const double neutralLift = neutralHeight(FP_NEUTRAL_CUP_SPOT);
    // Flip once the lift has climbed a little (never above the neutral height itself, or it would never flip)
    const double flipAtLift = std::min(FP_CUP_FLIP_LIFT_DEG, neutralLift - FP_LIFT_READY_TOLERANCE);
    bool flipStarted = false;
    uint32_t flipStartMs = 0;
    auto startFlip = [&]() {
        if (flipStarted) return;
        flipStarted = true;
        flipStartMs = pros::millis();
        note("flip the pin into the cup");
        flipperUp();
    };
    auto cupSequence = [&]() {
        if (liftClimbingPast(flipAtLift)) startFlip();
    };
    auto flipDone = [&]() {
        return flipStarted && pros::millis() - flipStartMs >= static_cast<uint32_t>(FP_CUP_FLIP_MIN_MS);
    };
    {
        // Straight off the wall, then the back swings left so the nose comes round to point at the neutral goal
        Node start = hereNode();
        Curve curve = makeCurve({start, along(start, -FP_WALL_BACK_OUT), along(neutralScore, -FP_NEUTRAL_SCORE_RUN)},
                                true, FP_MAX_SPEED, FP_CUSP_SPEED);
        liftTo(neutralLift);
        MoveOptions options;
        options.brakeAtEnd = false;
        options.tick = [&](double done, double left) {
            (void)done; (void)left;
            cupSequence();
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("swung round to the neutral goal");
    }
    {
        // Onto the goal and on until the goal stops it
        Curve curve = makeCurve({hereNode(), along(neutralScore, -FP_GOAL_LEAD_IN), neutralScore, along(neutralScore, FP_GOAL_PUSH_IN)},
                                false, FP_MAX_SPEED, 0.0);
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, FP_NEUTRAL_X, FP_NEUTRAL_Y, FP_NEUTRAL_TAG_ID);   // line up on the goal's tag
        MoveOptions options;
        pushIntoGoal(curve, options, FP_GOAL_PUSH_IN, FP_NEUTRAL_SCORE_SLOW_MS);
        // This one still stops FP_SCORE_GATE_IN before the goal if the lift isn't up or the pin isn't in the cup yet
        options.gateBeforeEnd = FP_SCORE_GATE_IN + FP_GOAL_PUSH_IN;
        options.gateReady = [&]() { return liftReady() && flipDone() && !tipping(); };
        options.gateEta = [&]() {
            // Whichever is later: the rest of the lift's climb, or the rest of the flip
            double climb = neutralLift - FP_LIFT_READY_TOLERANCE - std::max(getDR4BPosition(), 0.0);
            double flipLeftMs = flipStarted ? FP_CUP_FLIP_MIN_MS - static_cast<double>(pros::millis() - flipStartMs) : FP_CUP_FLIP_MIN_MS;
            return std::max({std::max(climb, 0.0) / FP_LIFT_UP_DPS, flipLeftMs / 1000.0, 0.0});
        };
        options.tick = [&](double done, double left) {
            (void)done;
            cupSequence();
            if (liftReady() && flipDone() && left <= FP_FLIPPER_BACK_BEFORE_IN + FP_GOAL_PUSH_IN) flipperDown();   // out of the way right before scoring
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at the neutral goal");
        aimStop();
        startFlip();   // (only if the lift never got to the flip height, e.g. after a tip)
        while (!flipDone()) {
            if (!waitMs(10)) return;
        }
        if (!waitMs(FP_NEUTRAL_SETTLE_MS)) return;
        flipperDown();
        // Lift starts down, claw opens, the distance sensors check the position while it's standing there
        if (!letGo(0, FP_NEUTRAL_RELEASE_MS, FP_NEUTRAL_LOWER_BEFORE_OPEN_MS)) return;
    }

    // ---------------- 5. Back out with the nose swinging up-left, through pin 3, left into the alliance goal ----------------
    if (!backOutTo(FP_NEUTRAL_BACK_OUT, turn3)) return;
    {
        std::vector<Node> nodes = {
            hereNode(),
            along(pin3Pose, -FP_PIN_LEAD_IN, 8.0, 0),
            node(pin3Pose.x, pin3Pose.y, pin3Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(pin3Pose, FP_PIN_LEAD_OUT, 0, 7.0),
            along(pin3Score, -FP_GOAL_LEAD_IN, 7.0, 0),
            pin3Score,
        };
        if (!pickUpAndScoreWith(nodes, 2, FP_PIN3_X, FP_PIN3_Y, -24.0, -24.0, FP_PIN3_SLOW_MS,
                                alliancePlan(FP_ALLIANCE_FIELD_PIN3_SPOT, FP_PIN3_SCORE_SLOW_MS, FP_PIN3_SETTLE_MS))) return;
    }

    // ---------------- 6. Back out, nose at pin 4, through pin 4, left into the alliance goal, lower ----------------
    if (!backOutTo(FP_PIN3_BACK_OUT, turn4)) return;
    {
        std::vector<Node> nodes = {
            hereNode(),
            along(pin4Pose, -FP_PIN_LEAD_IN),
            node(pin4Pose.x, pin4Pose.y, pin4Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(pin4Pose, FP_PIN_LEAD_OUT, 0, 8.0),
            along(pin4Score, -FP_GOAL_LEAD_IN, 8.0, 0),
            pin4Score,
        };
        if (!pickUpAndScoreWith(nodes, 2, FP_PIN4_X, FP_PIN4_Y, -48.0, -48.0, FP_PIN4_SLOW_MS,
                                alliancePlan(FP_ALLIANCE_FIELD_PIN4_SPOT, FP_PIN4_SCORE_SLOW_MS, FP_PIN4_SETTLE_MS))) return;
        liftDown();   // no backing up: just lower
        while (!liftIsDown()) {
            if (!waitMs(10)) return;   // out of time: the lift is already limp and falls by itself
        }
        clawOff();
        liftCoast();   // PID stays off until driver control turns it back on (opcontrol does)
        note("done");
    }
}

}  // namespace

// Runs the route, then leaves everything safe however it ended (finished, or the 15 s ran out part way):
// drive stopped, intake and flipper off, claw off or on its light hold, lift limp if time ran out.
static void fourPin(bool turned, bool mirrored) {
    runFourPin(turned, mirrored);
    finishSafe();
}

// The four starts (see include/fourpin.hpp)
void fourPinRedBottom() { fourPin(false, false); }
void fourPinBlueTop() { fourPin(true, false); }
void fourPinRedLeft() { fourPin(false, true); }
void fourPinBlueRight() { fourPin(true, true); }
