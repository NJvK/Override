// =====================================================================
//   SKILLS RUN (1 minute)
// =====================================================================
// Red, back on the LEFT wall toggle (the middle of the left wall), facing right. Everything is in FIELD coordinates:
// (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// The driving, lift / claw / flipper service, AI Vision aiming and the timer are in src/auton-core.cpp
// (include/auton-core.hpp), shared with the 4-pin qual auton.
//
// The route:
//   start    lift up SK_TOGGLE_LIFT_DEG and back down, then a full-power 10" shove forward (the first toggle)
//   1        curve into goal A (the red goal nearest the start) from above, claw open; flip the preload on, hold 1 s
//   2, 3     straight back with a small swing, curve through the field pin near the middle (P1), round onto goal A
//   4, 5     straight back, turn, straight through the field pin near the corner (P2), hook round onto goal A
//   6        matchloader (bottom-left corner) -> goal A, SK_GOAL_A_LOADS times: goal A ends with 5
//   7        matchloader -> goal B (the other red goal), SK_GOAL_B_LOADS times: goal B ends with 5
//   end      backs round goal B in one curve onto the BOTTOM wall toggle (lift up SK_TOGGLE2_LIFT_DEG just before the
//            wall, down to SK_TOGGLE2_DROP_DEG the moment it's on it), then straight forward into the middle zone
//
// HOW THE LOADER TRIPS MOVE: straight lines and one turn in place at each end, lift down the whole way.
//   loader -> goal   straight back out of the loader (SK_LOADER_BACK_OUT), turn to face the goal, straight in. The
//                    lift stays at the carry height until the goal is close (ScorePlan::liftLate).
//   goal -> loader   straight back down the same line to the turn spot on the loader line, turn to face the loader,
//                    straight in (goal B can also turn round and drive there forwards: SK_GOAL_B_REVERSE_TO_LOADER)
// (They used to be backwards loops that swung the nose round, 270 degrees at the loader, with the lift already up:
// slow under the lift's speed cap, and a few inches of odometry error bent the loops out of shape.)
//
// LINING UP: every goal approach is lined up sideways on the goal's AprilTag (field camera) and carries on
// SK_GOAL_PUSH_IN past where odometry says the goal is, until the goal stops the robot. Every pin pickup is lined up
// on the pin (both cameras). A distance-sensor reset runs at every goal and at the loader while the robot is stopped.
//
// TIME: before every loader trip the robot checks that the trip plus the toggle and the park still fit
// (SK_SIM_... below, scaled by how much slower than the simulator its last trip really was). If not, it skips the
// rest of the loads and goes for the toggle and the park. In the last SK_DASH_TURN_MS + SK_DASH_DRIVE_MS, wherever
// it is, it points an end at the zone and drives flat out so it's carried in.
// Simulator (sh sim/run-sim.sh --start -61.85,0,90 --time 60 --size 18): 5 + 5, toggle and parked at 51.3 s. With
// a slower, slippier drive (--profile sim/slow-robot-profile.txt) the same, parked at 57.1 s.
// =====================================================================
#include "auton-core.hpp"
#include "skills.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace ac;

int fieldAwareReset(double maxJump);   // src/field-mcl.cpp

// =====================================================================
//   TIMER + HOW MANY
// =====================================================================
constexpr int SK_AUTON_MS = 60000;
constexpr int SK_GOAL_A_LOADS = 2;    // matchloads onto goal A after the preload + 2 field pins: 5 on goal A
constexpr int SK_GOAL_B_LOADS = 5;    // matchloads onto goal B: 5 on goal B
// The time guard. Simulator times (ms) for one loader trip (leaving a goal -> the loader -> let go on the goal) and
// for the end (leaving a goal -> the toggle -> parked). The robot is slower than the simulator: each real trip is
// timed and the next estimates are scaled by (real / simulator), starting from SK_SLOWER_THAN_SIM.
// A trip takes longer the taller the stack it's adding to (the lift has further to go): SK_SIM_TRIP_MS is a trip
// onto an empty goal, plus SK_SIM_TRIP_PER_SPOT_MS for each thing already on it.
constexpr int SK_SIM_TRIP_MS = 5000;           // goal -> loader -> let go on a goal with nothing on it
constexpr int SK_SIM_TRIP_PER_SPOT_MS = 180;   // more for each thing already on the goal
constexpr int SK_SIM_END_FROM_A_MS = 5100;     // goal A -> toggle -> parked
constexpr int SK_SIM_END_FROM_B_MS = 4750;     // goal B -> toggle -> parked
constexpr double SK_SLOWER_THAN_SIM = 1.20;    // first guess, until the first loader trip has been timed
constexpr int SK_GUARD_SPARE_MS = 400;         // spare on top of the estimate (the last-moment dash is the net under it)
constexpr bool SK_USE_TIME_GUARD = true;       // false = never skips a load (whatever isn't done at 60 s doesn't happen)
// The last moment: the robot stops what it's doing this long before the end, turns an end toward the zone if neither
// end points into it (SK_DASH_TURN_MS), and drives flat out for the last SK_DASH_DRIVE_MS
constexpr int SK_DASH_TURN_MS = 100;
constexpr int SK_DASH_DRIVE_MS = 150;

// =====================================================================
//   FIELD SPOTS
// =====================================================================
constexpr double SK_START_BACK = 8.4;                               // measured: the robot's center is 8.4" from the wall at the start
constexpr double SK_START_X = -FP_FIELD_HALF + SK_START_BACK;       // -61.85: back on the left wall toggle
constexpr double SK_START_Y = 0.0;
constexpr double SK_START_HEADING = 90.0;                           // facing right
constexpr double SK_GOAL_A_X = -47.09, SK_GOAL_A_Y = -23.55;   // red goal nearest the start (the preload goal)
constexpr double SK_GOAL_B_X = -23.55, SK_GOAL_B_Y = -47.09;   // the other red goal (bottom)
constexpr double SK_P1_X = -23.55, SK_P1_Y = -23.55;           // field pin on the diagonal, near the middle
constexpr double SK_P2_X = -47.09, SK_P2_Y = -47.09;           // field pin on the diagonal, near the corner
// AprilTags on the goals (field camera). -1 = any tag it sees near where that goal should be. The brain screen
// shows each tag's id when the camera looks at it.
constexpr int SK_GOAL_A_TAG = 2;      // "red goal apriltag 2"
constexpr int SK_GOAL_B_TAG = 3;      // "goal b is apriltag 3"
// Matchloader (bottom-left corner, on the left wall): the robot drives WEST along the loader line into it
constexpr double SK_LOADER_DEPTH = 4.0;                      // the loader's face is this far out from the wall
constexpr double SK_LOADER_Y = -59.0;                        // the loader's center (59" from the field's center line) = the line it drives in on
constexpr double SK_LOADER_PIN_X = -FP_FIELD_HALF + SK_LOADER_DEPTH / 2.0;   // -68.25: the next pin sits in the middle of the loader
// How deep it drives in. The claw sticks out FP_CLAW_TIP_FORWARD (10.5") from the center and the loader is only 4" deep,
// so the claw's tips reach the wall behind the loader with the flex wheels about on the pin: the drive aims the claw's
// tips at the wall, and whatever stops the robot first (the loader or the wall) ends the move.
constexpr double SK_LOADER_EXTRA = 2.5;                      // + = aims this much deeper still ("go an extra 2 inches into the matchloader")
constexpr double SK_LOADER_GRAB_X = -FP_FIELD_HALF + FP_CLAW_TIP_FORWARD - SK_LOADER_EXTRA;   // -62.25: robot center when loading
constexpr double SK_LOADER_RUN_IN = 16.0;                    // inches of dead-straight drive along the line into the loader if it ever
                                                             // comes in from further out than the turn spot (it doesn't on this route)
constexpr double SK_LOADER_IN_SPEED = FP_WALL_SPEED;         // in/s for the last stretch into the loader: the 4-pin's speed into the wall cup (26)...
constexpr int SK_LOADER_SLOW_MS = 300;                       // ...for this long (26 in/s x 0.3 s = the last 8")
constexpr int SK_LOADER_IN_MAX_MS = 1300;                    // the drive into the loader never takes longer than this (it's about 0.7 s:
                                                             // pressed on the loader but still creeping sideways used to wait out 2+ s)
constexpr double SK_LOADER_BACK_OUT = 11.0;                  // straight back out along the line before it turns. 8 clears the loader's
                                                             // face; 11 also gives the drive back IN enough straight to square up on the
                                                             // line (at 8 it met the loader 6-14 degrees off when it had drifted an inch)
constexpr double SK_LOADER_CLAW_ON_DIST = 14.0;              // claw starts spinning closed this far before the loader
constexpr double SK_LOADER_CARRY_DEG = 20.0;                 // lift height it carries the matchload at (off the floor, no more)
// Where it turns between the loader line and the goal lines: straight out from the loader
constexpr double SK_TURN_X = SK_LOADER_GRAB_X + SK_LOADER_BACK_OUT;   // -51.25
constexpr double SK_TURN_Y = SK_LOADER_Y;
// Coming back from a goal it backs to a spot this far ABOVE the loader line and turns there. It's backing toward the
// bottom wall, and a robot that brakes a little late ends up past the spot: this way that puts it on the line, not
// 2" below it with a back corner in the wall when it turns. The drive into the loader takes up what's left.
constexpr double SK_TURN_ABOVE_LINE = 1.5;
// "Stalled against the loader the robot is square to the wall, so its heading IS 270": that was the idea, and
// odometry's heading used to be set to 270 at the loader every trip. OFF now. Only the claw's tips (7" apart) touch
// anything there, which doesn't square a robot, and the video run met the loader at an angle: calling that 270 turned
// the whole rest of the run by the difference (the last goal B trips went to the wrong spot). The IMU with its
// measured imu_scale is the better heading. If it's ever turned back on, it only acts when the IMU already agrees to
// within SK_SQUARE_MAX_DEG.
constexpr bool SK_SQUARE_AT_LOADER = false;
constexpr double SK_LOADER_HEADING = 270.0;
constexpr double SK_SQUARE_MAX_DEG = 6.0;
constexpr double SK_WALL_SQUARE_MAX_DEG = 15.0;              // same idea, backed flat onto the bottom wall at the second toggle
// The second toggle: the middle of the BOTTOM wall (on goal B's side). The robot backs onto it, like the start.
constexpr double SK_TOGGLE2_X = 0.0;
constexpr double SK_TOGGLE2_Y = -FP_FIELD_HALF + SK_START_BACK;   // -61.85: the robot's center with its back on the wall
// The middle zone (where it parks), and where the robot's center stops in it coming up from the bottom toggle: the
// claw's tips end 1.5" short of the middle goal's base
constexpr double SK_ZONE_X = 0.0, SK_ZONE_Y = 0.0;
constexpr double SK_ZONE_RADIUS = 12.0;                      // about how far the zone reaches from its middle (for the last-moment dash)
constexpr double SK_PARK_X = 0.0, SK_PARK_Y = -16.0;

// =====================================================================
//   ROUTE SHAPE (headings the robot points at each spot)
// =====================================================================
constexpr double SK_TOGGLE_SHOVE_IN = 10.0;        // the full-power shove off the toggle
constexpr int SK_TOGGLE_SHOVE_MAX_MS = 800;
// --- Preload ---
constexpr double SK_PRELOAD_HEADING = 200.0;       // into goal A from above, pointing down and a little left (the 10" shove
                                                   // leaves it at x -51: straight down (180) is too sharp a hook from there)
constexpr double SK_PRELOAD_LEAD_IN = 10.0;        // inches of straight run into goal A (was 6: it was still swinging when it got there)
constexpr double SK_PRELOAD_HOOK_SPEED = 30.0;     // in/s top speed round the hook into goal A. It comes off the shove at 45+ and
                                                   // has a 110 degree turn in 14": any faster and it runs wide of the goal
constexpr double SK_PRELOAD_IN_SPEED = 30.0;       // in/s on the straight run
constexpr double SK_PRELOAD_PUSH_IN = 3.0;         // drives on this far past where goal A should be, until the goal stops it
// --- Preload -> P1: straight back off the goal, a short swing of the nose, then one forward curve through the pin ---
constexpr double SK_PRELOAD_BACK_STRAIGHT = 9.5;   // dead straight first, until the claw's tips are clear of the goal (at 5" the
                                                   // claw's side caught the goal as the nose swung)
constexpr double SK_PRELOAD_BACK_ARC = 4.5;        // then this many inches with the nose swinging...
constexpr double SK_PRELOAD_BACK_SWING = -35.0;    // ...this many degrees (- = anticlockwise, toward P1)
constexpr double SK_PIN_LEAD_IN = 7.0;             // inches of straight run into a field pin (the cameras get a steady look at it)
constexpr double SK_P1_HEADING = 122.0;            // through P1 (from the upper left)
constexpr double SK_P1_SCORE_HEADING = 300.0;      // P1 onto goal A (from the lower right, round a loop). Backing straight out
                                                   // on this line leaves the most room to turn between goal A and goal B
constexpr double SK_P1_LOOP_TANGENT = 7.5;         // how wide that loop swings (bigger = wider; goal B is below it)
// --- P1 -> P2: straight back, turn to face P2, straight through it, hook round onto goal A ---
constexpr double SK_P2_SCORE_HEADING = 20.0;       // P2 onto goal A (from below, a little left)
constexpr double SK_P2_LEAD_OUT = 2.5;             // inches straight on after grabbing P2 before the hook
constexpr double SK_P2_HOOK_TANGENT = 5.0;
// --- Goals ---
constexpr double SK_GOAL_PUSH_IN = 2.5;            // every score: drives on this far past where the goal should be, until it stops it
constexpr double SK_GOAL_BACK_SLOW_IN = 4.0;       // the first inches straight back off a goal are at SK_BACK_OUT_SPEED
// Before it turns in place next to a goal it backs straight off until its center is this far from the goal's center.
// The front corners swing 11.85" out from the center and a goal's base is 4" in radius: 15.85" is touching.
constexpr double SK_TURN_CLEAR_OF_GOAL = 17.3;
// Goal B -> the loader, two ways.
// true  = like goal A: backs straight down the line it came in on to the turn spot on the loader line, turns to face
//         the loader, straight in. One turn, and it's square to the loader before it drives in.
// false = backs off until it's clear of the goal, turns round, drives forward to the turn spot, turns the last 20 degrees onto
//         the loader line, straight in. Two turns: about 0.4 s a trip slower.
// (Turning round and merging onto the loader line in one forward curve was tried: in the simulator it was still
// swinging onto the line when it got to the loader, 11-18 degrees off square in one run out of four.)
constexpr bool SK_GOAL_B_REVERSE_TO_LOADER = true;
constexpr double SK_MOVE_END_SPEED = 0.0;          // in/s left at the end of a straight back-out, where it then turns in place.
                                                   // 0 = it slows right down onto the spot. (At 10 a drive that brakes less well
                                                   // than planned ran 4" past the turn spot, toward the wall, and then met the
                                                   // loader at an angle.)
// --- The end: backwards round goal B onto the bottom toggle (each spot: x, y, the way the NOSE points) ---
// The back of the robot leads. From goal B's left it goes up, over the top of goal B (16.5" from its center), down
// its right side and straight back onto the wall.
constexpr double SK_END_T1_X = -37.8, SK_END_T1_Y = -38.8, SK_END_T1_HEADING = 210.0;   // upper left of goal B (only from goal B)
constexpr double SK_END_T2_X = -23.5, SK_END_T2_Y = -30.6, SK_END_T2_HEADING = 270.0;   // over the top of goal B
constexpr double SK_END_T3_X = -9.5, SK_END_T3_Y = -38.0, SK_END_T3_HEADING = 322.0;    // upper right of goal B
constexpr double SK_END_T4_Y = -52.0;                                                    // on the toggle's line, nose straight up
constexpr double SK_TOGGLE2_PUSH_IN = 3.0;         // aims this far into the wall: the wall stops it
constexpr double SK_TOGGLE2_IN_SPEED = 24.0;       // in/s onto the wall
constexpr double SK_TOGGLE2_LIFT_AT_IN = 12.0;     // the lift starts up this far before the wall
// Fine tuning per spot (inches, + = further forward)
constexpr double SK_PRELOAD_EXTRA = 0.0;
constexpr double SK_P1_GRAB_EXTRA = 0.0, SK_P1_SCORE_EXTRA = 0.0;
constexpr double SK_P2_GRAB_EXTRA = 0.0, SK_P2_SCORE_EXTRA = 0.0;
constexpr double SK_LOAD_A_SCORE_EXTRA = 0.0;
constexpr double SK_GOAL_B_SCORE_EXTRA = 0.0;

// =====================================================================
//   WAITS AND SPEEDS
// =====================================================================
constexpr int SK_PRELOAD_FLIP_WAIT_MS = 1000;   // stopped at goal A while the preload flips ("hold for a second")
constexpr int SK_LOADER_WAIT_MS = 500;          // stopped at the loader, claw spinning, lift pushing down (was 1000: too long)
constexpr int SK_LOWER_BEFORE_OPEN_MS = 100;    // at a goal: the lift starts down, and this much later the claw opens (the pin is
                                                // set down, not dropped); the lift keeps coming down as it drives back
constexpr int SK_RELEASE_MS = 150;              // then this long for the claw to get off the pin before it drives back
constexpr int SK_SETTLE_MS = 0;                 // stopped at a goal before the claw opens
constexpr int SK_PIN_SLOW_MS = 0;               // slowdown (ms at the slow speed) through a field pin
constexpr double SK_BACK_OUT_SPEED = FP_MOVING_SPEED;   // in/s for the first straight inches off a goal, while the lift comes down
// Speed over the last SK_APPROACH_IN inches into a goal, by how many things are on it after this one. Each stack spot
// is a little slower than the one before; only the top two are slow (the taller the stack, the more a bump matters).
constexpr double SK_APPROACH_SPEED[] = {
    46.0,    // 1st thing on the goal
    42.0,    // 2nd
    36.0,    // 3rd
    30.0,    // 4th (it used to crawl in at 28 or less from the 3rd on)
    28.0,    // 5th
};
constexpr double SK_APPROACH_IN = 10.0;

// =====================================================================
//   LIFT
// =====================================================================
constexpr double SK_TOGGLE_LIFT_DEG = 70.0;     // start: lift up this far...
constexpr int SK_TOGGLE_UP_MAX_MS = 500;
constexpr double SK_TOGGLE_GO_DEG = 20.0;       // ...and the shove starts once it's back down to this
constexpr int SK_TOGGLE_DOWN_MAX_MS = 500;
// The second toggle: the lift goes up over it as the robot backs onto the wall, comes down onto it, and driving
// forward then spins it
constexpr double SK_TOGGLE2_LIFT_DEG = 50.0;    // up to this just before the wall...
constexpr double SK_TOGGLE2_DROP_DEG = 20.0;    // ...and down to this the moment it's on the wall
constexpr int SK_TOGGLE2_DROP_MAX_MS = 200;     // drives off once the lift is down to it, or after this long

namespace {

double height(int thingsOnGoalAfter) { return getStackHeight(GOAL_ALLIANCE, thingsOnGoalAfter); }   // main.cpp's list

// Heading from one spot to another
double headingFromTo(double fromX, double fromY, double toX, double toY) {
    return std::atan2(toX - fromX, toY - fromY) * 180.0 / FP_PI;
}
double headingTo(double x, double y) {
    Node here = hereNode();
    return headingFromTo(here.x, here.y, x, y);
}

// How to score the thingsAfter-th thing on a goal
ScorePlan planFor(double goalX, double goalY, int tagId, int thingsAfter) {
    const int spots = static_cast<int>(sizeof(SK_APPROACH_SPEED) / sizeof(SK_APPROACH_SPEED[0]));
    ScorePlan plan;
    plan.height = height(thingsAfter);
    plan.goalX = goalX;
    plan.goalY = goalY;
    plan.tagId = tagId;
    plan.settleMs = SK_SETTLE_MS;
    plan.releaseMs = SK_RELEASE_MS;
    plan.lowerBeforeOpenMs = SK_LOWER_BEFORE_OPEN_MS;
    plan.liftLate = true;   // lift down while it turns and drives, up as late as it can be
    plan.approachSpeed = SK_APPROACH_SPEED[std::min(std::max(thingsAfter, 1), spots) - 1];
    plan.approachIn = SK_APPROACH_IN;
    plan.pushIn = SK_GOAL_PUSH_IN;
    return plan;
}

// The time guard: how much slower than the simulator the robot's last loader trip was
double slowerThanSim = SK_SLOWER_THAN_SIM;
bool roomFor(int simMs) {
    if (!SK_USE_TIME_GUARD) return true;
    return elapsed() + slowerThanSim * simMs + SK_GUARD_SPARE_MS <= SK_AUTON_MS - SK_DASH_TURN_MS - SK_DASH_DRIVE_MS;
}
void tripTook(int realMs, int simMs) {
    slowerThanSim = clampTo(static_cast<double>(realMs) / simMs, 1.0, 2.0);
    std::printf("skills %5.2f s  loader trip took %.2f s (%.2f x the simulator's)\n", elapsed() / 1000.0, realMs / 1000.0, slowerThanSim);
}

// Backs dead straight off the goal at (goalX, goalY) until the robot's center is SK_TURN_CLEAR_OF_GOAL from the goal's,
// and stops: room to turn in place. The first inches gently, lift coming down, claw motor off.
bool backOffGoal(double goalX, double goalY) {
    Node start = hereNode();
    const double inches = std::max(SK_TURN_CLEAR_OF_GOAL - std::hypot(goalX - start.x, goalY - start.y), SK_GOAL_BACK_SLOW_IN + 2.0);
    Node slow = along(start, -SK_GOAL_BACK_SLOW_IN);
    slow.speed = SK_BACK_OUT_SPEED;
    Curve curve = makeCurve({start, slow, along(start, -inches)}, true, FP_MAX_SPEED, SK_MOVE_END_SPEED);
    MoveOptions options;
    options.tick = [&](double done, double left) {
        (void)left;
        if (done >= FP_LOWER_AFTER_BACK_IN) liftDown();
        if (done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
        return true;
    };
    MoveEnd end = follow(curve, options);
    liftDown();
    clawOff();
    return end != MoveEnd::timeUp;
}

// Into the matchloader from wherever the robot is (nose already about along the loader line): onto the line and
// straight in, claw spinning closed and the lift pushing down for the last stretch, the cameras lining the pin up.
// Waits at the loader, then lifts the matchload just off the floor. Ends stopped at the loader. false = time up.
bool intoLoader() {
    const Node grab = node(SK_LOADER_GRAB_X, SK_LOADER_Y, SK_LOADER_HEADING, 0, 0, SK_LOADER_IN_SPEED);
    const Node here = hereNode();
    std::vector<Node> nodes = {here};
    // Coming in off a curve: a straight run along the line first. Already on the line (from the turn spot): straight in.
    if (here.x - grab.x > SK_LOADER_RUN_IN + 3.0) nodes.push_back(along(grab, -SK_LOADER_RUN_IN));
    nodes.push_back(grab);
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    addSlowdownAtEnd(curve, SK_LOADER_SLOW_MS, SK_LOADER_IN_SPEED);
    aimAt(AimKind::pin, SK_LOADER_PIN_X, SK_LOADER_Y);   // the pin sticking out of the loader
    MoveOptions options;
    options.stallMs = 150;   // against the loader = there
    options.timeoutMs = SK_LOADER_IN_MAX_MS;
    options.tick = [&](double done, double left) {
        (void)done;
        if (left <= SK_LOADER_CLAW_ON_DIST) {
            clawClose();
            liftDown();
            liftPress(true);
        }
        return true;
    };
    if (follow(curve, options) == MoveEnd::timeUp) return false;
    aimStop();
    note("at the loader: claw spinning");
    clawCloseAgain();
    liftDown();
    liftPress(true);
    const uint32_t arrived = pros::millis();
    if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);   // standing still: the distance sensors check the position
    const int waited = static_cast<int>(pros::millis() - arrived);
    if (!waitMs(std::max(SK_LOADER_WAIT_MS - waited, 0))) return false;
    // How square it met the loader (on the terminal when it's more than a few degrees off: the claw won't be on the pin)
    const double offSquare = wrap180(getInertialHeading() - SK_LOADER_HEADING);
    if (std::fabs(offSquare) > SK_SQUARE_MAX_DEG) std::printf("skills %5.2f s  met the loader %.1f deg off square\n", elapsed() / 1000.0, offSquare);
    if (SK_SQUARE_AT_LOADER && std::fabs(offSquare) <= SK_SQUARE_MAX_DEG) setPose(x_pos, y_pos, SK_LOADER_HEADING);
    liftPress(false);
    liftTo(SK_LOADER_CARRY_DEG);   // just off the floor: it turns and drives with the lift down
    return true;
}

// Loader -> a goal: straight back out, turn to face the goal (lift still down), straight in with the lift going up
// as late as it can, let go. false = time up.
bool loaderToGoal(double goalX, double goalY, int tagId, int thingsAfter, double extra) {
    {
        // Back to the turn spot itself (not just 8" back the way it happens to point), so every trip turns at the same
        // place on the loader line
        Curve curve = makeCurve({hereNode(), node(SK_TURN_X, SK_TURN_Y, SK_LOADER_HEADING)}, true, FP_MAX_SPEED, SK_MOVE_END_SPEED);
        MoveOptions options;
        if (follow(curve, options) == MoveEnd::timeUp) return false;
    }
    if (!spinTo(headingTo(goalX, goalY))) return false;
    note("out of the loader, facing the goal");
    Node here = hereNode();
    Node seat = clawOn(goalX, goalY, headingFromTo(here.x, here.y, goalX, goalY), FP_CLAW_REACH + extra);
    return scoreWith({here, seat}, planFor(goalX, goalY, tagId, thingsAfter));
}

// A goal -> the loader, backwards: straight back down the line it came in on to the turn spot, turn, straight in
bool backDownToLoader() {
    Node start = hereNode();
    Node slow = along(start, -SK_GOAL_BACK_SLOW_IN);
    slow.speed = SK_BACK_OUT_SPEED;
    const double spotY = SK_TURN_Y + SK_TURN_ABOVE_LINE;
    Node spot = node(SK_TURN_X, spotY, headingFromTo(SK_TURN_X, spotY, start.x, start.y));   // nose still at the goal
    Curve curve = makeCurve({start, slow, spot}, true, FP_MAX_SPEED, SK_MOVE_END_SPEED);
    MoveOptions options;
    options.tick = [&](double done, double left) {
        (void)left;
        if (done >= FP_LOWER_AFTER_BACK_IN) liftDown();
        if (done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
        return true;
    };
    MoveEnd end = follow(curve, options);
    liftDown();
    clawOff();
    if (end == MoveEnd::timeUp) return false;
    if (!spinTo(SK_LOADER_HEADING)) return false;
    return intoLoader();
}

// Goal B -> the loader: a few inches straight back off the goal, turn round, forward onto the loader line and in
bool goalBToLoader() {
    if (SK_GOAL_B_REVERSE_TO_LOADER) return backDownToLoader();
    if (!backOffGoal(SK_GOAL_B_X, SK_GOAL_B_Y)) return false;
    if (!spinTo(headingTo(SK_TURN_X, SK_TURN_Y))) return false;
    {
        Node here = hereNode();
        Curve curve = makeCurve({here, node(SK_TURN_X, SK_TURN_Y, headingFromTo(here.x, here.y, SK_TURN_X, SK_TURN_Y))}, false,
                                FP_MAX_SPEED, SK_MOVE_END_SPEED);
        MoveOptions options;
        if (follow(curve, options) == MoveEnd::timeUp) return false;
    }
    if (!spinTo(SK_LOADER_HEADING)) return false;   // square to the loader before it drives in
    return intoLoader();
}

// The end: backs round goal B onto the bottom wall toggle (lift up over it, then down onto it), then straight forward
// into the middle zone. fromGoalB: it's at goal B (otherwise goal A). false = time up.
bool toggleAndPark(bool fromGoalB) {
    if (!backOffGoal(fromGoalB ? SK_GOAL_B_X : SK_GOAL_A_X, fromGoalB ? SK_GOAL_B_Y : SK_GOAL_A_Y)) return false;
    // Turn so the BACK points the way it's going: at the first spot of the curve
    const double firstX = fromGoalB ? SK_END_T1_X : SK_END_T2_X, firstY = fromGoalB ? SK_END_T1_Y : SK_END_T2_Y;
    if (!spinTo(headingTo(firstX, firstY) + 180.0)) return false;
    {
        std::vector<Node> nodes = {hereNode()};
        if (fromGoalB) nodes.push_back(node(SK_END_T1_X, SK_END_T1_Y, SK_END_T1_HEADING));
        nodes.push_back(node(SK_END_T2_X, SK_END_T2_Y, SK_END_T2_HEADING));
        nodes.push_back(node(SK_END_T3_X, SK_END_T3_Y, SK_END_T3_HEADING));
        nodes.push_back(node(SK_TOGGLE2_X, SK_END_T4_Y, 0.0, 0, 0, SK_TOGGLE2_IN_SPEED + 12.0));
        nodes.push_back(node(SK_TOGGLE2_X, SK_TOGGLE2_Y, 0.0, 0, 0, SK_TOGGLE2_IN_SPEED));
        nodes.push_back(node(SK_TOGGLE2_X, SK_TOGGLE2_Y - SK_TOGGLE2_PUSH_IN, 0.0));   // into the wall: the wall stops it
        Curve curve = makeCurve(nodes, true, FP_MAX_SPEED, 0.0);
        bool lifted = false;
        MoveOptions options;
        options.endStallMs = 80;                              // on the wall = there
        options.endStallIn = SK_TOGGLE2_PUSH_IN + 2.0;
        options.tick = [&](double done, double left) {
            (void)done;
            if (!lifted && left <= SK_TOGGLE2_LIFT_AT_IN + SK_TOGGLE2_PUSH_IN) {
                lifted = true;
                liftTo(SK_TOGGLE2_LIFT_DEG);   // up over the toggle as it backs onto it
            }
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return false;
    }
    note("back on the wall at the toggle");
    liftTo(SK_TOGGLE2_DROP_DEG);   // straight down onto the toggle
    // Backed onto the wall it's square to it and its distance from it is known (same as the start)
    if (std::fabs(y_pos - SK_TOGGLE2_Y) <= 6.0) {
        const double offSquare = wrap180(getInertialHeading());
        setPose(x_pos, SK_TOGGLE2_Y, std::fabs(offSquare) <= SK_WALL_SQUARE_MAX_DEG ? 0.0 : getInertialHeading());
    }
    for (uint32_t t0 = pros::millis(); getDR4BPosition() > SK_TOGGLE2_DROP_DEG + 6.0 && pros::millis() - t0 < static_cast<uint32_t>(SK_TOGGLE2_DROP_MAX_MS);) {
        if (!waitMs(10)) return false;
    }
    // Straight forward into the zone, as fast as it goes (driving forward is what spins the toggle)
    {
        Curve curve = makeCurve({hereNode(), node(SK_PARK_X, SK_PARK_Y, 0.0)}, false, FP_MAX_SPEED, 0.0);
        MoveOptions options;
        if (follow(curve, options) == MoveEnd::timeUp) return false;
    }
    note("parked in the zone");
    return true;
}

// =====================================================================
//   THE ROUTE
// =====================================================================
void runSkills() {
    setRouteFrame(false, false);   // written in field coordinates
    coreStart("skills", SK_AUTON_MS);
    setEndReserve(SK_DASH_TURN_MS + SK_DASH_DRIVE_MS);   // the last moment belongs to the dash for the zone
    setGateHold(false);   // never stops for the lift before a goal: it keeps rolling in while the lift finishes
    setPose(SK_START_X, SK_START_Y, SK_START_HEADING);
    slowerThanSim = SK_SLOWER_THAN_SIM;

    const Node preloadPose = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_PRELOAD_HEADING, FP_CLAW_REACH + SK_PRELOAD_EXTRA);
    const Node p1Pose = clawOn(SK_P1_X, SK_P1_Y, SK_P1_HEADING, FP_CLAW_REACH + SK_P1_GRAB_EXTRA);
    const Node p1Score = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_P1_SCORE_HEADING, FP_CLAW_REACH + SK_P1_SCORE_EXTRA);
    const Node p2Score = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_P2_SCORE_HEADING, FP_CLAW_REACH + SK_P2_SCORE_EXTRA);
    int onGoalA = 0, onGoalB = 0;   // things already on each goal

    // ---------------- Start: lift up and down (the first toggle), then a full-power shove off the wall ----------------
    liftRaiseDirect(SK_TOGGLE_LIFT_DEG, SK_TOGGLE_UP_MAX_MS);
    note("lift up for the toggles");
    liftDown();
    {
        uint32_t t0 = pros::millis();
        while (getDR4BPosition() > SK_TOGGLE_GO_DEG && pros::millis() - t0 < static_cast<uint32_t>(SK_TOGGLE_DOWN_MAX_MS)) {
            service();
            pros::delay(10);
        }
    }
    if (!shoveForward(SK_TOGGLE_SHOVE_IN, SK_TOGGLE_SHOVE_MAX_MS)) return;
    note("shoved off the toggle");

    // ---------------- 1. Curve into goal A from above, claw open; flip the preload on ----------------
    {
        // A straight run in (lined up on the goal's tag), gently onto the goal, and on until the goal stops it
        Node leadIn = along(preloadPose, -SK_PRELOAD_LEAD_IN);
        leadIn.speed = SK_PRELOAD_IN_SPEED;
        Node seat = preloadPose;
        seat.speed = FP_CONTACT_SPEED;
        Curve curve = makeCurve({hereNode(), leadIn, seat, along(preloadPose, SK_PRELOAD_PUSH_IN)}, false, SK_PRELOAD_HOOK_SPEED, 0.0);
        clawOpen();   // open before it gets to the goal
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG);
        MoveOptions options;
        options.endStallMs = FP_CONTACT_STALL_MS;   // on the goal = there
        options.endStallIn = SK_PRELOAD_PUSH_IN + 1.5;
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at goal A (preload)");
        aimStop();
        stopChassis(pros::MotorBrake::brake);
        flipperUp();
        const uint32_t resetStart = pros::millis();
        if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);   // standing still: the distance sensors check the position
        // holds so the preload falls straight (the reset counts toward the hold: no extra time)
        if (!waitMs(std::max(SK_PRELOAD_FLIP_WAIT_MS - static_cast<int>(pros::millis() - resetStart), 0))) return;
        note("preload flipped");
        // (Odometry used to be snapped onto the goal here. In the video run the robot wasn't on the goal, and the
        // snap put a 3-4" error into everything that followed. The distance sensors do the checking now.)
        onGoalA = 1;
    }

    // ---------------- 2, 3. Straight back with a small swing, through P1, round the loop onto goal A ----------------
    {
        Curve curve = makeCurve(backArc(hereNode(), SK_PRELOAD_BACK_STRAIGHT, SK_PRELOAD_BACK_ARC, SK_PRELOAD_BACK_SWING), true,
                                FP_MAX_SPEED, FP_CUSP_SPEED);
        MoveOptions options;
        options.brakeAtEnd = false;   // rolls straight into the forward curve
        options.tick = [&](double done, double left) {
            (void)left;
            if (done >= SK_PRELOAD_BACK_STRAIGHT) flipperDown();
            if (done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
            return true;
        };
        MoveEnd end = follow(curve, options);
        flipperDown();
        liftDown();
        if (end == MoveEnd::timeUp) return;
        note("backed off goal A");
    }
    {
        std::vector<Node> nodes = {
            hereNode(),
            along(p1Pose, -SK_PIN_LEAD_IN),
            node(p1Pose.x, p1Pose.y, p1Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(p1Pose, FP_PIN_LEAD_OUT, 0, SK_P1_LOOP_TANGENT),
            along(p1Score, -FP_GOAL_LEAD_IN, SK_P1_LOOP_TANGENT, 0),
            p1Score,
        };
        if (!pickUpAndScoreWith(nodes, 2, SK_P1_X, SK_P1_Y, -24.0, -24.0, SK_PIN_SLOW_MS,
                                planFor(SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG, onGoalA + 1))) return;
        onGoalA++;
    }

    // ---------------- 4, 5. Straight back (into the gap between the goals), turn to face P2, straight through it,
    //                        hook round onto goal A ----------------
    if (!backOffGoal(SK_GOAL_A_X, SK_GOAL_A_Y)) return;
    if (!spinTo(headingTo(SK_P2_X, SK_P2_Y))) return;
    {
        Node here = hereNode();
        const Node p2Pose = clawOn(SK_P2_X, SK_P2_Y, headingFromTo(here.x, here.y, SK_P2_X, SK_P2_Y), FP_CLAW_REACH + SK_P2_GRAB_EXTRA);
        std::vector<Node> nodes = {
            here,
            node(p2Pose.x, p2Pose.y, p2Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(p2Pose, SK_P2_LEAD_OUT, 0, SK_P2_HOOK_TANGENT),
            along(p2Score, -FP_GOAL_LEAD_IN, SK_P2_HOOK_TANGENT, 0),
            p2Score,
        };
        if (!pickUpAndScoreWith(nodes, 1, SK_P2_X, SK_P2_Y, -48.0, -48.0, SK_PIN_SLOW_MS,
                                planFor(SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG, onGoalA + 1))) return;
        onGoalA++;
    }

    // ---------------- 6. Matchloader -> goal A ----------------
    bool atGoalB = false;
    bool outOfTime = false;
    for (int i = 0; i < SK_GOAL_A_LOADS && !outOfTime; i++) {
        const int simTrip = SK_SIM_TRIP_MS + SK_SIM_TRIP_PER_SPOT_MS * onGoalA;
        if (!roomFor(simTrip + SK_SIM_END_FROM_A_MS)) {
            outOfTime = true;
            break;
        }
        const int t0 = elapsed();
        if (!backDownToLoader()) return;
        if (!loaderToGoal(SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG, onGoalA + 1, SK_LOAD_A_SCORE_EXTRA)) return;
        onGoalA++;
        tripTook(elapsed() - t0, simTrip);
    }

    // ---------------- 7. Matchloader -> goal B ----------------
    for (int i = 0; i < SK_GOAL_B_LOADS && !outOfTime; i++) {
        const int simTrip = SK_SIM_TRIP_MS + SK_SIM_TRIP_PER_SPOT_MS * onGoalB;
        if (!roomFor(simTrip + SK_SIM_END_FROM_B_MS)) {
            outOfTime = true;
            break;
        }
        const int t0 = elapsed();
        if (!(atGoalB ? goalBToLoader() : backDownToLoader())) return;
        if (!loaderToGoal(SK_GOAL_B_X, SK_GOAL_B_Y, SK_GOAL_B_TAG, onGoalB + 1, SK_GOAL_B_SCORE_EXTRA)) return;
        onGoalB++;
        atGoalB = true;
        tripTook(elapsed() - t0, simTrip);
    }
    if (outOfTime) note("no time for another load: toggle and park");
    std::printf("skills %5.2f s  goal A has %d, goal B has %d\n", elapsed() / 1000.0, onGoalA, onGoalB);
    char counts[32];
    std::snprintf(counts, sizeof(counts), "A%d B%d, to toggle", onGoalA, onGoalB);
    note(counts);   // stays on the controller if time runs out on the way

    // ---------------- End: the bottom wall toggle, then the middle zone ----------------
    if (!toggleAndPark(atGoalB)) return;
    liftCoast();
    std::snprintf(counts, sizeof(counts), "done A%d B%d parked", onGoalA, onGoalB);
    note(counts);
}

}  // namespace

void skillsAuton() {
    runSkills();
    // Out of time before it was parked: point an end at the zone and drive flat out, so it's carried in
    const bool dashed = timeUp();
    endDash(SK_ZONE_X, SK_ZONE_Y, SK_ZONE_RADIUS, SK_DASH_DRIVE_MS);
    finishSafe(!dashed);   // after the dash the drive is left coasting
}
