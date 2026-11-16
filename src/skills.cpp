// =====================================================================
//   SKILLS RUN (1 minute)
// =====================================================================
// Red, back on the LEFT wall toggle (the middle of the left wall), facing right. Everything is in FIELD coordinates:
// (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// The driving, lift / claw / flipper service, AI Vision aiming and the timer are in src/auton-core.cpp
// (include/auton-core.hpp), shared with the 4-pin qual auton.
//
// The route (the numbers are the arrows on the drawing):
//   start  lift up SK_TOGGLE_LIFT_DEG and back down, then a full-power 10" shove forward (the wall toggle)
//   1      curve into goal A (the red goal nearest the start) from above, claw open; flip the preload on, hold 1 s
//   2, 3   back out, pick up the field pin near the middle (P1) and score it on goal A
//   4, 5   back out, pick up the field pin near the corner (P2) and score it on goal A
//   6      matchloader (bottom-left corner) -> goal A, SK_GOAL_A_LOADS times (pin + cup each)
//   7      matchloader -> goal B (the other red goal), SK_GOAL_B_LOADS times (the first is just a pin)
//   9      matchloader -> the MIDDLE goal (arrow 8, the cluster pin, is skipped)
//   10-13  the two yellow pins behind the middle goal (Y1, Y2), each scored on the middle goal
// Goal A and goal B approaches line up on the goal's AprilTag (field camera, port GOAL_VISION_PORT; the middle goal has
// no tag: odometry + MCL), every pin pickup on the pin (both cameras). Odometry + MCL drive everything else, and a
// distance-sensor reset runs at every goal while the robot is stopped there letting go. While the claw is open going
// into a pin, or spinning at the loader, the lift pushes down (liftPress) so the claw stays low.
// Heights come from main.cpp's ALLIANCE_HEIGHTS (getStackHeight), one per thing already on that goal + 1.
//
// HOW THE LOADER TRIPS MOVE (no turning in place anywhere in them): the robot only ever stops at the loader (claw
// spinning) and at a goal (letting go). Everything between is one curve into the next: it backs out of the loader
// round a LOOP that swings the nose onto the goal and rolls straight into the forward run at the goal, and it backs
// out of the goal round a loop that swings the nose onto the loader line. The loops are drawn with the SK_LOOP_
// spots below (field inches, and the way the nose points there). The loops sit in the open floor between the loader,
// goal A and goal B; the simulator's footprint check (robot body + claw against walls, goals and the small pins) is
// how they were placed.
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
constexpr int SK_GOAL_A_LOADS = 2;    // matchloads onto goal A after the preload + 2 field pins (arrow 6)
constexpr int SK_GOAL_B_LOADS = 5;    // matchloads onto goal B (arrow 7): 1 pin, then pin + cups
constexpr int SK_MIDDLE_LOADS = 1;    // matchloads onto the middle goal (arrow 9), before the two yellow pins

// =====================================================================
//   FIELD SPOTS
// =====================================================================
constexpr double SK_START_BACK = 8.4;                               // measured: the robot's center is 8.4" from the wall at the start
constexpr double SK_START_X = -FP_FIELD_HALF + SK_START_BACK;       // -61.85: back on the left wall toggle
constexpr double SK_START_Y = 0.0;
constexpr double SK_START_HEADING = 90.0;                           // facing right
constexpr double SK_GOAL_A_X = -47.09, SK_GOAL_A_Y = -23.55;   // red goal nearest the start (the preload goal)
constexpr double SK_GOAL_B_X = -23.55, SK_GOAL_B_Y = -47.09;   // the other red goal (bottom)
constexpr double SK_MIDDLE_X = 0.0, SK_MIDDLE_Y = 0.0;         // the middle goal
constexpr double SK_P1_X = -23.55, SK_P1_Y = -23.55;           // field pin on the diagonal, near the middle
constexpr double SK_P2_X = -47.09, SK_P2_Y = -47.09;           // field pin on the diagonal, near the corner
constexpr double SK_Y1_X = 23.55, SK_Y1_Y = 23.55;             // yellow pin behind the middle goal, near
constexpr double SK_Y2_X = 47.09, SK_Y2_Y = 47.09;             // yellow pin behind the middle goal, far
// AprilTags on the goals (field camera). -1 = any tag it sees near where that goal should be. The brain screen
// shows each tag's id when the camera looks at it.
constexpr int SK_GOAL_A_TAG = 2;      // "red goal apriltag 2"
constexpr int SK_GOAL_B_TAG = 3;      // "goal b is apriltag 3"
// The middle goal has no tag for the camera (it's only about 9" tall): the robot goes by odometry + MCL, it's at (0, 0)
constexpr int SK_MIDDLE_TAG = NO_GOAL_TAG;
// Matchloader (bottom-left corner, on the left wall): the robot drives WEST along the loader line into it
constexpr double SK_LOADER_DEPTH = 4.0;                      // the loader's face is this far out from the wall
constexpr double SK_LOADER_FACE_X = -FP_FIELD_HALF + SK_LOADER_DEPTH;   // -66.25
constexpr double SK_LOADER_Y = -59.0;                        // the loader's center (59" from the field's center line) = the line it drives in on
constexpr double SK_LOADER_PIN_X = -FP_FIELD_HALF + SK_LOADER_DEPTH / 2.0;   // -68.25: the next pin sits in the middle of the loader
// How deep it drives in. The claw sticks out FP_CLAW_TIP_FORWARD (10.5") from the center and the loader is only 4" deep,
// so the claw's tips reach the wall behind the loader with the flex wheels about on the pin: the drive aims the claw's
// tips at the wall, and whatever stops the robot first (the loader or the wall) ends the move.
constexpr double SK_LOADER_EXTRA = 2.5;                      // + = aims this much deeper still ("go an extra 2 inches into the matchloader")
constexpr double SK_LOADER_GRAB_X = -FP_FIELD_HALF + FP_CLAW_TIP_FORWARD - SK_LOADER_EXTRA;   // -62.25: robot center when loading
constexpr double SK_LOADER_RUN_IN = 8.0;                     // inches of dead-straight drive along the line into the loader
constexpr double SK_LOADER_IN_SPEED = FP_WALL_SPEED;         // in/s for the last stretch into the loader: the 4-pin's speed into the wall cup (26)...
constexpr int SK_LOADER_SLOW_MS = 300;                       // ...for this long (26 in/s x 0.3 s = the last 8")
constexpr double SK_LOADER_BACK_OUT = 10.0;                  // straight back out along the line before the loop starts (the claw sweeps
                                                             // toward the wall as the nose turns: a short back-out clipped it)
constexpr double SK_LOADER_CLAW_ON_DIST = 14.0;              // claw starts spinning closed this far before the loader
// Stalled against the loader the robot is square to the wall, so its heading IS 270: odometry's heading is set to that
// every trip. The IMU drifts a little with every turn (the loops turn it a lot), and over a minute that adds up to
// several degrees, which is several inches at the end of a long run.
constexpr bool SK_SQUARE_AT_LOADER = true;
constexpr double SK_LOADER_HEADING = 270.0;

// =====================================================================
//   ROUTE SHAPE (headings the robot points at each spot)
// =====================================================================
constexpr double SK_TOGGLE_SHOVE_IN = 10.0;        // the full-power shove off the toggle
constexpr int SK_TOGGLE_SHOVE_MAX_MS = 800;
constexpr double SK_PRELOAD_HEADING = 200.0;       // into goal A from above, pointing down and a little left (the 10" shove
                                                   // leaves it at x -51: straight down (180) is too sharp a hook from there)
constexpr double SK_TURN1_X = -50.0, SK_TURN1_Y = -5.0, SK_TURN1_HEADING = 120.0;   // after the preload: backs here, nose at P1
constexpr double SK_P1_HEADING = 140.0;            // through P1 (from the upper left; 150 took a corner within 1" of the small pin at (-23.55, 0))
constexpr double SK_P1_SCORE_HEADING = 285.0;      // P1 onto goal A (from the right)
constexpr double SK_TURN2_X = -27.1, SK_TURN2_Y = -25.4, SK_TURN2_HEADING = 228.0;  // after P1: backs here, nose at P2
constexpr double SK_P2_HEADING = 225.0;            // through P2 (along the diagonal, from the upper right)
constexpr double SK_P2_SCORE_HEADING = 25.0;       // P2 onto goal A (from the lower left)
constexpr double SK_LOAD_A_SCORE_HEADING = 10.0;   // matchloads onto goal A (from below)
constexpr double SK_GOAL_B_SCORE_HEADING = 85.0;   // matchloads onto goal B (from the left, coming off the loader loop)
constexpr double SK_MIDDLE_WAY_X = -34.5, SK_MIDDLE_WAY_Y = -30.0;   // on the way to the middle goal (between goal A and goal B)
constexpr double SK_MIDDLE_SW_HEADING = 45.0;      // onto the middle goal from the lower left (the matchload)
constexpr double SK_MIDDLE_NE_HEADING = 225.0;     // onto the middle goal from the upper right (Y1, Y2)
// --- The loops (see the top of the file). Each spot: x, y, and the way the NOSE points there. ---
constexpr double SK_LOOP_SPEED = 30.0;   // in/s top speed round a loop (they're tight: faster slips the wheels)
// Out of the loader to goal B: a loop up and round to the left (anticlockwise), ending pointing at goal B
constexpr double SK_LOOP_B1_X = -43.0, SK_LOOP_B1_Y = -56.0, SK_LOOP_B1_HEADING = 225.0;
constexpr double SK_LOOP_B2_X = -40.0, SK_LOOP_B2_Y = -49.0, SK_LOOP_B2_HEADING = 180.0;
constexpr double SK_LOOP_B3_X = -44.0, SK_LOOP_B3_Y = -44.0, SK_LOOP_B3_HEADING = 90.0;
constexpr double SK_LOOP_B4_X = -49.0, SK_LOOP_B4_Y = -46.0, SK_LOOP_B4_HEADING = 83.0;   // the switch to forwards: nose at goal B
// Out of goal B back to the loader line: straight back, then a loop down and round (anticlockwise), ending pointing at the loader
constexpr double SK_LOOP_BL1_X = -43.0, SK_LOOP_BL1_Y = -53.0, SK_LOOP_BL1_HEADING = 0.0;
constexpr double SK_LOOP_BL2_X = -38.5, SK_LOOP_BL2_Y = -58.0, SK_LOOP_BL2_HEADING = 270.0;
constexpr double SK_LOOP_BL3_X = -33.0, SK_LOOP_BL3_Y = -56.5, SK_LOOP_BL3_HEADING = 250.0;   // the switch to forwards: nose at the loader line
// Out of the loader to goal A (and to the middle goal): a loop up and round to the left, ending pointing up at goal A
constexpr double SK_LOOP_A1_X = -45.0, SK_LOOP_A1_Y = -54.0, SK_LOOP_A1_HEADING = 225.0;
constexpr double SK_LOOP_A2_X = -46.0, SK_LOOP_A2_Y = -47.0, SK_LOOP_A2_HEADING = 150.0;
constexpr double SK_LOOP_A3_X = -52.0, SK_LOOP_A3_Y = -45.0, SK_LOOP_A3_HEADING = 70.0;
constexpr double SK_LOOP_A4_X = -55.0, SK_LOOP_A4_Y = -50.0, SK_LOOP_A4_HEADING = 15.0;    // the switch to forwards: nose at goal A
// Out of goal A (and after P2) back to the loader line: straight back, then a hook down to the right, ending pointing at the loader
constexpr double SK_LOOP_AL1_X = -49.5, SK_LOOP_AL1_Y = -49.0, SK_LOOP_AL1_HEADING = 330.0;
constexpr double SK_LOOP_AL2_X = -44.0, SK_LOOP_AL2_Y = -54.0, SK_LOOP_AL2_HEADING = 290.0;
constexpr double SK_LOOP_AL3_X = -38.0, SK_LOOP_AL3_Y = -56.0, SK_LOOP_AL3_HEADING = 250.0;   // the switch to forwards: nose at the loader line
// Y1: out of the middle goal round a loop that lands the nose pointing west (clear of the small pin below the goal),
// then round the outside of the small pin left of the middle goal, up, and into Y1 from the upper left
constexpr double SK_Y1_BACK = 10.0;                // straight back from the middle goal first
constexpr double SK_Y1_L1_X = -22.0, SK_Y1_L1_Y = -19.0, SK_Y1_L1_HEADING = 10.0;
constexpr double SK_Y1_L2_X = -19.0, SK_Y1_L2_Y = -26.0, SK_Y1_L2_HEADING = 300.0;
constexpr double SK_Y1_L3_X = -14.0, SK_Y1_L3_Y = -24.0, SK_Y1_L3_HEADING = 268.0;   // the switch to forwards: nose west
constexpr double SK_Y1_W1_X = -38.0, SK_Y1_W1_Y = -6.0, SK_Y1_W1_HEADING = 0.0;
constexpr double SK_Y1_W2_X = -35.3, SK_Y1_W2_Y = 11.8, SK_Y1_W2_HEADING = 25.0;   // the gap between the small pin and the upper-left goal
constexpr double SK_Y1_W3_X = -13.0, SK_Y1_W3_Y = 32.0, SK_Y1_W3_HEADING = 40.0;   // (further up-left and a corner nears the upper-left goal)
constexpr double SK_Y1_HEADING = 135.0;            // through Y1 (from the upper left)
// Y2: out of the middle goal back up the diagonal (Y1 is gone by then) round a loop that lands the nose on Y2, through
// Y2, then a forward loop in the corner that brings the nose back round onto the middle goal
constexpr double SK_Y2_BACK = 22.0;
constexpr double SK_Y2_L1_X = 26.0, SK_Y2_L1_Y = 26.0, SK_Y2_L1_HEADING = 180.0;
constexpr double SK_Y2_L2_X = 24.0, SK_Y2_L2_Y = 31.0, SK_Y2_L2_HEADING = 100.0;
constexpr double SK_Y2_L3_X = 17.0, SK_Y2_L3_Y = 31.0, SK_Y2_L3_HEADING = 40.0;
constexpr double SK_Y2_L4_X = 15.0, SK_Y2_L4_Y = 27.0, SK_Y2_L4_HEADING = 30.0;     // the switch to forwards: nose toward Y2
constexpr double SK_Y2_HEADING = 45.0;             // through Y2 (along the diagonal)
constexpr double SK_Y2_C1_X = 50.0, SK_Y2_C1_Y = 48.0, SK_Y2_C1_HEADING = 90.0;     // the corner loop, forwards, clockwise...
constexpr double SK_Y2_C2_X = 55.0, SK_Y2_C2_Y = 43.0, SK_Y2_C2_HEADING = 170.0;
constexpr double SK_Y2_C3_X = 54.0, SK_Y2_C3_Y = 38.0, SK_Y2_C3_HEADING = 230.0;
constexpr double SK_Y2_C4_X = 48.0, SK_Y2_C4_Y = 37.0, SK_Y2_C4_HEADING = 275.0;    // ...heading west toward the gap between the two goals
constexpr double SK_Y2_GAP_X = 35.3, SK_Y2_GAP_Y = 35.3, SK_Y2_GAP_HEADING = 235.0; // the middle of that gap, then down to the middle goal
// Inches straight back before the nose starts to swing (so the front corners miss the goal)
constexpr double SK_GOAL_BACK_OUT = 5.0;
constexpr double SK_P1_BACK_OUT = 6.0;
constexpr double SK_GOAL_A_BACK_OUT = 9.0;         // off goal A toward the loader (the hook needs the room)
constexpr double SK_GOAL_B_BACK_OUT = 7.5;         // off goal B toward the loader
// Fine tuning per spot (inches, + = further forward)
constexpr double SK_PRELOAD_EXTRA = 0.0;
constexpr double SK_P1_GRAB_EXTRA = 0.0, SK_P1_SCORE_EXTRA = 0.0;
constexpr double SK_P2_GRAB_EXTRA = 0.0, SK_P2_SCORE_EXTRA = 0.0;
constexpr double SK_LOAD_A_SCORE_EXTRA = 0.0;
constexpr double SK_GOAL_B_SCORE_EXTRA = 0.0;
constexpr double SK_MIDDLE_SCORE_EXTRA = 0.0;
constexpr double SK_Y1_GRAB_EXTRA = 0.0, SK_Y2_GRAB_EXTRA = 0.0;

// =====================================================================
//   WAITS AND SLOWDOWNS
// =====================================================================
constexpr int SK_PRELOAD_FLIP_WAIT_MS = 1000;   // stopped at goal A while the preload flips ("hold for a second")
constexpr int SK_LOADER_WAIT_MS = 500;          // stopped at the loader, claw spinning, lift pushing down (was 1000: too long)
constexpr int SK_LOWER_BEFORE_OPEN_MS = 100;    // at a goal: the lift starts down, and this much later the claw opens (the pin is
                                                // set down, not dropped); the lift keeps coming down as it drives back
constexpr int SK_RELEASE_MS = 150;              // then this long for the claw to get off the pin before it drives back
constexpr int SK_SETTLE_MS = 0;                 // stopped at a goal before the claw opens
constexpr int SK_PIN_SLOW_MS = 0;               // slowdowns (ms at the slow speed): through a field pin...
constexpr int SK_SCORE_SLOW_MS = 0;             // ...and into a goal
constexpr double SK_BACK_OUT_SPEED = FP_MOVING_SPEED;   // in/s for the first straight inches off a goal, while the lift comes down (20)

// =====================================================================
//   LIFT
// =====================================================================
constexpr double SK_TOGGLE_LIFT_DEG = 70.0;     // start: lift up this far...
constexpr int SK_TOGGLE_UP_MAX_MS = 500;
constexpr double SK_TOGGLE_GO_DEG = 20.0;       // ...and the shove starts once it's back down to this
constexpr int SK_TOGGLE_DOWN_MAX_MS = 500;

// The middle goal is only about 9" tall, so the alliance heights are probably wrong for it. PLACEHOLDER: lift by hand
// onto the middle goal with 0, 1 and 2 things on it and read the lift (CONTROLLER_READOUT 0, "L").
constexpr double SK_MIDDLE_HEIGHTS[] = {
    160.0,    // 1st thing on the middle goal   (copied from ALLIANCE_HEIGHTS: MEASURE)
    240.0,    // 2nd
    315.0,    // 3rd
};

namespace {

double height(int thingsOnGoalAfter) { return getStackHeight(GOAL_ALLIANCE, thingsOnGoalAfter); }   // main.cpp's list
double middleHeight(int thingsOnGoalAfter) {
    const int n = static_cast<int>(sizeof(SK_MIDDLE_HEIGHTS) / sizeof(SK_MIDDLE_HEIGHTS[0]));
    return std::max(SK_MIDDLE_HEIGHTS[std::min(std::max(thingsOnGoalAfter, 1), n) - 1], DR4B_BOTTOM_DEADZONE + 1.0);
}

// A backwards curve through these spots (the first is where the robot is now). It doesn't stop at the end: the next
// (forwards) move picks up from the switch, carrying a little speed through it, the way the 4-pin route does.
// fromGoal: the lift comes down and the claw motor goes off in the first inches (leaving a goal)
bool reverseThrough(std::vector<Node> nodes, bool fromGoal) {
    Curve curve = makeCurve(nodes, true, FP_MAX_SPEED, FP_CUSP_SPEED);
    MoveOptions options;
    options.brakeAtEnd = false;
    options.tick = [&](double done, double left) {
        (void)left;
        if (fromGoal && done >= FP_LOWER_AFTER_BACK_IN) liftDown();
        if (fromGoal && done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
        return true;
    };
    MoveEnd end = follow(curve, options);
    if (fromGoal) {
        liftDown();
        clawOff();
    }
    return end != MoveEnd::timeUp;
}

// Into the matchloader from wherever the robot is (nose already roughly along the loader line, coming off a loop):
// one curve onto the line and straight in, claw spinning closed and the lift pushing down for the last stretch, the
// pin camera lining the pin up. Waits at the loader, lifts a little, and leaves the lift heading for scoreHeight.
// Ends stopped at the loader. false = time up.
bool intoLoader(double scoreHeight) {
    const Node grab = node(SK_LOADER_GRAB_X, SK_LOADER_Y, 270.0, 0, 0, SK_LOADER_IN_SPEED);
    const Node runIn = along(grab, -SK_LOADER_RUN_IN, 6.0, 0);
    Curve curve = makeCurve({hereNode(), runIn, grab}, false, FP_MAX_SPEED, 0.0);
    addSlowdownAtEnd(curve, SK_LOADER_SLOW_MS, SK_LOADER_IN_SPEED);
    aimAt(AimKind::pin, SK_LOADER_PIN_X, SK_LOADER_Y);   // the pin sticking out of the loader
    MoveOptions options;
    options.stallMs = 150;   // against the loader = there
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
    if (!waitMs(SK_LOADER_WAIT_MS)) return false;
    if (SK_SQUARE_AT_LOADER) setPose(x_pos, y_pos, SK_LOADER_HEADING);   // squared up on the loader: the heading is known
    liftPress(false);
    liftTo(scoreHeight);   // up while it drives to the goal (no waiting)
    return true;
}

// Out of the loader: straight back along the line, then round the loop whose spots are given, ending at the switch
// with the nose on the goal. Still rolling. false = time up.
bool outOfLoader(const std::vector<Node>& loop) {
    Node start = hereNode();
    std::vector<Node> nodes = {start, along(start, -SK_LOADER_BACK_OUT)};
    nodes.insert(nodes.end(), loop.begin(), loop.end());
    if (!reverseThrough(nodes, false)) return false;
    note("round the loop: nose on the goal");
    return true;
}

// Out of a goal: straight back `straight` inches at an easy speed (lift coming down), then round the loop whose spots
// are given, ending at the switch with the nose on the loader line. Still rolling. false = time up.
bool goalToLoaderLine(double straight, const std::vector<Node>& loop) {
    Node start = hereNode();
    Node off = along(start, -straight);
    off.speed = SK_BACK_OUT_SPEED;
    std::vector<Node> nodes = {start, off};
    nodes.insert(nodes.end(), loop.begin(), loop.end());
    if (!reverseThrough(nodes, true)) return false;
    note("round the loop: nose on the loader");
    return true;
}

// A loop spot: x, y, nose heading, at the loop speed
Node loopNode(double x, double y, double heading) { return node(x, y, heading, 0, 0, SK_LOOP_SPEED); }

// The loops (see ROUTE SHAPE)
std::vector<Node> loopLoaderToGoalB() {
    return {loopNode(SK_LOOP_B1_X, SK_LOOP_B1_Y, SK_LOOP_B1_HEADING), loopNode(SK_LOOP_B2_X, SK_LOOP_B2_Y, SK_LOOP_B2_HEADING),
            loopNode(SK_LOOP_B3_X, SK_LOOP_B3_Y, SK_LOOP_B3_HEADING), loopNode(SK_LOOP_B4_X, SK_LOOP_B4_Y, SK_LOOP_B4_HEADING)};
}
std::vector<Node> loopGoalBToLoader() {
    return {loopNode(SK_LOOP_BL1_X, SK_LOOP_BL1_Y, SK_LOOP_BL1_HEADING), loopNode(SK_LOOP_BL2_X, SK_LOOP_BL2_Y, SK_LOOP_BL2_HEADING),
            loopNode(SK_LOOP_BL3_X, SK_LOOP_BL3_Y, SK_LOOP_BL3_HEADING)};
}
std::vector<Node> loopLoaderToGoalA() {
    return {loopNode(SK_LOOP_A1_X, SK_LOOP_A1_Y, SK_LOOP_A1_HEADING), loopNode(SK_LOOP_A2_X, SK_LOOP_A2_Y, SK_LOOP_A2_HEADING),
            loopNode(SK_LOOP_A3_X, SK_LOOP_A3_Y, SK_LOOP_A3_HEADING), loopNode(SK_LOOP_A4_X, SK_LOOP_A4_Y, SK_LOOP_A4_HEADING)};
}
std::vector<Node> loopGoalAToLoader() {
    return {loopNode(SK_LOOP_AL1_X, SK_LOOP_AL1_Y, SK_LOOP_AL1_HEADING), loopNode(SK_LOOP_AL2_X, SK_LOOP_AL2_Y, SK_LOOP_AL2_HEADING),
            loopNode(SK_LOOP_AL3_X, SK_LOOP_AL3_Y, SK_LOOP_AL3_HEADING)};
}

// Forward from the switch into a goal and let go
bool scoreInto(const Node& scorePose, double scoreHeight, double goalX, double goalY, int tagId) {
    return scoreOnGoal({hereNode(), along(scorePose, -FP_GOAL_LEAD_IN, 8.0, 0), scorePose}, scoreHeight, goalX, goalY, tagId,
                       SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS);
}

// =====================================================================
//   THE ROUTE
// =====================================================================
void runSkills() {
    setRouteFrame(false, false);   // written in field coordinates
    coreStart("skills", SK_AUTON_MS);
    setGateHold(false);   // never stops for the lift before a goal: it keeps rolling in while the lift finishes
    setPose(SK_START_X, SK_START_Y, SK_START_HEADING);

    const Node preloadPose = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_PRELOAD_HEADING, FP_CLAW_REACH + SK_PRELOAD_EXTRA);
    const Node turn1 = node(SK_TURN1_X, SK_TURN1_Y, SK_TURN1_HEADING);
    const Node p1Pose = clawOn(SK_P1_X, SK_P1_Y, SK_P1_HEADING, FP_CLAW_REACH + SK_P1_GRAB_EXTRA);
    const Node p1Score = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_P1_SCORE_HEADING, FP_CLAW_REACH + SK_P1_SCORE_EXTRA);
    const Node turn2 = node(SK_TURN2_X, SK_TURN2_Y, SK_TURN2_HEADING);
    const Node p2Pose = clawOn(SK_P2_X, SK_P2_Y, SK_P2_HEADING, FP_CLAW_REACH + SK_P2_GRAB_EXTRA);
    const Node p2Score = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_P2_SCORE_HEADING, FP_CLAW_REACH + SK_P2_SCORE_EXTRA);
    const Node loadAScore = clawOn(SK_GOAL_A_X, SK_GOAL_A_Y, SK_LOAD_A_SCORE_HEADING, FP_CLAW_REACH + SK_LOAD_A_SCORE_EXTRA);
    const Node goalBScore = clawOn(SK_GOAL_B_X, SK_GOAL_B_Y, SK_GOAL_B_SCORE_HEADING, FP_CLAW_REACH + SK_GOAL_B_SCORE_EXTRA);
    const Node middleSW = clawOn(SK_MIDDLE_X, SK_MIDDLE_Y, SK_MIDDLE_SW_HEADING, FP_CLAW_REACH + SK_MIDDLE_SCORE_EXTRA);
    const Node middleNE = clawOn(SK_MIDDLE_X, SK_MIDDLE_Y, SK_MIDDLE_NE_HEADING, FP_CLAW_REACH + SK_MIDDLE_SCORE_EXTRA);
    const Node y1Pose = clawOn(SK_Y1_X, SK_Y1_Y, SK_Y1_HEADING, FP_CLAW_REACH + SK_Y1_GRAB_EXTRA);
    const Node y2Pose = clawOn(SK_Y2_X, SK_Y2_Y, SK_Y2_HEADING, FP_CLAW_REACH + SK_Y2_GRAB_EXTRA);
    int onGoalA = 0, onGoalB = 0, onMiddle = 0;   // things already on each goal

    // ---------------- Start: lift up and down (the toggles), then a full-power shove off the wall ----------------
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
        Curve curve = makeCurve({hereNode(), along(preloadPose, -FP_GOAL_LEAD_IN, 8.0, 0), preloadPose}, false, FP_MAX_SPEED, 0.0);
        addSlowdownAtEnd(curve, SK_SCORE_SLOW_MS, FP_SCORE_SLOW_SPEED);
        clawOpen();   // open before it gets to the goal
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG);
        MoveOptions options;
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at goal A (preload)");
        aimStop();
        stopChassis(pros::MotorBrake::brake);
        flipperUp();
        if (FP_RESET_AT_GOALS) fieldAwareReset(FP_RESET_MAX_JUMP);   // standing still: the distance sensors check the position
        if (!waitMs(SK_PRELOAD_FLIP_WAIT_MS)) return;   // holds so the preload falls straight
        note("preload flipped");
        // The aligner has seated the robot on goal A: that IS where it is, whatever odometry says (it was arriving ~3"
        // left). Everything after this is measured from here.
        snapToGoal(SK_GOAL_A_X, SK_GOAL_A_Y);
        onGoalA = 1;
    }

    // ---------------- 2, 3. Back out, through P1, score it on goal A ----------------
    if (!backOutTo(SK_GOAL_BACK_OUT, turn1, [](double done) { if (done >= SK_GOAL_BACK_OUT) flipperDown(); })) return;
    {
        std::vector<Node> nodes = {
            hereNode(),
            along(p1Pose, -FP_PIN_LEAD_IN, 8.0, 0),
            node(p1Pose.x, p1Pose.y, p1Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(p1Pose, FP_PIN_LEAD_OUT, 0, 7.0),
            along(p1Score, -FP_GOAL_LEAD_IN, 7.0, 0),
            p1Score,
        };
        if (!pickUpAndScore(nodes, 2, SK_P1_X, SK_P1_Y, -24.0, -24.0, height(onGoalA + 1), SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG,
                            SK_PIN_SLOW_MS, SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS)) return;
        onGoalA++;
    }

    // ---------------- 4, 5. Back out, through P2, score it on goal A ----------------
    if (!backOutTo(SK_P1_BACK_OUT, turn2)) return;
    {
        std::vector<Node> nodes = {
            hereNode(),
            along(p2Pose, -FP_PIN_LEAD_IN),
            node(p2Pose.x, p2Pose.y, p2Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(p2Pose, FP_PIN_LEAD_OUT, 0, 8.0),
            along(p2Score, -FP_GOAL_LEAD_IN, 8.0, 0),
            p2Score,
        };
        if (!pickUpAndScore(nodes, 2, SK_P2_X, SK_P2_Y, -48.0, -48.0, height(onGoalA + 1), SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG,
                            SK_PIN_SLOW_MS, SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS)) return;
        onGoalA++;
    }

    // ---------------- 6. Matchloader -> goal A ----------------
    if (!goalToLoaderLine(SK_GOAL_A_BACK_OUT, loopGoalAToLoader())) return;
    for (int i = 0; i < SK_GOAL_A_LOADS; i++) {
        if (!intoLoader(height(onGoalA + 1))) return;
        if (!outOfLoader(loopLoaderToGoalA())) return;
        if (!scoreInto(loadAScore, height(onGoalA + 1), SK_GOAL_A_X, SK_GOAL_A_Y, SK_GOAL_A_TAG)) return;
        onGoalA++;
        if (!goalToLoaderLine(SK_GOAL_A_BACK_OUT, loopGoalAToLoader())) return;
    }

    // ---------------- 7. Matchloader -> goal B ----------------
    for (int i = 0; i < SK_GOAL_B_LOADS; i++) {
        if (!intoLoader(height(onGoalB + 1))) return;
        if (!outOfLoader(loopLoaderToGoalB())) return;
        if (!scoreInto(goalBScore, height(onGoalB + 1), SK_GOAL_B_X, SK_GOAL_B_Y, SK_GOAL_B_TAG)) return;
        onGoalB++;
        if (!goalToLoaderLine(SK_GOAL_B_BACK_OUT, loopGoalBToLoader())) return;
    }

    // ---------------- 9. Matchloader -> the middle goal (the goal A loop, then on between the goals; arrow 8 is skipped) ----------------
    for (int i = 0; i < SK_MIDDLE_LOADS; i++) {
        if (!intoLoader(middleHeight(onMiddle + 1))) return;
        if (!outOfLoader(loopLoaderToGoalA())) return;
        std::vector<Node> nodes = {
            hereNode(),
            node(SK_MIDDLE_WAY_X, SK_MIDDLE_WAY_Y, SK_MIDDLE_SW_HEADING),
            along(middleSW, -FP_GOAL_LEAD_IN, 7.0, 0),
            middleSW,
        };
        if (!scoreOnGoal(nodes, middleHeight(onMiddle + 1), SK_MIDDLE_X, SK_MIDDLE_Y, SK_MIDDLE_TAG, SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS)) return;
        onMiddle++;
        if (i + 1 < SK_MIDDLE_LOADS) {
            // Back to the loader line: straight back toward the gap between the goals, then the goal A hook
            if (!goalToLoaderLine(SK_GOAL_BACK_OUT, loopGoalAToLoader())) return;
        }
    }

    // ---------------- 10, 11. Loop out of the middle goal, round the small pin, through Y1 from the upper left, score it from the upper right ----------------
    {
        Node start = hereNode();
        Node off = along(start, -SK_Y1_BACK);
        off.speed = SK_BACK_OUT_SPEED;
        if (!reverseThrough({start, off, loopNode(SK_Y1_L1_X, SK_Y1_L1_Y, SK_Y1_L1_HEADING), loopNode(SK_Y1_L2_X, SK_Y1_L2_Y, SK_Y1_L2_HEADING),
                             loopNode(SK_Y1_L3_X, SK_Y1_L3_Y, SK_Y1_L3_HEADING)}, true)) return;
        std::vector<Node> nodes = {
            hereNode(),
            node(SK_Y1_W1_X, SK_Y1_W1_Y, SK_Y1_W1_HEADING),
            node(SK_Y1_W2_X, SK_Y1_W2_Y, SK_Y1_W2_HEADING),
            node(SK_Y1_W3_X, SK_Y1_W3_Y, SK_Y1_W3_HEADING),
            along(y1Pose, -FP_PIN_LEAD_IN, 8.0, 0),
            node(y1Pose.x, y1Pose.y, y1Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(y1Pose, FP_PIN_LEAD_OUT, 0, 7.0),
            along(middleNE, -FP_GOAL_LEAD_IN, 7.0, 0),
            middleNE,
        };
        if (!pickUpAndScore(nodes, 5, SK_Y1_X, SK_Y1_Y, 24.0, 24.0, middleHeight(onMiddle + 1), SK_MIDDLE_X, SK_MIDDLE_Y, SK_MIDDLE_TAG,
                            SK_PIN_SLOW_MS, SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS)) return;
        onMiddle++;
    }

    // ---------------- 12, 13. Loop back up the diagonal onto Y2, through Y2, loop in the corner, back to the middle goal ----------------
    {
        Node start = hereNode();
        Node off = along(start, -SK_Y2_BACK);
        off.speed = SK_BACK_OUT_SPEED;
        if (!reverseThrough({start, off, loopNode(SK_Y2_L1_X, SK_Y2_L1_Y, SK_Y2_L1_HEADING), loopNode(SK_Y2_L2_X, SK_Y2_L2_Y, SK_Y2_L2_HEADING),
                             loopNode(SK_Y2_L3_X, SK_Y2_L3_Y, SK_Y2_L3_HEADING), loopNode(SK_Y2_L4_X, SK_Y2_L4_Y, SK_Y2_L4_HEADING)}, true)) return;
        std::vector<Node> nodes = {
            hereNode(),
            along(y2Pose, -FP_PIN_LEAD_IN, 8.0, 0),
            node(y2Pose.x, y2Pose.y, y2Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(y2Pose, FP_PIN_LEAD_OUT, 0, 6.0),
            loopNode(SK_Y2_C1_X, SK_Y2_C1_Y, SK_Y2_C1_HEADING),
            loopNode(SK_Y2_C2_X, SK_Y2_C2_Y, SK_Y2_C2_HEADING),
            loopNode(SK_Y2_C3_X, SK_Y2_C3_Y, SK_Y2_C3_HEADING),
            loopNode(SK_Y2_C4_X, SK_Y2_C4_Y, SK_Y2_C4_HEADING),
            node(SK_Y2_GAP_X, SK_Y2_GAP_Y, SK_Y2_GAP_HEADING),
            along(middleNE, -FP_GOAL_LEAD_IN, 8.0, 0),
            middleNE,
        };
        if (!pickUpAndScore(nodes, 2, SK_Y2_X, SK_Y2_Y, 48.0, 48.0, middleHeight(onMiddle + 1), SK_MIDDLE_X, SK_MIDDLE_Y, SK_MIDDLE_TAG,
                            SK_PIN_SLOW_MS, SK_SCORE_SLOW_MS, SK_SETTLE_MS, SK_RELEASE_MS, SK_LOWER_BEFORE_OPEN_MS)) return;
        onMiddle++;
    }

    // Done: back off a little and lower
    if (!backStraight(SK_GOAL_BACK_OUT)) return;
    liftDown();
    clawOff();
    while (!liftIsDown()) {
        if (!waitMs(10)) return;
    }
    liftCoast();
    note("done");
}

}  // namespace

void skillsAuton() {
    runSkills();
    finishSafe();
}
