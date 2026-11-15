// =====================================================================
//   4-PIN QUALIFICATION AUTON (15 s)
// =====================================================================
// Everything for this auton is in this file: the route, the curve follower, the lift / claw / flipper / intake
// timing, the tip guard, the AI Vision aiming and the 15 s timer. It shares nothing with skillsauto.cpp.
//
// The route (drawn for RED, back on the BOTTOM wall toggle; the other three starts are the same route turned
// or flipped, see include/fourpin.hpp):
//   start  lift up FP_TOGGLE_LIFT_DEG and straight back down (drops the toggles), then drive off the toggle
//   1      curve left into the alliance goal, claw open, lift down; flip the preload onto the goal
//   2      back out and curve BACKWARDS into the red pin of the 4-pin cluster (intake on the back of the robot)
//   3      forward around the small red pin, pick up pin 3 with the claw without stopping, curve left into the
//          goal while the lift goes up, let go
//   4      back out while the lift drops, swing the nose at pin 4, pick it up on the way through, curve left
//          into the goal (one height higher), let go
//   5      back out, swing the nose at the wall cup, drive into it and the wall with the claw spinning closed,
//          back off with the lift going up to score, flip the intake pin over into the cup on the way, drive into the
//          goal, flipper back down, let go and lower. The lift goes limp just before 15 s.
//
// How it drives: every move is a smooth curve (no turn-in-place, no straight-line PID moves). A curve is a few
// "nodes" (a spot + the way the robot points there); the follower drives through them with the speed planned
// ahead (slower in tight curves, slowing down in time for the end). It only stops where it has to: at the goal
// (to let go), at the intake pin, and where it switches between forwards and backwards.
//
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// All the spots below are for RED BOTTOM. The code turns / flips them for the other starts.
// =====================================================================
#include "robot.hpp"
#include "fourpin.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <vector>

#ifndef RW_SIM_PROS
#include "pros/ai_vision.hpp"   // the simulator has no AI Vision sensor
#endif

// Devices that live in main.cpp and aren't in robot.hpp
extern pros::Motor flipper;
extern pros::Motor intake;
void fieldMclPinPickedUp(double x, double y);   // src/field-mcl.cpp

// =====================================================================
//   ROBOT SIZE (estimates for the rebuilt robot: change these when it's measured)
// =====================================================================
constexpr double FP_FIELD_HALF = 70.25;
constexpr double FP_CENTER_TO_FRONT = 9.0;    // tracking center to the front of the robot (18" long)
constexpr double FP_CENTER_TO_BACK = 9.0;     // tracking center to the back
constexpr double FP_HALF_WIDTH = 8.5;         // 17" wide
// Tracking center to the middle of a pin / cup when the claw closes on it (also where the flipper drops a pin)
constexpr double FP_CLAW_REACH = 7.5;
// Tracking center to the TIP of the cluster pin when the intake has hold of it. 9 = the back edge of the robot,
// more = the intake reaches past the back edge.
constexpr double FP_INTAKE_REACH = 10.5;

// =====================================================================
//   FIELD SPOTS (red, bottom wall)
// =====================================================================
constexpr double FP_START_X = 0.0;
constexpr double FP_START_Y = -FP_FIELD_HALF + FP_CENTER_TO_BACK;   // -61.25: back on the wall
constexpr double FP_START_HEADING = 0.0;
constexpr double FP_GOAL_X = -23.55, FP_GOAL_Y = -47.09;       // alliance goal
constexpr double FP_PIN2_TIP_X = 17.0, FP_PIN2_TIP_Y = -23.55; // tip of the red cluster pin that points at the goal side
constexpr double FP_PIN3_X = -23.55, FP_PIN3_Y = -23.55;       // field pin on the diagonal, near the middle
constexpr double FP_PIN4_X = -47.09, FP_PIN4_Y = -47.09;       // field pin on the diagonal, near the corner
constexpr double FP_CUP5_X = -26.2, FP_CUP5_Y = -68.7;         // wall cup (the left one of the pair below the goal)

// =====================================================================
//   ROUTE SHAPE (the way the robot points at each spot, and the spots where it switches direction)
// =====================================================================
constexpr double FP_PRELOAD_HEADING = 265.0;    // pointing at the goal for the preload
// Backing into the cluster pin. 270 would be straight in line with the pin, but an 18" robot doesn't fit between
// the pin's tip and the small red pin at (0, -23.55), so it comes in at an angle with its nose below that pin.
constexpr double FP_PIN2_HEADING = 228.0;
constexpr double FP_PIN3_HEADING = 300.0;       // driving through pin 3
constexpr double FP_PIN3_SCORE_HEADING = 165.0; // scoring pin 3 (comes at the goal from above)
constexpr double FP_TURN4_X = -25.4, FP_TURN4_Y = -27.1, FP_TURN4_HEADING = 222.0;   // backs up to here, nose swung at pin 4
constexpr double FP_PIN4_HEADING = 225.0;       // driving through pin 4, along the diagonal between the two red goals
constexpr double FP_PIN4_SCORE_HEADING = 65.0;  // scoring pin 4 (comes at the goal from the lower left)
constexpr double FP_TURN5_X = -43.5, FP_TURN5_Y = -51.0, FP_TURN5_HEADING = 135.0;   // backs up to here, nose swung at the cup
constexpr double FP_CUP5_HEADING = 140.0;       // driving into the wall cup (squarer than this and the back corner gets too close to the goal)
constexpr double FP_CUP5_SCORE_HEADING = 85.0;  // scoring the cup (comes at the goal from the left)
constexpr double FP_CUP5_SCORE_RUN = 16.0;      // inches of straight run into the goal for the last score
constexpr double FP_GOAL_LEAD_IN = 6.0;         // inches of straight run into the goal (the goal has to slide into the claw opening)
// Inches straight back before the nose starts to swing (so the front corners miss the goal)
constexpr double FP_PRELOAD_BACK_OUT = 5.0;     // from the goal after the preload
constexpr double FP_PIN3_BACK_OUT = 6.0;        // from the goal after pin 3
constexpr double FP_PIN4_BACK_OUT = 8.0;        // from the goal after pin 4
constexpr double FP_WALL_BACK_OUT = 2.0;        // from the wall cup
constexpr double FP_START_STRAIGHT = 5.0;       // inches straight off the toggle before it starts to curve
constexpr double FP_PIN2_STRAIGHT_OUT = 7.0;    // inches straight away from the cluster pin before it curves (its nose is next to the small red pin)
constexpr double FP_PIN_LEAD_IN = 5.0;          // inches of straight run into a pin
constexpr double FP_PIN_LEAD_OUT = 3.0;         // inches straight on after grabbing before it curves away

// Fine tuning per spot (inches, + = further forward)
constexpr double FP_PRELOAD_EXTRA = 0.0;
constexpr double FP_PIN2_EXTRA = 4.0;           // + = backs further into the cluster pin (the auton line guard still stops it)
constexpr double FP_PIN3_GRAB_EXTRA = 0.0;
constexpr double FP_PIN3_SCORE_EXTRA = 0.0;
constexpr double FP_PIN4_GRAB_EXTRA = 0.0;
constexpr double FP_PIN4_SCORE_EXTRA = 0.0;
constexpr double FP_CUP5_SCORE_EXTRA = 1.0;

// The auton line (the double diagonal line through the cluster). The back corners stay at least this far on
// our side of it while backing into the cluster pin; if one gets closer the robot stops backing right there.
constexpr double FP_LINE_MARGIN = 1.5;

// =====================================================================
//   DRIVING
// =====================================================================
constexpr double FP_MAX_SPEED = 56.0;        // top speed, in/s (the drive tops out around 58-60)
constexpr double FP_WHEEL_MAX = 58.0;        // fastest one side is asked to go (the outside of a curve)
constexpr double FP_ACCEL = 170.0;           // speeding up, in/s per second
constexpr double FP_DECEL = 150.0;           // slowing down for a stop or a tight curve
constexpr double FP_LAT_ACCEL = 160.0;       // how hard it corners (in/s^2): lower = slower in curves
constexpr double FP_MAX_TURN_DPS = 240.0;    // fastest it's allowed to spin in a curve (degrees/s)
constexpr double FP_MIN_SPEED = 5.0;         // never slower than this while it's still driving
constexpr double FP_CUSP_SPEED = 8.0;        // speed left at a forwards <-> backwards switch (it doesn't settle there)
constexpr double FP_GRAB_SPEED = 30.0;       // speed through a pin while the claw closes on it (in/s)
constexpr double FP_INTAKE_SPEED = 26.0;     // speed for the last inches backing into the cluster pin
constexpr double FP_WALL_SPEED = 26.0;       // speed into the wall cup
// Steering: the same as chasing a point this far ahead on the curve (inches). Lower = follows tighter, twitchier.
constexpr double FP_STEER_DIST = 6.0;
constexpr double FP_STEER_PREVIEW_S = 0.08;  // looks this far ahead in time for the curve's bend (covers the motors' lag)
constexpr double FP_HEADING_GAIN = 6.0;      // pointing correction at low speed (degrees/s of turn per degree off)
constexpr double FP_FF_ACCEL_MAX = 300.0;    // most wheel acceleration the feedforward asks for (in/s^2)
constexpr double FP_SPEED_FEEDBACK = 0.10;   // extra volts per in/s a side is off its speed
// Coming to a stop: on top of FP_DECEL, the speed is never more than this many in/s per inch left to go, so the
// last inch is gentle and it doesn't slide past the spot. Lower = softer and a bit slower.
constexpr double FP_STOP_GAIN = 8.0;
constexpr double FP_END_TOLERANCE = 0.6;     // inches from the end of a curve that counts as there
// With the lift up it drives slower so it doesn't tip. Full speed below LO, FP_TALL_SPEED at HI and above.
constexpr double FP_TALL_LIFT_LO = 150.0;    // lift degrees
constexpr double FP_TALL_LIFT_HI = 450.0;
constexpr double FP_TALL_SPEED = 30.0;       // in/s
constexpr double FP_TALL_ACCEL = 90.0;       // speeding up / slowing down with the lift at HI
// While the lift is up or headed up to score (and on the way back down after letting go), the drive is capped at
// this too, so the lift can't tip it. Lower = safer, slower. Only for heights more than FP_MOVING_SLOW_MARGIN above
// the 2nd pin's scoring height (FP_PIN3_STACK), so scoring the 2nd pin on the goal isn't slowed.
constexpr double FP_MOVING_SLOW_MARGIN = 20.0;  // lift degrees above the 2nd pin's height
constexpr double FP_MOVING_SPEED = 20.0;        // in/s
constexpr double FP_MOVING_ACCEL = 70.0;        // speeding up, in/s per second
// A move ends if the robot is being told to drive but hasn't moved for this long (it's against something)
constexpr int FP_STALL_MS = 250;

// =====================================================================
//   LIFT
// =====================================================================
constexpr double FP_TOGGLE_LIFT_DEG = 50.0;      // start: lift goes up this far to drop the toggles...
constexpr int FP_TOGGLE_UP_MAX_MS = 400;         // ...(gives up after this long)
constexpr double FP_TOGGLE_GO_DEG = 20.0;        // ...and the robot drives off once it's back down to this
constexpr int FP_TOGGLE_DOWN_MAX_MS = 400;
// Scoring heights: which entry of ALLIANCE_HEIGHTS in main.cpp each one uses (1 = lowest). The preload is
// flipped onto the empty goal with the lift down, so pin 3 is the 2nd thing on the goal.
constexpr int FP_PIN3_STACK = 2;
constexpr int FP_PIN4_STACK = 3;
constexpr int FP_CUP5_STACK = 4;
constexpr double FP_LIFT_READY_TOLERANCE = 15.0; // lift counts as up when within this of its height
// Pickup hop: when the claw closes on a pin / cup off the field (not the intake), the lift hops up a little, the
// same as driver control's pin pickup (PIN_LIFT_HEIGHT / PIN_LIFT_MIN_RISE / PIN_LIFT_VOLTAGE in robot.hpp).
// No stall check: the route already knows when the claw closes, so it just waits this long for the claw to shut.
constexpr int FP_PICKUP_LIFT_DELAY_MS = 100;
// The robot won't go closer to the goal than this until the lift is up (so the pin can't hit the stack)
constexpr double FP_SCORE_GATE_IN = 7.0;
constexpr int FP_GATE_MAX_MS = 1500;             // ...but it stops waiting after this long
// Last score (the cup): instead of stopping at the gate to wait for the lift, it works out how long the lift still
// needs and drives just fast enough (never over FP_MOVING_SPEED) to reach the gate as the lift gets there. It only
// stops at the gate if the lift is later than this guess.
constexpr double FP_LIFT_UP_DPS = 550.0;         // about how fast the lift climbs (deg/s); lower = drives in slower
constexpr double FP_GATE_CREEP_MIN = 4.0;        // never slower than this while creeping in (in/s)
constexpr double FP_GATE_ETA_BUFFER_S = 0.1;     // aims to reach the gate this long after the lift is up (so it never brakes there)
constexpr double FP_LOWER_AFTER_BACK_IN = 2.0;   // after letting go, the lift starts down once it has backed this far
constexpr double FP_CUP_FLIP_LIFT_DEG = 30.0;    // the pin flips into the cup once the lift has climbed this far (it keeps going up)
// Lowering: full power down, then a gentle push for the last bit, then a light hold on the bottom
constexpr double FP_GROUND_FAST_ABOVE = 30.0;    // full power down while above this
constexpr int FP_GROUND_PUSH_MV = 4000;
constexpr int FP_GROUND_HOLD_MV = 1200;
constexpr int FP_GROUND_PUSH_MAX_MS = 400;

// =====================================================================
//   CLAW, FLIPPER, INTAKE
// =====================================================================
constexpr int FP_CLAW_RPM = 200;                 // claw full speed (closing and opening)
constexpr int FP_CLAW_FAST_MS = 1000;            // closes at full speed this long, then drops to the hold
constexpr int FP_CLAW_HOLD_MV = 2000;            // hold squeeze (same as driver control)
// Opening: full speed (move_velocity(-FP_CLAW_RPM)) the whole time it's open, the same as holding R2 in driver
// control. Motor off = it closes by itself.
constexpr double FP_CLAW_OPEN_DIST = 18.0;       // claw starts opening this many inches before a pin
constexpr double FP_GRAB_TOLERANCE = 3.0;        // claw closes when the pin is this close to the middle of the claw
constexpr int FP_SCORE_RELEASE_MS = 150;         // wait at the goal after the claw starts opening
constexpr double FP_CLAW_OFF_AFTER_BACK_IN = 5.0;   // claw motor off once it has backed this far from the goal
constexpr int FP_FLIP_RPM = 75;                  // flipper up
constexpr int FP_FLIP_DOWN_RPM = 50;             // flipper back down
constexpr double FP_PRELOAD_CLAW_OPEN_AFTER_IN = 6.0;   // claw starts opening once it has driven this far off the toggle
constexpr int FP_PRELOAD_FLIP_WAIT_MS = 500;     // preload: stopped at the goal this long while it flips, then backs away
constexpr double FP_FLIPPER_DOWN_AFTER_IN = 5.0; // after the preload, flipper goes back down once backed this far (after lift down + claw off)
constexpr double FP_FLIPPER_STALL_DEG = 5.0;     // going down: moved less than this...
constexpr int FP_FLIPPER_STALL_MS = 500;         // ...for this long = it's home, motor off
constexpr int FP_FLIPPER_DOWN_MAX_MS = 2000;
constexpr double FP_FLIPPER_BACK_BEFORE_IN = 7.0;   // last score: flipper goes back down this far before the goal (once the lift is up)
constexpr int FP_INTAKE_MV = 12000;
constexpr double FP_INTAKE_ON_DIST = 14.0;       // intake starts this many inches before the cluster pin
constexpr int FP_INTAKE_DWELL_MS = 600;          // wait at the cluster pin while the intake takes it
constexpr int FP_INTAKE_KEEP_MS = 6000;          // intake keeps running this long after it leaves the pin
constexpr int FP_WALL_PUSH_MV = 6000;            // push into the wall on the cup...
constexpr int FP_WALL_PUSH_MS = 500;             // ...for this long
// The drive into the wall ends when the robot stops moving (it's on the wall). Backup: it also ends once odometry
// says the front corner is this far PAST the wall, which can only mean it's there.
constexpr double FP_WALL_OVERDRIVE = 1.0;

// =====================================================================
//   TIP GUARD
// =====================================================================
// If the robot tips with the lift up, the lift drops and the robot slows right down. When it's level again the
// lift goes back to the height it was heading for, and the route carries on.
constexpr bool FP_TIP_GUARD = true;
constexpr double FP_TIP_ROLL_LIMIT = 13.0;       // degrees (same axis and limit as driver control's tip protection)
constexpr double FP_TIP_ROLL_CLEAR = 8.0;        // level again below this
constexpr int FP_TIP_CONFIRM_TICKS = 3;          // 10 ms each
constexpr double FP_TIP_MIN_LIFT = 60.0;         // only counts with the lift above this
constexpr double FP_TIP_SPEED = 12.0;            // in/s while it recovers

// =====================================================================
//   15 SECOND TIMER
// =====================================================================
constexpr int FP_AUTON_MS = 15000;
constexpr int FP_COAST_BEFORE_END_MS = 150;      // lift PID off and motors limp this long before the end

// =====================================================================
//   AI VISION
// =====================================================================
// Pin camera: on the drivetrain under the claw, in the middle, looking straight ahead. On the way into a pin it
// finds the pin / cup and nudges the curve sideways so the pin ends up in the middle of the claw.
constexpr bool FP_USE_PIN_VISION = true;
constexpr int FP_PIN_CAM_PORT = 5;
constexpr int FP_PIN_CLASS_ID = -1;              // -1 = any object the Override model finds, or one class id
constexpr double FP_PIN_CAM_FORWARD = 6.0;       // inches the camera sits in front of the tracking center
constexpr double FP_PIN_CAM_YAW = 0.0;           // degrees the camera points right of straight ahead (0 = straight)
constexpr bool FP_PIN_CAM_UPSIDE_DOWN = false;   // true if it's mounted upside down (left and right swap in its picture)
// Goal camera: finds the goal's AprilTag and nudges the scoring approach the same way. OFF for now (the
// approach uses odometry + the distance sensors). Set true once the second sensor is on and aimed.
constexpr bool FP_USE_GOAL_TAG = false;
constexpr int FP_GOAL_CAM_PORT = 21;
constexpr int FP_GOAL_TAG_ID = -1;               // -1 = any tag, or the goal's tag id
constexpr double FP_GOAL_CAM_FORWARD = 6.0;
constexpr double FP_GOAL_CAM_YAW = 0.0;
constexpr bool FP_GOAL_CAM_UPSIDE_DOWN = false;
// Both cameras
constexpr double FP_CAM_WIDTH_PX = 320.0;
constexpr double FP_CAM_FOV_DEG = 74.0;          // side to side
constexpr double FP_AIM_WINDOW_DEG = 14.0;       // only trusts something this close to where the pin / tag should be
constexpr double FP_AIM_MAX_SHIFT = 3.0;         // most the curve is nudged sideways (inches)
constexpr double FP_AIM_MAX_DIST = 30.0;         // starts aiming this far out
constexpr double FP_AIM_MIN_DIST = 5.0;          // stops updating this close (the pin is under the camera's view)
constexpr double FP_AIM_SMOOTH = 0.25;           // 0-1: how fast the nudge follows the camera (lower = smoother)
constexpr int FP_AIM_PERIOD_MS = 30;

namespace {

// =====================================================================
//   SMALL MATH
// =====================================================================
constexpr double FP_PI = 3.14159265358979323846;
double rad(double deg) { return deg * FP_PI / 180.0; }
double wrap180(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0) deg += 360.0;
    return deg - 180.0;
}
double clampTo(double value, double low, double high) { return value < low ? low : (value > high ? high : value); }

struct Vec { double x, y; };
Vec forwardOf(double headingDeg) { return {std::sin(rad(headingDeg)), std::cos(rad(headingDeg))}; }   // unit vector the heading points along
Vec rightOf(double headingDeg) { return {std::cos(rad(headingDeg)), -std::sin(rad(headingDeg))}; }     // unit vector to its right

// =====================================================================
//   WHICH START (the route is written for red bottom, and turned / flipped for the others)
// =====================================================================
bool fpTurned = false;     // true = turned 180 degrees (blue)
bool fpMirrored = false;   // true = flipped across the diagonal the field pins sit on (left / right wall starts)

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
// The other way: field -> the red bottom drawing
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

// A spot on the route and the way the robot points there. tangentIn / tangentOut shape the curve on each side
// of it (inches; 0 = automatic). speed = top speed near this spot (0 = no extra limit).
struct Node {
    double x, y, heading;
    double tangentIn = 0, tangentOut = 0;
    double speed = 0;
};
Node node(double x, double y, double heading, double tangentIn = 0, double tangentOut = 0, double speed = 0) {
    Node n;
    n.x = x; n.y = y; n.heading = heading; n.tangentIn = tangentIn; n.tangentOut = tangentOut; n.speed = speed;
    return n;
}
// The same pose moved along its own heading (+ = forward, - = backward)
Node along(const Node& from, double inches, double tangentIn = 0, double tangentOut = 0, double speed = 0) {
    Vec f = forwardOf(from.heading);
    return node(from.x + f.x * inches, from.y + f.y * inches, from.heading, tangentIn, tangentOut, speed);
}
// Robot pose with the claw (reach inches ahead of center) on a spot
Node clawOn(double x, double y, double heading, double reach) {
    Vec f = forwardOf(heading);
    return node(x - f.x * reach, y - f.y * reach, heading);
}
// Where the robot is now, in route coordinates
Node hereNode() {
    Vec p = toRoute({x_pos, y_pos});
    return node(p.x, p.y, headingToRoute(getInertialHeading()));
}

// =====================================================================
//   AUTON CLOCK
// =====================================================================
uint32_t fpStartMs = 0;
bool fpTimeUp = false;
int fpElapsed() { return static_cast<int>(pros::millis() - fpStartMs); }
// One line per step on the terminal (PROS terminal on the robot, or the simulator's output), with the auton time
void note(const char* what) {
    std::printf("4pin %5.2f s  %-28s x %6.1f  y %6.1f  heading %5.0f  lift %4.0f\n", fpElapsed() / 1000.0, what, x_pos, y_pos,
                std::fmod(std::fmod(getInertialHeading(), 360.0) + 360.0, 360.0), getDR4BPosition());
}

// =====================================================================
//   LIFT
// =====================================================================
enum class LiftMode { idle, pid, ground, coast, direct, hop };
LiftMode fpLiftMode = LiftMode::idle;
double fpLiftGoal = 0.0;          // height the route wants (pid mode)
uint32_t fpGroundStartMs = 0;
bool fpGroundPushing = false;
uint32_t fpGroundPushStartMs = 0;
bool fpLiftPending = false;       // a pickup hop waiting for its start time
uint32_t fpLiftPendingAtMs = 0;
double fpHopThen = -1.0;          // height to go to after the hop (-1 = stay at the hop height)
double fpHopGoal = 0.0;
uint32_t fpHopStartMs = 0;

// Lift to a height with the lift PID in main.cpp
void liftTo(double height) {
    fpLiftPending = false;
    fpLiftMode = LiftMode::pid;
    fpLiftGoal = height;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bTarget = height;
    dr4bManualMode = false;
}
// Pickup hop, FP_PICKUP_LIFT_DELAY_MS after the claw starts closing: full power up to
// max(PIN_LIFT_HEIGHT, here + PIN_LIFT_MIN_RISE) like driver control, then on to thenHeight (-1 = hold there)
void liftHopThen(double thenHeight) {
    fpLiftPending = true;
    fpLiftPendingAtMs = pros::millis() + FP_PICKUP_LIFT_DELAY_MS;
    fpHopThen = thenHeight;
}
void startHop() {
    fpLiftPending = false;
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
// All the way down as fast as it goes, then a light hold on the bottom
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
// Lift PID off and the motor limp (the end of auton: the lift falls by itself)
void liftCoast() {
    fpLiftPending = false;
    fpLiftMode = LiftMode::coast;
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    DR4B.move_voltage(0);
}
bool liftReady() {
    return fpLiftMode == LiftMode::pid && getDR4BPosition() >= fpLiftGoal - FP_LIFT_READY_TOLERANCE;
}
bool liftIsDown() { return getDR4BPosition() <= 5.0; }

// =====================================================================
//   CLAW
// =====================================================================
enum class ClawMode { off, opening, closing, hold };
ClawMode fpClawMode = ClawMode::off;
uint32_t fpClawSinceMs = 0;
void clawSet(ClawMode mode) {
    if (fpClawMode == mode) return;
    if (mode == ClawMode::closing && fpClawMode == ClawMode::hold) return;       // already holding
    fpClawMode = mode;
    fpClawSinceMs = pros::millis();
}
void clawOpen() { clawSet(ClawMode::opening); }
void clawClose() { clawSet(ClawMode::closing); }
// Restarts the full-speed squeeze (the wall cup: the 1 s starts when it's against the wall)
void clawCloseAgain() { fpClawMode = ClawMode::closing; fpClawSinceMs = pros::millis(); }
void clawOff() { clawSet(ClawMode::off); }

// =====================================================================
//   FLIPPER + INTAKE
// =====================================================================
enum class FlipMode { off, up, down };
FlipMode fpFlipMode = FlipMode::off;
uint32_t fpFlipSinceMs = 0;
double fpFlipCheckPos = 0.0;
uint32_t fpFlipCheckMs = 0;
void flipperUp() {
    if (fpFlipMode == FlipMode::up) return;
    fpFlipMode = FlipMode::up;
    fpFlipSinceMs = pros::millis();
}
void flipperDown() {
    if (fpFlipMode != FlipMode::up) return;   // only from the flipped position
    fpFlipMode = FlipMode::down;
    fpFlipSinceMs = pros::millis();
    fpFlipCheckPos = flipper.get_position();
    fpFlipCheckMs = fpFlipSinceMs;
}

bool fpIntakeOn = false;
uint32_t fpIntakeOffAtMs = 0;   // 0 = no off time set

// =====================================================================
//   TIP GUARD
// =====================================================================
bool fpTipping = false;
int fpTipTicks = 0;
double fpTipRestoreGoal = 0.0;

// =====================================================================
//   AI VISION AIM
// =====================================================================
// While aiming, the curve is nudged sideways by fpAimShift (field inches) so it ends on what the camera sees.
enum class AimKind { none, pin, goalTag };
AimKind fpAimKind = AimKind::none;
Vec fpAimSpot = {0, 0};      // where the pin / goal should be (field)
Vec fpAimShift = {0, 0};     // current nudge (field)
uint32_t fpAimLastMs = 0;

#ifndef RW_SIM_PROS
pros::AIVision& pinCamera() {
    static pros::AIVision camera(FP_PIN_CAM_PORT);
    return camera;
}
pros::AIVision& goalCamera() {
    static pros::AIVision camera(FP_GOAL_CAM_PORT);
    return camera;
}
#endif

void visionStart() {
#ifndef RW_SIM_PROS
    if (FP_USE_PIN_VISION) pinCamera().enable_detection_types(pros::AivisionModeType::objects);
    if (FP_USE_GOAL_TAG) goalCamera().enable_detection_types(pros::AivisionModeType::tags);
#endif
}

// Finds the thing the camera should be looking at: the bearing (degrees right of straight ahead, from the
// camera) of the detection closest to expectedBearing. false = nothing believable in view.
bool cameraBearing(AimKind kind, double expectedBearing, double& bearing) {
#ifdef RW_SIM_PROS
    // Simulator: there's no camera. SIM_PIN_SHIFT="dx,dy" pretends every pin sits that far (route inches) from
    // its spot and that the camera sees it perfectly, to test the aiming.
    const char* text = std::getenv("SIM_PIN_SHIFT");
    double dx = 0, dy = 0;
    if (kind != AimKind::pin || text == nullptr || std::sscanf(text, "%lf,%lf", &dx, &dy) != 2) return false;
    Vec moved = toField({dx, dy});
    double heading = getInertialHeading();
    Vec f = forwardOf(heading);
    double cx = x_pos + f.x * FP_PIN_CAM_FORWARD, cy = y_pos + f.y * FP_PIN_CAM_FORWARD;
    bearing = wrap180(std::atan2(fpAimSpot.x + moved.x - cx, fpAimSpot.y + moved.y - cy) * 180.0 / FP_PI - heading);
    return std::fabs(bearing - expectedBearing) <= FP_AIM_WINDOW_DEG;
#else
    bool tags = kind == AimKind::goalTag;
    if (tags ? !FP_USE_GOAL_TAG : !FP_USE_PIN_VISION) return false;
    pros::AIVision& camera = tags ? goalCamera() : pinCamera();
    double yaw = tags ? FP_GOAL_CAM_YAW : FP_PIN_CAM_YAW;
    int32_t count = camera.get_object_count();
    if (count <= 0 || count > 64) return false;   // nothing, or the sensor isn't answering (PROS_ERR)
    bool found = false;
    double bestOff = FP_AIM_WINDOW_DEG;
    for (int32_t i = 0; i < count; i++) {
        pros::AIVision::Object seen = camera.get_object(i);
        double centerPx = 0;
        if (tags) {
            if (!(seen.type & pros::E_AIVISION_DETECTED_TAG)) continue;
            if (FP_GOAL_TAG_ID >= 0 && seen.id != FP_GOAL_TAG_ID) continue;
            centerPx = (seen.object.tag.x0 + seen.object.tag.x1 + seen.object.tag.x2 + seen.object.tag.x3) / 4.0;
        } else {
            if (!(seen.type & pros::E_AIVISION_DETECTED_OBJECT)) continue;
            if (FP_PIN_CLASS_ID >= 0 && seen.id != FP_PIN_CLASS_ID) continue;
            centerPx = seen.object.element.xoffset + seen.object.element.width / 2.0;
        }
        if (tags ? FP_GOAL_CAM_UPSIDE_DOWN : FP_PIN_CAM_UPSIDE_DOWN) centerPx = FP_CAM_WIDTH_PX - centerPx;
        double seenBearing = (centerPx - FP_CAM_WIDTH_PX / 2.0) * (FP_CAM_FOV_DEG / FP_CAM_WIDTH_PX) + yaw;
        double off = std::fabs(seenBearing - expectedBearing);
        if (off <= bestOff) {
            bestOff = off;
            bearing = seenBearing;
            found = true;
        }
    }
    return found;
#endif
}

void aimAt(AimKind kind, double routeX, double routeY) {
    fpAimKind = kind;
    fpAimSpot = toField({routeX, routeY});
    fpAimLastMs = 0;
}
void aimStop() { fpAimKind = AimKind::none; }

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

    double forwardOffset = fpAimKind == AimKind::goalTag ? FP_GOAL_CAM_FORWARD : FP_PIN_CAM_FORWARD;
    double heading = getInertialHeading();
    Vec f = forwardOf(heading), r = rightOf(heading);
    double cx = x_pos + f.x * forwardOffset, cy = y_pos + f.y * forwardOffset;
    double dx = fpAimSpot.x - cx, dy = fpAimSpot.y - cy;
    double dist = std::hypot(dx, dy);
    if (dist > FP_AIM_MAX_DIST || dist < FP_AIM_MIN_DIST) return;
    double expected = wrap180(std::atan2(dx, dy) * 180.0 / FP_PI - heading);
    if (std::fabs(expected) > FP_CAM_FOV_DEG / 2.0 - 4.0) return;   // not in view yet
    double seen = 0;
    if (!cameraBearing(fpAimKind, expected, seen)) return;
    // It's this far to the side of where odometry says it should be
    double sideways = clampTo(dist * std::tan(rad(seen - expected)), -FP_AIM_MAX_SHIFT, FP_AIM_MAX_SHIFT);
    fpAimShift.x += FP_AIM_SMOOTH * (r.x * sideways - fpAimShift.x);
    fpAimShift.y += FP_AIM_SMOOTH * (r.y * sideways - fpAimShift.y);
}

// =====================================================================
//   SERVICE: runs every 10 ms while the robot drives or waits. Lift, claw, flipper, intake, tip guard, timer.
// =====================================================================
void service() {
    uint32_t now = pros::millis();

    // 15 s timer: lift PID off and limp just before the end
    if (!fpTimeUp && fpElapsed() >= FP_AUTON_MS - FP_COAST_BEFORE_END_MS) {
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
            DR4B.move_voltage(settled ? -FP_GROUND_HOLD_MV : -FP_GROUND_PUSH_MV);
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

// Waits while everything keeps running. false = the 15 s are up.
bool waitMs(int ms) {
    uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<uint32_t>(ms)) {
        service();
        if (fpTimeUp) return false;
        pros::delay(10);
    }
    return true;
}

// =====================================================================
//   CURVES
// =====================================================================
constexpr double FP_SAMPLE_STEP = 0.5;   // the curve is stored as a point every half inch

struct Sample {
    double x, y;        // field
    double heading;     // direction of travel along the curve (field, degrees)
    double curvature;   // 1/inches, + = curving right (as seen driving along it)
    double speed;       // planned speed here, in/s
};
struct Curve {
    std::vector<Sample> points;
    std::vector<int> nodeIndex;   // which point each node landed on
    bool reverse = false;
    double length() const { return points.empty() ? 0.0 : (points.size() - 1) * FP_SAMPLE_STEP; }
};

// Builds the curve through the nodes (route coordinates): a smooth piece between each pair, leaving and
// arriving along the way the robot points. Then plans the speed along it.
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

// =====================================================================
//   FOLLOWER
// =====================================================================
enum class MoveEnd { arrived, stalled, timedOut, cut, timeUp };

struct MoveOptions {
    // Called every 10 ms with the inches driven and the inches left on this curve. Return false to end the move now.
    std::function<bool(double done, double left)> tick;
    // The robot holds this many inches before the end until gateReady() is true (-1 = no gate)
    double gateBeforeEnd = -1.0;
    std::function<bool()> gateReady;
    // Optional: seconds until gateReady() should be true. With it, the robot creeps toward the gate so it gets
    // there about then, instead of driving up to it and stopping.
    std::function<double()> gateEta;
    bool brakeAtEnd = true;     // false = it's switching direction or carrying on, don't brake
    int stallMs = FP_STALL_MS;  // told to drive but not moving for this long = it's against something, the move ends
    int timeoutMs = 0;          // 0 = worked out from the curve
};

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
    const double slowAbove = getStackHeight(GOAL_ALLIANCE, FP_PIN3_STACK) + FP_MOVING_SLOW_MARGIN;
    bool headedUp = (fpLiftMode == LiftMode::pid && fpLiftGoal > slowAbove) ||
                    ((fpLiftPending || fpLiftMode == LiftMode::hop) && fpHopThen > slowAbove);
    if (headedUp || lift > slowAbove) {
        cap = std::min(cap, FP_MOVING_SPEED);
        accelNow = std::min(accelNow, FP_MOVING_ACCEL);
    }
    if (fpTipping) cap = std::min(cap, FP_TIP_SPEED);
    return cap;
}

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
            if (room <= 0.4) {
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

// =====================================================================
//   ROUTE HELPERS
// =====================================================================
// How close the back corners are to the auton line (inches on our side; the line is x + y = 0 in route coordinates)
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

// How far past the bottom wall (route coordinates) odometry says the lowest front corner is (negative = not there yet)
double frontCornerPastWall() {
    Node now = hereNode();
    Vec f = forwardOf(now.heading), r = rightOf(now.heading);
    double lowest = 1e9;
    for (double side : {-FP_HALF_WIDTH, FP_HALF_WIDTH}) {
        lowest = std::min(lowest, now.y + f.y * FP_CENTER_TO_FRONT + r.y * side);
    }
    return -FP_FIELD_HALF - lowest;
}

// How far the middle of the claw is from a spot (route coordinates), with the camera's nudge
double clawGap(double routeX, double routeY) {
    Vec spot = toField({routeX, routeY});
    Vec f = forwardOf(getInertialHeading());
    double cx = x_pos + f.x * FP_CLAW_REACH, cy = y_pos + f.y * FP_CLAW_REACH;
    return std::hypot(spot.x + fpAimShift.x - cx, spot.y + fpAimShift.y - cy);
}

// Drives a curve through a pin (node pinNode of the curve), picks it up on the way through without stopping,
// then carries on along the curve into the goal with the lift going up, and lets go.
// false = the 15 s ran out.
bool pickUpAndScore(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY, double scoreHeight) {
    Curve curve = makeCurve(nodes, false, FP_MAX_SPEED, 0.0);
    double grabAt = curve.nodeIndex[pinNode] * FP_SAMPLE_STEP;   // inches along the curve where the pin is in the claw
    bool grabbed = false;
    aimAt(AimKind::pin, pinX, pinY);

    MoveOptions options;
    options.gateBeforeEnd = FP_SCORE_GATE_IN;
    options.gateReady = []() { return liftReady() && !fpTipping; };
    options.tick = [&](double done, double left) {
        (void)left;
        if (!grabbed) {
            if (grabAt - done <= FP_CLAW_OPEN_DIST) clawOpen();
            if (done >= grabAt || (grabAt - done < 4.0 && clawGap(pinX, pinY) <= FP_GRAB_TOLERANCE)) {
                // The pin is in the claw: close on it, full speed for FP_CLAW_FAST_MS then the hold. The lift hops
                // off the ground like driver control's pin pickup, then carries on up to the scoring height.
                grabbed = true;
                note("claw closes on a pin");
                clawClose();
                liftHopThen(scoreHeight);
                aimStop();
                Vec gone = toField({mclPinX, mclPinY});
                fieldMclPinPickedUp(gone.x, gone.y);   // MCL stops expecting a pin there
                if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, FP_GOAL_X, FP_GOAL_Y);
            }
        }
        return true;
    };
    MoveEnd end = follow(curve, options);
    aimStop();
    if (end == MoveEnd::timeUp) return false;

    // At the goal: let go
    note("at the goal: let go");
    clawOpen();
    return waitMs(FP_SCORE_RELEASE_MS);
}

// Backs away from the goal and swings the nose to a new heading (one backwards curve). The lift drops and the
// claw motor turns off on the way. false = the 15 s ran out.
bool backOutTo(double straight, const Node& turnSpot) {
    // Straight back the way it's really pointing first (the goal is still in the claw opening), then the swing
    Node start = hereNode();
    Curve curve = makeCurve({start, along(start, -straight), turnSpot}, true, FP_MAX_SPEED, FP_CUSP_SPEED);
    MoveOptions options;
    options.brakeAtEnd = false;
    options.tick = [&](double done, double left) {
        (void)left;
        if (done >= FP_LOWER_AFTER_BACK_IN) liftDown();
        if (done >= FP_CLAW_OFF_AFTER_BACK_IN) clawOff();
        return true;
    };
    MoveEnd end2 = follow(curve, options);
    note("backed out, nose swung");
    liftDown();
    clawOff();
    return end2 != MoveEnd::timeUp;
}

// =====================================================================
//   THE ROUTE
// =====================================================================
void runFourPin(bool turned, bool mirrored) {
    fpTurned = turned;
    fpMirrored = mirrored;
    fpStartMs = pros::millis();
    fpTimeUp = false;
    fpLiftMode = LiftMode::idle;
    fpLiftPending = false;
    fpClawMode = ClawMode::off;
    fpFlipMode = FlipMode::off;
    fpIntakeOn = false;
    fpIntakeOffAtMs = 0;
    fpTipping = false;
    fpTipTicks = 0;
    fpAimKind = AimKind::none;
    fpAimShift = {0, 0};

    // Where the robot starts: back on the wall at the toggle
    Vec startSpot = toField({FP_START_X, FP_START_Y});
    setPose(startSpot.x, startSpot.y, headingToField(FP_START_HEADING));
    left_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_chassis.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    flipper.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);   // off = limp, never held against its stop
    visionStart();

    // The poses the route is built from (red bottom drawing)
    const Node preloadPose = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PRELOAD_HEADING, FP_CLAW_REACH + FP_PRELOAD_EXTRA);
    Vec pin2Back = forwardOf(FP_PIN2_HEADING);
    const Node pin2Pose = node(FP_PIN2_TIP_X + pin2Back.x * (FP_INTAKE_REACH - FP_PIN2_EXTRA),
                               FP_PIN2_TIP_Y + pin2Back.y * (FP_INTAKE_REACH - FP_PIN2_EXTRA), FP_PIN2_HEADING);
    const Node pin3Pose = clawOn(FP_PIN3_X, FP_PIN3_Y, FP_PIN3_HEADING, FP_CLAW_REACH + FP_PIN3_GRAB_EXTRA);
    const Node pin3Score = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PIN3_SCORE_HEADING, FP_CLAW_REACH + FP_PIN3_SCORE_EXTRA);
    const Node turn4 = node(FP_TURN4_X, FP_TURN4_Y, FP_TURN4_HEADING);
    const Node pin4Pose = clawOn(FP_PIN4_X, FP_PIN4_Y, FP_PIN4_HEADING, FP_CLAW_REACH + FP_PIN4_GRAB_EXTRA);
    const Node pin4Score = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_PIN4_SCORE_HEADING, FP_CLAW_REACH + FP_PIN4_SCORE_EXTRA);
    const Node turn5 = node(FP_TURN5_X, FP_TURN5_Y, FP_TURN5_HEADING);
    const Node cup5Pose = clawOn(FP_CUP5_X, FP_CUP5_Y, FP_CUP5_HEADING, FP_CLAW_REACH);
    const Node cup5Score = clawOn(FP_GOAL_X, FP_GOAL_Y, FP_CUP5_SCORE_HEADING, FP_CLAW_REACH + FP_CUP5_SCORE_EXTRA);

    // ---------------- Start: lift up and straight back down (the toggles drop onto the field toggle) ----------------
    fpLiftMode = LiftMode::direct;
    dr4bManualMode = true;
    dr4bSettling = false;
    dr4bTargetForced = false;
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    {
        uint32_t t0 = pros::millis();
        while (getDR4BPosition() < FP_TOGGLE_LIFT_DEG && pros::millis() - t0 < static_cast<uint32_t>(FP_TOGGLE_UP_MAX_MS)) {
            DR4B.move_voltage(12000);
            service();
            pros::delay(10);
        }
        note("lift up for the toggles");
        liftDown();
        t0 = pros::millis();
        while (getDR4BPosition() > FP_TOGGLE_GO_DEG && pros::millis() - t0 < static_cast<uint32_t>(FP_TOGGLE_DOWN_MAX_MS)) {
            service();
            pros::delay(10);
        }
    }

    // ---------------- 1. Off the toggle and curve into the goal, claw open, lift down. Flip the preload on. ----------------
    {
        Node start = hereNode();
        start.heading = FP_START_HEADING;   // straight off the wall
        Curve curve = makeCurve({start, along(start, FP_START_STRAIGHT, 0, 6.0), along(preloadPose, -FP_GOAL_LEAD_IN, 8.0, 0), preloadPose},
                                false, FP_MAX_SPEED, 0.0);
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, FP_GOAL_X, FP_GOAL_Y);
        MoveOptions options;
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
        if (!waitMs(FP_PRELOAD_FLIP_WAIT_MS)) return;
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
        MoveOptions options;
        options.tick = [&](double done, double left) {
            if (done >= 1.0) clawOff();                                    // claw motor off: it closes by itself
            if (done >= 3.0) liftDown();                                   // lift held down for the intake
            if (done >= FP_FLIPPER_DOWN_AFTER_IN) flipperDown();           // then the flipper back to the intake position
            if (left <= FP_INTAKE_ON_DIST) fpIntakeOn = true;
            if (left <= 8.0 && backCornerLineGap() < FP_LINE_MARGIN) return false;   // never over the auton line
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at the cluster pin");
        flipperDown();
        fpIntakeOn = true;
        if (!waitMs(FP_INTAKE_DWELL_MS)) return;
        fpIntakeOffAtMs = pros::millis() + FP_INTAKE_KEEP_MS;
    }

    // ---------------- 3. Around the small red pin, through pin 3, left into the goal ----------------
    {
        Node start = hereNode();
        std::vector<Node> nodes = {
            start,
            along(pin2Pose, FP_PIN2_STRAIGHT_OUT, 0, 7.0),
            along(pin3Pose, -FP_PIN_LEAD_IN, 8.0, 0),
            node(pin3Pose.x, pin3Pose.y, pin3Pose.heading, 0, 0, FP_GRAB_SPEED),
            along(pin3Pose, FP_PIN_LEAD_OUT, 0, 7.0),
            along(pin3Score, -FP_GOAL_LEAD_IN, 7.0, 0),
            pin3Score,
        };
        if (!pickUpAndScore(nodes, 3, FP_PIN3_X, FP_PIN3_Y, -24.0, -24.0, getStackHeight(GOAL_ALLIANCE, FP_PIN3_STACK))) return;
    }

    // ---------------- 4. Back out, nose at pin 4, through pin 4, left into the goal ----------------
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
        if (!pickUpAndScore(nodes, 2, FP_PIN4_X, FP_PIN4_Y, -48.0, -48.0, getStackHeight(GOAL_ALLIANCE, FP_PIN4_STACK))) return;
    }

    // ---------------- 5. Back out, nose at the wall cup, into the cup and the wall with the claw spinning ----------------
    if (!backOutTo(FP_PIN4_BACK_OUT, turn5)) return;
    {
        Node end = cup5Pose;
        end.speed = FP_WALL_SPEED;
        Curve curve = makeCurve({hereNode(), along(cup5Pose, -5.0), end}, false, FP_MAX_SPEED, FP_WALL_SPEED);
        MoveOptions options;
        options.brakeAtEnd = false;
        options.stallMs = 100;   // against the wall = there
        options.tick = [&](double done, double left) {
            (void)done; (void)left;
            clawClose();   // spinning closed the whole way in (not opened for this one)
            liftDown();
#ifdef RW_SIM_PROS
            // The simulator's wall only stops the middle of the robot, so a front corner would drive through it.
            // This ends the move where a real wall would stop the robot.
            if (frontCornerPastWall() >= 0.0) return false;
#endif
            return frontCornerPastWall() < FP_WALL_OVERDRIVE;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;   // arrived or stalled on the wall: both are fine
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
            if (fpTimeUp) { stopChassis(pros::MotorBrake::brake); return; }
            pros::delay(10);
        }
    }

    // ---------------- Back off the wall with the lift going straight up to score, flip the pin into the cup on the way ----------------
    bool flipStarted = false;
    const bool liftGoing = true;   // the lift heads for the scoring height right away (no waiting for the flip)
    auto cupSequence = [&]() {
        // Flip once the lift has climbed a little (the flip keeps going while the lift carries on up)
        if (!flipStarted && fpLiftMode == LiftMode::pid && getDR4BPosition() >= FP_CUP_FLIP_LIFT_DEG) {
            flipStarted = true;
            note("flip the pin into the cup");
            flipperUp();
        }
    };
    {
        Node start = hereNode();
        Curve curve = makeCurve({start, along(start, -FP_WALL_BACK_OUT), along(cup5Score, -FP_CUP5_SCORE_RUN)}, true, FP_MAX_SPEED, FP_CUSP_SPEED);
        liftTo(getStackHeight(GOAL_ALLIANCE, FP_CUP5_STACK));
        MoveOptions options;
        options.brakeAtEnd = false;
        options.tick = [&](double done, double left) {
            (void)done; (void)left;
            cupSequence();
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("backed off the wall");
    }

    // ---------------- Into the goal, flipper back just before, let go, lower. Lift limp before 15 s. ----------------
    {
        Curve curve = makeCurve({hereNode(), along(cup5Score, -FP_GOAL_LEAD_IN), cup5Score}, false, FP_MAX_SPEED, 0.0);
        if (FP_USE_GOAL_TAG) aimAt(AimKind::goalTag, FP_GOAL_X, FP_GOAL_Y);
        MoveOptions options;
        options.gateBeforeEnd = FP_SCORE_GATE_IN;
        options.gateReady = [&]() { return liftGoing && liftReady() && !fpTipping; };
        options.gateEta = [&]() {
            // The rest of the climb to the scoring height
            double climb = getStackHeight(GOAL_ALLIANCE, FP_CUP5_STACK) - FP_LIFT_READY_TOLERANCE - std::max(getDR4BPosition(), 0.0);
            return std::max(climb, 0.0) / FP_LIFT_UP_DPS;
        };
        options.tick = [&](double done, double left) {
            (void)done;
            cupSequence();
            if (liftGoing && liftReady() && left <= FP_FLIPPER_BACK_BEFORE_IN) flipperDown();   // out of the way right before scoring
            return true;
        };
        if (follow(curve, options) == MoveEnd::timeUp) return;
        note("at the goal: let go (last)");
        aimStop();
        flipperDown();
        clawOpen();
        if (!waitMs(FP_SCORE_RELEASE_MS)) return;
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
    stopChassis(pros::MotorBrake::brake);
    intake.move_voltage(0);
    flipper.move_voltage(0);
    if (fpClawMode == ClawMode::closing || fpClawMode == ClawMode::hold) claw.move_voltage(FP_CLAW_HOLD_MV);
    else claw.move_voltage(0);
    if (fpTimeUp) note("15 s up: lift limp");
}

// The four starts (see include/fourpin.hpp)
void fourPinRedBottom() { fourPin(false, false); }
void fourPinBlueTop() { fourPin(true, false); }
void fourPinRedLeft() { fourPin(false, true); }
void fourPinBlueRight() { fourPin(true, true); }
