#pragma once
// =====================================================================
//   AUTON CORE: everything the autons share (src/auton-core.cpp)
// =====================================================================
// The curve follower, the lift / claw / flipper / intake service, the AI Vision aiming (pin camera + goal camera),
// the tip guard and the timer. The 4-pin qual auton (src/fourpin.cpp) and the skills run (src/skills.cpp) are just
// routes written with these. The route files hold their own spots, headings and waits; the driving and mechanism
// tuning lives here, so a change to how the robot drives or lifts applies to both.
//
// How a route is written: a move is a smooth curve through a few "nodes" (a spot + the way the robot points there),
// built with makeCurve() and driven with follow(). The follower plans the speed along the curve (slower in tight
// curves, slowing down in time for the end), and service() runs every 10 ms underneath it: lift, claw, flipper,
// intake, tip guard, timer and the camera aiming.
//
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// Routes can be written in their own "drawing" frame and turned / flipped onto the field (setRouteFrame).
// =====================================================================
#include "robot.hpp"

#include <functional>
#include <vector>

namespace ac {

// =====================================================================
//   ROBOT SIZE (measured, from the tracking center = the middle of the drive wheels; wheelbase 10.0, track 12.6)
// =====================================================================
inline constexpr double FP_FIELD_HALF = 70.25;
// The chassis is 15.5 long and 15.0 wide (16.5 at the gears on the claw end); the robot is 17 long at the start and 21
// with the claw out. The body's corners are what hit walls and goals; the claw is narrower (7.25" open) and sticks out
// further on the center line, so it's counted separately (FP_CLAW_TIP_FORWARD).
inline constexpr double FP_CENTER_TO_FRONT = 8.5;    // tracking center to the front of the BODY (the chassis corners)
inline constexpr double FP_CENTER_TO_BACK = 8.5;     // tracking center to the back (the intake rollers are 7.4 back)
inline constexpr double FP_HALF_WIDTH = 8.25;        // half of 16.5
inline constexpr double FP_CLAW_TIP_FORWARD = 10.5;  // how far the claw sticks out on the center line (flex wheel axles at 8.5 + the wheels)
inline constexpr double FP_CLAW_HALF_WIDTH = 3.6;    // half the claw's open width (7.25" between the flex wheel centers)
// Tracking center to the middle of a pin / cup when the claw closes on it (the inside of the claw: the flex wheel
// centers are at 8.5, their closed gap 3.55", open 7.25")
inline constexpr double FP_CLAW_REACH = 7.0;
// Tracking center to the TIP of the cluster pin when the intake has hold of it. The intake rollers are 7.4 back and
// the back edge 8.5: the pin's tip ends up about at the back edge. PLACEHOLDER: check on the robot.
inline constexpr double FP_INTAKE_REACH = 8.5;

// =====================================================================
//   SHARED ROUTE SHAPE
// =====================================================================
inline constexpr double FP_GOAL_LEAD_IN = 6.0;       // inches of straight run into a goal (the goal has to slide into the claw opening)
inline constexpr double FP_PIN_LEAD_IN = 5.0;        // inches of straight run into a pin
inline constexpr double FP_PIN_LEAD_OUT = 3.0;       // inches straight on after grabbing before it curves away
// Slowdowns (never a stop): the robot drives the last stretch before a spot at a slow speed instead.
inline constexpr double FP_PICKUP_SLOW_SPEED = 15.0;   // in/s, into pins / the cup
inline constexpr double FP_SCORE_SLOW_SPEED = 12.0;    // in/s, into a goal

// =====================================================================
//   DRIVING
// =====================================================================
inline constexpr double FP_MAX_SPEED = 56.0;        // top speed, in/s (the drive tops out around 58-60)
inline constexpr double FP_WHEEL_MAX = 58.0;        // fastest one side is asked to go (the outside of a curve)
inline constexpr double FP_ACCEL = 170.0;           // speeding up, in/s per second
inline constexpr double FP_DECEL = 150.0;           // slowing down for a stop or a tight curve
inline constexpr double FP_LAT_ACCEL = 160.0;       // how hard it corners (in/s^2): lower = slower in curves
inline constexpr double FP_MAX_TURN_DPS = 240.0;    // fastest it's allowed to spin in a curve (degrees/s)
inline constexpr double FP_MIN_SPEED = 5.0;         // never slower than this while it's still driving
inline constexpr double FP_CUSP_SPEED = 8.0;        // speed left at a forwards <-> backwards switch (it doesn't settle there)
inline constexpr double FP_GRAB_SPEED = 30.0;       // speed through a pin while the claw closes on it (in/s)
inline constexpr double FP_INTAKE_SPEED = 26.0;     // speed for the last inches backing into the cluster pin
inline constexpr double FP_WALL_SPEED = 26.0;       // speed into the wall cup / the matchloader
// Steering: the same as chasing a point this far ahead on the curve (inches). Lower = follows tighter, twitchier.
inline constexpr double FP_STEER_DIST = 6.0;
inline constexpr double FP_STEER_PREVIEW_S = 0.08;  // looks this far ahead in time for the curve's bend (covers the motors' lag)
inline constexpr double FP_HEADING_GAIN = 6.0;      // pointing correction at low speed (degrees/s of turn per degree off)
inline constexpr double FP_FF_ACCEL_MAX = 300.0;    // most wheel acceleration the feedforward asks for (in/s^2)
inline constexpr double FP_SPEED_FEEDBACK = 0.10;   // extra volts per in/s a side is off its speed
// Coming to a stop: on top of FP_DECEL, the speed is never more than this many in/s per inch left to go, so the
// last inch is gentle and it doesn't slide past the spot. Lower = softer and a bit slower.
inline constexpr double FP_STOP_GAIN = 8.0;
inline constexpr double FP_END_TOLERANCE = 0.6;     // inches from the end of a curve that counts as there
// With the lift up it drives slower so it doesn't tip. Full speed below LO, FP_TALL_SPEED at HI and above.
inline constexpr double FP_TALL_LIFT_LO = 150.0;    // lift degrees
inline constexpr double FP_TALL_LIFT_HI = 450.0;
inline constexpr double FP_TALL_SPEED = 30.0;       // in/s
inline constexpr double FP_TALL_ACCEL = 90.0;       // speeding up / slowing down with the lift at HI
// While the lift is up or headed up to score (and on the way back down after letting go), the drive is capped at
// this too, so the lift can't tip it. Lower = safer, slower. Only for heights more than FP_MOVING_SLOW_MARGIN above
// the 2nd alliance height (main.cpp's ALLIANCE_HEIGHTS), so the lower stack spots aren't slowed.
inline constexpr double FP_MOVING_SLOW_MARGIN = 20.0;  // lift degrees above the 2nd alliance height
inline constexpr double FP_MOVING_SPEED = 28.0;        // in/s (was 20: the loader trips spend most of their time under this cap)
inline constexpr double FP_MOVING_ACCEL = 90.0;        // speeding up, in/s per second
// A move ends if the robot is being told to drive but hasn't moved for this long (it's against something)
inline constexpr int FP_STALL_MS = 250;
// Turning in place (spinTo): proportional spin, capped
inline constexpr double FP_SPIN_MAX_DPS = 300.0;
inline constexpr double FP_SPIN_GAIN = 5.0;         // degrees/s per degree off
inline constexpr double FP_SPIN_DONE_DEG = 2.0;     // counts as there when this close...
inline constexpr int FP_SPIN_DONE_MS = 60;          // ...for this long

// =====================================================================
//   LIFT
// =====================================================================
inline constexpr double FP_LIFT_READY_TOLERANCE = 15.0; // lift counts as up when within this of its height
// Pickup hop: when the claw closes on a pin / cup off the field (not the intake), the lift hops up a little, the
// same as driver control's pin pickup (PIN_LIFT_HEIGHT / PIN_LIFT_MIN_RISE / PIN_LIFT_VOLTAGE in robot.hpp).
// No stall check: the route already knows when the claw closes, so it just waits this long for the claw to shut.
inline constexpr int FP_PICKUP_LIFT_DELAY_MS = 100;
// The lift gate: this many inches before a goal the lift should be up (so the pin can't hit the stack). With gate
// holding ON (the 4-pin) the robot stops there and waits for the lift; with it OFF (skills: setGateHold(false)) it
// never stops, it just keeps rolling in at the creep speed while the lift finishes.
inline constexpr double FP_SCORE_GATE_IN = 7.0;
inline constexpr int FP_GATE_MAX_MS = 1500;             // holding: stops waiting after this long
// Gate creep (gateEta): instead of stopping at the gate to wait for the lift, it works out how long the lift still
// needs and drives just fast enough (never over FP_MOVING_SPEED) to reach the gate as the lift gets there.
inline constexpr double FP_LIFT_UP_DPS = 550.0;         // about how fast the lift climbs (deg/s); lower = drives in slower
inline constexpr double FP_GATE_CREEP_MIN = 6.0;        // never slower than this while creeping in (in/s)
inline constexpr double FP_GATE_ETA_BUFFER_S = 0.1;     // aims to reach the gate this long after the lift is up (so it never brakes there)
inline constexpr double FP_LOWER_AFTER_BACK_IN = 2.0;   // after letting go, the lift starts down once it has backed this far
// Lowering: full power down, then a gentle push for the last bit, then a light hold on the bottom
inline constexpr double FP_GROUND_FAST_ABOVE = 30.0;    // full power down while above this
inline constexpr int FP_GROUND_PUSH_MV = 4000;
inline constexpr int FP_GROUND_HOLD_MV = 1200;
inline constexpr int FP_GROUND_PUSH_MAX_MS = 400;
// Picking up (auton only): while the claw is open and heading into a pin, or spinning at the matchloader, the lift
// keeps pushing down with this instead of the light hold, so the claw stays flat on the floor / the loader's pin.
inline constexpr int FP_PICKUP_PRESS_MV = 3500;

// =====================================================================
//   CLAW, FLIPPER, INTAKE
// =====================================================================
inline constexpr int FP_CLAW_RPM = 200;                 // claw full speed (closing and opening)
inline constexpr int FP_CLAW_FAST_MS = 1000;            // closes at full speed this long, then drops to the hold
inline constexpr int FP_CLAW_HOLD_MV = 2000;            // hold squeeze (same as driver control)
// Opening: full speed (move_velocity(-FP_CLAW_RPM)) the whole time it's open, the same as holding R2 in driver
// control. Motor off = it closes by itself.
inline constexpr double FP_CLAW_OPEN_DIST = 18.0;       // claw starts opening this many inches before a pin
inline constexpr double FP_GRAB_TOLERANCE = 3.0;        // claw closes when the pin is this close to the middle of the claw
inline constexpr double FP_CLAW_OFF_AFTER_BACK_IN = 5.0;   // claw motor off once it has backed this far from the goal
inline constexpr int FP_FLIP_RPM = 110;                 // flipper up (flipping), same as FLIPPER_RPM in main.cpp
inline constexpr int FP_FLIP_DOWN_RPM = 50;             // flipper back down
inline constexpr double FP_FLIPPER_STALL_DEG = 5.0;     // going down: moved less than this...
inline constexpr int FP_FLIPPER_STALL_MS = 500;         // ...for this long = it's home, motor off
inline constexpr int FP_FLIPPER_DOWN_MAX_MS = 2000;
inline constexpr int FP_INTAKE_MV = 12000;

// =====================================================================
//   TIP GUARD
// =====================================================================
// If the robot tips with the lift up, the lift drops and the robot slows right down. When it's level again the
// lift goes back to the height it was heading for, and the route carries on.
inline constexpr bool FP_TIP_GUARD = true;
inline constexpr double FP_TIP_ROLL_LIMIT = 13.0;       // degrees (same axis and limit as driver control's tip protection)
inline constexpr double FP_TIP_ROLL_CLEAR = 8.0;        // level again below this
inline constexpr int FP_TIP_CONFIRM_TICKS = 3;          // 10 ms each
inline constexpr double FP_TIP_MIN_LIFT = 60.0;         // only counts with the lift above this
inline constexpr double FP_TIP_SPEED = 12.0;            // in/s while it recovers

// =====================================================================
//   TIMER
// =====================================================================
inline constexpr int FP_COAST_BEFORE_END_MS = 150;      // lift PID off and motors limp this long before the end

// =====================================================================
//   DISTANCE RESETS
// =====================================================================
// The only time the robot stands still is at a goal letting go, so that's the only time it does a one-shot distance
// sensor reset (fieldAwareReset in src/field-mcl.cpp: same field-aware checks as the MCL that runs all the time while
// driving). It never stops anywhere just to reset.
inline constexpr bool FP_RESET_AT_GOALS = true;
inline constexpr double FP_RESET_MAX_JUMP = 4.0;        // inches: a reset that would move X or Y more than this is ignored

// =====================================================================
//   AI VISION
// =====================================================================
// Two cameras (see the photos of the robot):
//   CLAW camera  (port AI_VISION_PORT, 5): in the middle of the claw, behind the red gears, looking out through the claw
//                opening. Sees pins / cups (the AI model). The gears block the bottom third of its picture. Its job is
//                the last part of a pickup: a pin it sees is lined up with the middle of the claw.
//   FIELD camera (port GOAL_VISION_PORT, 21): low at the front-LEFT corner of the chassis, turned inward (pointing to
//                the right of straight ahead, toward the claw). Sees pins / cups AND the AprilTags on the goals. It finds pins from
//                further out, before the claw camera has them, and it lines up every goal approach on the goal's tag.
// On the way into a pin the claw camera is asked first; if it has nothing, the field camera is used. Goals: field only.
inline constexpr bool FP_USE_PIN_VISION = true;
inline constexpr int FP_PIN_CAM_PORT = AI_VISION_PORT;   // the claw camera (include/robot.hpp)
inline constexpr int FP_PIN_CLASS_ID = -1;              // -1 = any object the Override model finds, or one class id
inline constexpr double FP_PIN_CAM_FORWARD = 4.9;       // inches the claw camera's lens sits in front of the tracking center (7.6" up)
inline constexpr double FP_PIN_CAM_YAW = 0.0;           // degrees it points right of straight ahead (it's in the middle: 0)
inline constexpr double FP_PIN_CAM_TILT_DEG = 15.0;     // degrees it points DOWN. A tilted camera sees sideways angles a little
                                                        // squashed; the bearing math undoes that
inline constexpr bool FP_PIN_CAM_UPSIDE_DOWN = false;   // true if it's mounted upside down (left and right swap in its picture)
inline constexpr int FP_CLAW_CAM_BLOCKED_FROM_PX = 160; // the red gears fill the picture from this row down (240 rows): anything whose
                                                        // TOP is below this is a gear, not a pin. A pin cut off by the gears still counts
                                                        // (only its sideways position is used, and that's still right)
inline constexpr bool FP_USE_GOAL_TAG = true;           // the field camera lines goal approaches up on the tags
inline constexpr bool FP_FIELD_CAM_SEES_PINS = true;    // ...and finds pins / cups from further out than the claw camera
inline constexpr int FP_GOAL_CAM_PORT = GOAL_VISION_PORT;   // the field camera (include/robot.hpp)
// Where the field camera's lens is, from the tracking center (the point odometry tracks, between the drive wheels).
// Measured: 6.5" forward, 5.5" LEFT, 3.6" up.
inline constexpr double FP_GOAL_CAM_FORWARD = 6.5;      // inches forward
inline constexpr double FP_GOAL_CAM_RIGHT = -5.5;       // inches to the right (- = left: it's on the left side)
// How far it's turned inward: degrees it points RIGHT of straight ahead. It sits on the left and is turned in toward
// the claw, so this is positive. Measured 23.58. To check it: put a goal with its tag dead ahead of the robot's
// center line, 36" from the tracking center (tape measure down the middle of the robot). The brain screen's FIELD CAM
// line shows the tag's "raw" bearing (degrees right of the camera's own straight ahead). Then
//   FP_GOAL_CAM_YAW = (the bearing the tag SHOULD have from the camera) - raw
// where the should-have bearing = atan2(-FP_GOAL_CAM_RIGHT, 36 - FP_GOAL_CAM_FORWARD) in degrees = about +10.6 with the
// numbers above (the tag is to the right of a camera that sits on the left). Example: raw reads -13, so yaw = 10.6 + 13 = 23.6.
inline constexpr double FP_GOAL_CAM_YAW = 23.58;
inline constexpr bool FP_GOAL_CAM_UPSIDE_DOWN = false;
inline constexpr double FP_GOAL_TAG_SIZE_IN = GOAL_TAG_SIZE_IN;   // include/robot.hpp: the tag's black square, edge to edge
inline constexpr double FP_GOAL_TAG_TO_CENTER = 4.5;    // PLACEHOLDER: inches from the tag (on the goal's face) in to the goal's center
inline constexpr double FP_GOAL_TAG_MIN_PX = 12.0;      // a tag smaller than this is too far / too blurry to trust its distance
inline constexpr double FP_GOAL_AIM_MAX_DIST = 48.0;    // starts lining up on the tag this far out (tags read from further than pins)
inline constexpr double FP_GOAL_AIM_MIN_DIST = 8.0;     // stops updating this close to the goal (camera to tag: the tag leaves the picture)
inline constexpr double FP_GOAL_MAX_SHIFT = 6.0;        // most the approach is moved (inches, any direction): bigger = odometry is
                                                        // really lost, and it's safer to trust the route than one tag
inline constexpr double FP_GOAL_TAG_WINDOW_IN = 9.0;    // only trusts a tag that puts the goal within this of where it should be
                                                        // (inches, not degrees: close up, a small misplacement is a big angle)
inline constexpr double FP_GOAL_AIM_SMOOTH = 0.5;       // 0-1: the tag gives a position outright, so the nudge follows it faster
                                                        // than a pin's (some approaches only get two or three looks at the tag)
// Both cameras
inline constexpr double FP_CAM_WIDTH_PX = 320.0;
inline constexpr double FP_CAM_HEIGHT_PX = 240.0;
inline constexpr double FP_CAM_FOV_DEG = 74.0;          // side to side
inline constexpr double FP_AIM_WINDOW_DEG = 14.0;       // only trusts a pin this close to where it should be (degrees, from the camera)
inline constexpr double FP_AIM_MAX_SHIFT = 3.0;         // most the curve is nudged sideways for a pin (inches)
inline constexpr double FP_AIM_MAX_DIST = 30.0;         // pins, claw camera: starts aiming this far out
inline constexpr double FP_FIELD_PIN_MAX_DIST = 48.0;   // pins, field camera: starts aiming this far out
inline constexpr double FP_AIM_MIN_DIST = 5.0;          // pins: stops updating this close (the pin is under the camera's view)
inline constexpr double FP_AIM_SMOOTH = 0.25;           // 0-1: how fast the nudge follows the camera (lower = smoother)
inline constexpr int FP_AIM_PERIOD_MS = 30;

// =====================================================================
//   SMALL MATH
// =====================================================================
inline constexpr double FP_PI = 3.14159265358979323846;
double rad(double deg);
double wrap180(double deg);
double clampTo(double value, double low, double high);
struct Vec { double x, y; };
Vec forwardOf(double headingDeg);   // unit vector the heading points along
Vec rightOf(double headingDeg);     // unit vector to its right

// =====================================================================
//   ROUTE FRAME: a route drawn for one start, turned / flipped onto the field
// =====================================================================
// turned = 180 degrees (the other alliance), mirrored = flipped across the diagonal the field pins sit on
void setRouteFrame(bool turned, bool mirrored);
Vec toField(Vec p);
double headingToField(double heading);
Vec toRoute(Vec p);
double headingToRoute(double heading);

// A spot on the route and the way the robot points there. tangentIn / tangentOut shape the curve on each side
// of it (inches; 0 = automatic). speed = top speed near this spot (0 = no extra limit).
struct Node {
    double x, y, heading;
    double tangentIn = 0, tangentOut = 0;
    double speed = 0;
};
Node node(double x, double y, double heading, double tangentIn = 0, double tangentOut = 0, double speed = 0);
Node along(const Node& from, double inches, double tangentIn = 0, double tangentOut = 0, double speed = 0);   // moved along its own heading
Node clawOn(double x, double y, double heading, double reach);   // robot pose with the claw (reach inches ahead of center) on a spot
Node hereNode();                                                  // where the robot is now, in route coordinates

// =====================================================================
//   CLOCK + LOG
// =====================================================================
// Resets everything (lift, claw, flipper, intake, aiming, timer) at the start of a route. limitMs = the auton
// length: FP_COAST_BEFORE_END_MS before it the lift goes limp and every wait / move returns.
void coreStart(const char* routeName, int limitMs);
int elapsed();        // ms since coreStart
bool timeUp();        // the limit has been reached (the lift is already limp)
void note(const char* what);   // one line on the terminal with the time, pose and lift
// Leaves everything safe however a route ended: drive stopped, intake / flipper off, claw off or on its hold
void finishSafe();

// =====================================================================
//   LIFT
// =====================================================================
void liftTo(double height);           // to a height with the lift PID
void liftHopThen(double thenHeight);  // pickup hop FP_PICKUP_LIFT_DELAY_MS from now, then on to thenHeight (-1 = hold there)
void liftDown();                      // all the way down as fast as it goes, then a light hold on the bottom
void liftPress(bool on);              // while down: push with FP_PICKUP_PRESS_MV instead of the light hold (picking up)
void liftCoast();                     // PID off and the motor limp
void liftRaiseDirect(double height, int maxMs);   // full power up until it reads height (or maxMs): the start toggles
bool liftReady();                     // at the height it was sent to (pid mode)
bool liftIsDown();
bool liftClimbingPast(double deg);    // pid mode and already above deg
bool tipping();
double liftGoal();

// =====================================================================
//   CLAW, FLIPPER, INTAKE
// =====================================================================
void clawOpen();
void clawClose();
void clawCloseAgain();   // restarts the full-speed squeeze
void clawOff();
bool clawHolding();      // closing or on its hold
void flipperUp();        // flips (and the intake runs while it's up)
void flipperDown();
void intakeOn();
void intakeOffAfter(int ms);
void intakeOff();

// =====================================================================
//   AI VISION AIM
// =====================================================================
enum class AimKind { none, pin, goalTag };
void visionStart();
// Starts lining the current move up on a pin (pin camera) or a goal's AprilTag (goal camera) that should be at
// (routeX, routeY). tagId: the goal's tag (-1 = any tag near where the goal should be, NO_GOAL_TAG = don't use the
// goal camera for this goal: odometry + MCL only).
inline constexpr int NO_GOAL_TAG = -2;
void aimAt(AimKind kind, double routeX, double routeY, int tagId = -1);
void aimStop();
Vec aimShift();   // the current nudge (field inches)

// =====================================================================
//   SERVICE
// =====================================================================
void service();          // runs every 10 ms while the robot drives or waits
bool waitMs(int ms);     // waits while everything keeps running. false = time is up

// =====================================================================
//   CURVES + FOLLOWER
// =====================================================================
inline constexpr double FP_SAMPLE_STEP = 0.5;   // the curve is stored as a point every half inch
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
Curve makeCurve(const std::vector<Node>& nodes, bool reverse, double maxSpeed, double endSpeed);
void addSlowdown(Curve& curve, double endAt, int ms, double speed);
void addSlowdownAtEnd(Curve& curve, int ms, double speed);

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
    bool gateHold = true;       // false = never stops at the gate: keeps creeping in while the lift finishes
    bool brakeAtEnd = true;     // false = it's switching direction or carrying on, don't brake
    int stallMs = FP_STALL_MS;  // told to drive but not moving for this long = it's against something, the move ends
    int timeoutMs = 0;          // 0 = worked out from the curve
};
MoveEnd follow(const Curve& curve, const MoveOptions& options);
void setGateHold(bool hold);   // whether the scoring moves below may stop at the lift gate (default true; skills: false)
// Turns in place to a field heading (proportional spin). false = time is up.
bool spinTo(double heading, int maxMs = 1500);
// Full power straight ahead until it has gone this far (odometry) or stopped against something. false = time is up.
bool shoveForward(double inches, int maxMs);

// =====================================================================
//   ROUTE HELPERS
// =====================================================================
double backCornerLineGap();     // back corners to the auton line (x + y = 0 in route coordinates), inches on our side
double frontCornerPastWall();   // how far past the bottom wall (route) the lowest front corner is (negative = not there)
double clawGap(double routeX, double routeY);   // middle of the claw to a spot, with the camera's nudge
// Letting go at a goal, both functions below:
//   lowerBeforeOpenMs = 0:  claw opens, then releaseMs later the move ends (the lift comes down on the way out)
//   lowerBeforeOpenMs > 0:  the lift starts DOWN first, the claw opens that many ms later (the pin is set down on the
//                           goal, not dropped), then releaseMs later the move ends with the lift still coming down
// Drives a curve through a pin (node pinNode), picks it up on the way through without stopping, then carries on
// along the curve into the goal at (goalX, goalY) with the lift going up to scoreHeight, and lets go. false = time up.
bool pickUpAndScore(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY,
                    double scoreHeight, double goalX, double goalY, int goalTagId, int pinSlowMs, int scoreSlowMs, int settleMs,
                    int releaseMs, int lowerBeforeOpenMs = 0);
// Drives a curve through a pin (node pinNode) and picks it up on the way through, then stops at the end of the
// curve with the lift hopping up to thenHeight (-1 = just the hop). false = time up.
bool pickUp(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY, double thenHeight,
            int pinSlowMs);
// Drives a curve into the goal at (goalX, goalY) with what the claw already holds, lift going up to scoreHeight,
// and lets go. false = time up.
bool scoreOnGoal(const std::vector<Node>& nodes, double scoreHeight, double goalX, double goalY, int goalTagId, int scoreSlowMs,
                 int settleMs, int releaseMs, int lowerBeforeOpenMs = 0);
// Odometry snap: the claw is physically on the goal at (goalX, goalY) (the aligner seated it), so the robot's position
// is FP_CLAW_REACH behind the goal along its heading, whatever odometry thought. Sets odometry to that.
void snapToGoal(double goalX, double goalY);
// Backs away from the goal and swings the nose to a new heading (one backwards curve). The lift drops and the
// claw motor turns off on the way. onTick (optional) is called with the inches backed. false = time up.
bool backOutTo(double straight, const Node& turnSpot, const std::function<void(double done)>& onTick = nullptr);
// Backs dead straight this far and stops. false = time up.
bool backStraight(double inches);

}  // namespace ac
