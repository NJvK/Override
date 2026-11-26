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
// NOTE 2026-11-23: measured on the robot, the front of the flex wheels is 8.6" ahead of the tracking center (field
// camera lens 6.6" + 2"), not 10.5. Left as it is: the loader drive and the route *_EXTRA values were tuned on the
// robot with 10.5 / FP_CLAW_REACH 7.0, and changing them moves every tuned stop. Remeasure FP_CLAW_REACH (tracking
// center to the middle of a cup when the claw closes on it) if pickups close early / late.
inline constexpr double FP_CLAW_HALF_WIDTH = 3.6;    // half the claw's open width (7.25" between the flex wheel centers)
// Tracking center to the middle of a pin / cup when the claw closes on it (the inside of the claw: the flex wheel
// centers are at 8.5, their closed gap 3.55", open 7.25")
inline constexpr double FP_CLAW_REACH = 7.0;   // measured 6.75 on 2026-11-23; kept at 7.0 (routes tuned with it, 0.25" is in the noise)
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
inline constexpr double FP_DECEL = 130.0;           // slowing down for a stop or a tight curve (was 150: a drive that can't brake
                                                    // that hard runs past the end of every move)
inline constexpr double FP_LAT_ACCEL = 160.0;       // how hard it corners (in/s^2): lower = slower in curves
inline constexpr double FP_MAX_TURN_DPS = 240.0;    // fastest it's allowed to spin in a curve (degrees/s)
inline constexpr double FP_MIN_SPEED = 5.0;         // never slower than this while it's still driving
inline constexpr double FP_CUSP_SPEED = 8.0;        // speed left at a forwards <-> backwards switch (it doesn't settle there)
inline constexpr double FP_GRAB_SPEED = 30.0;       // speed through a pin while the claw closes on it (in/s)
inline constexpr double FP_INTAKE_SPEED = 26.0;     // speed for the last inches backing into the cluster pin
inline constexpr double FP_WALL_SPEED = 26.0;       // speed into the wall cup / the matchloader
// Steering: the same as chasing a point this far ahead on the curve (inches). Lower = follows tighter, twitchier.
// The distance grows with speed (FP_STEER_TIME_S seconds of driving ahead), between FP_STEER_DIST and FP_STEER_DIST_MAX:
// a fixed 6" at 50 in/s is a tenth of a second ahead, which is what made the fast approaches zigzag.
inline constexpr double FP_STEER_DIST = 6.0;
inline constexpr double FP_STEER_DIST_MAX = 12.0;
inline constexpr double FP_STEER_TIME_S = 0.20;
// The sideways correction never acts on more than this many inches off the curve. A position fix (MCL, a distance
// reset, the camera's nudge) can move the robot's idea of where it is by inches in one step; without the cap that
// was a full-lock swerve.
inline constexpr double FP_STEER_MAX_OFF = 4.0;
inline constexpr double FP_STEER_PREVIEW_S = 0.08;  // looks this far ahead in time for the curve's bend (covers the motors' lag)
inline constexpr double FP_HEADING_GAIN = 6.0;      // pointing correction at low speed (degrees/s of turn per degree off)
inline constexpr double FP_FF_ACCEL_MAX = 300.0;    // most wheel acceleration the feedforward asks for (in/s^2)
inline constexpr double FP_SPEED_FEEDBACK = 0.20;   // extra volts per in/s a side is off its speed (was 0.10: too weak to hold the
                                                    // planned braking when the drive is slower than the feedforward thinks)
// Coming to a stop: on top of FP_DECEL, the speed is never more than this many in/s per inch left to go, so the
// last inch is gentle and it doesn't slide past the spot. Lower = softer and a bit slower.
inline constexpr double FP_STOP_GAIN = 8.0;
inline constexpr double FP_END_TOLERANCE = 0.6;     // inches from the end of a curve that counts as there
// With the lift up it drives slower so it doesn't tip: full speed with the lift at or below LO, FP_TALL_SPEED at HI
// and above, a straight line between. "The lift" here is the higher of where it is and where it's been told to go,
// so the cap is already on while it climbs. With main.cpp's ALLIANCE_HEIGHTS (160 / 240 / 315 / 395 / 522) that's
// 56 for spot 1, 53 for spot 2, 43 for spot 3 and 32 for spots 4 and 5: each stack spot a bit slower than the last,
// and only the top two at the slow speed. (There used to be a flat 28 in/s cap for everything above spot 2.)
inline constexpr double FP_TALL_LIFT_LO = 220.0;    // lift degrees
inline constexpr double FP_TALL_LIFT_HI = 400.0;
inline constexpr double FP_TALL_SPEED = 31.0;       // in/s
inline constexpr double FP_TALL_ACCEL = 90.0;       // speeding up with the lift at HI, in/s per second
inline constexpr double FP_TALL_DECEL = 130.0;      // most it brakes with the lift at HI (hard braking pitches it forward)
inline constexpr double FP_MOVING_SPEED = 28.0;     // in/s for the first straight inches off a goal while the lift comes down
// A move ends if the robot is being told to drive but hasn't moved for this long (it's against something)
inline constexpr int FP_STALL_MS = 250;
// Driving on into a goal until it stops the robot (ScorePlan::pushIn): the speed it meets the goal at, and how long
// stopped counts as "it's on the goal"
inline constexpr double FP_CONTACT_SPEED = 12.0;    // in/s
inline constexpr int FP_CONTACT_STALL_MS = 60;
// Turning in place (spinTo): spins up to FP_SPIN_MAX_DPS and brakes at FP_SPIN_DECEL_DPS2 so it arrives without
// crawling the last degrees (the old proportional spin spent 0.7 s on the last 60 degrees). It doesn't wait to
// settle: once it's within FP_SPIN_DONE_DEG the next move starts and squares up the rest on the way.
inline constexpr double FP_SPIN_MAX_DPS = 360.0;
inline constexpr double FP_SPIN_ACCEL_DPS2 = 1500.0;   // spinning up
inline constexpr double FP_SPIN_DECEL_DPS2 = 1100.0;   // braking into the heading
inline constexpr double FP_SPIN_MIN_DPS = 45.0;        // never slower than this while it's still turning
inline constexpr double FP_SPIN_DONE_DEG = 3.0;        // counts as there when this close...
inline constexpr double FP_SPIN_DONE_DPS = 90.0;       // ...and turning slower than this (degrees/s)

// =====================================================================
//   LIFT
// =====================================================================
inline constexpr double FP_LIFT_READY_TOLERANCE = 15.0; // lift counts as up when within this of its height
// ...or once it has STOPPED CLIMBING short of that (it tops out a little low with a heavy stack / weak bands). It
// used to wait up to FP_GATE_MAX_MS (1.5 s) at the goal for a lift that was never going to get any higher, then let
// go anyway: the "stops right before it lowers to score" on the robot. Now it scores the moment the lift stops rising.
inline constexpr double FP_LIFT_RISE_DEG = 2.0;         // climbing = it rose at least this much...
inline constexpr int FP_LIFT_STUCK_MS = 120;            // ...in this long. Not climbing for this long = as high as it gets
inline constexpr int FP_LIFT_STUCK_MIN_MS = 300;        // never decides that sooner than this after it was sent up
// Pickup hop: when the claw closes on a pin / cup off the field (not the intake), the lift hops up a little, the
// same as driver control's pin pickup (PIN_LIFT_HEIGHT / PIN_LIFT_MIN_RISE / PIN_LIFT_VOLTAGE in robot.hpp).
// No stall check: the route already knows when the claw closes, so it just waits this long for the claw to shut.
inline constexpr int FP_PICKUP_LIFT_DELAY_MS = 100;
// The lift gate: this many inches before a goal the lift should be up (so the pin can't hit the stack). With gate
// holding ON (the default) the robot stops there and waits for the lift; with it OFF (setGateHold(false): skills and
// the 4-pin's field pins) it never stops, it just keeps rolling in at the creep speed while the lift finishes.
inline constexpr double FP_SCORE_GATE_IN = 7.0;
inline constexpr int FP_GATE_MAX_MS = 1500;             // holding: stops waiting after this long
// Gate creep (gateEta): instead of stopping at the gate to wait for the lift, it works out how long the lift still
// needs and drives just fast enough (never over FP_MOVING_SPEED) to reach the gate as the lift gets there.
inline constexpr double FP_LIFT_UP_DPS = 550.0;         // about how fast the lift climbs (deg/s); lower = drives in slower
inline constexpr double FP_LIFT_START_S = 0.25;         // ...plus this long to get going and settle (seconds)
// "Late" lifting (ScorePlan::liftLate): the lift stays down while the robot turns and drives, and starts up when the
// time left to the gate is the time the lift needs (liftSeconds) plus this margin. If the lift on the robot is slower
// than FP_LIFT_UP_DPS says, the robot creeps the last stretch waiting for it: lower FP_LIFT_UP_DPS.
inline constexpr double FP_LIFT_LEAD_MARGIN_S = 0.15;
inline constexpr double FP_GATE_CREEP_MIN = 8.0;        // never slower than this while creeping in (in/s)
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
// lift goes to the height the route wants by then (or stays down, if the route has let go and lowered it since),
// and the route carries on.
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
//                opening. The gears block the bottom third of its picture. Its job is the last part of a pickup: the
//                cup it sees is lined up with the middle of the claw.
//   FIELD camera (port GOAL_VISION_PORT, 21): low at the front-LEFT corner of the chassis, turned inward (pointing to
//                the right of straight ahead, toward the claw). Sees cups AND the AprilTags on the goals, and it lines up
//                every goal approach on the goal's tag. Better lit than the claw camera.
// PICKUPS ARE AIMED ON CUPS ONLY (2026-11-23): the model finds cups far better than pins, and every pickup has a cup
// (a lone cup, a stack = a pin in a cup, the matchload). Pin detections are ignored. Odometry + MCL are already close,
// so a camera only corrects when it is really close (FP_AIM_MAX_DIST / FP_FIELD_PIN_MAX_DIST) and sees exactly ONE cup
// near the spot (FP_PIN_WINDOW_IN). None, or two that disagree (FP_AIM_SAME_IN) = it isn't sure: that frame is ignored.
// On the way into a pickup the claw camera is asked first (only with the claw open); if it has nothing, the field camera.
inline constexpr bool FP_USE_PIN_VISION = true;
inline constexpr int FP_PIN_CAM_PORT = AI_VISION_PORT;   // the claw camera (include/robot.hpp)
// The model's class ids for a cup (the sensor doesn't report class names). Claw Cam 2026-11-23: cup = 4 (pins = 2 / 3,
// not used). If a stack or the matchload shows up as another id in the Claw Cam tool, add it here.
inline constexpr int FP_CUP_CLASS_IDS[] = {4};
inline bool isCupClass(int id) {
    for (int cup : FP_CUP_CLASS_IDS) if (id == cup) return true;
    return false;
}
inline constexpr double FP_PIN_CAM_FORWARD = 4.9;       // inches the claw camera's lens sits in front of the tracking center (7.6" up)
inline constexpr double FP_PIN_CAM_YAW = 2.0;           // degrees it points right of straight ahead. Claw Cam 2026-11-23: a pin on
                                                        // the claw's center line read -3 / -1 / -2 deg (close / 12" / 24") with 0
inline constexpr double FP_PIN_CAM_TILT_DEG = 15.0;     // degrees it points DOWN. A tilted camera sees sideways angles a little
                                                        // squashed; the bearing math undoes that
inline constexpr bool FP_PIN_CAM_UPSIDE_DOWN = false;   // true if it's mounted upside down (left and right swap in its picture)
// The claw's own parts fool the model (2026-11-23 screenshots: the red gears read as "Cup" / "Red Yellow Pin" at
// center rows 188 / 207, a claw arm as "Cup" at row 164, while a real pin 12-24" out centers about row 105):
//   - anything whose box CENTER is at or below this row (240 rows, 0 = top) is the robot, not a cup
inline constexpr int FP_CLAW_CAM_MAX_CENTER_Y = 160;
//   - the autos only use the claw camera while the claw is OPEN: opening, and for at least this long (the arms
//     swing out of the picture). Closed / off (it closes by itself) = the arms are in view, so it's not read.
inline constexpr int FP_CLAW_CAM_OPEN_MS = 200;   // measured 2026-11-23: the claw opens fully in 200 ms
inline constexpr bool FP_USE_GOAL_TAG = true;           // the field camera lines goal approaches up on the tags
// ...and lines pickups up on cups too, close up only (FP_FIELD_PIN_MAX_DIST). It's the only camera for pickups the
// claw doesn't open for (the 4-pin's wall cup, the skills matchload).
// Back ON 2026-11-23 once FP_GOAL_CAM_YAW was fixed (the cup check read +0.2" at 18" and +0.8" at 30").
inline constexpr bool FP_FIELD_CAM_SEES_PINS = true;
inline constexpr int FP_GOAL_CAM_PORT = GOAL_VISION_PORT;   // the field camera (include/robot.hpp)
// Where the field camera's lens is, from the tracking center (the point odometry tracks, between the drive wheels).
// Measured: 6 5/8" forward, 5 1/8" LEFT (2026-11-23, was 6.5 / 5.5), 3.6" up.
inline constexpr double FP_GOAL_CAM_FORWARD = 6.625;    // inches forward
inline constexpr double FP_GOAL_CAM_RIGHT = -5.125;     // inches to the right (- = left: it's on the left side)
// How far it's turned inward: degrees it points RIGHT of straight ahead. It sits on the left and is turned in toward
// the claw, so this is positive. Measured 23.58. To check it: put a goal with its tag dead ahead of the robot's
// center line, 36" from the tracking center (tape measure down the middle of the robot). The brain screen's FIELD CAM
// line shows the tag's "raw" bearing (degrees right of the camera's own straight ahead). Then
//   FP_GOAL_CAM_YAW = (the bearing the tag SHOULD have from the camera) - raw
// where the should-have bearing = atan2(-FP_GOAL_CAM_RIGHT, 36 - FP_GOAL_CAM_FORWARD) in degrees = about +10.6 with the
// numbers above (9.9 now) (the tag is to the right of a camera that sits on the left). Example: raw reads -13, so yaw = 10.6 + 13 = 23.6.
// 33.5 (2026-11-23, was 23.58 from an old tag check). Field Cam cup check, cup dead ahead at 18" / 30" from the
// tracking center: Y 34.0 / 33.1 (cup +0.2" / +0.8" right with 35.0). Tag at 36" NOT checked yet (no goal).
// Before that, measured on the robot: the camera's straight-ahead line crosses the
// robot's center line 5" in front of the claw, 8.5" from the lens, with the lens 2" behind the front of the claw:
// acos(7 / 8.5) = 34.6; atan(5.125 / 7) = 36.2. The Field Cam cup check at 18" gave 32-36 (depending on where the 18"
// was measured from). The old 23.58 made things look about 12 deg further LEFT than they are. Check it with the Field
// Cam tool: a cup dead ahead at 18" and 30", and a tag dead ahead at 36", should all read about this.
inline constexpr double FP_GOAL_CAM_YAW = 33.5;
inline constexpr bool FP_GOAL_CAM_UPSIDE_DOWN = false;
// Goal tags are used for DIRECTION only: the approach is moved sideways (across the line of sight) so it points at
// the tag. The tag's size in the picture is not used to work out how far the goal is. It used to be (distance =
// focal x tag size / pixels, then FP_GOAL_TAG_TO_CENTER further in), and with a tag size and a tag-to-center depth
// that were never measured the goal came out too close to the camera. The camera sits on the robot's left, so
// "too close to the camera" pulled every approach left and short: what the skills run did at goal A and goal B.
// How far to drive in comes from odometry plus ScorePlan::pushIn (drive on until the goal stops the robot).
inline constexpr double FP_GOAL_TAG_TO_CENTER = 4.0;    // inches from the tag (on the goal's side) in to the goal's center
inline constexpr double FP_GOAL_TAG_MIN_PX = 10.0;      // a tag smaller than this is too far / too blurry
// Only a tag that faces the camera is trusted. A tag seen from the side sits off to one side of the goal's center
// (up to the goal's radius), so lining up on it would miss the middle. Squareness = how wide the tag looks divided by
// how tall: 1 = face on, 0.9 = turned 26 degrees away (about 1.7" off center at most).
inline constexpr double FP_GOAL_TAG_MIN_SQUARENESS = 0.90;
inline constexpr double FP_GOAL_AIM_MAX_DIST = 48.0;    // starts lining up on the tag this far out (tags read from further than pins)
inline constexpr double FP_GOAL_AIM_MIN_DIST = 8.0;     // stops updating this close to the goal (camera to tag: the tag leaves the picture)
inline constexpr double FP_GOAL_MAX_SHIFT = 4.0;        // most the approach is moved sideways (inches)
inline constexpr double FP_GOAL_TAG_WINDOW_IN = 7.0;    // only trusts a tag within this of where the goal should be (inches sideways)
inline constexpr double FP_GOAL_AIM_SMOOTH = 0.45;      // 0-1: how fast the nudge follows the tag
// Both cameras
inline constexpr double FP_CAM_WIDTH_PX = 320.0;
inline constexpr double FP_CAM_HEIGHT_PX = 240.0;
inline constexpr double FP_CAM_FOV_DEG = 74.0;          // side to side
// Pickups: only trusts a cup within this many INCHES (sideways) of where it should be (was 6.5: odometry + MCL are
// close, so a cup further off is a different one). The 4-pin's wall cup uses its own, smaller FP_CUP_WINDOW_IN.
inline constexpr double FP_PIN_WINDOW_IN = 3.0;
// Two cups in the window further apart than this (inches) = not sure which one: that frame is ignored. Closer than
// this they're the same cup seen twice (the model often draws two boxes on one thing).
inline constexpr double FP_AIM_SAME_IN = 1.5;
inline constexpr double FP_AIM_MAX_SHIFT = 3.0;         // most the curve is nudged sideways for a pickup (inches; was 5)
// The cameras report what they saw a moment ago. While the robot is turning that matters: at 200 deg/s, 50 ms is 10
// degrees, which read as "the pin is 4 inches to the side". Every sighting is compared with where the robot was and
// which way it pointed FP_CAM_LATENCY_MS ago, and sightings taken while spinning faster than FP_AIM_MAX_TURN_DPS
// (too blurred to trust) are skipped.
inline constexpr int FP_CAM_LATENCY_MS = 50;
inline constexpr double FP_AIM_MAX_TURN_DPS = 230.0;
// The nudge never moves faster than this (inches per second), so one odd sighting can't jerk the steering
inline constexpr double FP_AIM_SLEW_IPS = 14.0;
inline constexpr double FP_AIM_FADE_IPS = 6.0;          // and it fades back to nothing this fast once the aiming stops
// Claw camera: starts aiming this far out (camera to cup). The camera (4.9") is 3.7" behind the front of the flex
// wheels (8.6"), so 18 = about 14" in front of the claw. Claw Cam 2026-11-23, cup on the center line: +0.5 deg at 8"
// and 0.0 at 12" in front of the claw, nothing at 4" (was 30, then 14)
inline constexpr double FP_AIM_MAX_DIST = 18.0;
// ...and only at a cup nearly straight ahead of the claw: within this many degrees of the robot's straight ahead
// (claw camera only: the field camera sits off to the side)
inline constexpr double FP_AIM_AHEAD_DEG = 12.0;
inline constexpr double FP_FIELD_PIN_MAX_DIST = 20.0;   // field camera: starts aiming this far out (camera to cup; was 48)
inline constexpr double FP_AIM_MIN_DIST = 5.0;          // stops updating this close (the cup is under the camera's view)
inline constexpr double FP_AIM_SMOOTH = 0.35;           // 0-1: how fast the nudge follows the camera (lower = smoother)
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
int timeLeft();       // ms until the auton's limit
bool timeUp();        // the limit has been reached (the lift is already limp)
// How long before the limit everything stops (default FP_COAST_BEFORE_END_MS). Call after coreStart.
void setEndReserve(int ms);
// Last thing in a route whose time ran out: gets the robot rolling at (routeX, routeY) so it's carried into the zone
// after the motors cut out. If the zone (a circle of zoneRadius around the spot) isn't ahead of it or behind it, it
// turns toward it first, but always keeps the last driveMs for driving flat out, forwards or backwards, whichever
// end faces the zone. Does nothing unless timeUp(). Leaves the drive on coast.
void endDash(double routeX, double routeY, double zoneRadius, int driveMs);
void note(const char* what);   // one line on the terminal with the time, pose and lift (and on controller line 2)
// The route's name + time and the pose on controller lines 0 / 1 while it runs, its last step on line 2, and the end
// time + pose once finishSafe() runs. coreStart() turns it on; the tuning tools turn it off to use the lines themselves.
void showStatusOnController(bool on);
// Leaves everything safe however a route ended: drive stopped (unless stopDrive is false: after endDash the drive is
// left coasting), intake / flipper off, claw off or on its hold
void finishSafe(bool stopDrive = true);

// =====================================================================
//   LIFT
// =====================================================================
void liftTo(double height);           // to a height with the lift PID
void liftHopThen(double thenHeight);  // pickup hop FP_PICKUP_LIFT_DELAY_MS from now, then on to thenHeight (-1 = hold there)
void liftAfterHop(double height);     // to a height, but if a pickup hop is waiting or running it finishes first
double liftSeconds(double height);    // about how long the lift takes to get up to a height from the bottom
void liftDown();                      // all the way down as fast as it goes, then a light hold on the bottom
void liftPress(bool on);              // while down: push with FP_PICKUP_PRESS_MV instead of the light hold (picking up)
void liftCoast();                     // PID off and the motor limp
void liftRaiseDirect(double height, int maxMs);   // full power up until it reads height (or maxMs): the start toggles
bool liftReady();                     // at the height it was sent to (pid mode)
bool liftStuck();                     // sent up (pid mode), short of its height, and no longer climbing
bool liftUpOrStuck();                 // liftReady() or liftStuck(): as high as it's going to get
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
// windowIn: only trusts a sighting within this many inches (sideways) of where the thing should be. 0 = the usual
// window (FP_PIN_WINDOW_IN / FP_GOAL_TAG_WINDOW_IN). Smaller where two of the same thing sit side by side.
inline constexpr int NO_GOAL_TAG = -2;
void aimAt(AimKind kind, double routeX, double routeY, int tagId = -1, double windowIn = 0.0);
void aimStop();
Vec aimShift();   // the current nudge (field inches)
int aimSightings();   // camera frames that found the thing during the last aim (still there after aimStop)
// Vision Pickup tool: where the two cameras put ONE cup / stack, robot standing still. Robot frame: x = inches right of
// the tracking center, y = inches ahead. A fix = both cameras see exactly one cup in the same frame and their lines of
// sight cross in front of the robot. Watches for `ms`; with 5+ fixes, x / y are their medians and spread = the
// furthest a fix was from that.
struct CupFix {
    int frames = 0, fixes = 0, clawOnly = 0, fieldOnly = 0;
    double x = 0, y = 0, spread = 0;
};
CupFix locateCup(int ms);

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
    // Driving INTO something at the end of the curve (a goal, the wall): within endStallIn inches of the end the stall
    // time is endStallMs instead, so touching it ends the move almost at once (0 = same as stallMs)
    int endStallMs = 0;
    double endStallIn = 0.0;
    int timeoutMs = 0;          // 0 = worked out from the curve
};
MoveEnd follow(const Curve& curve, const MoveOptions& options);
void setGateHold(bool hold);   // whether the scoring moves below may stop at the lift gate (default true; skills: false)
// Turns in place to a route heading, the short way round. false = time is up.
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
// ---- Scoring with a plan (the skills run uses these; the two functions above are the same thing with the lift
// going up straight away and no push) ----
struct ScorePlan {
    double height = 0;                 // lift height to let go at
    double goalX = 0, goalY = 0;       // the goal (route coordinates)
    int tagId = NO_GOAL_TAG;           // its AprilTag (-1 = any, NO_GOAL_TAG = don't use the camera)
    int settleMs = 0, releaseMs = 150, lowerBeforeOpenMs = 0;   // letting go (see above)
    int scoreSlowMs = 0;               // old-style slowdown: this long at FP_SCORE_SLOW_SPEED at the end
    // true = the lift stays down while it turns and drives and starts up as late as it can (FP_LIFT_LEAD_MARGIN_S);
    // false = it starts up straight away (after the pickup hop, if there's a pickup)
    bool liftLate = false;
    // The last approachIn inches before the goal are driven at no more than approachSpeed (0 = no cap)
    double approachSpeed = 0;
    double approachIn = 10.0;
    // The curve carries on this many inches past the spot where the claw should be on the goal, and the move ends as
    // soon as the goal stops the robot. Covers odometry reading a few inches long ("didn't drive far enough in").
    double pushIn = 0;
    // > 0: the claw starts opening this many inches before the goal (no pickup on the way: the goal goes into the open
    // claw, e.g. so the flipper can put the intake's pin on it)
    double clawOpenIn = 0;
    // false = it doesn't let go: the move ends stopped on the goal with the lift at its height and the claw as it is
    // (the route does what comes next: flips, or just stays there at the end of the run)
    bool release = true;
    // The lift never starts up before this time (elapsed() ms; 0 = no limit), e.g. so the intake can finish passing a
    // pin to the flipper. If that makes it late, the robot keeps rolling in slowly while the lift catches up.
    int liftNotBeforeMs = 0;
};
// Stopped at a goal: lets go as described above, with a distance-sensor reset while it stands there (FP_RESET_AT_GOALS).
// The scoring functions call it themselves; a route that drives into a goal its own way calls it too. false = time up.
bool letGo(int settleMs, int releaseMs, int lowerBeforeOpenMs);
// Drives the curve into the goal with what the claw holds and lets go. The last node is the robot's spot with the
// claw on the goal. false = time up.
bool scoreWith(const std::vector<Node>& nodes, const ScorePlan& plan);
// Same, picking a pin up on the way through (node pinNode). false = time up.
bool pickUpAndScoreWith(const std::vector<Node>& nodes, int pinNode, double pinX, double pinY, double mclPinX, double mclPinY,
                        int pinSlowMs, const ScorePlan& plan);
// Nodes for backing straight `straight` inches and then on round an arc `arcIn` inches long that swings the nose
// noseTurnDeg (+ = clockwise), starting from `start`. For reversing out of a goal without aiming at a fixed spot.
std::vector<Node> backArc(const Node& start, double straight, double arcIn, double noseTurnDeg);
// Odometry snap: the claw is physically on the goal at (goalX, goalY) (the aligner seated it), so the robot's position
// is FP_CLAW_REACH behind the goal along its heading, whatever odometry thought. Sets odometry to that.
void snapToGoal(double goalX, double goalY);
// Backs away from the goal and swings the nose to a new heading (one backwards curve). The lift drops and the
// claw motor turns off on the way. onTick (optional) is called with the inches backed. false = time up.
bool backOutTo(double straight, const Node& turnSpot, const std::function<void(double done)>& onTick = nullptr);
// Backs dead straight this far and stops. false = time up.
bool backStraight(double inches);

}  // namespace ac
