// SKILLS autonomous (1 minute). main.cpp calls manyPinAlliance() at the bottom of this file.
// The qual routes still live in auton.cpp (twoPinAlliance).
#include "robot.hpp"
#ifdef RW_SIM_PROS
#include <cstdlib>   // getenv (SIM_MISS / SIM_DROP, see skillsClawHasPin)
#include <string>
#endif

// Everything in this file is inside its own namespace, so it can live next to auton.cpp
// (which has functions with the same names) without "multiple definition" errors.
namespace skills_file {

// =====================================================================
//              AUTON: 2 PINS ON THE ALLIANCE GOAL (tuning)
// =====================================================================
// Field coordinates are in inches, with (0, 0) at the center of the field.
// +X = toward the right wall (the wall the robot starts on), +Y = toward the top of the field map.
// Headings: 0 = facing the top of the map, 90 = facing right, 180 = facing down, 270 = facing left.
// One tile = 24 inches.

// ---------------------------------------------------------------------
// IDEAL FIELD + ROBOT GEOMETRY
// Every position below is worked out from these: exact tile spots on the field (24" tiles) and the walls,
// adjusted for the robot's size so the CLAW (not the robot's center) ends up on the right spot.
// All of them are the IDEAL field values: the old field-tuned offsets were reset to 0, because with MCL /
// distance resets the robot knows where it is instead of the numbers making up for it.
// ---------------------------------------------------------------------
constexpr double TILE = 24.0;                    // one field tile
// Walls: RW-Template's field_half_size. The perimeter sits on the edge tiles, so the inside of the walls is
// at +-70.25, not +-72 (tiles are still 24", so field elements stay on multiples of 24). MCL and the
// distance resets use 70.25 too, so every wall-based spot here matches what the sensors measure.
constexpr double FIELD_HALF = 70.25;
// Whole robot (not just the drivetrain), measured
constexpr double ROBOT_LENGTH = 16.35;           // front to toggle (back) (was 16.5)
constexpr double ROBOT_WIDTH = 16.78;            // (was 17)
constexpr double ROBOT_CENTER_TO_BACK = 8.3;     // tracking center (middle of the drivetrain) to the back, measured
constexpr double ROBOT_CENTER_TO_FRONT = ROBOT_LENGTH - ROBOT_CENTER_TO_BACK;   // 8.05: tracking center to the front
constexpr double ROBOT_HALF_DIAG = 11.80;        // sqrt(8.3^2 + 8.39^2): how far the back corners swing out when it turns in place
                                                 // (the front ones, sqrt(8.05^2 + 8.39^2) = 11.63, swing a little less)
constexpr double GOAL_HALF = 4.0;                // half the width of a goal's base
constexpr double SMALL_PIN_RADIUS = 2.0;         // the small pins on the diamond corners

// Start: back of the robot flat against the right wall, centered on the wall, claw facing into the field.
constexpr double AUTON_START_X = FIELD_HALF - ROBOT_CENTER_TO_BACK;   // 70.25 - 8.3 = 61.95
constexpr double AUTON_START_Y = 0.0;                 // middle of the wall
constexpr double AUTON_START_HEADING = 270.0;

// Alliance goal (the one being scored on)
constexpr double AUTON_GOAL_X = 2.0 * TILE;          // 48: alliance goal sits on a tile corner
constexpr double AUTON_GOAL_Y = 1.0 * TILE;          // 24

// Pin + cup 1: one tile left of the goal
constexpr double AUTON_PIN1_X = 23.55;               // RW field grid, same as the goals (was 24)
constexpr double AUTON_PIN1_Y = 23.55;               // (was 24)
constexpr double AUTON_PIN1_APPROACH_HEADING = 322.0;  // drives in from the lower right (stays clear of the small pin below it)

// Pin + cup 2: one tile above the goal
constexpr double AUTON_PIN2_X = 47.09;               // RW field grid (was 48)
constexpr double AUTON_PIN2_Y = 47.09;               // (was 48)
constexpr double AUTON_PIN2_APPROACH_HEADING = 43.0;   // drives in along the diagonal line from the lower left (stays clear of the goal)

// Lift
constexpr double AUTON_LOW_HEIGHT = 30.0;       // degrees: lift height for driving and grabbing. It drops here after the start lift and after each score
constexpr double AUTON_LIFT_TOLERANCE = 5.0;    // degrees: the lift counts as "there" when this close
// Coast lowering: after letting go of a pin/preload, the lift is lowered with the PID until it is this many degrees
// above AUTON_LOW_HEIGHT, then the motors switch to coast (no power) so momentum carries it the rest of the way down
// instead of the PID slowing it down at the end. Bigger = lets go sooner. 0 = off (PID all the way down).
constexpr double AUTON_LIFT_COAST_DIST = 40.0;
// Grounding: while driving to each pin the lift is pushed all the way down to 0 (the PID alone stops
// around 10 because of its bottom deadzone). It pushes harder until it reaches the bottom, then holds lightly.
constexpr int AUTON_GROUND_PUSH_MV = 4000;      // mV pushing down until it reaches the bottom
constexpr int AUTON_GROUND_HOLD_MV = 1200;      // mV holding it down once it's there (low so the motors don't heat up)
constexpr double AUTON_GROUND_DONE_DEG = 1.0;   // counts as at the bottom at or below this

// Claw / grabbing
constexpr double AUTON_CLAW_REACH = 6.5;        // inches from the robot's center (middle of the drive wheels) to the pin when the claw closes on it,
                                                // measured (was 8). The front of the robot is 1.55" past it
constexpr double AUTON_CLAW_OPEN_DIST = 5.0;    // starts spinning the claw when this many inches from the grab spot
constexpr int AUTON_APPROACH_CLAW_SPEED = 300;  // rpm the claw spins at while driving into a pin (keeps spinning, never stops to open).
                                                // + = same way as R1 (closing), - = same way as R2 (opening)
constexpr double AUTON_SLOW_DIST = 4.0;         // the last this-many inches to the pin are driven slowly
constexpr int AUTON_SLOW_SPEED = 25;            // speed driving straight into a pin to pick it up (out of 127) (was 40)
constexpr int AUTON_CLAW_HOLD_SPEED = 100;       // slow grabbing speed (same as driver control)
constexpr int AUTON_CLAW_OPEN_MS = 300;         // how long the claw spins backward to open
constexpr int AUTON_CLAW_FAST_MS = 1000;        // claw squeezes at full speed this long after closing, then drops to the slow speed
constexpr int AUTON_GRAB_TIMEOUT_MS = 300;      // if no claw stall is seen by then, it does the small lift anyway (was 400)
constexpr double AUTON_PIN_PICKUP_RISE = 25.0;  // degrees the lift goes up right after grabbing pin 1 / pin 2 (was about 10, same as driver).
                                                // Auton only: driver control still uses PIN_LIFT_HEIGHT / PIN_LIFT_MIN_RISE

// Scoring
constexpr double AUTON_PRE_SCORE_GAP = 4.0;     // stops this many inches short of the goal and waits for the lift before going over it
constexpr int AUTON_SCORE_SPEED = 80;           // speed for those last inches over the goal (out of 127) (was 50)
// Pins only (the preload keeps the two settings above):
constexpr double AUTON_PIN_PRE_SCORE_GAP = 2.0; // inches of slower driving at the end when scoring a pin (was 4)
constexpr int AUTON_PIN_SCORE_SPEED = 110;      // speed for that last bit over the goal when scoring a pin (was 80)
// Finish: if the robot still stopped short of the goal when scoring a pin (the drive timed out, or the pin
// bumped the goal because the lift wasn't up yet), it waits for the lift and drives the rest straight in.
constexpr double AUTON_SCORE_FINISH_TOLERANCE = 0.5;  // inches short that still counts as "there"
constexpr int AUTON_SCORE_FINISH_SPEED = 40;          // speed for that straight finish (out of 127)
constexpr int AUTON_SCORE_FINISH_TIMEOUT_MS = 1000;   // gives up after this long (in case it's stuck on the goal)
constexpr double AUTON_BACKUP_DIST = 4.0;       // backs up this far once the lift is halfway down
constexpr double AUTON_CLEAR_DIST = 6.0;        // then backs up this much more so the robot's corners don't hit the goal when it turns (was 4, +2" more)

// Start: toggle mechanism (back of the robot)
constexpr double AUTON_TOGGLE_LIFT_HEIGHT = 55.0;   // degrees: lift goes up this much first so the toggles clear
                                                    // (was 40: the start toggle still missed sometimes, so +15; 30 before that)
constexpr double AUTON_TOGGLE_ROTATIONS = 2.0;      // full turns of the toggle (output side). Never goes past this
constexpr double AUTON_TOGGLE_GEAR_RATIO = 2.0;     // motor turns per toggle turn (1:2 = motor spins 2x for 1 toggle turn)
constexpr int AUTON_TOGGLE_SPEED = 200;             // motor rpm (same direction as holding A in driver control)
constexpr int AUTON_TOGGLE_TIMEOUT_MS = 2500;
constexpr double AUTON_TOGGLE_START_LIFT = 40.0;    // degrees: toggle starts spinning once the lift is this high (doesn't wait for the full lift)
                                                    // (was 25, +15 like the lift height)
constexpr double AUTON_TOGGLE_DONE_DEG = 90.0;      // motor degrees: robot starts moving when the toggle is this close to done.
                                                    // The toggle still finishes its 2 turns exactly on its own and stops there.       // stops trying after this, in case it gets stuck

// Preload (scored first, right after the toggles)
constexpr double AUTON_PRELOAD_CARRY_HEIGHT = 100.0; // degrees: lift rises to this (from 30) while driving to the goal with the preload
constexpr double AUTON_PRELOAD_DROP_HEIGHT = 10.0;  // degrees: lift lowers to this over the goal before the claw opens
constexpr int AUTON_PRELOAD_CLAW_OPEN_MS = 500;     // claw opens longer for the preload than for pins (AUTON_CLAW_OPEN_MS)
// Turns this many degrees LESS than straight at the goal for the preload, then drives straight.
// Raise it if it still turns too far, lower it (or go negative) if it doesn't turn enough.
constexpr double AUTON_PRELOAD_EXTRA_DIST = 0.0;       // ideal: claw lands exactly on the goal (tuned value was -2)
constexpr double AUTON_PRELOAD_TURN_LESS_DEG = 0.0;    // ideal: aims straight at the goal (tuned value was -6.8)
constexpr double AUTON_PRELOAD_BACKUP = 10.0;       // inches it backs away from the goal after letting go (was 8, +2" more)

// Which stack spot each pin goes on (picks the height from ALLIANCE_HEIGHTS).
// The preload is already on the goal, so the pins go on spots 2 and 3.
// If the preload doesn't count as a spot in your height list, change these to 1 and 2.
constexpr int AUTON_PIN1_STACK = 2;
constexpr int AUTON_PIN2_STACK = 3;

// Per-pin tuning offsets
// GRAB_TURN_OFFSET: degrees. Turns the robot this much extra right before the slow drive into the pin.
//                   + = more to the RIGHT (clockwise), - = more to the LEFT
// GRAB_EXTRA_DIST:  inches. + = drives FURTHER forward into the pin when grabbing, - = stops shorter
// SCORE_TURN_LESS:  degrees. + = turns LESS toward the goal than straight at it, - = turns MORE (same idea as the preload)
// SCORE_EXTRA_DIST: inches. + = drives FURTHER toward the goal when scoring (it was stopping short), - = stops shorter
constexpr double AUTON_PIN1_GRAB_TURN_OFFSET = 0.0;   // (was field-tuned to 3)
constexpr double AUTON_PIN1_GRAB_EXTRA_DIST = -4.0;   // drove too far into pin 1 on the robot (was 0; field-tuned to 2 on the old route)
constexpr double AUTON_PIN1_SCORE_TURN_LESS = 0.0;    // (was field-tuned to 2)
constexpr double AUTON_PIN1_SCORE_EXTRA_DIST = 0.0;   // (was field-tuned to -1)

constexpr double AUTON_PIN2_GRAB_TURN_OFFSET = 0.0;   // (was field-tuned to 8)
constexpr double AUTON_PIN1_SLOW_DIST = 6.0;          // inches before the grab spot where it switches to slow (was 3, same as pin 2 now)
constexpr int AUTON_PIN1_APPROACH_SPEED = 70;         // top speed driving up to pin 1 (out of 127). Was full speed (127). Lower = slower
constexpr int AUTON_PIN1_SLOW_SPEED = 20;             // speed for the last slow inches into pin 1 (out of 127). Was AUTON_SLOW_SPEED (30)
constexpr double AUTON_PIN2_SLOW_DIST = 6.0;          // starts slow earlier for pin 2 (it was still knocking it over)
constexpr int AUTON_PIN2_EXTRA_CLAW_MS = 400;         // claw keeps spinning at full speed this much longer on pin 2 before the lift goes up (pin 1 = 0)
constexpr double AUTON_PIN2_GRAB_EXTRA_DIST = -2.0;   // drove too far into pin 2 on the robot (was 0)
constexpr double AUTON_PIN2_SCORE_TURN_LESS = 0.0;    // not tested yet
constexpr double AUTON_PIN2_SCORE_EXTRA_DIST = 0.0;   // not tested yet

// Speed-ups: instead of coming to a full stop between moves that go straight into each other
// (fast drive -> slow drive, turn -> drive), the first move hands off early while still rolling.
constexpr int AUTON_CHAIN_MIN_SPEED = 60;        // drive speed kept at the hand-off (out of 127) (was 40)
constexpr double AUTON_CHAIN_EXIT_DIST = 1.5;    // inches early the fast part hands off to the slow part
constexpr int AUTON_TURN_MIN_SPEED = 40;         // turn speed kept at the hand-off (was 30)
constexpr double AUTON_TURN_EXIT_DEG = 5.0;      // degrees early a turn hands off to the next drive
// true = the lift starts going up to scoring height while the robot turns toward the goal
// (it was only starting once the turn finished). false = old behavior.
constexpr bool AUTON_LIFT_DURING_TURN = true;
// false = never stops in front of the goal to wait for the lift; it keeps driving while the lift finishes.
// Set to true if the pin/cup starts hitting the goal because the lift isn't up in time.
constexpr bool AUTON_WAIT_FOR_LIFT_AT_GOAL = false;

// Aiming turns (the turn right before driving into a pin or the goal) always finish completely.
// This is the most time they're allowed. They only use it if they need it, so a turn is never
// cut off partway just because the battery is a bit low.
constexpr int AUTON_AIM_TURN_TIMEOUT_MS = 1500;
// Aiming turns count as done once they're this close (degrees), instead of waiting to settle
// perfectly. Waiting for the last degree or two was eating most of the 15 seconds.
// Lower = more exact but slower. Raise a little if turns still hang at the end.
constexpr double AUTON_AIM_EXIT_DEG = 1.5;
constexpr int AUTON_AIM_MIN_SPEED = 10;
// Same idea for drives that end in a stop (final drive into a pin/goal, backing up): done once this close (inches)
constexpr double AUTON_STOP_EXIT_DIST = 0.5;
constexpr int AUTON_STOP_MIN_SPEED = 10;
constexpr double AUTON_BACKUP_EXIT_DIST = 1.0;   // backing up doesn't need to be exact
// =====================================================================

// =====================================================================
//                     WHICH SIDE THE ROBOT STARTS ON
// =====================================================================
// false = RIGHT wall start (the original route above: toggle on the right wall, turns right to score)
// true  = TOP wall start (mirror route below: toggle on the top wall, turns left to score, other alliance goal)
// Everything that isn't listed in the TOP block (speeds, lift heights, claw, timings) is shared by both sides.
constexpr bool AUTON_START_ON_TOP = false;

// true = run the 1-minute SKILLS routine instead (see the SKILLS block further down). It starts on the LEFT wall.
// AUTON_START_ON_TOP is ignored when this is true.
constexpr bool AUTON_RUN_SKILLS = false;

// =====================================================================
//        TOP-WALL START (mirror of the right-wall route) - tuning
// =====================================================================
// This route is the right-wall route flipped across the field's diagonal line (the white line the pins sit on).
// Flipping swaps X and Y for every spot, and every heading becomes (90 - heading). Left and right swap,
// so the grab turn offsets change sign. Distance offsets and "turn less" offsets don't change.
// Starting values below are the right-wall tuning flipped. Tune these on the field like you did the right side.

// Start: back of the robot against the TOP wall, in the middle of the wall, claw facing down into the field.
constexpr double TOP_START_X = 0.0;            // right side: AUTON_START_Y (0)
constexpr double TOP_START_Y = FIELD_HALF - ROBOT_CENTER_TO_BACK;   // 61.95, right side: AUTON_START_X (was 62)
constexpr double TOP_START_HEADING = 180.0;    // 90 - 270 = facing down

// The other alliance goal (one tile left of the right-side goal, one tile higher)
constexpr double TOP_GOAL_X = 24.0;            // right side goal was (48, 24)
constexpr double TOP_GOAL_Y = 48.0;

// Pin + cup 1: one tile BELOW this goal (it's the same pin as right-side pin 1, it sits on the diagonal)
constexpr double TOP_PIN1_X = 24.0;            // right side (24, 24) flipped (was field-tuned to 23)
constexpr double TOP_PIN1_Y = 24.0;
constexpr double TOP_PIN1_APPROACH_HEADING = 128.0;   // 90 - 322: drives in from the upper left

// Pin + cup 2: one tile RIGHT of this goal (same pin as right-side pin 2)
constexpr double TOP_PIN2_X = 48.0;
constexpr double TOP_PIN2_Y = 48.0;
constexpr double TOP_PIN2_APPROACH_HEADING = 47.0;    // 90 - 43: drives in along the diagonal from the lower left

// Preload (same meaning as AUTON_PRELOAD_TURN_LESS_DEG / AUTON_PRELOAD_EXTRA_DIST)
constexpr double TOP_PRELOAD_TURN_LESS_DEG = 0.0;   // (was field-tuned to -6.8)
constexpr double TOP_PRELOAD_EXTRA_DIST = 0.0;      // (was field-tuned to -2)

// Per-pin offsets (same meaning as the AUTON_PIN... offsets: + grab turn = more RIGHT)
constexpr double TOP_PIN1_GRAB_TURN_OFFSET = 0.0;     // (was field-tuned to -3)
constexpr double TOP_PIN1_GRAB_EXTRA_DIST = 0.0;      // (was field-tuned to 2)
constexpr double TOP_PIN1_SCORE_TURN_LESS = 0.0;      // (was field-tuned to 2)
constexpr double TOP_PIN1_SCORE_EXTRA_DIST = 0.0;     // (was field-tuned to -1)

constexpr double TOP_PIN2_GRAB_TURN_OFFSET = 0.0;     // (was field-tuned to -8)
constexpr double TOP_PIN2_GRAB_EXTRA_DIST = 0.0;
constexpr double TOP_PIN2_SCORE_TURN_LESS = 0.0;
constexpr double TOP_PIN2_SCORE_EXTRA_DIST = 0.0;
// =====================================================================

// Everything that changes between the right-wall and top-wall starts
struct AutonRoute {
    double startX, startY, startHeading;
    double goalX, goalY;
    double preloadTurnLess, preloadExtraDist;
    double pin1X, pin1Y, pin1Heading, pin1GrabTurn, pin1GrabExtra, pin1ScoreTurnLess, pin1ScoreExtra;
    double pin2X, pin2Y, pin2Heading, pin2GrabTurn, pin2GrabExtra, pin2ScoreTurnLess, pin2ScoreExtra;
};

// Goal the scoring functions drive to (set from the route at the start of auton)
double autonGoalX = AUTON_GOAL_X;
double autonGoalY = AUTON_GOAL_Y;

// =====================================================================
//                 SKILLS: ANTI-TIP DRIVING (tuning)
// =====================================================================
// Drives with the lift up use a gentle speed-up and slow-down so the robot doesn't tip. Top speed is still high.
// "Tall" = lift above SKILLS_TALL_LIFT_DEG.
// Only ABOVE the preload + 2 pin stack height (spot 3 = 368.5) counts as tall now: below it the robot drives at full
// speed. Above it, everything below stays exactly as it was (the speeds that don't tip it). (was 150)
constexpr double SKILLS_TALL_LIFT_DEG = 380.0;
// The video showed the robot rocking/tipping even with the lift low (the old 8 / 10 were about 1 g of
// speed-up and 0.5 g of braking). These are about 3x gentler. Raise a little at a time if it's solid.
constexpr double SKILLS_ACCEL = 3.2;          // speed added per 10 ms when the lift is low (0 -> 120 in ~0.4 s) (was 2.5: no tipping, so a little quicker)
constexpr double SKILLS_ACCEL_TALL = 1.2;     // speed added per 10 ms when the lift is tall (was 3.5)
constexpr double SKILLS_DECEL = 5.0;          // slow-down: allowed speed per inch left to go (lift low) (was 4)
constexpr double SKILLS_DECEL_TALL = 1.5;     // same with the lift tall (starts slowing down earlier) (was 2: it rocked hard at goal A)
constexpr double SKILLS_BRAKE_PER_TICK = 4.0;      // most the speed can drop in 10 ms for any other reason (lift low)
constexpr double SKILLS_BRAKE_PER_TICK_TALL = 1.0; // same with the lift tall (was 1.5)
constexpr double SKILLS_TALL_MAX_SPEED = 50.0;     // top drive speed whenever the lift is tall (out of 127)
constexpr double SKILLS_MIN_DRIVE_SPEED = 15.0;   // never slower than this while still driving (was 18)
constexpr double SKILLS_HEADING_KP = 2.0;     // keeps the robot pointed straight while driving (per degree off)
constexpr int SKILLS_DRIVE_SPEED = 120;       // top speed for these drives (out of 127)
constexpr int SKILLS_TALL_TURN_SPEED = 50;    // top turning speed while the lift is tall (out of 127) (was 70)
constexpr int SKILLS_TURN_SPEED = 90;         // top turning speed with the lift low (was full 127: the fast turn + hand-off is what flung the preload)
constexpr int SKILLS_MATCHLOAD_TURN_SPEED = 110;  // turning toward a goal with a matchload (pin + cup) and the lift low (was 90 like the preload)
constexpr int SKILLS_EMPTY_TURN_SPEED = 127;  // turns with the claw empty and the lift down (toward the loader, the route to the sweep):
                                              // nothing to fling
// While turning toward a goal, the lift only goes up to this height; the rest happens while driving straight in
constexpr double SKILLS_TURN_LIFT_MAX = 370.0;   // most the lift is raised to while turning (just under SKILLS_TALL_LIFT_DEG, so turns stay
                                                 // fast); the rest of the way up happens while driving to the goal. (was 140)
// =====================================================================

// Speed the robot is still rolling at when one skills move hands off to the next (+ forward, - backward)
double skillsRollSpeed = 0.0;

// Lift grounding state (see AUTON_GROUND_PUSH_MV)
bool autonLiftGrounding = false;
uint32_t autonGroundStartMs = 0;

// Coast lowering state (see AUTON_LIFT_COAST_DIST)
bool autonLiftCoasting = false;   // true while lowering and waiting to reach the coast point
double autonCoastAtDeg = 0.0;     // lift height where the motors switch to coast

// Auton claw timers (0 = nothing scheduled)
uint32_t autonClawSlowAtMs = 0;   // when the claw drops from full speed to the slow grabbing speed
uint32_t autonClawStopAtMs = 0;   // when the claw stops spinning backward after opening

// Runs the claw timers. Called every 10 ms while auton waits on anything.
// ---------------------------------------------------------------------
// PIN CHECK: is the claw really holding the pin it just picked up?
// ---------------------------------------------------------------------
// Holding a pin, the claw stalls or creeps. If it stays faster than SKILLS_PIN_HELD_RPM (never dropping below it) for
// SKILLS_PIN_LOST_MS while it should be holding (hold speed 100 rpm), there's nothing in it: the pickup missed. Only checked while
// holding (from the end of a pickup until the claw opens): opening stalls the claw too.
constexpr double SKILLS_PIN_HELD_RPM = 50.0;      // (was 10: every pin and cup set it off, the claw creeps while it squeezes)
constexpr uint32_t SKILLS_PIN_LOST_MS = 1000;     // (was 500)
bool skillsPinWatch = false;       // the claw should be holding a pin right now
uint32_t skillsPinSpinSince = 0;   // when it started spinning freely (0 = stalled, holding)
bool skillsPinLost = false;        // the check decided the claw is empty
bool skillsAbortOnLost = false;    // carrying to a goal: drives and turns stop as soon as skillsPinLost is set
int skillsPickupCount = 0;         // pickups so far (1, 2 = field pins), for the simulator's pretend misses
uint32_t skillsPinWatchStart = 0;  // when the hold check started (the end of the pickup)

#ifdef RW_SIM_PROS
// Simulator only: is this pickup number in the list in environment variable `name` (e.g. SIM_MISS=1,2)?
bool skillsSimListed(const char* name) {
    const char* v = std::getenv(name);
    const std::string list = std::string(",") + (v ? v : "") + ",";
    return list.find("," + std::to_string(skillsPickupCount) + ",") != std::string::npos;
}
#endif

bool skillsClawHasPin() {
#ifdef RW_SIM_PROS
    // The simulator's claw never stalls (there's nothing to grab), so it pretends every pickup worked, except:
    //   SIM_MISS = pickup numbers that grab nothing (e.g. SIM_MISS=1,2 for both field pins): caught during the pickup
    //   SIM_DROP = pickup numbers that grab the pin but lose it 0.3 s later, on the way out (caught by this hold check)
    if (skillsSimListed("SIM_MISS")) return false;
    return !(skillsSimListed("SIM_DROP") && pros::millis() - skillsPinWatchStart > 300);
#else
    return std::fabs(claw.get_actual_velocity()) <= SKILLS_PIN_HELD_RPM;
#endif
}

// Starts / stops the pin check (the end of a pickup / the claw opening)
void skillsWatchPin(bool on) {
    skillsPinWatch = on;
    skillsPinSpinSince = 0;
    if (on) {
        skillsPinLost = false;
        skillsPinWatchStart = pros::millis();
    }
}

void autonTick() {
    uint32_t now = pros::millis();

    if (skillsPinWatch && !skillsPinLost) {
        if (skillsClawHasPin()) skillsPinSpinSince = 0;
        else if (skillsPinSpinSince == 0) skillsPinSpinSince = now;
        else if (now - skillsPinSpinSince >= SKILLS_PIN_LOST_MS) skillsPinLost = true;
    }

    if (autonClawSlowAtMs != 0 && now >= autonClawSlowAtMs) {
        claw.move_velocity(AUTON_CLAW_HOLD_SPEED);
        autonClawSlowAtMs = 0;
    }

    if (autonClawStopAtMs != 0 && now >= autonClawStopAtMs) {
        claw.move_velocity(0);
        autonClawStopAtMs = 0;
    }

    // Lift grounding: push down hard until at the bottom (or stopped on the hard stop), then hold lightly
    if (autonLiftGrounding) {
        double pos = getDR4BPosition();
        double speed = std::fabs(dr4bRotation.get_velocity() / 100.0);   // degrees per second
        bool atBottom = pos <= AUTON_GROUND_DONE_DEG
                        || (pos < DR4B_BOTTOM_DEADZONE && now - autonGroundStartMs > 250 && speed < 20.0);
        DR4B.move_voltage(atBottom ? -AUTON_GROUND_HOLD_MV : -AUTON_GROUND_PUSH_MV);
    }

    // Coast lowering: once the lift is down to the coast point, cut the power and let it coast the rest of the way
    if (autonLiftCoasting && getDR4BPosition() <= autonCoastAtDeg) {
        autonLiftCoasting = false;
        dr4bSettling = false;
        dr4bManualMode = true;   // keeps the lift PID from slowing it down
        DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        DR4B.move_voltage(0);
    }
}

// Starts pushing the lift down to 0. Keeps going in the background (autonTick) until the claw grabs.
void autonStartGrounding() {
    autonLiftCoasting = false;
    dr4bSettling = false;
    dr4bManualMode = true;   // keeps the lift PID out of the way
    dr4bTargetForced = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    DR4B.move_voltage(-AUTON_GROUND_PUSH_MV);
    autonGroundStartMs = pros::millis();
    autonLiftGrounding = true;
}

// Waits for the current drive motion to finish
void autonWaitMotion() {
    while (chassis.isInMotion()) {
        if (skillsAbortOnLost && skillsPinLost) {   // carrying a pin that isn't there: stop right away
            chassis.cancelMotion();
            break;
        }
        autonTick();
        pros::delay(10);
    }
}

// Sends the lift to a height (the DR4B PID task does the moving)
void autonSetLift(double degrees) {
    autonLiftCoasting = false;   // a new lift command cancels any coast lowering
    dr4bTarget = degrees;
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

// Lowers the lift toward a height with the PID, then switches to coast AUTON_LIFT_COAST_DIST degrees above it
// so momentum carries it the rest of the way (the switch happens in autonTick). Doesn't wait.
void autonLowerLiftCoast(double degrees) {
    autonSetLift(degrees);
    if (AUTON_LIFT_COAST_DIST > 0.0) {
        autonCoastAtDeg = degrees + AUTON_LIFT_COAST_DIST;
        autonLiftCoasting = true;
    }
}

// Waits until the lift reaches a height (or the timeout runs out)
void autonWaitLift(double target, uint32_t timeoutMs) {
    uint32_t start = pros::millis();
    while (std::fabs(getDR4BPosition() - target) > AUTON_LIFT_TOLERANCE
           && pros::millis() - start < timeoutMs) {
        autonTick();
        pros::delay(10);
    }
}

// Opens the claw (spins backward for openMs, then stops). Doesn't wait.
void autonOpenClaw(int openMs = AUTON_CLAW_OPEN_MS) {
    skillsWatchPin(false);   // opening: not holding anything any more (and opening stalls the claw too)
    autonClawSlowAtMs = 0;
    claw.move_velocity(-200);
    autonClawStopAtMs = pros::millis() + openMs;
}

// Closes the claw at full speed, waits for the stall (it has the pin), then does the
// same small lift as the driver pin pickup. The robot is stopped the whole time.
// extraSpinMs: the claw keeps spinning at full speed this much longer before lifting (0 = lift as soon as it grabs)
void autonCloseClawAndLift(int extraSpinMs) {
    uint32_t start = pros::millis();

    autonClawStopAtMs = 0;
    claw.move_velocity(200);
    autonClawSlowAtMs = start + AUTON_CLAW_FAST_MS + extraSpinMs;   // drops to the slow speed 1 s after the extra spin (during the turn)

    // Same stall check as driver control
    bool spunUp = false;
    bool slow = false;
    uint32_t slowStart = 0;

    while (pros::millis() - start < AUTON_GRAB_TIMEOUT_MS) {
        uint32_t now = pros::millis();
        double clawSpeed = std::fabs(claw.get_actual_velocity());

        if (clawSpeed >= CLAW_SPINUP_SPEED || now - start >= CLAW_STALL_GRACE_MS) {
            spunUp = true;
        }

        if (spunUp && clawSpeed < CLAW_STALL_SPEED) {
            if (!slow) {
                slow = true;
                slowStart = now;
            }
            if (now - slowStart >= CLAW_STALL_CONFIRM_MS) {
                break;   // grabbed
            }
        } else {
            slow = false;
        }

        autonTick();
        pros::delay(10);
    }

    // Extra spin at full speed before lifting (robot still stopped, lift still held down)
    uint32_t extraStart = pros::millis();
    while (pros::millis() - extraStart < static_cast<uint32_t>(extraSpinMs)) {
        autonTick();
        pros::delay(10);
    }

    // Stop holding the lift down; the small lift takes over from here
    autonLiftGrounding = false;

    // Small lift: goes up AUTON_PIN_PICKUP_RISE degrees from where it grabbed the pin
    double liftGoal = getDR4BPosition() + AUTON_PIN_PICKUP_RISE;
    pinLiftCount++;

    dr4bSettling = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.move_voltage(PIN_LIFT_VOLTAGE);

    uint32_t liftStart = pros::millis();
    while (getDR4BPosition() < liftGoal && pros::millis() - liftStart < PIN_LIFT_TIMEOUT_MS) {
        autonTick();
        pros::delay(10);
    }

    dr4bTarget = getDR4BPosition();
    dr4bTargetForced = true;   // hold it here even if it's inside the bottom deadzone
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.brake();
    dr4bManualMode = false;
}

// Drives straight forward along `heading` with both sides at the same speed (no turning at all)
// until it has gone far enough to reach (targetX, targetY), then stops.
void autonDriveStraight(double targetX, double targetY, double heading, int speed, uint32_t timeoutMs) {
    Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    double total = (targetX - start.x) * hx + (targetY - start.y) * hy;   // distance to go along the heading

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        Pose p = chassis.getPose();
        double done = (p.x - start.x) * hx + (p.y - start.y) * hy;
        if (done >= total - 0.25) break;
        chassis.tank(speed, speed, true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

// Drives to a pin + cup, coming in along approachHeading, and grabs it
// turnOffset: + = turns that many degrees more right before the slow drive in, - = more left
// extraDist:  + = drives that many inches further forward into the pin, - = stops shorter
// approachSpeed: top speed driving up to the pin. slowSpeed: speed for the last slowDist inches into it
void autonGrabPin(double pinX, double pinY, double approachHeading, double turnOffset, double extraDist,
                  double slowDist, int approachSpeed, int slowSpeed, int extraClawMs) {
    double rad = approachHeading * PI / 180.0;
    double ux = std::sin(rad);
    double uy = std::cos(rad);

    // Where the slow part starts (same spot as before, lined up with the pin)
    double slowX = pinX - (AUTON_CLAW_REACH + AUTON_SLOW_DIST) * ux;
    double slowY = pinY - (AUTON_CLAW_REACH + AUTON_SLOW_DIST) * uy;

    // Heading for the slow drive in, with the turn offset added
    double grabHeading = approachHeading + turnOffset;
    double gx = std::sin(grabHeading * PI / 180.0);
    double gy = std::cos(grabHeading * PI / 180.0);

    // Where the robot's center ends up: straight ahead from the slow spot, plus extraDist
    double grabX = slowX + (AUTON_SLOW_DIST + extraDist) * gx;
    double grabY = slowY + (AUTON_SLOW_DIST + extraDist) * gy;

    // The fast drive covers the extra distance too, so the slow part is always just the last slowDist inches
    slowX = grabX - slowDist * gx;
    slowY = grabY - slowDist * gy;

    // Lift goes all the way down to 0 while driving to the pin
    autonStartGrounding();

    // Face the approach point, then drive there and line up with the pin
    chassis.turnToPoint(slowX, slowY, 700,
                        {.minSpeed = AUTON_TURN_MIN_SPEED, .earlyExitRange = AUTON_TURN_EXIT_DEG});
    autonWaitMotion();

    // Fast drive to slowDist before the grab spot. Comes to a real stop there
    // (no rolling hand-off) so it can't coast into the pin before the slow part starts.
    chassis.moveToPose(slowX, slowY, grabHeading, 2500, {.lead = 0.3, .maxSpeed = static_cast<float>(approachSpeed)});
    // Start spinning the claw about 5" before reaching the pin (instead of opening it)
    bool clawSpinning = false;
    while (chassis.isInMotion()) {
        if (!clawSpinning
            && chassis.getPose().distance(Pose(grabX, grabY)) <= AUTON_CLAW_OPEN_DIST) {
            autonClawStopAtMs = 0;   // cancel any leftover open/slow timers so nothing stops the spin
            autonClawSlowAtMs = 0;
            claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
            clawSpinning = true;
        }
        autonTick();
        pros::delay(10);
    }
    if (!clawSpinning) {
        autonClawStopAtMs = 0;
        autonClawSlowAtMs = 0;
        claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
    }

    // Lift has to be all the way down before going in (only waits if it isn't there yet)
    {
        uint32_t groundWait = pros::millis();
        while (getDR4BPosition() > AUTON_GROUND_DONE_DEG + 2.0 && pros::millis() - groundWait < 500) {
            autonTick();
            pros::delay(10);
        }
    }

    // Square up exactly to the grab heading (the drive above hands off early, so it may not be fully turned yet)
    chassis.turnToHeading(grabHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Last few inches: slow and dead straight (both sides same speed, no turning) with the claw spinning
    autonDriveStraight(grabX, grabY, grabHeading, slowSpeed, 1500);

    // Stopped: close, wait for the grab, small lift
    autonCloseClawAndLift(extraClawMs);
}

// Drives straight along `heading` for `distance` inches (+ forward, - backward) with a gentle speed-up
// and slow-down. Speeds up/slows down even more gently whenever the lift is tall, so the robot doesn't tip.
// Keeps the robot pointed at `heading` the whole way (no wandering).
void skillsRampDrive(double distance, double heading, int maxSpeed, uint32_t timeoutMs) {
    Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    int dir = distance >= 0 ? 1 : -1;
    double target = std::fabs(distance);
    double speed = 0.0;

    // Ends once the robot has stopped moving (under 0.15" in 150 ms, after the first 300 ms), instead of holding
    // until the timeout (the park sat still for 0.8 s at the end that way)
    Pose stallPose = start;
    uint32_t stallMark = pros::millis();
    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        Pose p = chassis.getPose();
        double remaining = target - ((p.x - start.x) * hx + (p.y - start.y) * hy) * dir;
        if (remaining <= 0.3) break;
        if (pros::millis() - stallMark >= 150) {
            bool moved = std::hypot(p.x - stallPose.x, p.y - stallPose.y) >= 0.15;
            stallMark = pros::millis();
            stallPose = p;
            if (!moved && pros::millis() - t0 >= 300) break;
        }

        bool tall = getDR4BPosition() > SKILLS_TALL_LIFT_DEG;
        speed = std::min(speed + (tall ? SKILLS_ACCEL_TALL : SKILLS_ACCEL), static_cast<double>(maxSpeed));
        speed = std::min(speed, SKILLS_MIN_DRIVE_SPEED + (tall ? SKILLS_DECEL_TALL : SKILLS_DECEL) * remaining);

        double err = std::remainder(heading - p.theta, 360.0);   // degrees off, + = needs to turn right
        double corr = std::clamp(SKILLS_HEADING_KP * err, -20.0, 20.0);
        double base = dir * speed;
        chassis.tank(static_cast<int>(base + corr), static_cast<int>(base - corr), true);

        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    skillsRollSpeed = 0.0;
}

// Turns to the alliance goal, lifts while driving to it, scores, then lowers and backs off (skills version:
// gentle acceleration with the lift up, slower turns with the lift tall, lift waits until it's up before going over)
// stackPosition = which pin/cup on the goal (1 = first, 2 = second...), uses ALLIANCE_HEIGHTS
// turnLessDeg: + = turns that many degrees less than straight at the goal, - = more
// extraDist:   + = drives that many inches further toward the goal, - = stops shorter
void autonScore(int stackPosition, double turnLessDeg, double extraDist) {
    double height = getStackHeight(GOAL_ALLIANCE, stackPosition);

    // Only part of the way up while turning (a tall lift + a fast turn is what tipped it)
    autonSetLift(std::min(height, SKILLS_TURN_LIFT_MAX));

    Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(autonGoalX - startPose.x, autonGoalY - startPose.y) * 180.0 / PI;
    double turn = std::remainder(headingToGoal - startPose.theta, 360.0);   // -180 to 180
    turn -= (turn >= 0 ? turnLessDeg : -turnLessDeg);
    double aimHeading = startPose.theta + turn;

    int turnSpeed = getDR4BPosition() > SKILLS_TALL_LIFT_DEG ? SKILLS_TALL_TURN_SPEED : 127;
    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS,
                          {.maxSpeed = turnSpeed, .minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    Pose pose = chassis.getPose();
    double dist = std::hypot(autonGoalX - pose.x, autonGoalY - pose.y);
    double travel = dist - AUTON_CLAW_REACH + extraDist;   // how far the robot center goes to put the claw over the goal

    // Rest of the way up while driving in (gentle start since the lift is getting tall)
    autonSetLift(height);
    if (travel > AUTON_PIN_PRE_SCORE_GAP + 0.5) {
        skillsRampDrive(travel - AUTON_PIN_PRE_SCORE_GAP, aimHeading, SKILLS_DRIVE_SPEED, 2500);
    }

    // Lift must be all the way up before the last bit over the goal
    autonWaitLift(height, 1500);
    {
        Pose now = chassis.getPose();
        double ux = std::sin(aimHeading * PI / 180.0), uy = std::cos(aimHeading * PI / 180.0);
        double left = travel - ((now.x - pose.x) * ux + (now.y - pose.y) * uy);
        if (left > 0.3) skillsRampDrive(left, aimHeading, 50, 1200);
    }

    // Lower the lift and open the claw at the same time
    autonLowerLiftCoast(AUTON_LOW_HEIGHT);
    autonOpenClaw();

    // Halfway down: back up (gently, the lift is still high)
    double halfway = (height + AUTON_LOW_HEIGHT) / 2.0;
    uint32_t lowerStart = pros::millis();
    while (getDR4BPosition() > halfway && pros::millis() - lowerStart < 600) {
        autonTick();
        pros::delay(10);
    }
    skillsRampDrive(-(AUTON_BACKUP_DIST + AUTON_CLEAR_DIST), aimHeading, SKILLS_DRIVE_SPEED, 1500);
}

// Drives the preload to the alliance goal, lowers it onto the goal, lets go and backs off
// turnLessDeg / extraDist: same as AUTON_PRELOAD_TURN_LESS_DEG / AUTON_PRELOAD_EXTRA_DIST (or the TOP_ versions)
void autonScorePreload(double turnLessDeg, double extraDist) {
    // Turn toward the goal, but turnLessDeg less than straight at it
    Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(autonGoalX - startPose.x, autonGoalY - startPose.y) * 180.0 / PI;
    double turn = std::remainder(headingToGoal - startPose.theta, 360.0);   // -180 to 180
    if (std::fabs(turn) > turnLessDeg) {
        turn -= (turn > 0 ? turnLessDeg : -turnLessDeg);
    } else {
        turn = 0.0;
    }
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along that heading, stopping when the claw is as far out as the goal
    Pose pose = chassis.getPose();
    double dist = std::hypot(autonGoalX - pose.x, autonGoalY - pose.y);
    double ux = std::sin(aimHeading * PI / 180.0);
    double uy = std::cos(aimHeading * PI / 180.0);

    // Robot center at the drop spot (adjusted by extraDist)
    double travel = dist - AUTON_CLAW_REACH + extraDist;
    double scoreX = pose.x + travel * ux;
    double scoreY = pose.y + travel * uy;

    // (The lift is already going up to the carry height: twoPinAlliance starts it right after the toggle)

    // Drive most of the way at full speed, the last few inches slower (no stop in between)
    if (travel > AUTON_PRE_SCORE_GAP + 0.5) {
        chassis.moveToPoint(scoreX - AUTON_PRE_SCORE_GAP * ux, scoreY - AUTON_PRE_SCORE_GAP * uy, 1000,
                            {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = AUTON_CHAIN_EXIT_DIST});
        autonWaitMotion();
    }

    // Lift should be at the carry height before going over the goal (only waits if it isn't there yet)
    if (AUTON_WAIT_FOR_LIFT_AT_GOAL) {
        autonWaitLift(AUTON_PRELOAD_CARRY_HEIGHT, 600);
    }

    chassis.moveToPose(scoreX, scoreY, aimHeading, 900,   // holds the aim heading
                       {.lead = 0.1, .maxSpeed = AUTON_SCORE_SPEED,
                        .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_STOP_EXIT_DIST});
    autonWaitMotion();

    // Lower onto the goal, then open the claw
    autonSetLift(AUTON_PRELOAD_DROP_HEIGHT);
    autonWaitLift(AUTON_PRELOAD_DROP_HEIGHT, 700);

    autonOpenClaw(AUTON_PRELOAD_CLAW_OPEN_MS);   // opens a little longer for the preload
    while (autonClawStopAtMs != 0) {   // wait until it has finished opening
        autonTick();
        pros::delay(10);
    }

    // Back straight away from the goal
    chassis.moveToPoint(scoreX - AUTON_PRELOAD_BACKUP * ux, scoreY - AUTON_PRELOAD_BACKUP * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();

    // Back up to grab height for the pins (happens while it drives to pin 1)
    autonLowerLiftCoast(AUTON_LOW_HEIGHT);   // coasts the last AUTON_LIFT_COAST_DIST degrees
}

// Runs the preload + 2 pins routine for one side (the right-wall or top-wall route)
void runThreePinRoute(const AutonRoute& r) {
    autonGoalX = r.goalX;
    autonGoalY = r.goalY;

    chassis.setPose(r.startX, r.startY, r.startHeading);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);   // driver control sets it back to coast

    autonClawSlowAtMs = 0;
    autonClawStopAtMs = 0;

    // 1. Claw at the slow grabbing speed
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);

    // 2. Lift up 20 degrees so the toggles clear, then spin the toggle exactly 2 turns.
    //    move_absolute stops the motor right at the target, so it can't go past 2 turns.
    autonSetLift(AUTON_TOGGLE_LIFT_HEIGHT);
    {
        // Start the toggle as soon as the lift is high enough, instead of waiting for the full lift
        uint32_t liftStart = pros::millis();
        while (getDR4BPosition() < AUTON_TOGGLE_START_LIFT && pros::millis() - liftStart < 1000) {
            autonTick();
            pros::delay(10);
        }
    }

    double toggleTargetDeg = AUTON_TOGGLE_ROTATIONS * AUTON_TOGGLE_GEAR_RATIO * 180.0;   // 2 * 2 * 180 = 720 motor degrees
    toggleMechanism.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    toggleMechanism.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    toggleMechanism.tare_position();
    toggleMechanism.move_absolute(toggleTargetDeg, AUTON_TOGGLE_SPEED);

    uint32_t toggleStart = pros::millis();
    while (std::fabs(toggleMechanism.get_position() - toggleTargetDeg) > AUTON_TOGGLE_DONE_DEG
           && pros::millis() - toggleStart < AUTON_TOGGLE_TIMEOUT_MS) {
        autonTick();
        pros::delay(10);
    }
    // No stop command here: move_absolute keeps going and stops itself exactly at 2 turns
    // while the robot starts turning toward the goal. (If it got stuck and timed out, stop it.)
    if (pros::millis() - toggleStart >= AUTON_TOGGLE_TIMEOUT_MS) {
        toggleMechanism.move_velocity(0);
    }

    // Lift goes straight from 30 up to the preload carry height now, while the robot turns
    // toward the goal (it used to wait until the turn was finished)
    autonSetLift(AUTON_PRELOAD_CARRY_HEIGHT);

    // Lift to grab height (AUTON_LOW_HEIGHT) for the preload drive
    // autonSetLift(AUTON_LOW_HEIGHT);

    // 3. Preload -> goal (lower to 10, open claw, back off)
    autonScorePreload(r.preloadTurnLess, r.preloadExtraDist);

    // 4. Pin + cup 1 -> goal
    autonGrabPin(r.pin1X, r.pin1Y, r.pin1Heading,
                 r.pin1GrabTurn, r.pin1GrabExtra, AUTON_PIN1_SLOW_DIST,
                 AUTON_PIN1_APPROACH_SPEED, AUTON_PIN1_SLOW_SPEED, 0);
    autonScore(AUTON_PIN1_STACK, r.pin1ScoreTurnLess, r.pin1ScoreExtra);

    // 5. Pin + cup 2 -> goal
    autonGrabPin(r.pin2X, r.pin2Y, r.pin2Heading,
                 r.pin2GrabTurn, r.pin2GrabExtra, AUTON_PIN2_SLOW_DIST,
                 127, AUTON_SLOW_SPEED, AUTON_PIN2_EXTRA_CLAW_MS);   // pin 2: full speed in, same slow speed as before, longer claw spin
    autonScore(AUTON_PIN2_STACK, r.pin2ScoreTurnLess, r.pin2ScoreExtra);

    claw.move_velocity(0);
}

// Right-wall start (the original route, exactly the same values as before)
const AutonRoute RIGHT_ROUTE = {
    AUTON_START_X, AUTON_START_Y, AUTON_START_HEADING,
    AUTON_GOAL_X, AUTON_GOAL_Y,
    AUTON_PRELOAD_TURN_LESS_DEG, AUTON_PRELOAD_EXTRA_DIST,
    AUTON_PIN1_X, AUTON_PIN1_Y, AUTON_PIN1_APPROACH_HEADING,
    AUTON_PIN1_GRAB_TURN_OFFSET, AUTON_PIN1_GRAB_EXTRA_DIST, AUTON_PIN1_SCORE_TURN_LESS, AUTON_PIN1_SCORE_EXTRA_DIST,
    AUTON_PIN2_X, AUTON_PIN2_Y, AUTON_PIN2_APPROACH_HEADING,
    AUTON_PIN2_GRAB_TURN_OFFSET, AUTON_PIN2_GRAB_EXTRA_DIST, AUTON_PIN2_SCORE_TURN_LESS, AUTON_PIN2_SCORE_EXTRA_DIST
};

// Top-wall start (mirror route)
const AutonRoute TOP_ROUTE = {
    TOP_START_X, TOP_START_Y, TOP_START_HEADING,
    TOP_GOAL_X, TOP_GOAL_Y,
    TOP_PRELOAD_TURN_LESS_DEG, TOP_PRELOAD_EXTRA_DIST,
    TOP_PIN1_X, TOP_PIN1_Y, TOP_PIN1_APPROACH_HEADING,
    TOP_PIN1_GRAB_TURN_OFFSET, TOP_PIN1_GRAB_EXTRA_DIST, TOP_PIN1_SCORE_TURN_LESS, TOP_PIN1_SCORE_EXTRA_DIST,
    TOP_PIN2_X, TOP_PIN2_Y, TOP_PIN2_APPROACH_HEADING,
    TOP_PIN2_GRAB_TURN_OFFSET, TOP_PIN2_GRAB_EXTRA_DIST, TOP_PIN2_SCORE_TURN_LESS, TOP_PIN2_SCORE_EXTRA_DIST
};


// =====================================================================
//                    SKILLS (1 minute) - tuning
// =====================================================================
// Start: back of the robot against the LEFT wall (red side), in the middle of the wall, claw facing right.
// The opening is the right-wall qual route turned 180 degrees around the field center: every spot becomes
// (-x, -y) and every heading +180. Turning around keeps left/right the same, so ALL the right-wall tuning
// (turn offsets, distance offsets) carries over unchanged.
//
// Red goal A = (-47.09, -23.55): preload, both field pins, then SKILLS_A_LOADS matchloads.
// Red goal B = (-23.55, -47.09): the goal in the bottom quadrant. Matchloads (its first one goes on spot 1), as many as
// the time guard allows. Every pickup is the same fixed SKILLS_PICKUP_MS, and nothing ever waits for the lift.

// --- Matchloader (bottom of the left wall) ---
// Field element spots come from RW-Template's Override field (the same field its simulator draws, so the
// replay shows exactly where the robot is compared to them).
// The robot NEVER touches the loader: it drives straight in along the loader line and stops with the claw
// SKILLS_LOADER_GAP short of the loader's face. No pushing, no "seat" by touch (touching it is what knocked the position off).
//
//   turn spot (T) ---one drive, no stops: steers down onto the loader line---> line-up spot (P) ===square===> grab
//   T: far enough from the bottom wall and goal B that it can turn in place without any corner touching.
//   P: on the loader line. The drive steers onto the line before P (at most SKILLS_LINEUP_MIN_HEADING, about
//      18 deg off), so from P in it is square to the loader. It used to stop at P and turn; now it never stops.
constexpr double SKILLS_LOADER_Y = -58.7;                                         // loader center (RW field: +-58.7)
// Sideways offset for the line it drives in on: + = stops more to the ROBOT'S RIGHT as it faces the loader (+Y here).
// On the robot it stopped 1-2" too far right, so it was -1.5. That put the robot's side only 1.66" off the bottom wall,
// and every trip in and out of the loader brought a corner 0.7-1" from the wall (simulator). -0.75: the corners stay
// 1.5"+ off the wall, and it stops 0.75" further right than with -1.5. If the claw now misses the loader pin to the
// right, go back toward -1.5 (and SKILLS_LOADER_EXIT_WALL_SPARE back toward 0.75).
constexpr double SKILLS_LOADER_RIGHT_ADJUST = -0.75;
constexpr double SKILLS_LOADER_LINE_Y = SKILLS_LOADER_Y + SKILLS_LOADER_RIGHT_ADJUST;   // -59.45: the line it drives in on
constexpr double SKILLS_LOADER_FACE_X = -FIELD_HALF + 3.75;                       // -66.5: front face of the loader (sticks out 3.75" from the wall)
constexpr double SKILLS_LOADER_GAP = 1.5;          // the FRONT of the robot stops this far from the loader face (was 1.0: in the simulator it
                                                   // came within 0.4" of the loader with 1.0). Raise it if
                                                   // the claw still touches the loader, lower it if it can't reach the pin.
// The FRONT of the robot (not the claw) is what has to stay off the loader: the claw is 1.55" behind it,
// so the claw ends SKILLS_LOADER_GAP + 1.55" from the loader face.
constexpr double SKILLS_LOADER_EXTRA_DIST = 1.5;   // + = drives this much further into the loader than SKILLS_LOADER_GAP (robot: 1.5" more
                                                   // was better). Note: 1.5 with a 1.5 gap puts the front right at the loader face
constexpr double SKILLS_LOADER_GRAB_X = SKILLS_LOADER_FACE_X + SKILLS_LOADER_GAP + ROBOT_CENTER_TO_FRONT - SKILLS_LOADER_EXTRA_DIST;   // -56.95: robot center when loading
constexpr double SKILLS_LOADER_RUN_IN = 7.5;       // inches of dead-straight drive along the loader line into the loader
constexpr double SKILLS_LOADER_LINEUP_X = SKILLS_LOADER_GRAB_X + SKILLS_LOADER_RUN_IN;   // -49.95 (spot P)
// (SKILLS_LOADER_TURN_X/Y, spot T, are worked out below, after goal B)
constexpr double SKILLS_LOADER_ROLL = 0.6;         // the robot rolls about this far after it stops driving into the loader, so it stops this early
constexpr int SKILLS_LOADER_DRIVE_IN_SPEED = 55;   // (was 80: slower so it stops in the same spot every time)   // top speed once it's on the loader line (P on in); slows down on its own at the end
constexpr int SKILLS_LINEUP_SPEED = 90;            // top speed from T to P
constexpr double SKILLS_LINEUP_MIN_HEADING = 252.0; // steepest heading allowed from T to P (a steeper one lets the corner get close to the bottom wall)
constexpr double SKILLS_LINEUP_STEER = 6.0;        // degrees of heading toward the loader line per inch off it (T to P)
constexpr double SKILLS_PRECISE_TOL = 0.6;          // turn spot T: it fixes any slide past it bigger than this (inches) before turning
constexpr int SKILLS_PRECISE_SPEED = 35;
constexpr double SKILLS_LOADER_EXIT_STRAIGHT = 2.5;  // leaving the loader: dead straight back this far before it starts climbing away
// Leaving the loader it can let the corners get this close (square on the loader line its side is 2.41" off the wall,
// so with SKILLS_LOADER_WALL_SPARE it could barely start climbing back to T)
constexpr double SKILLS_LOADER_EXIT_WALL_SPARE = 1.5;    // (0.75 with the -1.5 loader line, where 1.0 reached T up to 2" low and
                                                         // turning there clipped the bottom wall. With the -0.75 line there's
                                                         // room for 1.5, and it still reaches T's height)
// ...and eases off the climb over the last inches: at most this many degrees per inch still to climb. Climbing at up to 40
// degrees right to the end, it overshot T's height by about 2" (in the simulator), so the turn at T came within 1.6" of
// goal B and the next trip in started too high and reached the loader up to 1.4" off the loader line.
constexpr double SKILLS_LOADER_EXIT_STEER = 12.0;
constexpr double SKILLS_LOADER_WALL_SPARE = 1.75;   // driving next to the loader, the corners always stay at least this far off the bottom wall
                                                   // (turns in place there keep SKILLS_LOADER_TURN_SPARE)
constexpr int SKILLS_LOADER_TURNING_SPEED = 45;     // near the loader: most speed while the heading is still 5+ degrees off what it should be
constexpr int SKILLS_LOADER_CLIMB_SPEED = 60;       // leaving the loader: most speed while it climbs away from the wall (was 45)
constexpr double SKILLS_LOADER_STEER_KP = 2.5;      // steering next to the loader (stronger than SKILLS_STEER_KP, so it turns quickly)
constexpr double SKILLS_LOADER_STEER_MAX = 50.0;
// --- Every pickup (matchloads, the first goal B load, both field pins) ---
// The robot stands still for exactly SKILLS_PICKUP_MS: the claw spins at full speed the whole time, and in the last
// SKILLS_PICKUP_LIFT_MS the lift kicks up (15 deg at the loader, 25 deg for a field pin). Then it drives off.
// It does NOT check the claw for a stall and does NOT wait for the lift: it's a fixed time, every time.
// (The old "3-4 s at the loader" was a hidden timer: the drive into the loader kept creeping for up to 2.5 s whenever
// odometry said it wasn't quite there yet. That drive now gives up as soon as the robot stops moving, see
// SKILLS_STALL_MS, and has a short time limit.)
constexpr int SKILLS_PICKUP_MS = 1200;             // total time stopped for a pickup (was 1500: 7 pickups, 2.1 s back for the second toggle)
constexpr int SKILLS_PICKUP_LIFT_MS = 150;         // the last part of it, with the lift kicking up
constexpr double SKILLS_LOAD_RISE = 15.0;          // degrees the lift goes up at the loader before it moves or turns
constexpr double SKILLS_FIELD_PIN_RISE = AUTON_PIN_PICKUP_RISE;   // 25 degrees for a field pin (off the floor)
// Pin check DURING the pickup: the claw spins at full speed (200 rpm) the whole pickup. With a pin + cup in it the flex
// wheels jam on it and the claw nearly stops; with nothing in it, it spins freely near 200 rpm. If it was faster than
// SKILLS_PICKUP_FREE_RPM for at least SKILLS_PICKUP_FREE_SHARE of the readings from SKILLS_PICKUP_CHECK_FROM_MS until the
// lift kick, the pickup missed: the lift does NOT kick up, and the robot skips scoring right there (no lift going up and
// coming back down next to a wall, no carrying nothing to the goal). The hold check (SKILLS_PIN_HELD_RPM) still catches a
// pin that falls out later.
constexpr double SKILLS_PICKUP_FREE_RPM = 140.0;
constexpr int SKILLS_PICKUP_CHECK_FROM_MS = 600;
constexpr double SKILLS_PICKUP_FREE_SHARE = 0.8;
// Dropping a pin it lost on the way to a goal (the hold check), with the lift up: the DR4B swings out past the frame
// while it's partly up, and next to a wall it hangs over the wall and hooks it on the way down. So if any corner is
// closer than SKILLS_DROP_WALL_CLEAR to a wall, it first drives toward the goal (open field) until it isn't, then opens
// the claw and lowers. It then waits a fixed SKILLS_DROP_LOWER_MS (the lift coming down) before driving anywhere, so it
// never backs toward a wall with the lift still up (that's what put it into the side of the field after missing pin 2).
constexpr double SKILLS_DROP_LIFT_UP_DEG = 60.0;   // "lift up" = above this
constexpr double SKILLS_DROP_WALL_CLEAR = 8.0;
constexpr int SKILLS_DROP_MOVE_SPEED = 45;
constexpr int SKILLS_DROP_MOVE_MAX_MS = 1200;
constexpr int SKILLS_DROP_LOWER_MS = 600;
// Wall resets (fieldAwareReset) at every spot where the robot is standing still anyway: the start (before and after
// the toggle), each pin line-up and pickup, before big turns, while the claw lets go at a goal, the turn spot T, the
// loader, and before the sweep. A reading that says it's more than this far off is skipped (a reading that far off
// is something in the beam, not drift).
constexpr double SKILLS_RESET_JUMP = 3.0;
// After a missed matchload (the pin check), the next trip in clears the loader first. On the loader line itself only
// about 4 degrees of turn fit (the robot's side is only 1.7" off the bottom wall there), which was too small to move
// anything and still clipped the wall. So the clear happens one level up, at the turn spot's height (4.25" above the
// loader line, where the corners have room to swing):
//   from T it drives straight in (heading 270, no steering down) until the FRONT is SKILLS_LOADER_CLEAR_GAP from the
//   loader face, turns right as far as fits (about 24 degrees: the front sweeps fallen pins and cups in front of the
//   loader away toward the open field; the back-left corner stays SKILLS_LOADER_CLEAR_WALL_SPARE off the bottom wall and
//   the front corners SKILLS_LOADER_CLEAR_FACE_SPARE off the loader), turns back square, backs straight out to T and
//   drives in for the pickup the normal way (it needs that run-up to come down onto the loader line without the
//   corners touching the bottom wall).
constexpr double SKILLS_LOADER_CLEAR_GAP = 4.0;           // front of the robot to the loader face while it sweeps (3-4")
constexpr double SKILLS_LOADER_CLEAR_WALL_SPARE = 1.5;    // corners to the bottom wall during the sweep (was 1.0: it touched)
constexpr double SKILLS_LOADER_CLEAR_FACE_SPARE = 1.5;    // corners to the loader face / left wall during the sweep (it turns
                                                         // about 2 degrees past where it stops driving the turn)
constexpr double SKILLS_LOADER_CLEAR_COAST_DEG = 3.0;     // stops driving the turn this early (it keeps turning a little) (was 1.5)
constexpr int SKILLS_LOADER_CLEAR_TURN_SPEED = 70;        // the sweep (driven directly) (was 45: firmer push now)
constexpr double SKILLS_LOADER_CLEAR_MAX_DEG = 30.0;      // (was 20)
constexpr uint32_t SKILLS_CLEAR_EXTRA_MS = 2700;          // how much longer a trip with the clear takes (time guard)
bool skillsClearLoaderNext = false;   // set by a missed matchload
constexpr int SKILLS_LOADER_IN_MAX_MS = 1800;      // the drive from the turn spot into the loader never takes longer than this
// Any drive gives up once the robot has stopped moving (it's touching something or can't get closer): moved less
// than SKILLS_STALL_MOVE inches in SKILLS_STALL_MS, after its first 2 x SKILLS_STALL_MS. Pushing into things is what
// knocks the robot's position off; this way it never pushes.
constexpr int SKILLS_STALL_MS = 150;
constexpr double SKILLS_STALL_MOVE = 0.15;
// Before every big turn in place (more than SKILLS_SETTLE_TURN_DEG) it waits until it has really stopped (under
// SKILLS_SETTLE_SPEED in/s, at most SKILLS_SETTLE_MAX_MS). Turning while still rolling is what slid the robot about
// 2" past the turn spot next to the loader, so its corner came within 0.8" of the bottom wall.
constexpr double SKILLS_SETTLE_TURN_DEG = 45.0;
constexpr double SKILLS_SETTLE_SPEED = 2.0;
constexpr int SKILLS_SETTLE_MAX_MS = 250;

// --- Continuous driving (no stopping to aim or settle between moves) ---
// The only waits longer than a second are the start toggle and the pickups (SKILLS_PICKUP_MS).
constexpr int SKILLS_ROLL_SPEED = 70;              // speed kept when one move hands off to the next (out of 127) (was 50)
constexpr double SKILLS_AIM_SKIP_DEG = 20.0;       // smaller aim errors than this aren't turned first; the drive steers them out
constexpr double SKILLS_ROLL_TURN_EXIT_DEG = 10.0; // turns hand off to the drive this many degrees early (no settling)
constexpr int SKILLS_ROLL_TURN_MIN_SPEED = 40;
constexpr double SKILLS_STEER_KP = 1.5;            // steering per degree off while driving at a point
constexpr double SKILLS_STEER_MAX = 30.0;          // most steering added on top of the drive speed
constexpr double SKILLS_LIFT_DEG_PER_S = 600.0;   // how fast the lift climbs (main.cpp's DR4B_SLEW_UP): about 1 s to the top
constexpr double SKILLS_TALL_IN_PER_S = 25.0;     // about how fast the robot drives with the lift tall (SKILLS_TALL_MAX_SPEED)
constexpr double SKILLS_RAISE_SPARE = 2.0;        // inches added to the raise distance, so the lift is up just before the goal
// Scoring never waits for the lift (the lift reaches the top in about 1 s): it goes up to SKILLS_TURN_LIFT_MAX while
// the robot turns toward the goal, then all the way while it drives there. As soon as the claw is over the goal it
// lowers and lets go. The only speed limit from the lift is the one that was already there for a tall lift
// (above SKILLS_TALL_LIFT_DEG): same speeds as before.
// After the hold it always goes and scores (no going back to the loader if it missed: it acts as if it has the pin).
// Position at the loader: it stops by odometry (it never touches anything). While it holds there, the distance
// sensors check X / Y (localizeWithWalls, when use_distance_resets is on) and MCL keeps correcting while driving.
constexpr int SKILLS_RELEASE_MS = 250;             // matchloads: after the claw starts opening on a goal, this long before backing off.
                                                   // (100 was too short: the robot backed off while the claw still had the pin and
                                                   // dragged it off the goal. That's what knocked the preload off in the video.)
constexpr int SKILLS_PRELOAD_RELEASE_MS = 250;     // same for the preload (was 400)
// First preload only (the start): the lift lowers onto the goal slower so the preload doesn't fling off,
// and waits longer to get all the way down (90 degrees at 200 deg/s is 0.45 s). Everything else keeps the normal speed.
constexpr double SKILLS_FIRST_PRELOAD_SLEW_DOWN = 200.0;   // degrees/second (auton normally lowers at DR4B_AUTON_SLEW_DOWN, 560)
constexpr int SKILLS_FIRST_PRELOAD_LOWER_MS = 500;
// Matchloads and field pins: the lift starts lowering onto the goal and the claw opens this long after (it used to
// open right away, so the pin dropped from above the goal and fell off instead of scoring)
constexpr int SKILLS_RELEASE_DELAY_MS = 700;
constexpr int SKILLS_PRELOAD_LOWER_MS = 200;       // preload: lowers onto the goal for this long, then opens (fixed time, no waiting
                                                   // for the lift; 100 -> 10 deg takes about 0.2 s). Was "wait for the lift, up to 0.4 s"
constexpr double SKILLS_SCORE_BACKUP = 4.0;        // backs straight off a goal this far after scoring, then rolls into the next move
// Every turn in place happens with at least SKILLS_TURN_SPARE inches between the corners and anything (walls, goals).
// (was 1": the real robot is never exactly where it thinks, so 1" wasn't enough)
constexpr double SKILLS_TURN_SPARE = 3.0;
// After the preload: far enough back that it can turn with SKILLS_TURN_SPARE to goal A
constexpr double SKILLS_PRELOAD_BACKUP = ROBOT_HALF_DIAG + GOAL_HALF + SKILLS_TURN_SPARE - AUTON_CLAW_REACH;   // 12.3
// Start: after the toggle, drives straight forward this far before turning toward goal A, so the back corners
// clear the wall when it turns (turning right at the wall swung a back corner into it: that shoved the robot
// and threw off where it thought goal A was). Worked out from the robot size: corner swing - center-to-back + 1".
constexpr double SKILLS_START_FORWARD = ROBOT_HALF_DIAG - ROBOT_CENTER_TO_BACK + SKILLS_TURN_SPARE;   // 6.5 (was +1")
constexpr double SKILLS_TOGGLE_FINISH_DEG = 10.0;  // start toggle: waits until it's this close to done before driving off the wall
constexpr int SKILLS_TOGGLE_LIFT_HEAD_MS = 200;    // start: the lift goes up this long before the toggle starts (fixed time)
                                                   // (was 150, +50 for the 15 degrees higher toggle lift)

// --- Goals (RW-Template's Override field: goals at (+-47.09, +-23.55) and (+-23.55, +-47.09)) ---
// If a goal is still off on your field, tape-measure from the LEFT wall to the goal's center (X = -70.25 + that)
// and from the BOTTOM wall (Y = -70.25 + that) and put those here.
constexpr double SKILLS_GOAL_B_X = -23.55;   // goal B: bottom quadrant (was -24)
constexpr double SKILLS_GOAL_B_Y = -47.09;   // (was -48)
constexpr double SKILLS_GOAL_A_X = -47.09;   // goal A: the preload goal (was -48)
constexpr double SKILLS_GOAL_A_Y = -23.55;   // (was -24)
constexpr double SKILLS_A_PRELOAD_EXTRA_DIST = -2.0; // + = preload goes further onto goal A, - = shorter (2" back, was 0)

// --- Route points worked out from the field + robot size ---
// Turn spot T (between the goals and the loader). Every turn near the loader happens here. Its Y leaves 2.7" between
// the corners and the bottom wall when it turns in place (corners swing out 11.85"); its X leaves SKILLS_TURN_GOAL_SPARE
// between the corners and goal B's base. From T it steers down onto the loader line (at most about 18 deg) without
// stopping, so it's square to the loader by the line-up spot P and the corners stay about 1" off the bottom wall.
constexpr double SKILLS_TURN_GOAL_SPARE = SKILLS_TURN_SPARE;
constexpr double SKILLS_LOADER_TURN_SPARE = 2.5;    // corners to the bottom wall when it turns in place at T
constexpr double SKILLS_LOADER_TURN_Y = -FIELD_HALF + ROBOT_HALF_DIAG + SKILLS_LOADER_TURN_SPARE;   // -55.95 (2.75" above the loader line)
const double SKILLS_LOADER_TURN_X = SKILLS_GOAL_B_X - std::sqrt(std::pow(ROBOT_HALF_DIAG + GOAL_HALF + SKILLS_TURN_GOAL_SPARE, 2)
                                                                - std::pow(SKILLS_LOADER_TURN_Y - SKILLS_GOAL_B_Y, 2));   // -40.3
// Leaving the loader it backs out the same way (straight back to P, then back to T) and turns toward the goal at T.
const double SKILLS_SCORE_SPOT_X = SKILLS_LOADER_TURN_X;
constexpr double SKILLS_SCORE_SPOT_Y = SKILLS_LOADER_TURN_Y;

// --- Scoring distance offsets (+ = claw goes further onto the goal, - = shorter). Start at 0 and tune ---
// (Aim is always straight at the goal now: the drive steers at the goal center the whole way.)
constexpr double SKILLS_A_SCORE_EXTRA_DIST = 0.0;
constexpr double SKILLS_B_PRELOAD_EXTRA_DIST = -2.0;   // same as goal A's preload (SKILLS_A_PRELOAD_EXTRA_DIST) (was -1)   // goal B's first matchload (spot 1): 2" back (was 0; field-tuned to -1.5 before)
constexpr double SKILLS_B_SCORE_EXTRA_DIST = 0.0;      // matchloads onto goal B (was field-tuned to -1.5)

// --- Bottom-wall toggle + park ---
// Route from goal B to the toggle goes up through the gap between goal A and goal B, then around the
// right side of goal B (there isn't room for the robot to squeeze between goal B and the bottom wall).
constexpr double SKILLS_WAY1_X = -36.0, SKILLS_WAY1_Y = -36.0;   // gap between goal A and goal B
constexpr double SKILLS_WAY2_X = -14.0, SKILLS_WAY2_Y = -30.0;   // above-right of goal B
constexpr double SKILLS_TOGGLE_X = 0.0;            // toggle is in the middle of the bottom wall
constexpr double SKILLS_TOGGLE_TURN_Y = -48.0;     // turns to face the toggle here (far enough from the wall to swing)
constexpr double SKILLS_TOGGLE_START_Y = -59.0;    // robot center 4" before the claw touches the toggle
constexpr double SKILLS_TOGGLE_PUSH_DIST = 5.0;    // drives this far into the toggle each time (a little extra to make sure it flips)
constexpr int SKILLS_TOGGLE_PUSH_SPEED = 50;          // gentler push so it doesn't bounce off and twist (was 70)
constexpr int SKILLS_TOGGLE_PUSH_MS = 700;         // max time per push (the wall stops it)
constexpr double SKILLS_TOGGLE_LIFT_HEIGHT = 30.0; // lift height while pushing the toggle with the claw
// Park: straight back from the toggle (same line as the toggle, like your drawing) until the robot's center
// reaches this Y. -14 stops the back of the robot 2" short of the center goal.
// Note: on x = 0 it pushes the small pin at the bottom corner of the center diamond along with it.
constexpr double SKILLS_PARK_Y = -14.0;

// --- Skills plan ---
// Goal A: preload (spot 1), the two field pins, then SKILLS_A_LOADS matchloads. Goal B: as many as still fit.
// Every pin goes on the next free spot of its goal: a missed pickup (the pin check) leaves no gap.
// The field pins sit where the robot drives to the loader (pin 2 is 10" from the loader line), so instead of going
// around them it picks them up and scores them: each one is a loader trip saved.
constexpr int SKILLS_A_LOADS = 3;   // goal A: preload + 2 field pins + 3 matchloads = 6 pins (was 1)
// Goal B gets as many pins as the time guard says still fit (its first one is lowered slowly like the preload), up to
// this many. Every try counts as a full cycle for the time guard whether or not the pickup worked (backup scoring).
constexpr int SKILLS_B_MAX_PINS = 5;  // goal B: first load + 4 matchloads = 5 pins (was 3). The time guard stops early if one won't fit
// The 6th pin on goal A needs a 6th lift height (main.cpp's list stops at 5). TODO: measure it: put 5 pins on a goal,
// raise the lift by hand until a 6th clears the stack, and read "Lift" on the controller. 795 is the list's spacing
// carried one more step (the 5th height + the 4th-to-5th gap). If the lift can't go that high, lower it to its top.
constexpr double SKILLS_STACK_HEIGHT_6 = 795.0;
// First trip from goal A (after the preload) to the matchloader: goes around the RIGHT side of goal A
// (there isn't room to pass goal A on the wall side and still turn), then down the middle to the loader turn spot.
// WAY1 is above-right of goal A (clear of the small red pin at the left corner of the center diamond),
// WAY2 is between goal A and goal B.
// WAY1: halfway between goal A's row and the center row (y = -0.5 tiles), far enough right of goal A
//       that the corners clear it when the robot turns down.
// WAY2: the tile corner between goal A and goal B (-1.5, -1.5 tiles), the middle of the gap between them.
constexpr double SKILLS_FIRST_WAY1_X = (SKILLS_GOAL_A_X + GOAL_HALF) + ROBOT_HALF_DIAG + 0.5;   // -30.7 (moves with goal A)
constexpr double SKILLS_FIRST_WAY1_Y = -0.5 * TILE;                                             // -12
constexpr double SKILLS_FIRST_WAY2_X = (SKILLS_GOAL_A_X + SKILLS_GOAL_B_X) / 2.0;               // -35.3: middle of the gap
constexpr double SKILLS_FIRST_WAY2_Y = (SKILLS_GOAL_A_Y + SKILLS_GOAL_B_Y) / 2.0;               // -35.3

// Field pins: grabbed the way the qual route grabs them (turned 180 degrees for this side, so the same approach
// angles: pin 1 from the upper left, pin 2 from the upper right, the open side of each).
// Smooth pickup (no sideways jerk): it stops on the pin's approach line SKILLS_PIN1_LINEUP / SKILLS_PIN2_LINEUP inches
// before the pin (spots with SKILLS_TURN_SPARE around them), turns once to face along the line, then one straight
// drive steers along the line into the pin with the claw spinning and stops with the claw on it. No second turn.
// Pin 2's line runs through the middle of the gap between goal A and goal B; the lineup spot is past the gap,
// because there's no room to turn inside it.
constexpr double SKILLS_PIN1_LINEUP = 20.5;   // inches from pin 1 (from the upper left)
constexpr double SKILLS_PIN2_LINEUP = 28.0;   // inches from pin 2 (from the upper right, through the gap)
constexpr int SKILLS_PIN_IN_SPEED = 60;       // top speed along the line into a pin (was 100: it knocked the pins over)
// It still went into pins 1 and 2 too fast (pin 1 got knocked over in the video), so it starts slowing down earlier and
// ends slower: from SKILLS_PIN_BRAKE_DIST out it eases down from SKILLS_PIN_IN_SPEED, reaching SKILLS_PIN_SLOW_SPEED at
// SKILLS_PIN_SLOW_DIST out, and drives the rest at that speed. (Distances are to the grab spot.)
constexpr double SKILLS_PIN_BRAKE_DIST = 14.0;
constexpr double SKILLS_PIN_SLOW_DIST = 8.0;  // the last this-many inches into a pin... (was 6)
constexpr int SKILLS_PIN_SLOW_SPEED = 20;     // ...at this speed (was 25)
// The qual autons (runOpeningRoute, src/quals.cpp) use the same gentle approach (true). The old one (the last 6" at 25, no
// easing: false) is about 1 s quicker for the two pins, but it's the one that knocked pin 1 over in the skills video.
constexpr bool QUALS_GENTLE_PIN_APPROACH = true;
constexpr double QUALS_PIN_SLOW_DIST = 6.0;
constexpr int QUALS_PIN_SLOW_SPEED = 25;
bool skillsRunning = false;   // true during the 1-minute skills run (skillsAuton), false in the qual opening
// --- Qual autons only (runOpeningRoute, 15 s). Skills keeps its own values. ---
constexpr int QUALS_PICKUP_MS = 600;          // fixed field-pin pickup (skills: SKILLS_PICKUP_MS 1200). The claw is already
                                              // spinning for the last 5" of the drive in, so the pin is in by then
// Quicker scoring of the two field pins (everything from the turn toward the goal on; the drive to each pin, the pickup
// and the lift kick after it stay the same). Moderate, so it doesn't tip:
constexpr int QUALS_SCORE_TURN_SPEED = 127;      // turn toward the goal with the pin (skills: SKILLS_MATCHLOAD_TURN_SPEED 110)
constexpr double QUALS_SCORE_ACCEL = 4.5;        // speed-up per 10 ms driving to the goal and backing off (skills: SKILLS_ACCEL 3.2)
constexpr double QUALS_SCORE_DECEL = 7.0;        // slow-down per inch left (skills: SKILLS_DECEL 5)
constexpr double QUALS_LOWER_SLEW = 700.0;       // degrees/second the lift lowers onto the goal (auton normally 560, driver 700)
constexpr int QUALS_RELEASE_DELAY_MS = 250;      // pin 1: lowers onto the goal this long, then the claw opens (skills: 700)
constexpr int QUALS_RELEASE_MS = 100;            // pin 1: then this long before backing off (skills: 250)
constexpr double QUALS_PIN1_BACKUP = 12.5;       // backs off the goal after pin 1: just turning room for the trip to pin 2
                                                 // (skills: SKILLS_PIN1_BACKUP 14)
// Pin 2 (the last thing in quals): the claw does NOT open and nothing waits for the lift. The moment the claw is over
// the goal, the lift goes limp (skillsCoastLift) with a small push down to get it moving, and falls onto the goal under
// its own weight, still holding the pin. No drive back.
// End of the qual autons: lift PID off and both lift motors on coast, so the lift falls by itself and keeps falling
// after auton ends (a pin that's almost on still drops onto the goal). opcontrol() in main.cpp turns the lift PID back
// on first thing (holding the lift wherever it ended up).
// pushDown: at the same moment it goes on coast, QUALS_END_PUSH_MV down for QUALS_END_PUSH_MS (a lift sitting still on
// coast can just stay there; the push gets it falling), then no power at all
constexpr int QUALS_END_PUSH_MV = 3000;   // small: a quarter of full power (12000)
constexpr int QUALS_END_PUSH_MS = 150;
void skillsCoastLift(bool pushDown = false) {
    autonLiftCoasting = false;
    autonLiftGrounding = false;
    dr4bSettling = false;
    dr4bTargetForced = false;
    dr4bManualMode = true;   // the lift PID task leaves the lift alone while this is true
    DR4B.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);   // both motors (set_brake_mode only sets the first one)
    if (pushDown) {
        DR4B.move_voltage(-QUALS_END_PUSH_MV);
        const uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(QUALS_END_PUSH_MS)) {
            autonTick();
            pros::delay(10);
        }
    }
    DR4B.move_voltage(0);
}
// In quals the lift never goes above spot 3, so "tall" (SKILLS_TALL_LIFT_DEG) only came from the lift overshooting its
// target a little, which put the drive to the goal with pin 2 on the slow tall-lift speeds. Quals only count the lift as
// tall when it's also aimed above the line. Skills is unchanged.
bool skillsLiftIsTall() {
    if (getDR4BPosition() <= SKILLS_TALL_LIFT_DEG) return false;
    return skillsRunning || dr4bTarget > SKILLS_TALL_LIFT_DEG;
}
double skillsAccelLow = SKILLS_ACCEL;   // speed-up / slow-down used with the lift low (quals scoring raises them)
double skillsDecelLow = SKILLS_DECEL;
// After pin 1 it backs this far off goal A before heading for pin 2 (room to turn: SKILLS_TURN_SPARE to goal A)
constexpr double SKILLS_PIN1_BACKUP = 14.0;

// --- Sweep toggle at the end (claw side, right corner first then left corner) ---
// Route from goal B: up through the gap between the two goals to WAY1 (above goal B), then over to the start spot.
// (Assumes both field pins the qual auton picks up are off the field.)
// WAY0: 13" before the middle of the gap, on the gap's diagonal, so it goes through the gap square to it
//       (crossing it at an angle from goal B put a corner on goal A: the gap is only a few inches wider than the robot).
//       13" leaves about 5" between the corners and goal A when it turns there (odometry can be 3-4" off by now).
// WAY1: 8" past the middle of the gap between goal A and goal B, heading out toward the field center.
constexpr double SKILLS_SWEEP_WAY0_X = SKILLS_FIRST_WAY2_X - 13.0 * 0.7071;  // -44.5
constexpr double SKILLS_SWEEP_WAY0_Y = SKILLS_FIRST_WAY2_Y - 13.0 * 0.7071;  // -44.5
constexpr double SKILLS_SWEEP_WAY1_X = SKILLS_FIRST_WAY2_X + 8.0 * 0.7071;   // -29.7 (8" out: at 6" a corner came within 0.1" of goal A)
constexpr double SKILLS_SWEEP_WAY1_Y = SKILLS_FIRST_WAY2_Y + 8.0 * 0.7071;   // -29.7
// Sweep start spot. From here it faces SKILLS_SWEEP_HEADING_1 and drives in, so the robot's front-RIGHT corner
// hits the toggle first (around x = 0), then it swings to SKILLS_SWEEP_HEADING_2 so the front-LEFT corner hits.
// The start spot is worked out from the toggle's position and the robot's size (see skillsSweepToggleAndPark):
// it's SKILLS_SWEEP_RUN_IN inches back from where the front-right corner touches the toggle.
constexpr double SKILLS_SWEEP_TOGGLE_X = 0.0;          // toggle is centered on the bottom wall
constexpr double SKILLS_SWEEP_TOGGLE_DEPTH = 1.0;      // how far the toggle sticks out from the wall: the corner meets it here (was field-tuned to 3)
constexpr double SKILLS_SWEEP_TOGGLE_FACE_Y = -FIELD_HALF + SKILLS_SWEEP_TOGGLE_DEPTH;   // -69.25
constexpr double SKILLS_SWEEP_RUN_IN = 21.0;           // inches of straight drive into the toggle
// The robot NEVER touches the wall: the drive in and the swing both stop as soon as a corner is this close to the
// bottom wall (by odometry). The toggle has to stick out past this for the corner to push it.
// 1.5 (was 1): odometry lags about 0.5" there and the robot slides about 0.3" after it stops, which used up all of 1".
constexpr double SKILLS_WALL_GAP = 1.5;
constexpr double SKILLS_SWEEP_HEADING_1 = 165.0;   // facing the bottom wall, angled so the right corner leads
constexpr double SKILLS_SWEEP_HEADING_2 = 195.0;   // swung the other way so the left corner hits
constexpr double SKILLS_SWEEP_DRIVE_IN = SKILLS_SWEEP_RUN_IN + 0.5;   // max drive in (stops early when it hits, or at SKILLS_WALL_GAP) (was +2)
constexpr int SKILLS_SWEEP_DRIVE_SPEED = 110;     // (was 70) it slows down on its own as a corner gets near the wall
// The swing into toggle #2 RAMS it: full power on the left side (the right side holds), no slowing down and no stopping
// short of the wall (toggles are the one thing it's allowed to hit). It used to slow down and stop as the corner got
// near the wall, so it never curved all the way into the toggle. It swings until it's SKILLS_SWEEP_SWING_PAST_DEG past
// SKILLS_SWEEP_HEADING_2, it's stopped turning (jammed against the toggle), or SKILLS_SWEEP_SWING_MS, then goes
// straight into the full-speed drive back to the park.
constexpr int SKILLS_SWEEP_SWING_SPEED = 127;      // left side speed for the swing (right side stays put)
constexpr double SKILLS_SWEEP_SWING_PAST_DEG = 5.0;
constexpr int SKILLS_SWEEP_SWING_MS = 600;         // max time for the swing (was 450)
// Then fast straight back toward the center and a braked stop (it used to cut power and coast, and in the
// simulator the coast rolled it 10" too far, into the center goal). Where it stops: back of the robot
// SKILLS_PARK_GAP short of the center goal's base. Lower the gap to park closer.
constexpr double SKILLS_PARK_GAP = 3.0;   // (was 2: with 3-4" of odometry drift by the end of the run it could reach the goal)
constexpr double SKILLS_PARK_ROLL = 2.0;   // it slides about this far after braking from full speed (simulator), so it brakes this early
constexpr double SKILLS_PARK_STOP_Y = -(GOAL_HALF + ROBOT_CENTER_TO_BACK + SKILLS_PARK_GAP);    // -15.3
constexpr int SKILLS_PARK_SPEED = 127;
constexpr int SKILLS_PARK_MAX_MS = 2500;

// --- Time guard ---
// 60 s is NOT enough for everything (about 85-90 s of driving). Before each matchload cycle the robot checks
// the clock; if that cycle plus the toggle + park won't fit, it skips the rest and goes straight to the toggle
// and park, so those points are never lost.
constexpr int SKILLS_PINS_PER_GOAL = 5;            // fill each goal to this many (uses ALLIANCE_HEIGHTS spots 1-5)
constexpr uint32_t SKILLS_TIME_LIMIT_MS = 60000;
constexpr uint32_t SKILLS_SAFETY_MS = 500;         // spare time kept at the end (was 1000)
// Once a matchload cycle has run, the guard uses how long that one really took (+ this margin) instead of
// SKILLS_CYCLE_MS, so when the cycles are quicker on the real robot it goes for the extra pin.
constexpr uint32_t SKILLS_CYCLE_MARGIN_MS = 400;
// The time guard's estimates below are simulator times. The robot runs slower than the simulator, so once goal A
// cycles have been timed it scales the goal B cycles and the end reserve by (real goal A cycle) / SKILLS_SIM_A_CYCLE_MS
// (never less than 1), so the toggle and park still fit on a slower robot.
constexpr uint32_t SKILLS_SIM_A_CYCLE_MS = 8400;   // a goal A matchload cycle in the simulator (8.1-8.7 s with the 0.7 s release
                                                   // delay, taller spots take longer) (was 7800 before the delay)
constexpr uint32_t SKILLS_END_RESERVE_MS = 7000;     // time for the sweep toggle + park from goal B (6.4 s in the simulator with the
                                                     // wall resets at every stop; was 6500 before them) (5.9 s in the simulator with the
                                                     // no-wall sweep and rolling route; the robot ran ~10% slower than the simulator before) (was 7500)
constexpr uint32_t SKILLS_CYCLE_MS = 8500;  // (was 7500, before the release delay)         // time for one matchload cycle (loader -> goal): 6.2-6.9 s in the simulator with the
                                                   // no-stop loader drive (the old path was 9.0 s there and 10-11 s on the robot) (was 9500)
constexpr uint32_t SKILLS_PIN_ONLY_CYCLE_MS = 7500;  // (was 8500: the first goal B load takes about 7.1 s in the simulator,
                                                     // and 8500 made the guard skip it with time to spare)  // (7.6 s in the simulator with the release delay; was 7800)  // time for goal B's first matchload (loader -> goal B)
// One goal B matchload cycle (turn spot -> loader -> goal B -> back): 6.35 s in the simulator. Used for the next goal B
// load until a goal B to goal B cycle has been timed (the first goal B load starts from goal A, so it runs longer and
// made the guard skip a load that fits: the run ended at 53 s).
constexpr uint32_t SKILLS_B_CYCLE_MS = 7200;   // (was 6500, before the 0.7 s release delay)
// true  = steps always run in order, but once the next matchload won't fit it skips to the toggle + park, so
//         those always happen before 60 s (the full list of 6 matchloads needs ~80 s, it can't all fit).
// false = never skips; whatever isn't done at 60 s just doesn't happen (usually the toggle + park).
constexpr bool SKILLS_USE_TIME_GUARD = true;

// =====================================================================

// Skills opening = right-wall qual route turned 180 degrees (same tuning, same turn directions)
const AutonRoute SKILLS_ROUTE = {
    -AUTON_START_X, -AUTON_START_Y, std::fmod(AUTON_START_HEADING + 180.0, 360.0),
    SKILLS_GOAL_A_X, SKILLS_GOAL_A_Y,
    AUTON_PRELOAD_TURN_LESS_DEG, SKILLS_A_PRELOAD_EXTRA_DIST,
    -AUTON_PIN1_X, -AUTON_PIN1_Y, std::fmod(AUTON_PIN1_APPROACH_HEADING + 180.0, 360.0),
    AUTON_PIN1_GRAB_TURN_OFFSET, AUTON_PIN1_GRAB_EXTRA_DIST, AUTON_PIN1_SCORE_TURN_LESS, AUTON_PIN1_SCORE_EXTRA_DIST,
    -AUTON_PIN2_X, -AUTON_PIN2_Y, std::fmod(AUTON_PIN2_APPROACH_HEADING + 180.0, 360.0),
    AUTON_PIN2_GRAB_TURN_OFFSET, AUTON_PIN2_GRAB_EXTRA_DIST, AUTON_PIN2_SCORE_TURN_LESS, AUTON_PIN2_SCORE_EXTRA_DIST
};

uint32_t skillsStartMs = 0;

// True if a step that takes neededMs still leaves time for the toggle + park
double skillsSlowerThanSim = 1.0;   // real goal A cycle / SKILLS_SIM_A_CYCLE_MS (at least 1), set after the goal A loads

bool skillsHaveTime(uint32_t neededMs) {
    if (!SKILLS_USE_TIME_GUARD) return true;
    uint32_t elapsed = pros::millis() - skillsStartMs;
    uint32_t endReserve = static_cast<uint32_t>(SKILLS_END_RESERVE_MS * skillsSlowerThanSim);
    return elapsed + neededMs + endReserve + SKILLS_SAFETY_MS <= SKILLS_TIME_LIMIT_MS;
}

// Drives straight along `heading` (both sides same speed, no turning). + distance = forward, - = backward.
void autonDriveDist(double distance, double heading, int speed, uint32_t timeoutMs) {
    Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    int dir = distance >= 0 ? 1 : -1;
    double target = std::fabs(distance);

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        Pose p = chassis.getPose();
        double done = ((p.x - start.x) * hx + (p.y - start.y) * hy) * dir;
        if (done >= target - 0.25) break;
        chassis.tank(dir * speed, dir * speed, true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

// Small lift after grabbing (same as after a field pin: AUTON_PIN_PICKUP_RISE degrees)
void skillsSmallLift(double rise, uint32_t maxWaitMs = PIN_LIFT_TIMEOUT_MS) {
    autonLiftGrounding = false;
    double liftGoal = getDR4BPosition() + rise;
    pinLiftCount++;

    dr4bSettling = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.move_voltage(PIN_LIFT_VOLTAGE);

    uint32_t liftStart = pros::millis();
    while (getDR4BPosition() < liftGoal && pros::millis() - liftStart < maxWaitMs) {
        autonTick();
        pros::delay(10);
    }

    if (getDR4BPosition() >= liftGoal) {
        dr4bTarget = getDR4BPosition();
        DR4B.brake();
    } else {
        dr4bTarget = liftGoal;   // not there yet: the lift PID keeps taking it up while the robot moves
    }
    dr4bTargetForced = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    dr4bManualMode = false;
}

// Next to the loader the bottom wall is only 3.2" from the robot's side even when it's square (the loader line is
// 11.55" from the wall, the robot is 8.39" from its center to its side). Any angle swings a front or back corner
// closer. This is the most the robot may be angled off square (degrees) when its center is `above` inches above the
// loader line, so its lowest corner stays SKILLS_LOADER_WALL_SPARE off the wall. (40 at most.)
// wallSpare: how close the corners may get to the bottom wall (the way out uses SKILLS_LOADER_EXIT_WALL_SPARE)
double skillsLoaderMaxAngle(double above, double wallSpare = SKILLS_LOADER_WALL_SPARE) {
    const double room = (SKILLS_LOADER_LINE_Y + FIELD_HALF) - wallSpare + above;   // center to wall, minus spare
    double best = 0.0;
    for (int deg = 0; deg <= 40; deg++) {
        double a = deg * PI / 180.0;
        double reach = std::max(ROBOT_CENTER_TO_FRONT, ROBOT_CENTER_TO_BACK) * std::sin(a) + (ROBOT_WIDTH / 2.0) * std::cos(a);
        if (reach > room) break;
        best = deg;
    }
    return best;
}

// Lift height for a stack spot on our alliance goal: spots 1-5 from main.cpp's list, spot 6 = SKILLS_STACK_HEIGHT_6
double skillsStackHeight(int spot) {
    return spot >= 6 ? SKILLS_STACK_HEIGHT_6 : getStackHeight(GOAL_ALLIANCE, spot);
}

// The claw is spinning freely (nothing in it), for the check during a pickup (SKILLS_PICKUP_FREE_RPM)
bool skillsClawSpinningFree() {
#ifdef RW_SIM_PROS
    return skillsSimListed("SIM_MISS");   // simulator: the pickups listed in SIM_MISS grab nothing
#else
    return std::fabs(claw.get_actual_velocity()) >= SKILLS_PICKUP_FREE_RPM;
#endif
}

// A pickup: the robot is stopped with the claw on the pin. Exactly SKILLS_PICKUP_MS, every time: the claw spins at full
// speed, and in the last SKILLS_PICKUP_LIFT_MS the lift kicks up `rise` degrees at full power. No stall wait, no
// waiting for the lift: the lift PID finishes the rise (if it isn't done) while the robot drives off.
// resetJump > 0: also does a field-aware wall reset (fieldAwareReset) at the start, inside the fixed pickup time,
// so it costs nothing
// Returns false if the claw spun freely the whole time (SKILLS_PICKUP_FREE_RPM: nothing grabbed). Then the lift does
// NOT kick up (it stays held down) and the claw stops. Same fixed time either way.
bool skillsPickup(double rise, double resetJump = 0.0) {
    const uint32_t t0 = pros::millis();
    const int pickupMs = skillsRunning ? SKILLS_PICKUP_MS : QUALS_PICKUP_MS;
    const int checkFromMs = skillsRunning ? SKILLS_PICKUP_CHECK_FROM_MS : QUALS_PICKUP_MS / 2;
    skillsPickupCount++;
    autonClawStopAtMs = 0;
    autonClawSlowAtMs = 0;
    claw.move_velocity(200);
    if (resetJump > 0.0) fieldAwareReset(resetJump);
    int readings = 0, freeReadings = 0;
    while (pros::millis() - t0 < static_cast<uint32_t>(pickupMs - SKILLS_PICKUP_LIFT_MS)) {
        if (pros::millis() - t0 >= static_cast<uint32_t>(checkFromMs)) {
            readings++;
            if (skillsClawSpinningFree()) freeReadings++;
        }
        autonTick();
        pros::delay(10);
    }
    if (readings > 0 && freeReadings >= SKILLS_PICKUP_FREE_SHARE * readings) {
        while (pros::millis() - t0 < static_cast<uint32_t>(pickupMs)) {
            autonTick();
            pros::delay(10);
        }
        claw.move_velocity(0);
        return false;
    }
    autonLiftGrounding = false;
    const double liftGoal = getDR4BPosition() + rise;
    pinLiftCount++;
    dr4bSettling = false;
    dr4bManualMode = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.move_voltage(PIN_LIFT_VOLTAGE);
    while (pros::millis() - t0 < static_cast<uint32_t>(pickupMs)) {
        autonTick();
        pros::delay(10);
    }
    dr4bTarget = std::max(getDR4BPosition(), liftGoal);   // holds there (keeps going up if it isn't there yet)
    dr4bTargetForced = true;
    dr4bManualMode = false;
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);   // keeps squeezing gently while it carries the pin
    skillsWatchPin(true);                        // holding now: check it really has the pin (SKILLS_PIN_HELD_RPM)
    return true;
}

// Waits until the robot has really stopped (under SKILLS_SETTLE_SPEED in/s, at most SKILLS_SETTLE_MAX_MS), so a
// turn in place starts where the robot is supposed to be instead of sliding on past it
void skillsSettle() {
    chassis.tank(0, 0, true);
    Pose last = chassis.getPose();
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_SETTLE_MAX_MS)) {
        autonTick();
        pros::delay(20);
        Pose p = chassis.getPose();
        double speed = std::hypot(p.x - last.x, p.y - last.y) / 0.02;
        last = p;
        if (speed < SKILLS_SETTLE_SPEED) break;
    }
}

// Stall check for the drive loops: true once the robot has moved less than SKILLS_STALL_MOVE in the last
// SKILLS_STALL_MS (after the first 2 x SKILLS_STALL_MS of the drive). Call it once per loop.
struct SkillsStallCheck {
    uint32_t start = pros::millis(), mark = pros::millis();
    Pose markPose = chassis.getPose();
    bool stuck() {
        uint32_t now = pros::millis();
        if (now - mark < static_cast<uint32_t>(SKILLS_STALL_MS)) return false;
        Pose p = chassis.getPose();
        bool moved = std::hypot(p.x - markPose.x, p.y - markPose.y) >= SKILLS_STALL_MOVE;
        mark = now;
        markPose = p;
        return !moved && now - start >= static_cast<uint32_t>(2 * SKILLS_STALL_MS);
    }
};

// ---------------------------------------------------------------------
// CONTINUOUS DRIVING (skills): no stopping to aim or settle between moves
// ---------------------------------------------------------------------
bool skillsDriveUntilStopped(double distance, double heading, int speed, uint32_t timeoutMs);   // further down

// -180 to 180
// (std::remainder: works for any heading. The old fmod(+540) version broke once the robot's heading had
//  counted past 540 degrees, which made it spin the wrong way late in the run)
double skillsWrap(double deg) {
    return std::remainder(deg, 360.0);
}

// Heading the robot needs to face to drive at (x, y). forwards = false: the heading for backing toward it.
double skillsBearing(double x, double y, bool forwards) {
    Pose p = chassis.getPose();
    double h = std::atan2(x - p.x, y - p.y) * 180.0 / PI;
    return forwards ? h : h + 180.0;
}

// Turns toward (x, y) only if it's more than SKILLS_AIM_SKIP_DEG off, and hands off to the drive
// SKILLS_ROLL_TURN_EXIT_DEG early without settling (the drive steers out the rest while moving).
// turnSpeed: top turning speed while the lift is low (it's always SKILLS_TALL_TURN_SPEED when the lift is tall)
// settleFirst = false: no settle / wall reset before a big turn (the quals' turn toward the goal right after a pickup,
// which has just done the reset)
void skillsAimAt(double x, double y, bool forwards, int turnSpeed = SKILLS_TURN_SPEED, bool settleFirst = true) {
    Pose p = chassis.getPose();
    double err = skillsWrap(skillsBearing(x, y, forwards) - p.theta);
    if (std::fabs(err) > SKILLS_AIM_SKIP_DEG) {
        if (settleFirst && std::fabs(err) > SKILLS_SETTLE_TURN_DEG && skillsRollSpeed == 0.0) {
            skillsSettle();   // big turn in place: from a full stop, right where it should be
            fieldAwareReset(SKILLS_RESET_JUMP);
            p = chassis.getPose();
            err = skillsWrap(skillsBearing(x, y, forwards) - p.theta);
        }
        int maxSpeed = skillsLiftIsTall() ? SKILLS_TALL_TURN_SPEED : turnSpeed;
        chassis.turnToHeading(p.theta + err, 1200,
                              {.maxSpeed = maxSpeed,
                               .minSpeed = SKILLS_ROLL_TURN_MIN_SPEED,
                               .earlyExitRange = SKILLS_ROLL_TURN_EXIT_DEG});
        autonWaitMotion();
        skillsRollSpeed = 0.0;
    }
}

// Drives at (x, y), steering toward it the whole way (so it fixes its aim while moving instead of stopping to turn).
//   forwards:   false = backs toward the point
//   stopShort:  stops this many inches before the point (claw reach when scoring, 0 for a waypoint)
//   endSpeed:   0 = slows down and stops at the end. > 0 = doesn't stop, hands off to the next move still rolling.
//   liftNeeded: not used any more (it never waits for the lift); kept so the calls stay the same
// Gentle speed-up / slow-down when the lift is tall, same as skillsRampDrive (anti-tip).
void skillsDriveToPoint(double x, double y, bool forwards, int maxSpeed, double stopShort,
                        int endSpeed, double liftNeeded, uint32_t timeoutMs) {
    const int dir = forwards ? 1 : -1;
    double speed = (skillsRollSpeed * dir > 0.0) ? std::fabs(skillsRollSpeed) : 0.0;   // keep rolling from the last move
    double aim = skillsBearing(x, y, forwards);
    (void)liftNeeded;
    SkillsStallCheck stall;

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        if (skillsAbortOnLost && skillsPinLost) { endSpeed = 0; break; }   // carrying a pin that isn't there: stop
        Pose p = chassis.getPose();
        if (stall.stuck()) break;   // stopped moving: it's touching something, so it doesn't push

        // Inside 6" it keeps its last heading so it can't spin around the point
        if (std::hypot(x - p.x, y - p.y) > 6.0) aim = skillsBearing(x, y, forwards);

        // Distance left along the direction of travel (goes negative if it passes the point)
        double moveDir = (forwards ? aim : aim - 180.0) * PI / 180.0;
        double remaining = (x - p.x) * std::sin(moveDir) + (y - p.y) * std::cos(moveDir) - stopShort;
        if (remaining <= (endSpeed > 0 ? 1.0 : 0.3)) break;

        bool tall = skillsLiftIsTall();
        double prev = speed;
        // Speed limit from the lift height. The robot is never made to drop
        // below this limit faster than SKILLS_BRAKE_PER_TICK, so a lift going tall mid-drive can't slam it.
        double limit = tall ? std::min(static_cast<double>(maxSpeed), SKILLS_TALL_MAX_SPEED) : static_cast<double>(maxSpeed);
        // Distance slow-down at the end (already gentle by itself)
        double stopCap = SKILLS_MIN_DRIVE_SPEED + (tall ? SKILLS_DECEL_TALL : skillsDecelLow) * remaining;
        if (endSpeed > 0) stopCap = std::max(stopCap, static_cast<double>(endSpeed));

        double maxDrop = tall ? SKILLS_BRAKE_PER_TICK_TALL : SKILLS_BRAKE_PER_TICK;
        limit = std::max(limit, prev - maxDrop);   // ease down to a new limit instead of slamming
        speed = std::max(0.0, std::min({prev + (tall ? SKILLS_ACCEL_TALL : skillsAccelLow), limit, stopCap}));

        double err = skillsWrap(aim - p.theta);   // + = needs to turn right
        double corr = std::clamp(SKILLS_STEER_KP * err, -SKILLS_STEER_MAX, SKILLS_STEER_MAX);
        double base = dir * speed * std::max(0.0, std::cos(err * PI / 180.0));   // less forward while it's still turning a lot
        chassis.tank(static_cast<int>(base + corr), static_cast<int>(base - corr), true);

        autonTick();
        pros::delay(10);
    }

    if (endSpeed > 0) {
        skillsRollSpeed = dir * speed;   // motors keep going; the next move picks up from here
    } else {
        chassis.tank(0, 0, true);
        skillsRollSpeed = 0.0;
    }
}

// Like skillsDriveToPoint with a full stop, but it really ends ON the point: it waits until the robot has stopped,
// and if it slid past (or stopped short) by more than SKILLS_PRECISE_TOL it drives that bit back, slowly and dead
// straight. Used for the spots where it then turns in place near a wall or goal (the turn spot T).
void skillsDriveToPointPrecise(double x, double y, bool forwards, uint32_t timeoutMs) {
    skillsDriveToPoint(x, y, forwards, SKILLS_DRIVE_SPEED, 0.0, 0, 0.0, timeoutMs);
    skillsSettle();
    // Standing still at T: X / Y from the walls (the bottom wall gives how far it is from the loader line), so the
    // steer onto the loader line starts from where it really is. Field-aware: RW's localizeWithWalls skipped every
    // reading here (all inside its 18" corner margin)
    fieldAwareReset(SKILLS_RESET_JUMP);
    Pose p = chassis.getPose();
    double h = p.theta * PI / 180.0;
    double along = (x - p.x) * std::sin(h) + (y - p.y) * std::cos(h);   // + = the spot is ahead of it
    if (std::fabs(along) > SKILLS_PRECISE_TOL) {
        skillsRampDrive(along, p.theta, SKILLS_PRECISE_SPEED, 600);
        skillsSettle();
        fieldAwareReset(SKILLS_RESET_JUMP);
    }
}

// Backs straight off along the current heading (no turning). endSpeed as above.
void skillsBackStraight(double dist, int endSpeed) {
    Pose p = chassis.getPose();
    double h = p.theta * PI / 180.0;
    skillsDriveToPoint(p.x - dist * std::sin(h), p.y - dist * std::cos(h), false,
                       SKILLS_DRIVE_SPEED, 0.0, endSpeed, 0.0, 1500);
}

// Scores what the claw is holding on a goal. No waiting for the lift anywhere:
//   lift up to SKILLS_TURN_LIFT_MAX while it turns toward the goal, then all the way up while it drives there,
//   and the moment the claw is over the goal it lowers and lets go, waits a fixed release time and backs straight off.
//   preloadStyle = true: lowers onto the goal for SKILLS_PRELOAD_LOWER_MS, then opens (the preload)
//   backupDist / backupEndSpeed: how far it backs off, and whether it keeps rolling into the next move
//   turnSpeed: top speed for the turn toward the goal (the tall-lift turn speed takes over if the lift is tall)
//   preloadLowerMs: preload style only, how long it lowers onto the goal before opening
// Smallest distance (inches) from any corner of the robot to any wall, from odometry
double skillsWallClearance(const Pose& p) {
    const double h = p.theta * PI / 180.0;
    const double fx = std::sin(h), fy = std::cos(h), rx = std::cos(h), ry = -std::sin(h);   // forward, right
    double best = 1e9;
    for (double along : {ROBOT_CENTER_TO_FRONT, -ROBOT_CENTER_TO_BACK}) {
        for (double side : {-1.0, 1.0}) {
            const double cx = p.x + along * fx + side * (ROBOT_WIDTH / 2.0) * rx;
            const double cy = p.y + along * fy + side * (ROBOT_WIDTH / 2.0) * ry;
            best = std::min({best, FIELD_HALF - std::fabs(cx), FIELD_HALF - std::fabs(cy)});
        }
    }
    return best;
}

// Before lowering a lift that's up (see SKILLS_DROP_WALL_CLEAR): if any corner is closer than SKILLS_DROP_WALL_CLEAR to
// a wall, it turns toward (towardX, towardY) (the goal it was carrying to: open field) and drives that way until every
// corner is that far off the walls (never closer than turning room to that spot), gently since the lift is up.
void skillsMoveOffWalls(double towardX, double towardY) {
    if (skillsWallClearance(chassis.getPose()) >= SKILLS_DROP_WALL_CLEAR) return;
    skillsAimAt(towardX, towardY, true, SKILLS_MATCHLOAD_TURN_SPEED);
    const double room = ROBOT_HALF_DIAG + GOAL_HALF + SKILLS_TURN_SPARE;
    double speed = 0.0;
    SkillsStallCheck stall;
    const uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_DROP_MOVE_MAX_MS)) {
        Pose p = chassis.getPose();
        const double short_ = SKILLS_DROP_WALL_CLEAR - skillsWallClearance(p);   // inches still to gain
        if (short_ <= 0.0 || std::hypot(towardX - p.x, towardY - p.y) <= room || stall.stuck()) break;
        const bool tall = getDR4BPosition() > SKILLS_TALL_LIFT_DEG;
        speed = std::min({speed + (tall ? SKILLS_ACCEL_TALL : SKILLS_ACCEL), static_cast<double>(SKILLS_DROP_MOVE_SPEED),
                          SKILLS_MIN_DRIVE_SPEED + (tall ? SKILLS_DECEL_TALL : SKILLS_DECEL) * short_});
        const double err = skillsWrap(skillsBearing(towardX, towardY, true) - p.theta);
        const double corr = std::clamp(SKILLS_STEER_KP * err, -SKILLS_STEER_MAX, SKILLS_STEER_MAX);
        chassis.tank(static_cast<int>(speed + corr), static_cast<int>(speed - corr), true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    skillsRollSpeed = 0.0;
}

// Room to turn in place next to a goal: if its center is closer than turning room to (goalX, goalY), it moves straight
// along its heading AWAY from the goal (backs up if the goal is in front, drives forward if it's behind) until it isn't.
// If the goal is off to its side, moving along the heading wouldn't help, so it stays put. (It used to always back up,
// which drove it toward goal B whenever it was facing away from it, standing near the loader turn spot.)
void skillsGetTurningRoom(double goalX, double goalY) {
    Pose p = chassis.getPose();
    const double room = ROBOT_HALF_DIAG + GOAL_HALF + SKILLS_TURN_SPARE;   // center to goal center to turn in place
    const double d = std::hypot(goalX - p.x, goalY - p.y);
    if (d >= room) return;
    const double h = p.theta * PI / 180.0;
    const double ahead = ((goalX - p.x) * std::sin(h) + (goalY - p.y) * std::cos(h)) / std::max(d, 0.1);   // 1 = dead ahead
    if (std::fabs(ahead) < 0.5) return;
    double dist = std::min((room - d) / std::fabs(ahead) + 0.5, 8.0);
    // ...but never so far that a corner gets within SKILLS_TURN_SPARE of a wall (backing off goal B heads for the bottom wall)
    const double dir = ahead > 0.0 ? -1.0 : 1.0;   // -1 = backing
    for (double s = 0.25; s <= dist; s += 0.25) {
        Pose q(p.x + dir * s * std::sin(h), p.y + dir * s * std::cos(h), p.theta);
        if (skillsWallClearance(q) < SKILLS_TURN_SPARE) { dist = s - 0.25; break; }
    }
    if (dist < 0.5) return;
    if (ahead > 0.0) {
        skillsBackStraight(dist, 0);
    } else {
        skillsDriveToPoint(p.x + dist * std::sin(h), p.y + dist * std::cos(h), true, SKILLS_DRIVE_SPEED, 0.0, 0, 0.0, 1500);
    }
}

// A pickup was lost on the way to a goal (the hold check): stop, get off the walls if the lift is up
// (skillsMoveOffWalls), open the claw (in case the check was wrong, so nothing is dragged along), lift all the way down,
// move off the goal until it has room to turn without touching it (skillsGetTurningRoom), and give the lift a fixed
// SKILLS_DROP_LOWER_MS to come down before it drives anywhere.
void skillsDropMissedPin(double goalX, double goalY) {
    skillsAbortOnLost = false;
    chassis.cancelMotion();
    chassis.tank(0, 0, true);
    skillsRollSpeed = 0.0;
    if (getDR4BPosition() > SKILLS_DROP_LIFT_UP_DEG) skillsMoveOffWalls(goalX, goalY);
    autonOpenClaw();
    autonStartGrounding();
    const uint32_t t0 = pros::millis();
    skillsGetTurningRoom(goalX, goalY);
    skillsSettle();
    fieldAwareReset(SKILLS_RESET_JUMP);   // standing still anyway (inside the fixed lowering time)
    while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_DROP_LOWER_MS)) {
        autonTick();
        pros::delay(10);
    }
}

// Returns true if it scored. Carrying a pin (not preload style), it stops as soon as the pin check says the claw is
// empty, then skillsDropMissedPin and returns false (the stack spot stays free for the next pin).
// Quals (not skills), carrying a field pin: the quicker scoring (QUALS_SCORE_ block). holdAtEnd (quals' pin 2): once
// the claw is over the goal the lift goes limp right away (skillsCoastLift), claw closed, no waiting, no backing off.
bool skillsScoreOnGoal(double goalX, double goalY, double height, double extraDist, bool preloadStyle,
                       double backupDist, int backupEndSpeed, int turnSpeed = SKILLS_MATCHLOAD_TURN_SPEED,
                       int preloadLowerMs = SKILLS_PRELOAD_LOWER_MS, bool holdAtEnd = false) {
    const bool quick = !skillsRunning && !preloadStyle;
    struct QuickScoring {   // quicker drives while scoring in quals, back to normal however it returns
        bool on;
        explicit QuickScoring(bool q) : on(q) {
            if (on) { skillsAccelLow = QUALS_SCORE_ACCEL; skillsDecelLow = QUALS_SCORE_DECEL; }
        }
        ~QuickScoring() { skillsAccelLow = SKILLS_ACCEL; skillsDecelLow = SKILLS_DECEL; }
    } quickScoring(quick);
    if (quick) turnSpeed = QUALS_SCORE_TURN_SPEED;
    autonGoalX = goalX;
    autonGoalY = goalY;
    skillsAbortOnLost = skillsPinWatch;   // only while it's carrying a picked-up pin (not the preload)
    auto lost = [&] {
        if (!(skillsAbortOnLost && skillsPinLost)) return false;
        skillsDropMissedPin(goalX, goalY);
        return true;
    };

    const double stopShort = AUTON_CLAW_REACH - extraDist;   // robot center stops here, claw over the goal

    autonSetLift(std::min(height, SKILLS_TURN_LIFT_MAX));
    skillsAimAt(goalX, goalY, true, turnSpeed, !quick);
    if (lost()) return false;
    if (height > SKILLS_TURN_LIFT_MAX) {
        // Tall stack spot: full speed with the lift at SKILLS_TURN_LIFT_MAX until it's just far enough out that the
        // lift can finish the climb on the way in (SKILLS_LIFT_DEG_PER_S, at the tall-lift speed), then the rest of
        // the way up and in at the tall-lift speeds. Nothing waits for the lift.
        double raiseDist = (height - SKILLS_TURN_LIFT_MAX) / SKILLS_LIFT_DEG_PER_S * SKILLS_TALL_IN_PER_S + SKILLS_RAISE_SPARE;
        Pose p = chassis.getPose();
        if (std::hypot(goalX - p.x, goalY - p.y) > stopShort + raiseDist + 2.0) {
            skillsDriveToPoint(goalX, goalY, true, SKILLS_DRIVE_SPEED, stopShort + raiseDist,
                               static_cast<int>(SKILLS_TALL_MAX_SPEED), 0.0, 3000);
            if (lost()) return false;
        }
    }
    autonSetLift(height);   // the rest of the way up while it drives in (about 1 s to the top)
    skillsDriveToPoint(goalX, goalY, true, SKILLS_DRIVE_SPEED, stopShort, 0, 0.0, 3000);
    if (lost()) return false;
    skillsAbortOnLost = false;

    if (quick) {
        const double normalSlewDown = dr4bSlewDown;
        dr4bSlewDown = QUALS_LOWER_SLEW;
        if (holdAtEnd) {
            // Claw over the goal with the pin: no lowering wait, the lift goes on coast with a small push down and drops
            // onto the goal (claw stays closed)
            dr4bSlewDown = normalSlewDown;
            skillsCoastLift(true);
            return true;
        }
        autonLowerLiftCoast(AUTON_LOW_HEIGHT);
        uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(QUALS_RELEASE_DELAY_MS)) {   // lowers onto the goal first
            autonTick();
            pros::delay(10);
        }
        autonOpenClaw();
        t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(QUALS_RELEASE_MS)) {
            autonTick();
            pros::delay(10);
        }
        dr4bSlewDown = normalSlewDown;
        skillsBackStraight(backupDist, backupEndSpeed);
        return true;
    }

    uint32_t releaseMs = SKILLS_RELEASE_MS;
    if (preloadStyle) {
        autonSetLift(AUTON_PRELOAD_DROP_HEIGHT);
        uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(preloadLowerMs)) {
            autonTick();
            pros::delay(10);
        }
        autonOpenClaw(AUTON_PRELOAD_CLAW_OPEN_MS);
        releaseMs = SKILLS_PRELOAD_RELEASE_MS;
    } else {
        autonLowerLiftCoast(AUTON_LOW_HEIGHT);
        uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_RELEASE_DELAY_MS)) {   // lowers onto the goal first
            autonTick();
            pros::delay(10);
        }
        autonOpenClaw();
    }

    // Fixed release time so the claw is off the pin before it backs away (otherwise it drags the pin off the goal)
    uint32_t r0 = pros::millis();
    fieldAwareReset(SKILLS_RESET_JUMP);   // standing still at the goal: inside the fixed release time, so it costs nothing
    while (pros::millis() - r0 < releaseMs) {
        autonTick();
        pros::delay(10);
    }

    skillsBackStraight(backupDist, backupEndSpeed);
    return true;
}

// To the matchloader and into it, nothing ever touches the field:
//   1. to the turn spot T (forwards = false right after scoring: it backs straight there along the line it scored on)
//   2. turn at T toward the loader line
//   3. one drive with no stops, claw spinning at clawSpeed: steers down onto the loader line (heading kept between
//      SKILLS_LINEUP_MIN_HEADING and 270), is square on the line by P, and stops at the grab spot with the claw
//      SKILLS_LOADER_GAP short of the loader
bool skillsNextTripForwards = false;   // set after missing pin 2: the next trip to T drives forwards (it's facing that way)

void skillsDriveIntoLoader(int clawSpeed, bool forwards) {
    autonStartGrounding();   // lift all the way down while driving
    forwards = forwards || skillsNextTripForwards;
    skillsNextTripForwards = false;

    // 1. To T (only if not already there)
    Pose p = chassis.getPose();
    if (std::hypot(SKILLS_LOADER_TURN_X - p.x, SKILLS_LOADER_TURN_Y - p.y) > 3.0) {
        skillsAimAt(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, forwards, SKILLS_EMPTY_TURN_SPEED);
        skillsDriveToPointPrecise(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, forwards, 2500);   // stops ON T
    }

    // Heading that steers it down onto the loader line: SKILLS_LINEUP_STEER degrees per inch above the line,
    // never steeper than SKILLS_LINEUP_MIN_HEADING, and dead square (270) from P in
    auto lineupAim = [](const Pose& q) {
        if (q.x <= SKILLS_LOADER_LINEUP_X) return 270.0;
        double above = std::max(0.0, q.y - SKILLS_LOADER_LINE_Y);
        double minHeading = std::max(SKILLS_LINEUP_MIN_HEADING, 270.0 - skillsLoaderMaxAngle(above));
        return std::clamp(270.0 - SKILLS_LINEUP_STEER * (q.y - SKILLS_LOADER_LINE_Y), minHeading, 270.0);
    };

    // 2. Turn at T toward the loader line (T has room to turn). The clear trip faces straight in (270) instead.
    const bool clearing = skillsClearLoaderNext;
    skillsClearLoaderNext = false;
    Pose now = chassis.getPose();
    double aim = clearing ? 270.0 : lineupAim(now);
    chassis.turnToHeading(now.theta + skillsWrap(aim - now.theta), 1200,
                          {.maxSpeed = SKILLS_EMPTY_TURN_SPEED, .minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    if (clearing) {
        // The last matchload missed: clear the loader first, at T's height (see SKILLS_LOADER_CLEAR_GAP). Claw off, so it
        // doesn't suck in a lying cup it would then carry to the loader.
        claw.move_velocity(0);
        Pose q = chassis.getPose();
        const double clearX = SKILLS_LOADER_FACE_X + SKILLS_LOADER_CLEAR_GAP + ROBOT_CENTER_TO_FRONT;   // -54.45
        if (q.x - clearX > 0.5) skillsRampDrive(q.x - clearX, 270.0, SKILLS_LINEUP_SPEED, 1500);
        skillsSettle();
        fieldAwareReset(SKILLS_RESET_JUMP);   // standing still: fix X / Y before swinging near the wall and loader
        // Biggest right turn that keeps every corner SKILLS_LOADER_CLEAR_WALL_SPARE off the bottom wall and
        // SKILLS_LOADER_CLEAR_FACE_SPARE off the loader / left wall
        q = chassis.getPose();
        double turn = 0.0;
        for (double deg = 1.0; deg <= SKILLS_LOADER_CLEAR_MAX_DEG; deg += 1.0) {
            double h = (q.theta + deg) * PI / 180.0;
            double fx = std::sin(h), fy = std::cos(h), rx = std::cos(h), ry = -std::sin(h);   // forward, right
            bool fits = true;
            for (double side : {-1.0, 1.0}) {
                for (double along : {ROBOT_CENTER_TO_FRONT, -ROBOT_CENTER_TO_BACK}) {
                    double cx = q.x + along * fx + side * (ROBOT_WIDTH / 2.0) * rx;
                    double cy = q.y + along * fy + side * (ROBOT_WIDTH / 2.0) * ry;
                    if (cy < -FIELD_HALF + SKILLS_LOADER_CLEAR_WALL_SPARE) fits = false;   // bottom wall
                    if (cx < SKILLS_LOADER_FACE_X + SKILLS_LOADER_CLEAR_FACE_SPARE) fits = false;   // loader / left wall
                }
            }
            if (!fits) break;
            turn = deg;
        }
        if (turn > SKILLS_LOADER_CLEAR_COAST_DEG) {
            // Right until it has turned `turn` degrees, then back square, driving the wheels directly
            const double start = q.theta;
            uint32_t tt = pros::millis();
            while (skillsWrap(chassis.getPose().theta - start) < turn - SKILLS_LOADER_CLEAR_COAST_DEG && pros::millis() - tt < 800) {
                chassis.tank(SKILLS_LOADER_CLEAR_TURN_SPEED, -SKILLS_LOADER_CLEAR_TURN_SPEED, true);
                autonTick();
                pros::delay(10);
            }
            tt = pros::millis();
            while (skillsWrap(chassis.getPose().theta - start) > SKILLS_LOADER_CLEAR_COAST_DEG && pros::millis() - tt < 800) {
                chassis.tank(-SKILLS_LOADER_CLEAR_TURN_SPEED, SKILLS_LOADER_CLEAR_TURN_SPEED, true);
                autonTick();
                pros::delay(10);
            }
            chassis.tank(0, 0, true);
            // square again (the drive back holds 270 too, this just makes sure it starts square)
            Pose s = chassis.getPose();
            chassis.turnToHeading(s.theta + skillsWrap(270.0 - s.theta), 600,
                                  {.maxSpeed = SKILLS_LOADER_CLEAR_TURN_SPEED, .minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
            autonWaitMotion();
        }
        // Back out to T (steering onto it, so the drive in starts from T's height), then the normal drive in from there
        skillsDriveToPointPrecise(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, false, 1500);
    }

    // 3. Claw on, then one drive onto the loader line and in to the grab spot
    autonClawStopAtMs = 0;
    autonClawSlowAtMs = 0;
    claw.move_velocity(clawSpeed);
    // Drives in along the loader line until the center reaches stopX. inSpeed: top speed once on the line.
    auto driveIn = [&](double stopX, double inSpeed) {
        double speed = 0.0;
        SkillsStallCheck stall;
        uint32_t t0 = pros::millis();
        while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_LOADER_IN_MAX_MS)) {
            Pose q = chassis.getPose();
            if (stall.stuck()) break;   // stopped short (touching the loader?): never pushes, the pickup starts right away
            double remaining = q.x - stopX;
            if (remaining <= SKILLS_LOADER_ROLL) break;
            double top = q.x > SKILLS_LOADER_LINEUP_X ? SKILLS_LINEUP_SPEED : inSpeed;
            if (std::fabs(skillsWrap(lineupAim(q) - q.theta)) > 5.0) top = std::min(top, static_cast<double>(SKILLS_LOADER_TURNING_SPEED));
            speed = std::min({speed + SKILLS_ACCEL, top, SKILLS_MIN_DRIVE_SPEED + SKILLS_DECEL * remaining});
            double corr = std::clamp(SKILLS_LOADER_STEER_KP * skillsWrap(lineupAim(q) - q.theta), -SKILLS_LOADER_STEER_MAX, SKILLS_LOADER_STEER_MAX);
            chassis.tank(static_cast<int>(speed + corr), static_cast<int>(speed - corr), true);
            autonTick();
            pros::delay(10);
        }
        chassis.tank(0, 0, true);
    };

    driveIn(SKILLS_LOADER_GRAB_X, SKILLS_LOADER_DRIVE_IN_SPEED);
    skillsRollSpeed = 0.0;
}

// Out of the loader and back to the turn spot T in one backing drive, no stops: dead straight back for
// SKILLS_LOADER_EXIT_STRAIGHT (the front corners are right next to the loader), then it climbs away from the bottom
// wall toward T, facing between 270 and SKILLS_LINEUP_MIN_HEADING (the shallower the closer it is to the wall, so the
// corners stay at least 2" off it), and stops on T. (It used to back to P and then steer at T: that curve ended
// about 1.3" below T, so the turn at T started 1.2" from the wall.) The lift starts going up once it's clear of the loader.
// liftTo < 0: the lift stays down (a missed pickup: nothing to lift)
void skillsBackOutOfLoader(double liftTo) {
    const bool raise = liftTo >= 0.0;
    double speed = 0.0;
    bool liftStarted = false;
    SkillsStallCheck stall;
    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 2000) {
        Pose q = chassis.getPose();
        if (stall.stuck()) break;
        double remaining = SKILLS_LOADER_TURN_X - q.x;   // backing toward +X
        if (remaining <= 0.3) break;
        if (raise && !liftStarted && q.x >= SKILLS_LOADER_LINEUP_X) {
            autonSetLift(liftTo);
            liftStarted = true;
        }
        double o = q.y - SKILLS_LOADER_LINE_Y;   // inches above the loader line
        double facing = 270.0;
        if (q.x > SKILLS_LOADER_GRAB_X + SKILLS_LOADER_EXIT_STRAIGHT) {
            // climbs as steeply as the wall allows (skillsLoaderMaxAngle, less near the wall), easing off over the last
            // inches (SKILLS_LOADER_EXIT_STEER) so it levels out right at T's height, then straight back
            const double toGo = SKILLS_LOADER_TURN_Y - SKILLS_LOADER_LINE_Y - o;   // inches still to climb
            double climb = (toGo <= 0.2) ? 0.0 : std::min(skillsLoaderMaxAngle(std::max(0.0, o), SKILLS_LOADER_EXIT_WALL_SPARE),
                                                          SKILLS_LOADER_EXIT_STEER * toGo);
            facing = 270.0 - climb;
        }
        bool tall = getDR4BPosition() > SKILLS_TALL_LIFT_DEG;
        double top = tall ? SKILLS_TALL_MAX_SPEED : static_cast<double>(SKILLS_DRIVE_SPEED);
        // slow while it's still climbing, so it can turn as fast as it moves (at full speed it reached T 1.3" low)
        if (std::fabs(skillsWrap(facing - q.theta)) > 5.0 || facing != 270.0)
            top = std::min(top, static_cast<double>(SKILLS_LOADER_CLIMB_SPEED));
        speed = std::min({speed + (tall ? SKILLS_ACCEL_TALL : SKILLS_ACCEL), top,
                          SKILLS_MIN_DRIVE_SPEED + (tall ? SKILLS_DECEL_TALL : SKILLS_DECEL) * remaining});
        double corr = std::clamp(SKILLS_LOADER_STEER_KP * skillsWrap(facing - q.theta), -SKILLS_LOADER_STEER_MAX, SKILLS_LOADER_STEER_MAX);
        chassis.tank(static_cast<int>(-speed + corr), static_cast<int>(-speed - corr), true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    if (raise && !liftStarted) autonSetLift(liftTo);
    skillsRollSpeed = 0.0;
    skillsSettle();
    fieldAwareReset(SKILLS_RESET_JUMP);   // standing still at T: fix X / Y before the turn toward the goal
}

// One matchload (pin + cup): into the loader with the claw spinning, the fixed pickup, out, score on (goalX, goalY)
// fromLoaderTrip: true = it's arriving from the first trip (drives forwards), false = right after scoring (backs)
// Returns true if it scored. A missed pickup (nothing in the claw) or a pin lost on the way skips the scoring and
// returns false (the stack spot stays free for the next one).
// slowLower: scores it like the first preload (lowers onto the goal at SKILLS_FIRST_PRELOAD_SLEW_DOWN, then opens)
bool skillsMatchloadCycle(double goalX, double goalY, int stack, double extraDist, bool fromLoaderTrip,
                          bool slowLower = false) {
    double height = skillsStackHeight(stack);

    skillsDriveIntoLoader(AUTON_APPROACH_CLAW_SPEED, fromLoaderTrip);

    // The fixed pickup, standing still at the loader: X / Y from the bottom and left walls during it (RW's
    // localizeWithWalls skipped every reading here, they're all inside its 18" corner margin)
    if (!skillsPickup(SKILLS_LOAD_RISE, SKILLS_RESET_JUMP)) {
        // Missed (the claw spun freely the whole pickup: something is in the way). Nothing in the claw and the lift
        // never went up: back out to T with the lift still down, and clear the loader on the way back in
        skillsBackOutOfLoader(-1.0);
        skillsClearLoaderNext = true;
        return false;
    }

    skillsBackOutOfLoader(std::min(height, SKILLS_TURN_LIFT_MAX));
    // Out of the loader (standing on T): if the hold check says the claw lost the pin on the way out, drop it. The lift
    // is up and T is next to the bottom wall, so skillsDropMissedPin first moves toward the goal, off the wall, then
    // lowers. Then skip scoring
    if (skillsPinLost) {
        skillsDropMissedPin(goalX, goalY);
        skillsClearLoaderNext = true;   // it may have dropped something in front of the loader
        return false;
    }
    bool scored;
    if (slowLower) {
        // long enough to lower all the way at the slow speed, plus 50 msec (the first preload's 500 for 90 degrees)
        const int lowerMs = static_cast<int>((height - AUTON_PRELOAD_DROP_HEIGHT) / SKILLS_FIRST_PRELOAD_SLEW_DOWN * 1000.0) + 50;
        const double normalSlewDown = dr4bSlewDown;
        dr4bSlewDown = SKILLS_FIRST_PRELOAD_SLEW_DOWN;
        scored = skillsScoreOnGoal(goalX, goalY, height, extraDist, true, SKILLS_SCORE_BACKUP, SKILLS_ROLL_SPEED,
                                   SKILLS_MATCHLOAD_TURN_SPEED, lowerMs);
        dr4bSlewDown = normalSlewDown;
    } else {
        scored = skillsScoreOnGoal(goalX, goalY, height, extraDist, false, SKILLS_SCORE_BACKUP, SKILLS_ROLL_SPEED);
    }
    if (!scored) skillsClearLoaderNext = true;   // missed: whatever it missed may be lying in front of the loader
    return scored;
}

// Bottom-wall toggle twice (claw first), then straight back to park in the middle
void skillsToggleAndPark() {
    autonLiftGrounding = false;
    autonSetLift(SKILLS_TOGGLE_LIFT_HEIGHT);
    autonClawSlowAtMs = 0;
    autonClawStopAtMs = 0;
    claw.move_velocity(0);

    // Up through the gap between the goals, around goal B, to the turn spot above the toggle
    chassis.turnToPoint(SKILLS_WAY1_X, SKILLS_WAY1_Y, 800,
                        {.minSpeed = AUTON_TURN_MIN_SPEED, .earlyExitRange = AUTON_TURN_EXIT_DEG});
    autonWaitMotion();
    chassis.moveToPoint(SKILLS_WAY1_X, SKILLS_WAY1_Y, 1200,
                        {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = 3.0});
    autonWaitMotion();
    chassis.moveToPoint(SKILLS_WAY2_X, SKILLS_WAY2_Y, 1200,
                        {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = 3.0});
    autonWaitMotion();
    chassis.moveToPoint(SKILLS_TOGGLE_X, SKILLS_TOGGLE_TURN_Y, 1500);
    autonWaitMotion();

    // Face the toggle (claw toward the bottom wall = heading 180), then straight to 4" away
    chassis.turnToHeading(180, AUTON_AIM_TURN_TIMEOUT_MS,
                          {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();
    autonDriveStraight(SKILLS_TOGGLE_X, SKILLS_TOGGLE_START_Y, 180, 70, 1200);

    // Into the toggle and back, twice. Before each push it squares back up on the toggle
    // (the first push can knock it a little sideways, which is why the second one missed)
    for (int push = 0; push < 2; push++) {
        if (push > 0) {
            skillsRampDrive(-SKILLS_TOGGLE_PUSH_DIST, 180, SKILLS_TOGGLE_PUSH_SPEED, SKILLS_TOGGLE_PUSH_MS);
            chassis.moveToPose(SKILLS_TOGGLE_X, SKILLS_TOGGLE_START_Y, 180, 800, {.lead = 0.3, .maxSpeed = 60});
            autonWaitMotion();
            chassis.turnToHeading(180, 800, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = 1.0});
            autonWaitMotion();
        }
        skillsRampDrive(SKILLS_TOGGLE_PUSH_DIST, 180, SKILLS_TOGGLE_PUSH_SPEED, SKILLS_TOGGLE_PUSH_MS);
    }

    // Immediately straight back to the middle (gentle start so it doesn't flip backward)
    Pose p = chassis.getPose();
    skillsRampDrive(-(SKILLS_PARK_Y - p.y), 180, SKILLS_DRIVE_SPEED, 3000);
}

// Toggle + preload only (the start of the qual route, without the field pins)
void skillsStartAndPreload(const AutonRoute& r) {
    autonGoalX = r.goalX;
    autonGoalY = r.goalY;

    chassis.setPose(r.startX, r.startY, r.startHeading);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    fieldAwareReset(SKILLS_RESET_JUMP);   // distance sensors check the start spot (only if use_distance_resets is on)

    autonClawSlowAtMs = 0;
    autonClawStopAtMs = 0;
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);

    // Motor toggle (2 turns), same as the qual auton. The lift gets a fixed SKILLS_TOGGLE_LIFT_HEAD_MS head start
    // (it's past AUTON_TOGGLE_START_LIFT by then) instead of the code waiting to see it get there
    autonSetLift(AUTON_TOGGLE_LIFT_HEIGHT);
    {
        uint32_t liftStart = pros::millis();
        while (pros::millis() - liftStart < static_cast<uint32_t>(SKILLS_TOGGLE_LIFT_HEAD_MS)) {
            autonTick();
            pros::delay(10);
        }
    }
    double toggleTargetDeg = AUTON_TOGGLE_ROTATIONS * AUTON_TOGGLE_GEAR_RATIO * 180.0;
    toggleMechanism.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    toggleMechanism.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    toggleMechanism.tare_position();
    toggleMechanism.move_absolute(toggleTargetDeg, AUTON_TOGGLE_SPEED);
    uint32_t toggleStart = pros::millis();
    // Waits for the toggle to (nearly) finish: the robot drives straight off the wall next, so the toggle has to
    // be done before the back leaves the wall
    while (std::fabs(toggleMechanism.get_position() - toggleTargetDeg) > SKILLS_TOGGLE_FINISH_DEG
           && pros::millis() - toggleStart < AUTON_TOGGLE_TIMEOUT_MS) {
        autonTick();
        pros::delay(10);
    }
    if (pros::millis() - toggleStart >= AUTON_TOGGLE_TIMEOUT_MS) {
        toggleMechanism.move_velocity(0);
    }
    fieldAwareReset(SKILLS_RESET_JUMP);   // still against the wall: check again (the toggle pushes on the wall toggle)

    // Straight forward off the wall first, so the back corners can't hit the wall when it turns toward goal A. It drives
    // to SKILLS_START_FORWARD out from the START SPOT (not from wherever it is now): if the lift pushed the robot off the
    // wall a bit while it went up to the toggle height (the lift touches the wall), it just drives that much less and
    // turns at the same spot. The wall reset above already counted the push, and the aim at goal A is worked out from
    // where the robot really is, so it still lines up with the goal.
    autonSetLift(AUTON_PRELOAD_CARRY_HEIGHT);
    {
        Pose p = chassis.getPose();
        const double hx = std::sin(r.startHeading * PI / 180.0), hy = std::cos(r.startHeading * PI / 180.0);
        const double alreadyOut = (p.x - r.startX) * hx + (p.y - r.startY) * hy;
        if (SKILLS_START_FORWARD - alreadyOut > 0.3) skillsRampDrive(SKILLS_START_FORWARD - alreadyOut, r.startHeading, 70, 800);
    }

    // Preload onto goal A: turn, drive straight at it (steering the whole way), lower, open, back off
    // (slower lowering for this one, see SKILLS_FIRST_PRELOAD_SLEW_DOWN; the lift PID's lowering speed goes back after)
    const double normalSlewDown = dr4bSlewDown;
    dr4bSlewDown = SKILLS_FIRST_PRELOAD_SLEW_DOWN;
    skillsScoreOnGoal(r.goalX, r.goalY, AUTON_PRELOAD_CARRY_HEIGHT, r.preloadExtraDist,
                      true, SKILLS_PRELOAD_BACKUP, 0, SKILLS_TURN_SPEED, SKILLS_FIRST_PRELOAD_LOWER_MS);
    dr4bSlewDown = normalSlewDown;
}

// Field pin pickup. The robot is stopped facing the pin. One dead-straight drive along `heading` (it only holds the
// heading, never steers back onto a line: that turned it into pin 2): up to SKILLS_PIN_IN_SPEED, the last
// SKILLS_PIN_SLOW_DIST at SKILLS_PIN_SLOW_SPEED, and stops with the claw on the pin. The claw starts spinning
// AUTON_CLAW_OPEN_DIST before.
// extraDist: + = drives further into the pin, - = stops shorter
void skillsLineIntoPin(double pinX, double pinY, double heading, double extraDist = 0.0) {
    const double hx = std::sin(heading * PI / 180.0), hy = std::cos(heading * PI / 180.0);
    const double grabX = pinX - (AUTON_CLAW_REACH - extraDist) * hx, grabY = pinY - (AUTON_CLAW_REACH - extraDist) * hy;
    double speed = 0.0;
    bool clawOn = false;
    SkillsStallCheck stall;
    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < 2500) {
        Pose q = chassis.getPose();
        if (stall.stuck()) break;
        double remaining = (grabX - q.x) * hx + (grabY - q.y) * hy;      // along the line
        if (remaining <= 0.3) break;
        if (!clawOn && remaining <= AUTON_CLAW_OPEN_DIST) {
            autonClawStopAtMs = 0;
            autonClawSlowAtMs = 0;
            claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
            clawOn = true;
        }
        double aim = heading;   // dead straight: only holds the heading, never steers back onto a line (that turned it
                                // into pin 2)
        // SKILLS_PIN_IN_SPEED until SKILLS_PIN_BRAKE_DIST out, easing down to SKILLS_PIN_SLOW_SPEED by SKILLS_PIN_SLOW_DIST
        // (the qual opening: the old way, see QUALS_GENTLE_PIN_APPROACH)
        double pinTop = SKILLS_PIN_SLOW_SPEED;
        if (!skillsRunning && !QUALS_GENTLE_PIN_APPROACH) {
            pinTop = remaining < QUALS_PIN_SLOW_DIST ? QUALS_PIN_SLOW_SPEED : SKILLS_PIN_IN_SPEED;
        } else if (remaining > SKILLS_PIN_SLOW_DIST) {
            pinTop = std::min(static_cast<double>(SKILLS_PIN_IN_SPEED),
                              SKILLS_PIN_SLOW_SPEED + (SKILLS_PIN_IN_SPEED - SKILLS_PIN_SLOW_SPEED) * (remaining - SKILLS_PIN_SLOW_DIST)
                                                      / (SKILLS_PIN_BRAKE_DIST - SKILLS_PIN_SLOW_DIST));
        }
        speed = std::min({speed + SKILLS_ACCEL, pinTop, SKILLS_MIN_DRIVE_SPEED + SKILLS_DECEL * remaining});
        double corr = std::clamp(SKILLS_STEER_KP * skillsWrap(aim - q.theta), -SKILLS_STEER_MAX, SKILLS_STEER_MAX);
        chassis.tank(static_cast<int>(speed + corr), static_cast<int>(speed - corr), true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    if (!clawOn) {
        autonClawStopAtMs = 0;
        autonClawSlowAtMs = 0;
        claw.move_velocity(AUTON_APPROACH_CLAW_SPEED);
    }
    skillsRollSpeed = 0.0;
}

// To a field pin and pick it up: stop on its approach line `lineup` inches out, one turn to face along the line,
// the smooth drive in, then the same fixed pickup as a matchload. Returns false if the claw grabbed nothing
// (skillsPickup's check: the lift stays down)
// extraDist: + = drives further into the pin, - = stops shorter (see skillsLineIntoPin)
// heading: the pin's approach line (where it lines up). It then turns once to face the pin itself from where it
// really stopped (turnOffset degrees more to the right), and drives dead straight into it.
bool skillsGrabFieldPin(double pinX, double pinY, double heading, double lineup, double extraDist = 0.0,
                        double turnOffset = 0.0) {
    const double hx = std::sin(heading * PI / 180.0), hy = std::cos(heading * PI / 180.0);
    const double lineupX = pinX - lineup * hx, lineupY = pinY - lineup * hy;
    autonStartGrounding();   // lift all the way down on the way
    skillsAimAt(lineupX, lineupY, true, SKILLS_EMPTY_TURN_SPEED);
    skillsDriveToPoint(lineupX, lineupY, true, SKILLS_DRIVE_SPEED, 0.0, 0, 0.0, 2500);
    skillsSettle();
    fieldAwareReset(SKILLS_RESET_JUMP);   // standing on the pin's line: fix X / Y before lining up into it
    Pose now = chassis.getPose();
    const double aim = now.theta + skillsWrap(skillsBearing(pinX, pinY, true) + turnOffset - now.theta);
    chassis.turnToHeading(aim, 1200,
                          {.maxSpeed = SKILLS_EMPTY_TURN_SPEED, .minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();
    skillsLineIntoPin(pinX, pinY, aim, extraDist);
    const bool got = skillsPickup(SKILLS_FIELD_PIN_RISE, SKILLS_RESET_JUMP);   // (the reset happens inside the fixed pickup time)
    if (got) fieldMclPinPickedUp(pinX, pinY);   // MCL can use beams past that spot now (if it missed, the pin may still be there)
    return got;
}

// After the preload: both field pins onto goal A (spots 2 and 3). Afterwards the way to the loader is clear.
// pinNGrabTurn turns the approach line around the pin (+ = more clockwise), pinNGrabExtra = further into the pin.
// lastBackup / lastEndSpeed: how it leaves goal A after pin 2 (skills rolls on into the loader cycles)
// onGoal: how many pins are on the goal already (1: the preload). Each pin goes on the next free spot, so a missed one
// (the pin check) doesn't leave a gap. Returns how many it scored.
int skillsFieldPins(const AutonRoute& r, int onGoal, double lastBackup = SKILLS_SCORE_BACKUP,
                    int lastEndSpeed = SKILLS_ROLL_SPEED) {
    int scored = 0;
    // A missed pickup (nothing in the claw) skips the trip to the goal: the lift stays down and it goes straight on
    if (skillsGrabFieldPin(r.pin1X, r.pin1Y, r.pin1Heading, SKILLS_PIN1_LINEUP, r.pin1GrabExtra, r.pin1GrabTurn)) {
        // Backs off far enough to turn toward pin 2 with room to spare
        if (skillsScoreOnGoal(r.goalX, r.goalY, skillsStackHeight(onGoal + 1), r.pin1ScoreExtra, false,
                              skillsRunning ? SKILLS_PIN1_BACKUP : QUALS_PIN1_BACKUP, 0)) scored++;
    }

    if (skillsGrabFieldPin(r.pin2X, r.pin2Y, r.pin2Heading, SKILLS_PIN2_LINEUP, r.pin2GrabExtra, r.pin2GrabTurn)) {
        // quals: pin 2 is the last thing: the lift drops it onto the goal on coast, claw still closed (holdAtEnd)
        if (skillsScoreOnGoal(r.goalX, r.goalY, skillsStackHeight(onGoal + scored + 1), r.pin2ScoreExtra, false,
                              lastBackup, lastEndSpeed, SKILLS_MATCHLOAD_TURN_SPEED, SKILLS_PRELOAD_LOWER_MS,
                              !skillsRunning)) scored++;
    } else {
        // missed pin 2: it's standing at pin 2 facing the bottom-left, lift down. The trip to the loader turn spot drives
        // there forwards (a small turn, and the front, not the back, goes toward the bottom wall)
        skillsNextTripForwards = true;
    }
    // the matchload cycles take it from here (back to the turn spot -> loader)
    return scored;
}

// Lowest Y of the robot's corners (how close it is to the bottom wall), from its size and odometry
double skillsLowestCornerY() {
    Pose p = chassis.getPose();
    double h = p.theta * PI / 180.0;
    double along = std::cos(h) >= 0 ? ROBOT_CENTER_TO_BACK : ROBOT_CENTER_TO_FRONT;   // the end pointing down
    return p.y - along * std::fabs(std::cos(h)) - (ROBOT_WIDTH / 2.0) * std::fabs(std::sin(h));
}

// Drives straight along `heading` at `speed` until it has gone `distance` inches, it gets stopped
// (hits the toggle), or a corner gets SKILLS_WALL_GAP from the bottom wall. Keeps the heading the whole way.
// Returns true if it stopped because it hit something (not because it went the whole distance).
bool skillsDriveUntilStopped(double distance, double heading, int speed, uint32_t timeoutMs) {
    bool hit = false;
    Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0), hy = std::cos(heading * PI / 180.0);
    Pose last = start;
    uint32_t t0 = pros::millis(), stillSince = 0;
    while (pros::millis() - t0 < timeoutMs) {
        Pose p = chassis.getPose();
        if ((p.x - start.x) * hx + (p.y - start.y) * hy >= distance) break;
        // Slows down as a corner gets close to the wall, so it stops at SKILLS_WALL_GAP instead of sliding past it
        // (at full speed it slid 2-3" after the check, into the wall)
        double gapLeft = skillsLowestCornerY() - (-FIELD_HALF + SKILLS_WALL_GAP);
        if (gapLeft <= 0) break;
        double drive = std::min(static_cast<double>(speed), SKILLS_MIN_DRIVE_SPEED + SKILLS_DECEL * gapLeft);
        // stopped moving for 120 ms after the first 250 ms = it hit something
        bool moving = std::hypot(p.x - last.x, p.y - last.y) > 0.05;
        last = p;
        uint32_t now = pros::millis();
        if (moving || now - t0 < 250) stillSince = now;
        else if (now - stillSince > 120) { hit = true; break; }
        double err = std::remainder(heading - p.theta, 360.0);
        double corr = std::clamp(SKILLS_HEADING_KP * err, -20.0, 20.0);
        chassis.tank(static_cast<int>(drive + corr), static_cast<int>(drive - corr), true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
    return hit;
}

// End: lift to 0 on the way, claw-side sweep of the bottom toggle (right corner, then the left corner rams it at full
// power), then straight into the full-speed drive back toward the center and the park.
void skillsSweepToggleAndPark() {
    autonClawSlowAtMs = 0;
    autonClawStopAtMs = 0;
    claw.move_velocity(0);
    autonStartGrounding();   // lift goes to 0 while driving to the toggle

    // Up through the gap between the goals to WAY1, then over above goal B to the sweep start, rolling from one
    // to the next (same drives as the matchload cycles: real turns only where it's more than SKILLS_AIM_SKIP_DEG off).
    // First to WAY0 and stop, turn square to the gap there, then straight through it to WAY1
    skillsAimAt(SKILLS_SWEEP_WAY0_X, SKILLS_SWEEP_WAY0_Y, true, SKILLS_EMPTY_TURN_SPEED);
    skillsDriveToPoint(SKILLS_SWEEP_WAY0_X, SKILLS_SWEEP_WAY0_Y, true, SKILLS_DRIVE_SPEED, 0.0,
                       0, 0.0, 1500);   // stops: rolling through the turn here drifted a corner within 0.2" of goal A
    skillsAimAt(SKILLS_SWEEP_WAY1_X, SKILLS_SWEEP_WAY1_Y, true, SKILLS_EMPTY_TURN_SPEED);
    skillsDriveToPoint(SKILLS_SWEEP_WAY1_X, SKILLS_SWEEP_WAY1_Y, true, SKILLS_DRIVE_SPEED, 0.0, SKILLS_ROLL_SPEED, 0.0, 1200);
    // Sweep start from the field + robot size: at heading 1, the front-right corner sits
    // (front * |cos| + halfWidth * |sin|) ahead of the center toward the wall.
    const double h1 = SKILLS_SWEEP_HEADING_1 * PI / 180.0;
    const double cornerDepth = ROBOT_CENTER_TO_FRONT * std::fabs(std::cos(h1)) + (ROBOT_WIDTH / 2.0) * std::fabs(std::sin(h1));
    const double contactX = SKILLS_SWEEP_TOGGLE_X;
    const double contactY = SKILLS_SWEEP_TOGGLE_FACE_Y + cornerDepth;   // robot center when the corner touches
    const double SKILLS_SWEEP_START_X = contactX - SKILLS_SWEEP_RUN_IN * std::sin(h1);
    const double SKILLS_SWEEP_START_Y = contactY - SKILLS_SWEEP_RUN_IN * std::cos(h1);

    skillsAimAt(SKILLS_SWEEP_START_X, SKILLS_SWEEP_START_Y, true, SKILLS_EMPTY_TURN_SPEED);
    // rolls into the turn below instead of stopping first (the sweep start is 20" from goal B: room to swing)
    skillsDriveToPoint(SKILLS_SWEEP_START_X, SKILLS_SWEEP_START_Y, true, SKILLS_DRIVE_SPEED, 0.0, SKILLS_ROLL_SPEED, 0.0, 1500);

    // Angle so the right corner leads, then drive in until it hits (toggle #1)
    chassis.turnToHeading(SKILLS_SWEEP_HEADING_1, 900,
                          {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = 2.0});
    autonWaitMotion();
    skillsDriveUntilStopped(SKILLS_SWEEP_DRIVE_IN, SKILLS_SWEEP_HEADING_1, SKILLS_SWEEP_DRIVE_SPEED, 1200);

    // Swing: right side holds, left side full power, so the left corner RAMS toggle #2 (see SKILLS_SWEEP_SWING_SPEED)
    {
        const uint32_t t0 = pros::millis();
        double markH = chassis.getPose().theta;
        uint32_t markMs = t0;
        while (pros::millis() - t0 < static_cast<uint32_t>(SKILLS_SWEEP_SWING_MS)) {
            const double h = chassis.getPose().theta;
            if (std::remainder(SKILLS_SWEEP_HEADING_2 + SKILLS_SWEEP_SWING_PAST_DEG - h, 360.0) <= 0.0) break;
            // jammed on the toggle / wall (turned less than half a degree in 120 ms, after the first 150 ms): done
            if (pros::millis() - markMs >= 120) {
                if (pros::millis() - t0 >= 150 && std::fabs(std::remainder(h - markH, 360.0)) < 0.5) break;
                markH = h;
                markMs = pros::millis();
            }
            chassis.tank(SKILLS_SWEEP_SWING_SPEED, 0, true);
            autonTick();
            pros::delay(10);
        }
    }

    // Immediately full speed straight back toward the center (lift is at 0, so it won't flip), slowing down and
    // stopping (brake) with the back of the robot SKILLS_PARK_GAP short of the center goal. Ramped start and stop.
    // It backs AT the park spot (the toggle's X, SKILLS_PARK_STOP_Y), steering on the way: after the ram it's turned
    // 20-25 degrees past straight, and just holding heading 180 let it drift 12" sideways into the next goal over (in
    // the simulator). Where it stops comes from odometry, so wherever the ram left it, it ends at the same spot.
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    skillsRollSpeed = 0.0;
    skillsDriveToPoint(SKILLS_SWEEP_TOGGLE_X, SKILLS_PARK_STOP_Y - SKILLS_PARK_ROLL, false, SKILLS_PARK_SPEED, 0.0, 0, 0.0,
                       SKILLS_PARK_MAX_MS);
}

// The full 1-minute skills routine
//   motor toggle -> preload on goal A -> both field pins on goal A -> matchloads on goal A -> matchloads on goal B
//   -> matchloads on goal B
//   -> claw-side sweep toggle -> full speed back to park
// Clears what one run leaves behind (in case auton runs twice without restarting the program)
void skillsResetRunState(bool skillsRun) {
    skillsRunning = skillsRun;
    skillsClearLoaderNext = false;
    skillsNextTripForwards = false;
    skillsPickupCount = 0;
    skillsAbortOnLost = false;
    skillsWatchPin(false);
    skillsPinLost = false;
    skillsSlowerThanSim = 1.0;
    skillsRollSpeed = 0.0;
}

void skillsAuton() {
    skillsStartMs = pros::millis();
    skillsResetRunState(true);

    // 1. Motor toggle + preload on goal A (no field pins)
    skillsStartAndPreload(SKILLS_ROUTE);
    const double goalAX = SKILLS_ROUTE.goalX;
    const double goalAY = SKILLS_ROUTE.goalY;

    // 2. Both field pins onto goal A, then matchloads onto goal A. Each pin goes on the next free spot (onGoalA = how
    //    many are on it), so a missed pickup (the pin check) never leaves a gap or puts one too high.
    int onGoalA = 1;   // the preload
    onGoalA += skillsFieldPins(SKILLS_ROUTE, onGoalA);
    bool outOfTime = false;
    uint32_t lastCycleMs = 0;   // how long the last full matchload cycle (one that scored) took (0 = none yet)
    {
        int scored = 0, tries = 0;   // a missed pickup doesn't count: it tries again (up to 2 extra tries)
        while (scored < SKILLS_A_LOADS && tries < SKILLS_A_LOADS + 2) {
            const uint32_t clearMs = skillsClearLoaderNext ? SKILLS_CLEAR_EXTRA_MS : 0;   // this trip clears the loader first
            if (!skillsHaveTime((lastCycleMs ? lastCycleMs + SKILLS_CYCLE_MARGIN_MS : SKILLS_CYCLE_MS) + clearMs)) { outOfTime = true; break; }
            uint32_t cycleStart = pros::millis();
            if (skillsMatchloadCycle(goalAX, goalAY, onGoalA + 1, SKILLS_A_SCORE_EXTRA_DIST, false)) {
                scored++;
                onGoalA++;
                lastCycleMs = pros::millis() - cycleStart;   // (a missed one is shorter: it would make the guard too hopeful)
            }
            tries++;
        }
    }

    // How much slower than the simulator this robot is, from the goal A cycles it timed (scales the estimates below)
    if (lastCycleMs > 0) skillsSlowerThanSim = std::max(1.0, static_cast<double>(lastCycleMs) / SKILLS_SIM_A_CYCLE_MS);

    // 3. Goal B with whatever time is left: as many as the time guard says fit, missed pickups included (the guard
    //    always plans a full cycle). Its first pin is lowered slowly like the preload (spot 1).
    if (!outOfTime) {
        int onGoalB = 0;
        bool first = true;
        lastCycleMs = 0;   // goal B cycles are different: SKILLS_PIN_ONLY_CYCLE_MS / SKILLS_B_CYCLE_MS until one is timed
        while (onGoalB < SKILLS_B_MAX_PINS) {
            uint32_t planned = lastCycleMs ? lastCycleMs + SKILLS_CYCLE_MARGIN_MS
                                           : static_cast<uint32_t>((first ? SKILLS_PIN_ONLY_CYCLE_MS : SKILLS_B_CYCLE_MS)
                                                                   * skillsSlowerThanSim);
            if (skillsClearLoaderNext) planned += SKILLS_CLEAR_EXTRA_MS;   // this trip clears the loader first
            if (!skillsHaveTime(planned)) break;
            uint32_t cycleStart = pros::millis();
            const bool slow = onGoalB == 0;   // spot 1: like the preload
            if (skillsMatchloadCycle(SKILLS_GOAL_B_X, SKILLS_GOAL_B_Y, onGoalB + 1,
                                     slow ? SKILLS_B_PRELOAD_EXTRA_DIST : SKILLS_B_SCORE_EXTRA_DIST, false, slow)) {
                onGoalB++;
                if (!first) lastCycleMs = pros::millis() - cycleStart;   // (the first one starts from goal A: longer)
            }
            first = false;
        }
    }

    // 4. Sweep toggle + park. First it gets room to turn:
    //    - next to goal A (it just scored there, facing it): moves straight away from it (skillsGetTurningRoom)
    //    - goal B side: back to the turn spot T, which is the spot with turning room next to goal B and the bottom wall
    //      (goal B is scored from T, so after scoring it just backs down the line it came in on). If the last try was a
    //      missed pickup, it's already standing on T. (Backing straight off goal B by distance alone could end up next to
    //      the bottom wall, or, capped short of the wall, close enough to goal B that the turn toward the sweep clipped it.)
    {
        Pose p = chassis.getPose();
        if (std::hypot(goalAX - p.x, goalAY - p.y) < std::hypot(SKILLS_GOAL_B_X - p.x, SKILLS_GOAL_B_Y - p.y)) {
            skillsGetTurningRoom(goalAX, goalAY);
        } else if (std::hypot(SKILLS_LOADER_TURN_X - p.x, SKILLS_LOADER_TURN_Y - p.y) > 1.5) {
            skillsAimAt(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, false, SKILLS_EMPTY_TURN_SPEED);
            skillsDriveToPointPrecise(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, false, 2000);
        }
    }
    fieldAwareReset(SKILLS_RESET_JUMP);   // stopped: check the position before the sweep, so the toggle and park line up
    skillsSweepToggleAndPark();
}

}  // namespace skills_file

// The skills opening (toggle -> preload -> both field pins onto one goal) on any route, for the qual autons
// (src/quals.cpp). Everything about the route, tuning included, comes from `route`.
void runOpeningRoute(const OpeningRoute& route) {
    using namespace skills_file;
    const AutonRoute r = {
        route.startX, route.startY, route.startHeading,
        route.goalX, route.goalY,
        0.0, route.preloadExtraDist,
        route.pin1X, route.pin1Y, route.pin1Heading, route.pin1GrabTurn, route.pin1GrabExtra, 0.0, route.pin1ScoreExtra,
        route.pin2X, route.pin2Y, route.pin2Heading, route.pin2GrabTurn, route.pin2GrabExtra, 0.0, route.pin2ScoreExtra
    };
    skillsResetRunState(false);
    skillsStartAndPreload(r);
    skillsFieldPins(r, 1, route.lastBackup, 0);
    chassis.tank(0, 0, true);
    // Lift on coast for the rest of auton, whatever happened (pin 2 scored, missed or dropped). Already done (with the
    // push down) if pin 2 reached the goal; otherwise the lift is down already, so no push. The claw keeps holding pin 2
    // until auton ends; it isn't opened or stopped.
    skillsCoastLift();
}

// Called from autonomous() in main.cpp: runs the 1-minute skills routine.
void manyPinAlliance() {
    skills_file::skillsAuton();
}