// Autonomous: preload + 2 pins on the alliance goal. Called from autonomous() in main.cpp.
// Two starting spots: right wall (original) or top wall (mirror). Pick with AUTON_START_ON_TOP.
#include "robot.hpp"

// =====================================================================
//              AUTON: 2 PINS ON THE ALLIANCE GOAL (tuning)
// =====================================================================
// Field coordinates are in inches, with (0, 0) at the center of the field.
// +X = toward the right wall (the wall the robot starts on), +Y = toward the top of the field map.
// Headings: 0 = facing the top of the map, 90 = facing right, 180 = facing down, 270 = facing left.
// One tile = 24 inches.

// Start: back of the robot against the right wall, in the middle of the wall, claw facing into the field.
// The robot is 18" long, so its center is 9" off the wall (wall is at X = 72).
constexpr double AUTON_START_X = 62.0;
constexpr double AUTON_START_Y = 0;
constexpr double AUTON_START_HEADING = 270.0;

// Alliance goal (the one being scored on)
constexpr double AUTON_GOAL_X = 48.0;
constexpr double AUTON_GOAL_Y = 24.0;

// Pin + cup 1: one tile left of the goal
constexpr double AUTON_PIN1_X = 24.0;
constexpr double AUTON_PIN1_Y = 23.0;
constexpr double AUTON_PIN1_APPROACH_HEADING = 322.0;  // drives in from the lower right (stays clear of the small pin below it)

// Pin + cup 2: one tile above the goal
constexpr double AUTON_PIN2_X = 48.0;
constexpr double AUTON_PIN2_Y = 48.0;
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
constexpr double AUTON_CLAW_REACH = 8.0;        // inches from the robot's center (middle of the drive wheels) to the pin when the claw closes on it. TUNE THIS FIRST
constexpr double AUTON_CLAW_OPEN_DIST = 5.0;    // starts spinning the claw when this many inches from the grab spot
constexpr int AUTON_APPROACH_CLAW_SPEED = 300;  // rpm the claw spins at while driving into a pin (keeps spinning, never stops to open).
                                                // + = same way as R1 (closing), - = same way as R2 (opening)
constexpr double AUTON_SLOW_DIST = 4.0;         // the last this-many inches to the pin are driven slowly
constexpr int AUTON_SLOW_SPEED = 25;            // speed driving straight into a pin to pick it up (out of 127) (was 40)
constexpr int AUTON_CLAW_HOLD_SPEED = 60;       // slow grabbing speed (same as driver control)
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
constexpr double AUTON_TOGGLE_LIFT_HEIGHT = 30.0;   // degrees: lift goes up this much first so the toggles clear
constexpr double AUTON_TOGGLE_ROTATIONS = 2.0;      // full turns of the toggle (output side). Never goes past this
constexpr double AUTON_TOGGLE_GEAR_RATIO = 2.0;     // motor turns per toggle turn (1:2 = motor spins 2x for 1 toggle turn)
constexpr int AUTON_TOGGLE_SPEED = 200;             // motor rpm (same direction as holding A in driver control)
constexpr int AUTON_TOGGLE_TIMEOUT_MS = 2500;
constexpr double AUTON_TOGGLE_START_LIFT = 15.0;    // degrees: toggle starts spinning once the lift is this high (doesn't wait for the full lift)
constexpr double AUTON_TOGGLE_DONE_DEG = 90.0;      // motor degrees: robot starts moving when the toggle is this close to done.
                                                    // The toggle still finishes its 2 turns exactly on its own and stops there.       // stops trying after this, in case it gets stuck

// Preload (scored first, right after the toggles)
constexpr double AUTON_PRELOAD_CARRY_HEIGHT = 100.0; // degrees: lift rises to this (from 30) while driving to the goal with the preload
constexpr double AUTON_PRELOAD_DROP_HEIGHT = 10.0;  // degrees: lift lowers to this over the goal before the claw opens
constexpr int AUTON_PRELOAD_CLAW_OPEN_MS = 500;     // claw opens longer for the preload than for pins (AUTON_CLAW_OPEN_MS)
// Turns this many degrees LESS than straight at the goal for the preload, then drives straight.
// Raise it if it still turns too far, lower it (or go negative) if it doesn't turn enough.
constexpr double AUTON_PRELOAD_EXTRA_DIST = -2.0;      // inches. + = drives further toward the goal, - = stops shorter (was -3, now 1" further)
constexpr double AUTON_PRELOAD_TURN_LESS_DEG = -6.8;   // negative = turns that many degrees MORE than straight at the goal (was -4.8, now 2 more)
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
constexpr double AUTON_PIN1_GRAB_TURN_OFFSET = 3.0;   // was 5, turned about 2 degrees too far right
constexpr double AUTON_PIN1_GRAB_EXTRA_DIST = 2.0;    // was 5: the straight slow drive now actually reaches its spot, so 3" less
constexpr double AUTON_PIN1_SCORE_TURN_LESS = 2.0;    // was turning about 2 degrees too far
constexpr double AUTON_PIN1_SCORE_EXTRA_DIST = -1.0;  // was 3, now 4" less into the goal

constexpr double AUTON_PIN2_GRAB_TURN_OFFSET = 8.0;   // was 5, now 3 degrees more right
constexpr double AUTON_PIN1_SLOW_DIST = 6.0;          // inches before the grab spot where it switches to slow (was 3, same as pin 2 now)
constexpr int AUTON_PIN1_APPROACH_SPEED = 70;         // top speed driving up to pin 1 (out of 127). Was full speed (127). Lower = slower
constexpr int AUTON_PIN1_SLOW_SPEED = 20;             // speed for the last slow inches into pin 1 (out of 127). Was AUTON_SLOW_SPEED (30)
constexpr double AUTON_PIN2_SLOW_DIST = 6.0;          // starts slow earlier for pin 2 (it was still knocking it over)
constexpr int AUTON_PIN2_EXTRA_CLAW_MS = 400;         // claw keeps spinning at full speed this much longer on pin 2 before the lift goes up (pin 1 = 0)
constexpr double AUTON_PIN2_GRAB_EXTRA_DIST = 0.0;    // was 3: 3" less, same reason as pin 1
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
constexpr double TOP_START_Y = 62.0;           // right side: AUTON_START_X (62)
constexpr double TOP_START_HEADING = 180.0;    // 90 - 270 = facing down

// The other alliance goal (one tile left of the right-side goal, one tile higher)
constexpr double TOP_GOAL_X = 24.0;            // right side goal was (48, 24)
constexpr double TOP_GOAL_Y = 48.0;

// Pin + cup 1: one tile BELOW this goal (it's the same pin as right-side pin 1, it sits on the diagonal)
constexpr double TOP_PIN1_X = 23.0;            // right side (24, 23) flipped
constexpr double TOP_PIN1_Y = 24.0;
constexpr double TOP_PIN1_APPROACH_HEADING = 128.0;   // 90 - 322: drives in from the upper left

// Pin + cup 2: one tile RIGHT of this goal (same pin as right-side pin 2)
constexpr double TOP_PIN2_X = 48.0;
constexpr double TOP_PIN2_Y = 48.0;
constexpr double TOP_PIN2_APPROACH_HEADING = 47.0;    // 90 - 43: drives in along the diagonal from the lower left

// Preload (same meaning as AUTON_PRELOAD_TURN_LESS_DEG / AUTON_PRELOAD_EXTRA_DIST)
constexpr double TOP_PRELOAD_TURN_LESS_DEG = -6.8;
constexpr double TOP_PRELOAD_EXTRA_DIST = -2.0;

// Per-pin offsets (same meaning as the AUTON_PIN... offsets: + grab turn = more RIGHT)
constexpr double TOP_PIN1_GRAB_TURN_OFFSET = -3.0;    // right side +3 (right) -> -3 (left) when flipped
constexpr double TOP_PIN1_GRAB_EXTRA_DIST = 2.0;
constexpr double TOP_PIN1_SCORE_TURN_LESS = 2.0;
constexpr double TOP_PIN1_SCORE_EXTRA_DIST = -1.0;

constexpr double TOP_PIN2_GRAB_TURN_OFFSET = -8.0;    // right side +8 -> -8 when flipped
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
void autonTick() {
    uint32_t now = pros::millis();

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
    lemlib::Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    double total = (targetX - start.x) * hx + (targetY - start.y) * hy;   // distance to go along the heading

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        lemlib::Pose p = chassis.getPose();
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
            && chassis.getPose().distance(lemlib::Pose(grabX, grabY)) <= AUTON_CLAW_OPEN_DIST) {
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

// Turns to the alliance goal, lifts while driving to it, scores, then lowers and backs off
// stackPosition = which pin/cup on the goal (1 = first, 2 = second...), uses ALLIANCE_HEIGHTS
// turnLessDeg: + = turns that many degrees less than straight at the goal, - = more
// extraDist:   + = drives that many inches further toward the goal, - = stops shorter
void autonScore(int stackPosition, double turnLessDeg, double extraDist) {
    double height = getStackHeight(GOAL_ALLIANCE, stackPosition);

    if (AUTON_LIFT_DURING_TURN) {
        autonSetLift(height);   // start lifting now so it's up sooner
    }

    // Turn toward the goal, adjusted by turnLessDeg (the claw drops to the slow speed during this turn)
    lemlib::Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(autonGoalX - startPose.x, autonGoalY - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    turn -= (turn >= 0 ? turnLessDeg : -turnLessDeg);
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along the aim heading
    lemlib::Pose pose = chassis.getPose();
    double dist = std::hypot(autonGoalX - pose.x, autonGoalY - pose.y);
    double ux = std::sin(aimHeading * PI / 180.0);
    double uy = std::cos(aimHeading * PI / 180.0);

    // How far the robot center travels: until the claw reaches the goal, plus extraDist
    double travel = dist - AUTON_CLAW_REACH + extraDist;
    double scoreX = pose.x + travel * ux;
    double scoreY = pose.y + travel * uy;

    // Lift up while driving toward the goal
    autonSetLift(height);

    if (travel > AUTON_PIN_PRE_SCORE_GAP + 0.5) {
        // Hands off early to the slower last part
        chassis.moveToPoint(scoreX - AUTON_PIN_PRE_SCORE_GAP * ux, scoreY - AUTON_PIN_PRE_SCORE_GAP * uy, 1000,
                            {.minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = AUTON_CHAIN_EXIT_DIST});
        autonWaitMotion();
    }

    // No stop here anymore (see AUTON_WAIT_FOR_LIFT_AT_GOAL): the lift keeps rising while it drives in
    if (AUTON_WAIT_FOR_LIFT_AT_GOAL) {
        autonWaitLift(height, 1200);
    }

    chassis.moveToPose(scoreX, scoreY, aimHeading, 900,   // holds the aim heading
                       {.lead = 0.1, .maxSpeed = AUTON_PIN_SCORE_SPEED,
                        .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_STOP_EXIT_DIST});
    autonWaitMotion();

    // Make sure it really got all the way to the goal before letting go
    {
        lemlib::Pose now = chassis.getPose();
        double shortBy = (scoreX - now.x) * ux + (scoreY - now.y) * uy;   // inches still to go along the aim heading
        if (shortBy > AUTON_SCORE_FINISH_TOLERANCE) {
            autonWaitLift(height, 800);   // only waits if the lift isn't up yet, so the pin clears the goal
            autonDriveStraight(scoreX, scoreY, aimHeading, AUTON_SCORE_FINISH_SPEED, AUTON_SCORE_FINISH_TIMEOUT_MS);
        }
    }

    // Lower the lift and open the claw at the same time
    autonLowerLiftCoast(AUTON_LOW_HEIGHT);   // coasts the last AUTON_LIFT_COAST_DIST degrees
    autonOpenClaw();

    // Halfway down: back up
    double halfway = (height + AUTON_LOW_HEIGHT) / 2.0;
    uint32_t lowerStart = pros::millis();
    while (getDR4BPosition() > halfway && pros::millis() - lowerStart < 600) {
        autonTick();
        pros::delay(10);
    }

    // Back up AUTON_BACKUP_DIST + AUTON_CLEAR_DIST in one move (it used to stop in the middle)
    double backDist = AUTON_BACKUP_DIST + AUTON_CLEAR_DIST;
    chassis.moveToPoint(scoreX - backDist * ux, scoreY - backDist * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();
}

// Drives the preload to the alliance goal, lowers it onto the goal, lets go and backs off
// turnLessDeg / extraDist: same as AUTON_PRELOAD_TURN_LESS_DEG / AUTON_PRELOAD_EXTRA_DIST (or the TOP_ versions)
void autonScorePreload(double turnLessDeg, double extraDist) {
    // Turn toward the goal, but turnLessDeg less than straight at it
    lemlib::Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(autonGoalX - startPose.x, autonGoalY - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    if (std::fabs(turn) > turnLessDeg) {
        turn -= (turn > 0 ? turnLessDeg : -turnLessDeg);
    } else {
        turn = 0.0;
    }
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along that heading, stopping when the claw is as far out as the goal
    lemlib::Pose pose = chassis.getPose();
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
// Red goal A = (-48, -24): the goal the qual part scores on (preload + 2 pins), then 2 matchloads = 5 total.
// Red goal B = (-24, -48): the goal in the bottom quadrant. A pin-only "preload" from the matchloader, then 4 matchloads.

// --- Matchloader (bottom of the left wall) ---
constexpr double SKILLS_LOADER_Y = -60.0;          // Y of the matchloader opening (robot lines up on this)
constexpr double SKILLS_LOADER_GRAB_X = -61.5;     // robot center X when the claw is right under the loader opening. TUNE THIS FIRST
constexpr double SKILLS_LOADER_LINEUP_X = -48.0;   // where the straight drive into the loader starts (13.5" out)
// The robot turns to face the loader HERE, 4" further from the bottom wall than the loader line, because
// its corners swing out 11.9" when it turns and would hit the wall if it turned on the loader line itself.
constexpr double SKILLS_LOADER_TURN_X = -40.0;
constexpr double SKILLS_LOADER_TURN_Y = -56.0;
constexpr int SKILLS_LOADER_DRIVE_IN_SPEED = 45;   // speed of the straight drive into the loader (out of 127)
constexpr double SKILLS_LOADER_BACKOUT_DIST = 8.0; // straight back out of the loader this far before anything else
// After backing out it goes here (away from the wall) before turning to a goal, so the corners clear the wall
constexpr double SKILLS_SCORE_SPOT_X = -44.0;
constexpr double SKILLS_SCORE_SPOT_Y = -54.0;
constexpr int SKILLS_MATCHLOAD_HOLD_MS = 2000;     // claw spins at full speed this long at the loader before lifting
constexpr int SKILLS_PIN_ONLY_WAIT_MS = 1000;      // claw stays open and still under the loader this long for the pin-only load

// --- Goal B (bottom quadrant) ---
constexpr double SKILLS_GOAL_B_X = -24.0;
constexpr double SKILLS_GOAL_B_Y = -48.0;

// --- Scoring offsets for the new scores (same meaning as the AUTON_ ones). Start at 0 and tune ---
constexpr double SKILLS_A_SCORE_TURN_LESS = 0.0;   // matchloads onto goal A
constexpr double SKILLS_A_SCORE_EXTRA_DIST = 0.0;
constexpr double SKILLS_B_PRELOAD_TURN_LESS = 0.0; // pin-only "preload" onto goal B
constexpr double SKILLS_B_PRELOAD_EXTRA_DIST = -2.0;   // same as the qual preload (-2) since it scores the same way
constexpr double SKILLS_B_SCORE_TURN_LESS = 0.0;   // matchloads onto goal B
constexpr double SKILLS_B_SCORE_EXTRA_DIST = 0.0;

// --- Bottom-wall toggle + park ---
// Route from goal B to the toggle goes up through the gap between goal A and goal B, then around the
// right side of goal B (there isn't room for the robot to squeeze between goal B and the bottom wall).
constexpr double SKILLS_WAY1_X = -36.0, SKILLS_WAY1_Y = -36.0;   // gap between goal A and goal B
constexpr double SKILLS_WAY2_X = -14.0, SKILLS_WAY2_Y = -30.0;   // above-right of goal B
constexpr double SKILLS_TOGGLE_X = 0.0;            // toggle is in the middle of the bottom wall
constexpr double SKILLS_TOGGLE_TURN_Y = -48.0;     // turns to face the toggle here (far enough from the wall to swing)
constexpr double SKILLS_TOGGLE_START_Y = -59.0;    // robot center 4" before the claw touches the toggle
constexpr double SKILLS_TOGGLE_PUSH_DIST = 5.0;    // drives this far into the toggle each time (a little extra to make sure it flips)
constexpr int SKILLS_TOGGLE_PUSH_SPEED = 70;
constexpr int SKILLS_TOGGLE_PUSH_MS = 700;         // max time per push (the wall stops it)
constexpr double SKILLS_TOGGLE_LIFT_HEIGHT = 30.0; // lift height while pushing the toggle with the claw
// Park: backs away from the toggle to a waypoint, then straight back into the middle of the field.
// x = -11 keeps the robot clear of goal B, the small pin at the bottom of the center diamond, and the center goal.
constexpr double SKILLS_PARK_WAY_X = -11.0, SKILLS_PARK_WAY_Y = -42.0;
constexpr double SKILLS_PARK_X = -11.0, SKILLS_PARK_Y = -13.0;

// --- Time guard ---
// 60 s is NOT enough for everything (about 85-90 s of driving). Before each matchload cycle the robot checks
// the clock; if that cycle plus the toggle + park won't fit, it skips the rest and goes straight to the toggle
// and park, so those points are never lost.
constexpr int SKILLS_PINS_PER_GOAL = 5;            // fill each goal to this many (uses ALLIANCE_HEIGHTS spots 1-5)
constexpr uint32_t SKILLS_TIME_LIMIT_MS = 60000;
constexpr uint32_t SKILLS_SAFETY_MS = 1000;        // spare time kept at the end
constexpr uint32_t SKILLS_END_RESERVE_MS = 8000;   // estimated time for toggle (x2) + park from a goal
constexpr uint32_t SKILLS_CYCLE_MS = 9500;         // estimated time for one matchload cycle (loader -> goal)
constexpr uint32_t SKILLS_PIN_ONLY_CYCLE_MS = 9000; // estimated time for the pin-only load -> goal B
// =====================================================================

// Skills opening = right-wall qual route turned 180 degrees (same tuning, same turn directions)
const AutonRoute SKILLS_ROUTE = {
    -AUTON_START_X, -AUTON_START_Y, std::fmod(AUTON_START_HEADING + 180.0, 360.0),
    -AUTON_GOAL_X, -AUTON_GOAL_Y,
    AUTON_PRELOAD_TURN_LESS_DEG, AUTON_PRELOAD_EXTRA_DIST,
    -AUTON_PIN1_X, -AUTON_PIN1_Y, std::fmod(AUTON_PIN1_APPROACH_HEADING + 180.0, 360.0),
    AUTON_PIN1_GRAB_TURN_OFFSET, AUTON_PIN1_GRAB_EXTRA_DIST, AUTON_PIN1_SCORE_TURN_LESS, AUTON_PIN1_SCORE_EXTRA_DIST,
    -AUTON_PIN2_X, -AUTON_PIN2_Y, std::fmod(AUTON_PIN2_APPROACH_HEADING + 180.0, 360.0),
    AUTON_PIN2_GRAB_TURN_OFFSET, AUTON_PIN2_GRAB_EXTRA_DIST, AUTON_PIN2_SCORE_TURN_LESS, AUTON_PIN2_SCORE_EXTRA_DIST
};

uint32_t skillsStartMs = 0;

// True if a step that takes neededMs still leaves time for the toggle + park
bool skillsHaveTime(uint32_t neededMs) {
    uint32_t elapsed = pros::millis() - skillsStartMs;
    return elapsed + neededMs + SKILLS_END_RESERVE_MS + SKILLS_SAFETY_MS <= SKILLS_TIME_LIMIT_MS;
}

// Drives straight along `heading` (both sides same speed, no turning). + distance = forward, - = backward.
void autonDriveDist(double distance, double heading, int speed, uint32_t timeoutMs) {
    lemlib::Pose start = chassis.getPose();
    double hx = std::sin(heading * PI / 180.0);
    double hy = std::cos(heading * PI / 180.0);
    int dir = distance >= 0 ? 1 : -1;
    double target = std::fabs(distance);

    uint32_t t0 = pros::millis();
    while (pros::millis() - t0 < timeoutMs) {
        lemlib::Pose p = chassis.getPose();
        double done = ((p.x - start.x) * hx + (p.y - start.y) * hy) * dir;
        if (done >= target - 0.25) break;
        chassis.tank(dir * speed, dir * speed, true);
        autonTick();
        pros::delay(10);
    }
    chassis.tank(0, 0, true);
}

// Small lift after grabbing (same as after a field pin: AUTON_PIN_PICKUP_RISE degrees)
void skillsSmallLift() {
    autonLiftGrounding = false;
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
    dr4bTargetForced = true;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    DR4B.brake();
    dr4bManualMode = false;
}

// Lines up with the matchloader and drives STRAIGHT into it (claw facing it), with the claw at clawSpeed
void skillsDriveIntoLoader(int clawSpeed) {
    autonStartGrounding();   // lift all the way down while driving

    // 1. Get to the turn spot (only if not already there)
    lemlib::Pose p = chassis.getPose();
    if (std::hypot(SKILLS_LOADER_TURN_X - p.x, SKILLS_LOADER_TURN_Y - p.y) > 3.0) {
        chassis.turnToPoint(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, 800,
                            {.minSpeed = AUTON_TURN_MIN_SPEED, .earlyExitRange = AUTON_TURN_EXIT_DEG});
        autonWaitMotion();
        chassis.moveToPoint(SKILLS_LOADER_TURN_X, SKILLS_LOADER_TURN_Y, 1500);
        autonWaitMotion();
    }

    // 2. Face the loader (left wall = heading 270) away from the bottom wall, then slide over onto the loader line
    chassis.turnToHeading(270, AUTON_AIM_TURN_TIMEOUT_MS,
                          {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();
    chassis.moveToPose(SKILLS_LOADER_LINEUP_X, SKILLS_LOADER_Y, 270, 1500, {.lead = 0.5});
    autonWaitMotion();
    chassis.turnToHeading(270, AUTON_AIM_TURN_TIMEOUT_MS,
                          {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // 3. Lift must be down, then straight in with the claw going
    {
        uint32_t groundWait = pros::millis();
        while (getDR4BPosition() > AUTON_GROUND_DONE_DEG + 2.0 && pros::millis() - groundWait < 400) {
            autonTick();
            pros::delay(10);
        }
    }
    autonClawStopAtMs = 0;
    autonClawSlowAtMs = 0;
    claw.move_velocity(clawSpeed);
    autonDriveStraight(SKILLS_LOADER_GRAB_X, SKILLS_LOADER_Y, 270, SKILLS_LOADER_DRIVE_IN_SPEED, 1500);
}

// Straight back out of the loader, then away from the wall to the scoring spot
void skillsBackOutOfLoader() {
    autonDriveDist(-SKILLS_LOADER_BACKOUT_DIST, 270, 70, 800);
    chassis.moveToPoint(SKILLS_SCORE_SPOT_X, SKILLS_SCORE_SPOT_Y, 1000,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();
}

// One matchload (pin + cup): into the loader spinning, hold, small lift, out, score on (goalX, goalY)
void skillsMatchloadCycle(double goalX, double goalY, int stack, double turnLess, double extraDist) {
    skillsDriveIntoLoader(AUTON_APPROACH_CLAW_SPEED);

    // Hold at the loader with the claw spinning at full speed
    uint32_t holdStart = pros::millis();
    while (pros::millis() - holdStart < static_cast<uint32_t>(SKILLS_MATCHLOAD_HOLD_MS)) {
        autonTick();
        pros::delay(10);
    }

    // Lift a little, slow the claw, back out
    skillsSmallLift();
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);
    skillsBackOutOfLoader();

    autonGoalX = goalX;
    autonGoalY = goalY;
    autonScore(stack, turnLess, extraDist);
}

// Pin-only load for goal B: claw open going in, stops open under the loader, closes on the pin, scores like the preload
void skillsPinOnlyCycle() {
    skillsDriveIntoLoader(-200);   // claw spins open while driving in
    claw.move_velocity(0);         // stops open, right under the loader

    uint32_t waitStart = pros::millis();
    while (pros::millis() - waitStart < static_cast<uint32_t>(SKILLS_PIN_ONLY_WAIT_MS)) {
        autonTick();
        pros::delay(10);
    }

    // Close on the pin (same stall check as the field grab, gives up after 400 ms)
    claw.move_velocity(200);
    {
        uint32_t start = pros::millis();
        bool slow = false;
        uint32_t slowStart = 0;
        while (pros::millis() - start < 400) {
            uint32_t now = pros::millis();
            double clawSpeed = std::fabs(claw.get_actual_velocity());
            if (now - start >= CLAW_STALL_GRACE_MS && clawSpeed < CLAW_STALL_SPEED) {
                if (!slow) { slow = true; slowStart = now; }
                if (now - slowStart >= CLAW_STALL_CONFIRM_MS) break;
            } else {
                slow = false;
            }
            autonTick();
            pros::delay(10);
        }
    }

    skillsSmallLift();
    claw.move_velocity(AUTON_CLAW_HOLD_SPEED);
    skillsBackOutOfLoader();

    // Score it on goal B the same way as the qual preload (carry height, lower to 10, open, back off)
    autonGoalX = SKILLS_GOAL_B_X;
    autonGoalY = SKILLS_GOAL_B_Y;
    autonSetLift(AUTON_PRELOAD_CARRY_HEIGHT);
    autonScorePreload(SKILLS_B_PRELOAD_TURN_LESS, SKILLS_B_PRELOAD_EXTRA_DIST);
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

    // Into the toggle and back, twice
    autonDriveDist(SKILLS_TOGGLE_PUSH_DIST, 180, SKILLS_TOGGLE_PUSH_SPEED, SKILLS_TOGGLE_PUSH_MS);
    autonDriveDist(-SKILLS_TOGGLE_PUSH_DIST, 180, SKILLS_TOGGLE_PUSH_SPEED, SKILLS_TOGGLE_PUSH_MS);
    autonDriveDist(SKILLS_TOGGLE_PUSH_DIST, 180, SKILLS_TOGGLE_PUSH_SPEED, SKILLS_TOGGLE_PUSH_MS);

    // Immediately back to the middle to park
    chassis.moveToPoint(SKILLS_PARK_WAY_X, SKILLS_PARK_WAY_Y, 1500,
                        {.forwards = false, .minSpeed = AUTON_CHAIN_MIN_SPEED, .earlyExitRange = 3.0});
    autonWaitMotion();
    chassis.moveToPoint(SKILLS_PARK_X, SKILLS_PARK_Y, 2000, {.forwards = false});
    autonWaitMotion();
}

// The full 1-minute skills routine
void skillsAuton() {
    skillsStartMs = pros::millis();

    // 1. Same as the qual auton (turned around): toggle, preload, pin 1, pin 2 -> goal A has 3
    runThreePinRoute(SKILLS_ROUTE);

    const double goalAX = SKILLS_ROUTE.goalX;
    const double goalAY = SKILLS_ROUTE.goalY;

    // 2. Matchloads onto goal A until it has 5 (stack spots 4 and 5)
    bool outOfTime = false;
    for (int stack = AUTON_PIN2_STACK + 1; stack <= SKILLS_PINS_PER_GOAL; stack++) {
        if (!skillsHaveTime(SKILLS_CYCLE_MS)) { outOfTime = true; break; }
        skillsMatchloadCycle(goalAX, goalAY, stack, SKILLS_A_SCORE_TURN_LESS, SKILLS_A_SCORE_EXTRA_DIST);
    }

    // 3. Pin-only load -> goal B (spot 1), then matchloads onto goal B (spots 2-5)
    if (!outOfTime && skillsHaveTime(SKILLS_PIN_ONLY_CYCLE_MS)) {
        skillsPinOnlyCycle();
        for (int stack = 2; stack <= SKILLS_PINS_PER_GOAL; stack++) {
            if (!skillsHaveTime(SKILLS_CYCLE_MS)) break;
            skillsMatchloadCycle(SKILLS_GOAL_B_X, SKILLS_GOAL_B_Y, stack,
                                 SKILLS_B_SCORE_TURN_LESS, SKILLS_B_SCORE_EXTRA_DIST);
        }
    }

    // 4. Always finish with the toggle (x2) and park
    skillsToggleAndPark();
}

// Called from autonomous() in main.cpp. Runs skills if AUTON_RUN_SKILLS, otherwise the qual route
// picked with AUTON_START_ON_TOP.
void manyPinAlliance() {
    if (AUTON_RUN_SKILLS) {
        skillsAuton();
    } else {
        runThreePinRoute(AUTON_START_ON_TOP ? TOP_ROUTE : RIGHT_ROUTE);
    }
}