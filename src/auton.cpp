// Autonomous: preload + 2 pins on the alliance goal. Called from autonomous() in main.cpp.
#include "robot.hpp"

// Everything in this file is private to it (so the other auton files can use the same names),
// except the routine at the bottom that autonomous() in main.cpp calls.
namespace {

// =====================================================================
//              AUTON: 2 PINS ON THE ALLIANCE GOAL (tuning)
// =====================================================================
// Field coordinates are in inches, with (0, 0) at the center of the field (RW-Template's field).
// +X = toward the right wall (the wall the robot starts on), +Y = toward the top of the field map.
// Walls are at +-70.25 (the inside of the perimeter, RW's field_half_size). Tiles are 24", so field elements
// sit on multiples of 24. All positions below are the IDEAL field spots (the old field-tuned offsets were
// reset to 0): with MCL / distance resets the robot knows where it is, so the spots don't need fudging.
// Headings: 0 = facing the top of the map, 90 = facing right, 180 = facing down, 270 = facing left.
// One tile = 24 inches.

// Start: back of the robot against the right wall, in the middle of the wall, claw facing into the field.
// The robot is 18" long, so its center is 9" off the wall (wall is at X = 70.25).
constexpr double AUTON_START_X = 70.25 - 9.0;   // 61.25 (was field-tuned to 62)
constexpr double AUTON_START_Y = 0;
constexpr double AUTON_START_HEADING = 270.0;

// Alliance goal (the one being scored on)
constexpr double AUTON_GOAL_X = 48.0;
constexpr double AUTON_GOAL_Y = 24.0;

// Pin + cup 1: one tile left of the goal
constexpr double AUTON_PIN1_X = 24.0;
constexpr double AUTON_PIN1_Y = 24.0;   // tile corner (was field-tuned to 23)
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
constexpr double AUTON_BACKUP_DIST = 4.0;       // backs up this far once the lift has started down (see AUTON_SCORE_BACKUP_DROP_DEG)
constexpr double AUTON_SCORE_BACKUP_DROP_DEG = 60.0;  // after letting go of a pin, starts backing up once the lift has dropped this many degrees
                                                      // (it used to wait until the lift was halfway down). Raise it if it drags the pin off
// Aiming turn toward the goal when scoring a pin (the preload keeps AUTON_AIM_EXIT_DEG / AUTON_AIM_MIN_SPEED)
constexpr double AUTON_SCORE_AIM_EXIT_DEG = 2.5;      // done once this close (was 1.5). Lower = more exact but slower
constexpr int AUTON_SCORE_AIM_MIN_SPEED = 25;         // speed kept at the end of the turn (was 10)
constexpr double AUTON_CLEAR_DIST = 6.0;        // then backs up this much more so the robot's corners don't hit the goal when it turns (was 4, +2" more)

// Start: toggle mechanism (back of the robot)
constexpr double AUTON_TOGGLE_LIFT_HEIGHT = 30.0;   // degrees: lift goes up this much first so the toggles clear
constexpr double AUTON_TOGGLE_ROTATIONS = 2.0;      // full turns of the toggle (output side). Never goes past this
constexpr double AUTON_TOGGLE_GEAR_RATIO = 2.0;     // motor turns per toggle turn (1:2 = motor spins 2x for 1 toggle turn)
constexpr int AUTON_TOGGLE_SPEED = 200;             // motor rpm (same direction as holding A in driver control)
constexpr int AUTON_TOGGLE_TIMEOUT_MS = 2500;       // the toggle is switched off after this, in case it gets stuck
constexpr double AUTON_TOGGLE_START_LIFT = 8.0;     // degrees: toggle starts spinning once the lift is this high (doesn't wait for the full lift). Was 15, lower = starts sooner
// The robot waits only AUTON_TOGGLE_HEAD_START_MS after the toggle starts spinning, then starts turning to the goal.
// The toggle still finishes its 2 turns exactly on its own and stops there.
constexpr int AUTON_TOGGLE_HEAD_START_MS = 300;     // short wait before driving off so it gets both toggles (raise if it still only gets one)

// Preload (scored first, right after the toggles)
constexpr double AUTON_PRELOAD_CARRY_HEIGHT = 100.0; // degrees: lift rises to this (from 30) while driving to the goal with the preload
constexpr double AUTON_PRELOAD_DROP_HEIGHT = 10.0;  // degrees: lift lowers to this over the goal before the claw opens
constexpr int AUTON_PRELOAD_CLAW_OPEN_MS = 500;     // claw opens longer for the preload than for pins (AUTON_CLAW_OPEN_MS)
constexpr int AUTON_PRELOAD_RELEASE_MS = 200;       // starts backing away this long after the claw starts opening (it keeps opening
                                                    // while backing up). Was the full AUTON_PRELOAD_CLAW_OPEN_MS (500)
// Turns this many degrees LESS than straight at the goal for the preload, then drives straight.
// Raise it if it still turns too far, lower it (or go negative) if it doesn't turn enough.
constexpr double AUTON_PRELOAD_EXTRA_DIST = 0.0;       // inches. + = drives further toward the goal, - = stops shorter (was field-tuned to -2)
constexpr double AUTON_PRELOAD_TURN_LESS_DEG = 0.0;    // negative = turns that many degrees MORE than straight at the goal (was field-tuned to -6.8)
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
constexpr double AUTON_PIN1_GRAB_EXTRA_DIST = 0.0;    // (was field-tuned to 2)
constexpr double AUTON_PIN1_SCORE_TURN_LESS = 0.0;    // (was field-tuned to 2)
constexpr double AUTON_PIN1_SCORE_EXTRA_DIST = 0.0;   // (was field-tuned to -1)

constexpr double AUTON_PIN2_GRAB_TURN_OFFSET = 0.0;   // (was field-tuned to 8)
constexpr double AUTON_PIN1_SLOW_DIST = 6.0;          // inches before the grab spot where it switches to slow (was 3, same as pin 2 now)
constexpr int AUTON_PIN1_APPROACH_SPEED = 70;         // top speed driving up to pin 1 (out of 127). Was full speed (127). Lower = slower
constexpr int AUTON_PIN1_SLOW_SPEED = 20;             // speed for the last slow inches into pin 1 (out of 127). Was AUTON_SLOW_SPEED (30)
constexpr double AUTON_PIN1_LOWER_SPEED = 480.0;      // degrees/second the lift lowers at after scoring pin 1 (slightly slower than the normal auton 560)
constexpr double AUTON_PIN2_LIFT_BEFORE_TURN = 50.0;  // degrees: after grabbing pin 2, the lift gets this high before the robot turns to the goal,
                                                      // then keeps going up to scoring height while it turns. 0 = turn and lift at the same time
constexpr double AUTON_PIN2_SLOW_DIST = 6.0;          // starts slow earlier for pin 2 (it was still knocking it over)
constexpr int AUTON_PIN2_EXTRA_CLAW_MS = 400;         // claw keeps spinning at full speed this much longer on pin 2 before the lift goes up (pin 1 = 0)
constexpr double AUTON_PIN2_GRAB_EXTRA_DIST = 0.0;
constexpr double AUTON_PIN2_SCORE_TURN_LESS = 0.0;    // not tested yet
constexpr double AUTON_PIN2_SCORE_EXTRA_DIST = 0.0;   // not tested yet

// Speed-ups: instead of coming to a full stop between moves that go straight into each other
// (fast drive -> slow drive, turn -> drive), the first move hands off early while still rolling.
constexpr int AUTON_CHAIN_MIN_SPEED = 60;        // drive speed kept at the hand-off (out of 127) (was 40)
constexpr double AUTON_CHAIN_EXIT_DIST = 1.5;    // inches early the fast part hands off to the slow part
// Turn toward each pin -> drive to it. Keep these small: handing off much earlier/faster (25 degrees at speed 70)
// made the robot spin in circles, because the drive started while it was still turning fast and far off.
constexpr int AUTON_TURN_MIN_SPEED = 40;         // turn speed kept at the hand-off
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

// Lift grounding state (see AUTON_GROUND_PUSH_MV)
bool autonLiftGrounding = false;
uint32_t autonGroundStartMs = 0;

// Coast lowering state (see AUTON_LIFT_COAST_DIST)
bool autonLiftCoasting = false;   // true while lowering and waiting to reach the coast point
double autonCoastAtDeg = 0.0;     // lift height where the motors switch to coast

// Auton claw timers (0 = nothing scheduled)
uint32_t autonClawSlowAtMs = 0;   // when the claw drops from full speed to the slow grabbing speed
uint32_t autonClawStopAtMs = 0;   // when the claw stops spinning backward after opening
uint32_t autonToggleStopAtMs = 0; // when the toggle gets switched off in case it's stuck (AUTON_TOGGLE_TIMEOUT_MS)

// Runs the claw timers. Called every 10 ms while auton waits on anything.
void autonTick() {
    uint32_t now = pros::millis();

    if (autonClawSlowAtMs != 0 && now >= autonClawSlowAtMs) {
        claw.move_velocity(AUTON_CLAW_HOLD_SPEED);
        autonClawSlowAtMs = 0;
    }

    if (autonToggleStopAtMs != 0 && now >= autonToggleStopAtMs) {
        toggleMechanism.move_velocity(0);   // normally it has already finished its 2 turns and stopped by itself
        autonToggleStopAtMs = 0;
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
// straightApproach: true = turn, then drive straight there right away (no curve). false = curve in (moveToPose)
void autonGrabPin(double pinX, double pinY, double approachHeading, double turnOffset, double extraDist,
                  double slowDist, int approachSpeed, int slowSpeed, int extraClawMs, bool straightApproach) {
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
    if (straightApproach) {
        // Straight at the point it just turned to face, so it starts driving the moment the turn is done
        // (the curved drive turned some more first). The square-up turn below lines it up with the pin.
        chassis.moveToPoint(slowX, slowY, 2500, {.maxSpeed = static_cast<float>(approachSpeed)});
    } else {
        chassis.moveToPose(slowX, slowY, grabHeading, 2500, {.lead = 0.3, .maxSpeed = static_cast<float>(approachSpeed)});
    }
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

// Turns to the alliance goal, lifts while driving to it, scores, then lowers and backs off
// stackPosition = which pin/cup on the goal (1 = first, 2 = second...), uses ALLIANCE_HEIGHTS
// turnLessDeg: + = turns that many degrees less than straight at the goal, - = more
// extraDist:   + = drives that many inches further toward the goal, - = stops shorter
// liftBeforeTurnDeg: > 0 = waits for the lift to get this high before turning (keeps lifting while it turns)
// lowerSpeed:  > 0 = lift lowers at this speed (degrees/second) after letting go, 0 = normal auton speed
void autonScore(int stackPosition, double turnLessDeg, double extraDist, double liftBeforeTurnDeg, double lowerSpeed) {
    double height = getStackHeight(GOAL_ALLIANCE, stackPosition);

    if (AUTON_LIFT_DURING_TURN || liftBeforeTurnDeg > 0.0) {
        autonSetLift(height);   // start lifting now so it's up sooner
    }

    // Lift it off the ground first, then turn (the lift keeps going up to full height during the turn)
    if (liftBeforeTurnDeg > 0.0) {
        uint32_t liftStart = pros::millis();
        while (getDR4BPosition() < liftBeforeTurnDeg && pros::millis() - liftStart < 800) {
            autonTick();
            pros::delay(10);
        }
    }

    // Turn toward the goal, adjusted by turnLessDeg (the claw drops to the slow speed during this turn)
    Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(AUTON_GOAL_X - startPose.x, AUTON_GOAL_Y - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    turn -= (turn >= 0 ? turnLessDeg : -turnLessDeg);
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_SCORE_AIM_MIN_SPEED, .earlyExitRange = AUTON_SCORE_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along the aim heading
    Pose pose = chassis.getPose();
    double dist = std::hypot(AUTON_GOAL_X - pose.x, AUTON_GOAL_Y - pose.y);
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
        Pose now = chassis.getPose();
        double shortBy = (scoreX - now.x) * ux + (scoreY - now.y) * uy;   // inches still to go along the aim heading
        if (shortBy > AUTON_SCORE_FINISH_TOLERANCE) {
            autonWaitLift(height, 800);   // only waits if the lift isn't up yet, so the pin clears the goal
            autonDriveStraight(scoreX, scoreY, aimHeading, AUTON_SCORE_FINISH_SPEED, AUTON_SCORE_FINISH_TIMEOUT_MS);
        }
    }

    // Lower the lift and open the claw at the same time
    double normalLowerSpeed = dr4bSlewDown;
    if (lowerSpeed > 0.0) {
        dr4bSlewDown = lowerSpeed;
    }
    autonLowerLiftCoast(AUTON_LOW_HEIGHT);   // coasts the last AUTON_LIFT_COAST_DIST degrees
    autonOpenClaw();

    // Back up once the lift has dropped AUTON_SCORE_BACKUP_DROP_DEG (or is halfway down, if that comes first)
    double backUpAt = std::max(height - AUTON_SCORE_BACKUP_DROP_DEG, (height + AUTON_LOW_HEIGHT) / 2.0);
    uint32_t lowerStart = pros::millis();
    while (getDR4BPosition() > backUpAt && pros::millis() - lowerStart < 600) {
        autonTick();
        pros::delay(10);
    }

    // Back up AUTON_BACKUP_DIST + AUTON_CLEAR_DIST in one move (it used to stop in the middle)
    double backDist = AUTON_BACKUP_DIST + AUTON_CLEAR_DIST;
    chassis.moveToPoint(scoreX - backDist * ux, scoreY - backDist * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();

    dr4bSlewDown = normalLowerSpeed;   // back to the normal auton lowering speed
}

// Drives the preload to the alliance goal, lowers it onto the goal, lets go and backs off
void autonScorePreload() {
    // Turn toward the goal, but AUTON_PRELOAD_TURN_LESS_DEG less than straight at it
    Pose startPose = chassis.getPose();
    double headingToGoal = std::atan2(AUTON_GOAL_X - startPose.x, AUTON_GOAL_Y - startPose.y) * 180.0 / PI;
    double turn = std::fmod(headingToGoal - startPose.theta + 540.0, 360.0) - 180.0;   // -180 to 180
    if (std::fabs(turn) > AUTON_PRELOAD_TURN_LESS_DEG) {
        turn -= (turn > 0 ? AUTON_PRELOAD_TURN_LESS_DEG : -AUTON_PRELOAD_TURN_LESS_DEG);
    } else {
        turn = 0.0;
    }
    double aimHeading = startPose.theta + turn;

    chassis.turnToHeading(aimHeading, AUTON_AIM_TURN_TIMEOUT_MS, {.minSpeed = AUTON_AIM_MIN_SPEED, .earlyExitRange = AUTON_AIM_EXIT_DEG});
    autonWaitMotion();

    // Drive straight along that heading, stopping when the claw is as far out as the goal
    Pose pose = chassis.getPose();
    double dist = std::hypot(AUTON_GOAL_X - pose.x, AUTON_GOAL_Y - pose.y);
    double ux = std::sin(aimHeading * PI / 180.0);
    double uy = std::cos(aimHeading * PI / 180.0);

    // Robot center at the drop spot (adjusted by AUTON_PRELOAD_EXTRA_DIST)
    double travel = dist - AUTON_CLAW_REACH + AUTON_PRELOAD_EXTRA_DIST;
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
    {
        // Short wait so the claw lets go, then back up while it finishes opening
        uint32_t releaseStart = pros::millis();
        while (autonClawStopAtMs != 0 && pros::millis() - releaseStart < AUTON_PRELOAD_RELEASE_MS) {
            autonTick();
            pros::delay(10);
        }
    }

    // Back straight away from the goal
    chassis.moveToPoint(scoreX - AUTON_PRELOAD_BACKUP * ux, scoreY - AUTON_PRELOAD_BACKUP * uy, 800,
                        {.forwards = false, .minSpeed = AUTON_STOP_MIN_SPEED, .earlyExitRange = AUTON_BACKUP_EXIT_DIST});
    autonWaitMotion();

    // Back up to grab height for the pins (happens while it drives to pin 1)
    autonLowerLiftCoast(AUTON_LOW_HEIGHT);   // coasts the last AUTON_LIFT_COAST_DIST degrees
}

}  // namespace

// Scores the preload + 2 pins and cups on the alliance goal
void twoPinAlliance() {
    chassis.setPose(AUTON_START_X, AUTON_START_Y, AUTON_START_HEADING);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);   // driver control sets it back to coast
    localizeWithWalls();   // distance sensors check the start spot (only if use_distance_resets is on)

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

    // No waiting for the toggle: move_absolute keeps going and stops itself exactly at 2 turns
    // while the robot starts turning toward the goal. If it gets stuck, autonTick switches it
    // off after AUTON_TOGGLE_TIMEOUT_MS.
    autonToggleStopAtMs = pros::millis() + AUTON_TOGGLE_TIMEOUT_MS;

    // Short head start for the toggle before the robot drives off
    {
        uint32_t headStart = pros::millis();
        while (pros::millis() - headStart < static_cast<uint32_t>(AUTON_TOGGLE_HEAD_START_MS)) {
            autonTick();
            pros::delay(10);
        }
    }

    // Lift goes straight from 30 up to the preload carry height now, while the robot turns
    // toward the goal (it used to wait until the turn was finished)
    autonSetLift(AUTON_PRELOAD_CARRY_HEIGHT);

    // Lift to grab height (AUTON_LOW_HEIGHT) for the preload drive
    // autonSetLift(AUTON_LOW_HEIGHT);

    // 3. Preload -> goal (lower to 10, open claw, back off)
    autonScorePreload();

    // 4. Pin + cup 1 (one tile left of the goal) -> goal
    autonGrabPin(AUTON_PIN1_X, AUTON_PIN1_Y, AUTON_PIN1_APPROACH_HEADING,
                 AUTON_PIN1_GRAB_TURN_OFFSET, AUTON_PIN1_GRAB_EXTRA_DIST, AUTON_PIN1_SLOW_DIST,
                 AUTON_PIN1_APPROACH_SPEED, AUTON_PIN1_SLOW_SPEED, 0,
                 true);   // pin 1: turn, then drive straight there right away
    autonScore(AUTON_PIN1_STACK, AUTON_PIN1_SCORE_TURN_LESS, AUTON_PIN1_SCORE_EXTRA_DIST,
               0.0, AUTON_PIN1_LOWER_SPEED);   // pin 1: lifts while turning, lowers slightly slower

    // 5. Pin + cup 2 (one tile above the goal) -> goal
    autonGrabPin(AUTON_PIN2_X, AUTON_PIN2_Y, AUTON_PIN2_APPROACH_HEADING,
                 AUTON_PIN2_GRAB_TURN_OFFSET, AUTON_PIN2_GRAB_EXTRA_DIST, AUTON_PIN2_SLOW_DIST,
                 127, AUTON_SLOW_SPEED, AUTON_PIN2_EXTRA_CLAW_MS,   // pin 2: full speed in, same slow speed as before, longer claw spin
                 false);  // pin 2: still curves in, so it stays clear of the goal
    autonScore(AUTON_PIN2_STACK, AUTON_PIN2_SCORE_TURN_LESS, AUTON_PIN2_SCORE_EXTRA_DIST,
               AUTON_PIN2_LIFT_BEFORE_TURN, 0.0);   // pin 2: lift it off the ground first, then turn while lifting the rest

    claw.move_velocity(0);
}