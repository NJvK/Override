// QUAL autons (15 s): the opening of the skills run (src/skillsauto.cpp) and nothing else:
//   motor toggle -> preload on the alliance goal -> field pin 1 on the goal -> field pin 2 on the goal, then stops
// The driving itself is shared with skills (runOpeningRoute), but every spot and tuning value is here, separately for
// each start, so tuning one doesn't change the others (or skills). The 15 s timing (shorter pickups and release, the
// quicker pin approach) is in skillsauto.cpp's QUALS_ block; skills keeps its own.
//
// Four starts, one per alliance wall (each alliance's two goals sit in its corner of the field):
//   RED LEFT    back on the LEFT wall (the red wall), centered, facing right. Red goal (-47.09, -23.55).
//               This is exactly the skills opening.
//   RED BOTTOM  back on the BOTTOM wall, centered, facing up. Red goal (-23.55, -47.09).
//               = RED LEFT flipped across the field's diagonal (the white line the pins sit on).
//   BLUE RIGHT  back on the RIGHT wall (the blue wall), centered, facing left. Blue goal (47.09, 23.55).
//               = RED LEFT turned 180 degrees around the field center.
//   BLUE TOP    back on the TOP wall, centered, facing down. Blue goal (23.55, 47.09).
//               = RED BOTTOM turned 180 degrees.
// Both alliances use the two field pins on the diagonal next to their goals (red: (-23.55, -23.55) and (-47.09, -47.09),
// blue: (23.55, 23.55) and (47.09, 47.09)).
//
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25 (the field map the simulator draws: red wall on
// the left, blue wall on the right). Headings: 0 = up, 90 = right, clockwise.
//
// How the tuning carries over between the starts:
//   turned 180 degrees (red <-> blue on the same kind of wall): every value stays the same
//   flipped across the diagonal (left/right wall <-> bottom/top wall): left and right swap, so GRAB_TURN changes sign;
//   the distance values stay the same
#include "robot.hpp"

// =====================================================================
//   Tuning meanings (same for every start)
// =====================================================================
// PRELOAD_EXTRA_DIST: + = preload goes further onto the goal, - = shorter
// PIN_GRAB_TURN:      degrees. + = turns that much more to the RIGHT (clockwise) before driving into the pin, - = left
// PIN_GRAB_EXTRA:     inches. + = drives further into the pin, - = stops shorter
// PIN_SCORE_EXTRA:    inches. + = the pin goes further onto the goal, - = shorter
// LAST_BACKUP:        backs off the goal this far after pin 2 (0: auton ends there, claw open)

// =====================================================================
//   RED LEFT - tuning (starts with the skills robot tuning: pin 1 -4", pin 2 -2", preload -2")
// =====================================================================
constexpr double RL_PRELOAD_EXTRA_DIST = -2.0;
constexpr double RL_PIN1_GRAB_TURN = 0.0;
constexpr double RL_PIN1_GRAB_EXTRA = -4.0;
constexpr double RL_PIN1_SCORE_EXTRA = 0.0;
constexpr double RL_PIN2_GRAB_TURN = 0.0;
constexpr double RL_PIN2_GRAB_EXTRA = -2.0;
constexpr double RL_PIN2_SCORE_EXTRA = 0.0;
constexpr double RL_LAST_BACKUP = 0.0;

const OpeningRoute QUAL_RED_LEFT = {
    -61.95, 0.0, 90.0,           // start: center 8.3" off the left wall (70.25 - 8.3), facing right
    -47.09, -23.55,              // goal
    RL_PRELOAD_EXTRA_DIST,
    -23.55, -23.55, 142.0,       // pin 1, approached from the upper left
    RL_PIN1_GRAB_TURN, RL_PIN1_GRAB_EXTRA, RL_PIN1_SCORE_EXTRA,
    -47.09, -47.09, 223.0,       // pin 2, approached from the upper right, through the gap between the two red goals
    RL_PIN2_GRAB_TURN, RL_PIN2_GRAB_EXTRA, RL_PIN2_SCORE_EXTRA,
    RL_LAST_BACKUP
};

// =====================================================================
//   RED BOTTOM - tuning (RED LEFT flipped across the diagonal: GRAB_TURN signs flipped)
// =====================================================================
constexpr double RB_PRELOAD_EXTRA_DIST = -2.0;
constexpr double RB_PIN1_GRAB_TURN = -RL_PIN1_GRAB_TURN;   // 0
constexpr double RB_PIN1_GRAB_EXTRA = -4.0;
constexpr double RB_PIN1_SCORE_EXTRA = 0.0;
constexpr double RB_PIN2_GRAB_TURN = -RL_PIN2_GRAB_TURN;   // 0
constexpr double RB_PIN2_GRAB_EXTRA = -2.0;
constexpr double RB_PIN2_SCORE_EXTRA = 0.0;
constexpr double RB_LAST_BACKUP = 0.0;

const OpeningRoute QUAL_RED_BOTTOM = {
    0.0, -61.95, 0.0,            // start: center 8.3" off the bottom wall, facing up
    -23.55, -47.09,              // goal
    RB_PRELOAD_EXTRA_DIST,
    -23.55, -23.55, 308.0,       // pin 1, approached from the lower right (90 - 142)
    RB_PIN1_GRAB_TURN, RB_PIN1_GRAB_EXTRA, RB_PIN1_SCORE_EXTRA,
    -47.09, -47.09, 227.0,       // pin 2, approached from the upper right through the gap (90 - 223)
    RB_PIN2_GRAB_TURN, RB_PIN2_GRAB_EXTRA, RB_PIN2_SCORE_EXTRA,
    RB_LAST_BACKUP
};

// =====================================================================
//   BLUE RIGHT - tuning (RED LEFT turned 180 degrees: same values)
// =====================================================================
constexpr double BR_PRELOAD_EXTRA_DIST = -2.0;
constexpr double BR_PIN1_GRAB_TURN = 0.0;
constexpr double BR_PIN1_GRAB_EXTRA = -4.0;
constexpr double BR_PIN1_SCORE_EXTRA = 0.0;
constexpr double BR_PIN2_GRAB_TURN = 0.0;
constexpr double BR_PIN2_GRAB_EXTRA = -2.0;
constexpr double BR_PIN2_SCORE_EXTRA = 0.0;
constexpr double BR_LAST_BACKUP = 0.0;

const OpeningRoute QUAL_BLUE_RIGHT = {
    61.95, 0.0, 270.0,           // start: center 8.3" off the right wall, facing left
    47.09, 23.55,                // goal
    BR_PRELOAD_EXTRA_DIST,
    23.55, 23.55, 322.0,         // pin 1, approached from the lower right (142 + 180)
    BR_PIN1_GRAB_TURN, BR_PIN1_GRAB_EXTRA, BR_PIN1_SCORE_EXTRA,
    47.09, 47.09, 43.0,          // pin 2, approached from the lower left through the gap (223 + 180)
    BR_PIN2_GRAB_TURN, BR_PIN2_GRAB_EXTRA, BR_PIN2_SCORE_EXTRA,
    BR_LAST_BACKUP
};

// =====================================================================
//   BLUE TOP - tuning (RED BOTTOM turned 180 degrees: same values)
// =====================================================================
constexpr double BT_PRELOAD_EXTRA_DIST = -2.0;
constexpr double BT_PIN1_GRAB_TURN = -RL_PIN1_GRAB_TURN;   // 0
constexpr double BT_PIN1_GRAB_EXTRA = -4.0;
constexpr double BT_PIN1_SCORE_EXTRA = 0.0;
constexpr double BT_PIN2_GRAB_TURN = -RL_PIN2_GRAB_TURN;   // 0
constexpr double BT_PIN2_GRAB_EXTRA = -2.0;
constexpr double BT_PIN2_SCORE_EXTRA = 0.0;
constexpr double BT_LAST_BACKUP = 0.0;

const OpeningRoute QUAL_BLUE_TOP = {
    0.0, 61.95, 180.0,           // start: center 8.3" off the top wall, facing down
    23.55, 47.09,                // goal
    BT_PRELOAD_EXTRA_DIST,
    23.55, 23.55, 128.0,         // pin 1, approached from the upper left (308 - 180)
    BT_PIN1_GRAB_TURN, BT_PIN1_GRAB_EXTRA, BT_PIN1_SCORE_EXTRA,
    47.09, 47.09, 47.0,          // pin 2, approached from the lower left through the gap (227 - 180)
    BT_PIN2_GRAB_TURN, BT_PIN2_GRAB_EXTRA, BT_PIN2_SCORE_EXTRA,
    BT_LAST_BACKUP
};

// =====================================================================

void qualRedLeft() {
    runOpeningRoute(QUAL_RED_LEFT);
}

void qualRedBottom() {
    runOpeningRoute(QUAL_RED_BOTTOM);
}

void qualBlueRight() {
    runOpeningRoute(QUAL_BLUE_RIGHT);
}

void qualBlueTop() {
    runOpeningRoute(QUAL_BLUE_TOP);
}

// The old names (in case the selector or main.cpp still uses them): left = red left wall, right = blue right wall
void qualLeftAlliance() {
    qualRedLeft();
}

void qualRightAlliance() {
    qualBlueRight();
}