// QUAL autons (selector: "Qual Left" / "Qual Right"). Both are the opening of the skills run (src/skillsauto.cpp):
//   motor toggle -> preload on the goal -> field pin 1 on the goal -> field pin 2 on the goal, then stops
// The driving itself is shared with skills (runOpeningRoute), but every spot and tuning value is here, separately
// for each side, so tuning one doesn't change the other (or skills).
#include "robot.hpp"

// =====================================================================
//   QUAL LEFT - tuning
// =====================================================================
// Start: back of the robot against the LEFT wall, in the middle of the wall, claw facing right (heading 90).
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25. Headings: 0 = up, 90 = right, clockwise.
// Goal (-47.09, -23.55), field pins at (-24, -24) and (-48, -48).
constexpr double QL_PRELOAD_EXTRA_DIST = -2.0;  // + = preload goes further onto the goal, - = shorter
constexpr double QL_PIN1_GRAB_TURN = 3.0;       // + = approach line turned more clockwise (comes in more to the RIGHT), - = left
constexpr double QL_PIN1_GRAB_EXTRA = 0.0;      // + = drives further into pin 1, - = shorter
constexpr double QL_PIN1_SCORE_EXTRA = 0.0;     // + = pin 1 goes further onto the goal, - = shorter
constexpr double QL_PIN2_GRAB_TURN = 3.0;       // same for pin 2
constexpr double QL_PIN2_GRAB_EXTRA = 0.0;
constexpr double QL_PIN2_SCORE_EXTRA = 0.0;
constexpr double QL_LAST_BACKUP = 0.0;          // backs off the goal this far after pin 2 (0: auton ends there, claw open)

const OpeningRoute QUAL_LEFT = {
    -61.95, 0.0, 90.0,           // start: center 8.3" off the left wall (70.25 - 8.3), facing right
    -47.09, -23.55,              // goal
    QL_PRELOAD_EXTRA_DIST,
    -24.0, -24.0, 142.0,         // pin 1, approached from the upper left
    QL_PIN1_GRAB_TURN, QL_PIN1_GRAB_EXTRA, QL_PIN1_SCORE_EXTRA,
    -48.0, -48.0, 223.0,         // pin 2, approached from the upper right through the gap between the goals
    QL_PIN2_GRAB_TURN, QL_PIN2_GRAB_EXTRA, QL_PIN2_SCORE_EXTRA,
    QL_LAST_BACKUP
};

// =====================================================================
//   QUAL RIGHT - tuning
// =====================================================================
// The left route mirrored across the field's center line: every X flips sign, every heading becomes 360 - heading
// (so left and right turns swap). Start: back of the robot against the RIGHT wall, centered, claw facing left (270).
// Goal (47.09, -23.55), field pins at (24, -24) and (48, -48).
// Left and right are swapped here, so the mirror of the left side's +3 GRAB_TURN is -3.
constexpr double QR_PRELOAD_EXTRA_DIST = -2.0;  // + = preload goes further onto the goal, - = shorter
constexpr double QR_PIN1_GRAB_TURN = -3.0;      // + = approach line turned more clockwise (comes in more to the RIGHT), - = left
constexpr double QR_PIN1_GRAB_EXTRA = 0.0;      // + = drives further into pin 1, - = shorter
constexpr double QR_PIN1_SCORE_EXTRA = 0.0;     // + = pin 1 goes further onto the goal, - = shorter
constexpr double QR_PIN2_GRAB_TURN = -3.0;      // same for pin 2
constexpr double QR_PIN2_GRAB_EXTRA = 0.0;
constexpr double QR_PIN2_SCORE_EXTRA = 0.0;
constexpr double QR_LAST_BACKUP = 0.0;          // see QL_LAST_BACKUP

const OpeningRoute QUAL_RIGHT = {
    61.95, 0.0, 270.0,           // start: center 8.3" off the right wall, facing left
    47.09, -23.55,               // goal
    QR_PRELOAD_EXTRA_DIST,
    24.0, -24.0, 218.0,          // pin 1, approached from the upper right (360 - 142)
    QR_PIN1_GRAB_TURN, QR_PIN1_GRAB_EXTRA, QR_PIN1_SCORE_EXTRA,
    48.0, -48.0, 137.0,          // pin 2, approached from the upper left (360 - 223)
    QR_PIN2_GRAB_TURN, QR_PIN2_GRAB_EXTRA, QR_PIN2_SCORE_EXTRA,
    QR_LAST_BACKUP
};

// =====================================================================

void qualLeftAlliance() {
    runOpeningRoute(QUAL_LEFT);
}

void qualRightAlliance() {
    runOpeningRoute(QUAL_RIGHT);
}
