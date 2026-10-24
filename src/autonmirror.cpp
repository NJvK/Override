// Qual autons "mirror" (mirrorAlliance / mirrorAllianceRight / mirrorAllianceTop): now the skills opening, like the
// qual routes in src/quals.cpp: motor toggle -> preload -> both field pins on the blue goal, then stops.
//   right wall start -> "Blue Right" in src/quals.cpp (blue goal (47.09, 23.55))
//   top wall start   -> "Blue Top"   in src/quals.cpp (blue goal (23.55, 47.09))
// The old LemLib-era routes that lived here are gone. Every spot and tuning value is in src/quals.cpp
// (QUAL_BLUE_RIGHT / QUAL_BLUE_TOP, the BR_ / BT_ values).
#include "robot.hpp"

void qualBlueRight();   // src/quals.cpp
void qualBlueTop();

// false = RIGHT wall start, true = TOP wall start (for mirrorAlliance only; the other two always run their own side)
constexpr bool AUTON_START_ON_TOP = false;

void mirrorAlliance() {
    if (AUTON_START_ON_TOP) {
        qualBlueTop();
    } else {
        qualBlueRight();
    }
}

void mirrorAllianceRight() {
    qualBlueRight();
}

void mirrorAllianceTop() {
    qualBlueTop();
}