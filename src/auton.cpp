// Qual auton "2-pin" (twoPinAlliance): now the skills opening from the RIGHT wall (blue), like the qual routes in
// src/quals.cpp: motor toggle -> preload on the blue goal (47.09, 23.55) -> field pin (23.55, 23.55) on the goal ->
// field pin (47.09, 47.09) on the goal, then stops.
// The old LemLib-era route that lived here (its own pickup, scoring and 18" robot numbers) is gone: it's the same as
// "Blue Right" in src/quals.cpp, and every spot and tuning value for it is there (QUAL_BLUE_RIGHT / the BR_ values).
#include "robot.hpp"

void qualBlueRight();   // src/quals.cpp

// Kept so anything that still calls twoPinAlliance (the selector, main.cpp) runs the new route
void twoPinAlliance() {
    qualBlueRight();
}