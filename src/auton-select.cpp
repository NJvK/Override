// =====================================================================
//   AUTON SELECTOR ON THE CONTROLLER (include/auton-select.hpp)
// =====================================================================
#include "auton-select.hpp"

#include "auton-core.hpp"
#include "fourpin.hpp"
#include "pad.hpp"
#include "robot.hpp"
#include "rw/hardware.h"   // hw::sdSave / sdLoad
#include "skills.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>

namespace {

struct Routine {
    const char* name;   // controller line 0 (13 characters fit after the "nn/nn " count)
    const char* what;   // controller line 1 while picking
    void (*run)();
    bool match;         // a match auton: nothing on the controller does anything while it runs
    bool sim;           // offered to the simulator (sh sim/run-sim.sh "<name>"); RW's list holds 12
};

// The order the arrows go through. Index 0 is the default.
const Routine ROUTINES[] = {
    {"Skills", "MATCH 60s L wall", skillsAuton, true, true},
    {"4P Red Bottom", "MATCH 15s bottom", fourPinRedBottom, true, true},
    {"4P Blue Top", "MATCH 15s top", fourPinBlueTop, true, true},
    {"4P Red Left", "MATCH 15s left", fourPinRedLeft, true, true},
    {"4P Blue Right", "MATCH 15s right", fourPinBlueRight, true, true},
    {"Odom Check", "TUNE push by hand", tuneOdomCheck, false, false},
    {"IMU Scale", "TUNE spin 10 turns", tuneImuScale, false, false},
    {"Feedforward", "TUNE 4ft clear F+B", tuneFeedforward, false, false},
    {"Drive Test", "TUNE 4ft straight", tuneDriveTest, false, true},
    {"Spin Test", "TUNE turns in place", tuneSpinTest, false, true},
    {"Curve Test", "TUNE 4ft fwd 3ft rt", tuneCurveTest, false, true},
    {"Lift Test", "TUNE lift timing", tuneLiftTest, false, false},
    {"Sensors + MCL", "TUNE @ skills start", tuneSensors, false, false},
    {"Claw Cam", "TUNE AI cam port 5", tuneClawCam, false, false},
    {"Field Cam", "TUNE AI cam port21", tuneFieldCam, false, false},
    {"Vision Pickup", "TUNE cup <=30in", tuneVisionPickup, false, true},
    {"Sim Calibrate", "TUNE sim drive fit", tuneSimCalibrate, false, false},
};
constexpr int COUNT = static_cast<int>(sizeof(ROUTINES) / sizeof(ROUTINES[0]));
const char* const SAVE_FILE = "override_auton.txt";
std::atomic<int> picked{0}; //do this for default skills
// std::atomic<int> picked{1}; //do this for default quals
std::atomic<bool> driverActive{false};    // driver control has started
std::atomic<bool> resultsUp{false};       // an auton's results are on the controller: the selector waits for an arrow
std::atomic<bool> benchRunning{false};    // bench mode (no field control): a routine started with Y is running
// Field control / a competition switch has been seen since the program started: bench mode is off until a restart,
// so a loose field cable mid-match can never let Y start an auton
std::atomic<bool> everConnected{false};

void benchLine0() { pad::line(0, "%s Y=run", ROUTINES[picked].name); }   // names are 13 characters at most

bool connected() { return pros::competition::is_connected(); }
bool autonNow() { return connected() && pros::competition::is_autonomous() && !pros::competition::is_disabled(); }
// Only with field control / a competition switch, and only while disabled
bool canPick() { return connected() && !autonNow() && pros::competition::is_disabled(); }

void save() {
    const char* name = ROUTINES[picked].name;
    hw::sdSave(SAVE_FILE, name, static_cast<int>(std::strlen(name)));
}
void load() {
    char saved[48] = {0};
    int length = hw::sdLoad(SAVE_FILE, saved, sizeof(saved) - 1);
    if (length <= 0) return;
    saved[length] = '\0';
    for (int i = 0; i < COUNT; i++) {
        if (std::strcmp(saved, ROUTINES[i].name) == 0) picked = i;
    }
}

void draw() {
    const Routine& r = ROUTINES[picked];
    pad::line(0, "%d/%d %s", picked + 1, COUNT, r.name);
    pad::line(1, "%s", r.what);
    pad::line(2, "</> pick %s %d%%", inertial_sensor.is_calibrating() ? "IMU CAL" : "IMU ok", static_cast<int>(pros::battery::get_capacity()));
}

void flush(pros::controller_digital_e_t button) { controller_1.get_digital_new_press(button); }

void selectorLoop() {
    bool wasPicking = false;
    uint32_t lastDraw = 0;
    while (true) {
        if (connected()) everConnected = true;
        const bool picking = canPick();
        if (picking) {
            if (!wasPicking) {
                flush(pros::E_CONTROLLER_DIGITAL_LEFT);   // a press from before doesn't count
                flush(pros::E_CONTROLLER_DIGITAL_RIGHT);
                if (!resultsUp) draw();
                lastDraw = pros::millis();
            }
            const bool left = controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
            const bool right = controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
            if (resultsUp) {
                // An auton's results stay up until an arrow is pressed; that press only brings the selector back
                if (left || right) {
                    resultsUp = false;
                    draw();
                    lastDraw = pros::millis();
                }
            } else if (left != right) {
                picked = (picked + (right ? 1 : COUNT - 1)) % COUNT;
                save();
                pad::rumble(".");
                draw();
                lastDraw = pros::millis();
            } else if (pros::millis() - lastDraw >= 500) {
                draw();   // IMU / battery
                lastDraw = pros::millis();
            }
        }
        wasPicking = picking;
        pros::delay(20);
    }
}
}  // namespace

#ifdef RW_SIM_PROS
extern const char* auton_names[];   // RW's selector list (sim/rw/auton-selector.cpp): the simulator picks by name
#endif

void selectorStart() {
    load();
    for (const Routine& r : ROUTINES) {
        if (r.sim) addAuton(r.name, r.run);
    }
    static pros::Task task(selectorLoop);
}

void runSelectedRoutine() {
    int index = picked;
#ifdef RW_SIM_PROS
    // The simulator chooses with RW's list ("sh sim/run-sim.sh <name>"; no name = the first one, Skills)
    for (int i = 0; i < COUNT; i++) {
        if (std::strcmp(ROUTINES[i].name, auton_names[getSelectedAuton()]) == 0) index = i;
    }
#endif
    resultsUp = true;   // when it's disabled again, what the auton left on the controller stays until an arrow is pressed
    const Routine& r = ROUTINES[index];
    std::printf("Running \"%s\"\n", r.name);
    pad::lines(r.name, r.match ? "starting" : "", "");
    r.run();
}

void driverStarting() {
    resultsUp = false;   // driver readouts take lines 1 and 2; disabled again after the match = the selector
    driverActive = true;
    // Plain program run: the selector never shows, so line 0 says what an auton would run
    if (connected()) everConnected = true;
    if (!everConnected) benchLine0();   // bench mode (benchRunWanted below): LEFT / RIGHT pick, Y runs
}

// =====================================================================
//   BENCH MODE: no field control / competition switch (plain program run)
// =====================================================================
// In driver control, LEFT / RIGHT pick (line 0), Y runs the pick right there. A tuning tool runs until B is held;
// a match auton runs to its end (nothing stops it but stopping the program). Its results stay on the controller
// until an arrow is pressed (that press only brings the driver readouts back).
bool benchRunWanted() {
    if (connected()) everConnected = true;
    if (everConnected) return false;   // ever on field control / a switch since the start: never (restart to use it)
    const bool left = controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    const bool right = controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);
    const bool run = controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);
    if (resultsUp) {
        if (left || right) {
            resultsUp = false;   // back to the driver readouts, pick unchanged
            benchLine0();
        }
    } else if (left != right) {
        picked = (picked + (right ? 1 : COUNT - 1)) % COUNT;
        save();
        pad::rumble(".");
        benchLine0();
    }
    if (run) benchRunning = true;
    return run;
}

void benchFinished() {
    benchRunning = false;
    controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y);   // presses made during the run don't count
    controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);
}

bool driverScreenOn() {
    if (benchRunning || (!connected() && resultsUp)) return false;   // a bench run / its results have the screen
    return driverActive && !autonNow() && !(connected() && pros::competition::is_disabled());
}

// With field control / a switch, tools run until autonomous ends. In bench mode, holding B ends them.
bool tuneExitWanted() { return benchRunning && controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B); }
