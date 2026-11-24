// =====================================================================
//   TUNING TOOLS (picked on the controller like any auton: src/auton-select.cpp)
// =====================================================================
// Every reading is on the CONTROLLER screen (3 lines) and the full numbers are on the PROS terminal too.
// The drive / spin / curve / lift tests use the same code the match routes do (src/auton-core.cpp: the curve
// follower, spinTo, the lift service), so tuning them tunes Skills and the 4-pin.
//
// Running one: on a competition switch / field control, pick it with LEFT / RIGHT while disabled, then enable
// autonomous. It runs until autonomous is switched off. After a test finishes, A runs it again (start it from the
// same spot).
//
// What each one is for (and the setting it tunes):
//   Odom Check     push the robot by hand: X / Y / heading and drive-motor inches. Tracking wheel signs / diameters
//                  (src/rw-config.cpp), wheel_distance_in. A = zero.
//   IMU Scale      spin it by hand IMU_TURNS turns clockwise: imu_scale (src/rw-config.cpp). A = start over.
//   Feedforward    6 short full-power-ish runs, forward and back (4 ft clear both ways): drive_kS / kV / kA
//                  (src/rw-config.cpp). The curve follower drives with these.
//   Drive Test     DRIVE_TEST_IN forward and back with the curve follower: time, how far past the end it stopped,
//                  sideways and heading error. FP_MAX_SPEED / FP_ACCEL / FP_DECEL / FP_STOP_GAIN / FP_SPEED_FEEDBACK.
//   Spin Test      spinTo 90 / 180 / 45 each way: time per size, worst error, drift after it stops.
//                  FP_SPIN_* (include/auton-core.hpp).
//   Curve Test     a forward curve 3 ft right / 3 ft up and backwards home: time, end error, how far it wandered
//                  off the curve. FP_LAT_ACCEL, FP_STEER_*, FP_HEADING_GAIN.
//   Lift Test      robot still, lift to each alliance height from the bottom: time until the auton calls it
//                  ready, against what it predicts. FP_LIFT_UP_DPS / FP_LIFT_START_S (lift-late timing).
//   Sensors + MCL  at the skills start: the 4 distance sensors, X / Y, MCL spread. A = one distance reset now,
//                  Y = back to the start pose. distance_sensor_scale, sensor offsets, use_mcl / resets.
//   Claw Cam       the AI Vision sensor in the claw (port 5): src/ai-vision-tuning.cpp
//   Field Cam      the AI Vision sensor on the chassis (port 21): tags (FP_GOAL_CAM_YAW) and pins
//   Sim Calibrate  records the drive for the simulator's fitter (sim/fit-profile.sh), ~15 s, 4 ft clear all round
//   Vision Pickup  a cup / stack up to 30" ahead: the cameras find it, the auton pickup code drives to it and grabs it
// =====================================================================
#include "auton-core.hpp"
#include "auton-select.hpp"
#include "pad.hpp"
#include "rw/hardware.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ac;

namespace {

constexpr int IMU_TURNS = 10;
constexpr double DRIVE_TEST_IN = 48.0;
constexpr double CURVE_TEST_X = 24.0, CURVE_TEST_Y = 36.0;   // where the curve test's forward curve ends (nose right)
constexpr double SKILLS_START_X = -61.85, SKILLS_START_Y = 0.0, SKILLS_START_HEADING = 90.0;   // = src/skills.cpp

// Start of every tool: auton-core's clock with a long limit, and the controller lines to itself
void begin(const char* name) {
    setRouteFrame(false, false);
    coreStart(name, 3600000);
    showStatusOnController(false);
}
bool pressed(pros::controller_digital_e_t button) { return controller_1.get_digital_new_press(button); }
// Waits with the lift / claw service running. false = leave the tool (tuneExitWanted)
bool idle(int ms) {
    const uint32_t start = pros::millis();
    while (pros::millis() - start < static_cast<uint32_t>(ms)) {
        if (tuneExitWanted()) return false;
        service();
        pros::delay(10);
    }
    return !tuneExitWanted();
}
// After a test: A = again. true = again.
bool again() {
    pressed(pros::E_CONTROLLER_DIGITAL_A);
    while (true) {
        if (tuneExitWanted()) return false;
        if (pressed(pros::E_CONTROLLER_DIGITAL_A)) return true;
        service();
        pros::delay(10);
    }
}
// The drive tests run anywhere on the field: MCL would "correct" them toward walls that aren't where it thinks
struct McPause {
    McPause() { pauseFieldMcl(true); }
    ~McPause() { pauseFieldMcl(false); }
};
double heading360() { return std::fmod(std::fmod(getInertialHeading(), 360.0) + 360.0, 360.0); }
double driveInches(double degrees) { return degrees / 360.0 * wheel_distance_in; }

// One curve-follower move from where the robot is to `to`, then 0.4 s to see where it really stops
struct MoveResult {
    int ms;
    double past;      // inches past the end, along the way it was going (- = short)
    double side;      // inches right of the end (looking along the end heading)
    double heading;   // degrees off the end heading (+ = clockwise of it)
    double peak;      // fastest it went (in/s)
    double wander;    // furthest it got from the curve (inches)
    double miss;      // straight-line distance from the end spot
};
MoveResult testMove(const Node& to, bool reverse) {
    Curve curve = makeCurve({hereNode(), to}, reverse, FP_MAX_SPEED, 0.0);
    MoveResult result{};
    MoveOptions options;
    options.tick = [&](double done, double left) {
        (void)left;
        result.peak = std::max(result.peak, std::fabs(odom_speed_ips));
        const int index = std::min(static_cast<int>(std::lround(done / FP_SAMPLE_STEP)), static_cast<int>(curve.points.size()) - 1);
        const Sample& on = curve.points[std::max(index, 0)];
        result.wander = std::max(result.wander, std::hypot(x_pos - on.x, y_pos - on.y));
        return !tuneExitWanted();
    };
    const uint32_t start = pros::millis();
    follow(curve, options);
    result.ms = static_cast<int>(pros::millis() - start);
    idle(400);
    const Vec f = forwardOf(to.heading), r = rightOf(to.heading);
    const double dx = x_pos - to.x, dy = y_pos - to.y;
    result.past = (dx * f.x + dy * f.y) * (reverse ? -1.0 : 1.0);
    result.side = dx * r.x + dy * r.y;
    result.heading = wrap180(getInertialHeading() - to.heading);
    result.miss = std::hypot(dx, dy);
    std::printf("  %s to (%.1f, %.1f, %.0f): %d ms, past end %+.2f in, sideways %+.2f in, heading %+.2f deg, peak %.1f in/s, "
                "furthest off the curve %.2f in\n",
                reverse ? "back" : "forward", to.x, to.y, to.heading, result.ms, result.past, result.side, result.heading, result.peak,
                result.wander);
    return result;
}

// Distance sensor reading in inches (x distance_sensor_scale), -1 = sees nothing
double sensorInches(pros::Distance& sensor) {
    const int32_t mm = sensor.get();
    return (mm > 0 && mm < 9999) ? mm / 25.4 * distance_sensor_scale : -1.0;
}
void sensorText(char* out, int size, char label, double inches) {
    if (inches < 0) std::snprintf(out, size, "%c --", label);
    else std::snprintf(out, size, "%c%.1f", label, inches);
}

}  // namespace

// =====================================================================
//   ODOM CHECK
// =====================================================================
// Nothing moves. Push the robot: forward -> Y up, to its right -> X up, clockwise -> heading up.
//   line 1  X / Y (tracking wheels + IMU)
//   line 2  heading, and the inches the LEFT / RIGHT drive motors counted (push 24": both about 24)
//   line 3  each left motor's inches in left_chassis order (one stuck at 0 = that motor isn't reporting)
// Push it 96" along a wall: Y should read 96.0 (tracking wheel diameters). A = zero everything.
void tuneOdomCheck() {
    begin("odom");
    McPause pause;
    stopChassis(pros::MotorBrake::coast);   // easy to push
    double startLeft = 0, startRight = 0, startEach[3] = {0, 0, 0};
    auto zero = [&]() {
        setPose(0, 0, 0);
        startLeft = getLeftRotationDegree();
        startRight = getRightRotationDegree();
        for (int i = 0; i < 3; i++) startEach[i] = left_chassis.get_position(i);
    };
    zero();
    while (!tuneExitWanted()) {
        if (pressed(pros::E_CONTROLLER_DIGITAL_A)) zero();
        pad::line(0, "X%.2f Y%.2f", x_pos, y_pos);
        pad::line(1, "H%.1f L%.1f R%.1f", heading360(), driveInches(getLeftRotationDegree() - startLeft),
                  driveInches(getRightRotationDegree() - startRight));
        double each[3];
        for (int i = 0; i < 3; i++) each[i] = driveInches(left_chassis.get_position(i) - startEach[i]);
        pad::line(2, "%.1f %.1f %.1f A=0", each[0], each[1], each[2]);
        pros::delay(50);
    }
}

// =====================================================================
//   IMU SCALE
// =====================================================================
// Line the robot up with a tile edge, spin it by hand exactly IMU_TURNS turns clockwise, line it up again.
// Line 3 is the imu_scale to put in src/rw-config.cpp ("n" = the one in there now). A = start over.
void tuneImuScale() {
    begin("imu");
    McPause pause;
    stopChassis(pros::MotorBrake::coast);
    hw::setImuDegrees(0);
    double lastPrint = 0;
    while (!tuneExitWanted()) {
        if (pressed(pros::E_CONTROLLER_DIGITAL_A)) hw::setImuDegrees(0);
        const double raw = hw::imuDegrees();
        pad::line(0, "IMU: spin %d CW", IMU_TURNS);
        pad::line(1, "raw %.1f / %d", raw, IMU_TURNS * 360);
        if (std::fabs(raw) > 90.0) {
            const double scale = IMU_TURNS * 360.0 / std::fabs(raw);
            pad::line(2, "use %.5f n%.4f", scale, imu_scale);
            if (std::fabs(raw - lastPrint) > 30.0) {
                lastPrint = raw;
                std::printf("IMU Scale: raw %.2f deg, imu_scale = %.5f (if that was %d turns)\n", raw, scale, IMU_TURNS);
            }
        } else {
            pad::line(2, "by hand. A=start");
        }
        pros::delay(50);
    }
}

// =====================================================================
//   FEEDFORWARD
// =====================================================================
// RW's measureDriveFeedforward() (sim/rw/path-following.cpp), with the answer on the controller: 3, 6 and 9 V
// forward and back, 0.8 s each. Needs about 4 ft clear in front and behind. Put the three numbers in
// src/rw-config.cpp (drive_kS / drive_kV / drive_kA): the curve follower's sideVolts() drives with them.
void tuneFeedforward() {
    begin("feedforward");
    McPause pause;
    const double inchesPerSecPerRpm = wheel_distance_in / 60.0;
    auto speedNow = [&]() {
        return std::fabs(left_chassis.get_actual_velocity() + right_chassis.get_actual_velocity()) / 2.0 * inchesPerSecPerRpm;
    };
    do {
        pad::lines("FEEDFORWARD", "4 ft clear F + B", "starting...");
        if (!idle(800)) return;
        const double volts[6] = {3, -3, 6, -6, 9, -9};
        double steady[6];
        double accelSum = 0, accelSpeedSum = 0;
        int accelCount = 0;
        for (int i = 0; i < 6; i++) {
            pad::line(2, "step %d/6  %+.0f V", i + 1, volts[i]);
            const uint32_t start = pros::millis();
            double speedSum = 0, earlySpeed = 0;
            int speedCount = 0, earlyMs = -1;
            while (pros::millis() - start < 800) {
                driveChassis(volts[i], volts[i]);
                const int t = static_cast<int>(pros::millis() - start);
                const double speed = speedNow();
                if (t > 550) {   // steady speed near the end of each step: kS and kV
                    speedSum += speed;
                    speedCount++;
                }
                if (std::fabs(volts[i]) == 9.0) {   // how fast the 9 V steps speed up: kA
                    if (t >= 30 && earlyMs < 0) {
                        earlyMs = t;
                        earlySpeed = speed;
                    } else if (earlyMs >= 0 && t >= earlyMs + 100 && t < earlyMs + 110) {
                        accelSum += (speed - earlySpeed) / ((t - earlyMs) / 1000.0);
                        accelSpeedSum += (speed + earlySpeed) / 2.0;
                        accelCount++;
                    }
                }
                pros::delay(10);
            }
            stopChassis(pros::MotorBrake::brake);
            steady[i] = speedCount > 0 ? speedSum / speedCount : 0.0;
            std::printf("Feedforward: %+.0f V -> %.1f in/s\n", volts[i], steady[i]);
            if (!idle(500)) {
                stopChassis(pros::MotorBrake::coast);
                return;
            }
        }
        // Straight line through (speed, volts): volts = kS + kV * speed
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < 6; i++) {
            const double x = steady[i], y = std::fabs(volts[i]);
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }
        const double kV = (6 * sxy - sx * sy) / std::max(6 * sxx - sx * sx, 1e-9);
        const double kS = (sy - kV * sx) / 6;
        double kA = 0;
        if (accelCount > 0 && accelSum > 0) {
            const double accel = accelSum / accelCount, midSpeed = accelSpeedSum / accelCount;
            kA = (9 - kS - kV * midSpeed) / accel;
        }
        std::printf("Feedforward: drive_kS = %.3f; drive_kV = %.4f; drive_kA = %.4f;  (src/rw-config.cpp)\n", kS, kV, kA);
        pad::line(0, "kS %.3f  kV %.4f", kS, kV);
        pad::line(1, "kA %.4f", kA);
        pad::line(2, "rw-config  A=again");
    } while (again());
}

// =====================================================================
//   DRIVE TEST
// =====================================================================
// DRIVE_TEST_IN forward with the curve follower, stop, the same back. Clear floor in front.
//   line 1  F <time> <inches past the end (+ = overshot)> s<inches sideways>
//   line 2  B ...the same for the drive back
//   line 3  pk<top speed> H<heading error forward>/<back>
// Overshoots (+) = brakes later than planned: lower FP_DECEL or FP_STOP_GAIN, raise FP_SPEED_FEEDBACK.
// Stops short or creeps the last inches = raise FP_STOP_GAIN / drive_kS. Top speed under FP_MAX_SPEED = it can't
// reach it (lower FP_MAX_SPEED to what it does reach, the speed plan is then honest).
void tuneDriveTest() {
    begin("drive test");
    McPause pause;
    do {
        pad::lines("DRIVE TEST", "forward + back", "go in 0.5 s");
        setPose(0, 0, 0);
        if (!idle(500)) return;
        std::printf("Drive Test (%.0f in):\n", DRIVE_TEST_IN);
        const MoveResult out = testMove(node(0, DRIVE_TEST_IN, 0), false);
        if (tuneExitWanted()) return;
        pad::line(0, "F %.2fs %+.1f s%+.1f", out.ms / 1000.0, out.past, out.side);
        const MoveResult back = testMove(node(0, 0, 0), true);
        if (tuneExitWanted()) return;
        pad::line(1, "B %.2fs %+.1f s%+.1f", back.ms / 1000.0, back.past, back.side);
        pad::line(2, "pk%.0f H%+.1f/%+.1f A", std::max(out.peak, back.peak), out.heading, back.heading);
    } while (again());
}

// =====================================================================
//   SPIN TEST
// =====================================================================
// spinTo (the turn in place the skills run uses at the loader) through 90, 180 and 45 degree turns, each way.
//   line 1  average time for the 90s and the 180s
//   line 2  average time for the 45s, and the worst error once it had stopped
//   line 3  drift: how much it kept turning after spinTo said it was there (overshoot)
// Slow but exact = raise FP_SPIN_DECEL_DPS2 / FP_SPIN_MAX_DPS. Overshoots / drifts = lower FP_SPIN_DECEL_DPS2.
void tuneSpinTest() {
    begin("spin test");
    McPause pause;
    const double targets[6] = {90, 0, 180, 0, 45, 0};
    do {
        pad::lines("SPIN TEST", "turns in place", "go in 0.5 s");
        setPose(0, 0, 0);
        if (!idle(500)) return;
        double time90 = 0, time180 = 0, time45 = 0, worst = 0, drift = 0;
        double from = 0;
        std::printf("Spin Test:\n");
        for (int i = 0; i < 6; i++) {
            const double size = std::fabs(wrap180(targets[i] - from));
            const uint32_t start = pros::millis();
            spinTo(targets[i]);
            const int ms = static_cast<int>(pros::millis() - start);
            const double atStop = wrap180(getInertialHeading() - targets[i]);
            if (!idle(300)) return;
            const double settled = wrap180(getInertialHeading() - targets[i]);
            worst = std::max(worst, std::fabs(settled));
            drift = std::max(drift, std::fabs(settled - atStop));
            if (size > 135) time180 += ms / 2000.0;
            else if (size > 67) time90 += ms / 2000.0;
            else time45 += ms / 2000.0;
            std::printf("  %3.0f deg turn to %3.0f: %d ms, %+.2f deg when it stopped, %+.2f deg 0.3 s later\n", size, targets[i], ms,
                        atStop, settled);
            pad::line(2, "%d/6 %.0fd %.2fs %+.1f", i + 1, size, ms / 1000.0, settled);
            from = targets[i];
        }
        pad::line(0, "90:%.2fs 180:%.2fs", time90, time180);
        pad::line(1, "45:%.2fs worst%+.1f", time45, worst);
        pad::line(2, "drift %.1f  A=again", drift);
    } while (again());
}

// =====================================================================
//   CURVE TEST
// =====================================================================
// A forward curve from (0, 0) facing up to (CURVE_TEST_X right, CURVE_TEST_Y up) facing right, then a backwards
// curve home. Needs about 4 ft forward and 3 ft to the right.
//   line 1  O <time> <inches from the end spot> <heading error>
//   line 2  B ...the same for the curve back
//   line 3  off: the furthest it got from the curve on the way
// Wanders off the curve = it's cornering faster than the tires hold (lower FP_LAT_ACCEL) or steering too softly
// (lower FP_STEER_DIST / FP_STEER_TIME_S). Wobbles side to side = steering too hard (raise them).
void tuneCurveTest() {
    begin("curve test");
    McPause pause;
    do {
        pad::lines("CURVE TEST", "4ft up, 3ft right", "go in 0.5 s");
        setPose(0, 0, 0);
        if (!idle(500)) return;
        std::printf("Curve Test:\n");
        const MoveResult out = testMove(node(CURVE_TEST_X, CURVE_TEST_Y, 90), false);
        if (tuneExitWanted()) return;
        pad::line(0, "O %.2fs %.1fin %+.0fd", out.ms / 1000.0, out.miss, out.heading);
        const MoveResult back = testMove(node(0, 0, 0), true);
        if (tuneExitWanted()) return;
        pad::line(1, "B %.2fs %.1fin %+.0fd", back.ms / 1000.0, back.miss, back.heading);
        pad::line(2, "off %.1f/%.1f A", out.wander, back.wander);
    } while (again());
}

// =====================================================================
//   LIFT TEST
// =====================================================================
// The robot stays still. The lift goes from the bottom to each alliance height (main.cpp's ALLIANCE_HEIGHTS) with
// the auton lift code, and the time until liftReady() (within FP_LIFT_READY_TOLERANCE) is compared with
// liftSeconds(), which the routes use to start the lift "as late as it can" (ScorePlan::liftLate).
//   line 1  UP_DPS from the measured times (now)       -> FP_LIFT_UP_DPS
//   line 2  START from the measured times (now)        -> FP_LIFT_START_S
//   line 3  each height in turn: r<ready s> p<predicted s> s<settled within 3 deg s>
// Ready later than predicted = the robot creeps into goals waiting for the lift: use the measured numbers.
void tuneLiftTest() {
    begin("lift test");
    struct Row { double height, ready, predicted, settled; };
    do {
        std::vector<Row> rows;
        for (int spot = 1; spot <= 5; spot++) {
            const double height = getStackHeight(GOAL_ALLIANCE, spot);
            pad::line(0, "LIFT TEST %d/5", spot);
            pad::line(1, "down...");
            liftDown();
            const uint32_t downStart = pros::millis();
            while (!liftIsDown() && pros::millis() - downStart < 2500) {
                if (!idle(10)) return;
            }
            if (!idle(400)) return;
            pad::line(1, "up to %.0f", height);
            liftTo(height);
            const uint32_t start = pros::millis();
            double ready = -1, settled = -1;
            uint32_t closeSince = 0;
            while (pros::millis() - start < 3000) {
                if (!idle(10)) return;
                const double t = (pros::millis() - start) / 1000.0;
                if (ready < 0 && liftReady()) ready = t;
                if (std::fabs(getDR4BPosition() - height) <= 3.0) {
                    if (closeSince == 0) closeSince = pros::millis();
                    if (settled < 0 && pros::millis() - closeSince >= 150) settled = t - 0.15;
                } else {
                    closeSince = 0;
                }
                if (ready >= 0 && settled >= 0) break;
            }
            const Row row = {height, ready, liftSeconds(height), settled};
            rows.push_back(row);
            std::printf("Lift Test: %.0f deg  ready %.2f s (predicted %.2f s)  settled %.2f s\n", row.height, row.ready, row.predicted,
                        row.settled);
            pad::line(2, "%.0f r%.2f p%.2f", row.height, row.ready, row.predicted);
        }
        liftDown();
        // Least squares: ready = START + height / UP_DPS
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (const Row& row : rows) {
            if (row.ready < 0) continue;
            n++;
            sx += row.height;
            sy += row.ready;
            sxx += row.height * row.height;
            sxy += row.height * row.ready;
        }
        const double slope = n >= 2 ? (n * sxy - sx * sy) / std::max(n * sxx - sx * sx, 1e-9) : 0.0;
        const double fitDps = slope > 1e-6 ? 1.0 / slope : 0.0;
        const double fitStart = n >= 2 ? (sy - slope * sx) / n : 0.0;
        std::printf("Lift Test: fit FP_LIFT_UP_DPS = %.0f (now %.0f), FP_LIFT_START_S = %.2f (now %.2f)\n", fitDps, FP_LIFT_UP_DPS, fitStart,
                    FP_LIFT_START_S);
        pad::line(0, "UP_DPS %.0f (%.0f)", fitDps, FP_LIFT_UP_DPS);
        pad::line(1, "START %.2f (%.2f)", fitStart, FP_LIFT_START_S);
        // Line 3 goes through the heights one at a time until A (again)
        pressed(pros::E_CONTROLLER_DIGITAL_A);
        size_t showing = 0;
        bool runAgain = false;
        while (!runAgain) {
            const Row& row = rows[showing % rows.size()];
            pad::line(2, "%.0f r%.2f p%.2f s%.2f", row.height, row.ready, row.predicted, row.settled);
            showing++;
            for (int k = 0; k < 150 && !runAgain; k++) {   // 1.5 s per height
                if (tuneExitWanted()) return;
                if (pressed(pros::E_CONTROLLER_DIGITAL_A)) runAgain = true;
                service();
                pros::delay(10);
            }
        }
    } while (true);
}

// =====================================================================
//   SENSORS + MCL
// =====================================================================
// Put the robot at the skills start (back on the left wall, centered, facing right). Nothing moves; push it around
// and watch X / Y follow.
//   lines 1-2  each distance sensor in inches (x distance_sensor_scale; "--" = sees nothing)
//   line 3     X, Y, MCL spread (small = sure; "s-" = MCL off), sensors the last A reset used
// A = one distance reset now (what the autons do at goals / the loader), Y = back to the start pose.
// Sensor scale: a tape-measured 2-4 ft from a wall, distance_sensor_scale = tape / reading.
void tuneSensors() {
    begin("sensors");
    auto toStart = []() { setPose(SKILLS_START_X, SKILLS_START_Y, SKILLS_START_HEADING); };
    toStart();
    stopChassis(pros::MotorBrake::coast);
    int lastResets = -1;
    while (!tuneExitWanted()) {
        if (pressed(pros::E_CONTROLLER_DIGITAL_A)) {
            lastResets = fieldAwareReset(FP_RESET_MAX_JUMP);
            std::printf("Sensors: distance reset used %d sensors -> x %.2f y %.2f\n", lastResets, x_pos, y_pos);
        }
        if (pressed(pros::E_CONTROLLER_DIGITAL_Y)) toStart();
        char f[12], r[12], b[12], l[12];
        sensorText(f, sizeof(f), 'F', sensorInches(front_sensor));
        sensorText(r, sizeof(r), 'R', sensorInches(right_sensor));
        sensorText(b, sizeof(b), 'B', sensorInches(back_sensor));
        sensorText(l, sizeof(l), 'L', sensorInches(left_sensor));
        pad::line(0, "%-8s %s", f, r);
        pad::line(1, "%-8s %s", b, l);
        char spread[12], resets[8];
        if (mcl_running) std::snprintf(spread, sizeof(spread), "s%.1f", mcl_spread);
        else std::snprintf(spread, sizeof(spread), "s-");
        if (lastResets >= 0) std::snprintf(resets, sizeof(resets), " r%d", lastResets);
        else resets[0] = '\0';
        pad::line(2, "%.1f,%.1f %s%s", x_pos, y_pos, spread, resets);
        pros::delay(100);
    }
}

// =====================================================================
//   SIM CALIBRATE
// =====================================================================
// RW's calibrateDrive(): ~15 s of driving and turning, logged for the simulator's fitter (sim/fit-profile.sh), so
// the simulator drives like this robot. Saves rw_drive_log.csv on the SD card, or prints it on the terminal.
void tuneSimCalibrate() {
    begin("sim calibrate");
    McPause pause;
    pad::lines("SIM CALIBRATE", "4 ft clear all round", "about 15 s...");
    if (!idle(800)) return;
    calibrateDrive();
    pad::line(1, "%s", pros::usd::is_installed() ? "saved rw_drive_log" : "log on the terminal");
    pad::line(2, "done");
    while (!tuneExitWanted()) {
        service();
        pros::delay(20);
    }
}

// =====================================================================
//   VISION PICKUP
// =====================================================================
// Put ONE cup / stack (or a matchload) up to 30" in front of the claw, roughly ahead (a few inches either side is
// fine). Nothing tells the robot where it is: standing still, both cameras look at it (locateCup: where their lines
// of sight cross), then the SAME pickup the autos use (pickUp: claw open, lift pressed down, slow through the grab,
// the close-range camera corrections, claw closes, lift hops) drives to that spot.
//   Before:  "A = go". The claw lets go, then its motor is off while it waits (no overheating); set the cup, then A.
//            The claw opens when A is pressed (200 ms + a little before it looks), and stays open until the grab.
//   After:   line 1  where the cameras put it: x (+ = right) and y (ahead) from the tracking center, and how many fixes
//            line 2  t = time from go to the claw closing, cam = camera frames that corrected it on the way, sp = how far
//                    apart the fixes were (big = the cameras disagree)
//            line 3  "A=drop+again": the claw lets go, put the cup back, A again
//   No fix: "NO FIX c# f#" = frames only the claw camera / only the field camera saw one cup (both must, together).
// Look at where the cup ends up in the claw: centered = it works.
void tuneVisionPickup() {
    begin("vision pickup");
    McPause pause;
    bool prompt = true;   // false = A was just pressed on a NO FIX / OUT OF RANGE screen: look again straight away
    while (true) {
        // The claw lets go of the last cup, then the motor goes off while it waits (holding it open would overheat it)
        clawOpen();
        liftDown();
        liftPress(true);
        if (!idle(400)) break;
        clawOff();
        if (prompt) {
            pad::lines("VISION PICKUP", "cup <=30in ahead", "A = go  B = stop");
#ifndef RW_SIM_PROS
            if (!again()) break;   // waits for A (service running: the claw opens, the lift goes down)
#endif
        }
        prompt = true;
        setPose(0, 0, 0);
        pad::lines("VISION PICKUP", "looking...", "");
        clawOpen();   // open only from here to the grab, like the autos
        if (!idle(FP_CLAW_CAM_OPEN_MS + 200)) break;   // the claw well open before the claw camera counts
        const CupFix fix = locateCup(1500);
        std::printf("Vision Pickup: %d frames, %d fixes (claw only %d, field only %d), cup at x %+.2f y %.2f, spread %.2f\n",
                    fix.frames, fix.fixes, fix.clawOnly, fix.fieldOnly, fix.x, fix.y, fix.spread);
        if (fix.fixes < 5) {
            pad::lines("NO FIX", "", "A = again");
            pad::line(1, "c%d f%d both%d", fix.clawOnly, fix.fieldOnly, fix.fixes);
#ifdef RW_SIM_PROS
            break;
#else
            if (!again()) break;
            prompt = false;
            continue;
#endif
        }
        if (fix.y < FP_CLAW_REACH + 3.0 || fix.y > 45.0 || std::fabs(fix.x) > 12.0) {
            pad::lines("OUT OF RANGE", "", "A = again");
            pad::line(1, "x%+.1f y%.1f", fix.x, fix.y);
#ifdef RW_SIM_PROS
            break;
#else
            if (!again()) break;
            prompt = false;
            continue;
#endif
        }
        // Straight at it: the robot ends with the middle of the claw on the cup, pointing the way it drove
        const double heading = std::atan2(fix.x, fix.y) * 180.0 / FP_PI;
        const Node grab = clawOn(fix.x, fix.y, heading, FP_CLAW_REACH);
        const std::vector<Node> nodes = {hereNode(), grab, along(grab, 4.0)};
        const uint32_t start = pros::millis();
        pickUp(nodes, 1, fix.x, fix.y, fix.x, fix.y, -1.0, 300);
        const double seconds = (pros::millis() - start) / 1000.0;
        stopChassis(pros::MotorBrake::brake);
        if (!idle(300)) break;
        // (how far the camera moved the curve is on the "pin camera: seen ..." line just above, from aimStop)
        std::printf("Vision Pickup: picked up in %.2f s, %d camera corrections on the way\n", seconds, aimSightings());
        pad::line(0, "x%+.1f y%.1f n%d", fix.x, fix.y, fix.fixes);
        pad::line(1, "t%.1f cam%d sp%.1f", seconds, aimSightings(), fix.spread);
        pad::line(2, "A=drop+again");
#ifdef RW_SIM_PROS
        break;
#else
        if (!again()) break;
#endif
    }
    finishSafe();
}
