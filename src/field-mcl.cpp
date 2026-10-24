#include "rw-chassis.hpp"   // rw-config.h (RW + devices + settings)
#include "rw/hardware.h"    // hw::distance*, hw::timeMicros
#include "rw/mcl.h"         // RW's particle filter: mclInit / mclPredict / mclCorrect / mclEstimate / mclNudge
#include "rw/utils.h"       // distanceResetPosition, beamToWall, localToGlobal, degToRad
#include <algorithm>   // std::swap
#include <cmath>

// =====================================================================
//   FIELD-AWARE MCL
// =====================================================================
// RW's own MCL thread (startMcl) uses every distance reading that points at a wall, even when a goal or a field pin
// is in the way. In the simulator that pulled Skills 2-3" off in 5 of 10 runs (touching a goal / the loader, missing a
// field pin). This runs the same particle filter (RW's mcl* functions, same settings) but only trusts a reading when
// nothing on the field sits in the beam before the wall:
//   1. RW's wall checks (beam angle, not into a corner), with a smaller corner margin, because...
//   2. ...the beam (it widens about 12 degrees each side) is checked against every field element below
//   3. a reading that would move the position more than distance_reset_max_jump is skipped (like the resets)
// Turn it on with use_mcl in src/rw-config.cpp (src/rw-user.cpp starts this instead of RW's startMcl).


// RW's MCL state (rw/motor-control.cpp; not in RW's headers)
extern double odom_motion_x, odom_motion_y;   // raw odometry movement, never changed by resets or MCL
extern bool mcl_recenter;                     // set by setPose() / distance resets: start the particles there
extern double field_half_size;

namespace {

// Corner margin for MCL's readings: the loaders are in the obstacle list, so it doesn't need RW's 18"
// (with 18", every reading at the loader was skipped and MCL couldn't help where it matters most)
constexpr double MCL_CORNER_MARGIN = 6.0;
// MCL only corrects while the robot is moving slowly. The distance sensors report about distance_sensor_delay_msec
// late; when the robot is pushed by hand that doesn't matter, but at auton speed a delay that's off by 20-30 msec is
// 1-2" (it was right outside auton and off in auton). Between corrections it keeps following odometry.
constexpr double MCL_MAX_SPEED_IPS = 20.0;
constexpr double MCL_MAX_TURN_DPS = 90.0;
constexpr double BEAM_SPREAD = 0.21;   // tan(12 deg): how much wider the beam gets per inch it travels
constexpr double BEAM_SPARE = 1.0;     // extra room around each element

// Field elements a beam can hit before the wall (RW's Override field, the same one the simulator draws)
struct Obstacle { double x, y, r; bool* gone; };
bool pin_gone[8] = {false, false, false, false, false, false, false, false};
const Obstacle obstacles[] = {
    // goals (radius = half the base)
    {0, 0, 4.0, nullptr},
    {-47.09, -23.55, 4.0, nullptr}, {-47.09, 23.55, 4.0, nullptr}, {47.09, -23.55, 4.0, nullptr}, {47.09, 23.55, 4.0, nullptr},
    {-23.55, -47.09, 4.0, nullptr}, {-23.55, 47.09, 4.0, nullptr}, {23.55, -47.09, 4.0, nullptr}, {23.55, 47.09, 4.0, nullptr},
    // field pins + cups on the diagonals (until the robot picks one up)
    {-24, -24, 1.6, &pin_gone[0]}, {-48, -48, 1.6, &pin_gone[1]}, {24, 24, 1.6, &pin_gone[2]}, {48, 48, 1.6, &pin_gone[3]},
    {-24, 24, 1.6, &pin_gone[4]}, {-48, 48, 1.6, &pin_gone[5]}, {24, -24, 1.6, &pin_gone[6]}, {48, -48, 1.6, &pin_gone[7]},
    // pin stacks along the walls (1.6" boxes, as circles around their corners)
    {-23.6, -68.7, 2.3, nullptr}, {23.6, -68.7, 2.3, nullptr}, {-23.6, 68.7, 2.3, nullptr}, {23.6, 68.7, 2.3, nullptr},
    {-68.7, -23.6, 2.3, nullptr}, {68.7, -23.6, 2.3, nullptr}, {-68.7, 23.6, 2.3, nullptr}, {68.7, 23.6, 2.3, nullptr},
    // loaders (4" x 5" boxes, as circles around their corners)
    {-68.5, -58.7, 3.2, nullptr}, {68.5, -58.7, 3.2, nullptr}, {-68.5, 58.7, 3.2, nullptr}, {68.5, 58.7, 3.2, nullptr},
};

// True if something on the field is in the beam closer than the wall
bool beamBlocked(double sensor_x, double sensor_y, double beam_x, double beam_y, double wall_dist) {
    for (const Obstacle& o : obstacles) {
        if (o.gone != nullptr && *o.gone) continue;
        double to_x = o.x - sensor_x, to_y = o.y - sensor_y;
        double along = to_x * beam_x + to_y * beam_y;              // how far down the beam it is
        if (along < 0 || along - o.r > wall_dist) continue;        // behind the sensor or past the wall
        double across = std::fabs(to_x * beam_y - to_y * beam_x);  // how far off the beam's center line
        if (across < o.r + BEAM_SPREAD * along + BEAM_SPARE) return true;
    }
    return false;
}

void fieldMclThread() {
    // Odometry movement and heading over the last 160 msec, for matching up delayed sensor readings (same as RW)
    const int history_size = 16;
    double history_x[history_size], history_y[history_size], history_heading[history_size];
    for (int i = 0; i < history_size; i++) {
        history_x[i] = odom_motion_x;
        history_y[i] = odom_motion_y;
        history_heading[i] = getInertialHeading();
    }
    double last_motion_x = odom_motion_x, last_motion_y = odom_motion_y, last_heading = getInertialHeading();
    double moved_since_correct = 0;

    const hw::DistanceSensor sensors[4] = {hw::front_distance, hw::right_distance, hw::back_distance, hw::left_distance};
    const double sensor_x[4] = {front_sensor_offsetX, right_sensor_offsetX, back_sensor_offsetX, left_sensor_offsetX};
    const double sensor_y[4] = {front_sensor_offsetY, right_sensor_offsetY, back_sensor_offsetY, left_sensor_offsetY};
    const double sensor_angle[4] = {0, 90, 180, 270};
    uint32_t last_stamp[4] = {0, 0, 0, 0};
    double last_reading[4] = {-1, -1, -1, -1};

    mclInit(mcl_particles, x_pos, y_pos, 1.0);
    mcl_running = true;
    int tick = 0;

    while (true) {
        double heading = getInertialHeading();
        history_x[tick % history_size] = odom_motion_x;
        history_y[tick % history_size] = odom_motion_y;
        history_heading[tick % history_size] = heading;

        if (mcl_recenter) {
            // setPose() or a distance reset just set the position; start the particles there
            mcl_recenter = false;
            mclInit(mcl_particles, x_pos, y_pos, 1.0);
        }

        // Every 30 msec (about how often the distance sensors update)
        if (tick % 3 == 0) {
            uint64_t cycle_start = hw::timeMicros();
            double delta_x = odom_motion_x - last_motion_x, delta_y = odom_motion_y - last_motion_y;
            double delta_heading = heading - last_heading;
            last_motion_x = odom_motion_x;
            last_motion_y = odom_motion_y;
            last_heading = heading;
            mclPredict(delta_x, delta_y, delta_heading);
            moved_since_correct += std::hypot(delta_x, delta_y) + std::fabs(delta_heading) * 0.05;

            // Readings lag behind, so compare them with where odometry says the robot was back then
            int delay_ticks = (int)std::fmin(std::fmax(std::round(distance_sensor_delay_msec / 10.0), 0), history_size - 1);
            int then = ((tick - delay_ticks) % history_size + history_size) % history_size;
            double shift_x = odom_motion_x - history_x[then], shift_y = odom_motion_y - history_y[then];
            double heading_then = history_heading[then];
            double x_then = x_pos - shift_x, y_then = y_pos - shift_y;

            MclReading readings[4];
            int count = 0;
            for (int i = 0; i < 4; i++) {
                if (!hw::distanceInstalled(sensors[i]) || !hw::distanceDetected(sensors[i])) continue;
                double reading_in = hw::distanceInches(sensors[i]) * distance_sensor_scale;
                uint32_t stamp = hw::distanceTimestamp(sensors[i]);
                bool repeat = stamp == last_stamp[i] || reading_in == last_reading[i];
                last_stamp[i] = stamp;
                last_reading[i] = reading_in;
                if (repeat || reading_in < 1.5 || reading_in > mcl_max_range) continue;

                // 1. RW's wall checks
                double new_pos = 0;
                int axis = distanceResetPosition(x_then, y_then, heading_then, sensor_x[i], sensor_y[i], sensor_angle[i],
                                                 reading_in, field_half_size, distance_reset_max_angle, MCL_CORNER_MARGIN, new_pos);
                if (axis == 0) continue;

                // 2. Nothing in the beam before the wall
                double offset_x = 0, offset_y = 0;
                localToGlobal(sensor_x[i], sensor_y[i], degToRad(heading_then), offset_x, offset_y);
                double beam_rad = degToRad(heading_then + sensor_angle[i]);
                double beam_x = std::sin(beam_rad), beam_y = std::cos(beam_rad);
                bool hits_x_wall = false;
                double other_gap = 0;
                double wall_dist = beamToWall(x_then + offset_x, y_then + offset_y, beam_x, beam_y, field_half_size,
                                              hits_x_wall, other_gap);
                if (beamBlocked(x_then + offset_x, y_then + offset_y, beam_x, beam_y, wall_dist)) continue;

                // 3. Not a big jump
                double current = axis == 1 ? x_then : y_then;
                if (distance_reset_max_jump > 0 && std::fabs(new_pos - current) > distance_reset_max_jump) continue;

                readings[count].s_x = sensor_x[i];
                readings[count].s_y = sensor_y[i];
                readings[count].angle_deg = sensor_angle[i];
                readings[count].reading_in = reading_in;
                count++;
            }

            // Only weigh after moving a little, and only while moving slowly (MCL_MAX_SPEED_IPS / MCL_MAX_TURN_DPS)
            if (count > 0 && moved_since_correct > 0.25 && odom_speed_ips < MCL_MAX_SPEED_IPS
                && odom_turn_rate_dps < MCL_MAX_TURN_DPS) {
                mclCorrect(heading_then, readings, count, field_half_size, odom_speed_ips * 0.01, shift_x, shift_y);
                moved_since_correct = 0;
            }
            double spread_x = 0, spread_y = 0;
            mclEstimate(mcl_x, mcl_y, spread_x, spread_y);
            mcl_spread = std::fmax(spread_x, spread_y);
            mclNudge(x_pos, y_pos, mcl_x, mcl_y, spread_x, spread_y);
            mcl_cycle_msec = 0.8 * mcl_cycle_msec + 0.2 * (hw::timeMicros() - cycle_start) / 1000.0;
        }

        tick++;
        hw::sleepMsec(10);
    }
}

}  // namespace

// One-shot reset while stopped, like RW's localizeWithWalls(), but with the same checks as the field-aware MCL:
// RW's wall checks with a 6" corner margin (RW's resets use 18", which skips every reading next to the loaders), nothing
// on the field in the beam before the wall, and a reading that would move X or Y more than maxJump is skipped.
// Each sensor uses the median of 3 fresh readings (it waits up to 120 msec for them; they update about every 33 msec),
// so one noisy reading can't move it.
// Returns how many sensors reset X or Y. Only does anything when use_distance_resets is on.
int fieldAwareReset(double maxJump) {
    if (!use_distance_resets) return 0;
    const hw::DistanceSensor sensors[4] = {hw::front_distance, hw::right_distance, hw::back_distance, hw::left_distance};
    const double sensor_x[4] = {front_sensor_offsetX, right_sensor_offsetX, back_sensor_offsetX, left_sensor_offsetX};
    const double sensor_y[4] = {front_sensor_offsetY, right_sensor_offsetY, back_sensor_offsetY, left_sensor_offsetY};
    const double sensor_angle[4] = {0, 90, 180, 270};

    // Up to 3 fresh readings per sensor
    double readings[4][3];
    int counts[4] = {0, 0, 0, 0};
    uint32_t stamps[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) stamps[i] = hw::distanceTimestamp(sensors[i]);
    double start = hw::timeMsec();
        while (hw::timeMsec() - start < 120) {
        bool done = true;   // stop early once every sensor that sees something has its 3 readings
        for (int i = 0; i < 4; i++) {
            if (hw::distanceInstalled(sensors[i]) && hw::distanceDetected(sensors[i]) && counts[i] < 3) done = false;
        }
        if (done) break;
        for (int i = 0; i < 4; i++) {
            if (counts[i] >= 3 || !hw::distanceInstalled(sensors[i]) || !hw::distanceDetected(sensors[i])) continue;
            uint32_t stamp = hw::distanceTimestamp(sensors[i]);
            if (stamp == stamps[i]) continue;
            stamps[i] = stamp;
            readings[i][counts[i]++] = hw::distanceInches(sensors[i]) * distance_sensor_scale;
        }
        hw::sleepMsec(10);
    }

    double heading = getInertialHeading();
    int resets = 0;
    for (int i = 0; i < 4; i++) {
        if (counts[i] == 0) continue;
        double* r = readings[i];
        if (counts[i] == 3) {   // median
            if (r[0] > r[1]) std::swap(r[0], r[1]);
            if (r[1] > r[2]) std::swap(r[1], r[2]);
            if (r[0] > r[1]) std::swap(r[0], r[1]);
        }
        double reading_in = counts[i] == 3 ? r[1] : (counts[i] == 2 ? 0.5 * (r[0] + r[1]) : r[0]);
        if (reading_in < 1.5 || reading_in > mcl_max_range) continue;

        double new_pos = 0;
        int axis = distanceResetPosition(x_pos, y_pos, heading, sensor_x[i], sensor_y[i], sensor_angle[i],
                                         reading_in, field_half_size, distance_reset_max_angle, MCL_CORNER_MARGIN, new_pos);
        if (axis == 0) continue;
        double offset_x = 0, offset_y = 0;
        localToGlobal(sensor_x[i], sensor_y[i], degToRad(heading), offset_x, offset_y);
        double beam_rad = degToRad(heading + sensor_angle[i]);
        double beam_x = std::sin(beam_rad), beam_y = std::cos(beam_rad);
        bool hits_x_wall = false;
        double other_gap = 0;
        double wall_dist = beamToWall(x_pos + offset_x, y_pos + offset_y, beam_x, beam_y, field_half_size, hits_x_wall, other_gap);
        if (beamBlocked(x_pos + offset_x, y_pos + offset_y, beam_x, beam_y, wall_dist)) continue;
        double& coord = axis == 1 ? x_pos : y_pos;
        if (std::fabs(new_pos - coord) > maxJump) continue;
        coord = new_pos;
        resets++;
    }
    if (resets > 0) mcl_recenter = true;   // MCL starts over from the fixed spot
    return resets;
}

void startFieldMcl() {
    hw::startThread(fieldMclThread);
}

void fieldMclPinPickedUp(double x, double y) {
    for (const Obstacle& o : obstacles) {
        if (o.gone != nullptr && std::hypot(o.x - x, o.y - y) < 2.0) *o.gone = true;
    }
}
