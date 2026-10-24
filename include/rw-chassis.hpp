#pragma once
// =====================================================================
//   RW CHASSIS BRIDGE: the auton files talk to RW-Template through this
// =====================================================================
// The autons were written for LemLib (chassis.moveToPoint(...), chassis.getPose(), ...). This file gives
// them the same calls, but every one of them runs on RW-Template underneath:
//   - position = RW's odometry (x_pos / y_pos), corrected by MCL and distance resets when those are on
//   - heading  = RW's IMU heading (with imu_scale), 0 = facing +Y, clockwise positive, keeps counting past 360
//   - turnToHeading / turnToPoint -> RW turnToAngle()      (turn PID: turn_kp / turn_kd in src/rw-config.cpp)
//   - moveToPoint                 -> RW moveToPoint()      (drive PID: distance_kp / distance_kd)
//   - moveToPose                  -> RW boomerang()        (lead = how curved; chase_power limits curve speed)
//   - tank                        -> RW driveChassis()     (volts)
// Speeds keep LemLib's scale (0-127) so none of the tuned speeds in the autons had to change.
// Motions run in RW's background motion task (runAsync), so the autons can keep checking the lift,
// claw timers and distances while the robot drives, exactly like they did with LemLib.
//
// Field: (0, 0) is the field center, +X right, +Y up, walls at +-70.25 (RW's field_half_size: the inside
// of the perimeter). Tiles are still 24", so field elements sit at multiples of 24.
// =====================================================================

#include "rw-config.h"
#include <cmath>

// A robot position: x, y in inches, theta in degrees
struct Pose {
    double x, y, theta;
    Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
    double distance(const Pose& other) const { return std::hypot(x - other.x, y - other.y); }
};

// Motion options: same names, order and meaning as LemLib's, so the auton code reads the same.
//   maxSpeed:       top speed, 0-127
//   minSpeed:       kept so the LemLib-style calls still compile; RW stops between moves (LemLib did too)
//   earlyExitRange: the move ends this close to the target (degrees for turns, inches for drives)
//                   instead of settling exactly, so the next move starts sooner
struct TurnToHeadingParams {
    int maxSpeed = 127;
    int minSpeed = 0;
    float earlyExitRange = 0;
};
struct MoveToPointParams {
    bool forwards = true;
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};
struct MoveToPoseParams {
    bool forwards = true;
    float horizontalDrift = 0;   // not used by RW (it uses chase_power in src/rw-config.cpp)
    float lead = 0.6;            // how curved the approach is (RW's dlead): 0 = straight at it, 0.6 = most curved
    float maxSpeed = 127;
    float minSpeed = 0;
    float earlyExitRange = 0;
};

class RWChassis {
public:
    Pose getPose() const;                                 // where the robot is now
    void setPose(double x, double y, double theta);       // sets odometry (and the IMU heading); restarts MCL there
    void setBrakeMode(pros::motor_brake_mode_e mode);     // what the drive does when told to stop
    void tank(int left, int right, bool disableDriveCurve = false);   // direct drive, -127..127 per side
    void turnToHeading(double theta, int timeout, TurnToHeadingParams params = {});
    void turnToPoint(double x, double y, int timeout, TurnToHeadingParams params = {});
    void moveToPoint(double x, double y, int timeout, MoveToPointParams params = {});
    void moveToPose(double x, double y, double theta, int timeout, MoveToPoseParams params = {});
    bool isInMotion();                                    // true while a move is running (also applies early exits)
    void waitUntilDone();                                 // blocks until the move is done
    void cancelMotion();                                  // ends the running move now
};

extern RWChassis chassis;

// Distance sensor resets (RW's resetPositionFront/Back/Left/Right) for every sensor that has a clean view
// of a wall right now. Only does anything when use_distance_resets is true (src/rw-config.cpp).
// RW checks every reading itself (beam angle, corner margin, max jump) and skips the bad ones.
// Call it while the robot is stopped. Returns how many sensors reset the position.
int localizeWithWalls();

// Field-aware MCL (src/field-mcl.cpp): RW's particle filter, but only trusting distance readings with nothing on the
// field in the beam before the wall. Started by runPreAutonomous() when use_mcl is on (instead of RW's startMcl()).
void startFieldMcl();
// Tells it a field pin at (x, y) is gone (picked up), so beams past that spot count again
void fieldMclPinPickedUp(double x, double y);
// One-shot reset while stopped with the field-aware checks (works next to the loaders, unlike localizeWithWalls).
// Skips a reading that would move X or Y more than maxJump. Returns how many sensors reset X or Y.
int fieldAwareReset(double maxJump);