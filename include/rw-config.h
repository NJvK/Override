#pragma once
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h).
// The lift, claw, toggle and optical sensor are defined in src/main.cpp and declared in include/robot.hpp.

// Localization switches (src/rw-config.cpp, LOCALIZATION block)
extern bool use_mcl;
extern bool use_distance_resets;

// Tuning / test routines (src/rw-autons.cpp), picked on the auton selector
void exampleAuton();      // "Tuning Test": drive and turn PID
void exampleAuton2();     // "Odom Test": drives a few points and back to the start
void odomCheckAuton();    // "Odom Check": nothing moves, push the robot and read X / Y / heading
void squareTestAuton();   // "Square Test": 24" square back to the start
void sensorCheckAuton();  // "Sensor Check": distance sensor readings, resets and MCL from the skills start
void imuScaleAuton();     // "IMU Scale": spin the robot 10 turns by hand to measure imu_scale

// Called from src/main.cpp (src/rw-user.cpp)
void runPreAutonomous();
void runAutonomous();
void runDriver();