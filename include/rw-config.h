#pragma once
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h).
// The lift, claw, toggle and optical sensor are defined in src/main.cpp and declared in include/robot.hpp.

// Localization switches (src/rw-config.cpp, LOCALIZATION block)
extern bool use_mcl;
extern bool use_distance_resets;

// Tuning tools, picked on the controller like the autons (the list is in src/auton-select.cpp).
// What each one shows and tunes: the top of src/tuning.cpp.
void tuneOdomCheck();     // "Odom Check": nothing moves, push the robot and read X / Y / heading
void tuneImuScale();      // "IMU Scale": spin the robot 10 turns by hand to measure imu_scale
void tuneFeedforward();   // "Feedforward": measures drive_kS / kV / kA (the curve follower drives with them)
void tuneDriveTest();     // "Drive Test": straight forward and back with the curve follower
void tuneSpinTest();      // "Spin Test": spinTo through 90 / 180 / 45 degree turns
void tuneCurveTest();     // "Curve Test": a forward curve and a backwards one home
void tuneLiftTest();      // "Lift Test": lift timing against what the autons predict
void tuneSensors();       // "Sensors + MCL": distance sensors, resets and MCL from the skills start
void tuneSimCalibrate();  // "Sim Calibrate": drive log for the simulator's fitter
void tuneClawCam();       // "Claw Cam": AI Vision in the claw (src/ai-vision-tuning.cpp)
void tuneFieldCam();      // "Field Cam": AI Vision on the chassis, tags and pins (src/ai-vision-tuning.cpp)
void tuneVisionPickup();  // "Vision Pickup": finds a cup with both cameras, picks it up with the auton pickup code

// Called from src/main.cpp (src/rw-user.cpp)
void runPreAutonomous();
void runAutonomous();
void runDriver();
