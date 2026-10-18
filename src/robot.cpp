// Robot hardware, the lift PID task, goal detection, and initialize().
#include "robot.hpp"

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Drive motors
pros::MotorGroup leftMotors({-1, -11, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({19, 10, 20}, pros::MotorGearset::blue);

// IMU
pros::Imu imu(18);

// Odometry rotation sensors
pros::Rotation horizontalEnc(21);
pros::Rotation verticalEnc(-16);

lemlib::TrackingWheel horizontal(
    &horizontalEnc,
    lemlib::Omniwheel::NEW_275,
    -5.75
);

lemlib::TrackingWheel vertical(
    &verticalEnc,
    lemlib::Omniwheel::NEW_275,
    -2.5
);

// DR4B motor
pros::MotorGroup DR4B({-7, 2}, pros::MotorGearset::green);

// DR4B rotation sensor
constexpr int DR4B_ROTATION_PORT = 3;
pros::Rotation dr4bRotation(DR4B_ROTATION_PORT);

// Other mechanisms
pros::Motor claw(9);
pros::Motor toggleMechanism(-4);

// Optical sensor
pros::Optical optical(OPTICAL_PORT);

// Auto stack state
bool autoHeightEnabled = AUTO_HEIGHT_ON_AT_START;
GoalType currentGoal = GOAL_ALLIANCE;          // goal list currently in use
int stackCount[2] = {0, 0};                    // how many presses we've used on each goal (0-5)
bool autoLiftInProgress = false;               // true while the lift is still going to an auto target
GoalType lastPressGoal = GOAL_ALLIANCE;        // goal used by the most recent auto press
int lastPressPrevCount = 0;                    // that goal's counter before the most recent press
int goalCandidate = -1;                        // goal being confirmed by the sensor
int goalCandidateSeen = 0;                     // loops the candidate has been seen
int goalCandidateMisses = 0;                   // loops in a row the candidate has been missed
int pinLiftCount = 0;                          // times the pin pickup moved the lift (shown on the brain screen)
int liftZeroCount = 0;                         // times the lift was zeroed by a stall (shown on the brain screen)

// Joystick deadband
int deadband(int value) {
    if (std::abs(value) < 5) {
        return 0;
    }
    return value;
}

// Drivetrain
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    10,
    lemlib::Omniwheel::NEW_275,
    360,
    2
);

// Linear PID
lemlib::ControllerSettings linearController(
    5.78,
    0,
    6,
    0.5,
    1,
    75,
    2,
    150,
    0
);

// Angular PID
lemlib::ControllerSettings angularController(
    3.7,
    0,
    25.5,
    0,
    1,
    50,
    2,
    200,
    0
);

// Odometry sensors
lemlib::OdomSensors sensors = {
    &vertical,
    nullptr,
    &horizontal,
    nullptr,
    &imu
};

// Driver curves
lemlib::ExpoDriveCurve throttleCurve(
    3,
    10,
    1.019
);

lemlib::ExpoDriveCurve steerCurve(
    3,
    10,
    1.019
);

// Chassis
lemlib::Chassis chassis(
    drivetrain,
    linearController,
    angularController,
    sensors,
    &throttleCurve,
    &steerCurve
);

// DR4B PID

// Position where the driver last left the lift
double dr4bTarget = 0.0;

// True when the target was set on purpose inside the bottom deadzone (the pin pickup height),
// so the PID must actually go there instead of treating it as "stay at the bottom"
bool dr4bTargetForced = false;

// True while L1 or L2 manually controls the lift (also true while it settles after release)
bool dr4bManualMode = false;

// True while the lift is braking after L1/L2 was released, waiting to capture its position
bool dr4bSettling = false;
uint32_t dr4bSettleStartMs = 0;

// Get rotation sensor position in degrees
double getDR4BPosition() {
    return dr4bRotation.get_position() / 100.0;
}

// Save current lift position
void holdDR4B() {
    dr4bTarget = getDR4BPosition();
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
}

// DR4B PID loop - Smooth Hold & Motion
void runDR4BPID() {
    constexpr double DT = 0.010;  // loop time in seconds (loop runs every 10 ms, matches the pros::delay calls below)

    double previousError = 0.0;
    double filteredDerivative = 0.0;
    double integral = 0.0;
    int belowLimitCount = 0;

    // Motor hold state
    bool brakeHolding = false;
    int settledCycles = 0;
    double lockedTarget = 0.0;    // the target the motor hold locked onto
    double lastPosition = getDR4BPosition();
    double positionSpeed = 0.0;   // smoothed lift speed in degrees/second

    // Setpoint that slides toward dr4bTarget at a limited speed so the lift
    // glides to each height instead of being slammed at it
    double profiledTarget = getDR4BPosition();
    double profileVelocity = 0.0;   // how fast that setpoint is moving (degrees/second, + = up)

    // Gains (low on purpose for a steady hold; travel power comes from the travel boost)
    constexpr double SMOOTH_KP = 70.0;
    constexpr double SMOOTH_KD = 20.0;  // Damps overshoot

    while (true) {

        // Track the lift's speed (used to decide when it has stopped)
        double nowPosition = getDR4BPosition();
        positionSpeed = 0.7 * positionSpeed + 0.3 * ((nowPosition - lastPosition) / DT);
        lastPosition = nowPosition;

        // Constantly check if the lift has gone lower than the lower limit.
        // If so, the sensor's zero is too high, so re-zero it at this spot.
        // Runs in every mode (manual, PID, auto).
        if (getDR4BPosition() < DR4B_LOWER_LIMIT) {
            belowLimitCount++;
        } else {
            belowLimitCount = 0;
        }

        if (belowLimitCount >= DR4B_LOWER_LIMIT_CONFIRM_CYCLES) {
            dr4bRotation.reset_position();
            belowLimitCount = 0;
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = 0.0;
            profileVelocity = 0.0;
            lastPosition = 0.0;
            positionSpeed = 0.0;

            // Don't leave a target below the new zero
            if (dr4bTarget < 0.0) {
                dr4bTarget = 0.0;
            }
        }

        // Driver manually controls the lift
        if (dr4bManualMode) {
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = getDR4BPosition();
            profileVelocity = 0.0;
            brakeHolding = false;
            settledCycles = 0;
            pros::delay(10);
            continue;
        }

        double position = getDR4BPosition();

        // No correction in bottom deadzone, as long as the target is down there too.
        // (If the target is higher, e.g. an auto lift press, PID must run to get it moving.
        // The same goes for the pin pickup height, which is inside the deadzone on purpose.)
        if (position >= 0.0 && position <= DR4B_BOTTOM_DEADZONE && dr4bTarget <= DR4B_BOTTOM_DEADZONE && !dr4bTargetForced) {
            if (brakeHolding) {
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                brakeHolding = false;
            }
            settledCycles = 0;
            DR4B.move_voltage(0);
            previousError = 0.0;
            filteredDerivative = 0.0;
            integral = 0.0;
            profiledTarget = position; profileVelocity = 0.0;
            pros::delay(10);
            continue;
        }

        // Simple PID (original style): DR4B_KP / DR4B_KD straight on the real target.
        // The lift is left alone until it is off its target by DR4B_HOLD_TOLERANCE degrees.
        if (DR4B_USE_SIMPLE_PID) {
            double simpleError = dr4bTarget - position;

            if (std::abs(simpleError) < DR4B_HOLD_TOLERANCE) {
                DR4B.move_voltage(0);
            } else {
                double simpleDerivative = (simpleError - previousError) / DT;
                double simpleOutput = (DR4B_KP * simpleError) + (DR4B_KD * simpleDerivative);

                if (simpleOutput > DR4B_MAX_HOLD_VOLTAGE) simpleOutput = DR4B_MAX_HOLD_VOLTAGE;
                if (simpleOutput < -DR4B_MAX_HOLD_VOLTAGE) simpleOutput = -DR4B_MAX_HOLD_VOLTAGE;

                // Pin pickup lift: make sure the short hop has enough power to get moving
                if (dr4bTargetForced && simpleError > 2.0 && simpleOutput < PIN_LIFT_MIN_VOLTAGE) {
                    simpleOutput = PIN_LIFT_MIN_VOLTAGE;
                }

                DR4B.move_voltage(static_cast<int>(simpleOutput));
            }

            previousError = simpleError;
            profiledTarget = position; profileVelocity = 0.0;
            pros::delay(10);
            continue;
        }

        // Motor hold: lock the lift once it has arrived and stopped (see DR4B_USE_BRAKE_HOLD)
        if (DR4B_USE_BRAKE_HOLD) {
            if (brakeHolding) {
                if (std::abs(dr4bTarget - position) > DR4B_HOLD_EXIT_ERROR
                    || std::abs(dr4bTarget - lockedTarget) > 0.5) {
                    // Target moved (new lift press) or the lift was pushed away: go back to PID
                    brakeHolding = false;
                    settledCycles = 0;
                    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                    previousError = 0.0;
                    filteredDerivative = 0.0;
                    integral = 0.0;
                    profiledTarget = position; profileVelocity = 0.0;   // start the slide from where the lift is now
                } else {
                    // Stay locked, nothing for the PID to do
                    previousError = 0.0;
                    filteredDerivative = 0.0;
                    integral = 0.0;
                    profiledTarget = position; profileVelocity = 0.0;
                    pros::delay(10);
                    continue;
                }
            } else if (std::abs(dr4bTarget - position) < DR4B_HOLD_ENTER_ERROR
                       && std::abs(positionSpeed) < DR4B_HOLD_ENTER_SPEED) {
                settledCycles++;
                if (settledCycles >= DR4B_HOLD_ENTER_CYCLES) {
                    brakeHolding = true;
                    lockedTarget = dr4bTarget;
                    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                    DR4B.brake();
                    profiledTarget = position; profileVelocity = 0.0;
                    pros::delay(10);
                    continue;
                }
            } else {
                settledCycles = 0;
            }
        }

        // Move the setpoint toward the real target with a speed-up rate, a top speed and a
        // slow-down rate. It starts slowing down early enough to stop exactly at the target.
        double remaining = dr4bTarget - profiledTarget;
        double direction = (remaining >= 0.0) ? 1.0 : -1.0;
        double topSpeed = (direction > 0.0) ? DR4B_SLEW_UP : DR4B_SLEW_DOWN;
        double decelRate = (direction > 0.0) ? DR4B_DECEL_UP : DR4B_DECEL_DOWN;
        double speedToward = profileVelocity * direction;   // negative = moving away from the target

        if (speedToward < 0.0) {
            // Moving the wrong way: slow down and turn around first
            speedToward += DR4B_ACCEL * DT;
            if (speedToward > 0.0) speedToward = 0.0;
        } else {
            double stopDistance = speedToward * speedToward / (2.0 * decelRate);
            if (std::abs(remaining) <= stopDistance) {
                speedToward -= decelRate * DT;
                if (speedToward < 0.0) speedToward = 0.0;
            } else {
                speedToward += DR4B_ACCEL * DT;
                if (speedToward > topSpeed) speedToward = topSpeed;
            }
        }

        double profileStep = speedToward * DT;
        if (std::abs(remaining) <= 1.0 || profileStep >= std::abs(remaining)) {
            profiledTarget = dr4bTarget;
            profileVelocity = 0.0;
        } else {
            profiledTarget += direction * profileStep;
            profileVelocity = direction * speedToward;
        }

        double error = profiledTarget - position;

        // Continuous derivative calculation with low-pass filter to prevent voltage spikes
        double rawDerivative = (error - previousError) / DT;
        filteredDerivative = 0.7 * filteredDerivative + 0.3 * rawDerivative;

        // Optional integral (off when DR4B_KI is 0), only builds close to the target
        if (DR4B_KI != 0.0 && std::abs(error) < DR4B_I_ZONE) {
            integral += error * DT;
            double integralLimit = DR4B_I_MAX / DR4B_KI;
            if (integral > integralLimit) integral = integralLimit;
            if (integral < -integralLimit) integral = -integralLimit;
        } else {
            integral = 0.0;
        }

        // Deadband: errors smaller than the deadband add no push, and the push grows
        // smoothly from zero outside it (a hard on/off cutoff made the lift hunt up and down)
        double pushError = 0.0;
        if (error > DR4B_PID_DEADBAND) {
            pushError = error - DR4B_PID_DEADBAND;
        } else if (error < -DR4B_PID_DEADBAND) {
            pushError = error + DR4B_PID_DEADBAND;
        }

        // Gains blend toward the low-height scales near the bottom (full at 0 degrees, none at DR4B_LOW_ZONE)
        double lowBlend = 1.0 - position / DR4B_LOW_ZONE;
        if (lowBlend < 0.0) lowBlend = 0.0;
        if (lowBlend > 1.0) lowBlend = 1.0;
        double kp = SMOOTH_KP * (1.0 + (DR4B_LOW_KP_SCALE - 1.0) * lowBlend);
        double kd = SMOOTH_KD * (1.0 + (DR4B_LOW_KD_SCALE - 1.0) * lowBlend);

        double output = (kp * pushError) + (DR4B_KI * integral);

        // Only damp (D term) when we are actually outside the deadband
        if (pushError != 0.0) {
            output += kd * filteredDerivative;
        }

        // Speed feedforward: the power to follow the moving target (drops to zero as the target stops)
        output += profileVelocity * (profileVelocity > 0.0 ? DR4B_KV_UP : DR4B_KV_DOWN);

        // How far the lift is from its real target (raises the voltage cap on long moves)
        double distToTarget = dr4bTarget - position;
        double fade = std::abs(distToTarget) / DR4B_TRAVEL_BOOST_FADE;
        if (fade > 1.0) fade = 1.0;

        // Allow bidirectional holding (both up and down correction); the voltage cap also
        // rises smoothly toward the move cap the farther the lift is from its target
        double maxVoltage = DR4B_MAX_HOLD_VOLTAGE + (DR4B_MAX_MOVE_VOLTAGE - DR4B_MAX_HOLD_VOLTAGE) * fade;
        if (output > maxVoltage) output = maxVoltage;
        if (output < -maxVoltage) output = -maxVoltage;

        // Pin pickup lift: the move is too short for the PID/feedforward to build enough power,
        // so push with at least PIN_LIFT_MIN_VOLTAGE until it is almost there
        if (dr4bTargetForced && distToTarget > 2.0 && output < PIN_LIFT_MIN_VOLTAGE) {
            output = PIN_LIFT_MIN_VOLTAGE;
        }

        DR4B.move_voltage(static_cast<int>(output));

        previousError = error;
        pros::delay(10);
    }
}

// ---------------------------------------------------------------------
// Auto stack helpers
// ---------------------------------------------------------------------

// True if hue is inside [lo, hi]. If lo > hi the range wraps past 0 (red).
bool hueInRange(double hue, double lo, double hi) {
    if (lo <= hi) {
        return hue >= lo && hue <= hi;
    }
    return hue >= lo || hue <= hi;
}

// Returns the goal the optical sensor currently sees, or -1 if nothing
// close enough (or a color we don't care about, like the opposing alliance goal)
int detectGoal() {
    if (optical.get_proximity() < GOAL_PROXIMITY_THRESHOLD) {
        return -1;
    }

    double hue = optical.get_hue();

    bool isAllianceColor = ALLIANCE_IS_RED
        ? hueInRange(hue, RED_HUE_MIN, RED_HUE_MAX)
        : hueInRange(hue, BLUE_HUE_MIN, BLUE_HUE_MAX);

    if (isAllianceColor) {
        return GOAL_ALLIANCE;
    }

    if (hueInRange(hue, NEUTRAL_HUE_MIN, NEUTRAL_HUE_MAX)) {
        return GOAL_NEUTRAL;
    }

    return -1;
}

// Height (degrees) for a goal and stack position (count is 1-5)
double getStackHeight(GoalType goal, int count) {
    const double* list = (goal == GOAL_ALLIANCE) ? ALLIANCE_HEIGHTS : NEUTRAL_HEIGHTS;
    double height = list[count - 1];

    // PID is off in the bottom deadzone, so keep targets above it
    if (height <= DR4B_BOTTOM_DEADZONE) {
        height = DR4B_BOTTOM_DEADZONE + 1.0;
    }

    return height;
}

// Called every loop: switches the active goal when the sensor confirms a different one.
// Runs even with auto height off so the last goal is always remembered.
void updateGoalDetection() {
    int detected = detectGoal();

    // Seeing the goal we're already on: nothing to switch to
    if (detected == static_cast<int>(currentGoal)) {
        goalCandidate = -1;
        goalCandidateSeen = 0;
        goalCandidateMisses = 0;
        return;
    }

    // Nothing recognised this loop: forgive a short dropout, give up after several in a row
    if (detected < 0) {
        if (goalCandidate >= 0) {
            goalCandidateMisses++;
            if (goalCandidateMisses > GOAL_MISS_TOLERANCE) {
                goalCandidate = -1;
                goalCandidateSeen = 0;
                goalCandidateMisses = 0;
            }
        }
        return;
    }

    if (detected == goalCandidate) {
        goalCandidateSeen++;
        goalCandidateMisses = 0;
    } else {
        goalCandidate = detected;
        goalCandidateSeen = 1;
        goalCandidateMisses = 0;
    }

    if (goalCandidateSeen < GOAL_CONFIRM_CYCLES) {
        return;
    }

    // Goal change confirmed
    GoalType oldGoal = currentGoal;
    currentGoal = static_cast<GoalType>(detected);
    goalCandidate = -1;
    goalCandidateSeen = 0;
    goalCandidateMisses = 0;

    if (autoHeightEnabled) {
        // If the lift was still going up for the old goal, that press was a misfire
        // (we were at a different goal), so give the old goal its count back.
        if (autoLiftInProgress
            && lastPressGoal == oldGoal
            && std::fabs(dr4bTarget - getDR4BPosition()) > AUTO_ARRIVE_TOLERANCE) {
            stackCount[oldGoal] = lastPressPrevCount;
        }

        autoLiftInProgress = false;
        controller.rumble(GOAL_CHANGE_RUMBLE);
    }
}

// L1 press in auto mode: advance this goal's counter and send the lift to that height.
// After pin/cup 5 the next press wraps back to pin/cup 1 of the same goal.
void autoLiftPress() {
    GoalType goal = currentGoal;

    lastPressGoal = goal;
    lastPressPrevCount = stackCount[goal];

    stackCount[goal]++;
    if (stackCount[goal] > NUM_STACK_HEIGHTS) {
        stackCount[goal] = 1;
    }

    dr4bTarget = getStackHeight(goal, stackCount[goal]);
    dr4bTargetForced = false;
    dr4bManualMode = false;
    dr4bSettling = false;
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    autoLiftInProgress = true;
}

// Initialize
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // Optical sensor LED
    optical.set_led_pwm(OPTICAL_LED_BRIGHTNESS);

    // Make the lift rotation sensor update every 5 ms (default is 20 ms) so the PID
    // gets fresh readings every loop and its damping (D term) isn't fed jumpy data
    dr4bRotation.set_data_rate(5);

    // DR4B should start fully down
    DR4B.tare_position();
    dr4bRotation.reset_position();

    dr4bTarget = getDR4BPosition();

    // PID controls holding instead of motor hold
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Start DR4B PID
    static pros::Task dr4bPIDTask([]() {
        runDR4BPID();
    });

    // Brain screen
    static pros::Task screenTask([]() {
        while (true) {
            {
                lemlib::Pose pose = chassis.getPose();
                pros::lcd::print(0, "X:%.1f Y:%.1f Th:%.0f", pose.x, pose.y, pose.theta);
            }
            // Claw speed (for tuning the pin stall values) and how many times pin pickup / zeroing have fired
            pros::lcd::print(1, "Claw V:%.0f  Pin:%d", claw.get_actual_velocity(), pinLiftCount);
            pros::lcd::print(2, "Zeroed:%d Roll:%.1f", liftZeroCount, imu.get_roll());
            pros::lcd::print(3, "Lift: %.2f", getDR4BPosition());
            pros::lcd::print(4, "Hold: %.2f", dr4bTarget);
            pros::lcd::print(5, "Hue: %.0f  Prox: %d", optical.get_hue(), static_cast<int>(optical.get_proximity()));
            {
                // See: what the optical sensor sees right now (A = alliance, N = neutral, - = nothing/other)
                int seen = detectGoal();
                pros::lcd::print(6, "Cnt A:%d N:%d See:%c Cf:%d", stackCount[GOAL_ALLIANCE], stackCount[GOAL_NEUTRAL],
                                 seen == GOAL_ALLIANCE ? 'A' : (seen == GOAL_NEUTRAL ? 'N' : '-'),
                                 goalCandidateSeen);
            }
            pros::lcd::print(7, "%s  Auto:%s",
                             currentGoal == GOAL_ALLIANCE ? "ALLIANCE" : "NEUTRAL",
                             autoHeightEnabled ? "ON" : "OFF");

            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            pros::delay(50);
        }
    });
}

// Disabled
void disabled() {

}

// Competition initialize
void competition_initialize() {

}
