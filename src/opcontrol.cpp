// Driver control.
#include "robot.hpp"

// Driver control
void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    // Hold wherever the lift is when driver control starts
    holdDR4B();

    // Match start: assume alliance goal, all counters at zero
    currentGoal = GOAL_ALLIANCE;
    stackCount[GOAL_ALLIANCE] = 0;
    stackCount[GOAL_NEUTRAL] = 0;
    autoLiftInProgress = false;
    goalCandidate = -1;
    goalCandidateSeen = 0;
    autoHeightEnabled = AUTO_HEIGHT_ON_AT_START;

    // Claw keeps spinning forward after R1 until R2 is used
    bool clawHolding = false;

    // Pin pickup state
    uint32_t r1PressStartMs = 0;
    uint32_t clawSlowStartMs = 0;
    bool clawSlow = false;
    bool clawSpunUp = false;         // true once the claw has sped up after an R1 press
    bool clawStallLatched = false;   // true once a stall was seen, so one grab only triggers once
    bool pinLiftActive = false;      // true while the lift is being driven up after a grab
    uint32_t pinLiftStartMs = 0;
    double pinLiftGoal = 0.0;

    // Lift zeroing state (only used while L2 is held)
    bool zeroPushing = false;
    uint32_t zeroPushStartMs = 0;
    bool zeroStalling = false;
    uint32_t zeroStallStartMs = 0;
    bool liftZeroedThisHold = false;

    // Tip protection state
    bool tipActive = false;
    int tipOverCount = 0;
    bool tipPushing = false;
    uint32_t tipPushStartMs = 0;
    bool tipStalling = false;
    uint32_t tipStallStartMs = 0;
    bool tipLiftDone = false;

    while (true) {

        // Drive
        int leftY = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
        int rightX = deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

        chassis.arcade(leftY, 0.9 * rightX);

        // B toggles the auto height feature.
        // Turning it back ON resets all counters, so the next L1 press goes to
        // height 1 of the last remembered goal (alliance if none yet).
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            autoHeightEnabled = !autoHeightEnabled;

            if (autoHeightEnabled) {
                stackCount[GOAL_ALLIANCE] = 0;
                stackCount[GOAL_NEUTRAL] = 0;
                autoLiftInProgress = false;
                goalCandidate = -1;
                goalCandidateSeen = 0;
            }

            controller.rumble(AUTO_TOGGLE_RUMBLE);
        }

        // Constantly watch the optical sensor for a goal change
        updateGoalDetection();

        // The auto lift press is finished once the lift reaches its target
        if (autoLiftInProgress && std::fabs(dr4bTarget - getDR4BPosition()) <= AUTO_ARRIVE_TOLERANCE) {
            autoLiftInProgress = false;
        }

        // Lift buttons
        // Auto on:  L1 = single press goes to next stack height, L2 = manual down (unchanged)
        // Auto off: L1 = manual up, L2 = manual down (original behavior)
        bool l1Held = !autoHeightEnabled && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l1Pressed = autoHeightEnabled && controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // Lift zeroing only lasts while L2 is held
        if (!l2Held) {
            zeroPushing = false;
            zeroStalling = false;
            liftZeroedThisHold = false;
        }

        // Tip protection: watch the roll angle (inertial sensor)
        {
            double roll = imu.get_roll();

            if (!TIP_PROTECTION_ENABLED || !std::isfinite(roll)) {
                // Off, or the sensor isn't giving a reading (still calibrating / unplugged): do nothing
                tipOverCount = 0;
                tipActive = false;
            } else {
                if (std::fabs(roll) > TIP_ROLL_LIMIT) {
                    if (tipOverCount < 1000) tipOverCount++;
                } else {
                    tipOverCount = 0;
                }

                if (!tipActive && tipOverCount >= TIP_CONFIRM_CYCLES) {
                    tipActive = true;
                } else if (tipActive && std::fabs(roll) < TIP_ROLL_LIMIT - TIP_ROLL_HYSTERESIS) {
                    tipActive = false;
                }
            }

            if (!tipActive) {
                tipPushing = false;
                tipStalling = false;
                tipLiftDone = false;
            }
        }

        // Tipping or a lift button always cancels a pin lift in progress
        if (tipActive || l1Held || l1Pressed || l2Held) {
            pinLiftActive = false;
        }

        // DR4B manual control
        if (tipActive) {

            // Tipping: lower the lift by itself (the lift buttons are ignored until the robot levels out).
            // It stops at the bottom or when it stops moving, and never keeps pushing against the hard stop.
            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            double position = getDR4BPosition();

            if (position > DR4B_BOTTOM_DEADZONE) {
                DR4B.move_velocity(TIP_LOWER_SPEED);
                tipPushing = false;
                tipStalling = false;
            } else if (position <= TIP_LOWER_STOP_POSITION || tipLiftDone) {
                DR4B.move_voltage(0);
            } else {
                uint32_t now = pros::millis();

                if (!tipPushing) {
                    tipPushing = true;
                    tipPushStartMs = now;
                    tipStalling = false;
                }

                DR4B.move_velocity(TIP_LOWER_SLOW_SPEED);

                double liftSpeed = dr4bRotation.get_velocity() / 100.0;   // degrees per second

                if (now - tipPushStartMs >= TIP_STALL_GRACE_MS && std::fabs(liftSpeed) < TIP_STALL_SPEED) {
                    if (!tipStalling) {
                        tipStalling = true;
                        tipStallStartMs = now;
                    }

                    if (now - tipStallStartMs >= TIP_STALL_MS) {
                        // The lift is on its hard stop: set the sensor to 0 right here
                        dr4bRotation.reset_position();
                        liftZeroCount++;
                        tipLiftDone = true;
                        DR4B.move_voltage(0);
                    }
                } else {
                    tipStalling = false;
                }
            }

        } else if (l1Held) {

            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            DR4B.move_velocity(DR4B_UP_SPEED);

        } else if (l2Held) {

            dr4bManualMode = true;
            dr4bTargetForced = false;
            dr4bSettling = false;
            autoLiftInProgress = false;
            DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

            double position = getDR4BPosition();

            if (position > DR4B_BOTTOM_DEADZONE) {
                DR4B.move_velocity(DR4B_DOWN_SPEED);
                zeroPushing = false;
                zeroStalling = false;
            } else if (!LIFT_ZERO_ON_STALL || liftZeroedThisHold) {
                DR4B.move_velocity(0);
            } else {
                // At the bottom with L2 held: keep pushing down gently, and when the lift
                // stops moving (it is on its hard stop) set the sensor to 0 right there
                uint32_t now = pros::millis();

                if (!zeroPushing) {
                    zeroPushing = true;
                    zeroPushStartMs = now;
                    zeroStalling = false;
                }

                DR4B.move_velocity(LIFT_ZERO_DRIVE_SPEED);

                double liftSpeed = dr4bRotation.get_velocity() / 100.0;   // degrees per second

                if (now - zeroPushStartMs >= LIFT_ZERO_GRACE_MS && std::fabs(liftSpeed) < LIFT_ZERO_STALL_SPEED) {
                    if (!zeroStalling) {
                        zeroStalling = true;
                        zeroStallStartMs = now;
                    }

                    if (now - zeroStallStartMs >= LIFT_ZERO_STALL_MS) {
                        dr4bRotation.reset_position();
                        liftZeroCount++;
                        liftZeroedThisHold = true;
                        DR4B.move_velocity(0);
                    }
                } else {
                    zeroStalling = false;
                }
            }

        } else if (pinLiftActive) {

            // Pin pickup: drive the lift straight up with the motors (not the PID) until it reaches
            // the goal, then lock it there. The PID is kept out of the way while this runs.
            dr4bManualMode = true;
            dr4bSettling = false;

            double position = getDR4BPosition();

            if (position >= pinLiftGoal || pros::millis() - pinLiftStartMs >= PIN_LIFT_TIMEOUT_MS) {
                pinLiftActive = false;
                dr4bTarget = position;
                dr4bTargetForced = true;   // hold it here even though it's inside the bottom deadzone
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                DR4B.brake();
                dr4bManualMode = false;
            } else {
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                DR4B.move_velocity(PIN_LIFT_SPEED);
            }

        } else {

            // Driver released L1/L2 (non-blocking so driving stays responsive)
            if (dr4bManualMode && !dr4bSettling) {
                // 1. Actively brake the motors to kill momentum
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
                DR4B.move_velocity(0);

                // 2. Start the settle timer so the DR4B structure and rubber bands can come to rest
                dr4bSettling = true;
                dr4bSettleStartMs = pros::millis();

            } else if (dr4bSettling && (pros::millis() - dr4bSettleStartMs) >= DR4B_SETTLE_TIME_MS) {
                // 3. Capture exact settled position
                dr4bTarget = getDR4BPosition();
                dr4bManualMode = false;
                dr4bSettling = false;

                // 4. Return brake mode back to coast so custom PID handles holding
                DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            }

            // Auto stack press (overrides any settling in progress)
            if (l1Pressed) {
                autoLiftPress();
            }
        }
        // Claw
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {

            clawHolding = true;
            claw.move_velocity(200);

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

            clawHolding = false;
            claw.move_velocity(-200);

        } else {

            if (clawHolding) {
                claw.move_velocity(60);
            } else {
                claw.move_velocity(0);
            }
        }
        // Pin pickup: R1 is held (claw driving forward) but the claw has slowed to a stall, so it has a pin.
        // If the lift is low, move it to PIN_LIFT_HEIGHT. This runs once per R1 press.
        // It watches the claw both while R1 is held AND after R1 is released (the claw keeps squeezing at
        // 60 rpm), so a quick tap of R1 still counts. Pressing R1 again or using R2 re-arms it.
        {
            bool r1Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
            bool r2Held = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
            uint32_t now = pros::millis();

            double clawSpeed = std::fabs(claw.get_actual_velocity());

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                r1PressStartMs = now;
                clawSlow = false;
                clawStallLatched = false;
                clawSpunUp = false;
            }

            // Claw opened: the next grab can trigger again
            if (r2Held) {
                clawStallLatched = false;
            }

            // Stall checks start as soon as the claw has actually got moving (or after a short
            // grace time, in case the element is already in the claw and it never speeds up)
            if (clawSpeed >= CLAW_SPINUP_SPEED || now - r1PressStartMs >= CLAW_STALL_GRACE_MS) {
                clawSpunUp = true;
            }

            bool clawSqueezing = r1Held || (clawHolding && !r2Held);
            double stallSpeed = r1Held ? CLAW_STALL_SPEED : CLAW_HOLD_STALL_SPEED;

            if (!PIN_LIFT_ENABLED || !clawSqueezing || clawStallLatched || !clawSpunUp) {
                clawSlow = false;
            } else if (clawSpeed < stallSpeed) {
                if (!clawSlow) {
                    clawSlow = true;
                    clawSlowStartMs = now;
                }

                if (now - clawSlowStartMs >= CLAW_STALL_CONFIRM_MS) {
                    clawStallLatched = true;

                    double liftPos = getDR4BPosition();

                    // Only when the driver isn't moving the lift (the short settle after
                    // releasing L1/L2 is fine) and the lift is in the low zone
                    if ((!dr4bManualMode || dr4bSettling) && !autoLiftInProgress && !tipActive
                        && !l1Held && !l2Held
                        && liftPos >= PIN_LIFT_ZONE_MIN && liftPos <= PIN_LIFT_ZONE_MAX) {
                        pinLiftGoal = std::max(PIN_LIFT_HEIGHT, liftPos + PIN_LIFT_MIN_RISE);
                        pinLiftActive = true;
                        pinLiftStartMs = now;
                        dr4bSettling = false;
                        pinLiftCount++;

                        // Start lifting right now instead of waiting for the next loop
                        dr4bManualMode = true;
                        DR4B.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                        DR4B.move_velocity(PIN_LIFT_SPEED);
                    }

                    controller.rumble(PIN_LIFT_RUMBLE);   // grab detected
                }
            } else {
                clawSlow = false;
            }
        }
        //code for the toggle mechanism, spins one way while holding A and stops when A is released
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            toggleMechanism.move_velocity(200); // Spins one way while holding A (change speed if needed)
        } else {
            toggleMechanism.move_velocity(0);   // Stops when A is released
        }
        // -----------------------------------------------
        pros::delay(20);
    }
}